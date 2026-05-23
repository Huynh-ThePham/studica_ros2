#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <arpa/inet.h>
#include <iostream>
#include <memory>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "VMXPi.h"
#include "imu.h"
#include "titan.h"
#include "vmx_lowlevel_driver/udp_protocol.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;
namespace protocol = vmx_lowlevel_driver::udp_protocol;

constexpr double kPi = 3.14159265358979323846;
constexpr float kStandardGravity = 9.80665f;

std::atomic_bool g_stop_requested{false};

void signal_handler(int) {
  g_stop_requested = true;
}

double env_double(const char * name, double fallback) {
  const char * value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    std::cerr << "Invalid " << name << "='" << value << "', using " << fallback << '\n';
    return fallback;
  }
}

int env_int(const char * name, int fallback) {
  const char * value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    std::cerr << "Invalid " << name << "='" << value << "', using " << fallback << '\n';
    return fallback;
  }
}

std::string env_string(const char * name, const std::string & fallback) {
  const char * value = std::getenv(name);
  return value == nullptr || value[0] == '\0' ? fallback : std::string(value);
}

int make_udp_socket(uint16_t bind_port) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket(AF_INET, SOCK_DGRAM) failed: ") + std::strerror(errno));
  }

  int reuse = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(bind_port);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("bind UDP port " + std::to_string(bind_port) + " failed: " + error);
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
  return fd;
}

bool command_valid(const protocol::CommandPacket & command, bool * checksum_error) {
  *checksum_error = false;
  if (command.magic != protocol::kCommandMagic ||
    command.version != protocol::kProtocolVersion ||
    command.size != sizeof(protocol::CommandPacket))
  {
    return false;
  }
  if (protocol::packet_checksum(command) != command.checksum) {
    *checksum_error = true;
    return false;
  }
  for (float value : command.motor) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

uint64_t monotonic_ns() {
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
}

float clamp_motor(float value) {
  if (!std::isfinite(value)) {
    return 0.0f;
  }
  return std::clamp(value, -1.0f, 1.0f);
}

void send_zero_motor_commands(studica_driver::Titan * titan) {
  if (titan == nullptr) {
    return;
  }
  for (uint8_t port = 0; port < protocol::kMotorCount; ++port) {
    titan->SetSpeed(port, 0.0);
  }
}

void stop_titan_outputs(studica_driver::Titan * titan, bool disable_device) {
  if (titan == nullptr) {
    return;
  }
  try {
    for (int repeat = 0; repeat < 4; ++repeat) {
      send_zero_motor_commands(titan);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (disable_device) {
      titan->Enable(false);
    }
  } catch (const std::exception & e) {
    std::cerr << "Titan stop warning: " << e.what() << '\n';
  } catch (...) {
    std::cerr << "Titan stop warning: unknown exception\n";
  }
}

void fill_imu(
  studica_driver::Imu * imu,
  protocol::TelemetryPacket * telemetry,
  bool * imu_ok)
{
  *imu_ok = false;
  telemetry->orientation_xyzw[0] = 0.0f;
  telemetry->orientation_xyzw[1] = 0.0f;
  telemetry->orientation_xyzw[2] = 0.0f;
  telemetry->orientation_xyzw[3] = 1.0f;
  telemetry->angular_velocity_rad_s[0] = 0.0f;
  telemetry->angular_velocity_rad_s[1] = 0.0f;
  telemetry->angular_velocity_rad_s[2] = 0.0f;
  telemetry->linear_acceleration_m_s2[0] = 0.0f;
  telemetry->linear_acceleration_m_s2[1] = 0.0f;
  telemetry->linear_acceleration_m_s2[2] = 0.0f;

  if (imu == nullptr) {
    return;
  }

  try {
    *imu_ok = imu->IsConnected();
    if (!*imu_ok) {
      return;
    }
    telemetry->orientation_xyzw[0] = imu->GetQuaternionX();
    telemetry->orientation_xyzw[1] = imu->GetQuaternionY();
    telemetry->orientation_xyzw[2] = imu->GetQuaternionZ();
    telemetry->orientation_xyzw[3] = imu->GetQuaternionW();
    constexpr float deg_to_rad = static_cast<float>(kPi / 180.0);
    telemetry->angular_velocity_rad_s[0] = imu->GetRawGyroX() * deg_to_rad;
    telemetry->angular_velocity_rad_s[1] = imu->GetRawGyroY() * deg_to_rad;
    telemetry->angular_velocity_rad_s[2] = imu->GetRawGyroZ() * deg_to_rad;
    telemetry->linear_acceleration_m_s2[0] = imu->GetWorldLinearAccelX() * kStandardGravity;
    telemetry->linear_acceleration_m_s2[1] = imu->GetWorldLinearAccelY() * kStandardGravity;
    telemetry->linear_acceleration_m_s2[2] = imu->GetWorldLinearAccelZ() * kStandardGravity;
  } catch (const std::exception & e) {
    *imu_ok = false;
    std::cerr << "IMU read warning: " << e.what() << '\n';
  } catch (...) {
    *imu_ok = false;
    std::cerr << "IMU read warning: unknown exception\n";
  }
}

void fill_titan(
  studica_driver::Titan * titan,
  protocol::TelemetryPacket * telemetry,
  bool * titan_ok)
{
  *titan_ok = titan != nullptr;
  for (uint8_t port = 0; port < protocol::kMotorCount; ++port) {
    telemetry->encoder[port] = 0;
    telemetry->rpm[port] = 0.0f;
  }
  if (titan == nullptr) {
    return;
  }
  try {
    for (uint8_t port = 0; port < protocol::kMotorCount; ++port) {
      telemetry->encoder[port] = titan->GetEncoderCount(port);
      telemetry->rpm[port] = static_cast<float>(titan->GetRPM(port));
    }
  } catch (const std::exception & e) {
    *titan_ok = false;
    std::cerr << "Titan telemetry warning: " << e.what() << '\n';
  } catch (...) {
    *titan_ok = false;
    std::cerr << "Titan telemetry warning: unknown exception\n";
  }
}

}  // namespace

int main() {
  std::cout.setf(std::ios::unitbuf);
  std::cerr.setf(std::ios::unitbuf);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGHUP, signal_handler);

  const uint16_t command_port = static_cast<uint16_t>(
    std::clamp(env_int("VMX_UDP_CMD_PORT", 15000), 1, 65535));
  const uint16_t telemetry_port = static_cast<uint16_t>(
    std::clamp(env_int("VMX_UDP_TELEM_PORT", 15001), 1, 65535));
  const int can_id = std::clamp(env_int("VMX_TITAN_CAN_ID", 42), 1, 63);
  const int motor_freq = std::clamp(env_int("VMX_TITAN_MOTOR_FREQ", 15600), 1000, 25000);
  const int ticks_per_rotation = std::max(env_int("VMX_TICKS_PER_ROTATION", 1464), 1);
  const double wheel_radius = std::max(env_double("VMX_WHEEL_RADIUS_M", 0.05), 0.001);
  const double control_hz = std::clamp(env_double("VMX_UDP_CONTROL_HZ", 10.0), 1.0, 50.0);
  const double telemetry_hz = std::clamp(env_double("VMX_UDP_TELEMETRY_HZ", 20.0), 1.0, 100.0);
  const double timeout_s = std::clamp(env_double("VMX_UDP_CMD_TIMEOUT_SEC", 0.50), 0.05, 5.0);
  const int titan_stop_mode = std::clamp(env_int("VMX_TITAN_STOP_MODE", 1), 0, 1);
  const double zero_hold_s = std::clamp(env_double("VMX_TITAN_ZERO_HOLD_SEC", 1.00), 0.10, 5.00);
  const double titan_current_limit_a =
    std::clamp(env_double("VMX_TITAN_CURRENT_LIMIT_A", -1.0), -1.0, 20.0);
  const int titan_current_limit_mode =
    std::clamp(env_int("VMX_TITAN_CURRENT_LIMIT_MODE", -1), -1, 1);
  const std::string fixed_pc_ip = env_string("VMX_UDP_PC_IP", "");

  std::cout << "vmx_udp_lowlevel_daemon starting"
            << " cmd_port=" << command_port
            << " telem_port=" << telemetry_port
            << " can_id=" << can_id
            << " control_hz=" << control_hz
            << " telemetry_hz=" << telemetry_hz
            << " timeout_s=" << timeout_s
            << " titan_stop_mode=" << titan_stop_mode
            << " zero_hold_s=" << zero_hold_s
            << " titan_current_limit_a=" << titan_current_limit_a
            << " titan_current_limit_mode=" << titan_current_limit_mode
            << '\n';

  int command_fd = -1;
  int telemetry_fd = -1;
  try {
    command_fd = make_udp_socket(command_port);
    telemetry_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (telemetry_fd < 0) {
      throw std::runtime_error(std::string("telemetry socket failed: ") + std::strerror(errno));
    }
  } catch (const std::exception & e) {
    std::cerr << "UDP setup failed: " << e.what() << '\n';
    return 2;
  }

  sockaddr_in telemetry_addr{};
  bool telemetry_addr_valid = false;
  bool telemetry_addr_fixed = false;
  if (!fixed_pc_ip.empty()) {
    telemetry_addr.sin_family = AF_INET;
    telemetry_addr.sin_port = htons(telemetry_port);
    if (::inet_pton(AF_INET, fixed_pc_ip.c_str(), &telemetry_addr.sin_addr) == 1) {
      telemetry_addr_valid = true;
      telemetry_addr_fixed = true;
      std::cout << "Telemetry fixed destination " << fixed_pc_ip << ':' << telemetry_port << '\n';
    } else {
      std::cerr << "Invalid VMX_UDP_PC_IP='" << fixed_pc_ip << "', using last command sender\n";
    }
  }

  std::shared_ptr<VMXPi> vmx;
  std::unique_ptr<studica_driver::Imu> imu;
  std::unique_ptr<studica_driver::Titan> titan;
  bool vmx_ok = false;

  try {
    vmx = std::make_shared<VMXPi>(true, 50);
    if (!vmx || !vmx->IsOpen()) {
      throw std::runtime_error("VMX HAL did not open; stop other HAL processes and run as root");
    }
    vmx_ok = true;
    std::cout << "VMX HAL open\n";
  } catch (const std::exception & e) {
    std::cerr << "VMX HAL init failed: " << e.what() << '\n';
  }

  if (vmx_ok) {
    try {
      imu = std::make_unique<studica_driver::Imu>(vmx);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::cout << "IMU init ok connected=" << (imu->IsConnected() ? "true" : "false")
                << " firmware=\"" << imu->GetFirmwareVersion() << "\"\n";
    } catch (const std::exception & e) {
      std::cerr << "IMU init warning: " << e.what() << '\n';
      imu.reset();
    } catch (...) {
      std::cerr << "IMU init warning: unknown exception\n";
      imu.reset();
    }

    try {
      const float dist_per_tick = static_cast<float>(
        (2.0 * kPi * wheel_radius) / static_cast<double>(ticks_per_rotation));
      titan = std::make_unique<studica_driver::Titan>(
        static_cast<uint8_t>(can_id),
        static_cast<uint16_t>(motor_freq),
        dist_per_tick,
        vmx);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::cout << "Titan init ok"
                << " id=" << static_cast<int>(titan->GetID())
                << " firmware=\"" << titan->GetFirmwareVersion() << "\""
                << " hardware=\"" << titan->GetHardwareVersion() << "\""
                << " temp_c=" << titan->GetControllerTemp()
                << '\n';
      for (uint8_t port = 0; port < protocol::kMotorCount; ++port) {
        titan->ConfigureEncoder(port, dist_per_tick);
        titan->ResetEncoder(port);
        if (titan_current_limit_a >= 0.0) {
          titan->SetCurrentLimit(port, static_cast<float>(titan_current_limit_a));
        }
        if (titan_current_limit_mode >= 0) {
          titan->SetCurrentLimitMode(port, static_cast<uint8_t>(titan_current_limit_mode));
        }
      }
      titan->SetPIDType(0);
      titan->SetMotorStopMode(static_cast<uint8_t>(titan_stop_mode));
      stop_titan_outputs(titan.get(), false);
    } catch (const std::exception & e) {
      std::cerr << "Titan init warning: " << e.what()
                << " (daemon will keep IMU/telemetry alive, motor control disabled)\n";
      titan.reset();
    } catch (...) {
      std::cerr << "Titan init warning: unknown exception"
                << " (daemon will keep IMU/telemetry alive, motor control disabled)\n";
      titan.reset();
    }
  }

  // pigpio/VMX HAL may install its own signal handlers during construction.
  // Reinstall ours after bring-up so systemd stop exits through stop_titan_outputs().
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGHUP, signal_handler);

  std::array<float, protocol::kMotorCount> motor_command{0.0f, 0.0f, 0.0f, 0.0f};
  std::array<bool, protocol::kMotorCount> port_active{false, false, false, false};
  uint32_t last_command_sequence = 0;
  uint32_t telemetry_seq = 0;
  bool command_seen = false;
  bool checksum_error = false;
  bool motors_enabled = false;
  bool requested_enable = false;
  auto last_cmd_time = Clock::now() - std::chrono::seconds(3600);
  auto next_control = Clock::now();
  auto next_telemetry = Clock::now();
  const auto control_period = std::chrono::duration_cast<Clock::duration>(
    Seconds(1.0 / control_hz));
  const auto telemetry_period = std::chrono::duration_cast<Clock::duration>(
    Seconds(1.0 / telemetry_hz));
  const auto zero_hold_period = std::chrono::duration_cast<Clock::duration>(
    Seconds(zero_hold_s));
  bool zero_hold_active = false;
  auto zero_hold_until = Clock::now();

  while (!g_stop_requested) {
    while (true) {
      protocol::CommandPacket command{};
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);
      const ssize_t received = ::recvfrom(
        command_fd,
        &command,
        sizeof(command),
        0,
        reinterpret_cast<sockaddr *>(&sender),
        &sender_len);
      if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          std::cerr << "recvfrom warning: " << std::strerror(errno) << '\n';
        }
        break;
      }
      if (received != static_cast<ssize_t>(sizeof(protocol::CommandPacket))) {
        continue;
      }

      bool packet_checksum_error = false;
      if (!command_valid(command, &packet_checksum_error)) {
        checksum_error = checksum_error || packet_checksum_error;
        continue;
      }

      for (std::size_t i = 0; i < motor_command.size(); ++i) {
        motor_command[i] = clamp_motor(command.motor[i]);
      }
      requested_enable = (command.flags & protocol::kCommandFlagEnable) != 0;
      command_seen = true;
      last_command_sequence = command.sequence;
      last_cmd_time = Clock::now();

      if (!telemetry_addr_fixed) {
        telemetry_addr = sender;
        telemetry_addr.sin_port = htons(telemetry_port);
        telemetry_addr_valid = true;
      }
    }

    const auto now = Clock::now();
    if (now >= next_control) {
      next_control = now + control_period;
      const bool command_fresh = command_seen && Seconds(now - last_cmd_time).count() <= timeout_s;
      const bool should_enable = titan != nullptr && command_fresh && requested_enable;

      if (should_enable) {
        try {
          zero_hold_active = false;
          if (!motors_enabled) {
            titan->Enable(true);
          }
          for (uint8_t port = 0; port < protocol::kMotorCount; ++port) {
            const float command = motor_command[port];
            if (std::abs(command) > 1e-5f || port_active[port]) {
              titan->SetSpeed(port, command);
              port_active[port] = std::abs(command) > 1e-5f;
            }
          }
          motors_enabled = true;
        } catch (const std::exception & e) {
          motors_enabled = false;
          std::cerr << "Titan command warning: " << e.what() << '\n';
          stop_titan_outputs(titan.get(), false);
        } catch (...) {
          motors_enabled = false;
          std::cerr << "Titan command warning: unknown exception\n";
          stop_titan_outputs(titan.get(), false);
        }
      } else if (motors_enabled) {
        stop_titan_outputs(titan.get(), false);
        port_active = {false, false, false, false};
        motors_enabled = false;
        zero_hold_active = true;
        zero_hold_until = now + zero_hold_period;
      } else if (zero_hold_active) {
        if (now < zero_hold_until) {
          try {
            send_zero_motor_commands(titan.get());
          } catch (const std::exception & e) {
            std::cerr << "Titan zero-hold warning: " << e.what() << '\n';
          } catch (...) {
            std::cerr << "Titan zero-hold warning: unknown exception\n";
          }
        } else {
          zero_hold_active = false;
        }
      }
    }

    if (now >= next_telemetry) {
      next_telemetry = now + telemetry_period;
      protocol::TelemetryPacket telemetry{};
      telemetry.magic = protocol::kTelemetryMagic;
      telemetry.version = protocol::kProtocolVersion;
      telemetry.size = sizeof(protocol::TelemetryPacket);
      telemetry.sequence = ++telemetry_seq;
      telemetry.vmx_time_ns = monotonic_ns();
      telemetry.last_command_sequence = last_command_sequence;

      bool imu_ok = false;
      bool titan_ok = false;
      fill_imu(imu.get(), &telemetry, &imu_ok);
      fill_titan(titan.get(), &telemetry, &titan_ok);

      const bool command_timeout =
        !command_seen || Seconds(Clock::now() - last_cmd_time).count() > timeout_s;
      telemetry.status_bits = 0;
      if (vmx_ok) {
        telemetry.status_bits |= protocol::kStatusVmxOk;
      }
      if (imu_ok) {
        telemetry.status_bits |= protocol::kStatusImuOk;
      }
      if (titan_ok) {
        telemetry.status_bits |= protocol::kStatusTitanOk;
      }
      if (motors_enabled) {
        telemetry.status_bits |= protocol::kStatusMotorEnabled;
      }
      if (zero_hold_active) {
        telemetry.status_bits |= protocol::kStatusStopping;
      }
      if (command_timeout) {
        telemetry.status_bits |= protocol::kStatusCommandTimeout;
      }
      if (command_seen) {
        telemetry.status_bits |= protocol::kStatusCommandSeen;
      }
      if (checksum_error) {
        telemetry.status_bits |= protocol::kStatusChecksumError;
      }

      telemetry.checksum = protocol::packet_checksum(telemetry);
      if (telemetry_addr_valid) {
        ::sendto(
          telemetry_fd,
          &telemetry,
          sizeof(telemetry),
          0,
          reinterpret_cast<const sockaddr *>(&telemetry_addr),
          sizeof(telemetry_addr));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "vmx_udp_lowlevel_daemon stopping\n";
  stop_titan_outputs(titan.get(), true);
  if (command_fd >= 0) {
    ::close(command_fd);
  }
  if (telemetry_fd >= 0) {
    ::close(telemetry_fd);
  }
  return 0;
}
