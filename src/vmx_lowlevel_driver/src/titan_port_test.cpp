#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "VMXPi.h"
#include "titan.h"

namespace {

std::atomic_bool stop_requested{false};

void signal_handler(int) {
  stop_requested = true;
}

std::vector<uint8_t> parse_ports(const std::string & text) {
  std::vector<uint8_t> ports;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const int port = std::stoi(item);
    if (port < 0 || port > 3) {
      throw std::runtime_error("Titan port must be in range 0..3");
    }
    ports.push_back(static_cast<uint8_t>(port));
  }
  if (ports.empty()) {
    throw std::runtime_error("At least one Titan port is required");
  }
  return ports;
}

void print_usage(const char * argv0) {
  std::cerr
    << "Usage: " << argv0
    << " [--ports 0,2] [--speed 0.20] [--duration 2.0] [--can-id 42]"
    << " [--current-limit 20] [--current-limit-mode 0|1]"
    << " [--stop-mode 0|1]"
    << " [--mode duty|velocity|raw_single|raw_signed|raw_four]"
    << " [--target-rpm 100] [--minimal] [--repeat-enable] [--period-ms 0]\n"
    << "       Speed is Titan SetSpeed normalized command in [-1.0, 1.0].\n";
}

constexpr uint32_t kTitanDeviceType = 33554432;
constexpr uint32_t kTitanManufacturerId = 786432;
constexpr uint32_t kTitanOffset = 64;
constexpr uint32_t kTitanBase = kTitanDeviceType + kTitanManufacturerId;
constexpr uint32_t kTitanEnabledFlag = kTitanBase + (kTitanOffset * 1);
constexpr uint32_t kTitanSetMotorSpeed = kTitanBase + (kTitanOffset * 2);

bool send_can(
  const std::shared_ptr<VMXPi> & vmx, uint32_t address, const uint8_t * data, int32_t period_ms)
{
  VMXErrorCode err;
  VMXCANMessage msg;
  msg.dataSize = 8;
  msg.setData(data, 8);
  msg.messageID = address;
  return vmx->can.SendMessage(msg, period_ms, &err);
}

}  // namespace

int main(int argc, char ** argv) {
  std::vector<uint8_t> ports{0, 2};
  double speed = 0.20;
  double duration_s = 2.0;
  double current_limit_a = -1.0;
  int current_limit_mode = -1;
  std::string mode{"duty"};
  int target_rpm = 100;
  int can_id = 42;
  int motor_freq = 15600;
  int ticks_per_rotation = 1464;
  int stop_mode = 1;
  double wheel_radius = 0.05;
  bool minimal = false;
  bool repeat_enable = false;
  int period_ms = 0;

  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--ports" && i + 1 < argc) {
        ports = parse_ports(argv[++i]);
      } else if (arg == "--speed" && i + 1 < argc) {
        speed = std::stod(argv[++i]);
      } else if (arg == "--duration" && i + 1 < argc) {
        duration_s = std::stod(argv[++i]);
      } else if (arg == "--can-id" && i + 1 < argc) {
        can_id = std::stoi(argv[++i]);
      } else if (arg == "--current-limit" && i + 1 < argc) {
        current_limit_a = std::stod(argv[++i]);
      } else if (arg == "--current-limit-mode" && i + 1 < argc) {
        current_limit_mode = std::stoi(argv[++i]);
      } else if (arg == "--mode" && i + 1 < argc) {
        mode = argv[++i];
      } else if (arg == "--target-rpm" && i + 1 < argc) {
        target_rpm = std::stoi(argv[++i]);
      } else if (arg == "--motor-freq" && i + 1 < argc) {
        motor_freq = std::stoi(argv[++i]);
      } else if (arg == "--ticks-per-rotation" && i + 1 < argc) {
        ticks_per_rotation = std::stoi(argv[++i]);
      } else if (arg == "--stop-mode" && i + 1 < argc) {
        stop_mode = std::stoi(argv[++i]);
      } else if (arg == "--wheel-radius" && i + 1 < argc) {
        wheel_radius = std::stod(argv[++i]);
      } else if (arg == "--minimal") {
        minimal = true;
      } else if (arg == "--repeat-enable") {
        repeat_enable = true;
      } else if (arg == "--period-ms" && i + 1 < argc) {
        period_ms = std::stoi(argv[++i]);
      } else if (arg == "--help" || arg == "-h") {
        print_usage(argv[0]);
        return 0;
      } else {
        print_usage(argv[0]);
        return 2;
      }
    }

    if (speed < -1.0 || speed > 1.0) {
      throw std::runtime_error("speed must be in range [-1.0, 1.0]");
    }
    if (duration_s <= 0.0 || duration_s > 60.0) {
      throw std::runtime_error("duration must be in range (0.0, 60.0]");
    }
    if (can_id < 1 || can_id > 63) {
      throw std::runtime_error("can-id must be in range 1..63");
    }
    if (current_limit_a > 20.0) {
      throw std::runtime_error("current-limit must be <= 20A");
    }
    if (current_limit_mode > 1) {
      throw std::runtime_error("current-limit-mode must be 0 or 1");
    }
    if (
      mode != "duty" && mode != "velocity" && mode != "raw_single" &&
      mode != "raw_signed" && mode != "raw_four")
    {
      throw std::runtime_error("mode must be duty, velocity, raw_single, raw_signed, or raw_four");
    }
    if (period_ms < 0 || period_ms > 1000) {
      throw std::runtime_error("period-ms must be in range 0..1000");
    }
    if (stop_mode < 0 || stop_mode > 1) {
      throw std::runtime_error("stop-mode must be 0 for coast or 1 for brake");
    }
    if (ticks_per_rotation <= 0 || wheel_radius <= 0.0) {
      throw std::runtime_error("ticks-per-rotation and wheel-radius must be positive");
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto vmx = std::make_shared<VMXPi>(true, 50);
    if (!vmx || !vmx->IsOpen()) {
      throw std::runtime_error("Unable to open VMX. Run as root and stop other VMX HAL processes.");
    }

    const float dist_per_tick =
      static_cast<float>((2.0 * M_PI * wheel_radius) / static_cast<double>(ticks_per_rotation));
    studica_driver::Titan titan(
      static_cast<uint8_t>(can_id), static_cast<uint16_t>(motor_freq), dist_per_tick, vmx);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "titan_info"
              << " can_id_arg=" << can_id
              << " device_id=" << static_cast<int>(titan.GetID())
              << " firmware=\"" << titan.GetFirmwareVersion() << "\""
              << " hardware=\"" << titan.GetHardwareVersion() << "\""
              << " temp_c=" << titan.GetControllerTemp()
              << '\n';

    for (const auto port : ports) {
      titan.ConfigureEncoder(port, dist_per_tick);
      titan.ResetEncoder(port);
      if (current_limit_a >= 0.0) {
        titan.SetCurrentLimit(port, static_cast<float>(current_limit_a));
      }
      if (current_limit_mode >= 0) {
        titan.SetCurrentLimitMode(port, static_cast<uint8_t>(current_limit_mode));
      }
    }
    if (!minimal) {
      titan.SetPIDType(mode == "velocity" ? 1 : 0);
      titan.SetMotorStopMode(static_cast<uint8_t>(stop_mode));
    }

    std::cout << "Titan direct port test: ports=";
    for (std::size_t i = 0; i < ports.size(); ++i) {
      std::cout << (i == 0 ? "" : ",") << static_cast<int>(ports[i]);
    }
    std::cout << " mode=" << mode << " speed=" << speed
              << " target_rpm=" << target_rpm
              << " duration=" << duration_s << "s"
              << " current_limit_a=" << current_limit_a
              << " current_limit_mode=" << current_limit_mode
              << " stop_mode=" << stop_mode
              << " minimal=" << minimal
              << " repeat_enable=" << repeat_enable
              << " period_ms=" << period_ms << "\n";

    std::cout << "before";
    for (const auto port : ports) {
      std::cout << " p" << static_cast<int>(port)
                << "_count=" << titan.GetEncoderCount(port)
                << " p" << static_cast<int>(port)
                << "_rpm=" << titan.GetRPM(port)
                << " p" << static_cast<int>(port)
                << "_limit_low=" << titan.GetLimitSwitch(port, 0)
                << " p" << static_cast<int>(port)
                << "_limit_high=" << titan.GetLimitSwitch(port, 1);
    }
    std::cout << '\n';

    titan.Enable(true);
    if (period_ms > 0) {
      const uint8_t enabled_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      send_can(vmx, kTitanEnabledFlag + static_cast<uint32_t>(can_id), enabled_data, period_ms);
    }

    const auto start = std::chrono::steady_clock::now();
    while (!stop_requested) {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= duration_s) {
        break;
      }
      const int duty = std::clamp(static_cast<int>(std::abs(speed) * 100.0), 0, 100);
      const int signed_duty = std::clamp(static_cast<int>(speed * 100.0), -100, 100);
      for (const auto port : ports) {
        if (repeat_enable) {
          titan.Enable(true);
        }
        if (mode == "raw_single") {
          const uint8_t data[8] = {
            port,
            static_cast<uint8_t>(duty),
            static_cast<uint8_t>(speed >= 0.0 ? 1 : 0),
            static_cast<uint8_t>(speed >= 0.0 ? 0 : 1),
            0, 0, 0, 0};
          send_can(vmx, kTitanSetMotorSpeed + static_cast<uint32_t>(can_id), data, period_ms);
        } else if (mode == "raw_signed") {
          const uint8_t data[8] = {
            port, static_cast<uint8_t>(static_cast<int8_t>(signed_duty)), 0, 0, 0, 0, 0, 0};
          send_can(vmx, kTitanSetMotorSpeed + static_cast<uint32_t>(can_id), data, period_ms);
        } else if (mode == "raw_four") {
          uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
          data[port] = static_cast<uint8_t>(static_cast<int8_t>(signed_duty));
          send_can(vmx, kTitanSetMotorSpeed + static_cast<uint32_t>(can_id), data, period_ms);
        } else if (mode == "velocity") {
          titan.SetTargetVelocity(port, static_cast<int16_t>(target_rpm));
        } else {
          titan.SetSpeed(port, speed);
        }
      }
      std::cout << "sample t=" << elapsed;
      for (const auto port : ports) {
        std::cout << " p" << static_cast<int>(port)
                  << "_count=" << titan.GetEncoderCount(port)
                  << " p" << static_cast<int>(port)
                  << "_rpm=" << titan.GetRPM(port);
      }
      std::cout << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (const auto port : ports) {
      if (mode == "velocity") {
        titan.SetTargetVelocity(port, 0);
      } else {
        titan.SetSpeed(port, 0.0);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    titan.Enable(false);

    std::cout << "after";
    for (const auto port : ports) {
      std::cout << " p" << static_cast<int>(port)
                << "_count=" << titan.GetEncoderCount(port)
                << " p" << static_cast<int>(port)
                << "_rpm=" << titan.GetRPM(port);
    }
    std::cout << '\n';
  } catch (const std::exception & e) {
    std::cerr << "titan_port_test failed: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
