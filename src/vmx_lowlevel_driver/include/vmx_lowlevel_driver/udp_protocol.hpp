#ifndef VMX_LOWLEVEL_DRIVER_UDP_PROTOCOL_HPP_
#define VMX_LOWLEVEL_DRIVER_UDP_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace vmx_lowlevel_driver::udp_protocol {

constexpr uint32_t kCommandMagic = 0x43584D56u;    // "VMXC", little-endian
constexpr uint32_t kTelemetryMagic = 0x54584D56u;  // "VMXT", little-endian
constexpr uint16_t kProtocolVersion = 1;
constexpr std::size_t kMotorCount = 4;

constexpr uint32_t kCommandFlagEnable = 1u << 0;

constexpr uint32_t kStatusVmxOk = 1u << 0;
constexpr uint32_t kStatusImuOk = 1u << 1;
constexpr uint32_t kStatusTitanOk = 1u << 2;
constexpr uint32_t kStatusMotorEnabled = 1u << 3;
constexpr uint32_t kStatusCommandTimeout = 1u << 4;
constexpr uint32_t kStatusCommandSeen = 1u << 5;
constexpr uint32_t kStatusChecksumError = 1u << 6;
constexpr uint32_t kStatusStopping = 1u << 7;

#pragma pack(push, 1)
struct CommandPacket {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint64_t host_time_ns;
  float motor[kMotorCount];
  uint32_t flags;
  uint32_t checksum;
};

struct TelemetryPacket {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint64_t vmx_time_ns;
  int32_t encoder[kMotorCount];
  float rpm[kMotorCount];
  float orientation_xyzw[4];
  float angular_velocity_rad_s[3];
  float linear_acceleration_m_s2[3];
  uint32_t status_bits;
  uint32_t last_command_sequence;
  uint32_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(CommandPacket) == 44, "CommandPacket ABI must stay stable");
static_assert(sizeof(TelemetryPacket) == 104, "TelemetryPacket ABI must stay stable");

inline uint32_t fnv1a32(const uint8_t * data, std::size_t size) {
  uint32_t hash = 2166136261u;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

template<typename PacketT>
uint32_t packet_checksum(const PacketT & packet) {
  return fnv1a32(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(PacketT) - sizeof(uint32_t));
}

}  // namespace vmx_lowlevel_driver::udp_protocol

#endif  // VMX_LOWLEVEL_DRIVER_UDP_PROTOCOL_HPP_
