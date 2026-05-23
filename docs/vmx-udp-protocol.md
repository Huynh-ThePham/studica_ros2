# VMX Direct UDP Protocol

This document defines the stable binary protocol between the PC ROS 2 bridge and
the VMX direct low-level daemon.

## Transport

```text
Command:   PC  -> VMX UDP/15000
Telemetry: VMX -> PC  UDP/15001
Endian:    little-endian
Version:   1
Checksum:  FNV-1a 32-bit over all bytes except the final checksum field
```

The protocol is intentionally fixed-size and allocation-free on the VMX side.

## Command Packet

Python struct format:

```text
<IHHIQ4fII
```

Size: 44 bytes.

| Field | Type | Meaning |
| --- | --- | --- |
| `magic` | `uint32` | `0x43584D56`, ASCII `VMXC` in little-endian memory |
| `version` | `uint16` | protocol version, currently `1` |
| `size` | `uint16` | packet size, `44` |
| `sequence` | `uint32` | PC-side command sequence |
| `host_time_ns` | `uint64` | PC wall-clock timestamp, diagnostic only |
| `motor[4]` | `float32[4]` | normalized duty command for Titan M0..M3, clipped to `[-1, 1]` |
| `flags` | `uint32` | bit 0 = enable motor output |
| `checksum` | `uint32` | FNV-1a checksum |

Command rules:

- `motor[0]..motor[3]` maps directly to Titan ports M0..M3.
- The VMX does not apply robot kinematics, signs, trims or deadband. Those belong
  in the PC high-level package.
- If `flags & 1 == 0`, the VMX sends zero duty for active motors and keeps
  repeating zero briefly during the stop-hold window.
- The PC bridge sends commands at 10 Hz by default.

## Telemetry Packet

Python struct format:

```text
<IHHIQ4i4f4f3f3fIII
```

Size: 104 bytes.

| Field | Type | Meaning |
| --- | --- | --- |
| `magic` | `uint32` | `0x54584D56`, ASCII `VMXT` in little-endian memory |
| `version` | `uint16` | protocol version, currently `1` |
| `size` | `uint16` | packet size, `104` |
| `sequence` | `uint32` | VMX telemetry sequence |
| `vmx_time_ns` | `uint64` | VMX monotonic timestamp |
| `encoder[4]` | `int32[4]` | Titan encoder count M0..M3 |
| `rpm[4]` | `float32[4]` | Titan-reported RPM M0..M3 |
| `orientation_xyzw[4]` | `float32[4]` | navX quaternion |
| `angular_velocity_rad_s[3]` | `float32[3]` | gyro rad/s |
| `linear_acceleration_m_s2[3]` | `float32[3]` | acceleration m/s^2 |
| `status_bits` | `uint32` | daemon status flags |
| `last_command_sequence` | `uint32` | last valid command seen by VMX |
| `checksum` | `uint32` | FNV-1a checksum |

Status bits:

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `vmx_ok` | VMX HAL opened |
| 1 | `imu_ok` | navX is connected and readable |
| 2 | `titan_ok` | Titan object exists and telemetry read succeeded |
| 3 | `motor_enabled` | daemon is in active motor-output state |
| 4 | `cmd_timeout` | no fresh valid command inside watchdog window |
| 5 | `cmd_seen` | at least one valid command packet has been accepted |
| 6 | `checksum_error` | at least one bad command checksum has been observed |
| 7 | `stopping` | daemon is actively holding zero duty after a stop/timeout |

## Compatibility Contract

Protocol changes must preserve these rules:

- Never change packet field order for version `1`.
- Never change packet sizes for version `1`.
- Add new fields only by creating protocol version `2`.
- Keep the PC Python module
  [`vmx_highlevel/udp_protocol.py`](../src/vmx_highlevel/vmx_highlevel/udp_protocol.py)
  and the VMX C++ header
  [`udp_protocol.hpp`](../src/vmx_lowlevel_driver/include/vmx_lowlevel_driver/udp_protocol.hpp)
  in sync.

## Reference Tests

Direct UDP motor test:

```bash
python3 scripts/test_vmx_udp_direct.py \
  --host 172.22.11.2 --ports 0 --speed 0.35 --duration 2.0 --rate 10.0 --verbose
```

Expected bench result from the current robot:

```text
M0 @0.35, 2s: delta ~= -1858 ticks
M0 HAL direct reference: delta ~= -1985 ticks
```
