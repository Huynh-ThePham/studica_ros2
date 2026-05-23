"""Binary UDP protocol shared by the PC bridge and VMX direct daemon.

The VMX and the PC are both little-endian ARM/x86 Linux targets in the supported
deployment. The packet ABI is intentionally fixed-size so it can be parsed
without allocation on the VMX side.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Iterable, List


COMMAND_MAGIC = 0x43584D56
TELEMETRY_MAGIC = 0x54584D56
PROTOCOL_VERSION = 1
MOTOR_COUNT = 4

COMMAND_FLAG_ENABLE = 1 << 0

STATUS_VMX_OK = 1 << 0
STATUS_IMU_OK = 1 << 1
STATUS_TITAN_OK = 1 << 2
STATUS_MOTOR_ENABLED = 1 << 3
STATUS_COMMAND_TIMEOUT = 1 << 4
STATUS_COMMAND_SEEN = 1 << 5
STATUS_CHECKSUM_ERROR = 1 << 6
STATUS_STOPPING = 1 << 7

COMMAND_FORMAT = "<IHHIQ4fII"
TELEMETRY_FORMAT = "<IHHIQ4i4f4f3f3fIII"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    vmx_time_ns: int
    encoder: List[int]
    rpm: List[float]
    orientation_xyzw: List[float]
    angular_velocity_rad_s: List[float]
    linear_acceleration_m_s2: List[float]
    status_bits: int
    last_command_sequence: int


def clamp_motor(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def fnv1a32(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value ^= byte
        value = (value * 16777619) & 0xFFFFFFFF
    return value


def packet_checksum(packet: bytes) -> int:
    return fnv1a32(packet[:-4])


def build_command_packet(sequence: int, motor: Iterable[float], enable: bool) -> bytes:
    values = [clamp_motor(value) for value in motor]
    if len(values) != MOTOR_COUNT:
        raise ValueError(f"motor command must contain exactly {MOTOR_COUNT} values")

    packet = struct.pack(
        COMMAND_FORMAT,
        COMMAND_MAGIC,
        PROTOCOL_VERSION,
        COMMAND_SIZE,
        sequence & 0xFFFFFFFF,
        time.time_ns(),
        values[0],
        values[1],
        values[2],
        values[3],
        COMMAND_FLAG_ENABLE if enable else 0,
        0,
    )
    return packet[:-4] + struct.pack("<I", packet_checksum(packet))


def parse_telemetry_packet(packet: bytes) -> Telemetry:
    if len(packet) != TELEMETRY_SIZE:
        raise ValueError(f"invalid telemetry size: {len(packet)} != {TELEMETRY_SIZE}")
    expected_checksum = struct.unpack_from("<I", packet, TELEMETRY_SIZE - 4)[0]
    if packet_checksum(packet) != expected_checksum:
        raise ValueError("telemetry checksum mismatch")

    values = struct.unpack(TELEMETRY_FORMAT, packet)
    magic, version, size, sequence, vmx_time_ns = values[:5]
    if magic != TELEMETRY_MAGIC:
        raise ValueError(f"invalid telemetry magic: 0x{magic:08x}")
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported telemetry version: {version}")
    if size != TELEMETRY_SIZE:
        raise ValueError(f"invalid telemetry declared size: {size}")

    return Telemetry(
        sequence=int(sequence),
        vmx_time_ns=int(vmx_time_ns),
        encoder=[int(value) for value in values[5:9]],
        rpm=[float(value) for value in values[9:13]],
        orientation_xyzw=[float(value) for value in values[13:17]],
        angular_velocity_rad_s=[float(value) for value in values[17:20]],
        linear_acceleration_m_s2=[float(value) for value in values[20:23]],
        status_bits=int(values[23]),
        last_command_sequence=int(values[24]),
    )
