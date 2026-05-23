#!/usr/bin/env python3
import socket
import struct
import time


COMMAND_MAGIC = 0x43584D56
PROTOCOL_VERSION = 1
COMMAND_FORMAT = "<IHHIQ4fII"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)


def fnv1a32(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value ^= byte
        value = (value * 16777619) & 0xFFFFFFFF
    return value


def packet(seq: int) -> bytes:
    raw = struct.pack(
        COMMAND_FORMAT,
        COMMAND_MAGIC,
        PROTOCOL_VERSION,
        COMMAND_SIZE,
        seq,
        time.time_ns(),
        0.0,
        0.0,
        0.0,
        0.0,
        0,
        0,
    )
    return raw[:-4] + struct.pack("<I", fnv1a32(raw[:-4]))


def main() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for seq in range(1, 8):
        sock.sendto(packet(seq), ("127.0.0.1", 15000))
        time.sleep(0.03)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
