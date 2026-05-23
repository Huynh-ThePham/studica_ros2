#!/usr/bin/env python3
import argparse
import socket
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_PYTHON = REPO_ROOT / "src" / "vmx_highlevel"
if str(SRC_PYTHON) not in sys.path:
    sys.path.insert(0, str(SRC_PYTHON))

from vmx_highlevel.udp_protocol import (  # noqa: E402
    STATUS_MOTOR_ENABLED,
    STATUS_STOPPING,
    STATUS_TITAN_OK,
    build_command_packet,
    parse_telemetry_packet,
)


def recv_latest(sock: socket.socket, timeout: float) -> Optional[dict]:
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            packet, _ = sock.recvfrom(2048)
        except socket.timeout:
            break
        try:
            telemetry = parse_telemetry_packet(packet)
        except ValueError:
            continue
        latest = {
            "seq": telemetry.sequence,
            "encoder": telemetry.encoder,
            "rpm": telemetry.rpm,
            "status_bits": telemetry.status_bits,
            "last_cmd_seq": telemetry.last_command_sequence,
        }
    return latest


def main() -> int:
    parser = argparse.ArgumentParser(description="Direct UDP motor test for VMX low-level daemon.")
    parser.add_argument("--host", default="172.22.11.2")
    parser.add_argument("--command-port", type=int, default=15000)
    parser.add_argument("--telemetry-port", type=int, default=15001)
    parser.add_argument("--ports", default="0", help="Comma-separated ports to drive, e.g. 0 or 0,2")
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    ports = [int(item) for item in args.ports.split(",") if item.strip()]
    if not ports or any(port < 0 or port > 3 for port in ports):
        raise SystemExit("--ports must contain motor ports in range 0..3")
    if not -1.0 <= args.speed <= 1.0:
        raise SystemExit("--speed must be in range [-1, 1]")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")

    command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    telemetry_sock.bind(("0.0.0.0", args.telemetry_port))

    seq = 0
    zero = [0.0, 0.0, 0.0, 0.0]
    command_sock.sendto(build_command_packet(seq, zero, False), (args.host, args.command_port))
    time.sleep(0.2)
    before = recv_latest(telemetry_sock, 1.0)
    if before is None:
        raise SystemExit("No telemetry received. Check vmx-udp-lowlevel.service and LAN.")

    print(
        "before "
        f"status=0x{before['status_bits']:08x} "
        f"titan_ok={bool(before['status_bits'] & STATUS_TITAN_OK)} "
        f"stopping={bool(before['status_bits'] & STATUS_STOPPING)} "
        f"encoder={before['encoder']} rpm={[round(v, 1) for v in before['rpm']]}"
    )

    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    last = before
    while time.monotonic() < deadline:
        seq = (seq + 1) & 0xFFFFFFFF
        motor = [0.0, 0.0, 0.0, 0.0]
        for port in ports:
            motor[port] = args.speed
        command_sock.sendto(build_command_packet(seq, motor, True), (args.host, args.command_port))
        sample = recv_latest(telemetry_sock, min(0.08, period))
        if sample is not None:
            last = sample
            if args.verbose:
                delta_now = [last["encoder"][port] - before["encoder"][port] for port in range(4)]
                print(
                    f"sample status=0x{last['status_bits']:08x} "
                    f"last_cmd_seq={last['last_cmd_seq']} "
                    f"enabled={bool(last['status_bits'] & STATUS_MOTOR_ENABLED)} "
                    f"stopping={bool(last['status_bits'] & STATUS_STOPPING)} "
                    f"delta={delta_now} rpm={[round(v, 1) for v in last['rpm']]}"
                )
        sleep_time = deadline - time.monotonic()
        if sleep_time > 0:
            time.sleep(min(period, sleep_time))

    for _ in range(5):
        seq = (seq + 1) & 0xFFFFFFFF
        command_sock.sendto(build_command_packet(seq, zero, False), (args.host, args.command_port))
        time.sleep(0.04)

    after = recv_latest(telemetry_sock, 1.0) or last
    delta = [after["encoder"][port] - before["encoder"][port] for port in range(4)]
    print(
        "after  "
        f"status=0x{after['status_bits']:08x} "
        f"enabled={bool(after['status_bits'] & STATUS_MOTOR_ENABLED)} "
        f"stopping={bool(after['status_bits'] & STATUS_STOPPING)} "
        f"encoder={after['encoder']} delta={delta} "
        f"rpm={[round(v, 1) for v in after['rpm']]}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
