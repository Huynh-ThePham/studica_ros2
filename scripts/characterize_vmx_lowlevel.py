#!/usr/bin/env python3
"""Characterize the VMX direct UDP low-level motor path.

This script is intentionally ROS-free. It sends direct UDP motor commands to the
VMX daemon, records encoder/RPM response per Titan port, and reports channels
that are weak relative to the other ports under the same command.
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List


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


@dataclass(frozen=True)
class TrialResult:
    speed: float
    port: int
    before_encoder: int
    after_encoder: int
    delta: int
    abs_delta: int
    average_running_rpm: float
    peak_running_rpm: float
    after_rpm: float
    after_motor_enabled: bool
    after_stopping: bool
    titan_ok: bool


def parse_int_list(text: str, *, lower: int, upper: int) -> List[int]:
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("list must not be empty")
    if any(value < lower or value > upper for value in values):
        raise ValueError(f"values must be in range {lower}..{upper}")
    return values


def parse_float_list(text: str) -> List[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise ValueError("list must not be empty")
    if any(value <= 0.0 or value > 1.0 for value in values):
        raise ValueError("speeds must be in range (0, 1]")
    return values


class UdpMotorCharacterizer:
    def __init__(self, host: str, command_port: int, telemetry_port: int) -> None:
        self._host = host
        self._command_port = command_port
        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._telemetry_socket.bind(("0.0.0.0", telemetry_port))
        self._telemetry_socket.settimeout(0.05)
        self._sequence = 0

    def close(self) -> None:
        self._telemetry_socket.close()
        self._command_socket.close()

    def _send(self, motor: Iterable[float], enable: bool) -> None:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._command_socket.sendto(
            build_command_packet(self._sequence, motor, enable),
            (self._host, self._command_port),
        )

    def recv_latest(self, timeout_s: float):
        latest = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                packet, _ = self._telemetry_socket.recvfrom(2048)
                latest = parse_telemetry_packet(packet)
            except socket.timeout:
                continue
            except ValueError:
                continue
        return latest

    def zero_and_sample(self, settle_s: float):
        for _ in range(8):
            self._send([0.0, 0.0, 0.0, 0.0], False)
            time.sleep(0.04)
        telemetry = self.recv_latest(settle_s)
        if telemetry is None:
            raise RuntimeError("no telemetry received while commanding zero")
        return telemetry

    def run_trial(
        self,
        *,
        port: int,
        speed: float,
        duration_s: float,
        rate_hz: float,
        settle_s: float,
    ) -> TrialResult:
        before = self.zero_and_sample(settle_s)
        running_rpm: List[float] = []
        period_s = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            motor = [0.0, 0.0, 0.0, 0.0]
            motor[port] = speed
            self._send(motor, True)
            sample = self.recv_latest(min(0.06, period_s))
            if sample is not None:
                running_rpm.append(float(sample.rpm[port]))
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(period_s, remaining))

        after = self.zero_and_sample(settle_s)
        delta = int(after.encoder[port] - before.encoder[port])
        peak_rpm = max(running_rpm, key=lambda value: abs(value)) if running_rpm else 0.0
        return TrialResult(
            speed=float(speed),
            port=int(port),
            before_encoder=int(before.encoder[port]),
            after_encoder=int(after.encoder[port]),
            delta=delta,
            abs_delta=abs(delta),
            average_running_rpm=float(statistics.mean(running_rpm)) if running_rpm else 0.0,
            peak_running_rpm=float(peak_rpm),
            after_rpm=float(after.rpm[port]),
            after_motor_enabled=bool(after.status_bits & STATUS_MOTOR_ENABLED),
            after_stopping=bool(after.status_bits & STATUS_STOPPING),
            titan_ok=bool(after.status_bits & STATUS_TITAN_OK),
        )


def signed_speeds(base_speeds: Iterable[float], mode: str) -> List[float]:
    positive = [abs(speed) for speed in base_speeds]
    if mode == "positive":
        return positive
    if mode == "negative":
        return [-speed for speed in positive]
    return [value for speed in positive for value in (speed, -speed)]


def print_csv(results: Iterable[TrialResult]) -> None:
    fields = list(TrialResult.__dataclass_fields__.keys())
    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()
    for result in results:
        writer.writerow(asdict(result))


def write_csv(path: Path, results: Iterable[TrialResult]) -> None:
    fields = list(TrialResult.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow(asdict(result))


def summarize(results: List[TrialResult], min_ratio: float) -> int:
    exit_code = 0
    print("\nsummary:")
    for speed in sorted({result.speed for result in results}, key=lambda value: (abs(value), value)):
        group = [result for result in results if result.speed == speed]
        if len(group) < 2:
            continue
        by_port = {result.port: result.abs_delta for result in group}
        print(f"  speed={speed:+.3f} abs_delta={by_port}")
        for result in group:
            others = [other.abs_delta for other in group if other.port != result.port]
            reference = statistics.median(others) if others else 0.0
            ratio = result.abs_delta / reference if reference > 0.0 else 0.0
            label = "OK"
            if ratio < min_ratio:
                label = "WEAK"
                exit_code = 1
            print(
                f"    M{result.port}: ratio_to_other_median={ratio:.2f} "
                f"peak_rpm={result.peak_running_rpm:.1f} after_rpm={result.after_rpm:.1f} {label}"
            )
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Reproducible low-level characterization for the VMX UDP daemon."
    )
    parser.add_argument("--host", default="172.22.11.2")
    parser.add_argument("--command-port", type=int, default=15000)
    parser.add_argument("--telemetry-port", type=int, default=15001)
    parser.add_argument("--ports", default="0,1,2,3")
    parser.add_argument("--speeds", default="0.12,0.16,0.20,0.24")
    parser.add_argument("--directions", choices=["positive", "negative", "both"], default="both")
    parser.add_argument("--duration", type=float, default=0.75)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--settle", type=float, default=0.45)
    parser.add_argument("--min-ratio", type=float, default=0.75)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    ports = parse_int_list(args.ports, lower=0, upper=3)
    speeds = signed_speeds(parse_float_list(args.speeds), args.directions)
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0")
    if args.settle <= 0.0:
        raise SystemExit("--settle must be > 0")

    characterizer = UdpMotorCharacterizer(args.host, args.command_port, args.telemetry_port)
    results: List[TrialResult] = []
    try:
        for speed in speeds:
            for port in ports:
                result = characterizer.run_trial(
                    port=port,
                    speed=speed,
                    duration_s=args.duration,
                    rate_hz=args.rate,
                    settle_s=args.settle,
                )
                results.append(result)
    finally:
        for _ in range(5):
            try:
                characterizer._send([0.0, 0.0, 0.0, 0.0], False)
            except OSError:
                pass
            time.sleep(0.04)
        characterizer.close()

    print_csv(results)
    if args.csv is not None:
        write_csv(args.csv, results)
    if args.json is not None:
        args.json.write_text(
            json.dumps([asdict(result) for result in results], indent=2) + "\n",
            encoding="utf-8",
        )
    return summarize(results, args.min_ratio)


if __name__ == "__main__":
    raise SystemExit(main())
