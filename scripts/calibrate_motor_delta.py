#!/usr/bin/env python3
import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String


@dataclass
class PhaseResult:
    label: str
    command: List[float]
    before: List[int]
    after: List[int]

    @property
    def delta(self) -> List[int]:
        return [self.after[i] - self.before[i] for i in range(4)]


class MotorDeltaCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("motor_delta_calibrator")
        self.encoders: Dict[int, int] = {}
        self.status = ""
        self.power_pub = self.create_publisher(Float32MultiArray, "/titan/motor_power", 1)
        for port in range(4):
            self.create_subscription(
                Int32,
                f"/titan/motor{port}/encoder",
                lambda msg, port=port: self._encoder_cb(port, msg),
                10,
            )
        self.create_subscription(String, "/lowlevel_status", self._status_cb, 5)

    def _encoder_cb(self, port: int, msg: Int32) -> None:
        self.encoders[port] = int(msg.data)

    def _status_cb(self, msg: String) -> None:
        self.status = msg.data

    def wait_for_encoders(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.encoders) == 4:
                return True
        return False

    def snapshot(self) -> List[int]:
        return [self.encoders.get(port, 0) for port in range(4)]

    def publish_power(self, command: List[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(value) for value in command]
        self.power_pub.publish(msg)

    def stop_all(self, settle_s: float = 0.3) -> None:
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            self.publish_power([0.0, 0.0, 0.0, 0.0])
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def publish_for(self, command: List[float], duration_s: float, rate_hz: float) -> None:
        if duration_s <= 0.0:
            return
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.publish_power(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run_phase(
        self,
        label: str,
        command: List[float],
        duration_s: float,
        rate_hz: float,
        warmup_s: float,
    ) -> PhaseResult:
        self.stop_all()
        self.publish_for(command, warmup_s, rate_hz)
        before = self.snapshot()
        self.publish_for(command, duration_s, rate_hz)
        self.stop_all()
        after = self.snapshot()
        return PhaseResult(label=label, command=command, before=before, after=after)


def estimate_gains(results: List[PhaseResult]) -> List[float]:
    magnitudes = [0.0, 0.0, 0.0, 0.0]
    counts = [0, 0, 0, 0]
    for result in results:
        for port, command in enumerate(result.command):
            if abs(command) > 1e-6:
                magnitudes[port] += abs(result.delta[port])
                counts[port] += 1
    avg = [magnitudes[i] / counts[i] if counts[i] else 0.0 for i in range(4)]
    nonzero = [value for value in avg if value > 0.0]
    if not nonzero:
        return [1.0, 1.0, 1.0, 1.0]
    target = max(nonzero)
    return [target / value if value > 0.0 else math.inf for value in avg]


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure VMX Titan motor encoder deltas per command.")
    parser.add_argument("--power", type=float, default=0.22)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.0,
        help="Seconds to command the phase before taking the before encoder snapshot.",
    )
    parser.add_argument("--mode", choices=["single", "pairs"], default="single")
    args = parser.parse_args()

    rclpy.init()
    node = MotorDeltaCalibrator()
    try:
        if not node.wait_for_encoders(8.0):
            raise RuntimeError("Timeout waiting for /titan/motor*/encoder topics.")

        p = float(args.power)
        phases = []
        if args.mode == "single":
            for port in range(4):
                cmd = [0.0, 0.0, 0.0, 0.0]
                cmd[port] = p
                phases.append((f"M{port} +{p:.3f}", cmd))
            for port in range(4):
                cmd = [0.0, 0.0, 0.0, 0.0]
                cmd[port] = -p
                phases.append((f"M{port} -{p:.3f}", cmd))
        else:
            phases = [
                (f"M0+M1 +{p:.3f}", [p, p, 0.0, 0.0]),
                (f"M2+M3 -{p:.3f}", [0.0, 0.0, -p, -p]),
                ("all sync", [p, p, -p, -p]),
            ]

        print(f"status: {node.status or '(waiting)'}")
        results = []
        for label, command in phases:
            result = node.run_phase(label, command, args.duration, args.rate, args.warmup)
            results.append(result)
            print(label)
            print(f"  command={result.command}")
            print(f"  before={result.before}")
            print(f"  after ={result.after}")
            print(f"  delta ={result.delta}")

        gains = estimate_gains(results)
        print("estimated_command_gain_to_match_strongest:")
        for port, gain in enumerate(gains):
            if math.isinf(gain):
                print(f"  M{port}: inf")
            else:
                print(f"  M{port}: {gain:.3f}")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
