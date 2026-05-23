#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32, String


@dataclass
class PhaseResult:
    label: str
    linear: float
    angular: float
    before: List[int]
    after: List[int]
    status: str

    @property
    def delta(self) -> List[int]:
        return [self.after[i] - self.before[i] for i in range(4)]


class CmdVelDeltaTester(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_delta_tester")
        self.encoders: Dict[int, int] = {}
        self.highlevel_status = ""
        self.lowlevel_status = ""
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        for port in range(4):
            self.create_subscription(
                Int32,
                f"/titan/motor{port}/encoder",
                lambda msg, port=port: self._encoder_cb(port, msg),
                10,
            )
        self.create_subscription(String, "/highlevel_status", self._highlevel_status_cb, 5)
        self.create_subscription(String, "/lowlevel_status", self._lowlevel_status_cb, 5)

    def _encoder_cb(self, port: int, msg: Int32) -> None:
        self.encoders[port] = int(msg.data)

    def _highlevel_status_cb(self, msg: String) -> None:
        self.highlevel_status = msg.data

    def _lowlevel_status_cb(self, msg: String) -> None:
        self.lowlevel_status = msg.data

    def wait_for_encoders(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.encoders) == 4:
                return True
        return False

    def wait_for_highlevel(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.highlevel_status:
                return True
        return False

    def wait_for_cmd_vel_subscribers(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cmd_pub.get_subscription_count() > 0:
                return True
        return False

    def snapshot(self) -> List[int]:
        return [self.encoders.get(port, 0) for port in range(4)]

    def publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_for(self, linear: float, angular: float, duration_s: float, rate_hz: float) -> None:
        if duration_s <= 0.0:
            return
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.publish_cmd(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop_all(self, settle_s: float = 0.4) -> None:
        self.publish_for(0.0, 0.0, settle_s, 50.0)

    def run_phase(
        self,
        label: str,
        linear: float,
        angular: float,
        duration_s: float,
        warmup_s: float,
        rate_hz: float,
    ) -> PhaseResult:
        self.stop_all()
        self.publish_for(linear, angular, warmup_s, rate_hz)
        before = self.snapshot()
        self.publish_for(linear, angular, duration_s, rate_hz)
        self.stop_all()
        after = self.snapshot()
        return PhaseResult(label, linear, angular, before, after, self.highlevel_status)


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure encoder deltas through PC high-level /cmd_vel.")
    parser.add_argument("--linear", type=float, default=0.154, help="Forward test linear.x in m/s.")
    parser.add_argument("--angular", type=float, default=0.8, help="Turn test angular.z in rad/s.")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--warmup", type=float, default=0.75)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--mode", choices=["straight", "all"], default="straight")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelDeltaTester()
    try:
        if not node.wait_for_encoders(8.0):
            raise RuntimeError("Timeout waiting for /titan/motor*/encoder topics.")
        if not node.wait_for_highlevel(8.0):
            raise RuntimeError("Timeout waiting for /highlevel_status; start vmx_highlevel_node first.")
        if not node.wait_for_cmd_vel_subscribers(8.0):
            raise RuntimeError("Timeout waiting for /cmd_vel subscribers.")

        phases = [
            ("forward", args.linear, 0.0),
            ("backward", -args.linear, 0.0),
        ]
        if args.mode == "all":
            phases.extend(
                [
                    ("turn_left", 0.0, args.angular),
                    ("turn_right", 0.0, -args.angular),
                ]
            )

        print(f"highlevel_status: {node.highlevel_status}")
        print(f"lowlevel_status: {node.lowlevel_status}")
        print(f"cmd_vel_subscribers: {node.cmd_pub.get_subscription_count()}")
        for label, linear, angular in phases:
            result = node.run_phase(label, linear, angular, args.duration, args.warmup, args.rate)
            print(f"{result.label}: cmd_vel linear={result.linear:.4f} angular={result.angular:.4f}")
            print(f"  before={result.before}")
            print(f"  after ={result.after}")
            print(f"  delta ={result.delta}")
            print(f"  highlevel_status={result.status}")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
