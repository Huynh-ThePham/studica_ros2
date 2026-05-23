import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def twist_is_active(msg: Twist, threshold: float) -> bool:
    values = (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    )
    return any(math.isfinite(value) and abs(value) > threshold for value in values)


def zero_twist() -> Twist:
    return Twist()


class CmdVelMuxNode(Node):
    """Small PC-side /cmd_vel arbiter for joystick and autonomy commands.

    Joystick commands have priority while the deadman is held. If the joy state
    topic is not present, the node falls back to non-zero Twist priority.
    """

    def __init__(self) -> None:
        super().__init__("vmx_cmd_vel_mux_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._last_joy_msg: Optional[Twist] = None
        self._last_nav_msg: Optional[Twist] = None
        self._last_joy_time = -1.0
        self._last_nav_time = -1.0
        self._last_joy_active_time = -1.0
        self._last_joy_state_time = -1.0
        self._joy_state_seen = False
        self._joy_enabled = False
        self._last_source = "idle"

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, self._joy_topic, self._joy_callback, qos)
        self.create_subscription(Twist, self._nav_topic, self._nav_callback, qos)
        self.create_subscription(
            Joy, self._joy_state_topic, self._joy_state_callback, qos
        )
        self._cmd_pub = self.create_publisher(Twist, self._output_topic, qos)
        self._status_pub = self.create_publisher(String, self._status_topic, 5)

        self.create_timer(1.0 / self._publish_rate_hz, self._publish_timer)
        self.create_timer(1.0 / self._status_rate_hz, self._status_timer)

        self.get_logger().info(
            "VMX cmd_vel mux ready: "
            f"joy={self._joy_topic}, joy_state={self._joy_state_topic}, "
            f"nav={self._nav_topic}, output={self._output_topic}, "
            f"joy_timeout={self._joy_timeout:.2f}s nav_timeout={self._nav_timeout:.2f}s"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "joy_topic": "/cmd_vel_joy",
            "joy_state_topic": "/joy",
            "nav_topic": "/cmd_vel_nav",
            "output_topic": "/cmd_vel",
            "status_topic": "/cmd_vel_mux_status",
            "joy_enable_button": 4,
            "publish_rate_hz": 30.0,
            "status_rate_hz": 2.0,
            "joy_timeout": 0.35,
            "nav_timeout": 0.35,
            "joy_lockout_after_active": 0.50,
            "active_threshold": 1e-4,
            "stop_on_idle": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._joy_topic = str(self.get_parameter("joy_topic").value)
        self._joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self._nav_topic = str(self.get_parameter("nav_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._joy_enable_button = int(self.get_parameter("joy_enable_button").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self._joy_timeout = float(self.get_parameter("joy_timeout").value)
        self._nav_timeout = float(self.get_parameter("nav_timeout").value)
        self._joy_lockout_after_active = float(
            self.get_parameter("joy_lockout_after_active").value
        )
        self._active_threshold = float(self.get_parameter("active_threshold").value)
        self._stop_on_idle = bool(self.get_parameter("stop_on_idle").value)

    def _validate_parameters(self) -> None:
        for name, value in (
            ("publish_rate_hz", self._publish_rate_hz),
            ("status_rate_hz", self._status_rate_hz),
            ("joy_timeout", self._joy_timeout),
            ("nav_timeout", self._nav_timeout),
            ("joy_lockout_after_active", self._joy_lockout_after_active),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be > 0")
        if self._active_threshold < 0.0:
            raise ValueError("active_threshold must be >= 0")
        if self._output_topic in (self._joy_topic, self._nav_topic):
            raise ValueError("output_topic must differ from joy_topic and nav_topic")
        if self._joy_enable_button < 0:
            raise ValueError("joy_enable_button must be >= 0")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _joy_callback(self, msg: Twist) -> None:
        self._last_joy_msg = msg
        self._last_joy_time = self._now()
        if twist_is_active(msg, self._active_threshold):
            self._last_joy_active_time = self._last_joy_time

    def _joy_state_callback(self, msg: Joy) -> None:
        self._joy_state_seen = True
        self._last_joy_state_time = self._now()
        self._joy_enabled = (
            self._joy_enable_button < len(msg.buttons) and
            msg.buttons[self._joy_enable_button] == 1
        )

    def _nav_callback(self, msg: Twist) -> None:
        self._last_nav_msg = msg
        self._last_nav_time = self._now()

    def _select_command(self) -> Tuple[str, Twist]:
        now = self._now()
        joy_fresh = (
            self._last_joy_msg is not None and
            now - self._last_joy_time <= self._joy_timeout
        )
        nav_fresh = (
            self._last_nav_msg is not None and
            now - self._last_nav_time <= self._nav_timeout
        )
        joy_state_fresh = (
            self._joy_state_seen and
            now - self._last_joy_state_time <= self._joy_timeout
        )
        joy_recently_active = (
            self._last_joy_active_time >= 0.0 and
            now - self._last_joy_active_time <= self._joy_lockout_after_active
        )

        if joy_state_fresh:
            if self._joy_enabled:
                return "joy", self._last_joy_msg if joy_fresh else zero_twist()
            if joy_recently_active:
                return "joy_stop", zero_twist()
        elif joy_fresh and joy_recently_active:
            return (
                "joy_fallback",
                self._last_joy_msg if self._last_joy_msg is not None else zero_twist(),
            )
        if nav_fresh:
            return "nav", self._last_nav_msg if self._last_nav_msg is not None else zero_twist()
        return "idle", zero_twist()

    def _publish_timer(self) -> None:
        source, command = self._select_command()
        self._last_source = source
        if source != "idle" or self._stop_on_idle:
            self._cmd_pub.publish(command)

    def _status_timer(self) -> None:
        now = self._now()
        joy_age = now - self._last_joy_time if self._last_joy_time >= 0.0 else -1.0
        joy_state_age = (
            now - self._last_joy_state_time if self._last_joy_state_time >= 0.0 else -1.0
        )
        nav_age = now - self._last_nav_time if self._last_nav_time >= 0.0 else -1.0
        msg = String()
        msg.data = (
            "mode=cmd_vel_mux "
            f"source={self._last_source} "
            f"joy_enabled={str(self._joy_enabled).lower()} "
            f"joy_age={joy_age:.3f} "
            f"joy_state_age={joy_state_age:.3f} "
            f"nav_age={nav_age:.3f} "
            f"joy_timeout={self._joy_timeout:.3f} "
            f"nav_timeout={self._nav_timeout:.3f}"
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._cmd_pub.publish(zero_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
