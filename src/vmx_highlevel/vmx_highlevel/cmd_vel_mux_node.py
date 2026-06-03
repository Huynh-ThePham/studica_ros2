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


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class CmdVelMuxNode(Node):
    """Small PC-side /cmd_vel arbiter for gamepad keypad and autonomy commands.

    D-pad and face button commands read directly from /joy have priority while
    active. Autonomous commands can still feed /cmd_vel_nav when the keypad is idle.
    """

    def __init__(self) -> None:
        super().__init__("vmx_cmd_vel_mux_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._last_nav_msg: Optional[Twist] = None
        self._last_nav_time = -1.0
        self._last_joy_state_time = -1.0
        self._joy_state_seen = False
        self._last_dpad_msg: Optional[Twist] = None
        self._last_dpad_active_time = -1.0
        self._last_source = "idle"

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
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
            f"joy_state={self._joy_state_topic}, "
            f"nav={self._nav_topic}, output={self._output_topic}, "
            f"joy_timeout={self._joy_timeout:.2f}s nav_timeout={self._nav_timeout:.2f}s"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "joy_state_topic": "/joy",
            "nav_topic": "/cmd_vel_nav",
            "output_topic": "/cmd_vel",
            "status_topic": "/cmd_vel_mux_status",
            "publish_rate_hz": 30.0,
            "status_rate_hz": 2.0,
            "joy_timeout": 0.35,
            "nav_timeout": 0.35,
            "joy_lockout_after_active": 0.50,
            "active_threshold": 1e-4,
            "stop_on_idle": True,
            "dpad_enable": True,
            "dpad_axis_linear_x": 7,
            "dpad_axis_angular_z": 6,
            "dpad_scale_linear_x": -0.70,
            "dpad_scale_angular_z": 3.20,
            "dpad_turbo_scale_linear_x": -1.00,
            "dpad_turbo_scale_angular_z": 5.20,
            "dpad_turbo_button": 5,
            "dpad_linear_turbo_button": 4,
            "dpad_angular_turbo_button": 5,
            "dpad_turn_left_button": 1,
            "dpad_turn_right_button": 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self._nav_topic = str(self.get_parameter("nav_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self._joy_timeout = float(self.get_parameter("joy_timeout").value)
        self._nav_timeout = float(self.get_parameter("nav_timeout").value)
        self._joy_lockout_after_active = float(
            self.get_parameter("joy_lockout_after_active").value
        )
        self._active_threshold = float(self.get_parameter("active_threshold").value)
        self._stop_on_idle = bool(self.get_parameter("stop_on_idle").value)
        self._dpad_enable = bool(self.get_parameter("dpad_enable").value)
        self._dpad_axis_linear_x = int(self.get_parameter("dpad_axis_linear_x").value)
        self._dpad_axis_angular_z = int(self.get_parameter("dpad_axis_angular_z").value)
        self._dpad_scale_linear_x = float(self.get_parameter("dpad_scale_linear_x").value)
        self._dpad_scale_angular_z = float(self.get_parameter("dpad_scale_angular_z").value)
        self._dpad_turbo_scale_linear_x = float(
            self.get_parameter("dpad_turbo_scale_linear_x").value
        )
        self._dpad_turbo_scale_angular_z = float(
            self.get_parameter("dpad_turbo_scale_angular_z").value
        )
        self._dpad_turbo_button = int(self.get_parameter("dpad_turbo_button").value)
        self._dpad_linear_turbo_button = int(
            self.get_parameter("dpad_linear_turbo_button").value
        )
        self._dpad_angular_turbo_button = int(
            self.get_parameter("dpad_angular_turbo_button").value
        )
        self._dpad_turn_left_button = int(
            self.get_parameter("dpad_turn_left_button").value
        )
        self._dpad_turn_right_button = int(
            self.get_parameter("dpad_turn_right_button").value
        )

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
        if self._output_topic == self._nav_topic:
            raise ValueError("output_topic must differ from nav_topic")
        if self._dpad_axis_linear_x < 0 or self._dpad_axis_angular_z < 0:
            raise ValueError("dpad axis indexes must be >= 0")
        if self._dpad_turbo_button < 0:
            raise ValueError("dpad_turbo_button must be >= 0")
        for name, value in (
            ("dpad_linear_turbo_button", self._dpad_linear_turbo_button),
            ("dpad_angular_turbo_button", self._dpad_angular_turbo_button),
            ("dpad_turn_left_button", self._dpad_turn_left_button),
            ("dpad_turn_right_button", self._dpad_turn_right_button),
        ):
            if value < 0:
                raise ValueError(f"{name} must be >= 0")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _joy_state_callback(self, msg: Joy) -> None:
        self._joy_state_seen = True
        self._last_joy_state_time = self._now()
        self._last_dpad_msg = self._dpad_command_from_joy(msg)
        if twist_is_active(self._last_dpad_msg, self._active_threshold):
            self._last_dpad_active_time = self._last_joy_state_time

    def _axis(self, msg: Joy, index: int) -> float:
        if 0 <= index < len(msg.axes) and math.isfinite(msg.axes[index]):
            return float(msg.axes[index])
        return 0.0

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def _dpad_command_from_joy(self, msg: Joy) -> Twist:
        command = Twist()
        if not self._dpad_enable:
            return command
        linear_turbo = self._button_pressed(msg, self._dpad_linear_turbo_button)
        angular_turbo = self._button_pressed(msg, self._dpad_angular_turbo_button)
        linear_scale = (
            self._dpad_turbo_scale_linear_x if linear_turbo else self._dpad_scale_linear_x
        )
        angular_scale = (
            self._dpad_turbo_scale_angular_z if angular_turbo else self._dpad_scale_angular_z
        )
        command.linear.x = self._axis(msg, self._dpad_axis_linear_x) * linear_scale
        if self._button_pressed(msg, self._dpad_turn_left_button):
            command.angular.z += angular_scale
        if self._button_pressed(msg, self._dpad_turn_right_button):
            command.angular.z -= angular_scale
        return command

    def _combined_joy_command(self) -> Twist:
        command = Twist()
        if self._last_dpad_msg is not None:
            command.linear.x += self._last_dpad_msg.linear.x
            command.angular.z += self._last_dpad_msg.angular.z
        max_linear = max(
            abs(self._dpad_scale_linear_x),
            abs(self._dpad_turbo_scale_linear_x),
            abs(command.linear.x),
        )
        max_angular = max(
            abs(self._dpad_scale_angular_z),
            abs(self._dpad_turbo_scale_angular_z),
            abs(command.angular.z),
        )
        command.linear.x = clamp(command.linear.x, -max_linear, max_linear)
        command.angular.z = clamp(command.angular.z, -max_angular, max_angular)
        return command

    def _nav_callback(self, msg: Twist) -> None:
        self._last_nav_msg = msg
        self._last_nav_time = self._now()

    def _select_command(self) -> Tuple[str, Twist]:
        now = self._now()
        nav_fresh = (
            self._last_nav_msg is not None and
            now - self._last_nav_time <= self._nav_timeout
        )
        joy_state_fresh = (
            self._joy_state_seen and
            now - self._last_joy_state_time <= self._joy_timeout
        )
        dpad_recently_active = (
            self._last_dpad_active_time >= 0.0 and
            now - self._last_dpad_active_time <= self._joy_lockout_after_active
        )

        if joy_state_fresh:
            if self._last_dpad_msg is not None and twist_is_active(
                self._last_dpad_msg,
                self._active_threshold,
            ):
                return "joy_keypad", self._combined_joy_command()
            if dpad_recently_active:
                return "joy_stop", zero_twist()
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
        joy_state_age = (
            now - self._last_joy_state_time if self._last_joy_state_time >= 0.0 else -1.0
        )
        nav_age = now - self._last_nav_time if self._last_nav_time >= 0.0 else -1.0
        msg = String()
        msg.data = (
            "mode=cmd_vel_mux "
            f"source={self._last_source} "
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
