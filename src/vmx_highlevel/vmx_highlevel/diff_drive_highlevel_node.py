import math
from typing import Dict, Iterable, List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray, Int32, String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


FIXED_4WD_WHEEL_PORTS = {
    "front_left": 2,
    "front_right": 0,
    "rear_left": 3,
    "rear_right": 1,
}
FIXED_4WD_MOTOR_COMMAND_SIGN = [1.0, 1.0, -1.0, -1.0]
FIXED_4WD_ENCODER_SIGN = [-1.0, -1.0, 1.0, 1.0]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> List[float]:
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def side_powers_from_twist(
    linear_velocity: float,
    angular_velocity: float,
    wheelbase: float,
    max_wheel_linear_velocity: float,
    max_motor_power: float,
) -> tuple[float, float]:
    left_velocity = linear_velocity - angular_velocity * wheelbase * 0.5
    right_velocity = linear_velocity + angular_velocity * wheelbase * 0.5
    left_power = clamp(
        left_velocity / max_wheel_linear_velocity,
        -max_motor_power,
        max_motor_power,
    )
    right_power = clamp(
        right_velocity / max_wheel_linear_velocity,
        -max_motor_power,
        max_motor_power,
    )
    return left_power, right_power


def apply_motor_trim(
    port: int,
    wheel_power: float,
    motor_command_sign: List[float],
    motor_command_gain: List[float],
    motor_min_power: List[float],
    max_motor_power: float,
) -> float:
    signed_power = wheel_power * motor_command_sign[port] * motor_command_gain[port]
    if abs(signed_power) > 1e-6 and motor_min_power[port] > 0.0:
        signed_power = math.copysign(
            max(abs(signed_power), motor_min_power[port]),
            signed_power,
        )
    return clamp(signed_power, -max_motor_power, max_motor_power)


def compute_4wd_motor_power(
    linear_velocity: float,
    angular_velocity: float,
    wheelbase: float,
    max_wheel_linear_velocity: float,
    max_motor_power: float,
    wheel_ports: Dict[str, int],
    motor_command_sign: List[float],
    motor_command_gain: List[float],
    motor_min_power: List[float],
) -> List[float]:
    left_power, right_power = side_powers_from_twist(
        linear_velocity,
        angular_velocity,
        wheelbase,
        max_wheel_linear_velocity,
        max_motor_power,
    )

    motor_power = [0.0, 0.0, 0.0, 0.0]
    motor_power[wheel_ports["front_left"]] = left_power
    motor_power[wheel_ports["rear_left"]] = left_power
    motor_power[wheel_ports["front_right"]] = right_power
    motor_power[wheel_ports["rear_right"]] = right_power
    return [
        apply_motor_trim(
            port,
            power,
            motor_command_sign,
            motor_command_gain,
            motor_min_power,
            max_motor_power,
        )
        for port, power in enumerate(motor_power)
    ]


class VmxHighlevelNode(Node):
    """PC-side differential-drive bridge for the VMX low-level topic contract."""

    def __init__(self) -> None:
        super().__init__("vmx_highlevel_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._dist_per_tick = (2.0 * math.pi * self._wheel_radius) / float(self._ticks_per_rotation)
        self._encoder_counts: Dict[int, Optional[int]] = {port: None for port in range(4)}
        self._last_encoder_counts: Dict[int, Optional[int]] = {port: None for port in range(4)}
        self._speed_rpm: Dict[int, float] = {port: 0.0 for port in range(4)}
        self._have_imu = False
        self._imu_yaw = 0.0
        self._last_imu_yaw: Optional[float] = None
        self._have_cmd = False
        self._last_cmd_time = self._now_sec()
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._last_odom_time: Optional[float] = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_motor_power = [0.0, 0.0, 0.0, 0.0]

        motor_cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._motor_power_pub = self.create_publisher(
            Float32MultiArray, self._motor_power_topic, motor_cmd_qos
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 5)
        self._odom_pub = (
            self.create_publisher(Odometry, self._odom_topic, 10)
            if self._publish_odom
            else None
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        sub_qos = (
            qos_profile_sensor_data
            if self._subscribe_best_effort
            else QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_callback, cmd_qos)
        self.create_subscription(Imu, self._imu_topic, self._imu_callback, sub_qos)
        for port in range(4):
            self.create_subscription(
                Int32,
                f"/titan/motor{port}/encoder",
                lambda msg, port=port: self._encoder_callback(port, msg),
                sub_qos,
            )
            self.create_subscription(
                Float32,
                f"/titan/motor{port}/speed",
                lambda msg, port=port: self._speed_callback(port, msg),
                sub_qos,
            )

        self._reset_srv = self.create_service(
            Empty, self._reset_odometry_service, self._reset_odometry_callback
        )

        self.create_timer(1.0 / self._control_rate_hz, self._control_timer_callback)
        self.create_timer(1.0 / self._odom_rate_hz, self._odom_timer_callback)
        self.create_timer(1.0 / self._status_rate_hz, self._status_timer_callback)

        self.get_logger().info(
            f"VMX PC high-level: layout={self._drive_layout_name()} "
            f"ports={self._wheel_ports} best_effort_sub={self._subscribe_best_effort} "
            f"reset_srv={self._reset_odometry_service} "
            "/cmd_vel -> motor_power; encoders+imu -> odom/tf."
        )

    def _drive_layout_name(self) -> str:
        if self._fixed_4wd_layout:
            return "fixed_4wd_m0m1_right_m2m3_left"
        return "custom_4wd"

    def _declare_parameters(self) -> None:
        defaults = {
            "fixed_4wd_layout": True,
            "wheel_radius": 0.05,
            "wheelbase": 0.28,
            "ticks_per_rotation": 1464,
            "front_left_port": FIXED_4WD_WHEEL_PORTS["front_left"],
            "front_right_port": FIXED_4WD_WHEEL_PORTS["front_right"],
            "rear_left_port": FIXED_4WD_WHEEL_PORTS["rear_left"],
            "rear_right_port": FIXED_4WD_WHEEL_PORTS["rear_right"],
            "motor_command_sign_m0": FIXED_4WD_MOTOR_COMMAND_SIGN[0],
            "motor_command_sign_m1": FIXED_4WD_MOTOR_COMMAND_SIGN[1],
            "motor_command_sign_m2": FIXED_4WD_MOTOR_COMMAND_SIGN[2],
            "motor_command_sign_m3": FIXED_4WD_MOTOR_COMMAND_SIGN[3],
            "motor_command_gain_m0": 1.05,
            "motor_command_gain_m1": 1.0,
            "motor_command_gain_m2": 1.04,
            "motor_command_gain_m3": 1.03,
            "motor_min_power_m0": 0.20,
            "motor_min_power_m1": 0.20,
            "motor_min_power_m2": 0.20,
            "motor_min_power_m3": 0.20,
            "encoder_sign_m0": FIXED_4WD_ENCODER_SIGN[0],
            "encoder_sign_m1": FIXED_4WD_ENCODER_SIGN[1],
            "encoder_sign_m2": FIXED_4WD_ENCODER_SIGN[2],
            "encoder_sign_m3": FIXED_4WD_ENCODER_SIGN[3],
            "max_linear_velocity": 0.70,
            "max_angular_velocity": 1.80,
            "max_wheel_linear_velocity": 0.70,
            "max_motor_power": 0.70,
            "cmd_vel_timeout": 0.25,
            "control_rate_hz": 20.0,
            "odom_rate_hz": 30.0,
            "status_rate_hz": 1.0,
            "use_imu_yaw_for_odom": True,
            "publish_odom": True,
            "publish_tf": True,
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "cmd_vel_topic": "/cmd_vel",
            "motor_power_topic": "/titan/motor_power",
            "imu_topic": "/imu",
            "odom_topic": "/odom",
            "status_topic": "/highlevel_status",
            "pose_covariance_xy": 0.02,
            "pose_covariance_yaw": 0.05,
            "twist_covariance_linear": 0.05,
            "twist_covariance_angular": 0.10,
            "subscribe_best_effort": False,
            "reset_odometry_service": "/reset_odometry",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._fixed_4wd_layout = bool(self.get_parameter("fixed_4wd_layout").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._ticks_per_rotation = int(self.get_parameter("ticks_per_rotation").value)
        if self._fixed_4wd_layout:
            self._wheel_ports = dict(FIXED_4WD_WHEEL_PORTS)
            self._motor_command_sign = list(FIXED_4WD_MOTOR_COMMAND_SIGN)
            self._encoder_sign = list(FIXED_4WD_ENCODER_SIGN)
        else:
            self._wheel_ports = {
                "front_left": int(self.get_parameter("front_left_port").value),
                "front_right": int(self.get_parameter("front_right_port").value),
                "rear_left": int(self.get_parameter("rear_left_port").value),
                "rear_right": int(self.get_parameter("rear_right_port").value),
            }
            self._motor_command_sign = [
                float(self.get_parameter(f"motor_command_sign_m{port}").value)
                for port in range(4)
            ]
            self._encoder_sign = [
                float(self.get_parameter(f"encoder_sign_m{port}").value)
                for port in range(4)
            ]
        self._motor_command_gain = [
            float(self.get_parameter(f"motor_command_gain_m{port}").value)
            for port in range(4)
        ]
        self._motor_min_power = [
            float(self.get_parameter(f"motor_min_power_m{port}").value)
            for port in range(4)
        ]
        self._max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self._max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self._max_wheel_linear_velocity = float(
            self.get_parameter("max_wheel_linear_velocity").value
        )
        self._max_motor_power = float(self.get_parameter("max_motor_power").value)
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._odom_rate_hz = float(self.get_parameter("odom_rate_hz").value)
        self._status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self._use_imu_yaw_for_odom = bool(self.get_parameter("use_imu_yaw_for_odom").value)
        self._publish_odom = bool(self.get_parameter("publish_odom").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._motor_power_topic = str(self.get_parameter("motor_power_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._pose_covariance_xy = float(self.get_parameter("pose_covariance_xy").value)
        self._pose_covariance_yaw = float(self.get_parameter("pose_covariance_yaw").value)
        self._twist_covariance_linear = float(
            self.get_parameter("twist_covariance_linear").value
        )
        self._twist_covariance_angular = float(
            self.get_parameter("twist_covariance_angular").value
        )
        self._subscribe_best_effort = bool(self.get_parameter("subscribe_best_effort").value)
        self._reset_odometry_service = str(self.get_parameter("reset_odometry_service").value)

    def _validate_parameters(self) -> None:
        if self._wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be > 0")
        if self._wheelbase <= 0.0:
            raise ValueError("wheelbase must be > 0")
        if self._ticks_per_rotation <= 0:
            raise ValueError("ticks_per_rotation must be > 0")
        if self._max_wheel_linear_velocity <= 0.0:
            raise ValueError("max_wheel_linear_velocity must be > 0")
        if self._cmd_vel_timeout <= 0.0:
            raise ValueError("cmd_vel_timeout must be > 0")
        for rate_name, rate in (
            ("control_rate_hz", self._control_rate_hz),
            ("odom_rate_hz", self._odom_rate_hz),
            ("status_rate_hz", self._status_rate_hz),
        ):
            if rate <= 0.0:
                raise ValueError(f"{rate_name} must be > 0")

        if self._control_rate_hz > 50.0:
            raise ValueError(
                "control_rate_hz must be <= 50 Hz (>50 Hz saturates the Titan/CAN "
                "path on the VMX side with 4 motors; field-tested sweet spot is 20 Hz)"
            )
        if self._control_rate_hz > 30.0:
            self.get_logger().warning(
                f"control_rate_hz={self._control_rate_hz:.1f} Hz is above the field-tested "
                "safe limit (~30 Hz). On 4-motor setups this can desync wheels via VMX/CAN. "
                "Recommended: 20 Hz (matches the VMX low-level default)."
            )

        ports = list(self._wheel_ports.values())
        if sorted(ports) != [0, 1, 2, 3]:
            raise ValueError(
                "front/rear left/right ports must be a unique permutation of [0, 1, 2, 3]"
            )
        for signs, label in (
            (self._motor_command_sign, "motor_command_sign"),
            (self._encoder_sign, "encoder_sign"),
        ):
            for port, sign in enumerate(signs):
                if sign not in (-1.0, 1.0):
                    raise ValueError(f"{label}_m{port} must be either -1.0 or 1.0")
        for gains, label in (
            (self._motor_command_gain, "motor_command_gain"),
            (self._motor_min_power, "motor_min_power"),
        ):
            for port, value in enumerate(gains):
                if value < 0.0:
                    raise ValueError(f"{label}_m{port} must be >= 0.0")
        for port, value in enumerate(self._motor_min_power):
            if value > self._max_motor_power:
                raise ValueError(f"motor_min_power_m{port} must be <= max_motor_power")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._target_linear = clamp(
            float(msg.linear.x), -self._max_linear_velocity, self._max_linear_velocity
        )
        self._target_angular = clamp(
            float(msg.angular.z), -self._max_angular_velocity, self._max_angular_velocity
        )
        self._last_cmd_time = self._now_sec()
        self._have_cmd = True

    def _encoder_callback(self, port: int, msg: Int32) -> None:
        self._encoder_counts[port] = int(msg.data)

    def _speed_callback(self, port: int, msg: Float32) -> None:
        self._speed_rpm[port] = float(msg.data)

    def _imu_callback(self, msg: Imu) -> None:
        self._imu_yaw = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self._have_imu = True

    def _reset_odometry_callback(self, _request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_time = self._now_sec()
        if all(count is not None for count in self._encoder_counts.values()):
            self._last_encoder_counts = dict(self._encoder_counts)
        else:
            self._last_encoder_counts = {port: None for port in range(4)}
        if self._have_imu:
            self._last_imu_yaw = self._imu_yaw
        else:
            self._last_imu_yaw = None
        self.get_logger().info("Odometry reset: pose (0,0,0), encoder/IMU baseline refreshed.")
        return response

    def _control_timer_callback(self) -> None:
        if not self._have_cmd or self._now_sec() - self._last_cmd_time > self._cmd_vel_timeout:
            self._publish_motor_power([0.0, 0.0, 0.0, 0.0])
            self._have_cmd = False
            return

        motor_power = compute_4wd_motor_power(
            self._target_linear,
            self._target_angular,
            self._wheelbase,
            self._max_wheel_linear_velocity,
            self._max_motor_power,
            self._wheel_ports,
            self._motor_command_sign,
            self._motor_command_gain,
            self._motor_min_power,
        )
        self._publish_motor_power(motor_power)

    def _apply_motor_trim(self, port: int, wheel_power: float) -> float:
        return apply_motor_trim(
            port,
            wheel_power,
            self._motor_command_sign,
            self._motor_command_gain,
            self._motor_min_power,
            self._max_motor_power,
        )

    def _publish_motor_power(self, motor_power: Iterable[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(value) for value in motor_power]
        self._last_motor_power = list(msg.data)
        self._motor_power_pub.publish(msg)

    def _odom_timer_callback(self) -> None:
        if not self._publish_odom and not self._publish_tf:
            return
        if not all(count is not None for count in self._encoder_counts.values()):
            return

        now = self._now_sec()
        if self._last_odom_time is None:
            self._last_odom_time = now
            self._last_encoder_counts = dict(self._encoder_counts)
            self._last_imu_yaw = self._imu_yaw if self._have_imu else None
            return

        dt = now - self._last_odom_time
        if dt <= 0.0:
            return

        wheel_delta = {}
        for port in range(4):
            current_count = self._encoder_counts[port]
            previous_count = self._last_encoder_counts[port]
            if current_count is None or previous_count is None:
                return
            delta_count = current_count - previous_count
            wheel_delta[port] = delta_count * self._dist_per_tick * self._encoder_sign[port]

        left_delta = 0.5 * (
            wheel_delta[self._wheel_ports["front_left"]]
            + wheel_delta[self._wheel_ports["rear_left"]]
        )
        right_delta = 0.5 * (
            wheel_delta[self._wheel_ports["front_right"]]
            + wheel_delta[self._wheel_ports["rear_right"]]
        )
        distance_delta = 0.5 * (left_delta + right_delta)
        wheel_yaw_delta = (right_delta - left_delta) / self._wheelbase

        yaw_delta = wheel_yaw_delta
        if self._use_imu_yaw_for_odom and self._have_imu:
            if self._last_imu_yaw is None:
                self._last_imu_yaw = self._imu_yaw
            yaw_delta = normalize_angle(self._imu_yaw - self._last_imu_yaw)

        yaw_mid = self._yaw + yaw_delta * 0.5
        self._x += distance_delta * math.cos(yaw_mid)
        self._y += distance_delta * math.sin(yaw_mid)
        self._yaw = normalize_angle(self._yaw + yaw_delta)

        linear_velocity = distance_delta / dt
        angular_velocity = yaw_delta / dt

        self._last_odom_time = now
        self._last_encoder_counts = dict(self._encoder_counts)
        self._last_imu_yaw = self._imu_yaw if self._have_imu else self._last_imu_yaw

        stamp = self.get_clock().now().to_msg()
        if self._odom_pub is not None:
            self._publish_odom_msg(stamp, linear_velocity, angular_velocity)
        if self._tf_broadcaster is not None:
            self._publish_tf_msg(stamp)

    def _publish_odom_msg(self, stamp, linear_velocity: float, angular_velocity: float) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        qx, qy, qz, qw = quaternion_from_yaw(self._yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity
        msg.pose.covariance[0] = self._pose_covariance_xy
        msg.pose.covariance[7] = self._pose_covariance_xy
        msg.pose.covariance[35] = self._pose_covariance_yaw
        msg.twist.covariance[0] = self._twist_covariance_linear
        msg.twist.covariance[35] = self._twist_covariance_angular
        self._odom_pub.publish(msg)

    def _publish_tf_msg(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame_id
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self._yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    def _status_timer_callback(self) -> None:
        age = self._now_sec() - self._last_cmd_time if self._have_cmd else -1.0
        encoders_ready = all(count is not None for count in self._encoder_counts.values())
        msg = String()
        msg.data = (
            "mode=pc_highlevel "
            f"layout={self._drive_layout_name()} "
            f"encoders_ready={str(encoders_ready).lower()} "
            f"imu_ready={str(self._have_imu).lower()} "
            f"have_cmd={str(self._have_cmd).lower()} "
            f"cmd_age={age:.3f} "
            f"last_motor_power=[{','.join(f'{value:.3f}' for value in self._last_motor_power)}] "
            f"odom_x={self._x:.4f} odom_y={self._y:.4f} odom_yaw={self._yaw:.4f}"
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VmxHighlevelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_motor_power([0.0, 0.0, 0.0, 0.0])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
