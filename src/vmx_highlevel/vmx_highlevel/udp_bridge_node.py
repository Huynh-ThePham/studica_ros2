import socket
import threading
import time
from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray, Int32, String

from vmx_highlevel.udp_protocol import (
    STATUS_CHECKSUM_ERROR,
    STATUS_COMMAND_SEEN,
    STATUS_COMMAND_TIMEOUT,
    STATUS_IMU_OK,
    STATUS_MOTOR_ENABLED,
    STATUS_STOPPING,
    STATUS_TITAN_OK,
    STATUS_VMX_OK,
    TELEMETRY_SIZE,
    build_command_packet,
    clamp_motor,
    parse_telemetry_packet,
)


class VmxUdpBridgeNode(Node):
    """PC-side ROS bridge for the ROS-free VMX UDP low-level daemon."""

    def __init__(self) -> None:
        super().__init__("vmx_udp_bridge_node")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._motor_command = [0.0, 0.0, 0.0, 0.0]
        self._last_motor_command_time = 0.0
        self._have_motor_command = False
        self._seq = 0
        self._last_telemetry_time = 0.0
        self._last_telemetry_seq = 0
        self._last_cmd_seq_seen_by_vmx = 0
        self._last_status_bits = 0
        self._last_encoder = [0, 0, 0, 0]
        self._last_rpm = [0.0, 0.0, 0.0, 0.0]
        self._packet_errors = 0
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Float32MultiArray,
            self._motor_power_topic,
            self._motor_power_callback,
            qos_reliable,
        )

        self._imu_pub = self.create_publisher(Imu, self._imu_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 5)
        self._encoder_pubs = [
            self.create_publisher(Int32, self._encoder_topic_format.format(port=port), 10)
            for port in range(4)
        ]
        self._speed_pubs = [
            self.create_publisher(Float32, self._speed_topic_format.format(port=port), 10)
            for port in range(4)
        ]

        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._telemetry_socket.bind(("0.0.0.0", self._telemetry_port))
        self._telemetry_socket.settimeout(0.1)

        self._receiver_thread = threading.Thread(
            target=self._receive_loop,
            name="vmx_udp_telemetry",
            daemon=True,
        )
        self._receiver_thread.start()

        self.create_timer(1.0 / self._command_rate_hz, self._send_command_timer)
        self.create_timer(1.0 / self._status_rate_hz, self._status_timer)

        self.get_logger().info(
            "VMX UDP bridge ready: "
            f"VMX={self._vmx_host}:{self._command_port}, "
            f"telemetry_port={self._telemetry_port}, "
            f"motor_power_topic={self._motor_power_topic}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "vmx_host": "172.22.11.2",
            "command_port": 15000,
            "telemetry_port": 15001,
            "command_rate_hz": 10.0,
            "command_timeout": 0.50,
            "status_rate_hz": 2.0,
            "telemetry_timeout": 1.0,
            "motor_power_topic": "/titan/motor_power",
            "imu_topic": "/imu",
            "status_topic": "/lowlevel_status",
            "encoder_topic_format": "/titan/motor{port}/encoder",
            "speed_topic_format": "/titan/motor{port}/speed",
            "imu_frame_id": "imu_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._vmx_host = str(self.get_parameter("vmx_host").value)
        self._command_port = int(self.get_parameter("command_port").value)
        self._telemetry_port = int(self.get_parameter("telemetry_port").value)
        self._command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self._telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)
        self._motor_power_topic = str(self.get_parameter("motor_power_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._encoder_topic_format = str(self.get_parameter("encoder_topic_format").value)
        self._speed_topic_format = str(self.get_parameter("speed_topic_format").value)
        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)

    def _validate_parameters(self) -> None:
        if not (1 <= self._command_port <= 65535):
            raise ValueError("command_port must be in range 1..65535")
        if not (1 <= self._telemetry_port <= 65535):
            raise ValueError("telemetry_port must be in range 1..65535")
        if self._command_rate_hz <= 0.0 or self._command_rate_hz > 50.0:
            raise ValueError("command_rate_hz must be in range (0, 50]")
        if self._status_rate_hz <= 0.0:
            raise ValueError("status_rate_hz must be > 0")
        if self._command_timeout <= 0.0:
            raise ValueError("command_timeout must be > 0")
        for template_name, template_value in (
            ("encoder_topic_format", self._encoder_topic_format),
            ("speed_topic_format", self._speed_topic_format),
        ):
            try:
                template_value.format(port=0)
            except Exception as exc:
                raise ValueError(f"{template_name} must contain '{{port}}'") from exc

    def _now(self) -> float:
        return time.monotonic()

    def _motor_power_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warning(
                f"Ignoring {self._motor_power_topic}: expected 4 values, got {len(msg.data)}",
                throttle_duration_sec=1.0,
            )
            return
        with self._lock:
            self._motor_command = [clamp_motor(float(msg.data[i])) for i in range(4)]
            self._last_motor_command_time = self._now()
            self._have_motor_command = True

    def _send_command_timer(self) -> None:
        with self._lock:
            command = list(self._motor_command)
            command_age = self._now() - self._last_motor_command_time
            fresh = self._have_motor_command and command_age <= self._command_timeout
            if not fresh:
                command = [0.0, 0.0, 0.0, 0.0]
            enable = fresh and any(abs(value) > 1e-5 for value in command)
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            seq = self._seq

        packet = build_command_packet(seq, command, enable=enable)
        try:
            self._command_socket.sendto(packet, (self._vmx_host, self._command_port))
        except OSError as exc:
            self.get_logger().warning(f"UDP command send failed: {exc}", throttle_duration_sec=1.0)

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                packet, _ = self._telemetry_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    self._packet_errors += 1
                continue

            if len(packet) != TELEMETRY_SIZE:
                self._packet_errors += 1
                continue
            try:
                self._handle_telemetry(packet)
            except Exception as exc:
                self._packet_errors += 1
                self.get_logger().warning(
                    f"Telemetry parse failed: {exc}",
                    throttle_duration_sec=1.0,
                )

    def _handle_telemetry(self, packet: bytes) -> None:
        telemetry = parse_telemetry_packet(packet)
        encoder = telemetry.encoder
        rpm = telemetry.rpm
        quat = telemetry.orientation_xyzw
        gyro = telemetry.angular_velocity_rad_s
        accel = telemetry.linear_acceleration_m_s2
        status_bits = telemetry.status_bits
        last_command_sequence = telemetry.last_command_sequence

        stamp = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._imu_frame_id
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.orientation_covariance[0] = 0.02
        imu_msg.orientation_covariance[4] = 0.02
        imu_msg.orientation_covariance[8] = 0.05
        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02
        imu_msg.linear_acceleration_covariance[0] = 0.10
        imu_msg.linear_acceleration_covariance[4] = 0.10
        imu_msg.linear_acceleration_covariance[8] = 0.10
        self._imu_pub.publish(imu_msg)

        for port in range(4):
            enc_msg = Int32()
            enc_msg.data = encoder[port]
            self._encoder_pubs[port].publish(enc_msg)
            speed_msg = Float32()
            speed_msg.data = rpm[port]
            self._speed_pubs[port].publish(speed_msg)

        with self._lock:
            self._last_telemetry_time = self._now()
            self._last_telemetry_seq = telemetry.sequence
            self._last_cmd_seq_seen_by_vmx = last_command_sequence
            self._last_status_bits = status_bits
            self._last_encoder = encoder
            self._last_rpm = rpm

    def _decode_status(self, status_bits: int) -> Dict[str, bool]:
        return {
            "vmx_ok": bool(status_bits & STATUS_VMX_OK),
            "imu_ok": bool(status_bits & STATUS_IMU_OK),
            "titan_ok": bool(status_bits & STATUS_TITAN_OK),
            "motor_enabled": bool(status_bits & STATUS_MOTOR_ENABLED),
            "cmd_timeout": bool(status_bits & STATUS_COMMAND_TIMEOUT),
            "cmd_seen": bool(status_bits & STATUS_COMMAND_SEEN),
            "checksum_error": bool(status_bits & STATUS_CHECKSUM_ERROR),
            "stopping": bool(status_bits & STATUS_STOPPING),
        }

    def _status_timer(self) -> None:
        with self._lock:
            age = self._now() - self._last_telemetry_time if self._last_telemetry_time > 0.0 else -1.0
            status_bits = self._last_status_bits
            status = self._decode_status(status_bits)
            encoder = list(self._last_encoder)
            rpm = list(self._last_rpm)
            last_seq = self._last_telemetry_seq
            last_cmd_seq = self._last_cmd_seq_seen_by_vmx
            packet_errors = self._packet_errors

        connected = age >= 0.0 and age <= self._telemetry_timeout
        msg = String()
        msg.data = (
            "mode=udp_direct "
            f"connected={str(connected).lower()} "
            f"telemetry_age={age:.3f} "
            f"seq={last_seq} last_cmd_seq={last_cmd_seq} "
            f"vmx_ok={str(status['vmx_ok']).lower()} "
            f"imu_ok={str(status['imu_ok']).lower()} "
            f"titan_ok={str(status['titan_ok']).lower()} "
            f"motor_enabled={str(status['motor_enabled']).lower()} "
            f"stopping={str(status['stopping']).lower()} "
            f"cmd_timeout={str(status['cmd_timeout']).lower()} "
            f"checksum_error={str(status['checksum_error']).lower()} "
            f"packet_errors={packet_errors} "
            f"encoder=[{','.join(str(value) for value in encoder)}] "
            f"rpm=[{','.join(f'{value:.1f}' for value in rpm)}]"
        )
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        for _ in range(3):
            packet = build_command_packet(
                (self._seq + 1) & 0xFFFFFFFF,
                [0.0, 0.0, 0.0, 0.0],
                enable=False,
            )
            try:
                self._command_socket.sendto(packet, (self._vmx_host, self._command_port))
            except OSError:
                pass
            time.sleep(0.02)
        self._stop_event.set()
        try:
            self._telemetry_socket.close()
        except OSError:
            pass
        try:
            self._command_socket.close()
        except OSError:
            pass
        self._receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VmxUdpBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
