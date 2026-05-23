from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    udp_params_file = LaunchConfiguration("udp_params_file")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")
    imu_tf_x = LaunchConfiguration("imu_tf_x")
    imu_tf_y = LaunchConfiguration("imu_tf_y")
    imu_tf_z = LaunchConfiguration("imu_tf_z")
    imu_tf_qx = LaunchConfiguration("imu_tf_qx")
    imu_tf_qy = LaunchConfiguration("imu_tf_qy")
    imu_tf_qz = LaunchConfiguration("imu_tf_qz")
    imu_tf_qw = LaunchConfiguration("imu_tf_qw")
    base_frame_id = LaunchConfiguration("base_frame_id")
    imu_frame_id = LaunchConfiguration("imu_frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vmx_highlevel"), "config", "vmx_highlevel.yaml"]
                ),
                description="YAML parameters for the PC-side VMX high-level bridge.",
            ),
            DeclareLaunchArgument(
                "udp_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vmx_highlevel"), "config", "vmx_udp_bridge.yaml"]
                ),
                description="YAML parameters for the PC-side VMX UDP direct bridge.",
            ),
            Node(
                package="vmx_highlevel",
                executable="vmx_udp_bridge_node",
                name="vmx_udp_bridge_node",
                output="screen",
                parameters=[udp_params_file],
            ),
            DeclareLaunchArgument(
                "publish_imu_tf",
                default_value="true",
                description="Publish static TF base_frame -> imu_frame (identity by default).",
            ),
            DeclareLaunchArgument(
                "imu_tf_x",
                default_value="0.0",
                description="Static translation X imu in base_link (m).",
            ),
            DeclareLaunchArgument(
                "imu_tf_y",
                default_value="0.0",
                description="Static translation Y imu in base_link (m).",
            ),
            DeclareLaunchArgument(
                "imu_tf_z",
                default_value="0.0",
                description="Static translation Z imu in base_link (m).",
            ),
            DeclareLaunchArgument(
                "imu_tf_qx",
                default_value="0.0",
                description="Static rotation quaternion x (imu in base_link).",
            ),
            DeclareLaunchArgument(
                "imu_tf_qy",
                default_value="0.0",
                description="Static rotation quaternion y.",
            ),
            DeclareLaunchArgument(
                "imu_tf_qz",
                default_value="0.0",
                description="Static rotation quaternion z.",
            ),
            DeclareLaunchArgument(
                "imu_tf_qw",
                default_value="1.0",
                description="Static rotation quaternion w.",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_link",
                description="Must match vmx_highlevel.yaml base_frame_id.",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="imu_link",
                description="Must match VMX imu_frame_id / vmx_highlevel.yaml imu_frame_id.",
            ),
            Node(
                package="vmx_highlevel",
                executable="vmx_highlevel_node",
                name="vmx_highlevel_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                condition=IfCondition(publish_imu_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_imu_static",
                arguments=[
                    imu_tf_x,
                    imu_tf_y,
                    imu_tf_z,
                    imu_tf_qx,
                    imu_tf_qy,
                    imu_tf_qz,
                    imu_tf_qw,
                    base_frame_id,
                    imu_frame_id,
                ],
            ),
        ]
    )
