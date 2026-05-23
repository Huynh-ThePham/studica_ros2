from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration("joy_dev")
    params_file = LaunchConfiguration("params_file")
    udp_params_file = LaunchConfiguration("udp_params_file")
    joy_params_file = LaunchConfiguration("joy_params_file")
    mux_params_file = LaunchConfiguration("mux_params_file")

    pkg_share = FindPackageShare("vmx_highlevel")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Linux joystick device.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "vmx_highlevel.yaml"]
                ),
                description="YAML parameters for the PC-side VMX high-level bridge.",
            ),
            DeclareLaunchArgument(
                "udp_params_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "vmx_udp_bridge.yaml"]
                ),
                description="YAML parameters for the PC-side VMX UDP direct bridge.",
            ),
            DeclareLaunchArgument(
                "joy_params_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "vmx_teleop_joy.yaml"]
                ),
                description="YAML parameters for joy_node and teleop_twist_joy.",
            ),
            DeclareLaunchArgument(
                "mux_params_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "vmx_cmd_vel_mux.yaml"]
                ),
                description="YAML parameters for the VMX cmd_vel mux.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "vmx_highlevel.launch.py"])
                ),
                launch_arguments={
                    "params_file": params_file,
                    "udp_params_file": udp_params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "vmx_gamepad_teleop.launch.py"])
                ),
                launch_arguments={
                    "joy_dev": joy_dev,
                    "joy_params_file": joy_params_file,
                    "mux_params_file": mux_params_file,
                }.items(),
            ),
        ]
    )
