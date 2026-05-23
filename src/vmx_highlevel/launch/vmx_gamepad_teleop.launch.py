from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_params_file = LaunchConfiguration("joy_params_file")
    mux_params_file = LaunchConfiguration("mux_params_file")
    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vmx_highlevel"), "config", "vmx_teleop_joy.yaml"]
                ),
                description="YAML parameters for joy_node and teleop_twist_joy.",
            ),
            DeclareLaunchArgument(
                "mux_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vmx_highlevel"), "config", "vmx_cmd_vel_mux.yaml"]
                ),
                description="YAML parameters for the VMX cmd_vel mux.",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Linux joystick device.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[joy_params_file, {"dev": joy_dev}],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[joy_params_file],
                remappings=[
                    ("cmd_vel", "/cmd_vel_joy"),
                ],
            ),
            Node(
                package="vmx_highlevel",
                executable="vmx_cmd_vel_mux_node",
                name="vmx_cmd_vel_mux_node",
                output="screen",
                parameters=[mux_params_file],
            ),
        ]
    )
