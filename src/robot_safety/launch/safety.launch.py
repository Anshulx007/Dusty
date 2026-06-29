"""
safety.launch.py
================
Launches safety_node in isolation.

When safety_node is active it sits between teleop/navigation and the
Arduino driver. The expected full-system topic flow is:

    teleop_node      ──► /drive_cmd_raw ──► safety_node ──► /drive_cmd ──► arduino_driver_node
    navigation_node  ──► /drive_cmd_raw ──┘

Until that wiring is complete (requires remapping teleop and navigation
to publish on /drive_cmd_raw), safety_node runs as a pass-through.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_safety")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "safety_params.yaml"]
        ),
        description="Path to safety parameters YAML",
    )

    safety_node = Node(
        package="robot_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        safety_node,
    ])
