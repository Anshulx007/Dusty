"""
teleop.launch.py
================
Launches joy_node (reads /dev/input/js0) + teleop_node.
Suitable for manual driving without the full bringup stack.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_teleop")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "teleop_params.yaml"]),
        description="Path to teleop parameters YAML",
    )

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id":   0,
            "device_name": "",
            "deadzone":    0.05,
            "autorepeat_rate": 20.0,
            "coalesce_interval_ms": 1,
        }],
    )

    teleop_node = Node(
        package="robot_teleop",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
