"""
navigation.launch.py
====================
Launches navigation_node in isolation.

The node currently publishes stop commands (stub). Replace the
_navigation_tick method in navigation_node.py to add real logic.

Requires arduino_driver_node to be running separately, or use
the full bringup launch instead:

    ros2 launch robot_bringup robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_navigation")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "navigation_params.yaml"]
        ),
        description="Path to navigation parameters YAML",
    )

    navigation_node = Node(
        package="robot_navigation",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            # When robot_safety is active, remap to the raw input topic:
            # ("/drive_cmd", "/drive_cmd_raw"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        navigation_node,
    ])
