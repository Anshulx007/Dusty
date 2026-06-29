"""
haptics.launch.py
=================
Launches haptics_node in isolation (useful for testing haptic effects).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_haptics")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "haptics_params.yaml"]),
        description="Path to haptics parameters YAML",
    )

    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Run without hardware (logs effects only)",
    )

    haptics_node = Node(
        package="robot_haptics",
        executable="haptics_node",
        name="haptics_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        dry_run_arg,
        haptics_node,
    ])
