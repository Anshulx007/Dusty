"""
driver.launch.py
================
Launches only the arduino_driver_node.
Useful for testing serial communication in isolation.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_driver")

    # Allow overriding the param file path at launch time
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "driver_params.yaml"]),
        description="Path to the driver parameters YAML file",
    )

    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Set 'true' to run without hardware (for development/testing)",
    )

    driver_node = Node(
        package="robot_driver",
        executable="arduino_driver_node",
        name="arduino_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
        remappings=[
            # Future: remap /drive_cmd to /safety/drive_cmd when robot_safety is added
        ],
    )

    return LaunchDescription([
        params_file_arg,
        dry_run_arg,
        driver_node,
    ])
