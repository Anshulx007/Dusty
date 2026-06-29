"""
recorder.launch.py
==================
Launches recorder_node in isolation or alongside the rest of the stack.

recorder_node subscribes to /drive_cmd and /arduino_status, and responds
to /record_cmd (std_msgs/Bool: True=start, False=stop).

Trigger recording from the command line:

    # Start
    ros2 topic pub --once /record_cmd std_msgs/msg/Bool '{data: true}'

    # Stop
    ros2 topic pub --once /record_cmd std_msgs/msg/Bool '{data: false}'

    # Monitor state
    ros2 topic echo /recording_state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("robot_recorder")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "recorder_params.yaml"]
        ),
        description="Path to recorder parameters YAML",
    )

    recorder_node = Node(
        package="robot_recorder",
        executable="recorder_node",
        name="recorder_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        recorder_node,
    ])
