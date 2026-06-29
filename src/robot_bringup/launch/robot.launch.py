"""
robot.launch.py
===============
Full robot bringup — launches all nodes:
    joy_node → teleop_node → arduino_driver_node
                           → haptics_node

Designed to be the single launch file run on the Pi at startup.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ---- Argument declarations ----

    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description=(
            "Set true to run all nodes without hardware "
            "(arduino_driver sends to console, haptics logs effects)"
        ),
    )

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path for joy_node",
    )

    namespace_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Optional ROS 2 namespace for all nodes",
    )

    # ---- Param file paths ----
    driver_params = PathJoinSubstitution([
        FindPackageShare("robot_driver"), "config", "driver_params.yaml"
    ])
    teleop_params = PathJoinSubstitution([
        FindPackageShare("robot_teleop"), "config", "teleop_params.yaml"
    ])
    haptics_params = PathJoinSubstitution([
        FindPackageShare("robot_haptics"), "config", "haptics_params.yaml"
    ])

    # ---- Nodes ----

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id":            0,
            "deadzone":             0.05,
            "autorepeat_rate":      20.0,
            "coalesce_interval_ms": 1,
        }],
    )

    teleop_node = Node(
        package="robot_teleop",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[teleop_params],
    )

    driver_node = Node(
        package="robot_driver",
        executable="arduino_driver_node",
        name="arduino_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            driver_params,
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    haptics_node = Node(
        package="robot_haptics",
        executable="haptics_node",
        name="haptics_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            haptics_params,
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    return LaunchDescription([
        dry_run_arg,
        joy_dev_arg,
        namespace_arg,
        joy_node,
        teleop_node,
        driver_node,
        haptics_node,
    ])
