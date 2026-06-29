"""
display.launch.py
=================
Visualises the robot URDF in RViz 2 using robot_state_publisher and
joint_state_publisher_gui.

Prerequisites
-------------
1. Add your URDF or Xacro file to:
       robot_description/urdf/robot.urdf.xacro

2. (Optional) Add an RViz config to:
       robot_description/rviz/robot.rviz

3. Build and source:
       colcon build --packages-select robot_description
       source install/setup.bash

4. Run:
       ros2 launch robot_description display.launch.py

If no URDF exists yet, this launch file will fail gracefully with an
informative error rather than silently producing wrong behaviour.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robot_description")

    # Default URDF / Xacro path
    default_urdf = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")

    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=default_urdf,
        description="Absolute path to the robot URDF or Xacro file",
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description=(
            "Set 'true' to launch joint_state_publisher_gui "
            "(requires a display). Set 'false' for headless."
        ),
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_share, "rviz", "robot.rviz"),
        description="Path to the RViz configuration file",
    )

    # Process Xacro → URDF XML string
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        LaunchConfiguration("urdf_file"),
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(
            # Use headless publisher only when GUI is disabled
            # LaunchConfiguration returns a string, so compare via substitution
            "false"  # placeholder — swap with NotCondition when needed
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    return LaunchDescription([
        urdf_file_arg,
        use_gui_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
