#!/usr/bin/env python3
"""
navigation_node (stub)
======================
Placeholder for future autonomous navigation.

Architecture contract
---------------------
This node publishes to /drive_cmd with source=SOURCE_NAV, exactly like
teleop_node does with source=SOURCE_TELEOP.

arduino_driver_node has no idea whether a DriveCmd came from teleop or
navigation — it just executes the command. Command arbitration (which
source wins at a given moment) will be the responsibility of robot_safety
when it is implemented.

Current behaviour: immediately stops. Replace the _navigation_tick method
with your VFH+, frontier exploration, or path following logic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robot_msgs.msg import DriveCmd, ArduinoStatus


class NavigationNode(Node):

    def __init__(self) -> None:
        super().__init__("navigation_node")

        self.declare_parameter("publish_rate_hz", 10.0)
        self._active = False
        self._arduino_connected = False

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._drive_pub = self.create_publisher(DriveCmd, "/drive_cmd", reliable)
        self._status_sub = self.create_subscription(
            ArduinoStatus, "/arduino_status", self._status_cb, best_effort
        )

        rate = self.get_parameter("publish_rate_hz").value
        self._timer = self.create_timer(1.0 / rate, self._navigation_tick)

        self.get_logger().info(
            "navigation_node started (STUB — publishes stop commands only)"
        )

    def _status_cb(self, msg: ArduinoStatus) -> None:
        self._arduino_connected = msg.connected

    def _navigation_tick(self) -> None:
        """
        Replace with real navigation logic.

        Publish DriveCmd with source=SOURCE_NAV.
        robot_safety (future) will arbitrate between this and teleop.
        """
        msg = DriveCmd()
        msg.linear  = 0.0
        msg.angular = 0.0
        msg.stop    = True
        msg.enable  = False      # disabled until navigation is implemented
        msg.source  = DriveCmd.SOURCE_NAV
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
