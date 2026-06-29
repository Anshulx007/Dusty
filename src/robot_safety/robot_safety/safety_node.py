#!/usr/bin/env python3
"""
safety_node (stub)
==================
Placeholder for the robot safety and command arbitration node.

When implemented, this node will sit between teleop/navigation and the
Arduino driver, acting as the sole publisher to /drive_cmd. Teleop and
navigation will publish to /drive_cmd_raw, and safety_node will decide
which command (if any) reaches the Arduino.

Current behaviour
-----------------
* Watchdog: publishes an E-stop DriveCmd if neither teleop nor navigation
  has published within their respective timeout windows.
* Pass-through: in the stub, teleop commands pass straight through.
* No arbitration logic yet.

Future responsibilities
-----------------------
* MODE_MANUAL / MODE_AUTONOMOUS / MODE_PAUSED / MODE_EMERGENCY_STOP
* Heartbeat watchdog for both teleop and navigation sources
* Hardware E-stop GPIO pin monitoring
* Command arbitration (which source wins)
* Rate-limiting / velocity limits
* Collision detection integration
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robot_msgs.msg import DriveCmd, ArduinoStatus, RobotMode


class SafetyNode(Node):

    _DEFAULTS = {
        "teleop_timeout_sec": 0.5,
        "nav_timeout_sec":    1.0,
        "watchdog_rate_hz":   20.0,
    }

    def __init__(self) -> None:
        super().__init__("safety_node")
        for name, val in self._DEFAULTS.items():
            self.declare_parameter(name, val)

        self._last_teleop_time = 0.0
        self._last_nav_time    = 0.0
        self._last_teleop_cmd: DriveCmd | None = None
        self._last_nav_cmd:    DriveCmd | None = None
        self._mode = RobotMode.MODE_MANUAL

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Future: teleop and nav publish to /drive_cmd_raw_{source}
        # and safety arbitrates. For now, subscribe to /drive_cmd and
        # re-publish (pass-through stub).
        self._drive_sub = self.create_subscription(
            DriveCmd, "/drive_cmd_raw", self._drive_raw_cb, reliable
        )
        self._drive_pub = self.create_publisher(DriveCmd, "/drive_cmd", reliable)
        self._mode_pub  = self.create_publisher(RobotMode, "/robot_mode", reliable)

        rate = self.get_parameter("watchdog_rate_hz").value
        self._watchdog_timer = self.create_timer(1.0 / rate, self._watchdog_tick)

        self.get_logger().info(
            "safety_node started (STUB — pass-through mode, no arbitration)"
        )

    def _drive_raw_cb(self, msg: DriveCmd) -> None:
        now = time.monotonic()
        if msg.source == DriveCmd.SOURCE_TELEOP:
            self._last_teleop_time = now
            self._last_teleop_cmd  = msg
        elif msg.source == DriveCmd.SOURCE_NAV:
            self._last_nav_time = now
            self._last_nav_cmd  = msg

        # Stub: pass through the command unmodified
        self._drive_pub.publish(msg)

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        teleop_timeout = self.get_parameter("teleop_timeout_sec").value

        if (self._last_teleop_cmd is not None
                and now - self._last_teleop_time > teleop_timeout):
            self.get_logger().warn("Teleop watchdog — publishing E-stop")
            stop = DriveCmd()
            stop.stop   = True
            stop.enable = True
            stop.source = DriveCmd.SOURCE_EMERGENCY
            self._drive_pub.publish(stop)
            self._last_teleop_cmd = None  # don't repeat every tick

    def destroy_node(self) -> None:
        stop = DriveCmd()
        stop.stop   = True
        stop.enable = True
        stop.source = DriveCmd.SOURCE_EMERGENCY
        self._drive_pub.publish(stop)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
