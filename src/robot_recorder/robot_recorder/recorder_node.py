#!/usr/bin/env python3
"""
recorder_node (stub)
====================
Dedicated ROS 2 bag-style recorder for robot-specific topics.

This node is a stub — it compiles, starts, and logs its subscriptions
but performs no recording. Implement the methods marked TODO to add
recording, serialisation, and playback triggering.

Rationale for a dedicated recorder
-----------------------------------
`ros2 bag record` captures raw ROS 2 messages. This node is intended for
a higher-level, robot-aware recording format that:

* Stores DriveCmd sequences with relative timestamps (as the original
  monolith did in path_buffer).
* Can be triggered by a teleop button press (LB in robot_teleop) via a
  dedicated /record_cmd topic, without coupling that logic to teleop_node.
* Supports labelling, metadata, and selective replay filtering.
* Decouples recording from the teleop node so the two can be versioned
  independently.

Current behaviour
-----------------
* Subscribes to /drive_cmd and /arduino_status.
* Subscribes to /record_cmd (std_msgs/Bool: True=start, False=stop).
* Logs state transitions.
* Does NOT write any files or buffers.

Future responsibilities
-----------------------
* Maintain a path_buffer of (relative_timestamp, DriveCmd) tuples.
* Serialize recordings to disk (JSON / SQLite / ROS 2 bag).
* Publish /playback_cmd topic that navigation_node can subscribe to for
  replay (or implement playback here directly).
* Support recording metadata: robot name, session ID, timestamp, notes.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from robot_msgs.msg import DriveCmd, ArduinoStatus


class RecorderNode(Node):
    """Dedicated topic recorder for robot drive sessions (stub)."""

    _DEFAULTS = {
        # Maximum recording duration before auto-stop (0 = unlimited)
        "max_duration_sec": 0.0,
        # Speed limit applied during recording (normalised [0, 1], 0 = no limit)
        "record_speed_limit": 0.0,
        # Directory for saved recordings (empty = do not save to disk)
        "output_dir": "",
        # Prefix for saved file names
        "output_prefix": "robot_session",
    }

    def __init__(self) -> None:
        super().__init__("recorder_node")

        for name, default in self._DEFAULTS.items():
            self.declare_parameter(name, default)

        # ---- internal state ----
        self._recording = False
        self._session_start: float = 0.0
        self._path_buffer: list[tuple[float, DriveCmd]] = []
        self._last_cmd_time: float = 0.0

        # ---- QoS ----
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # ---- subscriptions ----
        self._drive_sub = self.create_subscription(
            DriveCmd,
            "/drive_cmd",
            self._drive_cmd_cb,
            reliable,
        )
        self._status_sub = self.create_subscription(
            ArduinoStatus,
            "/arduino_status",
            self._arduino_status_cb,
            best_effort,
        )
        self._record_cmd_sub = self.create_subscription(
            Bool,
            "/record_cmd",
            self._record_cmd_cb,
            reliable,
        )

        # ---- publishers ----
        # Future: publish recording state so other nodes can react
        self._recording_state_pub = self.create_publisher(
            Bool,
            "/recording_state",
            reliable,
        )

        self.get_logger().info(
            "recorder_node ready (STUB — subscribing but not recording)"
        )
        self.get_logger().info(
            "  Publish True  to /record_cmd to start recording"
        )
        self.get_logger().info(
            "  Publish False to /record_cmd to stop  recording"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _record_cmd_cb(self, msg: Bool) -> None:
        """Start or stop a recording session on demand."""
        if msg.data and not self._recording:
            self._start_recording()
        elif not msg.data and self._recording:
            self._stop_recording()

    def _drive_cmd_cb(self, msg: DriveCmd) -> None:
        """Buffer incoming drive commands during an active recording."""
        if not self._recording:
            return

        now = time.monotonic()
        # TODO: append (relative_timestamp, msg) to self._path_buffer
        # Example:
        #   elapsed = now - self._session_start
        #   self._path_buffer.append((elapsed, msg))
        self._last_cmd_time = now

    def _arduino_status_cb(self, msg: ArduinoStatus) -> None:
        """Monitor Arduino health; abort recording on disconnect."""
        if self._recording and not msg.connected:
            self.get_logger().warn(
                "Arduino disconnected during recording — stopping session"
            )
            self._stop_recording()

    # ------------------------------------------------------------------
    # Recording lifecycle
    # ------------------------------------------------------------------

    def _start_recording(self) -> None:
        """Begin a new recording session."""
        self._recording = True
        self._session_start = time.monotonic()
        self._last_cmd_time = self._session_start
        self._path_buffer = []

        self.get_logger().info("Recording STARTED (stub — no data will be saved)")
        self._publish_state(True)

        # TODO: enforce max_duration_sec via a one-shot timer
        # TODO: apply record_speed_limit by monitoring incoming commands

    def _stop_recording(self) -> None:
        """End the current recording session."""
        if not self._recording:
            return

        duration = time.monotonic() - self._session_start
        count = len(self._path_buffer)

        self._recording = False
        self._publish_state(False)

        self.get_logger().info(
            f"Recording STOPPED — duration={duration:.1f}s, "
            f"commands buffered={count} (stub — nothing saved)"
        )

        # TODO: serialise self._path_buffer to output_dir / output_prefix

    def _publish_state(self, recording: bool) -> None:
        msg = Bool()
        msg.data = recording
        self._recording_state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._recording:
            self.get_logger().info("Shutdown during recording — stopping session")
            self._stop_recording()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
