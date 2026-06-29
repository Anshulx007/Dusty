#!/usr/bin/env python3
"""
arduino_driver_node
===================
Sole owner of the Arduino serial connection.

Responsibilities
----------------
* Open / auto-detect the Arduino USB serial port.
* Auto-reconnect on disconnect with exponential back-off.
* Subscribe to /drive_cmd (robot_msgs/DriveCmd) and convert to the
  existing text protocol: F<n>, B<n>, L<n>, R<n>, S0, MON, MOFF, BR<pwm>.
* Publish /arduino_status (robot_msgs/ArduinoStatus) at a fixed rate.

What this node does NOT do
--------------------------
* It does not read the joystick.
* It does not implement safety logic.
* It does not know about haptics or navigation.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    import serial
    import serial.serialutil
    from serial.tools import list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from robot_msgs.msg import DriveCmd, ArduinoStatus


# ---------------------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------------------

def drive_cmd_to_serial(msg: DriveCmd) -> str:
    """Convert a DriveCmd message to the Arduino text protocol string.

    Protocol (preserved exactly from the original monolith):
      F<n>   forward  at n % speed   (1–100)
      B<n>   backward at n % speed
      L<n>   left     at n % speed
      R<n>   right    at n % speed
      S0     stop

    linear   > 0  → forward  direction  (turn via angular)
    linear   < 0  → backward direction
    angular  > 0  → left  (while moving)
    angular  < 0  → right (while moving)
    stop=True      → S0 regardless of linear/angular
    """
    if msg.stop or not msg.enable:
        return "S0"

    speed = abs(int(round(msg.linear * 100)))
    speed = max(0, min(100, speed))

    if speed == 0:
        return "S0"

    angular_threshold = 0.3  # normalized — tune via parameter if needed

    if msg.linear > 0:
        if msg.angular > angular_threshold:
            return f"L{speed}"
        elif msg.angular < -angular_threshold:
            return f"R{speed}"
        else:
            return f"F{speed}"
    else:  # linear < 0 → backward; turns are mirrored
        if msg.angular > angular_threshold:
            return f"L{speed}"
        elif msg.angular < -angular_threshold:
            return f"R{speed}"
        else:
            return f"B{speed}"


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ArduinoDriverNode(Node):
    """ROS 2 node that owns the Arduino serial connection."""

    # Default parameter values — override via config/driver_params.yaml
    _DEFAULTS = {
        "port":             "/dev/ttyACM0",
        "baud":             9600,
        "connect_delay":    2.0,
        "reconnect_interval": 5.0,
        "ready_string":     "READY",
        "status_rate_hz":   5.0,
        "cmd_timeout_sec":  0.5,   # send S0 if no /drive_cmd received within this
        "scan_keywords": [
            "arduino", "ch340", "wch", "usb-serial",
            "usb serial", "ftdi", "cp210",
        ],
        "dry_run": False,          # set True to test without hardware
    }

    def __init__(self) -> None:
        super().__init__("arduino_driver_node")
        self._declare_params()

        # ---- serial state ----
        self._port_obj: Optional[serial.Serial] = None
        self._port_lock = threading.Lock()
        self._connected = False
        self._last_cmd = ""
        self._state = "INIT"

        # ---- cmd watchdog ----
        self._last_cmd_time = time.monotonic()
        self._cmd_timeout = self.get_parameter("cmd_timeout_sec").value

        # ---- QoS ----
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ---- subscriptions ----
        self._drive_sub = self.create_subscription(
            DriveCmd,
            "/drive_cmd",
            self._drive_cmd_cb,
            reliable_qos,
        )

        # ---- publishers ----
        self._status_pub = self.create_publisher(
            ArduinoStatus,
            "/arduino_status",
            best_effort_qos,
        )

        # ---- timers ----
        status_period = 1.0 / self.get_parameter("status_rate_hz").value
        self._status_timer = self.create_timer(status_period, self._publish_status)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_tick)

        # ---- connect ----
        if not self._get_param("dry_run"):
            if not HAS_SERIAL:
                self.get_logger().error("pyserial not installed — cannot connect")
            else:
                connected = self._try_connect()
                if not connected:
                    self._state = "DISCONNECTED"
                    self._schedule_reconnect()
        else:
            self.get_logger().warn("DRY RUN mode — no serial hardware used")
            self._connected = True
            self._state = "DRY_RUN"

        self.get_logger().info("arduino_driver_node ready")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        for name, default in self._DEFAULTS.items():
            self.declare_parameter(name, default)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------

    def _candidate_ports(self) -> list[str]:
        """Return prioritized list of serial device paths to try."""
        preferred = self._get_param("port")
        keywords = [k.lower() for k in self._get_param("scan_keywords")]

        arduino_like: list[str] = []
        other: list[str] = []

        for p in list_ports.comports():
            text = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
            if any(k in text for k in keywords):
                arduino_like.append(p.device)
            else:
                other.append(p.device)

        ordered: list[str] = []
        if preferred:
            ordered.append(preferred)
        for dev in arduino_like + other:
            if dev not in ordered:
                ordered.append(dev)
        return ordered

    def _try_connect(self) -> bool:
        """Scan ports and attempt to open the Arduino. Returns True on success."""
        baud = self._get_param("baud")
        delay = self._get_param("connect_delay")
        ready_str = self._get_param("ready_string").upper()
        preferred = self._get_param("port")

        for device in self._candidate_ports():
            self.get_logger().info(f"Trying {device} …")
            try:
                port = serial.Serial(device, baud, timeout=1)
            except serial.serialutil.SerialException as exc:
                self.get_logger().debug(f"  Could not open {device}: {exc}")
                continue

            time.sleep(delay)  # let the board finish its reset/boot

            confirmed = (device == preferred)
            try:
                line = port.readline().decode(errors="ignore").strip()
                self.get_logger().debug(f"  Board said: {line!r}")
                if ready_str in line.upper():
                    confirmed = True
            except Exception:
                pass

            if not confirmed:
                port.close()
                continue

            port.reset_input_buffer()
            port.reset_output_buffer()

            with self._port_lock:
                self._port_obj = port
                self._connected = True
                self._state = "IDLE"

            self.get_logger().info(f"Arduino connected on {device}")
            return True

        self.get_logger().error("Arduino NOT found on any USB serial port")
        return False

    def _disconnect(self) -> None:
        """Close the serial port and update flags."""
        with self._port_lock:
            if self._port_obj:
                try:
                    self._port_obj.close()
                except Exception:
                    pass
                self._port_obj = None
            self._connected = False
            self._state = "DISCONNECTED"
            self._last_cmd = ""

    def _schedule_reconnect(self) -> None:
        """Schedule a one-shot reconnect attempt after reconnect_interval seconds."""
        interval = self._get_param("reconnect_interval")
        self.create_timer(interval, self._reconnect_once)
        self.get_logger().info(f"Reconnect scheduled in {interval}s")

    def _reconnect_once(self) -> None:
        """Called by the one-shot timer. Retries connection; reschedules if needed."""
        if self._connected:
            return
        self.get_logger().info("Attempting reconnect …")
        if self._try_connect():
            self.get_logger().info("Reconnect succeeded")
        else:
            self._schedule_reconnect()

    # ------------------------------------------------------------------
    # Serial I/O
    # ------------------------------------------------------------------

    def _send(self, cmd: str) -> bool:
        """Write a command string to the serial port. Returns True on success."""
        dry_run = self._get_param("dry_run")

        if dry_run:
            self.get_logger().debug(f"[DRY] → {cmd}")
            self._last_cmd = cmd
            return True

        with self._port_lock:
            if not self._connected or self._port_obj is None:
                return False
            try:
                self._port_obj.write((cmd + "\n").encode())
                self._last_cmd = cmd
                return True
            except serial.serialutil.SerialException:
                self._connected = False
                self._state = "DISCONNECTED"
                self.get_logger().error(f"Arduino disconnected while sending '{cmd}'")

        # Outside lock — trigger reconnect
        self._schedule_reconnect()
        return False

    # ------------------------------------------------------------------
    # Drive command callback
    # ------------------------------------------------------------------

    def _drive_cmd_cb(self, msg: DriveCmd) -> None:
        self._last_cmd_time = time.monotonic()

        serial_str = drive_cmd_to_serial(msg)

        # Avoid spamming identical commands
        if serial_str == self._last_cmd:
            return

        ok = self._send(serial_str)
        if ok:
            self._state = "DRIVING" if serial_str != "S0" else "IDLE"
            self.get_logger().info(
                f"→ Arduino: {serial_str!r}  "
                f"(src={msg.source}, lin={msg.linear:.2f}, ang={msg.angular:.2f})"
            )

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_tick(self) -> None:
        """If no DriveCmd arrives within cmd_timeout_sec, send S0."""
        if not self._connected:
            return
        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed > self._cmd_timeout and self._last_cmd != "S0":
            self.get_logger().warn(
                f"DriveCmd timeout ({elapsed:.2f}s) — sending S0"
            )
            self._send("S0")
            self._state = "IDLE"

    # ------------------------------------------------------------------
    # Status publisher
    # ------------------------------------------------------------------

    def _publish_status(self) -> None:
        msg = ArduinoStatus()
        msg.connected = self._connected
        msg.battery = 0.0       # extend: parse "BATT:<v>" lines from board
        msg.estop = False       # extend: parse "ESTOP" from board
        msg.state = self._state
        msg.last_cmd = self._last_cmd
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down — sending S0 and closing port")
        self._send("S0")
        self._disconnect()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
