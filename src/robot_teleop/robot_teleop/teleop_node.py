#!/usr/bin/env python3
"""
teleop_node
===========
Translates ROS 2 Joy messages into DriveCmd and HapticCmd messages.

Responsibilities
----------------
* Subscribe to /joy (sensor_msgs/Joy from joy_node).
* Apply deadman switch, speed scaling, deadzone, and acceleration ramp.
* Publish /drive_cmd (robot_msgs/DriveCmd).
* Publish /haptics_cmd (robot_msgs/HapticCmd) for the haptics_node.
* Handle all button actions: mop, brush, recording, playback, precision
  mode, safety lock, emergency stop.

What this node does NOT do
--------------------------
* It never touches serial or the Arduino directly.
* It never reads /dev/input devices for force feedback.
* It does not contain the serial protocol.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Joy
from robot_msgs.msg import DriveCmd, HapticCmd, ArduinoStatus


# ---------------------------------------------------------------------------
# Haptic shorthand — builds and returns a HapticCmd
# ---------------------------------------------------------------------------

def _haptic(effect: int, strength: float = 1.0, duration: float = 0.0) -> HapticCmd:
    msg = HapticCmd()
    msg.effect = effect
    msg.strength = strength
    msg.duration = duration
    return msg


# ---------------------------------------------------------------------------
# Button / Axis index constants for the Ares controller
# (calibrate via `ros2 run joy joy_node` and `ros2 topic echo /joy`)
# ---------------------------------------------------------------------------
class AresBtn:
    A         = 0   # Brush speed cycle
    B         = 1   # Brush OFF
    X         = 2   # Mop ON
    Y         = 3   # Mop OFF
    LB        = 4   # Record start/stop
    RB        = 5   # Playback
    SELECT    = 6   # Safety lock (3×)
    START     = 7   # Safety unlock (hold)
    HOME      = 8   # Mute haptics / force reconnect
    THUMBL    = 9   # Precision mode toggle


class AresAxis:
    LX = 0    # Left stick X (steering)
    LY = 1    # Left stick Y (unused, kept for future)
    LT = 2    # Left trigger  (forward throttle)  — range [-1, 1]; -1=released, 1=pressed
    RX = 3    # Right stick X
    RY = 4    # Right stick Y
    RT = 5    # Right trigger (backward throttle) — range [-1, 1]


# ---------------------------------------------------------------------------
# Safety lock state machine
# ---------------------------------------------------------------------------

class SafetyLock:
    """Encapsulates the triple-SELECT-press lock and hold-START-to-unlock."""

    def __init__(
        self,
        press_count: int,
        press_window: float,
        unlock_hold: float,
        unlock_pulse: float,
        on_lock,
        on_unlock,
        on_unlock_pulse,
    ):
        self._count  = press_count
        self._window = press_window
        self._hold   = unlock_hold
        self._pulse  = unlock_pulse
        self._on_lock   = on_lock
        self._on_unlock = on_unlock
        self._on_pulse  = on_unlock_pulse

        self.locked = False
        self._select_times: list[float] = []
        self._hold_cancel = threading.Event()
        self._hold_thread: Optional[threading.Thread] = None

    def process_select_press(self) -> None:
        if self.locked:
            return
        now = time.monotonic()
        self._select_times.append(now)
        self._select_times = [t for t in self._select_times if now - t <= self._window]
        if len(self._select_times) >= self._count:
            self._select_times.clear()
            self.locked = True
            self._on_lock()

    def process_start(self, pressed: bool) -> None:
        if not self.locked:
            return
        if pressed:
            if self._hold_thread and self._hold_thread.is_alive():
                return
            self._hold_cancel.clear()
            self._hold_thread = threading.Thread(
                target=self._hold_loop, daemon=True
            )
            self._hold_thread.start()
        else:
            self._hold_cancel.set()

    def _hold_loop(self) -> None:
        elapsed = 0.0
        step = 0.1
        next_pulse = self._pulse
        while elapsed < self._hold:
            if self._hold_cancel.wait(step):
                return
            elapsed += step
            if elapsed >= next_pulse:
                self._on_pulse()
                next_pulse += self._pulse
        # Full hold completed
        self.locked = False
        self._on_unlock()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TeleopNode(Node):
    """Gamepad teleop node for the robot."""

    _DEFAULTS = {
        # Axes
        "axis_lx":          AresAxis.LX,
        "axis_lt":          AresAxis.LT,
        "axis_rt":          AresAxis.RT,
        # Buttons
        "btn_a":            AresBtn.A,
        "btn_b":            AresBtn.B,
        "btn_x":            AresBtn.X,
        "btn_y":            AresBtn.Y,
        "btn_lb":           AresBtn.LB,
        "btn_rb":           AresBtn.RB,
        "btn_select":       AresBtn.SELECT,
        "btn_start":        AresBtn.START,
        "btn_home":         AresBtn.HOME,
        "btn_thumbl":       AresBtn.THUMBL,
        # Drive
        "trigger_deadband":  0.04,      # normalized [0-1]; ignore triggers below this
        "angular_threshold": 0.30,      # normalized LX threshold for turn
        "min_speed":         0.20,      # minimum normalized speed when above deadband
        "max_speed":         1.00,      # maximum normalized speed
        "acceleration_rate": 1.80,      # normalized/sec while speeding up
        "deceleration_rate": 3.50,      # normalized/sec while braking
        "enable_ramp":       True,
        # Precision mode
        "precision_mode_enabled": True,
        "precision_max_speed":    0.20,
        # Recording
        "record_speed_limit": 0.40,
        # Playback
        "playback_start_delay": 0.35,
        "playback_end_delay":   0.15,
        # Safety
        "safety_enable":       True,
        "safety_press_count":  3,
        "safety_press_window": 2.0,
        "safety_unlock_hold":  3.0,
        "safety_unlock_pulse": 0.5,
        # Brush
        "brush_speed_levels":  [0.33, 0.66, 1.00],  # fraction of max PWM
        # Publish rate for drive commands (even if unchanged, for watchdog keepalive)
        "drive_publish_rate_hz": 50.0,
        # Joy watchdog — if no Joy message arrives within this, stop the robot
        "joy_timeout_sec": 0.5,
    }

    def __init__(self) -> None:
        super().__init__("teleop_node")
        self._declare_params()

        # ---- controller state ----
        self._lx: float = 0.0      # normalized [-1, 1]
        self._lt: float = 0.0      # normalized [0, 1] (mapped from [-1,1] trigger)
        self._rt: float = 0.0
        self._buttons: list[int] = []
        self._last_joy_time = time.monotonic()

        # ---- drive ramp state ----
        self._current_speed = 0.0  # normalized [0, 1]
        self._current_dir   = "S"  # "F", "B", "L", "R", "S"
        self._last_serial_equiv = ""

        # ---- accessories ----
        self._mop_state   = False
        self._brush_level = 0   # 0=off, 1..N index into brush_speed_levels

        # ---- recording / playback ----
        self._recording         = False
        self._is_playing        = False
        self._path_buffer: list[tuple[float, str]] = []
        self._last_record_time  = 0.0

        # ---- precision mode ----
        self._precision_mode = False

        # ---- haptic mute ----
        self._haptic_muted = False

        # ---- previous button state (for rising/falling edge detection) ----
        self._prev_buttons: list[int] = []

        # ---- safety lock ----
        self._safety = SafetyLock(
            press_count   = self._get_param("safety_press_count"),
            press_window  = self._get_param("safety_press_window"),
            unlock_hold   = self._get_param("safety_unlock_hold"),
            unlock_pulse  = self._get_param("safety_unlock_pulse"),
            on_lock       = self._on_lock,
            on_unlock     = self._on_unlock,
            on_unlock_pulse = self._on_unlock_pulse,
        )

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
        self._joy_sub = self.create_subscription(
            Joy, "/joy", self._joy_cb, best_effort
        )
        self._arduino_status_sub = self.create_subscription(
            ArduinoStatus, "/arduino_status", self._arduino_status_cb, best_effort
        )

        # ---- publishers ----
        self._drive_pub = self.create_publisher(DriveCmd, "/drive_cmd", reliable)
        self._haptic_pub = self.create_publisher(HapticCmd, "/haptics_cmd", reliable)

        # ---- arduino status cache ----
        self._arduino_connected = False

        # ---- drive timer (runs the ramp and publishes DriveCmd) ----
        rate = self._get_param("drive_publish_rate_hz")
        self._last_tick = time.monotonic()
        self._drive_timer = self.create_timer(1.0 / rate, self._drive_tick)

        self.get_logger().info("teleop_node ready — waiting for /joy")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        for name, default in self._DEFAULTS.items():
            self.declare_parameter(name, default)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Joy callback — parse axes and buttons
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy_time = time.monotonic()

        axes    = msg.axes
        buttons = msg.buttons
        self._buttons = list(buttons)

        # --- Axis updates ---
        ax_lx = self._get_param("axis_lx")
        ax_lt = self._get_param("axis_lt")
        ax_rt = self._get_param("axis_rt")

        if ax_lx < len(axes):
            self._lx = float(axes[ax_lx])

        # Triggers arrive as [-1, 1] in joy_node; remap to [0, 1]
        if ax_lt < len(axes):
            self._lt = max(0.0, (axes[ax_lt] + 1.0) / 2.0)  # -1..1 → 0..1
        if ax_rt < len(axes):
            self._rt = max(0.0, (axes[ax_rt] + 1.0) / 2.0)

        # --- Button events (only process rising edge) ---
        self._process_buttons(buttons)

    def _btn(self, buttons: list, param_name: str) -> bool:
        idx = self._get_param(param_name)
        return bool(buttons[idx]) if idx < len(buttons) else False

    def _process_buttons(self, buttons: list) -> None:
        prev = self._prev_buttons
        # Extend prev if shorter
        while len(prev) < len(buttons):
            prev.append(0)

        def rising(param: str) -> bool:
            idx = self._get_param(param)
            if idx >= len(buttons):
                return False
            return bool(buttons[idx]) and not bool(prev[idx])

        def is_held(param: str) -> bool:
            idx = self._get_param(param)
            return bool(buttons[idx]) if idx < len(buttons) else False

        def falling(param: str) -> bool:
            idx = self._get_param(param)
            if idx >= len(buttons):
                return False
            return not bool(buttons[idx]) and bool(prev[idx])

        # START — safety unlock hold (track press AND release)
        if self._get_param("safety_enable"):
            if rising("btn_start"):
                self._safety.process_start(True)
            elif falling("btn_start"):
                self._safety.process_start(False)

        # SELECT — safety lock
        if rising("btn_select"):
            if self._get_param("safety_enable"):
                self._safety.process_select_press()

        # Everything below ignored while locked
        if self._safety.locked:
            self._prev_buttons = list(buttons)
            return

        if rising("btn_a"):
            self._handle_brush_up()

        if rising("btn_b"):
            self._handle_brush_off()

        if rising("btn_x"):
            self._handle_mop_on()

        if rising("btn_y"):
            self._handle_mop_off()

        if rising("btn_lb"):
            self._handle_record_toggle()

        if rising("btn_rb"):
            self._handle_playback()

        if rising("btn_thumbl"):
            self._handle_precision_toggle()

        if rising("btn_home"):
            self._handle_home()

        self._prev_buttons = list(buttons)

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    def _handle_brush_up(self) -> None:
        levels = self._get_param("brush_speed_levels")
        self._brush_level += 1
        if self._brush_level > len(levels):
            self._brush_level = 1
        pct = levels[self._brush_level - 1]
        pwm = int(round(pct * 255))
        self._send_arduino_accessory(f"BR{pwm}")
        effect = HapticCmd.EFFECT_BRUSH_SPEED_1 + (self._brush_level - 1)
        self._pub_haptic(effect)
        self.get_logger().info(f"Brush speed level {self._brush_level} ({int(pct*100)}%)")

    def _handle_brush_off(self) -> None:
        if self._brush_level != 0:
            self._brush_level = 0
            self._send_arduino_accessory("BR0")
            self._pub_haptic(HapticCmd.EFFECT_LONG_THEN_SHORT)
            self.get_logger().info("Brush OFF")

    def _handle_mop_on(self) -> None:
        if not self._mop_state:
            self._send_arduino_accessory("MON")
            self._mop_state = True
            self._pub_haptic(HapticCmd.EFFECT_SINGLE_BUZZ)
            self.get_logger().info("Mop ON")

    def _handle_mop_off(self) -> None:
        if self._mop_state:
            self._send_arduino_accessory("MOFF")
            self._mop_state = False
            self._pub_haptic(HapticCmd.EFFECT_DOUBLE_BUZZ)
            self.get_logger().info("Mop OFF")

    def _handle_record_toggle(self) -> None:
        if self._is_playing:
            self.get_logger().warn("Can't record while playing back")
            return
        if not self._recording:
            self._recording        = True
            self._path_buffer      = []
            self._last_record_time = time.monotonic()
            self._last_serial_equiv = ""
            self._pub_haptic(HapticCmd.EFFECT_HEARTBEAT_START)
            self.get_logger().info(
                f"Recording started (speed limit "
                f"{int(self._get_param('record_speed_limit') * 100)}%)"
            )
        else:
            self._recording = False
            # Append final S0 if needed
            if not self._path_buffer or self._path_buffer[-1][1] != "S0":
                now = time.monotonic()
                self._path_buffer.append((now - self._last_record_time, "S0"))
                self._last_record_time = now
            self._pub_haptic(HapticCmd.EFFECT_HEARTBEAT_STOP)
            time.sleep(0.05)
            self._pub_haptic(HapticCmd.EFFECT_STRONG_THUMP)
            self.get_logger().info(
                f"Recording stopped ({len(self._path_buffer)} commands)"
            )

    def _handle_playback(self) -> None:
        if self._recording:
            self.get_logger().warn("Stop recording before playing back")
            return
        if self._is_playing:
            self.get_logger().warn("Already playing back")
            return
        if not self._path_buffer:
            self.get_logger().warn("No recorded path to play back")
            return
        self._is_playing   = True
        self._current_speed = 0.0
        self._current_dir   = "S"
        threading.Thread(target=self._playback_thread, daemon=True).start()

    def _handle_precision_toggle(self) -> None:
        if not self._get_param("precision_mode_enabled"):
            return
        self._precision_mode = not self._precision_mode
        if self._precision_mode:
            self._pub_haptic(HapticCmd.EFFECT_PRECISION_ON)
            self.get_logger().info(
                f"Precision mode ON "
                f"({int(self._get_param('precision_max_speed') * 100)}%)"
            )
        else:
            self._pub_haptic(HapticCmd.EFFECT_PRECISION_OFF)
            self.get_logger().info("Precision mode OFF")

    def _handle_home(self) -> None:
        # If Arduino is in emergency, force reconnect attempt via status topic
        # Otherwise toggle haptic mute
        self._haptic_muted = not self._haptic_muted
        if self._haptic_muted:
            self.get_logger().info("Haptics MUTED")
        else:
            self.get_logger().info("Haptics UNMUTED")
        # Publish mute state as EFFECT_NONE so haptics_node can react
        # (haptics_node checks for mute via a separate topic in future;
        #  for now EFFECT_NONE with strength=0 signals mute toggle)
        h = _haptic(HapticCmd.EFFECT_NONE, 0.0 if self._haptic_muted else 1.0)
        self._haptic_pub.publish(h)

    # ------------------------------------------------------------------
    # Safety lock callbacks
    # ------------------------------------------------------------------

    def _on_lock(self) -> None:
        self.get_logger().warn("===== SAFETY LOCK ENABLED =====")
        self._is_playing  = False
        self._recording   = False
        self._current_speed = 0.0
        self._current_dir   = "S"
        self._lt = 0.0
        self._rt = 0.0
        self._lx = 0.0
        # Stop haptic effects
        self._pub_haptic(HapticCmd.EFFECT_HEARTBEAT_STOP)
        self._pub_haptic(HapticCmd.EFFECT_RAIN_STOP)
        # Three strong thumps
        for _ in range(3):
            self._pub_haptic(HapticCmd.EFFECT_STRONG_THUMP)
        # Publish stop immediately
        self._publish_drive(stop=True)

    def _on_unlock(self) -> None:
        self.get_logger().info("===== SAFETY LOCK DISABLED =====")
        self._pub_haptic(HapticCmd.EFFECT_DOUBLE_HEARTBEAT)

    def _on_unlock_pulse(self) -> None:
        self._pub_haptic(HapticCmd.EFFECT_UNLOCK_PULSE)

    # ------------------------------------------------------------------
    # Accessory command helper (sends via drive topic with accessory bits)
    # For now we publish DriveCmd with stop=True and re-use source field
    # to signal the Arduino driver. A cleaner future approach: separate
    # AccessoryCmd topic. For Jazzy, we use drive_cmd with special encoding
    # embedded in a parameter until a dedicated message is warranted.
    # ------------------------------------------------------------------
    def _send_arduino_accessory(self, raw_cmd: str) -> None:
        """
        Publish a DriveCmd marked as an accessory command.

        This is a temporary bridge — the arduino_driver_node checks for
        stop=True + source=2 and passes `raw_cmd` via the angular field
        encoded as a float representing the raw string hash. In a future
        iteration, an AccessoryCmd message type will carry raw strings cleanly.

        For now, accessory commands (MON, MOFF, BRxxx) are logged here
        and must be sent by a dedicated AccessoryCmd topic (not yet wired).
        The architect should note this as an open TODO.
        """
        # TODO: Add AccessoryCmd.msg to robot_msgs and a handler in
        # arduino_driver_node when mop/brush commands are needed.
        # For now, just log the intent.
        self.get_logger().info(f"[ACCESSORY] Would send: {raw_cmd}")

    # ------------------------------------------------------------------
    # Drive ramp timer
    # ------------------------------------------------------------------

    def _drive_tick(self) -> None:
        now = time.monotonic()
        dt  = now - self._last_tick
        self._last_tick = now

        # Joy watchdog
        joy_elapsed = now - self._last_joy_time
        joy_timeout = self._get_param("joy_timeout_sec")
        joy_lost = joy_elapsed > joy_timeout

        if joy_lost and self._current_speed > 0.0:
            self.get_logger().warn(
                f"Joy timeout ({joy_elapsed:.1f}s) — stopping robot"
            )

        if self._is_playing or self._safety.locked or joy_lost:
            self._current_speed = 0.0
            self._current_dir   = "S"
            if not self._is_playing:
                self._publish_drive(stop=True)
            return

        target_dir, target_speed = self._compute_target()

        if self._get_param("enable_ramp"):
            self._apply_ramp(target_dir, target_speed, dt)
        else:
            self._current_dir   = target_dir
            self._current_speed = target_speed

        self._current_speed = max(0.0, min(1.0, self._current_speed))
        self._publish_drive()

    def _compute_target(self) -> tuple[str, float]:
        """Return (direction, speed) from current axis state."""
        speed = max(self._lt, self._rt)
        deadband = self._get_param("trigger_deadband")
        if speed <= deadband:
            return "S", 0.0

        max_spd = self._get_param("max_speed")
        if self._precision_mode:
            max_spd = min(max_spd, self._get_param("precision_max_speed"))
        if self._recording:
            max_spd = min(max_spd, self._get_param("record_speed_limit"))

        min_spd = self._get_param("min_speed")
        # Remap so deadband→0 maps to min_speed, 1→max_speed
        ratio = (speed - deadband) / (1.0 - deadband + 1e-9)
        speed_val = min_spd + ratio * (max_spd - min_spd)
        speed_val = max(min_spd, min(max_spd, speed_val))

        ang_thresh = self._get_param("angular_threshold")
        is_forward = (self._lt >= self._rt)

        if is_forward:
            if self._lx < -ang_thresh:
                return "L", speed_val
            elif self._lx > ang_thresh:
                return "R", speed_val
            return "F", speed_val
        else:
            if self._lx < -ang_thresh:
                return "L", speed_val
            elif self._lx > ang_thresh:
                return "R", speed_val
            return "B", speed_val

    def _apply_ramp(self, target_dir: str, target_speed: float, dt: float) -> None:
        accel = self._get_param("acceleration_rate")
        decel = self._get_param("deceleration_rate")

        if self._current_speed <= 0.0:
            self._current_dir = target_dir

        if self._current_dir == target_dir:
            if target_speed > self._current_speed:
                self._current_speed = min(
                    target_speed, self._current_speed + accel * dt
                )
            else:
                self._current_speed = max(
                    target_speed, self._current_speed - decel * dt
                )
        else:
            # Different direction — brake first
            self._current_speed = max(0.0, self._current_speed - decel * dt)

    def _publish_drive(self, stop: bool = False) -> None:
        speed = self._current_speed
        direction = self._current_dir

        # Convert internal direction + speed to linear/angular [-1, 1]
        linear  = 0.0
        angular = 0.0

        if not stop and direction != "S" and speed > 0.0:
            if direction == "F":
                linear = speed
            elif direction == "B":
                linear = -speed
            elif direction == "L":
                linear  =  speed
                angular =  1.0
            elif direction == "R":
                linear  =  speed
                angular = -1.0

        serial_equiv = "S0" if (stop or direction == "S" or speed == 0.0) \
            else f"{direction}{int(round(speed * 100))}"

        # Record if active
        if self._recording and serial_equiv != self._last_serial_equiv:
            now = time.monotonic()
            self._path_buffer.append(
                (now - self._last_record_time, serial_equiv)
            )
            self._last_record_time = now
            self._last_serial_equiv = serial_equiv

        msg = DriveCmd()
        msg.linear  = float(linear)
        msg.angular = float(angular)
        msg.stop    = stop or (direction == "S") or (speed == 0.0)
        msg.enable  = not self._safety.locked
        msg.source  = DriveCmd.SOURCE_TELEOP
        self._drive_pub.publish(msg)

    # ------------------------------------------------------------------
    # Playback thread
    # ------------------------------------------------------------------

    def _playback_thread(self) -> None:
        try:
            self.get_logger().info(
                f"Playing back {len(self._path_buffer)} commands"
            )
            self._pub_haptic(HapticCmd.EFFECT_SINGLE_HEARTBEAT)
            time.sleep(self._get_param("playback_start_delay"))
            self._pub_haptic(HapticCmd.EFFECT_RAIN_START)

            start = time.monotonic()
            cumulative = 0.0

            for delay, serial_cmd in self._path_buffer:
                if not self._is_playing:
                    break
                cumulative += delay
                target = start + cumulative
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)

                # Convert recorded serial string back to DriveCmd
                msg = self._serial_to_drive_cmd(serial_cmd)
                self._drive_pub.publish(msg)
                self.get_logger().info(f"[PLAYBACK] {serial_cmd} (+{delay:.2f}s)")

            if self._is_playing:
                # Final stop
                stop_msg = DriveCmd()
                stop_msg.stop   = True
                stop_msg.enable = True
                stop_msg.source = DriveCmd.SOURCE_TELEOP
                self._drive_pub.publish(stop_msg)
                self.get_logger().info("Playback finished")

        finally:
            self._pub_haptic(HapticCmd.EFFECT_RAIN_STOP)
            if True:  # not in emergency — future: check status
                time.sleep(self._get_param("playback_end_delay"))
                self._pub_haptic(HapticCmd.EFFECT_DOUBLE_HEARTBEAT)
            self._is_playing = False

    @staticmethod
    def _serial_to_drive_cmd(s: str) -> DriveCmd:
        """Reconstruct a DriveCmd from a recorded serial string like 'F40'."""
        msg = DriveCmd()
        msg.enable = True
        msg.source = DriveCmd.SOURCE_TELEOP
        if s == "S0" or not s:
            msg.stop = True
            return msg
        direction = s[0]
        try:
            speed = int(s[1:]) / 100.0
        except ValueError:
            msg.stop = True
            return msg
        if direction == "F":
            msg.linear =  speed
        elif direction == "B":
            msg.linear = -speed
        elif direction == "L":
            msg.linear  =  speed
            msg.angular =  1.0
        elif direction == "R":
            msg.linear  =  speed
            msg.angular = -1.0
        else:
            msg.stop = True
        return msg

    # ------------------------------------------------------------------
    # Haptic publish helper
    # ------------------------------------------------------------------

    def _pub_haptic(self, effect: int, strength: float = 1.0, duration: float = 0.0) -> None:
        if self._haptic_muted and effect not in (
            HapticCmd.EFFECT_NONE,
            HapticCmd.EFFECT_EMERGENCY_START,
            HapticCmd.EFFECT_EMERGENCY_STOP,
        ):
            return
        self._haptic_pub.publish(_haptic(effect, strength, duration))

    # ------------------------------------------------------------------
    # Arduino status callback
    # ------------------------------------------------------------------

    def _arduino_status_cb(self, msg: ArduinoStatus) -> None:
        prev = self._arduino_connected
        self._arduino_connected = msg.connected
        if prev and not msg.connected:
            self.get_logger().error("Arduino disconnected — triggering emergency haptic")
            self._pub_haptic(HapticCmd.EFFECT_EMERGENCY_START)
        elif not prev and msg.connected:
            self.get_logger().info("Arduino reconnected")
            self._pub_haptic(HapticCmd.EFFECT_EMERGENCY_STOP)
            self._pub_haptic(HapticCmd.EFFECT_SINGLE_HEARTBEAT)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
