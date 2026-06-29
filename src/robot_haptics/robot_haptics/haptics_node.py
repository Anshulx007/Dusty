#!/usr/bin/env python3
"""
haptics_node
============
Force-feedback haptics node. Opens the Ares controller for WRITE ONLY
(force feedback output). Never reads joystick input — that is joy_node's job.

Responsibilities
----------------
* Open /dev/input/eventX using python-evdev, only for FF_RUMBLE output.
* Subscribe to /haptics_cmd (robot_msgs/HapticCmd).
* Execute the requested effect (single buzz, heartbeat, rain, emergency, etc.).
* Maintain a notification queue so effects are serialised and don't overlap.
* Preserve the complete haptic repertoire from the original monolith.

What this node does NOT do
--------------------------
* It never reads joystick buttons or axes.
* It never sends commands to the Arduino.
* It has no knowledge of drive commands or navigation.
"""

from __future__ import annotations

import queue
import random
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robot_msgs.msg import HapticCmd

try:
    from evdev import InputDevice, ecodes, list_devices, ff
    HAS_EVDEV = True
except ImportError:
    HAS_EVDEV = False


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class HapticsNode(Node):
    """Force-feedback haptics node."""

    _DEFAULTS = {
        # Device
        "device_name":  "Ares",          # evdev device.name to search for
        "device_path":  "",               # set to override auto-search, e.g. /dev/input/event3
        "dry_run":      False,            # log effects without hardware

        # Global
        "haptic_scale":      1.0,         # 0.0–1.0 scales ALL magnitudes
        "ff_gain_normal":    0.70,        # FF_GAIN baseline (0.0–1.0)
        "ff_gain_emergency": 1.00,        # FF_GAIN during emergency

        # Heartbeat pulse
        "hb_strong1": 65000, "hb_weak1": 25000, "hb_dur1": 80,
        "hb_strong2": 60000, "hb_weak2": 22000, "hb_dur2": 60,
        "hb_gap":     0.05,
        "hb_period":  0.85,

        # Rain (playback ambience)
        "rain_weak_min":   22000, "rain_weak_max":   40000,
        "rain_strong_min":     0, "rain_strong_max": 10000,
        "rain_dur_min":       60, "rain_dur_max":      130,
        "rain_gap_min":     0.04, "rain_gap_max":     0.18,
        "rain_heavy_chance": 0.15,
        "rain_heavy_strong": 22000, "rain_heavy_weak": 50000, "rain_heavy_dur": 90,

        # Emergency
        "emergency_strong": 65535, "emergency_weak": 40000,
        "emergency_pulse":    150, "emergency_gap":  0.12,
        "emergency_pause":   0.60,

        # Mop
        "mop_on_weak": 30000,  "mop_on_dur": 120,  "mop_on_sleep": 0.17,
        "mop_off_weak": 28000, "mop_off_dur": 100, "mop_off_sleep": 0.15, "mop_off_gap": 0.08,

        # Brush
        "brush_on_weak": 32000, "brush_on_dur": 350, "brush_on_sleep": 0.40,
        "brush_off_short_weak": 20000, "brush_off_short_dur": 100, "brush_off_short_sleep": 0.15,
        "brush_off_gap": 0.10,
        "brush_speed_buzz_gap": 0.12,

        # Thump (recording stop)
        "thump_strong": 60000, "thump_dur": 120, "thump_sleep": 0.18,

        # Safety unlock progress
        "safety_pulse_weak": 20000, "safety_pulse_dur": 80,

        # Notification queue pauses
        "notif_rain_pause":  0.04,
        "notif_rain_resume": 0.05,

        # Thread join
        "thread_join_timeout": 1.5,
    }

    def __init__(self) -> None:
        super().__init__("haptics_node")
        self._declare_params()

        # ---- device ----
        self._device: Optional["InputDevice"] = None
        self._device_lock = threading.Lock()
        self._muted = False
        self._emergency_active = threading.Event()

        # ---- continuous effect system ----
        self._continuous_lock       = threading.Lock()
        self._continuous_generation = 0
        self._continuous_stop_event = threading.Event()
        self._continuous_thread: Optional[threading.Thread] = None
        self._current_mode: Optional[str] = None  # 'heartbeat' | 'rain' | None

        # ---- rain pause ----
        self._rain_paused = threading.Event()

        # ---- notification queue ----
        self._notify_queue: queue.Queue = queue.Queue()
        self._notify_thread = threading.Thread(
            target=self._notify_worker, daemon=True
        )
        self._notify_thread.start()

        # ---- open device ----
        if self._get_param("dry_run"):
            self.get_logger().warn("DRY RUN — no force-feedback hardware used")
        elif HAS_EVDEV:
            self._open_device()
        else:
            self.get_logger().error(
                "python-evdev not installed — haptics disabled. "
                "Install with: pip install evdev"
            )

        # ---- QoS ----
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20,
        )

        # ---- subscription ----
        self._haptic_sub = self.create_subscription(
            HapticCmd, "/haptics_cmd", self._haptic_cb, reliable
        )

        self.get_logger().info("haptics_node ready")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        for name, default in self._DEFAULTS.items():
            self.declare_parameter(name, default)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def _p(self, name: str):
        """Short alias for _get_param."""
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Device management
    # ------------------------------------------------------------------

    def _open_device(self) -> bool:
        override = self._get_param("device_path")
        target_name = self._get_param("device_name")

        if override:
            paths = [override]
        else:
            paths = list_devices()

        for path in paths:
            try:
                dev = InputDevice(path)
                if override or dev.name == target_name:
                    with self._device_lock:
                        self._device = dev
                    self._set_ff_gain(self._p("ff_gain_normal"))
                    self.get_logger().info(
                        f"Haptics device opened: {dev.name} @ {path}"
                    )
                    return True
            except Exception as exc:
                self.get_logger().debug(f"Could not open {path}: {exc}")

        self.get_logger().error(
            f"Force-feedback device '{target_name}' not found. "
            "Set 'device_path' parameter to specify it manually."
        )
        return False

    # ------------------------------------------------------------------
    # Low-level FF helpers
    # ------------------------------------------------------------------

    def _scaled(self, value: int) -> int:
        return min(65535, int(value * self._p("haptic_scale")))

    def _upload_rumble(self, strong: int, weak: int, duration_ms: int) -> int:
        if self._get_param("dry_run"):
            return 0  # fake effect id
        with self._device_lock:
            if self._device is None:
                return -1
            try:
                rumble = ff.Rumble(
                    strong_magnitude=self._scaled(strong),
                    weak_magnitude=self._scaled(weak),
                )
                effect = ff.Effect(
                    ecodes.FF_RUMBLE, -1, 0,
                    ff.Trigger(0, 0),
                    ff.Replay(duration_ms, 0),
                    ff.EffectType(ff_rumble_effect=rumble),
                )
                return self._device.upload_effect(effect)
            except Exception:
                self.get_logger().exception("[FF] upload failed")
                return -1

    def _play_effect(self, eid: int, repeat: int = 1) -> None:
        if eid < 0 or self._muted:
            return
        if self._get_param("dry_run"):
            return
        with self._device_lock:
            if self._device is None:
                return
            try:
                self._device.write(ecodes.EV_FF, eid, repeat)
            except Exception:
                self.get_logger().exception("[FF] play failed")

    def _erase_effect(self, eid: int) -> None:
        if eid < 0:
            return
        if self._get_param("dry_run"):
            return
        with self._device_lock:
            if self._device is None:
                return
            try:
                self._device.write(ecodes.EV_FF, eid, 0)
                self._device.erase_effect(eid)
            except Exception:
                pass

    def _set_ff_gain(self, fraction: float) -> None:
        if self._get_param("dry_run"):
            return
        with self._device_lock:
            if self._device is None:
                return
            try:
                self._device.write(ecodes.EV_FF, ecodes.FF_GAIN, int(fraction * 0xFFFF))
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Continuous effect system
    # ------------------------------------------------------------------

    def _stop_continuous(self, wait: bool = True) -> None:
        with self._continuous_lock:
            self._continuous_generation += 1
            self._continuous_stop_event.set()
            t = self._continuous_thread
            self._continuous_thread = None
        if wait and t is not None and t.is_alive():
            t.join(timeout=self._p("thread_join_timeout"))
        self._continuous_stop_event.clear()

    def _set_continuous(self, fn) -> None:
        self._stop_continuous()
        with self._continuous_lock:
            gen = self._continuous_generation
        t = threading.Thread(target=lambda: fn(gen), daemon=True)
        with self._continuous_lock:
            self._continuous_thread = t
        t.start()

    def _still_current(self, gen: int) -> bool:
        with self._continuous_lock:
            return (
                gen == self._continuous_generation
                and not self._continuous_stop_event.is_set()
            )

    def _wait(self, seconds: float) -> None:
        """Interruptible sleep respecting continuous stop event."""
        self._continuous_stop_event.wait(seconds)

    # ------------------------------------------------------------------
    # Notification queue
    # ------------------------------------------------------------------

    def _notify_worker(self) -> None:
        while True:
            fn = self._notify_queue.get()
            was_rain = (self._current_mode == "rain")
            if was_rain:
                self._rain_paused.set()
                time.sleep(self._p("notif_rain_pause"))
            try:
                fn()
            finally:
                if was_rain:
                    time.sleep(self._p("notif_rain_resume"))
                    self._rain_paused.clear()
                self._notify_queue.task_done()

    def _notify(self, fn) -> None:
        self._notify_queue.put(fn)

    def _clear_notify_queue(self) -> None:
        while not self._notify_queue.empty():
            try:
                self._notify_queue.get_nowait()
                self._notify_queue.task_done()
            except Exception:
                break

    # ------------------------------------------------------------------
    # Notification effect implementations
    # ------------------------------------------------------------------

    def _do_single_buzz(self) -> None:
        eid = self._upload_rumble(0, self._p("mop_on_weak"), self._p("mop_on_dur"))
        self._play_effect(eid)
        time.sleep(self._p("mop_on_sleep"))
        self._erase_effect(eid)
        self.get_logger().debug("[HAPTIC] single_buzz")

    def _do_double_buzz(self) -> None:
        for _ in range(2):
            eid = self._upload_rumble(0, self._p("mop_off_weak"), self._p("mop_off_dur"))
            self._play_effect(eid)
            time.sleep(self._p("mop_off_sleep"))
            self._erase_effect(eid)
            time.sleep(self._p("mop_off_gap"))
        self.get_logger().debug("[HAPTIC] double_buzz")

    def _do_long_buzz(self) -> None:
        eid = self._upload_rumble(0, self._p("brush_on_weak"), self._p("brush_on_dur"))
        self._play_effect(eid)
        time.sleep(self._p("brush_on_sleep"))
        self._erase_effect(eid)

    def _do_long_then_short_buzz(self) -> None:
        self._do_long_buzz()
        time.sleep(self._p("brush_off_gap"))
        eid = self._upload_rumble(
            0, self._p("brush_off_short_weak"), self._p("brush_off_short_dur")
        )
        self._play_effect(eid)
        time.sleep(self._p("brush_off_short_sleep"))
        self._erase_effect(eid)
        self.get_logger().debug("[HAPTIC] long_then_short_buzz")

    def _do_brush_speed_buzz(self, level: int) -> None:
        """Buzz `level` times (1 = slowest speed level)."""
        for i in range(level):
            eid = self._upload_rumble(
                0,
                self._p("brush_off_short_weak"),
                self._p("brush_off_short_dur"),
            )
            self._play_effect(eid)
            time.sleep(self._p("brush_off_short_sleep"))
            self._erase_effect(eid)
            if i < level - 1:
                time.sleep(self._p("brush_speed_buzz_gap"))
        self.get_logger().debug(f"[HAPTIC] brush_speed_buzz level={level}")

    def _do_strong_thump(self) -> None:
        eid = self._upload_rumble(self._p("thump_strong"), 0, self._p("thump_dur"))
        self._play_effect(eid)
        time.sleep(self._p("thump_sleep"))
        self._erase_effect(eid)
        self.get_logger().debug("[HAPTIC] strong_thump")

    def _do_single_heartbeat(self) -> None:
        eid1 = self._upload_rumble(
            self._p("hb_strong1"), self._p("hb_weak1"), self._p("hb_dur1")
        )
        self._play_effect(eid1)
        time.sleep(self._p("hb_dur1") / 1000.0)
        self._erase_effect(eid1)
        time.sleep(self._p("hb_gap"))
        eid2 = self._upload_rumble(
            self._p("hb_strong2"), self._p("hb_weak2"), self._p("hb_dur2")
        )
        self._play_effect(eid2)
        time.sleep(self._p("hb_dur2") / 1000.0)
        self._erase_effect(eid2)
        self.get_logger().debug("[HAPTIC] single_heartbeat")

    def _do_double_heartbeat(self) -> None:
        for _ in range(2):
            self._do_single_heartbeat()
            time.sleep(self._p("hb_period") * 0.45)
        self.get_logger().debug("[HAPTIC] double_heartbeat")

    def _do_unlock_pulse(self) -> None:
        eid = self._upload_rumble(
            0, self._p("safety_pulse_weak"), self._p("safety_pulse_dur")
        )
        self._play_effect(eid)
        time.sleep(self._p("safety_pulse_dur") / 1000.0)
        self._erase_effect(eid)

    # ------------------------------------------------------------------
    # Continuous: heartbeat
    # ------------------------------------------------------------------

    def _heartbeat_loop(self, gen: int) -> None:
        self._current_mode = "heartbeat"
        while self._still_current(gen):
            eid1 = self._upload_rumble(
                self._p("hb_strong1"), self._p("hb_weak1"), self._p("hb_dur1")
            )
            self._play_effect(eid1)
            self._wait(self._p("hb_dur1") / 1000.0)
            self._erase_effect(eid1)
            if not self._still_current(gen): break
            self._wait(self._p("hb_gap"))
            if not self._still_current(gen): break
            eid2 = self._upload_rumble(
                self._p("hb_strong2"), self._p("hb_weak2"), self._p("hb_dur2")
            )
            self._play_effect(eid2)
            self._wait(self._p("hb_dur2") / 1000.0)
            self._erase_effect(eid2)
            self._wait(self._p("hb_period"))
        self._current_mode = None

    # ------------------------------------------------------------------
    # Continuous: rain
    # ------------------------------------------------------------------

    def _rain_loop(self, gen: int) -> None:
        self._current_mode = "rain"
        while self._still_current(gen):
            if self._rain_paused.is_set():
                self._continuous_stop_event.wait(0.05)
                continue
            weak   = random.randint(self._p("rain_weak_min"),   self._p("rain_weak_max"))
            strong = random.randint(self._p("rain_strong_min"), self._p("rain_strong_max"))
            dur    = random.randint(self._p("rain_dur_min"),    self._p("rain_dur_max"))
            eid = self._upload_rumble(strong, weak, dur)
            self._play_effect(eid)
            self._wait(dur / 1000.0)
            self._erase_effect(eid)
            if not self._still_current(gen): break
            if (random.random() < self._p("rain_heavy_chance")
                    and not self._rain_paused.is_set()):
                eid2 = self._upload_rumble(
                    self._p("rain_heavy_strong"),
                    self._p("rain_heavy_weak"),
                    self._p("rain_heavy_dur"),
                )
                self._play_effect(eid2)
                self._wait(self._p("rain_heavy_dur") / 1000.0)
                self._erase_effect(eid2)
            self._wait(random.uniform(self._p("rain_gap_min"), self._p("rain_gap_max")))
        self._current_mode = None

    # ------------------------------------------------------------------
    # Emergency loop
    # ------------------------------------------------------------------

    def _emergency_loop(self) -> None:
        self._set_ff_gain(self._p("ff_gain_emergency"))
        while self._emergency_active.is_set():
            for _ in range(3):
                if not self._emergency_active.is_set(): break
                eid = self._upload_rumble(
                    self._p("emergency_strong"),
                    self._p("emergency_weak"),
                    self._p("emergency_pulse"),
                )
                self._play_effect(eid)
                self._emergency_active.wait(self._p("emergency_pulse") / 1000.0)
                self._erase_effect(eid)
                if not self._emergency_active.is_set(): break
                self._emergency_active.wait(self._p("emergency_gap"))
            self._emergency_active.wait(self._p("emergency_pause"))
        self._set_ff_gain(self._p("ff_gain_normal"))

    # ------------------------------------------------------------------
    # HapticCmd callback — dispatch effect
    # ------------------------------------------------------------------

    def _haptic_cb(self, msg: HapticCmd) -> None:  # noqa: C901
        E = HapticCmd
        effect = msg.effect

        self.get_logger().debug(f"[HAPTIC] effect={effect} strength={msg.strength:.2f}")

        # --- Mute toggle (EFFECT_NONE with strength=0 = mute; strength=1 = unmute) ---
        if effect == E.EFFECT_NONE:
            muted = (msg.strength == 0.0)
            if muted != self._muted:
                self._muted = muted
                if muted:
                    self._set_ff_gain(0)
                    self._stop_continuous()
                    self.get_logger().info("[HAPTIC] MUTED")
                else:
                    self._set_ff_gain(self._p("ff_gain_normal"))
                    self.get_logger().info("[HAPTIC] UNMUTED")
            return

        # --- One-shot notifications ---
        if effect == E.EFFECT_SINGLE_BUZZ:
            self._notify(self._do_single_buzz)
        elif effect == E.EFFECT_DOUBLE_BUZZ:
            self._notify(self._do_double_buzz)
        elif effect == E.EFFECT_LONG_BUZZ:
            self._notify(self._do_long_buzz)
        elif effect == E.EFFECT_LONG_THEN_SHORT:
            self._notify(self._do_long_then_short_buzz)
        elif effect == E.EFFECT_STRONG_THUMP:
            self._notify(self._do_strong_thump)
        elif effect == E.EFFECT_SINGLE_HEARTBEAT:
            self._notify(self._do_single_heartbeat)
        elif effect == E.EFFECT_DOUBLE_HEARTBEAT:
            self._notify(self._do_double_heartbeat)
        elif effect == E.EFFECT_UNLOCK_PULSE:
            self._notify(self._do_unlock_pulse)

        # --- Brush speed buzzes ---
        elif effect == E.EFFECT_BRUSH_SPEED_1:
            self._notify(lambda: self._do_brush_speed_buzz(1))
        elif effect == E.EFFECT_BRUSH_SPEED_2:
            self._notify(lambda: self._do_brush_speed_buzz(2))
        elif effect == E.EFFECT_BRUSH_SPEED_3:
            self._notify(lambda: self._do_brush_speed_buzz(3))

        # --- Precision mode ---
        elif effect == E.EFFECT_PRECISION_ON:
            self._notify(self._do_single_buzz)
        elif effect == E.EFFECT_PRECISION_OFF:
            self._notify(self._do_double_buzz)

        # --- Continuous: heartbeat ---
        elif effect == E.EFFECT_HEARTBEAT_START:
            self._clear_notify_queue()
            self._set_continuous(self._heartbeat_loop)
        elif effect == E.EFFECT_HEARTBEAT_STOP:
            self._stop_continuous()

        # --- Continuous: rain ---
        elif effect == E.EFFECT_RAIN_START:
            self._rain_paused.clear()
            self._set_continuous(self._rain_loop)
        elif effect == E.EFFECT_RAIN_STOP:
            self._stop_continuous()

        # --- Emergency ---
        elif effect == E.EFFECT_EMERGENCY_START:
            if not self._emergency_active.is_set():
                self._stop_continuous()
                self._emergency_active.set()
                threading.Thread(
                    target=self._emergency_loop, daemon=True
                ).start()
                self.get_logger().error("[HAPTIC] Emergency haptic started")
        elif effect == E.EFFECT_EMERGENCY_STOP:
            self._emergency_active.clear()
            self.get_logger().info("[HAPTIC] Emergency haptic stopped")

        else:
            self.get_logger().warn(f"Unknown haptic effect: {effect}")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info("haptics_node shutting down")
        self._stop_continuous()
        self._emergency_active.clear()
        self._clear_notify_queue()
        with self._device_lock:
            if self._device:
                try:
                    self._device.close()
                except Exception:
                    pass
                self._device = None
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = HapticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
