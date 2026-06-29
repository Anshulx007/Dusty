"""
haptic_effects.py
=================
Re-exports haptic effect constants from robot_msgs/HapticCmd.
Import from here to avoid coupling unrelated modules to the full message type.

Usage:
    from robot_haptics.haptic_effects import EFFECT_SINGLE_BUZZ
    # or
    from robot_haptics import haptic_effects as FX
    self._pub_haptic(FX.SINGLE_BUZZ)
"""

# These values must stay in sync with robot_msgs/msg/HapticCmd.msg
NONE             = 0
SINGLE_BUZZ      = 1
DOUBLE_BUZZ      = 2
LONG_BUZZ        = 3
LONG_THEN_SHORT  = 4
STRONG_THUMP     = 5
SINGLE_HEARTBEAT = 6
DOUBLE_HEARTBEAT = 7
HEARTBEAT_START  = 8
HEARTBEAT_STOP   = 9
RAIN_START       = 10
RAIN_STOP        = 11
EMERGENCY_START  = 12
EMERGENCY_STOP   = 13
BRUSH_SPEED_1    = 14
BRUSH_SPEED_2    = 15
BRUSH_SPEED_3    = 16
PRECISION_ON     = 17
PRECISION_OFF    = 18
UNLOCK_PULSE     = 19
