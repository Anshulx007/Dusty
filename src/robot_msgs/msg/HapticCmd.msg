# HapticCmd.msg
# Published by teleop_node; consumed by haptics_node.
# Describes a single haptic event to play on the controller.

# Effect type. Use the constants below.
uint8 effect

# Strength multiplier [0.0, 1.0]. Applied on top of global HAPTIC_SCALE.
float32 strength

# Effect duration in seconds. Ignored for preset effects that have
# their own internal timing (e.g. HEARTBEAT).
float32 duration

# Effect constants — kept in sync with haptics_node presets.
uint8 EFFECT_NONE=0
uint8 EFFECT_SINGLE_BUZZ=1
uint8 EFFECT_DOUBLE_BUZZ=2
uint8 EFFECT_LONG_BUZZ=3
uint8 EFFECT_LONG_THEN_SHORT=4
uint8 EFFECT_STRONG_THUMP=5
uint8 EFFECT_SINGLE_HEARTBEAT=6
uint8 EFFECT_DOUBLE_HEARTBEAT=7
uint8 EFFECT_HEARTBEAT_START=8
uint8 EFFECT_HEARTBEAT_STOP=9
uint8 EFFECT_RAIN_START=10
uint8 EFFECT_RAIN_STOP=11
uint8 EFFECT_EMERGENCY_START=12
uint8 EFFECT_EMERGENCY_STOP=13
uint8 EFFECT_BRUSH_SPEED_1=14
uint8 EFFECT_BRUSH_SPEED_2=15
uint8 EFFECT_BRUSH_SPEED_3=16
uint8 EFFECT_PRECISION_ON=17
uint8 EFFECT_PRECISION_OFF=18
uint8 EFFECT_UNLOCK_PULSE=19
