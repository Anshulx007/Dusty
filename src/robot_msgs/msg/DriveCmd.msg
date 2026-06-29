# DriveCmd.msg
# Unified drive command published by teleop_node and navigation_node.
# arduino_driver_node is the sole consumer — it never knows the source.

# Normalized [-1.0, 1.0] linear velocity. Positive = forward.
float32 linear

# Normalized [-1.0, 1.0] angular velocity. Positive = left turn.
float32 angular

# When true, override linear/angular and immediately stop the robot.
bool stop

# Safety enable flag. Command is only executed when this is true.
bool enable

# Command source identifier.
# 0 = teleop   (manual gamepad)
# 1 = navigation (autonomous)
# 2 = emergency  (E-stop / watchdog)
uint8 source

# Source constants
uint8 SOURCE_TELEOP=0
uint8 SOURCE_NAV=1
uint8 SOURCE_EMERGENCY=2
