# RobotMode.msg
# Published by robot_safety (future) or teleop_node.
# Describes the current operating mode of the robot.

uint8 mode

# Mode constants
uint8 MODE_MANUAL=0
uint8 MODE_AUTONOMOUS=1
uint8 MODE_PAUSED=2
uint8 MODE_EMERGENCY_STOP=3

# Requesting node identifier string, e.g. "teleop" or "navigation".
string requester
