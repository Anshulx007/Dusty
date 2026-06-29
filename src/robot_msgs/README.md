# robot_msgs

Custom ROS 2 message definitions for the robot project.

## Messages

### `DriveCmd.msg`

Unified motion command published by **teleop_node** and **navigation_node**.
`arduino_driver_node` is the sole subscriber — it never knows the source.

| Field | Type | Description |
|-------|------|-------------|
| `linear` | `float32` | Normalized `[-1.0, 1.0]`. Positive = forward |
| `angular` | `float32` | Normalized `[-1.0, 1.0]`. Positive = left turn |
| `stop` | `bool` | Override and stop immediately |
| `enable` | `bool` | Safety gate — command ignored when `false` |
| `source` | `uint8` | `0`=teleop, `1`=navigation, `2`=emergency |

### `ArduinoStatus.msg`

Published by **arduino_driver_node** to expose connection health.

| Field | Type | Description |
|-------|------|-------------|
| `connected` | `bool` | Serial port open and board confirmed |
| `battery` | `float32` | Voltage from sketch (0.0 if unsupported) |
| `estop` | `bool` | Arduino-side E-stop flag |
| `state` | `string` | e.g. `"IDLE"`, `"DRIVING"`, `"ERROR"` |
| `last_cmd` | `string` | Last raw string sent, e.g. `"F40"` |

### `HapticCmd.msg`

Published by **teleop_node**, consumed by **haptics_node**.

| Field | Type | Description |
|-------|------|-------------|
| `effect` | `uint8` | Effect constant (see message file) |
| `strength` | `float32` | Multiplier `[0.0, 1.0]` |
| `duration` | `float32` | Duration in seconds (preset effects ignore this) |

### `RobotMode.msg`

Describes current operating mode. Future use by **robot_safety**.

| Field | Type | Description |
|-------|------|-------------|
| `mode` | `uint8` | `0`=MANUAL, `1`=AUTO, `2`=PAUSED, `3`=ESTOP |
| `requester` | `string` | Node name requesting the mode |

## Building

```bash
cd ~/robot_ws
colcon build --packages-select robot_msgs
source install/setup.bash
```
