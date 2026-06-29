# robot_driver

Arduino serial driver node for the robot.

## Responsibility

`arduino_driver_node` is the **sole owner** of the Arduino serial connection.

It does exactly three things:

1. **Connects** to the Arduino (auto-detects USB port, confirms with `READY` string, reconnects on loss).
2. **Receives** `/drive_cmd` and converts it to the existing text protocol (`F40`, `B30`, `L40`, `R40`, `S0`).
3. **Publishes** `/arduino_status` so every other node can see connection health.

Nothing else belongs here — no joystick reading, no safety logic, no haptics.

## Serial Protocol (preserved exactly)

| DriveCmd | Serial string |
|----------|---------------|
| Forward 40% | `F40` |
| Backward 60% | `B60` |
| Left 50% | `L50` |
| Right 50% | `R50` |
| Stop | `S0` |
| Mop on | `MON` |
| Mop off | `MOFF` |
| Brush PWM | `BR<0-255>` |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/drive_cmd` | `robot_msgs/DriveCmd` | Subscribe |
| `/arduino_status` | `robot_msgs/ArduinoStatus` | Publish |

## Parameters

See `config/driver_params.yaml`. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyACM0` | Preferred serial port |
| `baud` | `9600` | Baud rate |
| `reconnect_interval` | `5.0` | Seconds between reconnect attempts |
| `cmd_timeout_sec` | `0.5` | Seconds before watchdog sends S0 |
| `dry_run` | `false` | Simulate without hardware |

## Building & Running

```bash
# Build
cd ~/robot_ws
colcon build --packages-select robot_msgs robot_driver
source install/setup.bash

# Run with hardware
ros2 launch robot_driver driver.launch.py

# Run in dry-run mode (no Arduino needed)
ros2 launch robot_driver driver.launch.py dry_run:=true

# Test: publish a drive command manually
ros2 topic pub /drive_cmd robot_msgs/msg/DriveCmd \
  '{linear: 0.4, angular: 0.0, stop: false, enable: true, source: 0}'

# Monitor status
ros2 topic echo /arduino_status
```

## Safety Notes

- The watchdog sends `S0` if no `/drive_cmd` is received within `cmd_timeout_sec`.
- On shutdown, `S0` is sent before the port is closed.
- On Arduino disconnect, reconnect is scheduled automatically.
