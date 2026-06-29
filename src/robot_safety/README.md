# robot_safety

Safety watchdog and command arbitration node for the robot.

**Current status: stub / pass-through.** Subscribes to `/drive_cmd_raw`
and re-publishes on `/drive_cmd` with a watchdog timer. Full arbitration
logic is not yet implemented.

---

## Intended architecture

```
teleop_node      ─┐
                  ├──► /drive_cmd_raw ──► safety_node ──► /drive_cmd ──► arduino_driver_node
navigation_node  ─┘
```

`safety_node` is the **sole publisher to `/drive_cmd`**. Teleop and
navigation publish to `/drive_cmd_raw`; safety decides what reaches the
Arduino.

Until this wiring is complete, `teleop_node` and `arduino_driver_node`
communicate directly on `/drive_cmd` as before.

---

## Responsibilities (planned)

- **Watchdog** — E-stop if no command arrives within `teleop_timeout_sec`
  or `nav_timeout_sec`
- **E-stop** — publishes `DriveCmd(stop=True, source=SOURCE_EMERGENCY)`
  immediately on trigger
- **Mode arbitration** — only one source active at a time
  (`MODE_MANUAL` → teleop wins; `MODE_AUTONOMOUS` → navigation wins)
- **Heartbeat** — monitors upstream node liveness
- **Velocity clamping** — enforces `max_linear` / `max_angular` limits
  regardless of source

---

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/drive_cmd_raw` | `robot_msgs/DriveCmd` | Subscribe (from teleop + nav) |
| `/drive_cmd` | `robot_msgs/DriveCmd` | Publish (to arduino_driver_node) |
| `/robot_mode` | `robot_msgs/RobotMode` | Publish |
| `/arduino_status` | `robot_msgs/ArduinoStatus` | Subscribe |

---

## Parameters

See `config/safety_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `teleop_timeout_sec` | `0.5` | Silence before teleop E-stop |
| `nav_timeout_sec` | `1.0` | Silence before navigation E-stop |
| `watchdog_rate_hz` | `20.0` | Timer rate for the watchdog tick |

---

## Enabling safety in the full stack

1. In `robot_bringup/launch/robot.launch.py`, add `safety_node`.
2. Remap `teleop_node` output: `/drive_cmd` → `/drive_cmd_raw`.
3. Remap `navigation_node` output: `/drive_cmd` → `/drive_cmd_raw`.
4. `arduino_driver_node` keeps subscribing to `/drive_cmd` (no change).

```python
# teleop_node in robot.launch.py
remappings=[("/drive_cmd", "/drive_cmd_raw")],

# navigation_node in robot.launch.py
remappings=[("/drive_cmd", "/drive_cmd_raw")],
```

---

## Building & Running

```bash
cd ~/robot_ws
colcon build --packages-select robot_msgs robot_safety
source install/setup.bash

ros2 launch robot_safety safety.launch.py
```
