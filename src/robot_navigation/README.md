# robot_navigation

Autonomous navigation node for the robot.

**Current status: stub.** The node compiles and runs but publishes stop
commands only (`enable: false`). Replace `_navigation_tick` in
`navigation_node.py` with real logic when ready.

---

## Architecture contract

This node publishes to `/drive_cmd` with `source = SOURCE_NAV (1)`,
exactly the same topic and message type that `teleop_node` uses with
`source = SOURCE_TELEOP (0)`.

`arduino_driver_node` has no knowledge of the source — it executes
whatever `DriveCmd` arrives. When `robot_safety` is implemented, it
will arbitrate between teleop and navigation commands before they reach
the driver. Until then, only one of the two should be active at a time.

```
navigation_node
      │
      ▼  /drive_cmd  (source = SOURCE_NAV)
arduino_driver_node
      │
      ▼  Serial
    Arduino
```

When `robot_safety` is in the graph, remap the output topic:

```python
# in navigation.launch.py
remappings=[("/drive_cmd", "/drive_cmd_raw")],
```

---

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/drive_cmd` | `robot_msgs/DriveCmd` | Publish |
| `/arduino_status` | `robot_msgs/ArduinoStatus` | Subscribe |

---

## Parameters

See `config/navigation_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate_hz` | `10.0` | Timer rate for the navigation tick |

---

## Building & Running

```bash
cd ~/robot_ws
colcon build --packages-select robot_msgs robot_navigation
source install/setup.bash

# Run the stub
ros2 launch robot_navigation navigation.launch.py

# Monitor output
ros2 topic echo /drive_cmd
```

---

## Implementing navigation

Replace `_navigation_tick` in `robot_navigation/navigation_node.py`.
The method must publish a `DriveCmd` message with:

```python
msg.source = DriveCmd.SOURCE_NAV   # 1
msg.enable = True                  # required for arduino_driver to act
msg.linear  = <float [-1, 1]>
msg.angular = <float [-1, 1]>
msg.stop    = False
```

Planned algorithms:

- **VFH+** — Vector Field Histogram for local obstacle avoidance
- **Frontier exploration** — autonomous mapping
- **Pure pursuit** — waypoint path following

---

## Planned sensor inputs

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR |
| `/odom` | `nav_msgs/Odometry` | Encoder odometry node (future) |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM node (future) |
