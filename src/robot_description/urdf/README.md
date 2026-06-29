# urdf/

Place your robot URDF or Xacro files here.

## Expected entry point

```
robot_description/
└── urdf/
    └── robot.urdf.xacro   ← primary model file (referenced by display.launch.py)
```

`display.launch.py` passes this file through `xacro` before handing it to
`robot_state_publisher`. If you use plain URDF instead of Xacro, rename
accordingly and update the `urdf_file` argument default in the launch file.

## Suggested structure

```
urdf/
├── robot.urdf.xacro       # top-level file — includes all others
├── base.urdf.xacro        # chassis, wheels, casters
├── sensors.urdf.xacro     # LIDAR, camera mounts
└── materials.xacro        # colour / material definitions
```

## Coordinate frame conventions (ROS REP-103)

- `base_link` — robot body origin, at floor level, centred between drive wheels
- `base_footprint` — projection of `base_link` onto the ground plane
- `left_wheel_link`, `right_wheel_link` — driven wheels
- `laser_link` — LIDAR sensor origin (x forward, y left, z up)

## Tools

```bash
# Validate URDF
check_urdf urdf/robot.urdf

# Process Xacro manually
xacro urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Visualise
ros2 launch robot_description display.launch.py
```

## Resources

- [URDF XML specification](https://wiki.ros.org/urdf/XML)
- [Xacro documentation](https://wiki.ros.org/xacro)
- [REP-103 coordinate frames](https://www.ros.org/reps/rep-0103.html)
- [REP-120 mobile robot conventions](https://www.ros.org/reps/rep-0120.html)
