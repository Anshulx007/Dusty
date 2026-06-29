# robot_description

Robot URDF/Xacro model and RViz visualisation package.

**Current status: skeleton.** No URDF is generated yet. Add your robot
model to `urdf/robot.urdf.xacro` — see `urdf/README.md` for conventions.

---

## Directory layout

```
robot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── display.launch.py      # RViz visualisation launch
├── meshes/                    # STL / DAE visual and collision meshes (add yours here)
├── rviz/                      # RViz configuration files (add robot.rviz here)
└── urdf/
    └── README.md              # Instructions for authoring the URDF
```

---

## Building

```bash
cd ~/robot_ws
colcon build --packages-select robot_description
source install/setup.bash
```

---

## Visualising (once URDF exists)

```bash
ros2 launch robot_description display.launch.py
```

Optional arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `urdf_file` | `urdf/robot.urdf.xacro` | Path to the model file |
| `use_gui` | `true` | Show `joint_state_publisher_gui` and RViz |
| `rviz_config` | `rviz/robot.rviz` | RViz config (falls back to default if missing) |

---

## Adding meshes

Place STL or DAE files in `meshes/`. Reference them from your Xacro using
the `package://` URI scheme:

```xml
<mesh filename="package://robot_description/meshes/base.stl"/>
```

---

## Integration with tf2 and navigation

When `robot_state_publisher` is running (via `display.launch.py` or the
full bringup), it publishes the `tf2` transform tree derived from the URDF.
Navigation nodes (e.g. Nav2) use this tree for sensor frame transforms.

Add `robot_state_publisher` to `robot_bringup/launch/robot.launch.py` once
the URDF is ready.
