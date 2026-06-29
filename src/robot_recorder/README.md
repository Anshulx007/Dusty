# robot_recorder

Dedicated drive-session recorder node for the robot.

**Current status: stub.** The node compiles and runs, subscribes to all
relevant topics, and responds to start/stop commands — but does not write
any data to disk. Implement the `TODO` sections in `recorder_node.py` to
add actual recording.

---

## Why a separate recorder?

`ros2 bag record` captures raw ROS 2 messages. This package is for a
**robot-aware** recording format that:

- Stores `DriveCmd` sequences with relative timestamps (matching the
  original `path_buffer` format from `app.py`).
- Can be triggered by a topic command rather than a terminal flag.
- Supports playback triggering via a ROS 2 topic (future).
- Decouples recording from `teleop_node` so both can evolve independently.

---

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/drive_cmd` | `robot_msgs/DriveCmd` | Subscribe |
| `/arduino_status` | `robot_msgs/ArduinoStatus` | Subscribe |
| `/record_cmd` | `std_msgs/Bool` | Subscribe — `True`=start, `False`=stop |
| `/recording_state` | `std_msgs/Bool` | Publish — current recording state |

---

## Parameters

See `config/recorder_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_duration_sec` | `0.0` | Auto-stop after N seconds (0 = unlimited) |
| `record_speed_limit` | `0.0` | Clamp speed during recording (0 = no limit) |
| `output_dir` | `""` | Directory for saved sessions (empty = disabled) |
| `output_prefix` | `"robot_session"` | Filename prefix |

---

## Building & Running

```bash
cd ~/robot_ws
colcon build --packages-select robot_msgs robot_recorder
source install/setup.bash

# Run the stub
ros2 launch robot_recorder recorder.launch.py

# In another terminal — start recording
ros2 topic pub --once /record_cmd std_msgs/msg/Bool '{data: true}'

# Stop recording
ros2 topic pub --once /record_cmd std_msgs/msg/Bool '{data: false}'

# Monitor recording state
ros2 topic echo /recording_state
```

---

## Implementing recording

Edit `robot_recorder/recorder_node.py`. The key method is `_drive_cmd_cb`:

```python
def _drive_cmd_cb(self, msg: DriveCmd) -> None:
    if not self._recording:
        return
    now = time.monotonic()
    elapsed = now - self._session_start
    self._path_buffer.append((elapsed, msg))  # add this line
    self._last_cmd_time = now
```

Then in `_stop_recording`, serialise `self._path_buffer` to disk using
JSON, SQLite, or a custom binary format.

---

## Planned features

- File output with ISO 8601 timestamped filenames
- JSON serialisation of `DriveCmd` sequences
- `/playback_cmd` publisher to trigger autonomous replay via `navigation_node`
- Session metadata (duration, command count, max speed reached)
- Speed clamping during recording (equivalent to `RECORD_SPEED_LIMIT` in `app.py`)
