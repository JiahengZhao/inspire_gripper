# inspire_hand

ROS2 Jazzy driver for the Inspire-Robots EG2-4X2 electric gripper.

Exposes a `ros2_control` `SystemInterface` plugin that speaks the Inspire custom serial protocol over USB-to-RS485. Integrates with the sibling `inspire_description` package for the gripper URDF (dual-gripper, revolute 4-bar parallel, left and right instances on a single bus).

## Architecture

Three layers:
- `protocol` — pure C++: frame encode/decode, checksum, command builders, command-classification helper (`is_write_cmd`).
- `serial_bus` — Linux termios RS485 I/O. Mutex-guarded `transact(Frame, timeout)` with ≥ 5 ms inter-command gap.
- `gripper_system` — the plugin. Owns one `SerialBus` and one or more `JointState`s. Hosts an internal `rclcpp::Node` for services and diagnostics on its own executor thread.

Joint mapping: hardware raw 0..1000 (0 = closed, 1000 = fully open 70 mm) inverts to URDF joint 0..0.8663 rad (0 rad = open as modeled, 0.8663 rad = closed). Action commands use the URDF value — `position: 0.0` opens, `position: 0.8663` closes, linear in between.

Controllers are `parallel_gripper_action_controller/GripperActionController`, which expose `control_msgs/action/ParallelGripperCommand`. The goal's `command` field is a `sensor_msgs/JointState`, so `name`, `position`, and `effort` are parallel arrays — use a single-element array with the joint name.

Multi-gripper support: the `inspire_description/urdf/inspire_gripper.urdf.xacro` macro accepts a `prefix` parameter (e.g. `"left_"` or `"right_"`) to namespace all links and joints. Each gripper on the bus gets a unique `gripper_id` parameter in its `<joint>` block inside `<ros2_control>`.

## Quick start

1. Connect the USB-to-RS485 adapter and verify the device path, e.g. `/dev/ttyUSB0`.
2. Ensure your user has access: `sudo usermod -aG dialout $USER` (logout/login required).
3. Each gripper on the bus needs a unique ID (factory default ID = 1 and 2 for left/right). To change, see the `set_id` service below.
4. Build and source:
   ```bash
   colcon build --packages-select inspire_description inspire_hand --symlink-install
   source install/setup.bash
   ```
5. Launch:
   ```bash
   ros2 launch inspire_hand inspire_hand.launch.py port:=/dev/ttyUSB0 left_gripper_id:=1 right_gripper_id:=2
   ```

## Verify

```bash
# Exported interfaces:
ros2 control list_hardware_interfaces

# Close the LEFT gripper fully with 300 g max grip force:
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['left_gripper_joint1'], position: [0.8663], effort: [300.0]}}"

# Open LEFT gripper fully:
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['left_gripper_joint1'], position: [0.0], effort: [0.0]}}"

# Close the RIGHT gripper fully with 300 g max grip force:
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['right_gripper_joint1'], position: [0.8663], effort: [300.0]}}"

# Open RIGHT gripper fully:
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['right_gripper_joint1'], position: [0.0], effort: [0.0]}}"

# Watch diagnostics:
ros2 topic echo /diagnostics

# Read run state of left gripper (id=1):
ros2 service call /inspire_hand/inspire_hand_bus0/read_run_state \
  inspire_hand/srv/ReadRunState "{gripper_id: 1}"

# Emergency stop:
ros2 service call /inspire_hand/inspire_hand_bus0/stop \
  inspire_hand/srv/Stop "{gripper_id: 1}"
```

## Services

All under `/inspire_hand/inspire_hand_bus0/`:

| Service | Purpose |
|---|---|
| `stop` | emergency stop (`MoveStophere`) |
| `clear_fault` | clear clearable faults (`ErrorClr`) |
| `save_params` | persist params to flash (`ParaSave`), 1.5 s timeout |
| `set_id` | change a gripper's ID; requires `allow_set_id:=true` |
| `calibrate_range` | set min/max raw opening (`SetEgPara`) |
| `set_force_threshold` | runtime force threshold override (grams, 50..1000) |
| `read_run_state` | decoded running state, error bits, temperature, opening, force |

## Launch arguments

| Arg | Default |
|---|---|
| `port` | `/dev/ttyUSB0` |
| `baudrate` | `115200` |
| `left_gripper_id` | `1` |
| `right_gripper_id` | `2` |

## URDF `<hardware>` parameters

| Param | Default | Notes |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device |
| `baudrate` | `115200` | must match the gripper firmware |
| `read_timeout_ms` | `30` | per-transaction timeout |
| `default_force_threshold` | `500` | grams, applied on activate |
| `allow_set_id` | `false` | must be `true` for `set_id` |

Per-joint parameters (one block per gripper):

| Joint name | Param | Default | Notes |
|---|---|---|---|
| `left_gripper_joint1` | `gripper_id` | `1` | unique on the bus |
| `left_gripper_joint1` | `default_speed` | `500` | 1..1000 |
| `right_gripper_joint1` | `gripper_id` | `2` | unique on the bus |
| `right_gripper_joint1` | `default_speed` | `500` | 1..1000 |

### Velocity override

The `velocity` command interface lets a controller override the per-move speed at runtime. Units are the hardware's raw 1..1000 (not rad/s — the gripper's internal speed control doesn't map linearly). Leave it NaN or ≤ 0 to fall back to the URDF `default_speed` per-joint parameter. Note that only `MoveCatchXg` (closing with force threshold) uses the commanded speed on the wire; `SeekPos` uses its own internal speed.

## Manual smoke test checklist

Run with hardware attached:

- [x] `ros2 launch inspire_hand inspire_hand.launch.py port:=/dev/ttyUSB0 left_gripper_id:=1 right_gripper_id:=2` starts cleanly; no "Failed to open" in logs.
- [ ] `ros2 control list_hardware_interfaces` shows 6 command interfaces (`left_gripper_joint1/position`, `left_gripper_joint1/effort`, `left_gripper_joint1/velocity`, `right_gripper_joint1/position`, `right_gripper_joint1/effort`, `right_gripper_joint1/velocity`) and 6 state interfaces (position/velocity/effort for each joint).
- [ ] Closing left gripper: `ParallelGripperCommand` to `/left_gripper_controller/gripper_cmd` with `position=[0.8663], effort=[300.0]` moves only the left gripper; action server reports succeeded.
- [ ] Opening left gripper: action with `position=[0.0], effort=[0.0]` reopens it fully.
- [ ] Closing right gripper: `ParallelGripperCommand` to `/right_gripper_controller/gripper_cmd` with `position=[0.8663], effort=[300.0]` moves only the right gripper independently.
- [ ] Opening right gripper: action with `position=[0.0], effort=[0.0]` reopens it fully.
- [ ] Both grippers can be commanded simultaneously without interference.
- [ ] `ros2 topic echo /diagnostics` publishes an `inspire_hand/gripper_1` and `inspire_hand/gripper_2` status at ~2 Hz with level OK.
- [ ] `read_run_state` returns a plausible temperature and opening for each gripper ID.
- [ ] `stop` halts a moving gripper immediately.

## Troubleshooting

- **Failed to open port**: check `ls -l /dev/ttyUSB0`, dialout group membership.
- **All reads time out**: confirm baud (factory 115200), A+/B- wiring, and gripper IDs. Try the `read_run_state` service directly.
- **Fault stuck on**: bit 1 (over-temp) self-clears below 60 °C; others clear via `clear_fault`. Persistent faults indicate hardware failure.
- **Gripper moves opposite direction**: the URDF↔hardware inversion is by design (raw 1000 = open = joint 0 rad). Match your action goal positions to the URDF joint limits.
- **ID conflicts**: both grippers must have unique IDs on the RS485 bus. Use `set_id` service with `allow_set_id:=true` to reassign.

## Tests

```bash
colcon test --packages-select inspire_hand --event-handlers console_direct+
```

Runs 23 unit tests for the protocol layer (no hardware needed).

Optional hardware-in-the-loop tests:
```bash
colcon build --packages-select inspire_hand --symlink-install \
  --cmake-args -DINSPIRE_HAND_HW_TESTS=ON
colcon test --packages-select inspire_hand --ctest-args -R test_integration \
  --event-handlers console_direct+
```

Requires the physical gripper on `$INSPIRE_HAND_PORT` (default `/dev/ttyUSB0`) with IDs 1 and 2.
