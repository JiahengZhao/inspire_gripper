# inspire_hand

ROS2 Jazzy driver for the Inspire-Robots EG2-4X2 electric gripper.

Exposes a `ros2_control` `SystemInterface` plugin that speaks the Inspire custom serial protocol over USB-to-RS485. Integrates with the sibling `inspire_description` package for the gripper URDF (single-gripper, revolute 4-bar parallel).

## Architecture

Three layers:
- `protocol` — pure C++: frame encode/decode, checksum, command builders, command-classification helper (`is_write_cmd`).
- `serial_bus` — Linux termios RS485 I/O. Mutex-guarded `transact(Frame, timeout)` with ≥ 5 ms inter-command gap.
- `gripper_system` — the plugin. Owns one `SerialBus` and one or more `JointState`s. Hosts an internal `rclcpp::Node` for services and diagnostics on its own executor thread.

Joint mapping: hardware raw 0..1000 (0 = closed, 1000 = fully open 70 mm) inverts to URDF joint 0..0.8663 rad (0 rad = open as modeled, 0.8663 rad = closed).

Currently single-gripper. Multi-gripper support requires parameterising `inspire_description/urdf/inspire_gripper.urdf.xacro` with a link/joint prefix and adding a second `<joint>` block inside `<ros2_control>`.

## Quick start

1. Connect the USB-to-RS485 adapter and verify the device path, e.g. `/dev/ttyUSB0`.
2. Ensure your user has access: `sudo usermod -aG dialout $USER` (logout/login required).
3. Each gripper on the bus needs a unique ID (default factory ID = 1). To change, see the `set_id` service below.
4. Build and source:
   ```bash
   colcon build --packages-select inspire_description inspire_hand --symlink-install
   source install/setup.bash
   ```
5. Launch:
   ```bash
   ros2 launch inspire_hand inspire_hand.launch.py port:=/dev/ttyUSB0
   ```

## Verify

```bash
# Exported interfaces:
ros2 control list_hardware_interfaces

# Close the gripper fully with 300 g max grip force:
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.8663, max_effort: 300.0}}"

# Open fully:
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 0.0}}"

# Watch diagnostics:
ros2 topic echo /diagnostics

# Read run state of gripper id=1:
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
| `gripper_id` | `1` |

## URDF `<hardware>` parameters

| Param | Default | Notes |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device |
| `baudrate` | `115200` | must match the gripper firmware |
| `read_timeout_ms` | `30` | per-transaction timeout |
| `default_force_threshold` | `500` | grams, applied on activate |
| `allow_set_id` | `false` | must be `true` for `set_id` |

Per-joint parameters:

| Param | Default | Notes |
|---|---|---|
| `gripper_id` | `1` | unique on the bus |
| `default_speed` | `500` | 1..1000 |

## Manual smoke test checklist

Run with hardware attached:

- [ ] `ros2 launch inspire_hand inspire_hand.launch.py` starts cleanly; no "Failed to open" in logs.
- [ ] `ros2 control list_hardware_interfaces` shows 2 command interfaces (`gripper_joint1/position`, `gripper_joint1/effort`) and 3 state interfaces (`gripper_joint1/position`, `.../velocity`, `.../effort`).
- [ ] Closing action `position=0.8663, max_effort=300` moves the gripper and the action server reports succeeded.
- [ ] Opening action `position=0.0, max_effort=0` reopens it fully.
- [ ] `ros2 topic echo /diagnostics` publishes an `inspire_hand/gripper_1` status at ~2 Hz with level OK.
- [ ] `read_run_state` returns a plausible temperature and opening.
- [ ] `stop` halts a moving gripper immediately.

## Troubleshooting

- **Failed to open port**: check `ls -l /dev/ttyUSB0`, dialout group membership.
- **All reads time out**: confirm baud (factory 115200), A+/B- wiring, and gripper ID. Try the `read_run_state` service directly.
- **Fault stuck on**: bit 1 (over-temp) self-clears below 60 °C; others clear via `clear_fault`. Persistent faults indicate hardware failure.
- **Gripper moves opposite direction**: the URDF↔hardware inversion is by design (raw 1000 = open = joint 0 rad). Match your action goal positions to the URDF joint limits.

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

Requires the physical gripper on `$INSPIRE_HAND_PORT` (default `/dev/ttyUSB0`) with ID 1.
