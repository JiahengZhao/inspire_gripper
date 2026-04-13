# Inspire EG2-4X2 ROS2 Driver — Design

**Target:** ROS2 Jazzy, C++17, Ubuntu 24.04.
**Hardware:** Inspire-Robots Electric Gripper EG2-4X2 (EG2-4B2 / EG2-4C2).
**Wire:** RS485, Inspire custom serial protocol (frame header `0xEB 0x90`), 115200-8N1.
**Host connection:** USB-to-RS485 adapter → `/dev/ttyUSB*`.

## Scope

- One ROS2 package, `inspire_hand`.
- `ros2_control` hardware system plugin owning one RS485 bus and up to N grippers (≥ 2 supported) distinguished by bus ID.
- Controlled by `parallel_gripper_action_controller/GripperActionController` (position + effort command interfaces).
- Companion node for services, diagnostics, and state broadcasting.
- Unit tests for protocol framing; opt-in hardware integration test.

Out of scope: Modbus RTU backend, CAN backend, dexterous-hand compatibility commands (listed in manual §2.4.13 but use different semantics).

## Package layout

```
inspire_hand/
├── include/inspire_hand/
│   ├── protocol.hpp          # frame build/parse, checksum, Cmd enum, error types
│   ├── serial_bus.hpp        # termios-based RS485 bus, mutex-guarded transact()
│   └── gripper_system.hpp    # ros2_control SystemInterface plugin
├── src/
│   ├── protocol.cpp
│   ├── serial_bus.cpp
│   ├── gripper_system.cpp
│   └── services.cpp          # services + diagnostics owned by the plugin
├── urdf/
│   └── inspire_eg2.ros2_control.xacro
├── config/
│   └── gripper_controllers.yaml
├── launch/
│   └── inspire_hand.launch.py
├── test/
│   ├── test_protocol.cpp
│   └── test_integration.cpp  # opt-in, hardware required
├── plugin_description.xml
├── CMakeLists.txt
└── package.xml
```

## Architecture — three layers

### Layer 1: `protocol` (no I/O, no ROS)

Pure byte-level request/response encoding.

```cpp
enum class Cmd : uint8_t {
  ParaSave     = 0x01,
  ParaIdSet    = 0x04,
  MoveCatchXg  = 0x10,
  MoveRelease  = 0x11,
  SetEgPara    = 0x12,
  ReadEgPara   = 0x13,
  ReadEgState  = 0x14,
  MoveStophere = 0x16,
  ErrorClr     = 0x17,
  MoveCatch2Xg = 0x18,
  ReadEgRun    = 0x41,
  SeekPos      = 0x54,
  ReadActPos   = 0xD9,
};

struct Frame { uint8_t id; Cmd cmd; std::vector<uint8_t> data; };

std::vector<uint8_t> encode(const Frame&);            // prepends EB 90, appends checksum
std::expected<Frame, ParseError> decode(std::span<const uint8_t>);  // validates EE 16 header + checksum + length
```

Checksum: low byte of sum of `[id, len, cmd, data...]` (manual §2.2).

### Layer 2: `serial_bus` (no ROS)

```cpp
class SerialBus {
public:
  SerialBus(const std::string& port, int baud, std::chrono::milliseconds read_timeout);
  ~SerialBus();

  std::expected<Frame, BusError> transact(uint8_t id, Cmd cmd, std::span<const uint8_t> data);

private:
  int fd_;
  std::mutex bus_;
  std::chrono::steady_clock::time_point last_tx_;   // enforce ≥5 ms inter-command gap
};
```

- `cfmakeraw` + 8N1, no parity (USB-to-RS485 adapters are typically half-duplex; driver direction is handled by adapter hardware).
- Drains RX before TX, writes request, reads until frame complete or timeout (default 30 ms; overridden to ≥1 s for `ParaSave` / `ParaIdSet` per manual §2.3.2).
- `BusError` variants: `Timeout`, `Checksum`, `IdMismatch`, `ResponseFlag` (the `0x55` failure flag from §2.3.1), `Io`.

### Layer 3: `gripper_system` (`ros2_control` plugin)

Implements `hardware_interface::SystemInterface`:

- `on_init()` — parses per-joint `gripper_id`, `default_speed` and hardware `port`, `baudrate`, `default_force_threshold`, `read_timeout_ms`, `allow_set_id`.
- `on_configure()` — opens `SerialBus`.
- `on_activate()` — sends `SetEgPara` to initialize ranges (if configured), primes force threshold via `MoveCatchXg` precursor skipped in favor of stored value; reads initial position.
- `read(time, period)` — iterates joints, calls `ReadEgState` (0x14) per gripper → fills position (scaled to meters), effort (grams), derives velocity from Δposition/Δt.
- `write(time, period)` — see command-selection policy below.
- `on_deactivate()` — best-effort `MoveStophere` per joint, close bus.

**Command-selection policy** in `write()`:

| Condition | Command |
|---|---|
| no change since last write | (skip) |
| `effort_cmd > 0` and `target < current` (closing) | `MoveCatchXg(speed, effort)` |
| `effort_cmd > 0` and `target > current` (opening) | `SeekPos(target)` |
| `effort_cmd == 0` or NaN | `SeekPos(target)` |

`speed` comes from URDF per-joint `default_speed` (1..1000). The controller doesn't expose a speed command interface; if per-move speed is needed later, we add a `velocity` command interface.

**Unit conversions:**

- Position: raw 0–1000 ↔ `meters = (raw / 1000.0) * 0.070`. URDF joint limit `[0, 0.070]`.
- Effort: grams, 1:1 with raw. URDF effort limit `1000`. Clamped at hardware boundary: `[50, 1000]`. Values `0 < e < 50` are treated as "effort == 0" (position-only mode) with a throttled warn.

## URDF `<ros2_control>` example

```xml
<ros2_control name="inspire_hand_bus0" type="system">
  <hardware>
    <plugin>inspire_hand/GripperSystem</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="baudrate">115200</param>
    <param name="read_timeout_ms">30</param>
    <param name="default_force_threshold">500</param>
    <param name="allow_set_id">false</param>
  </hardware>

  <joint name="gripper_left">
    <param name="gripper_id">1</param>
    <param name="default_speed">500</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="gripper_right">
    <param name="gripper_id">2</param>
    <param name="default_speed">500</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

Each `<joint>` must also be declared as a URDF `prismatic` joint with `limit lower="0" upper="0.070" effort="1000" velocity="0.05"`.

## Controller config (`gripper_controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50            # Hz — 20 ms period; safe for 2 grippers × ~5 ms round-trip
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    gripper_left_controller:
      type: parallel_gripper_action_controller/GripperActionController
    gripper_right_controller:
      type: parallel_gripper_action_controller/GripperActionController

gripper_left_controller:
  ros__parameters:
    joint: gripper_left
    max_effort: 500.0
    max_velocity: 0.05

gripper_right_controller:
  ros__parameters:
    joint: gripper_right
    max_effort: 500.0
    max_velocity: 0.05
```

## Services & diagnostics

Services and diagnostics are hosted by the hardware plugin itself, not a separate node. In `on_configure()`, the plugin creates an internal `rclcpp::Node` (`inspire_hand_<bus_name>`) and a `rclcpp::executors::SingleThreadedExecutor` that spins on a dedicated thread for the plugin's lifetime. Services and the diagnostics publisher are registered on that node and share the already-open `SerialBus` by direct C++ reference (same process, same plugin instance). The bus mutex serializes service calls against the periodic `read()`/`write()` traffic.

This keeps everything in one process with no IPC, and no second serial handle. Launch brings up only the standard `ros2_control_node`.

### Services (all parameter types `uint8 gripper_id` unless noted)

| Service | Request extras | Effect |
|---|---|---|
| `~/stop` | — | `MoveStophere` |
| `~/clear_fault` | — | `ErrorClr` (0x17) |
| `~/save_params` | — | `ParaSave`; timeout raised to 1.5 s |
| `~/set_id` | `uint8 new_id` | `ParaIdSet`; gated by `allow_set_id` parameter; returns error if disabled |
| `~/calibrate_range` | `uint16 min_raw, uint16 max_raw` | `SetEgPara` |
| `~/set_force_threshold` | `uint16 threshold_g` | stored in-process; applied on next CATCH move |
| `~/read_run_state` | — | returns decoded `ReadEgRun` response |

### Diagnostics (`/diagnostics`, 2 Hz)

Per gripper, one `DiagnosticStatus` named `inspire_hand/gripper_<id>`. Fields:
- `running_state` (manual §2.4.11 codes 0x01..0x06 decoded to strings)
- `error_bits` (locked-rotor, over-temp, over-current, driver-fault, comms-failure)
- `temperature_c`
- `opening_raw`, `force_set_g`
- Counters: `bus_timeouts`, `checksum_errors`, `retries`

Level mapping:
- `error_bits == 0` and state ∈ {idle, moving, gripped} → `OK`.
- State `0x06` ("stopped due to force control during clenching") → `OK` with message "object caught".
- Any `error_bits != 0` → `ERROR`.
- `bus_timeouts` exceeding threshold in rolling window → `WARN`.

## Error handling

- **Read failure in `read()` cycle**: log throttled, hold last good state for that joint, increment `bus_timeouts`. Never throw (would deactivate the whole component).
- **Write failure**: same, next `write()` retries automatically (controller will re-send on next cycle).
- **Value clamp**: position clamped to `[0, 0.070]` m, effort to `[50, 1000]` g (or treated as position-only if < 50). One throttled warn per clamp event.
- **Fault bits**: never auto-cleared. User must call `~/clear_fault`. Per manual §2.4.12, over-temp (bit 1) will not clear until temperature drops below 60 °C.
- **Deactivation**: `on_deactivate` attempts `MoveStophere` on each joint, ignoring errors.

## Testing

### Unit (gtest, no hardware, run in CI)

`test_protocol.cpp`:
- Round-trip: `decode(encode(frame)) == frame` for every `Cmd`.
- Verify checksums against the 12+ worked examples in the manual (§2.4.1 — 2.4.13).
- Malformed frames rejected: bad header, bad checksum, truncated, wrong length byte, unexpected ID.
- Error-flag parsing: response data `0x55` → `ProtocolError::ResponseFlag`.

### Integration (opt-in, hardware required)

Enabled via CMake `-DINSPIRE_HAND_HW_TESTS=ON`. Fixtures require two grippers at IDs 1 and 2 on `/dev/ttyUSB0`.

- Activate plugin; verify `list_hardware_interfaces` shows 10 interfaces (5 per joint).
- Send `GripperCommand` action `position=0.0, max_effort=0` (close, position-only) → state converges < 2 s.
- Send `position=0.035, max_effort=300` → closes to ~0.035 m OR stops early on force (check `run_state` == 0x06 if object present).
- Verify `~/stop`, `~/clear_fault`, `~/read_run_state`.

### Manual checklist (in README)

1. `ros2 launch inspire_hand inspire_hand.launch.py` starts cleanly.
2. `ros2 control list_hardware_interfaces` shows expected interfaces.
3. `ros2 action send_goal /gripper_left_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.035, max_effort: 300.0}}"` succeeds.
4. `ros2 topic echo /diagnostics` reports OK.
5. RViz shows joint motion tracking URDF.

## Timing budget (sanity)

- 50 Hz update → 20 ms period.
- Per gripper round-trip: ~2 ms wire + 5 ms inter-command gap = ~7 ms.
- 2 grippers × (1 read + at most 1 write) = up to 4 transactions/cycle = ~28 ms worst case.
- Margin: if write can be skipped when target unchanged (common at rest), typical cycle is 2 × 7 ms = 14 ms. Fits 20 ms budget.
- If expanded to > 2 grippers, reduce `update_rate` proportionally.

## Open follow-ups (deferred)

- Velocity command interface for per-move speed control.
- Fake/simulated backend for CI integration tests (currently CI runs unit tests only).
- Bringup for > 2 grippers needs a higher-rate bus or slower update loop.
