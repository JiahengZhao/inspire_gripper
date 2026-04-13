# Inspire EG2-4X2 ROS2 Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a `ros2_control` hardware plugin in C++ for the Inspire-Robots EG2-4X2 electric gripper over USB-to-RS485, supporting ≥ 2 grippers on one bus with services and diagnostics.

**Architecture:** Three internal layers — a pure-C++ `protocol` library (frame encode/decode/checksum), a `serial_bus` layer (termios RS485, mutex, inter-command spacing), and a `gripper_system` `hardware_interface::SystemInterface` plugin that owns the bus and N joints. Services + diagnostics are hosted by an internal `rclcpp::Node` spun by the plugin on its own thread, sharing the `SerialBus` by C++ reference.

**Tech Stack:** ROS2 Jazzy, C++17, ament_cmake, gtest, pluginlib, hardware_interface, parallel_gripper_action_controller, Linux termios.

---

## Reference — manual worked examples (used in tests)

From `INSPIRE-ROBOTS-ELECTRIC-GRIPPER-USER-MANUAL.pdf`:

| Command | Instruction bytes | Response bytes |
|---|---|---|
| ParaSave (id=1) | `EB 90 01 01 01 03` | `EE 16 01 02 01 01 05` |
| ParaIdSet (id=1, new=3) | `EB 90 01 02 04 03 0A` | `EE 16 01 02 04 01 08` |
| MoveCatchXg (id=1, spd=500, thr=100) | `EB 90 01 05 10 F4 01 64 00 6F` | `EE 16 01 02 10 01 14` |
| MoveCatch2Xg (id=1, spd=500, thr=100) | `EB 90 01 05 18 F4 01 64 00 77` | `EE 16 01 02 18 01 1C` |
| MoveRelease (id=1, spd=500) | `EB 90 01 03 11 F4 01 0A` | `EE 16 01 02 11 01 15` |
| SeekPos (id=1, pos=500) | `EB 90 01 03 54 F4 01 4D` | `EE 16 01 02 54 01 58` |
| MoveStophere (id=1) | `EB 90 01 01 16 18` | `EE 16 01 02 16 01 1A` |
| SetEgPara (id=1, max=1000, min=112) | `EB 90 01 05 12 E8 03 70 00 73` | `EE 16 01 02 12 01 16` |
| ReadEgPara (id=1) | `EB 90 01 01 13 15` | `EE 16 01 05 13 E8 03 70 00 74` |
| ReadActPos (id=1) | `EB 90 01 01 D9 DB` | `EE 16 01 03 D9 F1 01 CF` |
| ReadEgRun (id=1) | `EB 90 01 01 41 43` | `EE 16 01 08 41 01 00 23 E9 03 64 00 BD` |
| ErrorClr (id=1) | `EB 90 01 01 17 19` | `EE 16 01 02 17 01 1B` |

Running-state codes (manual §2.4.11): `0x01` unclenched/idle, `0x02` clenched/idle, `0x03` stopped/idle, `0x04` clenching, `0x05` unclenching, `0x06` stopped due to force control.
Error bits: bit0 locked-rotor, bit1 over-temp, bit2 over-current, bit3 driver-fault, bit4 internal-comms-failure.

---

## File Structure

```
inspire/
├── src/inspire_hand/
│   ├── include/inspire_hand/
│   │   ├── protocol.hpp        # Cmd enum, Frame, encode/decode, checksum
│   │   ├── serial_bus.hpp      # SerialBus class
│   │   └── gripper_system.hpp  # GripperSystem plugin class
│   ├── src/
│   │   ├── protocol.cpp
│   │   ├── serial_bus.cpp
│   │   ├── gripper_system.cpp
│   │   └── services.cpp        # service handlers + diagnostics publisher
│   ├── srv/
│   │   ├── Stop.srv
│   │   ├── ClearFault.srv
│   │   ├── SaveParams.srv
│   │   ├── SetId.srv
│   │   ├── CalibrateRange.srv
│   │   ├── SetForceThreshold.srv
│   │   └── ReadRunState.srv
│   ├── urdf/
│   │   └── inspire_eg2.ros2_control.xacro
│   ├── config/
│   │   └── gripper_controllers.yaml
│   ├── launch/
│   │   └── inspire_hand.launch.py
│   ├── test/
│   │   ├── test_protocol.cpp
│   │   └── test_integration.cpp  # opt-in, hw required
│   ├── plugin_description.xml
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
├── docs/superpowers/
│   ├── specs/2026-04-13-inspire-hand-ros2-driver-design.md
│   └── plans/2026-04-13-inspire-hand-ros2-driver.md (this file)
└── INSPIRE-ROBOTS-ELECTRIC-GRIPPER-USER-MANUAL.pdf
```

File responsibilities:
- `protocol.{hpp,cpp}` — no I/O, no ROS. Byte-level framing.
- `serial_bus.{hpp,cpp}` — Linux termios RS485 I/O; single mutex; inter-command gap.
- `gripper_system.{hpp,cpp}` — `SystemInterface` plugin. Owns `SerialBus`, N joints, internal node/executor for services.
- `services.cpp` — service handlers and diagnostics publisher, compiled into the plugin library.

---

## Task 0: Initialize repository and ROS2 package skeleton

**Files:**
- Create: `.gitignore`, `src/inspire_hand/package.xml`, `src/inspire_hand/CMakeLists.txt`, `src/inspire_hand/plugin_description.xml`

- [ ] **Step 1: Initialize git and source env**

Run:
```bash
cd /home/joe/Documents/Work/anitron/inspire
git init
git add INSPIRE-ROBOTS-ELECTRIC-GRIPPER-USER-MANUAL.pdf docs/
git commit -m "docs: add gripper manual and design/plan"
source /opt/ros/jazzy/setup.bash
```
Expected: commit created; `echo $ROS_DISTRO` prints `jazzy`.

- [ ] **Step 2: Create `.gitignore`**

Write `/home/joe/Documents/Work/anitron/inspire/.gitignore`:
```
build/
install/
log/
*.swp
.cache/
compile_commands.json
```

- [ ] **Step 3: Create `package.xml`**

Write `src/inspire_hand/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>inspire_hand</name>
  <version>0.1.0</version>
  <description>ROS2 Jazzy driver for Inspire-Robots EG2-4X2 electric gripper over USB-to-RS485.</description>
  <maintainer email="joe@anitron.local">joe</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>diagnostic_msgs</depend>
  <depend>std_msgs</depend>

  <exec_depend>controller_manager</exec_depend>
  <exec_depend>parallel_gripper_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <hardware_interface plugin="${prefix}/plugin_description.xml"/>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Create `CMakeLists.txt`**

Write `src/inspire_hand/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
project(inspire_hand LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(SRV_FILES
  srv/Stop.srv
  srv/ClearFault.srv
  srv/SaveParams.srv
  srv/SetId.srv
  srv/CalibrateRange.srv
  srv/SetForceThreshold.srv
  srv/ReadRunState.srv
)
rosidl_generate_interfaces(${PROJECT_NAME} ${SRV_FILES})

add_library(${PROJECT_NAME}_plugin SHARED
  src/protocol.cpp
  src/serial_bus.cpp
  src/gripper_system.cpp
  src/services.cpp
)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp rclcpp_lifecycle hardware_interface pluginlib diagnostic_msgs std_msgs)

rosidl_get_typesupport_target(${PROJECT_NAME}_typesupport
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_typesupport})

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle hardware_interface pluginlib diagnostic_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_plugin)

  option(INSPIRE_HAND_HW_TESTS "Build hardware-in-the-loop integration tests" OFF)
  if(INSPIRE_HAND_HW_TESTS)
    ament_add_gtest(test_integration test/test_integration.cpp)
    target_link_libraries(test_integration ${PROJECT_NAME}_plugin)
  endif()
endif()

ament_package()
```

- [ ] **Step 5: Create `plugin_description.xml`**

Write `src/inspire_hand/plugin_description.xml`:
```xml
<library path="inspire_hand_plugin">
  <class name="inspire_hand/GripperSystem"
         type="inspire_hand::GripperSystem"
         base_class_type="hardware_interface::SystemInterface">
    <description>ros2_control SystemInterface for Inspire EG2-4X2 electric grippers (RS485, custom protocol).</description>
  </class>
</library>
```

- [ ] **Step 6: Create empty headers and source stubs so the package builds**

Write `src/inspire_hand/include/inspire_hand/protocol.hpp`:
```cpp
#pragma once
namespace inspire_hand { /* filled in by later tasks */ }
```

Create the same one-line placeholders in `serial_bus.hpp`, `gripper_system.hpp`, and matching empty `.cpp` files under `src/`. Create empty `srv/Stop.srv` ... `srv/ReadRunState.srv` with a single `---` line so rosidl accepts them (they'll be filled in Task 14). Create empty `test/test_protocol.cpp` with `#include <gtest/gtest.h>`.

Create empty placeholder files:
```bash
cd /home/joe/Documents/Work/anitron/inspire/src/inspire_hand
for f in include/inspire_hand/serial_bus.hpp include/inspire_hand/gripper_system.hpp \
         src/protocol.cpp src/serial_bus.cpp src/gripper_system.cpp src/services.cpp; do
  mkdir -p "$(dirname "$f")"
  printf '#pragma once\nnamespace inspire_hand {}\n' > "$f"  # header-style placeholder
done
printf '#include <gtest/gtest.h>\nTEST(Placeholder, Ok){SUCCEED();}\n' > test/test_protocol.cpp
for s in Stop ClearFault SaveParams SetId CalibrateRange SetForceThreshold ReadRunState; do
  printf '%s\n' '---' > "srv/${s}.srv"
done
```

Then strip `#pragma once` from the `.cpp` files (those should not have it). Manually rewrite each `.cpp` stub to just `#include "inspire_hand/protocol.hpp"` (and corresponding header), with empty `namespace inspire_hand {}`.

- [ ] **Step 7: Build to verify scaffold compiles**

Run:
```bash
cd /home/joe/Documents/Work/anitron/inspire
colcon build --packages-select inspire_hand --symlink-install
```
Expected: build succeeds (`Summary: 1 package finished`).

- [ ] **Step 8: Commit**

```bash
git add src/inspire_hand .gitignore
git commit -m "feat(inspire_hand): package skeleton"
```

---

## Task 1: Protocol — checksum helper

**Files:**
- Modify: `src/inspire_hand/include/inspire_hand/protocol.hpp`, `src/inspire_hand/src/protocol.cpp`, `src/inspire_hand/test/test_protocol.cpp`

- [ ] **Step 1: Write failing test for checksum**

Replace `test/test_protocol.cpp` with:
```cpp
#include <gtest/gtest.h>
#include "inspire_hand/protocol.hpp"

using namespace inspire_hand;

TEST(Checksum, SumsBytesLowOrder) {
  // ParaSave id=1: after frame header the bytes are 01 01 01 -> sum 0x03
  const std::vector<uint8_t> body = {0x01, 0x01, 0x01};
  EXPECT_EQ(checksum(body), 0x03);
}

TEST(Checksum, WrapsAt256) {
  // SeekPos id=1 len=3 cmd=54 data F4 01 -> sum = 01+03+54+F4+01 = 14D -> low byte 0x4D
  const std::vector<uint8_t> body = {0x01, 0x03, 0x54, 0xF4, 0x01};
  EXPECT_EQ(checksum(body), 0x4D);
}
```

- [ ] **Step 2: Run and verify failure**

Run:
```bash
colcon build --packages-select inspire_hand --symlink-install
```
Expected: compile error — `checksum` is undeclared.

- [ ] **Step 3: Implement checksum**

Replace `include/inspire_hand/protocol.hpp` with:
```cpp
#pragma once
#include <cstdint>
#include <span>
#include <vector>

namespace inspire_hand {

uint8_t checksum(std::span<const uint8_t> bytes) noexcept;

}  // namespace inspire_hand
```

Replace `src/protocol.cpp` with:
```cpp
#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

uint8_t checksum(std::span<const uint8_t> bytes) noexcept {
  unsigned sum = 0;
  for (uint8_t b : bytes) sum += b;
  return static_cast<uint8_t>(sum & 0xFF);
}

}  // namespace inspire_hand
```

- [ ] **Step 4: Run test — verify pass**

Run:
```bash
colcon build --packages-select inspire_hand --symlink-install
colcon test --packages-select inspire_hand --event-handlers console_direct+
```
Expected: both `Checksum.*` tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(protocol): checksum helper"
```

---

## Task 2: Protocol — Cmd enum and Frame type

**Files:**
- Modify: `src/inspire_hand/include/inspire_hand/protocol.hpp`, `src/inspire_hand/test/test_protocol.cpp`

- [ ] **Step 1: Add test that references the types**

Append to `test/test_protocol.cpp`:
```cpp
TEST(Frame, ConstructsAndEquals) {
  Frame a{0x01, Cmd::SeekPos, {0xF4, 0x01}};
  Frame b{0x01, Cmd::SeekPos, {0xF4, 0x01}};
  EXPECT_EQ(a.id, b.id);
  EXPECT_EQ(a.cmd, b.cmd);
  EXPECT_EQ(a.data, b.data);
}

TEST(Cmd, ValuesMatchManual) {
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ParaSave),     0x01);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ParaIdSet),    0x04);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::MoveCatchXg),  0x10);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::MoveRelease),  0x11);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::SetEgPara),    0x12);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ReadEgPara),   0x13);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ReadEgState),  0x14);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::MoveStophere), 0x16);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ErrorClr),     0x17);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::MoveCatch2Xg), 0x18);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ReadEgRun),    0x41);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::SeekPos),      0x54);
  EXPECT_EQ(static_cast<uint8_t>(Cmd::ReadActPos),   0xD9);
}
```

- [ ] **Step 2: Build — verify failure**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: `Cmd`, `Frame` undeclared.

- [ ] **Step 3: Add types to header**

Insert into `include/inspire_hand/protocol.hpp` after `<vector>`:
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

struct Frame {
  uint8_t id{};
  Cmd cmd{};
  std::vector<uint8_t> data;
};
```

(`checksum` declaration remains below the types.)

- [ ] **Step 4: Build and test — verify pass**

Run: `colcon build --packages-select inspire_hand --symlink-install && colcon test --packages-select inspire_hand --event-handlers console_direct+`
Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(protocol): Cmd enum and Frame type"
```

---

## Task 3: Protocol — encode

**Files:**
- Modify: `protocol.hpp`, `protocol.cpp`, `test_protocol.cpp`

- [ ] **Step 1: Write failing test using manual worked examples**

Append to `test/test_protocol.cpp`:
```cpp
namespace {
std::vector<uint8_t> hex(std::initializer_list<uint8_t> b) { return b; }
}

TEST(Encode, ParaSaveId1) {
  auto out = encode(Frame{0x01, Cmd::ParaSave, {}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x01,0x01,0x03}));
}

TEST(Encode, SeekPosId1Pos500) {
  auto out = encode(Frame{0x01, Cmd::SeekPos, {0xF4, 0x01}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x03,0x54,0xF4,0x01,0x4D}));
}

TEST(Encode, MoveCatchXgId1Speed500Force100) {
  // data: F4 01 (speed 500), 64 00 (force 100)
  auto out = encode(Frame{0x01, Cmd::MoveCatchXg, {0xF4,0x01,0x64,0x00}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x05,0x10,0xF4,0x01,0x64,0x00,0x6F}));
}

TEST(Encode, SetEgParaId1Max1000Min112) {
  auto out = encode(Frame{0x01, Cmd::SetEgPara, {0xE8,0x03,0x70,0x00}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x05,0x12,0xE8,0x03,0x70,0x00,0x73}));
}

TEST(Encode, MoveStophereId1) {
  auto out = encode(Frame{0x01, Cmd::MoveStophere, {}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x01,0x16,0x18}));
}

TEST(Encode, ErrorClrId1) {
  auto out = encode(Frame{0x01, Cmd::ErrorClr, {}});
  EXPECT_EQ(out, hex({0xEB,0x90,0x01,0x01,0x17,0x19}));
}
```

- [ ] **Step 2: Build — verify fail**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: `encode` undeclared.

- [ ] **Step 3: Implement encode**

Add to `include/inspire_hand/protocol.hpp` (before `checksum`):
```cpp
std::vector<uint8_t> encode(const Frame& f);
```

Add to `src/protocol.cpp`:
```cpp
std::vector<uint8_t> encode(const Frame& f) {
  // Frame: EB 90 | id | len | cmd | data... | checksum
  // len = 1 (cmd byte) + data.size()
  const uint8_t len = static_cast<uint8_t>(1 + f.data.size());
  std::vector<uint8_t> out;
  out.reserve(5 + f.data.size());
  out.push_back(0xEB);
  out.push_back(0x90);
  out.push_back(f.id);
  out.push_back(len);
  out.push_back(static_cast<uint8_t>(f.cmd));
  out.insert(out.end(), f.data.begin(), f.data.end());
  // Checksum is over [id, len, cmd, data...] — i.e. out[2..] at this point.
  out.push_back(checksum(std::span<const uint8_t>(out).subspan(2)));
  return out;
}
```

- [ ] **Step 4: Build and test — verify pass**

Run: `colcon build --packages-select inspire_hand --symlink-install && colcon test --packages-select inspire_hand --event-handlers console_direct+`
Expected: all `Encode.*` tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(protocol): encode instruction frames"
```

---

## Task 4: Protocol — decode (happy path)

**Files:**
- Modify: `protocol.hpp`, `protocol.cpp`, `test_protocol.cpp`

- [ ] **Step 1: Write tests for decoding manual worked-example responses**

Append to `test/test_protocol.cpp`:
```cpp
TEST(Decode, ParaSaveResponse) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x02,0x01,0x01,0x05};
  auto r = decode(buf);
  ASSERT_TRUE(r.has_value()) << static_cast<int>(r.error());
  EXPECT_EQ(r->id, 0x01);
  EXPECT_EQ(r->cmd, Cmd::ParaSave);
  ASSERT_EQ(r->data.size(), 1u);
  EXPECT_EQ(r->data[0], 0x01);  // ok flag
}

TEST(Decode, ReadEgRunResponse) {
  // Manual §2.4.11 example: EE 16 01 08 41 01 00 23 E9 03 64 00 BD
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x08,0x41,0x01,0x00,0x23,0xE9,0x03,0x64,0x00,0xBD};
  auto r = decode(buf);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->id, 0x01);
  EXPECT_EQ(r->cmd, Cmd::ReadEgRun);
  ASSERT_EQ(r->data.size(), 7u);
  EXPECT_EQ(r->data[0], 0x01);  // running state
  EXPECT_EQ(r->data[1], 0x00);  // error code
  EXPECT_EQ(r->data[2], 0x23);  // temperature 35°C
}

TEST(Decode, ReadActPosResponse) {
  // EE 16 01 03 D9 F1 01 CF  -> opening 0x01F1 = 497
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x03,0xD9,0xF1,0x01,0xCF};
  auto r = decode(buf);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->cmd, Cmd::ReadActPos);
  ASSERT_EQ(r->data.size(), 2u);
  const uint16_t opening = static_cast<uint16_t>(r->data[0]) |
                           (static_cast<uint16_t>(r->data[1]) << 8);
  EXPECT_EQ(opening, 497);
}
```

- [ ] **Step 2: Build — verify fail**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: `decode`, `expected` undeclared.

- [ ] **Step 3: Add types + decode implementation**

Add to `include/inspire_hand/protocol.hpp` (above encode):
```cpp
#include <expected>

enum class ParseError {
  TooShort,
  BadHeader,
  BadLength,
  BadChecksum,
};

std::expected<Frame, ParseError> decode(std::span<const uint8_t> buf);
```

Add to `src/protocol.cpp`:
```cpp
std::expected<Frame, ParseError> decode(std::span<const uint8_t> buf) {
  if (buf.size() < 6) return std::unexpected(ParseError::TooShort);
  if (buf[0] != 0xEE || buf[1] != 0x16) return std::unexpected(ParseError::BadHeader);
  const uint8_t id   = buf[2];
  const uint8_t len  = buf[3];
  // total frame length = len + 5 (header 2 + id 1 + len 1 + checksum 1)
  if (buf.size() != static_cast<size_t>(len) + 5u) return std::unexpected(ParseError::BadLength);
  if (len < 1) return std::unexpected(ParseError::BadLength);
  const uint8_t cmd_byte = buf[4];
  const size_t data_len = static_cast<size_t>(len) - 1u;
  const uint8_t recv_cs = buf[4 + data_len + 1];

  // Checksum covers id, len, cmd, data...
  auto cs = checksum(buf.subspan(2, 1 + 1 + 1 + data_len));
  if (cs != recv_cs) return std::unexpected(ParseError::BadChecksum);

  Frame f;
  f.id = id;
  f.cmd = static_cast<Cmd>(cmd_byte);
  f.data.assign(buf.begin() + 5, buf.begin() + 5 + data_len);
  return f;
}
```

- [ ] **Step 4: Build and test — verify pass**

Run: `colcon build --packages-select inspire_hand --symlink-install && colcon test --packages-select inspire_hand --event-handlers console_direct+`
Expected: all `Decode.*` tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(protocol): decode response frames"
```

---

## Task 5: Protocol — decode error cases

**Files:**
- Modify: `test_protocol.cpp`

- [ ] **Step 1: Add negative-path tests**

Append to `test/test_protocol.cpp`:
```cpp
TEST(Decode, RejectsShort) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01};
  EXPECT_EQ(decode(buf).error(), ParseError::TooShort);
}

TEST(Decode, RejectsBadHeader) {
  const std::vector<uint8_t> buf = {0xEB,0x90,0x01,0x02,0x01,0x01,0x05};
  EXPECT_EQ(decode(buf).error(), ParseError::BadHeader);
}

TEST(Decode, RejectsBadLength) {
  // length byte claims 5 but buffer has data for 2
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x05,0x01,0x01,0x05};
  EXPECT_EQ(decode(buf).error(), ParseError::BadLength);
}

TEST(Decode, RejectsBadChecksum) {
  // Valid frame but last byte mangled
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x02,0x01,0x01,0xFF};
  EXPECT_EQ(decode(buf).error(), ParseError::BadChecksum);
}
```

- [ ] **Step 2: Build and test — verify pass**

Run: `colcon build --packages-select inspire_hand --symlink-install && colcon test --packages-select inspire_hand --event-handlers console_direct+`
Expected: new tests pass. (Implementation already covers all cases.)

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "test(protocol): negative decode paths"
```

---

## Task 6: Protocol — command builders

**Files:**
- Modify: `protocol.hpp`, `protocol.cpp`, `test_protocol.cpp`

- [ ] **Step 1: Write tests for typed builders**

Append:
```cpp
TEST(Build, SeekPos) {
  auto f = make_seek_pos(0x01, 500);
  EXPECT_EQ(f.cmd, Cmd::SeekPos);
  EXPECT_EQ(encode(f), (std::vector<uint8_t>{0xEB,0x90,0x01,0x03,0x54,0xF4,0x01,0x4D}));
}

TEST(Build, MoveCatchXg) {
  auto f = make_move_catch(0x01, 500, 100);
  EXPECT_EQ(f.cmd, Cmd::MoveCatchXg);
  EXPECT_EQ(encode(f), (std::vector<uint8_t>{0xEB,0x90,0x01,0x05,0x10,0xF4,0x01,0x64,0x00,0x6F}));
}

TEST(Build, MoveRelease) {
  auto f = make_move_release(0x01, 500);
  EXPECT_EQ(encode(f), (std::vector<uint8_t>{0xEB,0x90,0x01,0x03,0x11,0xF4,0x01,0x0A}));
}

TEST(Build, SetEgPara) {
  auto f = make_set_eg_para(0x01, 1000, 112);
  EXPECT_EQ(encode(f), (std::vector<uint8_t>{0xEB,0x90,0x01,0x05,0x12,0xE8,0x03,0x70,0x00,0x73}));
}

TEST(Build, SimpleReads) {
  EXPECT_EQ(encode(make_read_act_pos(0x01)),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x01,0xD9,0xDB}));
  EXPECT_EQ(encode(make_read_eg_run(0x01)),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x01,0x41,0x43}));
  EXPECT_EQ(encode(make_stop(0x01)),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x01,0x16,0x18}));
  EXPECT_EQ(encode(make_clear_fault(0x01)),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x01,0x17,0x19}));
  EXPECT_EQ(encode(make_para_save(0x01)),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x01,0x01,0x03}));
}

TEST(Build, ParaIdSet) {
  auto f = make_para_id_set(0x01, 0x03);
  EXPECT_EQ(encode(f),
            (std::vector<uint8_t>{0xEB,0x90,0x01,0x02,0x04,0x03,0x0A}));
}
```

- [ ] **Step 2: Build — verify fail**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: `make_*` undeclared.

- [ ] **Step 3: Add builders to header + impl**

Append to `include/inspire_hand/protocol.hpp`:
```cpp
Frame make_seek_pos(uint8_t id, uint16_t pos);              // pos in [0,1000]
Frame make_move_catch(uint8_t id, uint16_t speed, uint16_t force_g);
Frame make_move_catch2(uint8_t id, uint16_t speed, uint16_t force_g);
Frame make_move_release(uint8_t id, uint16_t speed);
Frame make_set_eg_para(uint8_t id, uint16_t max_open, uint16_t min_open);
Frame make_read_eg_para(uint8_t id);
Frame make_read_act_pos(uint8_t id);
Frame make_read_eg_state(uint8_t id);
Frame make_read_eg_run(uint8_t id);
Frame make_stop(uint8_t id);
Frame make_clear_fault(uint8_t id);
Frame make_para_save(uint8_t id);
Frame make_para_id_set(uint8_t id, uint8_t new_id);
```

Append to `src/protocol.cpp`:
```cpp
namespace {
std::vector<uint8_t> le16(uint16_t v) {
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF)};
}
}  // namespace

Frame make_seek_pos(uint8_t id, uint16_t pos) {
  return Frame{id, Cmd::SeekPos, le16(pos)};
}
Frame make_move_catch(uint8_t id, uint16_t speed, uint16_t force_g) {
  auto d = le16(speed); auto f = le16(force_g); d.insert(d.end(), f.begin(), f.end());
  return Frame{id, Cmd::MoveCatchXg, d};
}
Frame make_move_catch2(uint8_t id, uint16_t speed, uint16_t force_g) {
  auto d = le16(speed); auto f = le16(force_g); d.insert(d.end(), f.begin(), f.end());
  return Frame{id, Cmd::MoveCatch2Xg, d};
}
Frame make_move_release(uint8_t id, uint16_t speed) {
  return Frame{id, Cmd::MoveRelease, le16(speed)};
}
Frame make_set_eg_para(uint8_t id, uint16_t max_open, uint16_t min_open) {
  auto d = le16(max_open); auto mn = le16(min_open); d.insert(d.end(), mn.begin(), mn.end());
  return Frame{id, Cmd::SetEgPara, d};
}
Frame make_read_eg_para(uint8_t id) { return Frame{id, Cmd::ReadEgPara, {}}; }
Frame make_read_act_pos(uint8_t id) { return Frame{id, Cmd::ReadActPos, {}}; }
Frame make_read_eg_state(uint8_t id){ return Frame{id, Cmd::ReadEgState,{}}; }
Frame make_read_eg_run(uint8_t id)  { return Frame{id, Cmd::ReadEgRun,  {}}; }
Frame make_stop(uint8_t id)         { return Frame{id, Cmd::MoveStophere,{}}; }
Frame make_clear_fault(uint8_t id)  { return Frame{id, Cmd::ErrorClr,   {}}; }
Frame make_para_save(uint8_t id)    { return Frame{id, Cmd::ParaSave,   {}}; }
Frame make_para_id_set(uint8_t id, uint8_t new_id) {
  return Frame{id, Cmd::ParaIdSet, {new_id}};
}
```

- [ ] **Step 4: Build + test — verify pass**

Run: `colcon build --packages-select inspire_hand --symlink-install && colcon test --packages-select inspire_hand --event-handlers console_direct+`
Expected: all `Build.*` pass.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(protocol): typed command builders"
```

---

## Task 7: SerialBus — open/close

**Files:**
- Modify: `include/inspire_hand/serial_bus.hpp`, `src/serial_bus.cpp`

- [ ] **Step 1: Write the class skeleton**

Replace `include/inspire_hand/serial_bus.hpp`:
```cpp
#pragma once
#include <chrono>
#include <expected>
#include <mutex>
#include <span>
#include <string>

#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

enum class BusError {
  OpenFailed,
  Io,
  Timeout,
  Checksum,
  BadHeader,
  BadLength,
  IdMismatch,
  ResponseFlag,   // device returned 0x55 write-failure flag
  NotOpen,
};

class SerialBus {
public:
  SerialBus();
  ~SerialBus();

  SerialBus(const SerialBus&) = delete;
  SerialBus& operator=(const SerialBus&) = delete;

  std::expected<void, BusError> open(const std::string& port, int baud);
  void close() noexcept;
  bool is_open() const noexcept { return fd_ >= 0; }

  // Sends an instruction frame; blocks until response or timeout.
  std::expected<Frame, BusError> transact(const Frame& request,
                                          std::chrono::milliseconds timeout);

private:
  int fd_{-1};
  std::mutex mu_;
  std::chrono::steady_clock::time_point last_tx_{};
  static constexpr std::chrono::milliseconds kMinGap{5};
};

}  // namespace inspire_hand
```

- [ ] **Step 2: Implement open/close (no transact yet)**

Replace `src/serial_bus.cpp`:
```cpp
#include "inspire_hand/serial_bus.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>

namespace inspire_hand {

namespace {
speed_t baud_flag(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B0;
  }
}
}  // namespace

SerialBus::SerialBus() = default;

SerialBus::~SerialBus() { close(); }

std::expected<void, BusError> SerialBus::open(const std::string& port, int baud) {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ >= 0) ::close(fd_);
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return std::unexpected(BusError::OpenFailed);

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    return std::unexpected(BusError::OpenFailed);
  }
  cfmakeraw(&tty);
  const speed_t bf = baud_flag(baud);
  if (bf == B0) { ::close(fd_); fd_ = -1; return std::unexpected(BusError::OpenFailed); }
  cfsetispeed(&tty, bf);
  cfsetospeed(&tty, bf);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;     // 1 stop bit
  tty.c_cflag &= ~PARENB;     // no parity
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    return std::unexpected(BusError::OpenFailed);
  }
  tcflush(fd_, TCIOFLUSH);
  return {};
}

void SerialBus::close() noexcept {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

std::expected<Frame, BusError> SerialBus::transact(const Frame&,
                                                   std::chrono::milliseconds) {
  return std::unexpected(BusError::NotOpen);  // filled in Task 8
}

}  // namespace inspire_hand
```

- [ ] **Step 3: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success.

- [ ] **Step 4: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(serial_bus): open/close scaffolding"
```

---

## Task 8: SerialBus — transact with timeout

**Files:**
- Modify: `src/serial_bus.cpp`

- [ ] **Step 1: Implement transact**

Replace the `transact` stub in `src/serial_bus.cpp` with:
```cpp
std::expected<Frame, BusError> SerialBus::transact(const Frame& req,
                                                   std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ < 0) return std::unexpected(BusError::NotOpen);

  // Enforce minimum inter-command gap.
  const auto now = std::chrono::steady_clock::now();
  const auto gap = now - last_tx_;
  if (gap < kMinGap) std::this_thread::sleep_for(kMinGap - gap);

  tcflush(fd_, TCIFLUSH);

  const auto bytes = encode(req);
  size_t written = 0;
  while (written < bytes.size()) {
    ssize_t n = ::write(fd_, bytes.data() + written, bytes.size() - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      return std::unexpected(BusError::Io);
    }
    written += static_cast<size_t>(n);
  }
  // Ensure bytes have left the UART before switching to RX on half-duplex adapters.
  tcdrain(fd_);
  last_tx_ = std::chrono::steady_clock::now();

  // Read loop: accumulate until a full response frame is available or timeout.
  std::vector<uint8_t> rx;
  rx.reserve(32);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  uint8_t buf[64];

  auto total_needed = [&rx]() -> std::optional<size_t> {
    if (rx.size() < 4) return std::nullopt;
    if (rx[0] != 0xEE || rx[1] != 0x16) return std::nullopt;
    return static_cast<size_t>(rx[3]) + 5u;  // len + header(2) + id(1) + len(1) + cs(1) - len(1) = +5
  };

  while (true) {
    // Drop garbage bytes until we find the response header.
    while (rx.size() >= 2 && (rx[0] != 0xEE || rx[1] != 0x16)) {
      rx.erase(rx.begin());
    }
    if (auto need = total_needed(); need && rx.size() >= *need) break;

    if (std::chrono::steady_clock::now() >= deadline) return std::unexpected(BusError::Timeout);

    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (errno == EINTR) continue;
      return std::unexpected(BusError::Io);
    }
    if (n > 0) rx.insert(rx.end(), buf, buf + n);
  }

  auto decoded = decode(std::span<const uint8_t>(rx).subspan(0, rx[3] + 5u));
  if (!decoded) {
    switch (decoded.error()) {
      case ParseError::BadChecksum: return std::unexpected(BusError::Checksum);
      case ParseError::BadHeader:   return std::unexpected(BusError::BadHeader);
      case ParseError::BadLength:   return std::unexpected(BusError::BadLength);
      case ParseError::TooShort:    return std::unexpected(BusError::Io);
    }
  }
  if (decoded->id != req.id) return std::unexpected(BusError::IdMismatch);
  // For write-type commands the spec returns 0x01=ok / 0x55=fail in data[0].
  // Read-type commands return payload; only flag 0x55 when data.size()==1 && byte==0x55.
  if (decoded->data.size() == 1 && decoded->data[0] == 0x55) {
    return std::unexpected(BusError::ResponseFlag);
  }
  return *decoded;
}
```

Add to the top of `src/serial_bus.cpp` (with other headers):
```cpp
#include <optional>
#include <thread>
```

- [ ] **Step 2: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(serial_bus): transact with timeout + inter-command gap"
```

---

## Task 9: GripperSystem — skeleton + state/command interface export

**Files:**
- Modify: `include/inspire_hand/gripper_system.hpp`, `src/gripper_system.cpp`

- [ ] **Step 1: Write the class definition**

Replace `include/inspire_hand/gripper_system.hpp`:
```cpp
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "inspire_hand/serial_bus.hpp"

namespace inspire_hand {

struct JointState {
  uint8_t gripper_id{1};
  double default_speed{500.0};      // 1..1000
  double pos_cmd{std::numeric_limits<double>::quiet_NaN()};
  double eff_cmd{std::numeric_limits<double>::quiet_NaN()};
  double pos_state{0.0};            // meters
  double vel_state{0.0};            // m/s
  double eff_state{0.0};            // grams
  double last_pos_written{std::numeric_limits<double>::quiet_NaN()};
  double last_eff_written{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point last_read_time{};
  double last_pos_state{0.0};
};

class GripperSystem : public hardware_interface::SystemInterface {
public:
  GripperSystem();
  ~GripperSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

  // Exposed so services.cpp can share the bus + joints.
  SerialBus& bus() { return bus_; }
  std::vector<JointState>& joints() { return joints_; }
  std::mutex& state_mutex() { return state_mu_; }

  static constexpr double kStrokeMeters = 0.070;  // 70 mm full open
  static constexpr uint16_t kMaxRaw     = 1000;

private:
  std::vector<JointState> joints_;
  SerialBus bus_;
  std::mutex state_mu_;

  // Hardware params (URDF <hardware>).
  std::string port_{"/dev/ttyUSB0"};
  int baud_{115200};
  int read_timeout_ms_{30};
  uint16_t default_force_threshold_g_{500};
  bool allow_set_id_{false};

  // Internal services node + thread (set up by services.cpp).
  std::shared_ptr<rclcpp::Node> srv_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr srv_exec_;
  std::thread srv_thread_;
  std::atomic<bool> srv_stop_{false};

  void start_services();
  void stop_services();

  // Helpers
  double raw_to_meters(uint16_t raw) const { return (static_cast<double>(raw) / kMaxRaw) * kStrokeMeters; }
  uint16_t meters_to_raw(double m) const {
    const double r = (m / kStrokeMeters) * kMaxRaw;
    if (r < 0.0) return 0;
    if (r > kMaxRaw) return kMaxRaw;
    return static_cast<uint16_t>(r + 0.5);
  }
};

}  // namespace inspire_hand
```

- [ ] **Step 2: Implement on_init and interface export**

Replace `src/gripper_system.cpp`:
```cpp
#include "inspire_hand/gripper_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace inspire_hand {

using CallbackReturn = hardware_interface::CallbackReturn;

GripperSystem::GripperSystem() = default;
GripperSystem::~GripperSystem() { stop_services(); bus_.close(); }

CallbackReturn GripperSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  auto get = [&](const auto& map, const std::string& key, const std::string& fallback) {
    auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
  };

  port_                        = get(info.hardware_parameters, "port", "/dev/ttyUSB0");
  baud_                        = std::stoi(get(info.hardware_parameters, "baudrate", "115200"));
  read_timeout_ms_             = std::stoi(get(info.hardware_parameters, "read_timeout_ms", "30"));
  default_force_threshold_g_   = static_cast<uint16_t>(std::stoi(
      get(info.hardware_parameters, "default_force_threshold", "500")));
  allow_set_id_                = (get(info.hardware_parameters, "allow_set_id", "false") == "true");

  joints_.clear();
  for (const auto& j : info.joints) {
    JointState js;
    js.gripper_id    = static_cast<uint8_t>(std::stoi(get(j.parameters, "gripper_id", "1")));
    js.default_speed = std::stod(get(j.parameters, "default_speed", "500"));

    // Sanity on declared interfaces.
    bool has_pos_cmd=false, has_eff_cmd=false;
    for (auto& ci : j.command_interfaces) {
      if (ci.name == "position") has_pos_cmd = true;
      if (ci.name == "effort")   has_eff_cmd = true;
    }
    if (!has_pos_cmd) {
      RCLCPP_ERROR(rclcpp::get_logger("inspire_hand"),
                   "Joint '%s' must declare a 'position' command interface.", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!has_eff_cmd) {
      RCLCPP_WARN(rclcpp::get_logger("inspire_hand"),
                  "Joint '%s' has no 'effort' command interface; force threshold will use default.",
                  j.name.c_str());
    }
    joints_.push_back(js);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& name = info_.joints[i].name;
    out.emplace_back(name, "position", &joints_[i].pos_state);
    out.emplace_back(name, "velocity", &joints_[i].vel_state);
    out.emplace_back(name, "effort",   &joints_[i].eff_state);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> GripperSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& name = info_.joints[i].name;
    out.emplace_back(name, "position", &joints_[i].pos_cmd);
    // Effort command interface is optional; always export so controllers can attach.
    out.emplace_back(name, "effort",   &joints_[i].eff_cmd);
  }
  return out;
}

CallbackReturn GripperSystem::on_configure(const rclcpp_lifecycle::State&) {
  auto r = bus_.open(port_, baud_);
  if (!r) {
    RCLCPP_ERROR(rclcpp::get_logger("inspire_hand"),
                 "Failed to open %s @ %d", port_.c_str(), baud_);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_activate(const rclcpp_lifecycle::State&) {
  // Seed initial state by reading current position.
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  for (auto& j : joints_) {
    auto resp = bus_.transact(make_read_act_pos(j.gripper_id), timeout);
    if (resp && resp->data.size() == 2) {
      const uint16_t raw = static_cast<uint16_t>(resp->data[0]) |
                           (static_cast<uint16_t>(resp->data[1]) << 8);
      j.pos_state = raw_to_meters(raw);
    }
    j.pos_cmd           = j.pos_state;
    j.eff_cmd           = static_cast<double>(default_force_threshold_g_);
    j.last_pos_written  = std::numeric_limits<double>::quiet_NaN();
    j.last_eff_written  = std::numeric_limits<double>::quiet_NaN();
    j.last_read_time    = std::chrono::steady_clock::now();
    j.last_pos_state    = j.pos_state;
  }
  start_services();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  stop_services();
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  for (auto& j : joints_) (void)bus_.transact(make_stop(j.gripper_id), timeout);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_cleanup(const rclcpp_lifecycle::State&) {
  bus_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
  // Filled in Task 10.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
  // Filled in Task 11.
  return hardware_interface::return_type::OK;
}

void GripperSystem::start_services() { /* Task 13 */ }
void GripperSystem::stop_services()  { /* Task 13 */ }

}  // namespace inspire_hand

PLUGINLIB_EXPORT_CLASS(inspire_hand::GripperSystem, hardware_interface::SystemInterface)
```

- [ ] **Step 2b: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(gripper_system): plugin skeleton with lifecycle and interface exports"
```

---

## Task 10: GripperSystem — read() via ReadEgState

**Files:**
- Modify: `src/gripper_system.cpp`

- [ ] **Step 1: Implement read()**

Replace the `read()` stub in `src/gripper_system.cpp` with:
```cpp
hardware_interface::return_type GripperSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> g(state_mu_);
  for (auto& j : joints_) {
    // ReadEgState (0x14) returns: opening(2) + current force(2) + set threshold(2) = 6 bytes.
    auto resp = bus_.transact(make_read_eg_state(j.gripper_id), timeout);
    if (!resp) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *rclcpp::Clock::make_shared(), 2000,
        "read gripper %u failed (err=%d)", j.gripper_id, static_cast<int>(resp.error()));
      continue;  // hold last state
    }
    if (resp->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *rclcpp::Clock::make_shared(), 2000,
        "short ReadEgState response for id %u", j.gripper_id);
      continue;
    }
    const uint16_t opening = static_cast<uint16_t>(resp->data[0]) | (static_cast<uint16_t>(resp->data[1]) << 8);
    const uint16_t force   = static_cast<uint16_t>(resp->data[2]) | (static_cast<uint16_t>(resp->data[3]) << 8);
    const double pos_m = raw_to_meters(opening);
    const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - j.last_read_time).count();
    j.vel_state = (dt > 1e-3) ? (pos_m - j.last_pos_state) / dt : 0.0;
    j.last_pos_state = pos_m;
    j.last_read_time = now;
    j.pos_state = pos_m;
    j.eff_state = static_cast<double>(force);
  }
  return hardware_interface::return_type::OK;
}
```

Add at top of file:
```cpp
#include <rclcpp/clock.hpp>
```

- [ ] **Step 2: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(gripper_system): read state via ReadEgState"
```

---

## Task 11: GripperSystem — write() command-selection policy

**Files:**
- Modify: `src/gripper_system.cpp`

- [ ] **Step 1: Implement write()**

Replace the `write()` stub with:
```cpp
hardware_interface::return_type GripperSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);

  std::lock_guard<std::mutex> g(state_mu_);
  for (auto& j : joints_) {
    if (std::isnan(j.pos_cmd)) continue;  // controller not writing yet

    // Clamp position to stroke range.
    const double tgt_m = std::clamp(j.pos_cmd, 0.0, kStrokeMeters);
    const uint16_t tgt_raw = meters_to_raw(tgt_m);

    // Effort semantics: NaN or <50 → position-only; else CATCH on close.
    double eff_g = j.eff_cmd;
    bool use_force = !std::isnan(eff_g) && eff_g >= 50.0;
    if (use_force) eff_g = std::min(eff_g, 1000.0);

    // Skip if nothing changed since the last write.
    const bool pos_same = !std::isnan(j.last_pos_written) &&
                          std::abs(j.last_pos_written - tgt_m) < 1e-4;  // 0.1 mm
    const bool eff_same = !std::isnan(j.last_eff_written) &&
                          std::abs(j.last_eff_written - eff_g) < 1.0;   // 1 gram
    if (pos_same && eff_same) continue;

    const uint16_t speed = static_cast<uint16_t>(std::clamp(j.default_speed, 1.0, 1000.0));
    const double current_m = j.pos_state;

    std::expected<Frame, BusError> rc;
    if (use_force && tgt_m < current_m - 1e-4) {
      rc = bus_.transact(make_move_catch(j.gripper_id, speed, static_cast<uint16_t>(eff_g)),
                         timeout);
    } else {
      rc = bus_.transact(make_seek_pos(j.gripper_id, tgt_raw), timeout);
    }
    if (!rc) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *rclcpp::Clock::make_shared(), 2000,
        "write gripper %u failed (err=%d)", j.gripper_id, static_cast<int>(rc.error()));
      continue;
    }
    j.last_pos_written = tgt_m;
    j.last_eff_written = eff_g;
  }
  return hardware_interface::return_type::OK;
}
```

- [ ] **Step 2: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(gripper_system): write() command-selection policy"
```

---

## Task 12: Service definitions

**Files:**
- Modify: all 7 files in `src/inspire_hand/srv/`

- [ ] **Step 1: Write each srv file**

`srv/Stop.srv`:
```
uint8 gripper_id
---
bool success
string message
```

`srv/ClearFault.srv` — identical body to `Stop.srv`.

`srv/SaveParams.srv` — identical body to `Stop.srv`.

`srv/SetId.srv`:
```
uint8 gripper_id
uint8 new_id
---
bool success
string message
```

`srv/CalibrateRange.srv`:
```
uint8 gripper_id
uint16 min_raw
uint16 max_raw
---
bool success
string message
```

`srv/SetForceThreshold.srv`:
```
uint8 gripper_id
uint16 threshold_g
---
bool success
string message
```

`srv/ReadRunState.srv`:
```
uint8 gripper_id
---
bool success
string message
uint8 running_state
uint8 error_code
uint8 temperature_c
uint16 opening_raw
uint16 force_set_g
```

- [ ] **Step 2: Build (interface generation)**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success; `install/inspire_hand/include/inspire_hand/srv/*.hpp` generated.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(inspire_hand): ROS2 service definitions"
```

---

## Task 13: Services & diagnostics — internal node + handlers

**Files:**
- Modify: `src/services.cpp`, `src/gripper_system.cpp`, `CMakeLists.txt`

- [ ] **Step 1: Implement services.cpp**

Replace `src/services.cpp`:
```cpp
#include "inspire_hand/gripper_system.hpp"
#include "inspire_hand/protocol.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "inspire_hand/srv/stop.hpp"
#include "inspire_hand/srv/clear_fault.hpp"
#include "inspire_hand/srv/save_params.hpp"
#include "inspire_hand/srv/set_id.hpp"
#include "inspire_hand/srv/calibrate_range.hpp"
#include "inspire_hand/srv/set_force_threshold.hpp"
#include "inspire_hand/srv/read_run_state.hpp"

namespace inspire_hand {

namespace {
std::string error_bits_to_string(uint8_t e) {
  if (e == 0) return "none";
  std::string s;
  auto add = [&](const char* n){ if (!s.empty()) s += ","; s += n; };
  if (e & 0x01) add("locked_rotor");
  if (e & 0x02) add("over_temp");
  if (e & 0x04) add("over_current");
  if (e & 0x08) add("driver_fault");
  if (e & 0x10) add("comms_failure");
  return s;
}

const char* run_state_name(uint8_t s) {
  switch (s) {
    case 0x01: return "unclenched_idle";
    case 0x02: return "clenched_idle";
    case 0x03: return "stopped_idle";
    case 0x04: return "clenching";
    case 0x05: return "unclenching";
    case 0x06: return "caught_object";
    default:   return "unknown";
  }
}
}  // namespace

// Stored on the plugin instance so lifetime is tied to the plugin.
struct ServicesState {
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec;
  std::thread thread;
  std::atomic<bool> stop{false};

  rclcpp::Service<inspire_hand::srv::Stop>::SharedPtr               s_stop;
  rclcpp::Service<inspire_hand::srv::ClearFault>::SharedPtr         s_clear;
  rclcpp::Service<inspire_hand::srv::SaveParams>::SharedPtr         s_save;
  rclcpp::Service<inspire_hand::srv::SetId>::SharedPtr              s_setid;
  rclcpp::Service<inspire_hand::srv::CalibrateRange>::SharedPtr     s_cal;
  rclcpp::Service<inspire_hand::srv::SetForceThreshold>::SharedPtr  s_force;
  rclcpp::Service<inspire_hand::srv::ReadRunState>::SharedPtr       s_run;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
  rclcpp::TimerBase::SharedPtr diag_timer;
};

// One state per GripperSystem instance, keyed by pointer.
static std::mutex g_svc_mu;
static std::unordered_map<GripperSystem*, std::unique_ptr<ServicesState>> g_svc;

void GripperSystem::start_services() {
  std::lock_guard<std::mutex> g(g_svc_mu);
  auto st = std::make_unique<ServicesState>();
  const std::string ns = "inspire_hand/bus";
  st->node = std::make_shared<rclcpp::Node>("inspire_hand_bus", ns);
  auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  auto& bus = bus_;
  auto* self = this;

  auto find_joint = [self](uint8_t id) -> JointState* {
    for (auto& j : self->joints()) if (j.gripper_id == id) return &j;
    return nullptr;
  };

  st->s_stop = st->node->create_service<srv::Stop>(
      "stop",
      [&bus, timeout](const std::shared_ptr<srv::Stop::Request> req,
                      std::shared_ptr<srv::Stop::Response> res) {
        auto r = bus.transact(make_stop(req->gripper_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_clear = st->node->create_service<srv::ClearFault>(
      "clear_fault",
      [&bus, timeout](const std::shared_ptr<srv::ClearFault::Request> req,
                      std::shared_ptr<srv::ClearFault::Response> res) {
        auto r = bus.transact(make_clear_fault(req->gripper_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_save = st->node->create_service<srv::SaveParams>(
      "save_params",
      [&bus](const std::shared_ptr<srv::SaveParams::Request> req,
             std::shared_ptr<srv::SaveParams::Response> res) {
        // Flash erase can take ~850ms per manual §2.3.2.
        auto r = bus.transact(make_para_save(req->gripper_id), std::chrono::milliseconds(1500));
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  const bool allow_set_id = allow_set_id_;
  st->s_setid = st->node->create_service<srv::SetId>(
      "set_id",
      [&bus, timeout, allow_set_id](const std::shared_ptr<srv::SetId::Request> req,
                                    std::shared_ptr<srv::SetId::Response> res) {
        if (!allow_set_id) {
          res->success = false; res->message = "disabled (set allow_set_id=true in URDF)";
          return;
        }
        auto r = bus.transact(make_para_id_set(req->gripper_id, req->new_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok; remember to save_params to persist" : "bus error";
      });

  st->s_cal = st->node->create_service<srv::CalibrateRange>(
      "calibrate_range",
      [&bus, timeout](const std::shared_ptr<srv::CalibrateRange::Request> req,
                      std::shared_ptr<srv::CalibrateRange::Response> res) {
        auto r = bus.transact(make_set_eg_para(req->gripper_id, req->max_raw, req->min_raw), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_force = st->node->create_service<srv::SetForceThreshold>(
      "set_force_threshold",
      [self, find_joint](const std::shared_ptr<srv::SetForceThreshold::Request> req,
                         std::shared_ptr<srv::SetForceThreshold::Response> res) {
        std::lock_guard<std::mutex> g(self->state_mutex());
        auto* j = find_joint(req->gripper_id);
        if (!j) { res->success = false; res->message = "unknown gripper_id"; return; }
        if (req->threshold_g < 50 || req->threshold_g > 1000) {
          res->success = false; res->message = "threshold out of range [50,1000]"; return;
        }
        j->eff_cmd = req->threshold_g;
        res->success = true; res->message = "stored; applied on next CATCH move";
      });

  st->s_run = st->node->create_service<srv::ReadRunState>(
      "read_run_state",
      [&bus, timeout](const std::shared_ptr<srv::ReadRunState::Request> req,
                      std::shared_ptr<srv::ReadRunState::Response> res) {
        auto r = bus.transact(make_read_eg_run(req->gripper_id), timeout);
        if (!r || r->data.size() < 7) {
          res->success = false; res->message = "bus error or short response"; return;
        }
        res->success         = true;
        res->message         = run_state_name(r->data[0]);
        res->running_state   = r->data[0];
        res->error_code      = r->data[1];
        res->temperature_c   = r->data[2];
        res->opening_raw     = static_cast<uint16_t>(r->data[3]) | (static_cast<uint16_t>(r->data[4]) << 8);
        res->force_set_g     = static_cast<uint16_t>(r->data[5]) | (static_cast<uint16_t>(r->data[6]) << 8);
      });

  st->diag_pub = st->node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  st->diag_timer = st->node->create_wall_timer(
      std::chrono::milliseconds(500),
      [self, &bus, timeout, pub = st->diag_pub]() {
        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = rclcpp::Clock().now();
        for (auto& j : self->joints()) {
          auto r = bus.transact(make_read_eg_run(j.gripper_id), timeout);
          diagnostic_msgs::msg::DiagnosticStatus s;
          s.name = std::string("inspire_hand/gripper_") + std::to_string(j.gripper_id);
          s.hardware_id = "inspire_eg2_" + std::to_string(j.gripper_id);
          if (!r || r->data.size() < 7) {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            s.message = "bus error";
          } else {
            const uint8_t state = r->data[0];
            const uint8_t err = r->data[1];
            if (err != 0) { s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; s.message = "fault"; }
            else          { s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;    s.message = run_state_name(state); }
            auto kv = [&](const std::string& k, const std::string& v){
              diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; s.values.push_back(p);
            };
            kv("running_state",  run_state_name(state));
            kv("error_bits",     error_bits_to_string(err));
            kv("temperature_c",  std::to_string(r->data[2]));
            kv("opening_raw",    std::to_string(static_cast<uint16_t>(r->data[3]) | (static_cast<uint16_t>(r->data[4]) << 8)));
            kv("force_set_g",    std::to_string(static_cast<uint16_t>(r->data[5]) | (static_cast<uint16_t>(r->data[6]) << 8)));
          }
          arr.status.push_back(std::move(s));
        }
        pub->publish(arr);
      });

  st->exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  st->exec->add_node(st->node);
  st->thread = std::thread([exec = st->exec, stop = &st->stop](){
    while (!stop->load()) exec->spin_some(std::chrono::milliseconds(50));
  });

  g_svc[this] = std::move(st);
}

void GripperSystem::stop_services() {
  std::unique_ptr<ServicesState> st;
  {
    std::lock_guard<std::mutex> g(g_svc_mu);
    auto it = g_svc.find(this);
    if (it == g_svc.end()) return;
    st = std::move(it->second);
    g_svc.erase(it);
  }
  st->stop = true;
  if (st->thread.joinable()) st->thread.join();
  st->exec->remove_node(st->node);
}

}  // namespace inspire_hand
```

Add to top of `src/gripper_system.cpp` after existing includes:
```cpp
#include <unordered_map>
```

- [ ] **Step 2: Build**

Run: `colcon build --packages-select inspire_hand --symlink-install`
Expected: success. If link errors mention missing srv typesupport, verify `CMakeLists.txt` `target_link_libraries(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_typesupport})` is present (it is, from Task 0).

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(inspire_hand): services and diagnostics in internal node"
```

---

## Task 14: URDF + controller config + launch

**Files:**
- Create: `urdf/inspire_eg2.ros2_control.xacro`, `config/gripper_controllers.yaml`, `launch/inspire_hand.launch.py`

- [ ] **Step 1: Write URDF xacro**

Write `src/inspire_hand/urdf/inspire_eg2.ros2_control.xacro`:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="inspire_hand">

  <xacro:arg name="port"                    default="/dev/ttyUSB0"/>
  <xacro:arg name="baudrate"                default="115200"/>
  <xacro:arg name="read_timeout_ms"         default="30"/>
  <xacro:arg name="default_force_threshold" default="500"/>
  <xacro:arg name="allow_set_id"            default="false"/>
  <xacro:arg name="left_gripper_id"         default="1"/>
  <xacro:arg name="right_gripper_id"        default="2"/>

  <link name="world"/>

  <!-- Each gripper is modeled as a single prismatic joint (finger against a virtual base link). -->
  <link name="gripper_left_base"/>
  <link name="gripper_left_finger"/>
  <joint name="gripper_left" type="prismatic">
    <parent link="gripper_left_base"/>
    <child  link="gripper_left_finger"/>
    <axis   xyz="1 0 0"/>
    <limit  lower="0.0" upper="0.070" effort="1000.0" velocity="0.05"/>
    <origin xyz="0 0 0"/>
  </joint>
  <joint name="world_to_left" type="fixed">
    <parent link="world"/><child link="gripper_left_base"/>
  </joint>

  <link name="gripper_right_base"/>
  <link name="gripper_right_finger"/>
  <joint name="gripper_right" type="prismatic">
    <parent link="gripper_right_base"/>
    <child  link="gripper_right_finger"/>
    <axis   xyz="1 0 0"/>
    <limit  lower="0.0" upper="0.070" effort="1000.0" velocity="0.05"/>
    <origin xyz="0 0.1 0"/>
  </joint>
  <joint name="world_to_right" type="fixed">
    <parent link="world"/><child link="gripper_right_base"/>
  </joint>

  <ros2_control name="inspire_hand_bus0" type="system">
    <hardware>
      <plugin>inspire_hand/GripperSystem</plugin>
      <param name="port">$(arg port)</param>
      <param name="baudrate">$(arg baudrate)</param>
      <param name="read_timeout_ms">$(arg read_timeout_ms)</param>
      <param name="default_force_threshold">$(arg default_force_threshold)</param>
      <param name="allow_set_id">$(arg allow_set_id)</param>
    </hardware>

    <joint name="gripper_left">
      <param name="gripper_id">$(arg left_gripper_id)</param>
      <param name="default_speed">500</param>
      <command_interface name="position"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="gripper_right">
      <param name="gripper_id">$(arg right_gripper_id)</param>
      <param name="default_speed">500</param>
      <command_interface name="position"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
</robot>
```

- [ ] **Step 2: Write controller config**

Write `src/inspire_hand/config/gripper_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

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

- [ ] **Step 3: Write launch file**

Write `src/inspire_hand/launch/inspire_hand.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baudrate")
    pkg = FindPackageShare("inspire_hand")

    urdf = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "inspire_eg2.ros2_control.xacro"]),
        " port:=", port,
        " baudrate:=", baud,
    ])

    controllers_yaml = PathJoinSubstitution([pkg, "config", "gripper_controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": urdf}, controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_left_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_right_controller"],
            output="screen",
        ),
    ])
```

- [ ] **Step 4: Build and verify launch file parses**

Run:
```bash
colcon build --packages-select inspire_hand --symlink-install
source install/setup.bash
ros2 launch inspire_hand inspire_hand.launch.py --show-args
```
Expected: shows `port` and `baudrate` launch arguments; no Python errors.

- [ ] **Step 5: Commit**

```bash
git add src/inspire_hand
git commit -m "feat(inspire_hand): URDF, controller config, and launch file"
```

---

## Task 15: Hardware smoke test (manual)

**Files:**
- Create: `src/inspire_hand/README.md`

- [ ] **Step 1: Write README with manual verification checklist**

Write `src/inspire_hand/README.md`:
```markdown
# inspire_hand

ROS2 Jazzy driver for the Inspire-Robots EG2-4X2 electric gripper.

## Quick start

1. Connect the USB-to-RS485 adapter and verify the device path, e.g. `/dev/ttyUSB0`.
2. Ensure your user has access: `sudo usermod -aG dialout $USER` (logout/login required).
3. Grippers must have unique IDs on the bus (default factory ID=1). To change, see `~/set_id` service below.
4. Build and source:
   ```bash
   colcon build --packages-select inspire_hand --symlink-install
   source install/setup.bash
   ```
5. Launch:
   ```bash
   ros2 launch inspire_hand inspire_hand.launch.py port:=/dev/ttyUSB0
   ```

## Verify

```bash
# Interfaces exported:
ros2 control list_hardware_interfaces

# Send a position command (close to 35 mm, max 300 g grip force):
ros2 action send_goal /gripper_left_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.035, max_effort: 300.0}}"

# Watch diagnostics:
ros2 topic echo /diagnostics

# Read run state:
ros2 service call /inspire_hand/bus/read_run_state \
  inspire_hand/srv/ReadRunState "{gripper_id: 1}"

# Emergency stop:
ros2 service call /inspire_hand/bus/stop \
  inspire_hand/srv/Stop "{gripper_id: 1}"
```

## Services

All under `/inspire_hand/bus/`:
- `stop` — emergency stop (`MoveStophere`)
- `clear_fault` — clear clearable faults (`ErrorClr`)
- `save_params` — persist params to flash (`ParaSave`)
- `set_id` — change a gripper's ID; gated by `allow_set_id:=true`
- `calibrate_range` — set min/max opening raw values
- `set_force_threshold` — runtime force threshold override (grams, 50..1000)
- `read_run_state` — decoded running state, error bits, temperature, opening, force

## URDF parameters

| Param | Default | Notes |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device |
| `baudrate` | `115200` | must match the gripper firmware |
| `read_timeout_ms` | `30` | per-transaction timeout |
| `default_force_threshold` | `500` | grams, applied on activate |
| `allow_set_id` | `false` | must be `true` to use `~/set_id` |
| per-joint `gripper_id` | `1`/`2` | unique on the bus |
| per-joint `default_speed` | `500` | 1..1000 |

## Troubleshooting

- **Failed to open port**: check `ls -l /dev/ttyUSB0`, dialout group membership.
- **All reads time out**: confirm baud (factory 115200), A+/B- wiring, and gripper ID. Try `~/read_run_state` directly.
- **Fault stuck on**: bit 1 (over-temp) self-clears below 60 °C; others clear via `~/clear_fault`. Bit persisting after clearance indicates hardware failure.
```

- [ ] **Step 2: Run the actual hardware smoke test**

Power the gripper, plug in USB adapter, then run the README's "Verify" commands one at a time. Check:
- [ ] `list_hardware_interfaces` shows 4 command interfaces (position+effort × 2 joints) and 6 state interfaces (position+velocity+effort × 2 joints).
- [ ] The action `GripperCommand` moves the gripper and reports succeeded.
- [ ] `/diagnostics` publishes with level OK at ~2 Hz.
- [ ] `~/read_run_state` returns a valid temperature and opening.
- [ ] `~/stop` halts a moving gripper immediately.

If any step fails, capture the error from `ros2_control_node` stderr and debug before continuing.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "docs(inspire_hand): README with usage and verification"
```

---

## Task 16: Hardware integration test (opt-in)

**Files:**
- Create: `test/test_integration.cpp`

- [ ] **Step 1: Write the integration test**

Write `src/inspire_hand/test/test_integration.cpp`:
```cpp
// Opt-in test requiring two grippers at IDs 1 and 2 on INSPIRE_HAND_PORT (default /dev/ttyUSB0).
// Build with: colcon build --packages-select inspire_hand --cmake-args -DINSPIRE_HAND_HW_TESTS=ON

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cstdlib>

#include "inspire_hand/serial_bus.hpp"
#include "inspire_hand/protocol.hpp"

using namespace inspire_hand;
using namespace std::chrono_literals;

static std::string port() {
  const char* p = std::getenv("INSPIRE_HAND_PORT");
  return p ? p : "/dev/ttyUSB0";
}

class HwFixture : public ::testing::Test {
protected:
  SerialBus bus;
  void SetUp() override {
    ASSERT_TRUE(bus.open(port(), 115200).has_value()) << "open " << port();
  }
};

TEST_F(HwFixture, PingEachGripper) {
  for (uint8_t id : {1, 2}) {
    auto r = bus.transact(make_read_act_pos(id), 30ms);
    ASSERT_TRUE(r.has_value()) << "id=" << int(id);
    EXPECT_EQ(r->cmd, Cmd::ReadActPos);
    ASSERT_EQ(r->data.size(), 2u);
  }
}

TEST_F(HwFixture, MoveToKnownPositions) {
  for (uint8_t id : {1, 2}) {
    // Open fully.
    ASSERT_TRUE(bus.transact(make_seek_pos(id, 1000), 30ms).has_value());
    std::this_thread::sleep_for(1500ms);
    auto r1 = bus.transact(make_read_act_pos(id), 30ms);
    ASSERT_TRUE(r1.has_value());
    const uint16_t pos1 = r1->data[0] | (r1->data[1] << 8);
    EXPECT_GT(pos1, 900) << "did not fully open, id=" << int(id);

    // Close to midpoint.
    ASSERT_TRUE(bus.transact(make_seek_pos(id, 500), 30ms).has_value());
    std::this_thread::sleep_for(1500ms);
    auto r2 = bus.transact(make_read_act_pos(id), 30ms);
    ASSERT_TRUE(r2.has_value());
    const uint16_t pos2 = r2->data[0] | (r2->data[1] << 8);
    EXPECT_NEAR(pos2, 500, 50) << "did not reach midpoint, id=" << int(id);
  }
}

TEST_F(HwFixture, RunStateParses) {
  auto r = bus.transact(make_read_eg_run(1), 30ms);
  ASSERT_TRUE(r.has_value());
  ASSERT_GE(r->data.size(), 7u);
  EXPECT_LE(r->data[2], 100);  // temperature in plausible range
}
```

- [ ] **Step 2: Build and run (hardware must be connected)**

Run:
```bash
colcon build --packages-select inspire_hand --symlink-install \
  --cmake-args -DINSPIRE_HAND_HW_TESTS=ON
colcon test --packages-select inspire_hand --ctest-args -R test_integration \
  --event-handlers console_direct+
```
Expected: three tests pass; grippers move.

- [ ] **Step 3: Commit**

```bash
git add src/inspire_hand
git commit -m "test(inspire_hand): hardware-in-the-loop integration tests"
```

---

## Task 17: Final build + format pass

**Files:**
- (no new files)

- [ ] **Step 1: Clean build from scratch**

Run:
```bash
cd /home/joe/Documents/Work/anitron/inspire
rm -rf build install log
colcon build --packages-select inspire_hand --symlink-install
colcon test --packages-select inspire_hand --event-handlers console_direct+
colcon test-result --verbose
```
Expected: all unit tests pass; no build warnings.

- [ ] **Step 2: Verify `ros2_control` loads the plugin**

Run:
```bash
source install/setup.bash
ros2 launch inspire_hand inspire_hand.launch.py &
sleep 3
ros2 control list_hardware_interfaces
kill %1
```
Expected: 4 command interfaces, 6 state interfaces listed.

- [ ] **Step 3: Commit (if anything changed) or tag**

```bash
git tag v0.1.0
git log --oneline
```

---

## Self-Review (completed during plan authoring)

- **Spec coverage:**
  - §Scope — Tasks 0, 9, 14
  - §Package layout — Task 0
  - Layer 1 protocol — Tasks 1–6
  - Layer 2 serial bus — Tasks 7–8
  - Layer 3 plugin — Tasks 9–11
  - URDF example — Task 14
  - Controller config — Task 14
  - Services & diagnostics — Tasks 12, 13
  - Error handling — covered in Tasks 10, 11 (throttled warns, hold-last-state, clamp)
  - Testing — Tasks 1–6 (unit), Task 16 (HW)
  - Timing budget — addressed by 50 Hz controller config in Task 14 and the ≥5 ms gap in Task 7
- **Placeholder scan:** no TBD/TODO; Tasks 10/11 explicitly replace stubs introduced in Task 9.
- **Type consistency:** `Frame`, `Cmd`, `ParseError`, `BusError`, `JointState`, `make_*` functions, `raw_to_meters`/`meters_to_raw`, `start_services`/`stop_services` all consistent across tasks.
- **Ambiguity check:** command-selection policy is explicit (Task 11 table); effort threshold bounds enforced consistently (Tasks 11 and 13); service gating of `set_id` explicit (Task 13).
