// Opt-in hardware integration test. Requires a physical EG2-4X2 gripper at
// $INSPIRE_HAND_PORT (default /dev/ttyUSB0) with factory ID 1.
// Enable: colcon build ... --cmake-args -DINSPIRE_HAND_HW_TESTS=ON

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

TEST_F(HwFixture, PingGripper) {
  auto r = bus.transact(make_read_act_pos(1), 30ms);
  ASSERT_TRUE(r.has_value()) << "no response from gripper id=1; "
                             << "error=" << static_cast<int>(r.error());
  EXPECT_EQ(r->cmd, Cmd::ReadActPos);
  ASSERT_EQ(r->data.size(), 2u);
}

TEST_F(HwFixture, MoveOpenThenMid) {
  // Open fully.
  ASSERT_TRUE(bus.transact(make_seek_pos(1, 1000), 30ms).has_value());
  std::this_thread::sleep_for(1500ms);
  auto r1 = bus.transact(make_read_act_pos(1), 30ms);
  ASSERT_TRUE(r1.has_value());
  const uint16_t pos1 = static_cast<uint16_t>(r1->data[0]) |
                        (static_cast<uint16_t>(r1->data[1]) << 8);
  EXPECT_GT(pos1, 900) << "did not fully open";

  // Midpoint.
  ASSERT_TRUE(bus.transact(make_seek_pos(1, 500), 30ms).has_value());
  std::this_thread::sleep_for(1500ms);
  auto r2 = bus.transact(make_read_act_pos(1), 30ms);
  ASSERT_TRUE(r2.has_value());
  const uint16_t pos2 = static_cast<uint16_t>(r2->data[0]) |
                        (static_cast<uint16_t>(r2->data[1]) << 8);
  EXPECT_NEAR(pos2, 500, 50) << "did not reach midpoint";
}

TEST_F(HwFixture, RunStateParses) {
  auto r = bus.transact(make_read_eg_run(1), 30ms);
  ASSERT_TRUE(r.has_value());
  ASSERT_GE(r->data.size(), 7u);
  EXPECT_LE(r->data[2], 100);  // temperature plausible
}
