#include <gtest/gtest.h>
#include <vector>
#include "inspire_hand/protocol.hpp"

using namespace inspire_hand;

TEST(Checksum, SumsBytesLowOrder) {
  // ParaSave id=1: after frame header the bytes are 01 01 01 -> sum 0x03
  const std::vector<uint8_t> body = {0x01, 0x01, 0x01};
  EXPECT_EQ(checksum(body.data(), body.size()), 0x03);
}

TEST(Checksum, WrapsAt256) {
  // SeekPos id=1 len=3 cmd=54 data F4 01 -> sum = 01+03+54+F4+01 = 14D -> low byte 0x4D
  const std::vector<uint8_t> body = {0x01, 0x03, 0x54, 0xF4, 0x01};
  EXPECT_EQ(checksum(body.data(), body.size()), 0x4D);
}

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
