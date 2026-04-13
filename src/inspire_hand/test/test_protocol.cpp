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

TEST(Decode, ParaSaveResponse) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x02,0x01,0x01,0x05};
  auto r = decode(buf.data(), buf.size());
  ASSERT_TRUE(r.has_value()) << static_cast<int>(r.error());
  EXPECT_EQ(r->id, 0x01);
  EXPECT_EQ(r->cmd, Cmd::ParaSave);
  ASSERT_EQ(r->data.size(), 1u);
  EXPECT_EQ(r->data[0], 0x01);
}

TEST(Decode, ReadEgRunResponse) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x08,0x41,0x01,0x00,0x23,0xE9,0x03,0x64,0x00,0xBE};
  auto r = decode(buf.data(), buf.size());
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->id, 0x01);
  EXPECT_EQ(r->cmd, Cmd::ReadEgRun);
  ASSERT_EQ(r->data.size(), 7u);
  EXPECT_EQ(r->data[0], 0x01);
  EXPECT_EQ(r->data[1], 0x00);
  EXPECT_EQ(r->data[2], 0x23);
}

TEST(Decode, ReadActPosResponse) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x03,0xD9,0xF1,0x01,0xCF};
  auto r = decode(buf.data(), buf.size());
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->cmd, Cmd::ReadActPos);
  ASSERT_EQ(r->data.size(), 2u);
  const uint16_t opening = static_cast<uint16_t>(r->data[0]) | (static_cast<uint16_t>(r->data[1]) << 8);
  EXPECT_EQ(opening, 497);
}

TEST(Decode, RejectsShort) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01};
  EXPECT_EQ(decode(buf.data(), buf.size()).error(), ParseError::TooShort);
}

TEST(Decode, RejectsBadHeader) {
  const std::vector<uint8_t> buf = {0xEB,0x90,0x01,0x02,0x01,0x01,0x05};
  EXPECT_EQ(decode(buf.data(), buf.size()).error(), ParseError::BadHeader);
}

TEST(Decode, RejectsBadLength) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x05,0x01,0x01,0x05};
  EXPECT_EQ(decode(buf.data(), buf.size()).error(), ParseError::BadLength);
}

TEST(Decode, RejectsBadChecksum) {
  const std::vector<uint8_t> buf = {0xEE,0x16,0x01,0x02,0x01,0x01,0xFF};
  EXPECT_EQ(decode(buf.data(), buf.size()).error(), ParseError::BadChecksum);
}

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
