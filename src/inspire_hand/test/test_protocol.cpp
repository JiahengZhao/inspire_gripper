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
