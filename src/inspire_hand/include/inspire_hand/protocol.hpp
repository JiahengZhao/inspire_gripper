#pragma once
#include <cstddef>
#include <cstdint>
#include <expected>
#include <vector>

namespace inspire_hand {

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

enum class ParseError {
  TooShort,
  BadHeader,
  BadLength,
  BadChecksum,
};

uint8_t checksum(const uint8_t* data, std::size_t len) noexcept;
std::vector<uint8_t> encode(const Frame& f);
std::expected<Frame, ParseError> decode(const uint8_t* data, std::size_t len);

}  // namespace inspire_hand
