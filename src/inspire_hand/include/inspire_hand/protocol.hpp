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

// Returns true for commands whose response is a 1-byte ack (0x01 success, 0x55 failure).
// Read commands return multi-byte payloads and should not be classified as write commands.
bool is_write_cmd(Cmd c) noexcept;

enum class ParseError {
  TooShort,
  BadHeader,
  BadLength,
  BadChecksum,
};

uint8_t checksum(const uint8_t* data, std::size_t len) noexcept;
std::vector<uint8_t> encode(const Frame& f);
std::expected<Frame, ParseError> decode(const uint8_t* data, std::size_t len);

Frame make_seek_pos(uint8_t id, uint16_t pos);
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

}  // namespace inspire_hand
