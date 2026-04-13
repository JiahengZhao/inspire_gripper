#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

uint8_t checksum(const uint8_t* data, std::size_t len) noexcept {
  unsigned sum = 0;
  for (std::size_t i = 0; i < len; ++i) sum += data[i];
  return static_cast<uint8_t>(sum);
}

std::vector<uint8_t> encode(const Frame& f) {
  const uint8_t len = static_cast<uint8_t>(1 + f.data.size());
  std::vector<uint8_t> out;
  out.reserve(5 + f.data.size());
  out.push_back(0xEB);
  out.push_back(0x90);
  out.push_back(f.id);
  out.push_back(len);
  out.push_back(static_cast<uint8_t>(f.cmd));
  out.insert(out.end(), f.data.begin(), f.data.end());
  out.push_back(checksum(out.data() + 2, out.size() - 2));
  return out;
}

std::expected<Frame, ParseError> decode(const uint8_t* data, std::size_t len) {
  if (len < 6) return std::unexpected(ParseError::TooShort);
  if (data[0] != 0xEE || data[1] != 0x16) return std::unexpected(ParseError::BadHeader);
  const uint8_t id = data[2];
  const uint8_t data_len_byte = data[3];
  if (len != static_cast<size_t>(data_len_byte) + 5u) return std::unexpected(ParseError::BadLength);
  if (data_len_byte < 1) return std::unexpected(ParseError::BadLength);
  const uint8_t cmd_byte = data[4];
  const size_t payload_len = static_cast<size_t>(data_len_byte) - 1u;
  const uint8_t recv_cs = data[4 + payload_len + 1];

  auto cs = checksum(data + 2, 1 + 1 + 1 + payload_len);
  if (cs != recv_cs) return std::unexpected(ParseError::BadChecksum);

  Frame f;
  f.id = id;
  f.cmd = static_cast<Cmd>(cmd_byte);
  f.data.assign(data + 5, data + 5 + payload_len);
  return f;
}

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
Frame make_read_eg_para(uint8_t id)  { return Frame{id, Cmd::ReadEgPara,  {}}; }
Frame make_read_act_pos(uint8_t id)  { return Frame{id, Cmd::ReadActPos,  {}}; }
Frame make_read_eg_state(uint8_t id) { return Frame{id, Cmd::ReadEgState, {}}; }
Frame make_read_eg_run(uint8_t id)   { return Frame{id, Cmd::ReadEgRun,   {}}; }
Frame make_stop(uint8_t id)          { return Frame{id, Cmd::MoveStophere,{}}; }
Frame make_clear_fault(uint8_t id)   { return Frame{id, Cmd::ErrorClr,    {}}; }
Frame make_para_save(uint8_t id)     { return Frame{id, Cmd::ParaSave,    {}}; }
Frame make_para_id_set(uint8_t id, uint8_t new_id) {
  return Frame{id, Cmd::ParaIdSet, {new_id}};
}

}  // namespace inspire_hand
