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

}  // namespace inspire_hand
