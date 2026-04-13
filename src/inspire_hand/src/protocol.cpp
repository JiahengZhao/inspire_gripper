#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

uint8_t checksum(const std::vector<uint8_t>& bytes) noexcept {
  unsigned sum = 0;
  for (uint8_t b : bytes) sum += b;
  return static_cast<uint8_t>(sum & 0xFF);
}

}  // namespace inspire_hand
