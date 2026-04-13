#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

uint8_t checksum(const uint8_t* data, std::size_t len) noexcept {
  unsigned sum = 0;
  for (std::size_t i = 0; i < len; ++i) sum += data[i];
  return static_cast<uint8_t>(sum);
}

}  // namespace inspire_hand
