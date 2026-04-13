#pragma once
#include <cstdint>
#include <vector>

namespace inspire_hand {

uint8_t checksum(const std::vector<uint8_t>& bytes) noexcept;

}  // namespace inspire_hand
