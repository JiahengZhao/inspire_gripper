#pragma once
#include <cstddef>
#include <cstdint>

namespace inspire_hand {

uint8_t checksum(const uint8_t* data, std::size_t len) noexcept;

}  // namespace inspire_hand
