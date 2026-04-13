#pragma once
#include <chrono>
#include <expected>
#include <mutex>
#include <string>

#include "inspire_hand/protocol.hpp"

namespace inspire_hand {

enum class BusError {
  OpenFailed,
  Io,
  Timeout,
  Checksum,
  BadHeader,
  BadLength,
  IdMismatch,
  ResponseFlag,
  NotOpen,
};

class SerialBus {
public:
  SerialBus();
  ~SerialBus();

  SerialBus(const SerialBus&) = delete;
  SerialBus& operator=(const SerialBus&) = delete;

  std::expected<void, BusError> open(const std::string& port, int baud);
  void close() noexcept;
  bool is_open() const noexcept { return fd_ >= 0; }

  std::expected<Frame, BusError> transact(const Frame& request,
                                          std::chrono::milliseconds timeout);

private:
  int fd_{-1};
  std::mutex mu_;
  std::chrono::steady_clock::time_point last_tx_{};
  static constexpr std::chrono::milliseconds kMinGap{5};
};

}  // namespace inspire_hand
