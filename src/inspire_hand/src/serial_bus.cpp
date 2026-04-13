#include "inspire_hand/serial_bus.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <thread>

namespace inspire_hand {

namespace {
speed_t baud_flag(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B0;
  }
}
}  // namespace

SerialBus::SerialBus() = default;

SerialBus::~SerialBus() { close(); }

std::expected<void, BusError> SerialBus::open(const std::string& port, int baud) {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ >= 0) ::close(fd_);
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return std::unexpected(BusError::OpenFailed);

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    return std::unexpected(BusError::OpenFailed);
  }
  cfmakeraw(&tty);
  const speed_t bf = baud_flag(baud);
  if (bf == B0) { ::close(fd_); fd_ = -1; return std::unexpected(BusError::OpenFailed); }
  cfsetispeed(&tty, bf);
  cfsetospeed(&tty, bf);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_); fd_ = -1;
    return std::unexpected(BusError::OpenFailed);
  }
  tcflush(fd_, TCIOFLUSH);
  return {};
}

void SerialBus::close() noexcept {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

std::expected<Frame, BusError> SerialBus::transact(const Frame& req,
                                                   std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> g(mu_);
  if (fd_ < 0) return std::unexpected(BusError::NotOpen);

  // Enforce minimum inter-command gap.
  const auto now = std::chrono::steady_clock::now();
  const auto gap = now - last_tx_;
  if (gap < kMinGap) std::this_thread::sleep_for(kMinGap - gap);

  tcflush(fd_, TCIFLUSH);

  const auto bytes = encode(req);
  size_t written = 0;
  while (written < bytes.size()) {
    ssize_t n = ::write(fd_, bytes.data() + written, bytes.size() - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      return std::unexpected(BusError::Io);
    }
    written += static_cast<size_t>(n);
  }
  tcdrain(fd_);
  last_tx_ = std::chrono::steady_clock::now();

  std::vector<uint8_t> rx;
  rx.reserve(32);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  uint8_t buf[64];

  auto total_needed = [&rx]() -> std::optional<size_t> {
    if (rx.size() < 4) return std::nullopt;
    if (rx[0] != 0xEE || rx[1] != 0x16) return std::nullopt;
    return static_cast<size_t>(rx[3]) + 5u;
  };

  while (true) {
    // Drop garbage bytes until we find the response header.
    while (rx.size() >= 2 && (rx[0] != 0xEE || rx[1] != 0x16)) {
      rx.erase(rx.begin());
    }
    if (auto need = total_needed(); need && rx.size() >= *need) break;

    if (std::chrono::steady_clock::now() >= deadline) return std::unexpected(BusError::Timeout);

    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EAGAIN) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (errno == EINTR) continue;
      return std::unexpected(BusError::Io);
    }
    if (n > 0) rx.insert(rx.end(), buf, buf + n);
  }

  auto decoded = decode(rx.data(), rx[3] + 5u);
  if (!decoded) {
    switch (decoded.error()) {
      case ParseError::BadChecksum: return std::unexpected(BusError::Checksum);
      case ParseError::BadHeader:   return std::unexpected(BusError::BadHeader);
      case ParseError::BadLength:   return std::unexpected(BusError::BadLength);
      case ParseError::TooShort:    return std::unexpected(BusError::Io);
    }
  }
  if (decoded->id != req.id) return std::unexpected(BusError::IdMismatch);
  if (decoded->data.size() == 1 && decoded->data[0] == 0x55) {
    return std::unexpected(BusError::ResponseFlag);
  }
  return *decoded;
}

}  // namespace inspire_hand
