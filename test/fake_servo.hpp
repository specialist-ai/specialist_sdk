// Fake Feetech STS servos behind a pty, for tests. The Gripper opens path() as its port.

#pragma once

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace specialist_test {

class FakeServos {
 public:
  struct Write {
    std::uint8_t id, address;
    std::vector<std::uint8_t> data;
  };

  FakeServos() {
    char name[256];
    if (::openpty(&master_, &slave_, name, nullptr, nullptr) != 0) {
      std::perror("openpty");
      std::abort();
    }
    // A child process must not inherit the pty and hold the device open.
    for (const int fd : {master_, slave_}) ::fcntl(fd, F_SETFD, FD_CLOEXEC);
    termios tty{};
    ::tcgetattr(slave_, &tty);
    ::cfmakeraw(&tty);
    ::tcsetattr(slave_, TCSANOW, &tty);
    path_ = name;
    thread_ = std::thread([this] { run(); });
  }
  ~FakeServos() {
    stop_ = true;
    thread_.join();
    ::close(master_);
    ::close(slave_);
  }
  FakeServos(const FakeServos&) = delete;
  FakeServos& operator=(const FakeServos&) = delete;

  const std::string& path() const { return path_; }

  /// A servo as it boots: position mode, torque off, Torque_Limit 1000, 12.0 V.
  void add(std::uint8_t id, int position, int temperature = 30) {
    const std::lock_guard<std::mutex> lock(mutex_);
    auto& regs = servos_[id];
    regs.fill(0);
    put16(regs, 48, 1000);
    put16(regs, 42, position);
    put16(regs, 56, position);
    regs[62] = 120;
    regs[63] = static_cast<std::uint8_t>(temperature);
  }
  /// A power cycle: the RAM registers return to their boot values.
  void reset(std::uint8_t id) {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_[id][40] = 0;
    put16(servos_[id], 48, 1000);
  }

  int reg_u8(std::uint8_t id, int address) {
    const std::lock_guard<std::mutex> lock(mutex_);
    return servos_[id][address];
  }
  int reg_u16(std::uint8_t id, int address) {
    const std::lock_guard<std::mutex> lock(mutex_);
    return servos_[id][address] | (servos_[id][address + 1] << 8);
  }
  void set_u8(std::uint8_t id, int address, int value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_[id][address] = static_cast<std::uint8_t>(value);
  }
  void set_u16(std::uint8_t id, int address, int value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    put16(servos_[id], address, value);
  }
  void set_position(std::uint8_t id, int counts) { set_u16(id, 56, counts); }
  void set_temperature(std::uint8_t id, int celsius) { set_u8(id, 63, celsius); }

  /// The next `count` SYNC_READs report `celsius` for every servo.
  void hot_reads(int count, int celsius) {
    hot_temperature_ = celsius;
    hot_reads_ = count;
  }
  int hot_reads_left() const { return hot_reads_; }

  std::vector<Write> writes() {
    const std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

  std::atomic<bool> deaf{false};             // ignores every packet
  std::atomic<int> drop_sync_reads{0};       // ignores the next N SYNC_READs
  std::atomic<bool> torque_stuck_on{false};  // acknowledges Torque_Enable writes, keeps 1
  /// Each SYNC_READ advances the position by one count and derives the temperature
  /// and the load from it, so a torn sample is visible.
  std::atomic<bool> ramp{false};
  std::atomic<int> sync_reads{0};  // SYNC_READs answered

 private:
  using Registers = std::array<std::uint8_t, 256>;

  static void put16(Registers& regs, int address, int value) {
    regs[address] = static_cast<std::uint8_t>(value & 0xFF);
    regs[address + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  }

  void reply(std::uint8_t id, const std::uint8_t* data, std::size_t length) {
    std::vector<std::uint8_t> packet{0xFF, 0xFF, id, static_cast<std::uint8_t>(length + 2), 0};
    packet.insert(packet.end(), data, data + length);
    unsigned sum = 0;
    for (std::size_t i = 2; i < packet.size(); ++i) sum += packet[i];
    packet.push_back(static_cast<std::uint8_t>(~sum));
    for (std::size_t written = 0; written < packet.size();) {
      const ssize_t n = ::write(master_, packet.data() + written, packet.size() - written);
      if (n <= 0) return;
      written += static_cast<std::size_t>(n);
    }
  }

  // A packet is FF FF ID LEN INST PARAMS... CHK.
  void handle(const std::vector<std::uint8_t>& packet) {
    if (deaf) return;
    const std::uint8_t id = packet[2];
    const std::uint8_t* params = packet.data() + 5;
    const std::size_t count = packet[3] >= 2 ? packet[3] - 2u : 0u;
    const std::lock_guard<std::mutex> lock(mutex_);
    const bool known = servos_.count(id) != 0;
    switch (packet[4]) {
      case 0x01:  // PING
        if (known) reply(id, nullptr, 0);
        break;
      case 0x02:  // READ
        if (known && count >= 2 && params[0] + params[1] <= 256) {
          reply(id, servos_[id].data() + params[0], params[1]);
        }
        break;
      case 0x03: {  // WRITE
        if (!known || count < 1) break;
        auto& regs = servos_[id];
        writes_.push_back({id, params[0], {params + 1, params + count}});
        for (std::size_t i = 1; i < count && params[0] + i - 1 < regs.size(); ++i) {
          const std::size_t address = params[0] + i - 1;
          regs[address] = address == 40 && torque_stuck_on ? 1 : params[i];
        }
        reply(id, nullptr, 0);
        break;
      }
      case 0x82: {  // SYNC_READ
        if (drop_sync_reads > 0) {
          --drop_sync_reads;
          break;
        }
        const bool hot = hot_reads_ > 0;
        if (hot) --hot_reads_;
        for (std::size_t i = 2; i < count && servos_.count(params[i]) != 0; ++i) {
          auto& regs = servos_[params[i]];
          if (ramp) {
            const int position = 1800 + (regs[56] | (regs[57] << 8)) % 400 + 1;
            put16(regs, 56, position);
            regs[63] = static_cast<std::uint8_t>(20 + position % 30);
            put16(regs, 60, position % 500);
          }
          Registers sample = regs;
          if (hot) sample[63] = static_cast<std::uint8_t>(hot_temperature_);
          reply(params[i], sample.data() + params[0], params[1]);
        }
        ++sync_reads;
        break;
      }
      case 0x83: {  // SYNC_WRITE
        const std::uint8_t address = params[0];
        const std::uint8_t length = params[1];
        for (std::size_t i = 2; i + length < count; i += length + 1u) {
          if (servos_.count(params[i]) == 0) continue;
          auto& regs = servos_[params[i]];
          for (std::size_t k = 0; k < length; ++k) regs[address + k] = params[i + 1 + k];
        }
        break;
      }
      default:
        break;
    }
  }

  void run() {
    std::vector<std::uint8_t> buffer;
    while (!stop_) {
      pollfd pfd{master_, POLLIN, 0};
      if (::poll(&pfd, 1, 5) <= 0) continue;
      std::uint8_t chunk[512];
      const ssize_t n = ::read(master_, chunk, sizeof(chunk));
      if (n <= 0) continue;
      buffer.insert(buffer.end(), chunk, chunk + n);
      for (;;) {
        while (buffer.size() >= 2 && !(buffer[0] == 0xFF && buffer[1] == 0xFF)) {
          buffer.erase(buffer.begin());
        }
        if (buffer.size() < 4 || buffer.size() < buffer[3] + 4u) break;
        const std::size_t total = buffer[3] + 4u;
        const std::vector<std::uint8_t> packet(buffer.begin(), buffer.begin() + total);
        buffer.erase(buffer.begin(), buffer.begin() + total);
        handle(packet);
      }
    }
  }

  int master_{-1};
  int slave_{-1};
  std::string path_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<int> hot_reads_{0};
  std::atomic<int> hot_temperature_{0};
  std::mutex mutex_;
  std::map<std::uint8_t, Registers> servos_;
  std::vector<Write> writes_;
};

}  // namespace specialist_test
