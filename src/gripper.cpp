#include "specialist/gripper.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>

#include "feetech/bus.hpp"

namespace specialist {
namespace {

using Clock = std::chrono::steady_clock;
using std::memory_order_acquire, std::memory_order_relaxed, std::memory_order_release;
using namespace std::chrono_literals;

// The noexcept methods promise lock-free access.
static_assert(std::atomic<double>::is_always_lock_free &&
              std::atomic<std::uint64_t>::is_always_lock_free);

constexpr int kBaudRate = 1000000;
constexpr int kBusRateHz = 250;
constexpr int kMaxTemperature = 65;  // C, below the 70 C cutoff of the servo
constexpr int kHotSamples = 25;      // under load the ADC gives single false readings to 150 C
constexpr int kReadTimeoutMs = 20;
constexpr int kMaxReadFailures = 20;
constexpr double kTravelTolerance = 0.05;  // rad, the jaw play that connect() accepts
constexpr auto kResetCheckInterval = 100ms;
constexpr auto kReleaseRetryInterval = 100ms;  // each try at a silent servo costs a timeout
constexpr int kReleaseAttempts = 2;            // in disconnect(), which has no I/O thread to retry
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string format(const char* pattern, ...) __attribute__((format(printf, 1, 2)));
std::string format(const char* pattern, ...) {
  va_list args, measure;
  va_start(args, pattern);
  va_copy(measure, args);
  std::string out(std::max(std::vsnprintf(nullptr, 0, pattern, measure), 0), '\0');
  va_end(measure);
  std::vsnprintf(out.data(), out.size() + 1, pattern, args);
  va_end(args);
  return out;
}

double to_openness(const GripperConfig& gripper, double counts) {
  return counts * feetech::kRadiansPerCount / gripper.direction + gripper.offset;
}

double to_counts(const GripperConfig& gripper, double openness) {
  return (openness - gripper.offset) * gripper.direction / feetech::kRadiansPerCount;
}

// Returns why the driver refuses `grippers`, or "".
std::string validate(const std::vector<GripperConfig>& grippers) {
  if (grippers.empty()) return "there are no grippers.";
  for (std::size_t i = 0; i < grippers.size(); ++i) {
    const GripperConfig& gripper = grippers[i];
    const std::string at = "gripper '" + gripper.name + "': ";
    // Goals are clamped to one encoder turn, so the travel must not cross the wrap.
    const auto on_turn = [&gripper](double openness) {
      const double counts = to_counts(gripper, openness);
      return counts >= -0.5 && counts < feetech::kCountsPerTurn - 0.5;
    };
    if (gripper.name.empty()) return format("gripper %zu has no name.", i);
    if (gripper.port.empty()) return at + "port is empty.";
    if (gripper.id > feetech::kMaxServoId) return at + format("id %d is too high.", gripper.id);
    if (std::isnan(gripper.direction) || std::isnan(gripper.offset)) {
      return at + "is not calibrated. Run specialist-calibrate.";
    }
    if (std::abs(gripper.direction) != 1.0) return at + "direction must be 1 or -1.";
    if (gripper.torque_limit < 1 || gripper.torque_limit > feetech::kMaxTorqueLimit) {
      return at + "torque_limit must be 1 to 1000.";
    }
    if (!(gripper.closed < gripper.open)) return at + "closed must be below open.";
    // This also refuses an offset, a closed or an open that is not finite.
    if (!on_turn(gripper.closed) || !on_turn(gripper.open)) {
      return at + "the travel crosses the encoder wrap.";
    }
    for (std::size_t j = 0; j < i; ++j) {
      const GripperConfig& other = grippers[j];
      if (other.name == gripper.name) return at + "the name is used twice.";
      if (other.port == gripper.port && other.id == gripper.id) {
        return at + format("servo id %d is used twice on %s.", gripper.id, gripper.port.c_str());
      }
    }
  }
  return {};
}

static_assert(std::is_trivially_copyable_v<GripperState>);
constexpr std::size_t kStateWords = (sizeof(GripperState) + 7) / 8;
using StateWords = std::array<std::uint64_t, kStateWords>;

// What the I/O side knows about one gripper. Only the thread that holds the bus touches it.
struct Book {
  std::size_t port{0}, slot{0};  // its bus, and its place in that bus's SYNC_READ
  GripperState state;
  bool torque_on{false};        // the last Torque_Enable the servo acknowledged was 1
  bool release_pending{false};  // a torque write the servo has not acknowledged yet
  bool read_ok{false};
  int hot_streak{0};
  Clock::time_point next_release{};
};

struct Port {
  std::string name;
  feetech::Bus bus;
  std::vector<std::uint8_t> ids;
  std::vector<std::optional<feetech::Feedback>> feedback;
  std::vector<feetech::GoalEntry> goals;
  int read_failures{0};
};

}  // namespace

// The noexcept side of one gripper. The state is a double-buffered seqlock: the writer fills the
// half that readers do not use, so a reader never waits for a preempted writer. Release stores and
// acquire loads order the words against `seq` without fences, which ThreadSanitizer does not model.
struct detail::GripperIo {
  struct Slot {
    std::atomic<std::uint32_t> seq{0};  // odd while the writer fills it
    std::array<std::atomic<std::uint64_t>, kStateWords> words{};
  };
  std::atomic<double> target{kNaN};  // rad
  std::atomic<bool> torque{false};   // stored after `target`
  std::array<Slot, 2> slots;
  std::atomic<std::uint32_t> published{0};

  void publish(const GripperState& state) noexcept {
    StateWords words{};
    std::memcpy(words.data(), &state, sizeof(GripperState));
    const std::uint32_t next = published.load(memory_order_relaxed) ^ 1u;
    Slot& slot = slots[next];
    const std::uint32_t seq = slot.seq.load(memory_order_relaxed);
    slot.seq.store(seq + 1, memory_order_relaxed);
    for (std::size_t k = 0; k < kStateWords; ++k)
      slot.words[k].store(words[k], memory_order_release);
    slot.seq.store(seq + 2, memory_order_release);
    published.store(next, memory_order_release);
  }

  GripperState read() const noexcept {
    for (;;) {
      const Slot& slot = slots[published.load(memory_order_acquire)];
      const std::uint32_t before = slot.seq.load(memory_order_acquire);
      if ((before & 1u) != 0) continue;
      StateWords words;
      for (std::size_t k = 0; k < kStateWords; ++k)
        words[k] = slot.words[k].load(memory_order_acquire);
      if (slot.seq.load(memory_order_relaxed) != before) continue;
      GripperState state;
      std::memcpy(static_cast<void*>(&state), words.data(), sizeof(state));
      return state;
    }
  }
};

// connect() and disconnect() touch the buses only while no I/O thread runs, so no lock guards them.
struct Grippers::Impl {
  const std::vector<GripperConfig> config;
  std::unique_ptr<detail::GripperIo[]> io;  // as long as the Grippers, so no Gripper dangles
  std::vector<Book> book;
  std::vector<Port> ports;
  std::vector<std::uint8_t> reply;
  Clock::time_point next_reset_check{};

  std::mutex lifecycle_mutex;
  std::thread io_thread;
  std::atomic<bool> running{false};
  std::atomic<bool> faulted{false};
  mutable std::mutex error_mutex;
  std::string error;

  explicit Impl(std::vector<GripperConfig> grippers)
      : config(std::move(grippers)),
        io(std::make_unique<detail::GripperIo[]>(config.size())),
        book(config.size()) {
    if (std::string why = validate(config); !why.empty()) throw std::invalid_argument(why);
    for (std::size_t i = 0; i < config.size(); ++i) {
      auto port = std::find_if(ports.begin(), ports.end(),
                               [&](const Port& p) { return p.name == config[i].port; });
      if (port == ports.end()) {
        port = ports.emplace(ports.end());
        port->name = config[i].port;
      }
      book[i].port = static_cast<std::size_t>(port - ports.begin());
      book[i].slot = port->ids.size();
      port->ids.push_back(config[i].id);
    }
  }
  ~Impl() {
    for (std::size_t i = 0; i < config.size(); ++i) io[i].torque.store(false);
    try {
      (void)stop();
    } catch (...) {  // a destructor must not throw; the release was already sent
    }
  }

  feetech::Bus& bus(std::size_t i) { return ports[book[i].port].bus; }
  std::string servo(std::size_t i) const {
    const GripperConfig& gripper = config[i];
    return format("%s: gripper '%s' servo id %d", gripper.port.c_str(), gripper.name.c_str(),
                  gripper.id);
  }
  bool fail(std::string text) {
    set_error(std::move(text));
    return false;
  }
  void set_error(std::string text, bool append = false) {
    const std::scoped_lock lock(error_mutex);
    error = append && !error.empty() ? error + " " + text : std::move(text);
  }
  void publish(std::size_t i) { io[i].publish(book[i].state); }

  void record(std::size_t i, const feetech::Feedback& sample) {
    const GripperConfig& gripper = config[i];
    GripperState& s = book[i].state;
    s.openness = to_openness(gripper, sample.position_counts);
    s.velocity = sample.velocity_counts_per_sec * feetech::kRadiansPerCount / gripper.direction;
    s.load = gripper.direction * sample.load_permille / feetech::kMaxTorqueLimit;
    s.current = sample.current_amps;
    s.temperature = sample.temperature_celsius;
    s.valid = true;
    ++s.sequence;
  }

  int goal_counts(std::size_t i, double openness) const {
    const GripperConfig& gripper = config[i];
    const double clamped = std::clamp(openness, gripper.closed, gripper.open);
    const auto counts = static_cast<int>(std::lround(to_counts(gripper, clamped)));
    return std::clamp(counts, 0, feetech::kCountsPerTurn - 1);
  }

  bool read_byte(std::size_t i, std::uint8_t address) {
    return bus(i).read_registers(config[i].id, address, 1, reply) && reply.size() == 1;
  }
  bool write(std::size_t i, std::uint8_t address, const std::uint8_t* data, std::size_t size) {
    return bus(i).write_registers(config[i].id, address, data, size);
  }
  bool write_torque(std::size_t i, bool on) {
    const std::uint8_t value = on ? 1 : 0;
    book[i].release_pending = true;  // the servo may take a write whose acknowledgement is lost
    if (!write(i, feetech::reg::kTorqueEnable, &value, 1)) return false;
    book[i].torque_on = on;
    book[i].release_pending = false;
    return true;
  }
  // Torque_Limit is RAM and starts at full stall.
  bool write_limit(std::size_t i) {
    const int permille = config[i].torque_limit;
    const std::uint8_t value[2] = {static_cast<std::uint8_t>(permille & 0xFF),
                                   static_cast<std::uint8_t>(permille >> 8)};
    return write(i, feetech::reg::kTorqueLimit, value, 2);
  }
  bool write_goal(std::size_t i, int counts) {
    const auto block = feetech::encode_goal_block(counts);
    return write(i, feetech::reg::kAcceleration, block.data(), block.size());
  }

  // Returns one sentence for each servo on an open bus that did not acknowledge the release.
  std::string release_all(int attempts) {
    std::string error;
    for (std::size_t i = 0; i < config.size(); ++i) {
      if (!bus(i).is_open()) continue;
      bool released = false;
      for (int attempt = 0; attempt < attempts && !released; ++attempt) {
        released = write_torque(i, false);
      }
      if (!released) {
        error += (error.empty() ? "" : " ") + servo(i) +
                 " did not acknowledge the torque release, so its torque may still be on.";
      }
    }
    return error;
  }

  // The cause goes into the error before `faulted`, so a reader that sees the fault sees its
  // cause. The release comes before the report on stderr.
  void fault(std::string cause, bool again = false) {
    if (faulted.load(memory_order_relaxed) && !again) return;
    set_error(cause);
    faulted.store(true, memory_order_release);
    for (auto& b : book) b.next_release = Clock::now() + kReleaseRetryInterval;
    std::string outcome = "Torque released.";
    if (std::string error = release_all(1); !error.empty()) {
      outcome = error + " The driver resends the release.";
    }
    set_error(outcome, true);
    std::fprintf(stderr, "[specialist] %s %s\n", cause.c_str(), outcome.c_str());
  }

  // A servo that restarted, or whose own protection released the torque, reads 0.
  void check_reset(Clock::time_point now) {
    if (faulted.load(memory_order_relaxed) || now < next_reset_check) return;
    next_reset_check = now + kResetCheckInterval;
    for (std::size_t i = 0; i < config.size(); ++i) {
      // The SYNC_READ that follows counts a failed read.
      if (book[i].torque_on && read_byte(i, feetech::reg::kTorqueEnable) && reply[0] == 0) {
        return fault(servo(i) + " restarted or released its torque.");
      }
    }
  }

  // A servo that did not reply to the last read gets one release per kReleaseRetryInterval.
  void drive(Clock::time_point now) {
    for (auto& port : ports) port.goals.clear();
    const bool armed = !faulted.load(memory_order_acquire);
    for (std::size_t i = 0; i < config.size(); ++i) {
      Book& b = book[i];
      // The acquire pairs with set_target(), so `target` is at least as new as the request.
      const bool want = armed && io[i].torque.load(memory_order_acquire);
      const int goal = want ? goal_counts(i, io[i].target.load(memory_order_relaxed)) : 0;
      if (want && (!b.torque_on || b.release_pending)) {
        // The limit goes first, because a restarted servo is at full stall. The goal goes before
        // the torque, so the servo does not jump to an old goal.
        (void)(write_limit(i) && write_goal(i, goal) && write_torque(i, true));
      } else if (!want && (b.torque_on || b.release_pending) &&
                 (b.read_ok || now >= b.next_release)) {
        b.next_release = now + kReleaseRetryInterval;
        (void)write_torque(i, false);
      }
      if (want && b.torque_on) {
        ports[b.port].goals.push_back({config[i].id, feetech::encode_goal_block(goal)});
      }
    }
    // A failed send also fails the SYNC_READ that follows, which counts toward a lost bus.
    for (auto& port : ports) {
      if (!port.goals.empty()) (void)port.bus.sync_write_goals(port.goals);
    }
  }

  void read_all() {
    for (auto& port : ports) {
      const std::size_t answered = port.bus.sync_read_feedback(port.ids, port.feedback);
      port.read_failures =
          answered >= port.ids.size() ? 0 : std::min(port.read_failures + 1, kMaxReadFailures);
    }
    for (std::size_t i = 0; i < config.size(); ++i) {
      Book& b = book[i];
      const auto& feedback = ports[b.port].feedback;
      b.read_ok = b.slot < feedback.size() && feedback[b.slot].has_value();
      b.state.valid = false;
      if (b.read_ok) {
        record(i, *feedback[b.slot]);
        b.hot_streak =
            b.state.temperature >= kMaxTemperature ? std::min(b.hot_streak + 1, kHotSamples) : 0;
      }
      b.state.torque_enabled = b.torque_on;
      publish(i);
    }
    if (faulted.load(memory_order_relaxed)) return;
    for (const auto& port : ports) {
      if (port.read_failures < kMaxReadFailures) continue;
      std::string cause =
          format("%s: bus lost after %d failed reads.", port.name.c_str(), port.read_failures);
      for (std::size_t i = 0; i < config.size(); ++i) {
        if (&ports[book[i].port] == &port && !book[i].read_ok)
          cause += " " + servo(i) + " did not reply.";
      }
      fault(std::move(cause));
    }
    for (std::size_t i = 0; i < config.size(); ++i) {
      if (book[i].hot_streak < kHotSamples) continue;
      fault(servo(i) + format(" held %d C for %d reads.", book[i].state.temperature, kHotSamples));
    }
  }

  void io_loop() noexcept {
    try {
      constexpr auto period = std::chrono::nanoseconds(1'000'000'000LL / kBusRateHz);
      for (auto next_cycle = Clock::now(); running.load(memory_order_acquire);) {
        const auto now = Clock::now();
        check_reset(now);
        drive(now);
        read_all();
        // An overrun builds no debt.
        next_cycle = std::max(next_cycle + period, Clock::now());
        std::this_thread::sleep_until(next_cycle);
      }
    } catch (...) {
      try {
        fault("the I/O thread stopped on an error.", true);
      } catch (...) {
      }
    }
  }

  // Opens each port, releases every torque and checks each servo. Returns an error, or "".
  std::string open_and_check() {
    for (auto& port : ports) {
      port.bus.set_timeout_ms(kReadTimeoutMs);
      if (!port.bus.open(port.name, kBaudRate)) {
        return format("%s: cannot open the port: %s", port.name.c_str(),
                      port.bus.last_error().c_str());
      }
    }
    for (std::size_t i = 0; i < config.size(); ++i) {
      if (!bus(i).ping(config[i].id)) return servo(i) + " did not reply.";
    }
    // Torque_Enable survives a killed process, so a crashed run leaves the jaws clamped. Every
    // servo is released first, so one that fails leaves no other clamped.
    if (std::string error = release_all(1); !error.empty()) return error;
    for (auto& port : ports) (void)port.bus.sync_read_feedback(port.ids, port.feedback);
    for (std::size_t i = 0; i < config.size(); ++i) {
      const GripperConfig& gripper = config[i];
      Book& b = book[i];
      const auto& feedback = ports[b.port].feedback;
      if (!read_byte(i, feetech::reg::kTorqueEnable)) return servo(i) + " did not reply.";
      if (reply[0] != 0)
        return servo(i) + format(" reads Torque_Enable %d after a write of 0.", reply[0]);
      if (!write_limit(i)) return servo(i) + " did not take Torque_Limit.";
      if (b.slot >= feedback.size() || !feedback[b.slot] ||
          !read_byte(i, feetech::reg::kOperatingMode)) {
        return servo(i) + " did not reply.";
      }
      // A servo in wheel or PWM mode ignores Goal_Position and turns.
      if (reply[0] != feetech::kPositionMode) return servo(i) + " is not in position mode.";
      record(i, *feedback[b.slot]);
      b.read_ok = true;
      publish(i);
      if (b.state.temperature >= kMaxTemperature) {
        return servo(i) +
               format(" is at %d C. Let it cool below %d C.", b.state.temperature, kMaxTemperature);
      }
      // A jaw outside its travel means a wrong calibration, a wrong servo, or a jam. A goal
      // clamped from far away would drive it at full torque.
      const double openness = b.state.openness;
      if (openness < gripper.closed - kTravelTolerance ||
          openness > gripper.open + kTravelTolerance) {
        return servo(i) + format(" is at %.3f rad, outside its travel.", openness);
      }
    }
    return {};
  }

  // Stops the I/O thread, releases every torque and closes each port. Returns an error, or "",
  // and prints the error, since the destructor has no other way to report it.
  std::string stop() {
    running.store(false, memory_order_release);
    if (io_thread.joinable()) io_thread.join();
    std::string error = release_all(kReleaseAttempts);
    if (!error.empty()) std::fprintf(stderr, "[specialist] %s\n", error.c_str());
    for (auto& port : ports) port.bus.close();
    for (std::size_t i = 0; i < config.size(); ++i) {
      book[i].state.valid = book[i].read_ok = false;
      book[i].state.torque_enabled = book[i].torque_on;
      publish(i);
    }
    return error;
  }

  bool connect() {
    const std::scoped_lock lock(lifecycle_mutex);
    (void)stop();  // a failed release is reported by the release in open_and_check()
    for (std::size_t i = 0; i < config.size(); ++i) {
      Book fresh;
      fresh.port = book[i].port;
      fresh.slot = book[i].slot;
      fresh.state.sequence = book[i].state.sequence;
      book[i] = fresh;
      publish(i);
    }
    for (auto& port : ports) port.read_failures = 0;
    if (std::string error = open_and_check(); !error.empty()) {
      if (std::string unreleased = stop(); !unreleased.empty()) error += " " + unreleased;
      return fail(error);
    }
    // This drops the torque requests from before this point, also those from other threads.
    for (std::size_t i = 0; i < config.size(); ++i) io[i].torque.store(false, memory_order_release);
    next_reset_check = Clock::now() + kResetCheckInterval;
    set_error("");
    faulted.store(false, memory_order_release);
    running.store(true, memory_order_release);
    try {
      io_thread = std::thread([this] { io_loop(); });
    } catch (const std::system_error& e) {
      running.store(false, memory_order_release);
      (void)stop();
      return fail(format("cannot start the I/O thread: %s", e.what()));
    }
    return true;
  }
};

void Gripper::set_target(double openness) const noexcept {
  if (!std::isfinite(openness)) return;
  io_->target.store(openness, memory_order_relaxed);
  io_->torque.store(true, memory_order_release);
}

void Gripper::release() const noexcept { io_->torque.store(false, memory_order_release); }

GripperState Gripper::state() const noexcept { return io_->read(); }

Grippers::Grippers(std::vector<GripperConfig> config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Grippers::~Grippers() = default;

bool Grippers::connect() { return impl_->connect(); }

bool Grippers::disconnect() {
  const std::scoped_lock lock(impl_->lifecycle_mutex);
  for (std::size_t i = 0; i < impl_->config.size(); ++i) impl_->io[i].torque.store(false);
  std::string error = impl_->stop();
  if (!error.empty()) impl_->set_error(error);
  return error.empty();
}

bool Grippers::faulted() const noexcept { return impl_->faulted.load(memory_order_acquire); }

std::string Grippers::error() const {
  const std::scoped_lock lock(impl_->error_mutex);
  return impl_->error;
}

std::optional<Gripper> Grippers::gripper(const std::string& name) {
  for (std::size_t i = 0; i < impl_->config.size(); ++i) {
    if (impl_->config[i].name == name) return Gripper(impl_->io[i]);
  }
  return std::nullopt;
}

}  // namespace specialist
