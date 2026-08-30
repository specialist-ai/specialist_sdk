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
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>

#include "feetech/bus.hpp"
#include "log_internal.hpp"

namespace specialist {
namespace {

using Clock = std::chrono::steady_clock;
using std::memory_order_acquire, std::memory_order_relaxed, std::memory_order_release;
using namespace std::chrono_literals;

// The noexcept methods promise lock-free access.
static_assert(std::atomic<double>::is_always_lock_free &&
              std::atomic<std::uint64_t>::is_always_lock_free);

constexpr int kMaxTemperature = 65;  // C, below the 70 C cutoff of the servo
constexpr int kHotSamples = 25;      // under load the ADC gives single false readings to 150 C
constexpr int kReadTimeoutMs = 20;
constexpr int kMaxReadFailures = 20;
constexpr double kTravelTolerance = 0.05;  // rad, the finger play that activate() accepts
constexpr auto kResetCheckInterval = 100ms;
constexpr auto kReleaseRetryInterval = 100ms;  // each try at a silent servo costs a timeout
constexpr auto kMinGap = 1ms;                  // the bus is free this long for a lifecycle call
constexpr int kReleaseAttempts = 2;            // in disconnect(), which has no I/O thread to retry
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A log handler on an I/O thread that calls a lifecycle method fails instead of deadlocking.
thread_local bool t_on_io_thread = false;

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

double to_radians(const JointConfig& joint, double counts) {
  return joint.direction * (counts * feetech::kRadiansPerCount - joint.homing_offset);
}

double to_counts(const JointConfig& joint, double radians) {
  return (joint.direction * radians + joint.homing_offset) / feetech::kRadiansPerCount;
}

template <typename T>
T copy_under(std::mutex& mutex, const T& value) {
  const std::scoped_lock lock(mutex);
  return value;
}

// The handler gets the messages after every lock is released.
struct LogQueue {
  std::vector<std::pair<LogLevel, std::string>> entries;
  void add(LogLevel level, std::string text) { entries.emplace_back(level, std::move(text)); }
  ~LogQueue() {
    for (const auto& [level, text] : entries) log_message(level, text);
  }
};

static_assert(std::is_trivially_copyable_v<JointState>);
constexpr std::size_t kStateWords = (sizeof(JointState) + 7) / 8;
using StateWords = std::array<std::uint64_t, kStateWords>;

// The state is a double-buffered seqlock: the writer fills the half that readers do not use, so a
// reader never waits for a preempted writer. Release stores and acquire loads order the words
// against `seq` without fences, which ThreadSanitizer does not model.
struct JointIo {
  struct Slot {
    std::atomic<std::uint32_t> seq{0};  // odd while the writer fills it
    std::array<std::atomic<std::uint64_t>, kStateWords> words{};
  };
  std::atomic<double> command{kNaN};        // rad
  std::atomic<bool> command_is_new{false};  // set_target() since set_torque_enabled() or activate()
  std::atomic<bool> torque_request{false};
  std::atomic<double> torque_fraction{1.0};
  std::atomic<double> present{kNaN};  // rad, the last good read
  std::array<Slot, 2> slots;
  std::atomic<std::uint32_t> published{0};

  void publish(const JointState& state) noexcept {
    StateWords words{};
    std::memcpy(words.data(), &state, sizeof(JointState));
    const std::uint32_t next = published.load(memory_order_relaxed) ^ 1u;
    Slot& slot = slots[next];
    const std::uint32_t seq = slot.seq.load(memory_order_relaxed);
    slot.seq.store(seq + 1, memory_order_relaxed);
    for (std::size_t k = 0; k < kStateWords; ++k)
      slot.words[k].store(words[k], memory_order_release);
    slot.seq.store(seq + 2, memory_order_release);
    published.store(next, memory_order_release);
  }

  JointState read() const noexcept {
    for (;;) {
      const Slot& slot = slots[published.load(memory_order_acquire)];
      const std::uint32_t before = slot.seq.load(memory_order_acquire);
      if ((before & 1u) != 0) continue;
      StateWords words;
      for (std::size_t k = 0; k < kStateWords; ++k)
        words[k] = slot.words[k].load(memory_order_acquire);
      if (slot.seq.load(memory_order_relaxed) != before) continue;
      JointState state;
      std::memcpy(static_cast<void*>(&state), words.data(), sizeof(state));
      return state;
    }
  }
};

// What the bus side knows about one joint. Guarded by bus_mutex.
struct JointBook {
  JointState state;
  bool torque_on{false};        // the last Torque_Enable the servo acknowledged was 1
  bool release_pending{false};  // a release the servo has not acknowledged yet
  bool read_ok{false};
  int written_limit{-1};
  int hot_streak{0};
  Clock::time_point next_release_attempt{};
};

}  // namespace

struct Gripper::Impl {
  std::mutex lifecycle_mutex;  // lifecycle calls run one at a time

  // bus_mutex guards the bus and every field up to `config`.
  std::mutex bus_mutex;
  std::atomic<int> lifecycle_waiting{0};  // the I/O thread steps aside while this is positive
  feetech::Bus bus;
  std::array<JointBook, kMaxJoints> book;
  std::vector<std::uint8_t> ids;
  std::vector<feetech::GoalEntry> goals;
  std::vector<std::optional<feetech::Feedback>> feedback;
  std::vector<std::uint8_t> reply;
  int read_failures{0};
  Clock::time_point next_reset_check{};

  mutable std::mutex config_mutex;  // connect() writes `config` only while no I/O thread runs
  GripperConfig config;

  std::thread io_thread;
  std::atomic<bool> running{false};
  std::atomic<bool> connected{false};
  std::atomic<bool> active{false};
  std::atomic<std::uint32_t> fault{0};
  std::atomic<std::size_t> joint_count{0};
  std::array<JointIo, kMaxJoints> joints;

  mutable std::mutex text_mutex;
  std::string last_error;
  std::string fault_message;

  ~Impl() {
    running.store(false);  // covers a disconnect() that ~Gripper could not finish
    if (io_thread.joinable()) io_thread.join();
  }

  static JointIo* io(const std::unique_ptr<Impl>& impl, std::size_t joint) noexcept {
    if (!impl || joint >= impl->joint_count.load(memory_order_acquire)) return nullptr;
    return &impl->joints[joint];
  }

  // Takes bus_mutex ahead of the I/O thread, because std::mutex is not fair.
  std::unique_lock<std::mutex> lock_bus() {
    ++lifecycle_waiting;
    std::unique_lock lock(bus_mutex);
    --lifecycle_waiting;
    return lock;
  }

  bool fail(std::string message) {
    const std::scoped_lock lock(text_mutex);
    last_error = std::move(message);
    return false;
  }

  std::size_t count() const { return joint_count.load(memory_order_relaxed); }
  bool faulted() const { return fault.load(memory_order_acquire) != 0; }
  bool latched(Fault bit) const {
    return has(static_cast<Fault>(fault.load(memory_order_acquire)), bit);
  }
  bool driving() const { return active.load(memory_order_acquire) && !faulted(); }
  const char* port() const { return config.port.c_str(); }
  std::string servo(std::size_t i) const {
    const JointConfig& joint = config.joints[i];
    return format("%s: joint '%s' servo id %d", port(), joint.name.c_str(), joint.id);
  }

  void publish(std::size_t i) { joints[i].publish(book[i].state); }

  void record(std::size_t i, const feetech::Feedback& sample) {
    const JointConfig& joint = config.joints[i];
    JointState& s = book[i].state;
    s.position = to_radians(joint, sample.position_counts);
    s.velocity = joint.direction * sample.velocity_counts_per_sec * feetech::kRadiansPerCount;
    s.load = joint.direction * sample.load_permille / feetech::kMaxTorqueLimit;
    s.current = sample.current_amps;
    s.temperature = sample.temperature_celsius;
    s.servo_status = sample.status;
    s.valid = true;
    ++s.sequence;
    s.stamp_ns = std::chrono::nanoseconds(Clock::now().time_since_epoch()).count();
    joints[i].present.store(s.position, memory_order_release);
  }

  int effective_limit(std::size_t i) const {
    const double fraction = joints[i].torque_fraction.load(memory_order_relaxed);
    return std::min(config.joints[i].torque_limit,
                    static_cast<int>(std::lround(fraction * feetech::kMaxTorqueLimit)));
  }

  int goal_counts(std::size_t i, double radians) const {
    const JointConfig& joint = config.joints[i];
    if (!std::isfinite(radians)) radians = book[i].state.position;
    const double clamped = std::clamp(radians, joint.lower_limit, joint.upper_limit);
    const auto counts = static_cast<int>(std::lround(to_counts(joint, clamped)));
    return std::clamp(counts, 0, feetech::kCountsPerTurn - 1);
  }

  bool read_byte(std::size_t i, std::uint8_t address) {
    return bus.read_registers(config.joints[i].id, address, 1, reply) && reply.size() == 1;
  }
  bool write(std::size_t i, std::uint8_t address, const std::uint8_t* data, std::size_t size) {
    return bus.write_registers(config.joints[i].id, address, data, size);
  }
  bool write_torque(std::size_t i, bool on) {
    const std::uint8_t value = on ? 1 : 0;
    if (!write(i, feetech::reg::kTorqueEnable, &value, 1)) return false;
    book[i].torque_on = on;
    book[i].release_pending = false;
    return true;
  }
  bool release(std::size_t i) {
    book[i].release_pending = true;
    return write_torque(i, false);
  }
  bool write_limit(std::size_t i, int permille) {
    const std::uint8_t value[2] = {static_cast<std::uint8_t>(permille & 0xFF),
                                   static_cast<std::uint8_t>(permille >> 8)};
    if (!write(i, feetech::reg::kTorqueLimit, value, 2)) return false;
    book[i].written_limit = permille;
    return true;
  }
  bool write_goal(std::size_t i, int counts) {
    const auto block = feetech::encode_goal_block(counts);
    return write(i, feetech::reg::kAcceleration, block.data(), block.size());
  }

  // Returns one sentence for each servo that did not acknowledge the release.
  std::string release_all(int attempts) {
    std::string error;
    for (std::size_t i = 0; i < count(); ++i) {
      bool released = false;
      for (int attempt = 0; attempt < attempts && !released; ++attempt) released = release(i);
      if (!released) {
        error += (error.empty() ? "" : " ") + servo(i) +
                 " did not acknowledge the torque release, so its torque may still be on.";
      }
    }
    return error;
  }

  // Releases each servo, reads the release back, and writes Torque_Limit, which is RAM and starts
  // at full stall. Returns an error, or "".
  std::string reset_servos() {
    for (std::size_t i = 0; i < count(); ++i) book[i].release_pending = true;
    for (std::size_t i = 0; i < count(); ++i) {
      if (!release(i) || !read_byte(i, feetech::reg::kTorqueEnable)) {
        return servo(i) + " did not acknowledge the torque release.";
      }
      if (reply[0] != 0) {
        book[i].release_pending = true;
        return servo(i) + format(" reads Torque_Enable %d after a write of 0.", reply[0]);
      }
      if (!write_limit(i, effective_limit(i))) return servo(i) + " did not take Torque_Limit.";
    }
    return {};
  }

  // Under bus_mutex. faulted() is true before the release, and the log follows the release.
  // The cause goes into fault_message before the fault bit, so a reader that sees the fault
  // also sees its cause.
  void raise_fault(Fault cause, std::string message, LogQueue& logs, bool again = false) {
    if (latched(cause) && !again) return;
    add_fault_text(message);
    fault.fetch_or(static_cast<std::uint32_t>(cause));
    active.store(false, memory_order_release);
    for (std::size_t i = 0; i < count(); ++i) {
      book[i].next_release_attempt = Clock::now() + kReleaseRetryInterval;
    }
    std::string outcome = "Torque released.";
    if (!release_all(1).empty()) {
      fault.fetch_or(static_cast<std::uint32_t>(Fault::kReleaseFailed));
      outcome = "A servo did not acknowledge the torque release, so the driver resends it.";
    }
    add_fault_text(outcome);
    message += " " + outcome;
    {
      const std::scoped_lock lock(text_mutex);
      last_error = message;
    }
    logs.add(LogLevel::kError, std::move(message));
  }

  void add_fault_text(const std::string& text) {
    const std::scoped_lock lock(text_mutex);
    fault_message += (fault_message.empty() ? "" : " ") + text;
  }

  // A servo that restarted, or whose own protection released the torque, reads 0.
  void check_reset(Clock::time_point now, LogQueue& logs) {
    if (now < next_reset_check) return;
    next_reset_check = now + kResetCheckInterval;
    for (std::size_t i = 0; i < count(); ++i) {
      // The SYNC_READ that follows counts a failed read.
      if (book[i].torque_on && read_byte(i, feetech::reg::kTorqueEnable) && reply[0] == 0) {
        return raise_fault(Fault::kServoReset, servo(i) + " restarted or released its torque.",
                           logs);
      }
    }
  }

  void drive() {
    goals.clear();
    for (std::size_t i = 0; i < count(); ++i) {
      JointBook& b = book[i];
      // The acquire pairs with set_torque_enabled(), so `command` holds its reseed.
      const bool want = joints[i].torque_request.load(memory_order_acquire);
      const int goal = goal_counts(i, joints[i].command.load(memory_order_relaxed));
      const int limit = effective_limit(i);
      if (want && (!b.torque_on || b.release_pending)) {
        // The limit goes first, because a restarted servo is at full stall. The goal goes before
        // the torque, so the servo does not jump to an old goal.
        (void)(write_limit(i, limit) && write_goal(i, goal) && write_torque(i, true));
      } else if (want && b.written_limit != limit) {
        write_limit(i, limit);
      } else if (!want && (b.torque_on || b.release_pending)) {
        release(i);
      }
      if (b.torque_on) goals.push_back({config.joints[i].id, feetech::encode_goal_block(goal)});
    }
    // A failed send also fails the SYNC_READ that follows, which counts toward kBusLost.
    if (!goals.empty()) (void)bus.sync_write_goals(goals);
  }

  // A servo that did not reply to the last read gets one try per kReleaseRetryInterval.
  void resend_releases(Clock::time_point now, LogQueue& logs) {
    for (std::size_t i = 0; i < count(); ++i) {
      JointBook& b = book[i];
      if (!b.release_pending || (!b.read_ok && now < b.next_release_attempt)) continue;
      b.next_release_attempt = now + kReleaseRetryInterval;
      if (write_torque(i, false)) logs.add(LogLevel::kInfo, servo(i) + " released its torque.");
    }
  }

  void read_all(LogQueue& logs) {
    const std::size_t answered = bus.sync_read_feedback(ids, feedback);
    for (std::size_t i = 0; i < count(); ++i) {
      JointBook& b = book[i];
      b.read_ok = i < feedback.size() && feedback[i].has_value();
      b.state.valid = false;
      if (b.read_ok) {
        record(i, *feedback[i]);
        b.hot_streak =
            b.state.temperature >= kMaxTemperature ? std::min(b.hot_streak + 1, kHotSamples) : 0;
      }
      b.state.torque_enabled = b.torque_on;
      publish(i);
    }
    read_failures = answered >= count() ? 0 : std::min(read_failures + 1, kMaxReadFailures);
    if (read_failures >= kMaxReadFailures) {
      raise_fault(Fault::kBusLost,
                  format("%s: bus lost after %d failed reads.", port(), read_failures), logs);
    }
    for (std::size_t i = 0; i < count(); ++i) {
      if (book[i].hot_streak < kHotSamples) continue;
      raise_fault(
          Fault::kOverTemperature,
          servo(i) + format(" held %d C for %d reads.", book[i].state.temperature, kHotSamples),
          logs);
    }
  }

  void io_loop() noexcept {
    t_on_io_thread = true;
    try {
      const auto period = std::chrono::nanoseconds(1'000'000'000LL / config.bus_rate_hz);
      for (auto next_cycle = Clock::now(); running.load(memory_order_acquire);) {
        next_cycle += period;
        {
          LogQueue logs;  // declared first, so it flushes after the unlock
          while (lifecycle_waiting.load() > 0 && running.load(memory_order_relaxed)) {
            std::this_thread::sleep_for(100us);
          }
          const std::scoped_lock lock(bus_mutex);
          const auto now = Clock::now();
          if (driving()) check_reset(now, logs);
          if (driving()) {
            drive();
          } else {
            resend_releases(now, logs);
          }
          read_all(logs);
        }
        // An overrun builds no debt.
        next_cycle = std::max(next_cycle, Clock::now() + kMinGap);
        std::this_thread::sleep_until(next_cycle);
      }
    } catch (...) {
      try {
        LogQueue logs;
        const std::scoped_lock lock(bus_mutex);
        raise_fault(Fault::kBusLost, format("%s: the I/O thread stopped on an error.", port()),
                    logs, true);
      } catch (...) {
      }
    }
  }

  // The log queue is declared before the lock, so it flushes after the unlock.
  template <typename Body, typename... Args>
  bool lifecycle(const char* call, Body body, const Args&... args) {
    if (t_on_io_thread) return fail(format("%s() cannot run in a log handler.", call));
    LogQueue logs;
    const std::scoped_lock lock(lifecycle_mutex);
    return (this->*body)(logs, args...);
  }

  void stop_driving() {
    active.store(false, memory_order_release);
    for (std::size_t i = 0; i < count(); ++i) {
      joints[i].torque_request.store(false, memory_order_release);
    }
  }

  bool report(LogQueue& logs, const std::string& error) {
    if (error.empty()) return true;
    logs.add(LogLevel::kWarn, error);
    return fail(error);
  }

  bool disconnect_locked(LogQueue& logs) {
    stop_driving();
    running.store(false, memory_order_release);
    if (io_thread.joinable()) io_thread.join();
    std::string error;
    const auto bus_lock = lock_bus();
    if (bus.is_open()) {
      error = release_all(kReleaseAttempts);
      bus.close();
    }
    for (std::size_t i = 0; i < count(); ++i) {
      book[i].state.valid = book[i].read_ok = false;
      book[i].state.torque_enabled = book[i].torque_on;
      publish(i);
    }
    connected.store(false, memory_order_release);
    return report(logs, error);
  }

  bool connect_locked(LogQueue& logs, const GripperConfig& next) {
    disconnect_locked(logs);  // the log reports a failed release; the new session still starts
    std::string error;
    if (!Gripper::validate(next, error)) return fail(error);
    {
      const std::scoped_lock lock(config_mutex);
      config = next;
    }
    const auto bus_lock = lock_bus();
    ids.clear();
    for (const auto& joint : config.joints) ids.push_back(joint.id);
    read_failures = 0;
    for (std::size_t i = 0; i < kMaxJoints; ++i) {
      JointIo& io = joints[i];
      io.command.store(kNaN, memory_order_relaxed);
      io.torque_request.store(false, memory_order_relaxed);
      io.torque_fraction.store(1.0, memory_order_relaxed);
      io.present.store(kNaN, memory_order_relaxed);
      const std::uint64_t sequence = book[i].state.sequence;
      book[i] = JointBook{};
      book[i].state.sequence = sequence;
      publish(i);
    }
    joint_count.store(config.joints.size(), memory_order_release);

    const auto close_and_fail = [this](std::string message) {
      bus.close();
      return fail(std::move(message));
    };
    bus.set_timeout_ms(kReadTimeoutMs);
    if (!bus.open(config.port, config.baud_rate)) {
      return fail(format("%s: cannot open the port: %s", port(), bus.last_error().c_str()));
    }
    for (std::size_t i = 0; i < count(); ++i) {
      if (!bus.ping(config.joints[i].id)) return close_and_fail(servo(i) + " did not reply.");
    }
    // Torque_Enable survives a killed process, so a crashed run leaves the jaws clamped.
    if (error = reset_servos(); !error.empty()) return close_and_fail(error);
    running.store(true, memory_order_release);
    try {
      io_thread = std::thread([this] { io_loop(); });
    } catch (const std::system_error& e) {
      running.store(false, memory_order_release);
      return close_and_fail(format("cannot start the I/O thread: %s", e.what()));
    }
    connected.store(true, memory_order_release);
    {
      const std::scoped_lock text(text_mutex);
      last_error.clear();
    }
    logs.add(LogLevel::kInfo, format("%s: connected to %zu servos.", port(), count()));
    return true;
  }

  bool activate_locked(LogQueue&) {
    if (!connected.load(memory_order_acquire)) return fail("The gripper is not connected.");
    stop_driving();  // first, so a refusal leaves the jaw released
    const auto bus_lock = lock_bus();
    if (std::string error = reset_servos(); !error.empty()) return fail(error);
    (void)bus.sync_read_feedback(ids, feedback);
    for (std::size_t i = 0; i < count(); ++i) {
      const JointConfig& joint = config.joints[i];
      JointBook& b = book[i];
      if (i >= feedback.size() || !feedback[i] || !read_byte(i, feetech::reg::kOperatingMode)) {
        return fail(servo(i) + " did not reply.");
      }
      // A servo in wheel or PWM mode ignores Goal_Position and turns.
      if (reply[0] != feetech::kPositionMode) return fail(servo(i) + " is not in position mode.");
      record(i, *feedback[i]);
      b.read_ok = true;
      b.hot_streak = 0;
      b.state.torque_enabled = b.torque_on;
      publish(i);
      const double position = b.state.position;
      if (b.state.temperature >= kMaxTemperature) {
        return fail(servo(i) + format(" is at %d C. Let it cool below %d C.", b.state.temperature,
                                      kMaxTemperature));
      }
      // A jaw outside its travel means a wrong calibration, a wrong servo, or a jam. A goal
      // clamped from far away would drive it at full torque.
      if (position < joint.lower_limit - kTravelTolerance ||
          position > joint.upper_limit + kTravelTolerance) {
        return fail(servo(i) + format(" is at %.3f rad, outside its travel.", position));
      }
      const double seed = std::clamp(position, joint.lower_limit, joint.upper_limit);
      if (!write_goal(i, goal_counts(i, seed))) return fail(servo(i) + " did not take the goal.");
      joints[i].command.store(seed, memory_order_relaxed);
      joints[i].command_is_new.store(false, memory_order_relaxed);
      // This drops the torque requests from before this point, also those from other threads.
      joints[i].torque_request.store(false, memory_order_release);
    }
    read_failures = 0;
    next_reset_check = Clock::now() + kResetCheckInterval;
    fault.store(0, memory_order_release);
    {
      const std::scoped_lock text(text_mutex);
      fault_message.clear();
      last_error.clear();
    }
    active.store(true, memory_order_release);
    return true;
  }

  bool deactivate_locked(LogQueue& logs) {
    stop_driving();
    if (!connected.load(memory_order_acquire)) return true;
    const auto bus_lock = lock_bus();
    return report(logs, release_all(1));
  }

  void disconnect_quietly() noexcept {
    try {
      lifecycle("disconnect", &Impl::disconnect_locked);
    } catch (...) {  // ~Impl still stops the I/O thread
    }
  }
};

Gripper::Gripper() : impl_(std::make_unique<Impl>()) {}

Gripper::~Gripper() {
  if (impl_) impl_->disconnect_quietly();
}

Gripper::Gripper(Gripper&& other) noexcept = default;

Gripper& Gripper::operator=(Gripper&& other) noexcept {
  if (this == &other) return *this;
  if (impl_) impl_->disconnect_quietly();
  impl_ = std::move(other.impl_);
  return *this;
}

bool Gripper::validate(const GripperConfig& config, std::string& error) {
  const auto bad = [&error](std::string message) {
    error = std::move(message);
    return false;
  };
  const std::size_t n = config.joints.size();
  if (config.port.empty()) return bad("port is empty.");
  if (n == 0 || n > kMaxJoints) return bad(format("joints needs 1 to %zu entries.", kMaxJoints));
  if (config.baud_rate <= 0) return bad("baud_rate must be positive.");
  if (config.bus_rate_hz <= 0) return bad("bus_rate_hz must be positive.");
  for (std::size_t i = 0; i < n; ++i) {
    const JointConfig& joint = config.joints[i];
    const double lower = joint.lower_limit, upper = joint.upper_limit;
    const std::string at = "joint '" + joint.name + "': ";
    // Goals are clamped to one encoder turn, so the travel must not cross the wrap.
    const auto on_turn = [&joint](double limit) {
      const double counts = to_counts(joint, limit);
      return counts >= -0.5 && counts < feetech::kCountsPerTurn - 0.5;
    };
    if (joint.name.empty()) return bad(format("joint %zu has no name.", i));
    if (joint.id > feetech::kMaxServoId) return bad(at + format("id %d is too high.", joint.id));
    if (std::abs(joint.direction) != 1.0) return bad(at + "direction must be 1 or -1.");
    if (!std::isfinite(joint.homing_offset)) return bad(at + "homing_offset must be finite.");
    if (joint.torque_limit < 1 || joint.torque_limit > feetech::kMaxTorqueLimit) {
      return bad(at + "torque_limit must be 1 to 1000.");
    }
    if (!std::isfinite(lower) || !std::isfinite(upper)) return bad(at + "limits must be finite.");
    if (lower >= upper) return bad(at + "lower_limit must be below upper_limit.");
    if (!on_turn(lower) || !on_turn(upper)) return bad(at + "the travel crosses the encoder wrap.");
    for (std::size_t j = 0; j < i; ++j) {
      const JointConfig& other = config.joints[j];
      if (other.id == joint.id) return bad(format("servo id %d is used twice.", joint.id));
      if (other.name == joint.name) return bad(at + "the name is used twice.");
    }
  }
  return true;
}

bool Gripper::connect(const GripperConfig& config) {
  return impl_ && impl_->lifecycle("connect", &Impl::connect_locked, config);
}
bool Gripper::disconnect() {
  return !impl_ || impl_->lifecycle("disconnect", &Impl::disconnect_locked);
}
bool Gripper::activate() { return impl_ && impl_->lifecycle("activate", &Impl::activate_locked); }
bool Gripper::deactivate() {
  return !impl_ || impl_->lifecycle("deactivate", &Impl::deactivate_locked);
}

Mode Gripper::mode() const noexcept {
  if (!is_connected()) return Mode::kDisconnected;
  if (impl_->faulted()) return Mode::kFaulted;
  return impl_->active.load(memory_order_acquire) ? Mode::kActive : Mode::kConnected;
}
bool Gripper::is_connected() const noexcept {
  return impl_ && impl_->connected.load(memory_order_acquire);
}
bool Gripper::is_active() const noexcept { return mode() == Mode::kActive; }
bool Gripper::faulted() const noexcept { return impl_ && impl_->faulted(); }
Fault Gripper::fault() const noexcept {
  return impl_ ? static_cast<Fault>(impl_->fault.load(memory_order_acquire)) : Fault::kNone;
}
std::string Gripper::fault_message() const {
  return impl_ ? copy_under(impl_->text_mutex, impl_->fault_message) : std::string();
}

void Gripper::set_target(std::size_t joint, double radians) noexcept {
  JointIo* io = Impl::io(impl_, joint);
  if (io && std::isfinite(radians)) {
    io->command.store(radians, memory_order_relaxed);
    io->command_is_new.store(true, memory_order_relaxed);
  }
}

void Gripper::set_torque_enabled(std::size_t joint, bool enabled) noexcept {
  JointIo* io = Impl::io(impl_, joint);
  if (!io) return;
  // An old target is replaced by the present position, so a jaw moved by hand does not snap
  // back. The reseed goes before the request, so the I/O thread never enables with an old target.
  const double present = io->present.load(memory_order_acquire);
  const bool command_is_new = io->command_is_new.exchange(false, memory_order_relaxed);
  if (enabled && !io->torque_request.load(memory_order_relaxed) && !command_is_new &&
      std::isfinite(present)) {
    io->command.store(present, memory_order_relaxed);
  }
  io->torque_request.store(enabled, memory_order_release);
}

void Gripper::set_torque_limit(std::size_t joint, double fraction) noexcept {
  JointIo* io = Impl::io(impl_, joint);
  if (io && std::isfinite(fraction)) {
    io->torque_fraction.store(std::clamp(fraction, 0.0, 1.0), memory_order_relaxed);
  }
}

JointState Gripper::state(std::size_t joint) const noexcept {
  const JointIo* io = Impl::io(impl_, joint);
  return io ? io->read() : JointState{};
}

std::size_t Gripper::joint_count() const noexcept {
  return impl_ ? impl_->joint_count.load(memory_order_acquire) : 0;
}

std::optional<std::size_t> Gripper::joint_index(const std::string& name) const {
  const auto joints = config().joints;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].name == name) return i;
  }
  return std::nullopt;
}

GripperConfig Gripper::config() const {
  return impl_ ? copy_under(impl_->config_mutex, impl_->config) : GripperConfig();
}

std::string Gripper::last_error() const {
  return impl_ ? copy_under(impl_->text_mutex, impl_->last_error) : "this Gripper was moved from";
}

}  // namespace specialist
