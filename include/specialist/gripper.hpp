// Each joint value is the jaw openness in rad, and a positive change opens the jaw:
// openness = direction * (raw_counts * 2 pi / 4096 - homing_offset). The noexcept methods do not
// lock or allocate, and any thread may call them. The lifecycle methods block on bus I/O. A fault
// releases the torque and latches until activate().

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace specialist {

constexpr std::size_t kMaxJoints = 8;

struct JointConfig {
  std::string name;
  std::uint8_t id{0};
  double direction{1.0};                                         // +1 or -1
  double homing_offset{0.0};                                     // rad
  double lower_limit{std::numeric_limits<double>::quiet_NaN()};  // rad, the closed side
  double upper_limit{std::numeric_limits<double>::quiet_NaN()};  // rad, the open stop
  int torque_limit{667};                                         // permille of stall
};

struct GripperConfig {
  std::string port;
  int baud_rate{1000000};
  int bus_rate_hz{250};
  std::vector<JointConfig> joints;
};

struct JointState {
  double position{0.0};          // rad
  double velocity{0.0};          // rad/s
  double load{0.0};              // -1 to 1 of the maximum PWM duty, in the joint direction
  double current{0.0};           // A
  int temperature{0};            // C
  std::uint8_t servo_status{0};  // feetech::servo_error bits
  bool torque_enabled{false};    // the last Torque_Enable value the servo acknowledged
  bool valid{false};             // false after a failed read; the other fields keep the last read
  std::uint64_t sequence{0};     // never resets
  std::int64_t stamp_ns{0};      // std::chrono::steady_clock
};

enum class Mode { kDisconnected, kConnected, kActive, kFaulted };

enum class Fault : std::uint32_t {
  kNone = 0,
  kBusLost = 1,
  kOverTemperature = 2,
  kServoReset = 4,    // a servo restarted, or its own protection released the torque
  kReleaseFailed = 8  // a servo did not acknowledge the release; the I/O thread resends it
};

constexpr Fault operator|(Fault a, Fault b) noexcept {
  return Fault{static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)};
}
constexpr Fault operator&(Fault a, Fault b) noexcept {
  return Fault{static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)};
}
constexpr Fault& operator|=(Fault& a, Fault b) noexcept { return a = a | b; }
constexpr bool has(Fault set, Fault bit) noexcept {
  return bit != Fault::kNone && (set & bit) == bit;
}

class Gripper {
 public:
  Gripper();
  ~Gripper();  // releases the torque
  Gripper(const Gripper&) = delete;
  Gripper& operator=(const Gripper&) = delete;
  Gripper(Gripper&& other) noexcept;
  Gripper& operator=(Gripper&& other) noexcept;

  [[nodiscard]] static bool validate(const GripperConfig& config, std::string& error);
  /// Releases any torque that a crashed run left on, then starts the I/O thread.
  [[nodiscard]] bool connect(const GripperConfig& config);
  bool disconnect();  // false when a servo did not acknowledge the torque release
  /// Refuses a hot servo, a non-position mode, or a jaw outside its travel. Torque stays off.
  [[nodiscard]] bool activate();
  bool deactivate();

  Mode mode() const noexcept;
  bool is_connected() const noexcept;
  bool is_active() const noexcept;
  bool faulted() const noexcept;
  Fault fault() const noexcept;
  std::string fault_message() const;

  /// rad. The I/O thread clamps it into the travel.
  void set_target(std::size_t joint, double radians) noexcept;
  /// Off to on keeps a target that set_target() gave after the previous set_torque_enabled() or
  /// activate() call. Otherwise it sets the target to the present position.
  void set_torque_enabled(std::size_t joint, bool enabled) noexcept;
  /// 0 to 1 of stall torque, capped at JointConfig::torque_limit.
  void set_torque_limit(std::size_t joint, double fraction) noexcept;
  JointState state(std::size_t joint) const noexcept;
  std::size_t joint_count() const noexcept;
  std::optional<std::size_t> joint_index(const std::string& name) const;
  GripperConfig config() const;  // as the last connect() validated it
  std::string last_error() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace specialist
