// Grippers drives the grippers of a robot, one Feetech STS servo each, on one or more serial
// ports. A gripper's value is its jaw openness in rad, positive opening:
// openness = raw_counts * 2 pi / 4096 / direction + offset, as ros2_control's SimpleTransmission.
// The noexcept methods do not lock or allocate. A fault on any gripper releases them all and
// latches until the next connect().

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace specialist {

struct GripperConfig {
  std::string name;
  std::string port;  // grippers on one port share its bus
  std::uint8_t id{0};
  double closed{std::numeric_limits<double>::quiet_NaN()};     // rad
  double open{std::numeric_limits<double>::quiet_NaN()};       // rad, the open stop
  int torque_limit{667};                                       // permille of stall
  double direction{std::numeric_limits<double>::quiet_NaN()};  // +1 or -1, from the calibration
  double offset{std::numeric_limits<double>::quiet_NaN()};     // rad of openness, from calibration
};

struct GripperState {
  double openness{0.0};        // rad
  double velocity{0.0};        // rad/s
  double load{0.0};            // -1 to 1 of the maximum PWM duty, positive when opening
  double current{0.0};         // A
  int temperature{0};          // C
  bool torque_enabled{false};  // the last Torque_Enable value the servo acknowledged
  bool valid{false};           // false after a failed read; the other fields keep the last read
  std::uint64_t sequence{0};   // never resets
};

namespace detail {
struct GripperIo;
}

/// One gripper of a Grippers. It must not outlive it.
class Gripper {
 public:
  /// rad, clamped into [closed, open]. Turns the torque on, unless faulted.
  void set_target(double openness) const noexcept;
  /// Turns the torque off. The driver resends it until the servo acknowledges it.
  void release() const noexcept;
  GripperState state() const noexcept;

 private:
  friend class Grippers;
  explicit Gripper(detail::GripperIo& io) noexcept : io_(&io) {}
  detail::GripperIo* io_;
};

class Grippers {
 public:
  /// Throws std::invalid_argument for a config it refuses to drive. It opens no port.
  explicit Grippers(std::vector<GripperConfig> config);
  ~Grippers();  // releases the torque
  Grippers(const Grippers&) = delete;
  Grippers& operator=(const Grippers&) = delete;

  /// Opens each port and releases every torque, then refuses a servo that is silent, hot, not in
  /// position mode, or outside its travel. Clears a fault. Every torque stays off.
  [[nodiscard]] bool connect();
  /// False when a servo did not acknowledge the torque release, which also goes to stderr.
  bool disconnect();
  bool faulted() const noexcept;
  /// Why connect() or disconnect() failed, or why the grippers faulted.
  std::string error() const;
  std::optional<Gripper> gripper(const std::string& name);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace specialist
