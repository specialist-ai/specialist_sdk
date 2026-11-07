// Gripper against fake servos on a pty. No hardware.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_servo.hpp"
#include "feetech/protocol.hpp"
#include "specialist/gripper.hpp"
#include "specialist/log.hpp"

namespace {

using namespace std::chrono_literals;
using specialist::Fault;
using specialist::Gripper;
using specialist::LogLevel;
using specialist::Mode;

constexpr std::uint8_t kId = 18, kLeftId = 28;
constexpr int kCenter = 2048;  // 0 rad
constexpr double kOffset = kCenter * feetech::kRadiansPerCount;
constexpr double kLower = -0.698132, kUpper = 0.261799;
constexpr int kTorqueEnable = 40, kGoalBlock = 41, kGoal = 42, kTorqueLimit = 48;
constexpr int kDefaultLimit = 667;

int counts_at(double radians) {
  return static_cast<int>(std::lround((radians + kOffset) / feetech::kRadiansPerCount));
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

specialist::JointConfig make_joint(const std::string& name, std::uint8_t id) {
  return {name, id, 1.0, kOffset, kLower, kUpper, kDefaultLimit};
}

specialist::GripperConfig make_config(const std::string& port) {
  specialist::GripperConfig config;
  config.port = port;
  config.joints = {make_joint("right_finger", kId)};
  return config;
}

template <typename F>
bool wait_for(F condition, std::chrono::milliseconds timeout = 3000ms) {
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end && !condition()) std::this_thread::sleep_for(1ms);
  return condition();
}

// One servo at 0 rad behind a pty, and a log capture. The destructor body stops the I/O thread
// before the members go.
class GripperTest : public ::testing::Test {
 protected:
  struct Capture {
    std::mutex mutex;
    std::vector<std::pair<LogLevel, std::string>> messages;
    std::function<void(LogLevel, const std::string&)> hook;  // set it before connect()
  };

  GripperTest() {
    servos.add(kId, kCenter);
    specialist::set_log_handler([capture = capture_](LogLevel level, const std::string& message) {
      {
        const std::lock_guard<std::mutex> lock(capture->mutex);
        capture->messages.emplace_back(level, message);
      }
      if (capture->hook) capture->hook(level, message);
    });
  }
  ~GripperTest() override {
    gripper.disconnect();
    specialist::reset_log_handler();
  }

  bool logged(LogLevel level, const std::string& text) {
    const std::lock_guard<std::mutex> lock(capture_->mutex);
    for (const auto& [l, m] : capture_->messages) {
      if (l == level && contains(m, text)) return true;
    }
    return false;
  }
  void connect() { ASSERT_TRUE(gripper.connect(config)) << gripper.last_error(); }
  void activate() { ASSERT_TRUE(gripper.activate()) << gripper.last_error(); }
  void enable() {
    connect();
    activate();
    gripper.set_torque_enabled(0, true);
    ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  }
  // Waits for `count` more SYNC_READs.
  void reads(int count) {
    const int start = servos.sync_reads;
    ASSERT_TRUE(wait_for([&] { return servos.sync_reads - start >= count; }));
  }
  int torque() { return servos.reg_u8(kId, kTorqueEnable); }
  int limit() { return servos.reg_u16(kId, kTorqueLimit); }
  int goal() { return servos.reg_u16(kId, kGoal); }
  bool error_has(const std::string& part) { return contains(gripper.last_error(), part); }

  std::shared_ptr<Capture> capture_ = std::make_shared<Capture>();
  specialist_test::FakeServos servos;
  specialist::GripperConfig config = make_config(servos.path());
  Gripper gripper;
};

}  // namespace

TEST_F(GripperTest, LifecycleKeepsTheTorqueOffUntilRequested) {
  EXPECT_EQ(gripper.mode(), Mode::kDisconnected);
  EXPECT_FALSE(gripper.activate());
  EXPECT_TRUE(error_has("not connected"));
  servos.set_u8(kId, kTorqueEnable, 1);  // a crashed run left it on
  connect();
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(limit(), kDefaultLimit);
  EXPECT_EQ(gripper.mode(), Mode::kConnected);
  servos.set_position(kId, counts_at(-0.2));
  servos.set_u16(kId, kGoal, counts_at(0.2));
  gripper.set_torque_enabled(0, true);  // activate() drops a request from before it
  activate();
  EXPECT_EQ(gripper.mode(), Mode::kActive);
  EXPECT_TRUE(gripper.is_active());
  EXPECT_EQ(goal(), counts_at(-0.2));
  reads(5);
  EXPECT_EQ(torque(), 0);
  EXPECT_TRUE(gripper.deactivate());
  EXPECT_EQ(gripper.mode(), Mode::kConnected);
  EXPECT_TRUE(gripper.disconnect());
  EXPECT_EQ(gripper.mode(), Mode::kDisconnected);
  EXPECT_FALSE(gripper.state(0).valid);
  EXPECT_TRUE(gripper.disconnect());
}

TEST_F(GripperTest, ActivateChecksTheTravelAndTheTargetIsClampedIntoIt) {
  for (const double outside : {kLower - 0.1, kUpper + 0.1}) {
    servos.add(kId, counts_at(outside));
    connect();
    EXPECT_FALSE(gripper.activate());
    EXPECT_TRUE(error_has("outside its travel")) << gripper.last_error();
    EXPECT_EQ(gripper.mode(), Mode::kConnected);
  }
  servos.add(kId, counts_at(kLower - 0.03));  // inside the finger play
  connect();
  activate();
  EXPECT_EQ(goal(), counts_at(kLower));
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  EXPECT_EQ(goal(), counts_at(kLower));
  gripper.set_target(0, 10.0);
  EXPECT_TRUE(wait_for([&] { return goal() == counts_at(kUpper); }));
  gripper.set_target(0, -10.0);
  EXPECT_TRUE(wait_for([&] { return goal() == counts_at(kLower); }));
}

TEST_F(GripperTest, TorqueOnWritesTheLimitThenTheGoalThenTheTorque) {
  connect();
  activate();
  servos.set_u16(kId, kTorqueLimit, 1000);
  const auto earlier = servos.writes().size();
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  const auto writes = servos.writes();
  ASSERT_GE(writes.size(), earlier + 3);
  EXPECT_EQ(writes[earlier].address, kTorqueLimit);
  EXPECT_EQ(writes[earlier + 1].address, kGoalBlock);
  EXPECT_EQ(writes[earlier + 2].address, kTorqueEnable);
  EXPECT_EQ(limit(), kDefaultLimit);
  EXPECT_TRUE(wait_for([&] { return gripper.state(0).torque_enabled; }));
  gripper.set_target(0, -0.3);
  ASSERT_TRUE(wait_for([&] { return goal() == counts_at(-0.3); }));
  gripper.set_torque_enabled(0, false);
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }));
  servos.set_position(kId, counts_at(-0.1));  // moved by hand
  ASSERT_TRUE(wait_for([&] { return std::abs(gripper.state(0).position + 0.1) < 0.002; }));
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  reads(5);
  EXPECT_EQ(goal(), counts_at(-0.1)) << "the jaw snapped back to the old target";
  gripper.set_torque_enabled(0, false);
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }));
  gripper.set_target(0, 0.1);  // a new target, then torque on: it is kept
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  EXPECT_EQ(goal(), counts_at(0.1));
  gripper.set_torque_enabled(0, false);
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }));
  gripper.set_target(0, -0.2);
  activate();  // drops the target from before it
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  reads(5);
  EXPECT_EQ(goal(), counts_at(-0.1));  // the present position; the fake does not follow the goal
}

TEST_F(GripperTest, TheTorqueLimitIsCappedAndBadInputIsIgnored) {
  config.joints[0].torque_limit = 400;
  enable();
  const auto limit_becomes = [&](int permille) {
    return wait_for([&] { return limit() == permille; });
  };
  EXPECT_EQ(limit(), 400);
  gripper.set_torque_limit(0, 0.25);
  EXPECT_TRUE(limit_becomes(250));
  gripper.set_torque_limit(0, 0.9);
  EXPECT_TRUE(limit_becomes(400));
  gripper.set_torque_limit(0, std::nan(""));
  gripper.set_torque_limit(5, 0.1);
  gripper.set_target(0, std::nan(""));
  gripper.set_target(0, std::numeric_limits<double>::infinity());
  gripper.set_target(1, -0.5);
  gripper.set_torque_enabled(3, true);
  reads(5);
  EXPECT_EQ(limit(), 400);
  EXPECT_EQ(goal(), kCenter);
  gripper.set_torque_limit(0, -1.0);
  EXPECT_TRUE(limit_becomes(0));
}

TEST_F(GripperTest, OverTemperatureFaultsAfter25HotReadsAndReleasesBeforeTheLog) {
  const auto torque_at_log = std::make_shared<std::atomic<int>>(-1);
  capture_->hook = [this, torque_at_log](LogLevel level, const std::string&) {
    if (level == LogLevel::kError && *torque_at_log < 0) *torque_at_log = torque();
  };
  enable();
  servos.hot_reads(24, 70);  // one short
  ASSERT_TRUE(wait_for([&] { return servos.hot_reads_left() == 0; }));
  reads(3);
  EXPECT_FALSE(gripper.faulted());
  servos.hot_reads(25, 70);
  ASSERT_TRUE(wait_for([&] { return gripper.faulted(); }));
  EXPECT_EQ(gripper.fault(), Fault::kOverTemperature);
  EXPECT_EQ(gripper.mode(), Mode::kFaulted);
  ASSERT_TRUE(wait_for([&] { return *torque_at_log >= 0; }));
  EXPECT_EQ(torque_at_log->load(), 0) << "the log came before the release";
  EXPECT_TRUE(contains(gripper.fault_message(), "servo id 18 held 70 C for 25 reads"))
      << gripper.fault_message();
  servos.set_temperature(kId, 70);
  EXPECT_FALSE(gripper.activate());
  EXPECT_TRUE(error_has("Let it cool")) << gripper.last_error();
  EXPECT_EQ(torque(), 0);
  servos.set_temperature(kId, 40);
  activate();
  EXPECT_FALSE(gripper.faulted());
}

TEST_F(GripperTest, BusLostAfter20FailedReadsAndTheFaultLatchesUntilActivate) {
  enable();
  servos.drop_sync_reads = 19;
  ASSERT_TRUE(wait_for([&] { return servos.drop_sync_reads == 0; }));
  reads(3);
  EXPECT_FALSE(gripper.faulted());
  servos.drop_sync_reads = 20;
  ASSERT_TRUE(wait_for([&] { return gripper.faulted(); }));
  EXPECT_EQ(gripper.fault(), Fault::kBusLost);
  EXPECT_TRUE(wait_for([&] { return torque() == 0; }, 500ms));
  EXPECT_TRUE(wait_for([&] { return logged(LogLevel::kError, "bus lost"); }, 500ms));
  gripper.set_torque_enabled(0, false);
  gripper.set_torque_enabled(0, true);
  reads(10);
  EXPECT_EQ(torque(), 0);
  EXPECT_TRUE(gripper.deactivate());
  EXPECT_TRUE(gripper.disconnect());
  connect();
  EXPECT_EQ(gripper.mode(), Mode::kFaulted);
  EXPECT_FALSE(gripper.fault_message().empty());
  activate();
  EXPECT_EQ(gripper.mode(), Mode::kActive);
  EXPECT_EQ(gripper.fault(), Fault::kNone);
  EXPECT_TRUE(gripper.fault_message().empty());
}

TEST_F(GripperTest, AServoResetFaultsAndActivateRewritesTheLimit) {
  config.joints[0].torque_limit = 300;
  enable();
  EXPECT_EQ(limit(), 300);
  servos.reset(kId);  // a brown-out while the torque is on
  ASSERT_TRUE(wait_for([&] { return gripper.faulted(); }, 1000ms));
  EXPECT_EQ(gripper.fault(), Fault::kServoReset);
  EXPECT_TRUE(contains(gripper.fault_message(), "restarted")) << gripper.fault_message();
  reads(10);
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(limit(), 1000);
  activate();
  EXPECT_EQ(limit(), 300);
  gripper.set_torque_enabled(0, true);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  reads(40);  // several Torque_Enable checks
  EXPECT_FALSE(gripper.faulted());
}

TEST_F(GripperTest, TheReleaseIsResentUntilTheServoAcknowledgesIt) {
  enable();
  servos.deaf = true;  // a cable glitch: nothing gets through
  ASSERT_TRUE(
      wait_for([&] { return gripper.fault() == (Fault::kBusLost | Fault::kReleaseFailed); }));
  EXPECT_EQ(torque(), 1);
  std::this_thread::sleep_for(150ms);
  servos.deaf = false;
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }, 1000ms));
  EXPECT_TRUE(gripper.faulted());
  EXPECT_TRUE(wait_for([&] { return logged(LogLevel::kInfo, "released its torque"); }));
  EXPECT_TRUE(wait_for([&] { return gripper.state(0).valid && !gripper.state(0).torque_enabled; }));
}

TEST_F(GripperTest, DisconnectTheDestructorAndAMoveReleaseTheTorque) {
  enable();
  EXPECT_TRUE(gripper.disconnect());
  EXPECT_EQ(torque(), 0);
  {
    Gripper local;
    ASSERT_TRUE(local.connect(config) && local.activate());
    local.set_torque_enabled(0, true);
    ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  }
  EXPECT_EQ(torque(), 0);
  enable();
  Gripper second(std::move(gripper));
  EXPECT_EQ(second.mode(), Mode::kActive);
  EXPECT_EQ(torque(), 1);
  second = Gripper();
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(second.mode(), Mode::kDisconnected);
}

TEST_F(GripperTest, EveryLifecycleCallReportsAnUnacknowledgedRelease) {
  connect();
  servos.torque_stuck_on = true;  // acknowledges the write and keeps the torque on
  servos.set_u8(kId, kTorqueEnable, 1);
  EXPECT_FALSE(gripper.activate());
  EXPECT_TRUE(error_has("reads Torque_Enable 1")) << gripper.last_error();
  EXPECT_FALSE(gripper.connect(config));
  servos.torque_stuck_on = false;
  enable();
  servos.deaf = true;
  EXPECT_FALSE(gripper.deactivate());
  EXPECT_TRUE(error_has("may still be on")) << gripper.last_error();
  EXPECT_FALSE(gripper.disconnect());
  EXPECT_TRUE(error_has("may still be on")) << gripper.last_error();
  EXPECT_EQ(gripper.mode(), Mode::kDisconnected);
}

TEST_F(GripperTest, StateReportsEveryField) {
  servos.add(kId, counts_at(-0.25), 41);
  servos.set_u16(kId, 60, 300);  // Present_Load, permille
  servos.set_u8(kId, 65, 0x04);  // Status
  servos.set_u16(kId, 69, 100);  // Present_Current, 6.5 mA a count
  EXPECT_FALSE(gripper.state(0).valid);
  connect();
  ASSERT_TRUE(wait_for([&] { return gripper.state(0).valid; }));
  const auto s = gripper.state(0);
  EXPECT_NEAR(s.position, -0.25, 0.002);
  EXPECT_EQ(s.temperature, 41);
  EXPECT_NEAR(s.load, 0.3, 1e-9);
  EXPECT_NEAR(s.current, 0.65, 1e-9);
  EXPECT_EQ(s.servo_status, 0x04);
  EXPECT_FALSE(s.torque_enabled);
  EXPECT_GT(s.stamp_ns, 0);
  EXPECT_TRUE(wait_for([&] { return gripper.state(0).sequence > s.sequence; }));
  EXPECT_FALSE(gripper.state(1).valid);
  servos.drop_sync_reads = 2;
  ASSERT_TRUE(wait_for([&] { return !gripper.state(0).valid; }));
  EXPECT_NEAR(gripper.state(0).position, -0.25, 0.002) << "a failed read keeps the last values";
  EXPECT_EQ(gripper.joint_count(), 1u);
  EXPECT_EQ(gripper.joint_index("right_finger"), std::optional<std::size_t>(0));
  EXPECT_FALSE(gripper.joint_index("thumb").has_value());
  EXPECT_EQ(gripper.config().port, servos.path());
}

TEST_F(GripperTest, StateSamplesAreNeverTorn) {
  servos.ramp = true;
  connect();
  ASSERT_TRUE(wait_for([&] { return gripper.state(0).valid; }));
  std::atomic<bool> stop{false};
  std::atomic<long> samples{0}, torn{0};
  const auto reader = [&] {
    while (!stop) {
      const auto s = gripper.state(0);
      if (!s.valid) continue;
      const int counts = counts_at(s.position);
      torn += s.temperature != 20 + counts % 30 || std::lround(s.load * 1000) != counts % 500;
      ++samples;
    }
  };
  std::thread first(reader), second(reader);
  std::this_thread::sleep_for(300ms);
  stop = true;
  first.join();
  second.join();
  EXPECT_GT(samples.load(), 1000);
  EXPECT_EQ(torn.load(), 0);
}

TEST_F(GripperTest, ALifecycleCallFromTheLogHandlerIsRefused) {
  const auto refused = std::make_shared<std::atomic<int>>(0);
  capture_->hook = [this, refused](LogLevel level, const std::string&) {
    if (level == LogLevel::kError && !gripper.disconnect()) ++*refused;
  };
  enable();
  servos.drop_sync_reads = 20;  // the bus-lost fault logs on the I/O thread
  ASSERT_TRUE(wait_for([&] { return *refused > 0; }));
  EXPECT_TRUE(gripper.is_connected());
}

TEST(Validate, RefusesEachBadField) {
  std::string error;
  EXPECT_TRUE(Gripper::validate(make_config("/dev/null"), error)) << error;
  using Edit = std::function<void(specialist::GripperConfig&)>;
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  const std::pair<Edit, std::string> cases[] = {
      {[](auto& c) { c.port.clear(); }, "port"},
      {[](auto& c) { c.joints.clear(); }, "joints"},
      {[](auto& c) { c.joints.resize(specialist::kMaxJoints + 1, c.joints[0]); }, "joints"},
      {[](auto& c) { c.baud_rate = 0; }, "baud_rate"},
      {[](auto& c) { c.bus_rate_hz = -1; }, "bus_rate_hz"},
      {[](auto& c) { c.joints[0].name.clear(); }, "no name"},
      {[](auto& c) { c.joints[0].id = 254; }, "id 254"},
      {[](auto& c) { c.joints[0].direction = 0.0; }, "direction"},
      {[nan](auto& c) { c.joints[0].homing_offset = nan; }, "homing_offset"},
      {[](auto& c) { c.joints[0].torque_limit = 0; }, "torque_limit"},
      {[](auto& c) { c.joints[0].torque_limit = 1001; }, "torque_limit"},
      {[nan](auto& c) { c.joints[0].lower_limit = nan; }, "finite"},
      {[inf](auto& c) { c.joints[0].upper_limit = inf; }, "finite"},
      {[](auto& c) { c.joints[0].lower_limit = c.joints[0].upper_limit; }, "below"},
      {[](auto& c) { c.joints[0].homing_offset = 0.0; }, "encoder"},
      {[](auto& c) { c.joints.push_back(make_joint("left_finger", kId)); }, "servo id 18"},
      {[](auto& c) { c.joints.push_back(make_joint("right_finger", kLeftId)); }, "used twice"},
  };
  for (const auto& [edit, expected] : cases) {
    auto config = make_config("/dev/null");
    edit(config);
    EXPECT_FALSE(Gripper::validate(config, error)) << "expected a refusal naming " << expected;
    EXPECT_TRUE(contains(error, expected)) << error;
  }
}

TEST(FaultBits, Combine) {
  using specialist::has;
  const Fault both = Fault::kBusLost | Fault::kReleaseFailed;
  EXPECT_TRUE(has(both, Fault::kBusLost) && has(both, Fault::kReleaseFailed));
  EXPECT_FALSE(has(both, Fault::kOverTemperature) || has(both, Fault::kNone));
  Fault set = Fault::kNone;
  EXPECT_EQ(set |= Fault::kServoReset, Fault::kServoReset);
}
