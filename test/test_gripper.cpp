// Grippers against fake servos on a pty. No hardware.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_servo.hpp"
#include "feetech/protocol.hpp"
#include "specialist/gripper.hpp"

namespace {

using namespace std::chrono_literals;
using specialist::GripperConfig;
using specialist::Grippers;

constexpr std::uint8_t kId = 18, kLeftId = 28;
constexpr int kCenter = 2048;  // 0 rad
constexpr double kOffset = -kCenter * feetech::kRadiansPerCount;
constexpr double kClosed = -0.261799, kOpen = 0.698132;
using feetech::reg::kTorqueEnable, feetech::reg::kTorqueLimit;
constexpr auto kGoalBlock = feetech::reg::kAcceleration, kGoal = feetech::reg::kGoalPosition;
constexpr int kDefaultLimit = 667;

int counts_at(double radians) {
  return static_cast<int>(std::lround((radians - kOffset) / feetech::kRadiansPerCount));
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

GripperConfig make_gripper(const std::string& name, const std::string& port, std::uint8_t id,
                           double closed = kClosed, double open = kOpen) {
  return {name, port, id, closed, open, kDefaultLimit, 1.0, kOffset};
}

template <typename F>
bool wait_for(F condition, std::chrono::milliseconds timeout = 3000ms) {
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end && !condition()) std::this_thread::sleep_for(1ms);
  return condition();
}

// One servo at 0 rad behind a pty. grippers() makes the driver from `config` on first use.
class GrippersTest : public ::testing::Test {
 protected:
  GrippersTest() { servos.add(kId, kCenter); }

  Grippers& grippers() {
    if (!driver) driver = std::make_unique<Grippers>(config);
    return *driver;
  }
  void connect() { ASSERT_TRUE(grippers().connect()) << grippers().error(); }
  void enable() {
    connect();
    jaw().set_target(0.0);
    ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  }
  specialist::Gripper jaw(const std::string& name = "right_gripper") {
    return grippers().gripper(name).value();
  }
  // Waits for `count` more SYNC_READs.
  void reads(int count) {
    const int start = servos.sync_reads;
    ASSERT_TRUE(wait_for([&] { return servos.sync_reads - start >= count; }));
  }
  int torque(std::uint8_t id = kId) { return servos.reg_u8(id, kTorqueEnable); }
  int limit() { return servos.reg_u16(kId, kTorqueLimit); }
  int goal(std::uint8_t id = kId) { return servos.reg_u16(id, kGoal); }
  bool error_has(const std::string& part) { return contains(grippers().error(), part); }

  specialist_test::FakeServos servos;
  std::vector<GripperConfig> config{make_gripper("right_gripper", servos.path(), kId)};
  std::unique_ptr<Grippers> driver;
};

}  // namespace

TEST_F(GrippersTest, ConnectReleasesTheTorqueAndKeepsItOffUntilATarget) {
  servos.set_u8(kId, kTorqueEnable, 1);  // a crashed run left it on
  jaw().set_target(0.2);                 // connect() drops a request from before it
  connect();
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(limit(), kDefaultLimit);
  EXPECT_FALSE(grippers().faulted());
  reads(5);
  EXPECT_EQ(torque(), 0);
  EXPECT_TRUE(grippers().disconnect());
  EXPECT_FALSE(jaw().state().valid);
  EXPECT_TRUE(grippers().disconnect());
}

TEST_F(GrippersTest, ConnectChecksTheTravelTheModeAndTheTemperature) {
  for (const double outside : {kClosed - 0.06, kOpen + 0.06}) {
    servos.add(kId, counts_at(outside));
    EXPECT_FALSE(grippers().connect());
    EXPECT_TRUE(error_has("outside its travel")) << grippers().error();
    EXPECT_FALSE(jaw().state().valid);
  }
  servos.add(kId, counts_at(kClosed - 0.03));  // inside the play
  servos.set_u8(kId, feetech::reg::kOperatingMode, 1);
  EXPECT_FALSE(grippers().connect());
  EXPECT_TRUE(error_has("position mode")) << grippers().error();
  servos.set_u8(kId, feetech::reg::kOperatingMode, feetech::kPositionMode);
  servos.set_temperature(kId, 65);
  EXPECT_FALSE(grippers().connect());
  EXPECT_TRUE(error_has("Let it cool")) << grippers().error();
  servos.set_temperature(kId, 64);
  connect();
}

TEST_F(GrippersTest, ATargetWritesTheLimitThenTheGoalThenTheTorque) {
  connect();
  servos.set_u16(kId, kTorqueLimit, 1000);
  const auto earlier = servos.writes().size();
  jaw().set_target(0.5);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  const auto writes = servos.writes();
  ASSERT_GE(writes.size(), earlier + 3);
  EXPECT_EQ(writes[earlier].address, kTorqueLimit);
  EXPECT_EQ(writes[earlier + 1].address, kGoalBlock);
  EXPECT_EQ(writes[earlier + 2].address, kTorqueEnable);
  EXPECT_EQ(limit(), kDefaultLimit);
  EXPECT_EQ(goal(), counts_at(0.5));
  EXPECT_TRUE(wait_for([&] { return jaw().state().torque_enabled; }));
  jaw().set_target(10.0);
  EXPECT_TRUE(wait_for([&] { return goal() == counts_at(kOpen); }));
  jaw().set_target(std::nan(""));
  reads(5);
  EXPECT_EQ(goal(), counts_at(kOpen));
  jaw().release();
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }));
}

TEST_F(GrippersTest, OverTemperatureFaultsAfter25HotReadsUntilConnect) {
  enable();
  servos.hot_reads(24, 65);  // one short
  ASSERT_TRUE(wait_for([&] { return servos.hot_reads_left() == 0; }));
  reads(3);
  EXPECT_FALSE(grippers().faulted());
  servos.hot_reads(25, 65);
  ASSERT_TRUE(wait_for([&] { return error_has("Torque released."); })) << grippers().error();
  EXPECT_TRUE(grippers().faulted());
  EXPECT_TRUE(error_has("servo id 18 held 65 C for 25 reads.")) << grippers().error();
  EXPECT_EQ(torque(), 0);
  jaw().set_target(0.1);
  reads(10);
  EXPECT_EQ(torque(), 0);
  connect();
  EXPECT_FALSE(grippers().faulted());
  EXPECT_TRUE(grippers().error().empty());
}

TEST_F(GrippersTest, BusLostAfter20FailedReads) {
  enable();
  for (int round = 0; round < 2; ++round) {  // a good read starts the count again
    servos.drop_sync_reads = 19;
    ASSERT_TRUE(wait_for([&] { return servos.drop_sync_reads == 0; }));
    reads(3);
    EXPECT_FALSE(grippers().faulted());
  }
  servos.drop_sync_reads = 20;
  ASSERT_TRUE(wait_for([&] { return grippers().faulted(); }));
  EXPECT_TRUE(error_has("bus lost after 20 failed reads")) << grippers().error();
  EXPECT_TRUE(wait_for([&] { return torque() == 0; }, 500ms));
}

TEST_F(GrippersTest, AServoResetFaultsAndConnectRewritesTheLimit) {
  config[0].torque_limit = 300;
  enable();
  EXPECT_EQ(limit(), 300);
  servos.reset(kId);  // a brown-out while the torque is on
  ASSERT_TRUE(wait_for([&] { return grippers().faulted(); }, 1000ms));
  EXPECT_TRUE(error_has("restarted")) << grippers().error();
  reads(10);
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(limit(), 1000);
  enable();
  EXPECT_EQ(limit(), 300);
  reads(40);  // at least one Torque_Enable check
  EXPECT_FALSE(grippers().faulted());
}

TEST_F(GrippersTest, TheReleaseIsResentUntilTheServoAcknowledgesIt) {
  enable();
  servos.deaf = true;  // a cable glitch: nothing gets through
  ASSERT_TRUE(wait_for([&] { return error_has("The driver resends the release."); }));
  EXPECT_TRUE(error_has("bus lost")) << grippers().error();
  EXPECT_EQ(torque(), 1);
  const int sent = servos.ignored_writes;  // a silent servo keeps getting the release
  ASSERT_TRUE(wait_for([&] { return servos.ignored_writes >= sent + 2; }));
  servos.deaf = false;
  ASSERT_TRUE(wait_for([&] { return torque() == 0; }, 1000ms));
  EXPECT_TRUE(wait_for([&] { return jaw().state().valid && !jaw().state().torque_enabled; }));
}

TEST_F(GrippersTest, ReleaseTurnsOffATorqueWhoseAcknowledgementWasLost) {
  connect();
  servos.unacknowledged = kTorqueEnable;
  jaw().set_target(0.0);
  ASSERT_TRUE(wait_for([&] { return torque() == 1; }));
  jaw().release();
  EXPECT_TRUE(wait_for([&] { return torque() == 0; }, 1000ms));
}

TEST_F(GrippersTest, DisconnectAndTheDestructorReleaseTheTorque) {
  enable();
  EXPECT_TRUE(grippers().disconnect());
  EXPECT_EQ(torque(), 0);
  enable();
  driver.reset();
  EXPECT_EQ(torque(), 0);
}

TEST_F(GrippersTest, ConnectAndDisconnectReportATorqueThatStaysOn) {
  servos.torque_stuck_on = true;  // acknowledges the write and keeps the torque on
  servos.set_u8(kId, kTorqueEnable, 1);
  EXPECT_FALSE(grippers().connect());
  EXPECT_TRUE(error_has("reads Torque_Enable 1")) << grippers().error();
  servos.torque_stuck_on = false;
  enable();
  servos.deaf = true;
  EXPECT_FALSE(grippers().disconnect());
  EXPECT_TRUE(error_has("may still be on")) << grippers().error();
  EXPECT_TRUE(wait_for([&] { return servos.ignored_writes >= 2; })) << "disconnect() tries twice";
}

TEST_F(GrippersTest, StateReportsEveryField) {
  servos.add(kId, counts_at(-0.25), 41);
  servos.set_u16(kId, 58, 100);  // Present_Speed, counts/s
  servos.set_u16(kId, 60, 300);  // Present_Load, permille
  servos.set_u16(kId, 69, 100);  // Present_Current, 6.5 mA a count
  connect();
  const auto s = jaw().state();
  EXPECT_NEAR(s.openness, -0.25, 0.002);
  EXPECT_NEAR(s.velocity, 100 * feetech::kRadiansPerCount, 1e-9);
  EXPECT_EQ(s.temperature, 41);
  EXPECT_NEAR(s.load, 0.3, 1e-9);
  EXPECT_NEAR(s.current, 0.65, 1e-9);
  EXPECT_FALSE(s.torque_enabled);
  EXPECT_TRUE(wait_for([&] { return jaw().state().sequence > s.sequence; }));
  servos.drop_sync_reads = 2;
  ASSERT_TRUE(wait_for([&] { return !jaw().state().valid; }));
  EXPECT_NEAR(jaw().state().openness, -0.25, 0.002) << "a failed read keeps the last values";
}

TEST_F(GrippersTest, StateSamplesAreNeverTorn) {
  connect();
  servos.ramp = true;
  std::atomic<bool> stop{false};
  std::atomic<long> samples{0}, torn{0};
  const auto reader = [&] {
    while (!stop) {
      const auto s = jaw().state();
      if (!s.valid) continue;
      const int counts = counts_at(s.openness);
      torn += s.temperature != 20 + counts % 30 || std::lround(s.load * 1000) != counts % 500;
      ++samples;
    }
  };
  reads(3);
  std::thread first(reader), second(reader);
  std::this_thread::sleep_for(300ms);
  stop = true;
  first.join();
  second.join();
  EXPECT_GT(samples.load(), 1000);
  EXPECT_EQ(torn.load(), 0);
}

TEST_F(GrippersTest, EachGripperHasItsOwnTravelTargetAndTorque) {
  config.push_back(make_gripper("left_gripper", servos.path(), kLeftId, -0.5, 0.3));
  config[1].torque_limit = 400;
  config[1].direction = -1.0;  // mounted the other way: its counts fall as it opens
  config[1].offset = -kOffset;
  servos.add(kLeftId, counts_at(-0.5));
  EXPECT_FALSE(grippers().connect());
  EXPECT_TRUE(error_has("gripper 'left_gripper' servo id 28 is at 0.500 rad, outside its travel"))
      << grippers().error();
  EXPECT_FALSE(grippers().gripper("thumb").has_value());
  servos.set_position(kLeftId, counts_at(0.0));
  servos.set_u16(kLeftId, 58, 100);  // Present_Speed
  servos.set_u16(kLeftId, 60, 300);  // Present_Load
  enable();
  EXPECT_NEAR(jaw("left_gripper").state().velocity, -100 * feetech::kRadiansPerCount, 1e-9);
  EXPECT_NEAR(jaw("left_gripper").state().load, -0.3, 1e-9);
  reads(5);
  EXPECT_EQ(torque(kLeftId), 0);
  for (const char* name : {"right_gripper", "left_gripper"}) jaw(name).set_target(10.0);
  ASSERT_TRUE(wait_for([&] { return torque(kLeftId) == 1; }));
  EXPECT_EQ(servos.reg_u16(kLeftId, kTorqueLimit), 400);
  EXPECT_EQ(limit(), kDefaultLimit);
  EXPECT_TRUE(
      wait_for([&] { return goal() == counts_at(kOpen) && goal(kLeftId) == counts_at(-0.3); }));
  jaw("left_gripper").set_target(-10.0);
  EXPECT_TRUE(wait_for([&] { return goal(kLeftId) == counts_at(0.5); }));
  EXPECT_EQ(goal(), counts_at(kOpen));
}

TEST_F(GrippersTest, AFaultOnOnePortReleasesTheGrippersOnEveryPortAndNamesIt) {
  specialist_test::FakeServos left;
  left.add(kLeftId, kCenter);
  config.push_back(make_gripper("left_gripper", left.path(), kLeftId));
  enable();
  jaw("left_gripper").set_target(0.0);
  ASSERT_TRUE(wait_for([&] { return left.reg_u8(kLeftId, kTorqueEnable) == 1; }));
  left.set_temperature(kLeftId, 65);
  ASSERT_TRUE(wait_for([&] { return error_has("Torque released."); })) << grippers().error();
  EXPECT_TRUE(error_has("'left_gripper' servo id 28 held 65 C")) << grippers().error();
  EXPECT_EQ(torque(), 0);
  EXPECT_EQ(left.reg_u8(kLeftId, kTorqueEnable), 0);
  left.set_temperature(kLeftId, 30);
  enable();
  left.silent_id = kLeftId;
  ASSERT_TRUE(wait_for([&] { return grippers().faulted(); }));
  EXPECT_TRUE(wait_for([&] { return torque() == 0; }, 500ms));
  EXPECT_TRUE(error_has("'left_gripper' servo id 28 did not reply")) << grippers().error();
  driver.reset();  // before `left` goes
}

TEST_F(GrippersTest, ConnectReleasesEveryGripperBeforeItFails) {
  servos.set_u8(kId, kTorqueEnable, 1);                               // a crashed run left it on
  config.push_back(make_gripper("left_gripper", servos.path(), 99));  // no servo answers
  EXPECT_FALSE(grippers().connect());
  EXPECT_TRUE(error_has("servo id 99 did not reply")) << grippers().error();
  EXPECT_EQ(torque(), 0);
}

// Returns why the constructor refuses `config`, or "".
std::string refusal(const std::vector<GripperConfig>& config) {
  try {
    Grippers grippers(config);
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

TEST(Grippers, TheConstructorRefusesEachBadField) {
  const std::vector<GripperConfig> good{make_gripper("right_gripper", "/dev/null", kId)};
  EXPECT_EQ(refusal(good), "");
  using Edit = std::function<void(std::vector<GripperConfig>&)>;
  const double nan = std::nan("");
  const std::pair<Edit, std::string> cases[] = {
      {[](auto& c) { c.clear(); }, "no grippers"},
      {[](auto& c) { c[0].port.clear(); }, "port"},
      {[](auto& c) { c[0].name.clear(); }, "no name"},
      {[](auto& c) { c[0].id = 254; }, "id 254"},
      {[](auto& c) { c[0] = {"right_gripper", "/dev/null", kId, kClosed, kOpen}; },
       "'right_gripper': is not calibrated. Run specialist-calibrate."},
      {[](auto& c) { c[0].direction = 0.0; }, "direction"},
      {[nan](auto& c) { c[0].offset = nan; }, "not calibrated"},
      {[](auto& c) { c[0].torque_limit = 1001; }, "torque_limit"},
      {[](auto& c) { c[0].torque_limit = 0; }, "torque_limit"},
      {[nan](auto& c) { c[0].closed = nan; }, "below"},
      {[](auto& c) { c[0].closed = c[0].open; }, "below"},
      {[](auto& c) { c[0].offset = 0.0; }, "encoder"},
      {[](auto& c) { c[0].open = 4.0; }, "encoder"},
      {[](auto& c) {
         c.push_back(c[0]);
         c[1].name = "left_gripper";
       },
       "servo id 18 is used twice on /dev/null"},
      {[](auto& c) {
         c.push_back(c[0]);
         c[1].id = kLeftId;
       },
       "name is used twice"},
  };
  for (const auto& [edit, expected] : cases) {
    auto config = good;
    edit(config);
    EXPECT_TRUE(contains(refusal(config), expected)) << refusal(config) << ", not " << expected;
  }
  auto apart = good;
  apart.push_back(make_gripper("left_gripper", "/dev/zero", kId));  // one id on two ports
  EXPECT_EQ(refusal(apart), "");
}
