// Opens and closes one gripper three times. It moves the jaw, so keep your hands clear.
// usage: specialist_open_close <port>

#include <chrono>
#include <cstdio>
#include <thread>

#include "specialist/gripper.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <port>\n", argv[0]);
    return 2;
  }

  // Replace these values with the calibration of your gripper.
  specialist::JointConfig joint;
  joint.name = "jaw";
  joint.id = 18;
  joint.direction = 1.0;
  joint.homing_offset = 4.358;  // rad
  joint.lower_limit = -0.262;   // rad, the closed side
  joint.upper_limit = 0.698;    // rad, the open stop
  specialist::GripperConfig config;
  config.port = argv[1];
  config.joints = {joint};

  specialist::Gripper gripper;
  if (!gripper.connect(config) || !gripper.activate()) {
    std::fprintf(stderr, "%s\n", gripper.last_error().c_str());
    return 1;
  }
  gripper.set_torque_enabled(0, true);  // the target starts at the present position
  for (int cycle = 0; cycle < 3 && !gripper.faulted(); ++cycle) {
    for (const double target : {joint.upper_limit, joint.lower_limit}) {
      gripper.set_target(0, target);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::printf("target %+.3f rad, measured %+.3f rad\n", target, gripper.state(0).position);
    }
  }
  if (gripper.faulted()) std::fprintf(stderr, "%s\n", gripper.fault_message().c_str());
  const bool released = gripper.disconnect();
  if (!released) std::fprintf(stderr, "%s\n", gripper.last_error().c_str());
  return gripper.faulted() || !released ? 1 : 0;
}
