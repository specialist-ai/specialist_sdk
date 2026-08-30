// Opens and closes each gripper three times. It moves the jaws, so keep your hands clear.
// usage: specialist_open_close <port>

#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "specialist/gripper.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <port>\n", argv[0]);
    return 2;
  }
  // Replace the grippers, their ports, ids and calibration with yours.
  const std::vector<specialist::GripperConfig> config{
      {"right_gripper", argv[1], 18, -0.159, 0.698, 667, 1.0, 2.699635},
      {"left_gripper", argv[1], 28, -0.262, 0.698, 667, 1.0, 2.771733},
  };
  specialist::Grippers grippers(config);
  if (!grippers.connect()) {
    std::fprintf(stderr, "%s\n", grippers.error().c_str());
    return 1;
  }
  std::vector<specialist::Gripper> jaws;
  for (const auto& gripper : config) jaws.push_back(*grippers.gripper(gripper.name));
  for (int move = 0; move < 6 && !grippers.faulted(); ++move) {
    for (std::size_t i = 0; i < jaws.size(); ++i) {
      jaws[i].set_target(move % 2 == 0 ? config[i].open : config[i].closed);  // torque on
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (std::size_t i = 0; i < jaws.size(); ++i) {
      std::printf("%s: measured %+.3f rad\n", config[i].name.c_str(), jaws[i].state().openness);
    }
  }
  const bool faulted = grippers.faulted(), released = grippers.disconnect();
  if (faulted || !released) std::fprintf(stderr, "%s\n", grippers.error().c_str());
  return faulted || !released ? 1 : 0;
}
