# specialist_sdk

Driver for the Specialist gripper on Feetech STS servos: the C++ library `specialist` and the Python module `specialist_sdk`. A background I/O thread drives the servos and applies the safety guards. The serial bus lives in [feetech_driver](https://github.com/specialist-ai/feetech_driver). Linux only.

## Joint frame

Each joint value is the jaw openness in radians: `direction * (raw_counts * 2π/4096 - homing_offset)`. A positive change opens the jaw. `upper_limit` is the open stop and `lower_limit` is the closed side. The SDK converts between openness and servo counts only at the servo. On the Specialist gripper, the openness is equal to the `left_finger` joint angle of the URDF and to the negative of the `right_finger` joint angle. The open stop is 0.698 and the closed side is -0.262.

The calibration values are plain `JointConfig` numbers: `direction`, `homing_offset` and `torque_limit` (permille of stall, 667 by default).

## Calibration

`specialist-calibrate --gripper gripper.yaml` measures `direction` and `homing_offset` with the torque off. You hold the jaws open against the stop, then move them part of the way closed. The tool writes the calibration file that the gripper file names, and keeps the previous file as `<file>.bak`. `specialist_sdk.config.load_gripper(path)` reads both files and returns `(GripperConfig, calibration_path)`. The docstring of `specialist_sdk/config.py` shows the two file formats.

The gripper file gives the limits of each joint as `closed` and `open`, and the calibration file starts with `frame: openness`. In older files, a positive change closed the jaw, and the loader refuses them. To convert an older pair by hand, change `lower_limit: a` and `upper_limit: b` to `closed: -b` and `open: -a`. Then change the sign of each `direction`, keep each `homing_offset`, and add `frame: openness` to the calibration file.

## Build

```bash
pixi run test                # C++ library, example and tests
uv sync && uv run pytest     # Python module
```

CMake uses an installed `feetech_driver`, then `../feetech_driver`, then clones the pinned commit.

## C++

```cpp
specialist::GripperConfig config;
config.port = "/dev/ttyACM0";
config.joints = {{"jaw", 18, 1.0, 4.358, -0.262, 0.698, 667}};
specialist::Gripper gripper;
if (!gripper.connect(config) || !gripper.activate()) return 1;  // see gripper.last_error()
gripper.set_torque_enabled(0, true);
gripper.set_target(0, 0.0);
specialist::JointState state = gripper.state(0);
```

See `examples/open_close.cpp` for a full program.

## Python

```python
import specialist_sdk as sdk

joint = sdk.JointConfig()
joint.name, joint.id, joint.direction, joint.homing_offset = "jaw", 18, 1.0, 4.358
joint.lower_limit, joint.upper_limit = -0.262, 0.698
config = sdk.GripperConfig()
config.port, config.joints = "/dev/ttyACM0", [joint]
with sdk.Gripper() as gripper:  # the block releases the torque when it ends
    gripper.connect(config)
    gripper.activate()
    gripper.set_torque_enabled(0, True)
    gripper.set_target(0, 0.0)
    print(gripper.state(0))
```

`connect()` and `activate()` raise `RuntimeError` in Python and return `false` in C++.

## Safety guards

- `connect()` and `activate()` release the torque. `activate()` sets the goal to the present position.
- Torque on does not move the jaw to an old target. The target becomes the present position, unless `set_target()` gave a new one after the previous `set_torque_enabled()` call.
- `activate()` refuses a hot servo, a servo outside position mode, and a jaw more than 0.05 rad outside its travel.
- The I/O thread clamps every target into the travel.
- A fault releases the torque: 25 reads in a row at 65 °C or more, 20 failed reads in a row, or a servo that restarted or released its own torque.
- A fault latches until the next successful `activate()`.
- The I/O thread sends the torque release again until the servo acknowledges it.
- `disconnect()` and the destructor release the torque.

The SDK has no stall detection and no command watchdog. Keep your hands clear while the torque is on. Use this software at your own risk.
