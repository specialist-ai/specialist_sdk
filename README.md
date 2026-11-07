# specialist_sdk

Driver for the Specialist grippers, each one Feetech STS servo: the C++ library `specialist` and the Python module `specialist_sdk`. One `Grippers` object drives every gripper of a robot, on one or more serial ports, from one background I/O thread that applies the safety guards. The serial bus lives in [feetech_driver](https://github.com/specialist-ai/feetech_driver). Linux only.

## Openness

Each gripper takes one command and gives one reading: the jaw openness in radians, positive opening. It is `(left_finger - right_finger) / 2` of the `specialist_description` URDF, the definition of the policies and the datasets. `specialist_sdk.fingers(openness)` gives those finger joint angles by name. The servo reads `openness = raw_counts * 2π/4096 / direction + offset`, the position map of ros2_control's `SimpleTransmission` with `direction` (+1 or -1) as its mechanical reduction and `offset` as its joint offset. The calibration finds `direction` and `offset`, so the way the servo is mounted does not matter.

## Grippers file

One file lists the grippers of a robot by a name of your choice: the port, the servo id, the travel and the calibration of each. Grippers on one port share its bus. Name each port by `/dev/serial/by-id`: ttyACM numbers can swap between boots.

```yaml
right_gripper:
  port: /dev/serial/by-id/usb-...5111-if00
  id: 18
  closed: -0.159       # rad
  open: 0.698          # rad, the open stop
  torque_limit: 667    # optional, permille of stall
  direction: 1         # written by specialist-calibrate
  offset: -2.699635    # written by specialist-calibrate
left_gripper: {port: /dev/serial/by-id/usb-...5376-if00, id: 28, closed: -0.262, open: 0.698, direction: 1, offset: -2.771733}
```

`specialist_sdk.config.load(path)` reads it into a `GripperConfig` per name, in the order of the file, and raises `ValueError` for a missing, unknown or mistyped key. `Grippers(...)` checks the values. `specialist-calibrate --config grippers.yaml` measures `direction` and `offset` of each gripper in turn with its torque off: hold the jaw open against the stop, then move it part of the way closed. `--gripper right_gripper` calibrates one, and `--dry-run` writes no file. The tool rewrites the file once at the end (comments are not kept) and keeps the previous one as `grippers.yaml.bak`.

## Build

```bash
pixi run test                # C++ library, example and tests
uv sync && uv run pytest     # Python 3.14 module
```

CMake uses an installed `feetech_driver`, then `../feetech_driver`, then clones the pinned commit.

## Use

```python
import specialist_sdk as sdk

grippers = sdk.Grippers(list(sdk.config.load("grippers.yaml").values()))
with grippers:  # connects; the block releases the torque when it ends
    right = grippers["right_gripper"]
    right.set_target(0.0)  # turns the torque on
    print(right.state())
```

`Grippers(configs)` raises `ValueError` for a config it refuses, before it opens a port. `with grippers:` connects, and raises `RuntimeError` when a port or a servo fails the checks; its end disconnects. `grippers[name]` gives one gripper, which has `set_target`, `release` and `state`, and raises `KeyError` for an unknown name. `grippers.faulted` and `grippers.error` tell a fault. C++ has the same API: the constructor throws `std::invalid_argument`, `connect()` and `disconnect()` return `false` and set `error()`, and `gripper()` returns an empty `std::optional`. See `examples/open_close.cpp`.

## Safety guards

- The constructor refuses a gripper that is not calibrated, a travel that crosses the encoder wrap, or a servo id used twice on a port.
- `connect()` releases the torque of every gripper, also when one of them fails, then refuses a silent servo, a hot one, one outside position mode, and a jaw more than 0.05 rad outside the `[closed, open]` of its gripper. Every torque stays off until its first `set_target()`.
- The I/O thread clamps every target into the `[closed, open]` of its gripper, and writes the torque limit of each gripper before its goal, and the goal before the torque.
- A fault on any gripper releases the torque of all: 25 reads in a row at 65 °C or more, 20 failed reads in a row on a port, or a servo restarted or released its own torque. The fault names the port, the gripper and the servo id, goes to stderr, and latches until the next `connect()`.
- The I/O thread sends the torque release again until each servo acknowledges it.
- `disconnect()` and the destructor release the torque, and print a release a servo did not acknowledge to stderr.

The SDK has no stall detection and no command watchdog. Keep your hands clear while the torque is on. Use this software at your own risk.
