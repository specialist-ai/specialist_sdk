"""Measure the direction and the zero of each joint at its open stop. The torque stays off.

    1. Hold the jaws fully open against the stop. Press Enter.
    2. Move the jaws part of the way to closed. Press Enter.

Stop each program that uses the bus first.

    specialist-calibrate --gripper gripper.yaml
"""

import argparse
import copy
import math
import time

import yaml

from specialist_sdk import config as files
from specialist_sdk._core import Gripper, GripperConfig, JointConfig

COUNT = 2 * math.pi / 4096  # rad
SAMPLES = 12  # A hand holds the jaw, so the tool averages the reads.
MINIMUM_TRAVEL = 20 * COUNT


def solve(upper_limit: float, at_open: float, at_closed: float) -> tuple[float, float]:
    """Return (direction, homing_offset) from two raw encoder angles in rad.

    Opening raises the openness, and the joint reads upper_limit at the open stop.
    """
    moved = at_open - at_closed
    if abs(moved) < MINIMUM_TRAVEL:
        raise ValueError(f"the jaw moved {moved:+.4f} rad. Move it farther")
    direction = 1.0 if moved > 0 else -1.0
    return direction, at_open - direction * upper_limit


def raw(config: GripperConfig) -> GripperConfig:
    """The same servos, with the joint angle equal to the raw encoder angle."""
    joints = [copy.copy(joint) for joint in config.joints]
    for joint in joints:
        joint.direction, joint.homing_offset = 1.0, 0.0
        joint.lower_limit, joint.upper_limit = 0.0, 4095 * COUNT
    result = copy.copy(config)
    result.joints = joints
    return result


def measure(gripper: Gripper, prompt: str) -> list[float]:
    """Wait for Enter, then return the mean of SAMPLES new reads of each joint."""
    input(f"{prompt}, then press Enter. ")
    means = []
    for index in range(gripper.joint_count):
        samples, sequence, end = [], None, time.monotonic() + 1.0
        while len(samples) < SAMPLES:
            if time.monotonic() > end:
                raise RuntimeError(f"joint {index} stopped replying")
            state = gripper.state(index)
            if state.valid and state.sequence != sequence:
                samples.append(state.position)
                sequence = state.sequence
            time.sleep(0.001)
        means.append(sum(samples) / len(samples))
    return means


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--gripper", required=True, help="gripper file")
    parser.add_argument("--dry-run", action="store_true", help="show the result and write no file")
    args = parser.parse_args()

    config, path = files.load_gripper(args.gripper, allow_missing_calibration=True)
    if path is None:
        raise SystemExit(f"error: {args.gripper} names no calibration_file")
    gripper = Gripper()
    try:
        gripper.connect(raw(config))  # releases the torque
        print("The torque is off. The jaws move freely.")
        at_open = measure(gripper, "1/2  Hold the jaws fully open against the stop")
        at_closed = measure(gripper, "2/2  Move the jaws part of the way to closed")
        values = (yaml.safe_load(path.read_text()) or {}) if path.exists() else {}
        if values.get("frame") != files.FRAME:  # the other joints of an older file are not openness
            names = {joint.name for joint in config.joints}
            values = {k: v for k, v in values.items() if k.startswith("_") or k in names}
        default_limit = JointConfig().torque_limit
        joints = []
        for joint, opened, closed in zip(config.joints, at_open, at_closed, strict=True):
            joint.direction, joint.homing_offset = solve(joint.upper_limit, opened, closed)
            joint.torque_limit = values.get(joint.name, {}).get("torque_limit", default_limit)
            values[joint.name] = {
                "direction": joint.direction,
                "homing_offset": joint.homing_offset,
                "torque_limit": joint.torque_limit,
            }
            joints.append(joint)
            print(
                f"{joint.name}: direction {joint.direction:+.0f}, homing_offset {joint.homing_offset:+.6f}"
            )
        config.joints = joints
        Gripper.validate(config)  # refuses a travel that crosses the encoder wrap
        if not args.dry_run:
            files.save_calibration(path, values)
            print(f"Wrote {path}. Restart each program that uses this gripper.")
    except (RuntimeError, ValueError) as reason:
        raise SystemExit(f"error: {str(reason).rstrip('.')}. Nothing was written.") from None
    except (KeyboardInterrupt, EOFError):
        raise SystemExit("\nStopped. Nothing was written.") from None
    finally:
        gripper.disconnect()


if __name__ == "__main__":
    main()
