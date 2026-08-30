"""Measure the direction and the offset of each jaw at its open stop, one gripper at a time.
The torque stays off.

    1. Hold the jaw fully open against the stop. Press Enter.
    2. Move the jaw part of the way to closed. Press Enter.

Stop each program that uses the bus first.

    specialist-calibrate --config grippers.yaml                          # every gripper
    specialist-calibrate --config grippers.yaml --gripper right_gripper  # one
"""

import argparse
import math
import time

from specialist_sdk._core import Gripper, GripperConfig, Grippers
from specialist_sdk.config import load, save

COUNT = 2 * math.pi / 4096  # rad
SAMPLES = 12  # A hand holds the jaw, so the tool averages the reads.
MINIMUM_TRAVEL = 20 * COUNT


def solve(open_stop: float, at_open: float, at_closed: float) -> tuple[float, float]:
    """Return (direction, offset) from two raw encoder angles in rad, so that the openness
    rises as the jaw opens and reads open_stop at the stop."""
    moved = at_open - at_closed
    if abs(moved) < MINIMUM_TRAVEL:
        raise ValueError(f"the jaw moved {moved:+.4f} rad. Move it farther")
    if abs(moved) > math.pi:  # no jaw travels half a turn
        raise ValueError("the travel crosses the encoder wrap. Remount the servo horn")
    direction = 1.0 if moved > 0 else -1.0
    return direction, open_stop - at_open / direction


def raw(gripper: GripperConfig) -> GripperConfig:
    """The same servo, with its openness equal to the raw encoder angle."""
    result = GripperConfig(gripper)
    result.direction, result.offset, result.closed, result.open = 1.0, 0.0, 0.0, 4095 * COUNT
    return result


def measure(gripper: Gripper, prompt: str) -> float:
    """Wait for Enter, then return the mean of SAMPLES new reads."""
    input(f"{prompt}, then press Enter. ")
    samples: list[float] = []
    sequence, end = None, time.monotonic() + 1.0
    while len(samples) < SAMPLES:
        if time.monotonic() > end:
            raise RuntimeError("the servo stopped replying")
        state = gripper.state()
        if state.valid and state.sequence != sequence:
            samples.append(state.openness)
            sequence = state.sequence
        time.sleep(0.001)
    return sum(samples) / len(samples)


def calibrate(chosen: list[GripperConfig]) -> None:
    """Set the direction and the offset of each gripper, measured with its torque off."""
    with Grippers([raw(g) for g in chosen]) as driver:  # releases the torque
        print("The torque is off. The jaws move freely.")
        for g in chosen:
            jaw = driver[g.name]
            at_open = measure(jaw, f"{g.name} 1/2  Hold the jaw fully open against the stop")
            at_closed = measure(jaw, f"{g.name} 2/2  Move the jaw part of the way to closed")
            g.direction, g.offset = solve(g.open, at_open, at_closed)
            print(f"{g.name}: direction {g.direction:+.0f}, offset {g.offset:+.6f}")
    Grippers(chosen)  # refuses a travel across the encoder wrap; opens nothing


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--config", required=True, help="grippers file")
    parser.add_argument("--gripper", help="calibrate only this gripper")
    parser.add_argument("--dry-run", action="store_true", help="show the result and write no file")
    args = parser.parse_args()

    try:
        chosen = [c for n, c in load(args.config).items() if args.gripper in (None, n)]
        if not chosen:
            raise ValueError(f"{args.config} has no gripper {args.gripper}")
        calibrate(chosen)
        if not args.dry_run:
            save(args.config, chosen)
            print(f"Wrote {args.config}. Restart each program that uses these grippers.")
    except (RuntimeError, ValueError) as reason:
        raise SystemExit(f"error: {str(reason).rstrip('.')}. Nothing was written.") from None
    except KeyboardInterrupt, EOFError:
        raise SystemExit("\nStopped. Nothing was written.") from None


if __name__ == "__main__":
    main()
