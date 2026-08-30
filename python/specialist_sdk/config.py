"""The gripper file and the calibration file that it names, relative to its own directory.
Each value is the jaw openness in rad, so a positive change opens the jaw.

# gripper.yaml
port: /dev/ttyACM0
baud_rate: 1000000          # optional
bus_rate_hz: 250            # optional
calibration_file: calibration.yaml
joints:
  jaw: {id: 18, closed: -0.261799, open: 0.698132}  # lower_limit and upper_limit

# calibration.yaml, which specialist-calibrate writes
frame: openness
jaw: {direction: 1, homing_offset: 4.344063, torque_limit: 667}
"""

import os
import shutil
from pathlib import Path
from typing import NamedTuple

import yaml

from specialist_sdk._core import Gripper, GripperConfig, JointConfig

FRAME = "openness"  # Older files have no frame. In them, a positive change closed the jaw.
_BUS = {"port": str, "baud_rate": int, "bus_rate_hz": int}
_JOINT = {"id": int, "closed": float, "open": float}
_FIELD = {"closed": "lower_limit", "open": "upper_limit"}
_OLDER = "In an older file, a positive change closed the jaw. The README tells how to convert it"
_CALIBRATION = {"direction": float, "homing_offset": float, "torque_limit": int}
# Older files have these. The SDK always sends 0, which means no limit.
_ZERO_ONLY = ("goal_velocity", "acceleration")


class LoadedGripper(NamedTuple):
    config: GripperConfig
    calibration_path: Path | None


def _read(path: Path) -> dict:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError(f"{path} must hold a mapping")
    return document


def _apply(target, values, schema: dict, where: str, required=()) -> None:
    if not isinstance(values, dict):
        raise ValueError(f"{where} must be a mapping")
    for key, value in values.items():
        if key in _ZERO_ONLY and schema is _CALIBRATION:
            if value != 0:
                raise ValueError(f"{where}: {key} {value} is not supported. Only 0 is")
            continue
        if key not in schema:
            older = f". {_OLDER}" if key.endswith("_limit") else ""
            raise ValueError(f"{where}: '{key}' is not one of {', '.join(schema)}{older}")
        setattr(target, _FIELD.get(key, key), schema[key](value))
    for key in required:
        if key not in values:
            raise ValueError(f"{where} needs '{key}'")


def load_gripper(path: str | Path, *, allow_missing_calibration: bool = False) -> LoadedGripper:
    """Read a gripper file and apply its calibration. Return (config, calibration_path).

    With `allow_missing_calibration`, the calibration file is not read and the config is not
    validated. specialist-calibrate uses this.
    """
    path = Path(path).expanduser()
    document = _read(path)
    joints, name = document.pop("joints", None), document.pop("calibration_file", None)
    config = GripperConfig()
    _apply(config, document, _BUS, str(path))
    if not isinstance(joints, dict) or not joints:
        raise ValueError(f"{path} needs a 'joints' mapping")
    calibration_path = (path.parent / Path(name).expanduser()).resolve() if name else None
    values = {}
    if not allow_missing_calibration:
        if calibration_path is None or not calibration_path.exists():
            raise ValueError(f"{path} has no calibration file. Run specialist-calibrate")
        values = _read(calibration_path)
        if values.pop("frame", None) != FRAME:
            raise ValueError(
                f"{calibration_path} has no 'frame: {FRAME}'. {_OLDER}, or run specialist-calibrate"
            )
    loaded = []
    for joint_name, entry in joints.items():
        joint = JointConfig()
        joint.name = str(joint_name)
        _apply(joint, entry, _JOINT, f"{path}: joints.{joint_name}", required=_JOINT)
        if not allow_missing_calibration:
            if joint_name not in values:
                raise ValueError(
                    f"{calibration_path} has no '{joint_name}'. Run specialist-calibrate"
                )
            where = f"{calibration_path}: {joint_name}"
            _apply(joint, values[joint_name], _CALIBRATION, where, required=["homing_offset"])
        loaded.append(joint)
    config.joints = loaded
    if not allow_missing_calibration:
        Gripper.validate(config)
    return LoadedGripper(config, calibration_path)


def save_calibration(path: str | Path, values: dict[str, dict]) -> None:
    """Replace the calibration file atomically, and keep the previous one as `<file>.bak`."""
    path = Path(path).expanduser()
    document = {"frame": FRAME} | {  # keys that start with "_" are notes, and stay as they are
        name: entry
        if name.startswith("_")
        else {k: kind(entry[k]) for k, kind in _CALIBRATION.items()}
        for name, entry in values.items()
        if name != "frame"
    }
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(yaml.safe_dump(document, sort_keys=False))
    if path.exists():
        shutil.copy2(path, path.with_name(path.name + ".bak"))
    os.replace(temporary, path)
