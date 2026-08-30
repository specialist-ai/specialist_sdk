import pytest
import yaml

import specialist_sdk as sdk

GRIPPER = {
    "port": "/dev/ttyACM0",
    "bus_rate_hz": 200,
    "calibration_file": "calibration.yaml",
    "joints": {"jaw": {"id": 28, "closed": -0.261799, "open": 0.698132}},
}
CALIBRATION = {
    "frame": "openness",
    "jaw": {"direction": 1, "homing_offset": 2.771733, "torque_limit": 500},
}
OLDER_GRIPPER = {**GRIPPER, "joints": {"jaw": {"id": 28, "lower_limit": -0.7, "upper_limit": 0.3}}}
OLDER_CALIBRATION = {"jaw": {"direction": -1, "homing_offset": 2.771733}}


def write(folder, gripper=GRIPPER, calibration=CALIBRATION):
    (folder / "gripper.yaml").write_text(yaml.safe_dump(gripper))
    if calibration is not None:
        (folder / "calibration.yaml").write_text(yaml.safe_dump(calibration))
    return folder / "gripper.yaml"


def test_load_applies_the_calibration(tmp_path):
    config, path = sdk.config.load_gripper(write(tmp_path))
    assert path == tmp_path / "calibration.yaml"
    assert (config.port, config.baud_rate, config.bus_rate_hz) == ("/dev/ttyACM0", 1000000, 200)
    (joint,) = config.joints
    assert (joint.name, joint.id, joint.direction, joint.torque_limit) == ("jaw", 28, 1, 500)
    assert (joint.lower_limit, joint.upper_limit) == (-0.261799, 0.698132)
    assert joint.homing_offset == pytest.approx(2.771733)


@pytest.mark.parametrize(
    ("gripper", "calibration", "error", "message"),
    [
        (GRIPPER, None, ValueError, "specialist-calibrate"),
        ({**GRIPPER, "max_temp": 60}, CALIBRATION, ValueError, "'max_temp' is not one of"),
        ({**GRIPPER, "joints": {"x": {"id": 1}}}, CALIBRATION, ValueError, "needs 'closed'"),
        (OLDER_GRIPPER, CALIBRATION, ValueError, "'lower_limit' .* older file"),
        (GRIPPER, OLDER_CALIBRATION, ValueError, "no 'frame: openness'. In an older file"),
        (GRIPPER, {"frame": "openness", "left": {}}, ValueError, "has no 'jaw'"),
        (
            GRIPPER,
            {"frame": "openness", "jaw": {"homing_offset": 1, "acceleration": 5}},
            ValueError,
            "Only 0",
        ),
        (
            GRIPPER,
            {"frame": "openness", "jaw": {"homing_offset": 2.7, "direction": 0.5}},
            RuntimeError,
            "1 or -1",
        ),
    ],
)
def test_load_refuses_a_bad_file(tmp_path, gripper, calibration, error, message):
    with pytest.raises(error, match=message):
        sdk.config.load_gripper(write(tmp_path, gripper, calibration))


def test_load_without_the_calibration(tmp_path):
    config, path = sdk.config.load_gripper(
        write(tmp_path, calibration=None), allow_missing_calibration=True
    )
    assert config.joints[0].direction == 1 and path == tmp_path / "calibration.yaml"


def test_save_replaces_the_file_and_keeps_a_backup(tmp_path):
    old = {**CALIBRATION, "_comment": "by hand", "jaw": {**CALIBRATION["jaw"], "goal_velocity": 0}}
    path = write(tmp_path, calibration=old)
    assert sdk.config.load_gripper(path).config.joints[0].torque_limit == 500
    sdk.config.save_calibration(
        tmp_path / "calibration.yaml",
        {"_comment": "by hand", "jaw": {**CALIBRATION["jaw"], "direction": -1}},
    )
    assert sdk.config.load_gripper(path).config.joints[0].direction == -1
    assert yaml.safe_load((tmp_path / "calibration.yaml.bak").read_text()) == old
    saved = yaml.safe_load((tmp_path / "calibration.yaml").read_text())
    assert (saved["frame"], saved["_comment"]) == ("openness", "by hand")
