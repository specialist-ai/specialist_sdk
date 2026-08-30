import copy
import math

import pytest

import specialist_sdk as sdk


def _joint(name="right_finger", servo_id=18, lower=-0.7, upper=0.2):
    joint = sdk.JointConfig()
    joint.name, joint.id, joint.lower_limit, joint.upper_limit = name, servo_id, lower, upper
    joint.homing_offset = math.pi
    return joint


def _config(*joints, port="/dev/null"):
    config = sdk.GripperConfig()
    config.port = port
    config.joints = list(joints)
    return config


def test_validate_accepts_a_good_config():
    sdk.Gripper.validate(_config(_joint()))


@pytest.mark.parametrize(
    ("joints", "message"),
    [
        ((_joint("left"), _joint("right")), "servo id 18"),
        ((_joint(lower=1.0, upper=0.0),), "below"),
        ((sdk.JointConfig(),), "no name"),
    ],
)
def test_validate_refuses_a_bad_config(joints, message):
    with pytest.raises(RuntimeError, match=message):
        sdk.Gripper.validate(_config(*joints))


def test_connect_raises_with_the_error():
    with sdk.Gripper() as gripper:
        with pytest.raises(RuntimeError, match="cannot open"):
            gripper.connect(_config(_joint(), port="/nonexistent/tty"))
        assert gripper.mode == sdk.Mode.DISCONNECTED
        assert gripper.disconnect() and gripper.deactivate()


def test_a_bad_joint_index_raises():
    with sdk.Gripper() as gripper:
        for call in (
            lambda: gripper.set_target(0, 0.5),
            lambda: gripper.set_torque_enabled(7, True),
            lambda: gripper.set_torque_limit(0, 0.5),
            lambda: gripper.state(-1),
        ):
            with pytest.raises(IndexError, match="out of range"):
                call()
        assert gripper.fault == sdk.Fault.NONE and not gripper.faulted
        assert not gripper.is_connected and not gripper.is_active
        assert gripper.joint_count == 0 and gripper.joint_index("x") is None


def test_the_config_and_its_joints_are_copies():
    gripper = sdk.Gripper()
    gripper.config.port = "changed"
    assert gripper.config.port == "" and gripper.config.joints == ()
    config = _config(_joint())
    assert isinstance(config.joints, tuple)
    config.joints[0].torque_limit = 1
    assert config.joints[0].torque_limit == 667


def test_value_types_copy_and_print():
    original = _joint("left")
    clone = copy.deepcopy(original)
    clone.torque_limit = 300
    assert original.torque_limit == 667
    assert "name='left'" in repr(original) and "torque_limit=667" in repr(original)
    assert repr(sdk.JointState()).startswith("JointState(position=0.0")
    both = sdk.Fault.BUS_LOST | sdk.Fault.RELEASE_FAILED
    assert both & sdk.Fault.BUS_LOST == sdk.Fault.BUS_LOST
    assert sdk.__version__
