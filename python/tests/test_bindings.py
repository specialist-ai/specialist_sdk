import pytest

import specialist_sdk as sdk


def test_grippers_refuse_a_bad_config_before_a_port_and_raise_a_port_that_fails():
    config = sdk.GripperConfig()
    config.name, config.port, config.id = "right_gripper", "/nonexistent/tty", 18
    config.closed, config.open = -0.26, 0.7
    with pytest.raises(ValueError, match="'right_gripper': is not calibrated"):
        sdk.Grippers([config])
    config.direction, config.offset = 1, 0.0
    with pytest.raises(ValueError, match="crosses the encoder wrap"):
        sdk.Grippers([config])
    config.offset = -3.14
    grippers = sdk.Grippers([config])
    with pytest.raises(KeyError, match="no gripper 'left_gripper'"):
        grippers["left_gripper"]
    with pytest.raises(RuntimeError, match="/nonexistent/tty: cannot open"):
        with grippers:
            pass
    assert not grippers.faulted and not grippers["right_gripper"].state().valid
    assert sdk.fingers(0.5) == {"left_finger": 0.5, "right_finger": -0.5}
