import math

import pytest
import yaml

import specialist_sdk as sdk

LEFT = {"port": "/dev/ttyACM0", "id": 28, "closed": -0.262, "open": 0.698, "torque_limit": 500}


def write(folder, document):
    path = folder / "grippers.yaml"
    path.write_text(
        document if isinstance(document, str) else yaml.safe_dump(document, sort_keys=False)
    )
    return path


def test_load_reads_each_gripper_by_name(tmp_path):
    right = {"port": "/dev/ttyACM1", "id": 18, "closed": -0.159, "open": 0.698}
    right |= {"direction": -1, "offset": 3.77}
    loaded = sdk.config.load(write(tmp_path, {"right_gripper": right, "left_gripper": LEFT}))
    assert list(loaded) == ["right_gripper", "left_gripper"]
    right, left = loaded.values()
    assert (right.name, right.port, right.id) == ("right_gripper", "/dev/ttyACM1", 18)
    assert (right.torque_limit, right.direction, right.offset) == (667, -1, 3.77)
    assert left.torque_limit == 500 and math.isnan(left.direction) and math.isnan(left.offset)


@pytest.mark.parametrize(
    ("document", "message"),
    [
        ("a: {port: /dev/A}\na: {port: /dev/B}\n", "a is given twice"),
        ({"left_gripper": LEFT | {"max_temp": 60}}, "takes only port"),
        ({"left_gripper": {"id": 28, "closed": 0}}, "left_gripper needs port, open"),
        ({"left_gripper": LEFT | {"id": 300}}, "300 is not a valid id"),
        ({"left_gripper": LEFT | {"closed": True}}, "True is not a valid closed"),
        ({1: LEFT}, "1 must be a text name"),
        ({"left_gripper": 5}, "left_gripper must be a text name with a mapping"),
        ("a: {port: /dev/A, id: 1\n", "expected ',' or '}'"),
        ("- right_gripper\n", "must map each gripper name"),
    ],
)
def test_load_refuses_a_bad_file(tmp_path, document, message):
    with pytest.raises(ValueError, match=message):
        sdk.config.load(write(tmp_path, document))


def test_load_refuses_a_missing_file(tmp_path):
    with pytest.raises(ValueError, match="No such file"):
        sdk.config.load(tmp_path / "missing.yaml")
