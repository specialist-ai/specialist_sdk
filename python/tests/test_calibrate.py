import math
import os
import select
import threading
import time
import tty

import pytest
import yaml

import specialist_sdk as sdk
from specialist_sdk import calibrate

COUNT = 2 * math.pi / 4096
UPPER = 0.698132


def test_solve_makes_opening_positive_and_puts_the_open_stop_at_upper_limit():
    for opened, closed, direction in ((1000, 1300, -1.0), (2262, 1900, 1.0)):
        sign, offset = calibrate.solve(UPPER, opened * COUNT, closed * COUNT)
        assert sign == direction
        assert sign * (opened * COUNT - offset) == pytest.approx(UPPER)
        assert sign * (closed * COUNT - offset) < UPPER
    with pytest.raises(ValueError, match="farther"):
        calibrate.solve(UPPER, 1000 * COUNT, 1010 * COUNT)


@pytest.fixture
def servo():
    """Servo 28 on the master side of a pty. No hardware is opened."""
    master, slave = os.openpty()
    tty.setraw(slave)
    port, table, stop = os.ttyname(slave), bytearray(256), []
    os.close(slave)
    table[62], table[63] = 120, 30

    def status(data=b""):
        body = bytes([28, len(data) + 2, 0]) + data
        return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    def run():
        buf = b""
        while not stop:
            try:
                buf += os.read(master, 4096) if select.select([master], [], [], 0.02)[0] else b""
            except OSError:
                continue
            while len(buf) >= 4 and len(buf) >= 4 + buf[3]:
                packet, buf = buf[: 4 + buf[3]], buf[4 + buf[3] :]
                inst, p = packet[4], packet[5:-1]
                if inst == 3:
                    table[p[0] : p[0] + len(p) - 1] = p[1:]
                if inst == 0x82 and 28 in p[2:]:
                    os.write(master, status(bytes(table[p[0] : p[0] + p[1]])))
                elif inst in (1, 2, 3) and packet[2] == 28:
                    os.write(master, status(bytes(table[p[0] : p[0] + p[1]]) if inst == 2 else b""))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    yield port, table
    stop.append(True)
    thread.join()
    os.close(master)


@pytest.mark.parametrize("frame", ["openness", None])
def test_calibrate_writes_a_file_that_load_gripper_accepts(servo, tmp_path, monkeypatch, frame):
    port, table = servo
    gripper = {
        "port": port,
        "calibration_file": "calibration.yaml",
        "joints": {"jaw": {"id": 28, "closed": -0.261799, "open": UPPER}},
    }
    (tmp_path / "gripper.yaml").write_text(yaml.safe_dump(gripper))
    other = {"left": {"direction": 1, "homing_offset": 1.0, "torque_limit": 1}}
    (tmp_path / "calibration.yaml").write_text(yaml.safe_dump({"frame": frame, **other}))
    poses = iter([2262, 1900])  # open, then part of the way closed

    def hand(prompt):
        table[56:58] = next(poses).to_bytes(2, "little")
        time.sleep(0.05)  # several reads of the new pose

    monkeypatch.setattr("builtins.input", hand)
    monkeypatch.setattr(
        "sys.argv", ["specialist-calibrate", "--gripper", str(tmp_path / "gripper.yaml")]
    )
    calibrate.main()
    assert table[40] == 0
    config, _ = sdk.config.load_gripper(tmp_path / "gripper.yaml")
    (joint,) = config.joints
    assert joint.direction == 1 and joint.torque_limit == 667
    assert joint.homing_offset == pytest.approx(2262 * COUNT - UPPER)
    # An older file has no frame, so its other joints are not openness and are dropped.
    assert ("left" in yaml.safe_load((tmp_path / "calibration.yaml").read_text())) == bool(frame)
