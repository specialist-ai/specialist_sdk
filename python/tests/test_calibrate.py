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
OPEN = 0.698132


def test_solve_makes_opening_positive_and_puts_the_open_stop_at_open():
    for opened, closed, direction in ((1000, 1300, -1.0), (2262, 1900, 1.0)):
        sign, offset = calibrate.solve(OPEN, opened * COUNT, closed * COUNT)
        assert sign == direction
        assert opened * COUNT / sign + offset == pytest.approx(OPEN)
        assert closed * COUNT / sign + offset < OPEN
    with pytest.raises(ValueError, match="farther"):
        calibrate.solve(OPEN, 1000 * COUNT, 1010 * COUNT)
    with pytest.raises(ValueError, match="encoder wrap"):  # closing passes count 0
        calibrate.solve(OPEN, 100 * COUNT, 4000 * COUNT)


@pytest.fixture
def fake_bus():
    """Opens buses of fake servos by id on the master side of a pty. No hardware is opened."""
    buses = []

    def start(*ids):
        master, slave = os.openpty()
        tty.setraw(slave)
        port, tables, stop = os.ttyname(slave), {i: bytearray(256) for i in ids}, []
        os.close(slave)
        thread = threading.Thread(target=serve, args=(master, tables, stop), daemon=True)
        thread.start()
        buses.append((master, thread, stop))
        return port, tables

    yield start
    for master, thread, stop in buses:
        stop.append(True)
        thread.join()
        os.close(master)


def serve(master, tables, stop):
    def status(servo, data=b""):
        body = bytes([servo, len(data) + 2, 0]) + data
        return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    buf = b""
    while not stop:
        try:
            buf += os.read(master, 4096) if select.select([master], [], [], 0.02)[0] else b""
        except OSError:
            continue
        while len(buf) >= 4 and len(buf) >= 4 + buf[3]:
            packet, buf = buf[: 4 + buf[3]], buf[4 + buf[3] :]
            servo, inst, p = packet[2], packet[4], packet[5:-1]
            if inst == 0x82:
                for i in (i for i in p[2:] if i in tables):
                    os.write(master, status(i, bytes(tables[i][p[0] : p[0] + p[1]])))
            elif inst in (1, 2, 3) and servo in tables:
                table = tables[servo]
                if inst == 3:
                    table[p[0] : p[0] + len(p) - 1] = p[1:]
                os.write(
                    master, status(servo, bytes(table[p[0] : p[0] + p[1]]) if inst == 2 else b"")
                )


@pytest.mark.parametrize(
    ("only", "buses"), [(None, 1), ("left_gripper", 1), (None, 2), ("left_gripper", 2)]
)
def test_calibrate_writes_a_file_that_load_accepts(fake_bus, tmp_path, monkeypatch, only, buses):
    if buses == 1:
        port, tables = fake_bus(18, 28)
        ports = (port, port)
    else:
        (left_port, left_tables), (right_port, right_tables) = fake_bus(28), fake_bus(18)
        ports, tables = (left_port, right_port), left_tables | right_tables
    path = tmp_path / "grippers.yaml"
    left = {"port": ports[0], "id": 28, "closed": -0.261799, "open": OPEN}
    right = left | {"port": ports[1], "id": 18, "closed": -0.2, "direction": 1, "offset": -3.0}
    path.write_text(yaml.safe_dump({"left_gripper": left, "right_gripper": right}))
    # Each gripper open, then part of the way closed. Gripper 18 opens the other way.
    poses = iter([(28, 2262), (28, 1900), (18, 1000), (18, 1300)])

    def hand(prompt):
        servo, counts = next(poses)
        tables[servo][56:58] = counts.to_bytes(2, "little")
        time.sleep(0.05)  # several reads of the new pose

    monkeypatch.setattr("builtins.input", hand)
    argv = ["specialist-calibrate", "--config", str(path)] + (["--gripper", only] if only else [])
    monkeypatch.setattr("sys.argv", argv)
    calibrate.main()
    assert tables[28][40] == 0 and tables[18][40] == 0
    assert (tmp_path / "grippers.yaml.bak").exists()
    left, right = sdk.config.load(path).values()
    assert left.direction == 1 and left.torque_limit == 667
    assert left.offset == pytest.approx(OPEN - 2262 * COUNT)
    if only:
        assert (right.direction, right.offset) == (1, -3.0)
    else:
        assert right.direction == -1 and right.offset == pytest.approx(1000 * COUNT + OPEN)
