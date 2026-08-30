"""A crash at interpreter shutdown cannot be caught in-process, so each case runs in a subprocess."""

import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    "body",
    [
        "",
        "sdk.set_log_handler(lambda level, message: None)",
        "sdk.set_log_handler(lambda level, message: None)\nsdk.set_log_handler(None)",
        "sdk.set_log_handler(print)\nsdk.Gripper()",
    ],
)
def test_the_interpreter_exits_cleanly(body):
    # A log handler held by a C++ static segfaults if released after Py_Finalize.
    command = [sys.executable, "-c", f"import specialist_sdk as sdk\n{body}"]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, f"exit {result.returncode}\n{result.stderr}"
    assert "leaked" not in result.stderr, result.stderr
