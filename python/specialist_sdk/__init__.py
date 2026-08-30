"""Driver for the Specialist grippers on Feetech STS servos."""

from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as _version

from specialist_sdk import config
from specialist_sdk._core import (
    Fault,
    Gripper,
    GripperConfig,
    JointConfig,
    JointState,
    LogLevel,
    Mode,
    set_log_handler,
)

try:
    __version__ = _version("specialist-sdk")
except PackageNotFoundError:  # run from a source tree
    __version__ = "0+unknown"

__all__ = [
    "Fault",
    "Gripper",
    "GripperConfig",
    "JointConfig",
    "JointState",
    "LogLevel",
    "Mode",
    "__version__",
    "config",
    "set_log_handler",
]
