"""Driver for the Specialist grippers on Feetech STS servos."""

from typing import Any

from specialist_sdk import config
from specialist_sdk._core import Gripper, GripperConfig, Grippers, GripperState

__all__ = ["Gripper", "GripperConfig", "GripperState", "Grippers", "config", "fingers"]


def fingers(openness: Any) -> dict[str, Any]:
    """The ``specialist_description`` finger joint angles at ``openness``, by name."""
    return {"left_finger": openness, "right_finger": -openness}
