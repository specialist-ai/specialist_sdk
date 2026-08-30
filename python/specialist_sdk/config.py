"""The grippers file: each gripper by name. The README shows it."""

import os
import shutil
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import yaml

from specialist_sdk._core import GripperConfig

REQUIRED = ("port", "id", "closed", "open")
KEYS = REQUIRED + ("torque_limit", "direction", "offset")


class _Loader(yaml.SafeLoader):
    """Refuses a name given twice, which PyYAML would drop silently."""

    def construct_mapping(self, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
        keys = [key.value for key, _ in node.value if isinstance(key, yaml.ScalarNode)]
        if repeated := {key for key in keys if keys.count(key) > 1}:
            problem = f"{repeated.pop()} is given twice"
            raise yaml.constructor.ConstructorError(None, None, problem, node.start_mark)
        return super().construct_mapping(node, deep)


def read(path: Path) -> dict[str, Any]:
    """The file as written: each gripper name to its fields."""
    try:
        document = yaml.load(path.read_text(), _Loader)
    except (OSError, yaml.YAMLError) as reason:
        raise ValueError(f"{path}: {reason}") from None
    if not isinstance(document, dict) or not document:
        raise ValueError(f"{path} must map each gripper name to its fields")
    return document


def load(path: str | Path) -> dict[str, GripperConfig]:
    """Each gripper of a grippers file by name, in the order of the file. Grippers(...) checks
    the values."""
    path = Path(path).expanduser()
    grippers = {}
    for name, entry in read(path).items():
        where = f"{path}: {name}"
        if not isinstance(name, str) or not isinstance(entry, dict):
            raise ValueError(f"{where} must be a text name with a mapping of its fields")
        if entry.keys() - set(KEYS):
            raise ValueError(f"{where} takes only {', '.join(KEYS)}")
        if missing := [key for key in REQUIRED if key not in entry]:
            raise ValueError(f"{where} needs {', '.join(missing)}")
        gripper = grippers[name] = GripperConfig()
        gripper.name = name
        for key, value in entry.items():
            try:  # nanobind raises TypeError for a wrong type or an id out of range
                if isinstance(value, bool):  # which nanobind takes as a number
                    raise TypeError
                setattr(gripper, key, value)
            except TypeError:
                raise ValueError(f"{where}.{key}: {value!r} is not a valid {key}") from None
    return grippers


def save(path: str | Path, grippers: Iterable[GripperConfig]) -> None:
    """Write the direction and the offset of each gripper into the grippers file, and keep the
    old file as `<file>.bak`."""
    path = Path(path).expanduser()
    document = read(path)
    for gripper in grippers:
        document[gripper.name] |= {"direction": int(gripper.direction), "offset": gripper.offset}
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(yaml.safe_dump(document, sort_keys=False))
    shutil.copy2(path, path.with_name(path.name + ".bak"))
    os.replace(temporary, path)
