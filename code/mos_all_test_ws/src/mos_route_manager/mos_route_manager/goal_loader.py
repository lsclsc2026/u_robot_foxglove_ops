"""Strict loading of named navigation goals from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class GoalPose:
    """One named pose in the `map` frame."""

    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def load_goal_poses(path: Path) -> dict[str, GoalPose]:
    """Load named goal poses from the Lumos target-goals YAML file.

    Both `{name: {position, orientation}}` and
    `{goals: {name: {pose: {position, orientation}}}}` layouts are accepted.
    """
    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    except OSError as exc:
        raise ValueError(f"cannot read goal file {source}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid goal YAML {source}: {exc}") from exc

    if not isinstance(payload, dict):
        raise ValueError("goal YAML root must be a mapping")
    entries = payload.get("goals", payload.get("target_goals", payload))
    if not isinstance(entries, dict) or not entries:
        raise ValueError("goal YAML must contain a non-empty named-goal mapping")

    return {
        str(name): _parse_goal(str(name), raw_goal)
        for name, raw_goal in entries.items()
    }


def _parse_goal(name: str, raw_goal: Any) -> GoalPose:
    if not name:
        raise ValueError("goal name must not be empty")
    if not isinstance(raw_goal, dict):
        raise ValueError(f"goal {name!r} must be a mapping")

    pose = raw_goal.get("pose", raw_goal)
    if not isinstance(pose, dict):
        raise ValueError(f"goal {name!r} pose must be a mapping")
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f"goal {name!r} requires position and orientation mappings")

    values = {
        "x": _finite_component(name, "position.x", position.get("x")),
        "y": _finite_component(name, "position.y", position.get("y")),
        "z": _finite_component(name, "position.z", position.get("z")),
        "qx": _finite_component(name, "orientation.x", orientation.get("x")),
        "qy": _finite_component(name, "orientation.y", orientation.get("y")),
        "qz": _finite_component(name, "orientation.z", orientation.get("z")),
        "qw": _finite_component(name, "orientation.w", orientation.get("w")),
    }
    return GoalPose(name=name, **values)


def _finite_component(goal_name: str, component: str, value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"goal {goal_name!r} has invalid {component}") from exc
    if not isfinite(number):
        raise ValueError(f"goal {goal_name!r} {component} must be finite")
    return number
