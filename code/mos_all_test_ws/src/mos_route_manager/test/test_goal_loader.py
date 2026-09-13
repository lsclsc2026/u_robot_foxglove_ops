from pathlib import Path

import pytest

from mos_route_manager.goal_loader import load_goal_poses


def test_load_goal_poses_accepts_nested_goal_pose_mapping(tmp_path: Path):
    path = tmp_path / "goals.yaml"
    path.write_text(
        "goals:\n"
        "  grasp_a:\n"
        "    pose:\n"
        "      position: {x: 1, y: 2, z: 3}\n"
        "      orientation: {x: 0, y: 0, z: 0, w: 1}\n",
        encoding="utf-8",
    )

    goal = load_goal_poses(path)["grasp_a"]

    assert goal.name == "grasp_a"
    assert (goal.x, goal.y, goal.z) == (1.0, 2.0, 3.0)
    assert (goal.qx, goal.qy, goal.qz, goal.qw) == (0.0, 0.0, 0.0, 1.0)


def test_load_goal_poses_accepts_direct_position_orientation_mapping(tmp_path: Path):
    path = tmp_path / "goals.yaml"
    path.write_text(
        "dirty_car:\n"
        "  position: {x: 4.0, y: 5.0, z: 6.0}\n"
        "  orientation: {x: 0.0, y: 0.0, z: 0.5, w: 0.5}\n",
        encoding="utf-8",
    )

    goal = load_goal_poses(path)["dirty_car"]

    assert goal.x == 4.0
    assert goal.qz == 0.5


def test_load_goal_poses_rejects_non_finite_values(tmp_path: Path):
    path = tmp_path / "goals.yaml"
    path.write_text(
        "grasp_a:\n"
        "  position: {x: .nan, y: 2, z: 3}\n"
        "  orientation: {x: 0, y: 0, z: 0, w: 1}\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="finite"):
        load_goal_poses(path)
