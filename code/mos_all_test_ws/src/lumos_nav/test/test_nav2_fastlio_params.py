from pathlib import Path

import yaml


def _load_params():
    params_path = (
        Path(__file__).resolve().parents[1]
        / "params"
        / "nav2_params_fastlio.yaml"
    )
    return yaml.safe_load(params_path.read_text())


def test_fastlio_profile_uses_explicit_velocity_smoother():
    params = _load_params()
    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["smoothing_frequency"] >= 20.0
    assert smoother["scale_velocities"] is True
    assert smoother["max_velocity"][0] <= 0.35
    assert smoother["max_velocity"][2] <= 0.8


def test_fastlio_rpp_profile_prefers_stability_near_goal():
    params = _load_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["use_velocity_scaled_lookahead_dist"] is True
    assert follow_path["rotate_to_heading_angular_vel"] <= 0.4
    assert follow_path["rotate_to_heading_min_angle"] >= 0.5
    assert follow_path["max_angular_accel"] <= 1.0


def test_fastlio_planner_and_costmap_values_match_stability_profile():
    params = _load_params()
    planner = params["planner_server"]["ros__parameters"]["GridBased"]
    local_costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]
    inflation = local_costmap["inflation_layer"]
    assert planner["tolerance"] <= 0.25
    assert local_costmap["update_frequency"] >= 8.0
    assert inflation["inflation_radius"] >= 0.5
