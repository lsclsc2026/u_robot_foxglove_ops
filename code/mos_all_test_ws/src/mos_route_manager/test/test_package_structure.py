from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_uses_pure_ament_python():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml").getroot()

    assert package_xml.findtext("buildtool_depend") == "ament_python"
    assert package_xml.findtext("./export/build_type") == "ament_python"
    assert not (PACKAGE_ROOT / "CMakeLists.txt").exists()


def test_setup_installs_resource_config_launch_and_console_entry_point():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "share/ament_index/resource_index/packages" in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.py")' in setup_text
    assert (
        "mos_route_manager = mos_route_manager.route_manager_node:main"
        in setup_text
    )


def test_config_contains_required_map_only_scan_geometry():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "route_manager.yaml").read_text(encoding="utf-8")
    )
    params = config["mos_route_manager"]["ros__parameters"]

    assert params["scan_half_width_m"] == 0.50
    assert params["navigation_timeout_sec"] == 120.0
    assert params["scan_a_start_x"] == 0.6621
    assert params["scan_a_start_y"] == 0.2248
    assert params["scan_a_end_x"] == 3.3063
    assert params["scan_a_end_y"] == 0.8032
    assert params["scan_b_start_x"] == 3.8406
    assert params["scan_b_start_y"] == 0.7794
    assert params["scan_b_end_x"] == 8.3138
    assert params["scan_b_end_y"] == 0.7939
    assert params["scan_a_pickup_enabled"] is True
    assert params["scan_b_pickup_enabled"] is False
    assert params["scan_b_grasp_waypoint"] == ""


def test_launch_keeps_patrol_opt_in_by_default():
    launch_text = (PACKAGE_ROOT / "launch" / "route_manager.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'default_value="false"' in launch_text
    assert "goal_file" in launch_text


def test_route_manager_uses_nav2_action_and_not_legacy_navigation_topics():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    node_source = (
        PACKAGE_ROOT / "mos_route_manager" / "route_manager_node.py"
    ).read_text(encoding="utf-8")
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "route_manager.yaml").read_text(encoding="utf-8")
    )
    params = config["mos_route_manager"]["ros__parameters"]

    assert "<depend>nav2_msgs</depend>" in package_xml
    assert "ActionClient" in node_source
    assert "NavigateToPose" in node_source
    assert "navigation_action_name" in params
    assert "goal_topic" not in params
    assert "move_status_topic" not in params
