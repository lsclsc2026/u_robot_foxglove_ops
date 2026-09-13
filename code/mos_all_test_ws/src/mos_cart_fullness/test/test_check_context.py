import json

import pytest

from mos_cart_fullness.check_context import (
    fuse_view_decisions,
    parse_check_command,
)


def test_extended_verify_command_preserves_correlation_fields():
    command = parse_check_command(
        '{"check_id":"trip_1_cart_b_verify","mode":"verify_dual",'
        '"waypoint":"cart_b_grasp","zone_id":"scan_cart_b",'
        '"cart_id":"cart_b","wrist_camera":"left","timeout_sec":5.0}',
        waypoint_camera_map={},
        waypoint_cart_map={},
        default_verify_timeout_sec=5.0,
    )
    assert command.action == "start"
    assert command.context.check_id == "trip_1_cart_b_verify"
    assert command.context.mode == "verify_dual"
    assert command.context.cart_id == "cart_b"
    assert command.context.uses_head is True


def test_legacy_waypoint_remains_wrist_only():
    command = parse_check_command(
        "left_start",
        waypoint_camera_map={"left_start": "left"},
        waypoint_cart_map={"left_start": "cart_left"},
        default_verify_timeout_sec=5.0,
    )
    assert command.context.mode == "scan_wrist"
    assert command.context.wrist_camera == "left"
    assert command.context.cart_id == "cart_left"
    assert command.context.legacy is True


def test_dual_fusion_uses_any_available_view():
    assert fuse_view_decisions("verify_dual", "full", "full", False) == "full"
    assert fuse_view_decisions("verify_dual", "full", "not_full", False) == "not_full"
    assert fuse_view_decisions("verify_dual", "full", "unknown", False) == "full"
    assert fuse_view_decisions("verify_dual", "unknown", "full", False) == "full"
    assert fuse_view_decisions("verify_dual", "unknown", "unknown", False) == ""
    assert fuse_view_decisions("verify_dual", "full", "unknown", True) == "full"


def test_control_commands_do_not_require_waypoint_mappings():
    for text, action in (("", "off"), ("disable", "off"), ("disabled", "off"), ("off", "off"), ("reset", "reset")):
        assert parse_check_command(
            text,
            waypoint_camera_map={},
            waypoint_cart_map={},
            default_verify_timeout_sec=5.0,
        ).action == action


def test_invalid_extended_command_raises_value_error():
    with pytest.raises(ValueError, match="unsupported mode"):
        parse_check_command(
            '{"check_id":"check-1","mode":"invalid","wrist_camera":"left"}',
            waypoint_camera_map={},
            waypoint_cart_map={},
            default_verify_timeout_sec=5.0,
        )


@pytest.mark.parametrize(
    "timeout", ["not-a-number", "NaN", "Infinity", "-Infinity", True, False, None]
)
def test_invalid_extended_timeout_is_rejected(timeout):
    with pytest.raises(ValueError, match="timeout_sec"):
        parse_check_command(
            json.dumps({
                "check_id": "check-1",
                "mode": "verify_dual",
                "waypoint": "cart_a_grasp",
                "wrist_camera": "left",
                "timeout_sec": timeout,
            }),
            waypoint_camera_map={},
            waypoint_cart_map={},
            default_verify_timeout_sec=5.0,
        )
