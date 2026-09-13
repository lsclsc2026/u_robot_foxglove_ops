from pathlib import Path

from mos_arm_controller.cmd_vel_gate import NavCommandGate


def test_gate_starts_disabled_and_requires_explicit_enable():
    gate = NavCommandGate(timeout_sec=0.25)

    assert gate.accepts_command(now_sec=0.0) is False

    gate.set_enabled(True, now_sec=0.0)
    assert gate.accepts_command(now_sec=0.0) is True


def test_gate_stops_after_command_timeout():
    gate = NavCommandGate(timeout_sec=0.25)
    gate.set_enabled(True, now_sec=0.0)
    gate.record_command(now_sec=0.1)

    assert gate.needs_stop(now_sec=0.34) is False
    assert gate.needs_stop(now_sec=0.36) is True


def test_gate_disabling_requests_an_immediate_stop():
    gate = NavCommandGate(timeout_sec=0.25)
    gate.set_enabled(True, now_sec=0.0)
    gate.record_command(now_sec=0.1)

    gate.set_enabled(False, now_sec=0.2)

    assert gate.accepts_command(now_sec=0.2) is False
    assert gate.needs_stop(now_sec=0.2) is True


def test_hardware_controller_exposes_a_nav_velocity_gate_and_watchdog():
    source = (
        Path(__file__).parents[1] / "mos_arm_controller" / "hardware_controller.py"
    ).read_text(encoding="utf-8")

    assert '"/mos_arm_controller/set_nav_cmd_enabled"' in source
    assert "NavCommandGate" in source
    assert "cmd_vel_timeout_sec" in source
