from mos_route_manager.geometry import SegmentZone
from mos_route_manager.state_machine import (
    Navigate,
    PatrolLeg,
    PublishStatus,
    RouteManagerMachine,
    RouteState,
    RunPlace,
    ScanZone,
    SetFullnessContext,
    SetFullnessEnabled,
    SetNavCmdEnabled,
    StartPipeline,
)


def only(commands, command_type):
    matches = [command for command in commands if isinstance(command, command_type)]
    assert len(matches) == 1
    return matches[0]


def make_machine():
    zones = {
        "scan_a": ScanZone(
            zone_id="scan_a",
            cart_slot="cart_slot_1",
            pickup_enabled=True,
            grasp_waypoint="grasp_a",
            corridor=SegmentZone("scan_a", 0.0, 0.0, 3.0, 0.0, 0.5),
        ),
        "scan_b": ScanZone(
            zone_id="scan_b",
            cart_slot="cart_slot_2",
            pickup_enabled=False,
            grasp_waypoint="",
            corridor=SegmentZone("scan_b", 4.0, 0.0, 7.0, 0.0, 0.5),
        ),
    }
    patrol_legs = (
        PatrolLeg("left_scan", RouteState.LEFT_SCAN_LEG, "waypoint_left_end", "left", ("scan_a", "scan_b")),
        PatrolLeg("left_to_right", RouteState.LEFT_TO_RIGHT_TRANSIT, "waypoint_right", "", ()),
        PatrolLeg("right_scan", RouteState.RIGHT_SCAN_LEG, "waypoint_right_end", "right", ("scan_b", "scan_a")),
        PatrolLeg("right_to_left", RouteState.RIGHT_TO_LEFT_TRANSIT, "waypoint_left", "", ()),
    )
    return RouteManagerMachine(
        patrol_legs=patrol_legs,
        scan_zones=zones,
        initial_waypoint="waypoint_left",
        dirty_car_waypoint="dirty_car",
        clean_car_waypoint="clean_car",
        verify_timeout_sec=6.0,
    )


def start_left_scan(machine):
    assert only(machine.start(), Navigate).goal_name == "waypoint_left"
    commands = machine.on_goal_reached()
    assert machine.state is RouteState.LEFT_SCAN_LEG
    assert only(commands, Navigate).goal_name == "waypoint_left_end"


def arm_scan_a(machine):
    commands = machine.on_pose(1.0, 0.0)
    context = only(commands, SetFullnessContext).payload
    assert context["zone_id"] == "scan_a"
    return context["check_id"]


def arm_scan_b(machine):
    commands = machine.on_pose(5.0, 0.0)
    context = only(commands, SetFullnessContext).payload
    assert context["zone_id"] == "scan_b"
    return context["check_id"]


def reach_verify(machine):
    start_left_scan(machine)
    check_id = arm_scan_a(machine)
    commands = machine.on_route_signal({"check_id": check_id, "decision": "full"})
    assert only(commands, Navigate).goal_name == "grasp_a"
    commands = machine.on_goal_reached()
    assert machine.state is RouteState.VERIFY_DUAL_AT_GRASP
    assert only(commands, SetNavCmdEnabled).enabled is False
    return only(commands, SetFullnessContext).payload


def verify_full(machine):
    context = reach_verify(machine)
    commands = machine.on_route_signal({"check_id": context["check_id"], "decision": "full"})
    assert machine.state is RouteState.START_DIRTY_GRASP_PIPELINE
    assert isinstance(only(commands, StartPipeline), StartPipeline)
    assert machine.on_pipeline_status("succeeded", 5) == []
    assert machine.on_pipeline_start_requested("dirty_cart", 10) == []
    assert machine.on_pipeline_status("running", 10) == []
    assert machine.on_pipeline_status("succeeded", 11) == []
    commands = machine.on_pipeline_start_result("dirty_cart", True, "started")
    assert machine.state is RouteState.RUN_DIRTY_GRASP_PIPELINE
    assert only(commands, PublishStatus).payload["event"] == "pipeline_start_acknowledged"
    assert machine.on_pipeline_status("succeeded", 11) == []
    assert machine.on_pipeline_status("running", 12) == []


def test_left_scan_entry_starts_one_left_wrist_context():
    machine = make_machine()
    start_left_scan(machine)

    check_id = arm_scan_a(machine)

    assert check_id.startswith("epoch_1_left_scan_scan_a")
    assert machine.on_pose(1.0, 0.0) == []


def test_start_enables_nav_commands_before_initial_navigation():
    machine = make_machine()

    commands = machine.start()

    assert only(commands, SetNavCmdEnabled).enabled is True
    assert only(commands, Navigate).goal_name == "waypoint_left"


def test_zone_exit_without_result_stops_check_and_continues_patrol():
    machine = make_machine()
    start_left_scan(machine)
    arm_scan_a(machine)

    commands = machine.on_pose(3.6, 0.0)

    assert only(commands, SetFullnessContext).payload == "off"
    assert machine.state is RouteState.LEFT_SCAN_LEG
    status = only(commands, PublishStatus).payload
    assert status["event"] == "scan_abandoned"


def test_stale_fullness_signal_is_ignored():
    machine = make_machine()
    start_left_scan(machine)
    arm_scan_a(machine)

    assert machine.on_route_signal({"check_id": "stale", "decision": "full"}) == []
    assert machine.state is RouteState.LEFT_SCAN_LEG


def test_scan_a_full_preempts_once_to_grasp_a():
    machine = make_machine()
    start_left_scan(machine)
    check_id = arm_scan_a(machine)

    commands = machine.on_route_signal({"check_id": check_id, "decision": "full"})

    assert only(commands, Navigate).goal_name == "grasp_a"
    assert only(commands, SetFullnessContext).payload == "off"
    assert only(commands, SetNavCmdEnabled).enabled is False
    assert machine.state is RouteState.NAVIGATE_GRASP
    assert machine.on_route_signal({"check_id": check_id, "decision": "full"}) == []


def test_scan_b_full_is_observation_only():
    machine = make_machine()
    start_left_scan(machine)
    check_id = arm_scan_b(machine)

    commands = machine.on_route_signal({"check_id": check_id, "decision": "full"})

    assert not any(isinstance(command, Navigate) for command in commands)
    assert only(commands, SetFullnessContext).payload == "off"
    assert only(commands, PublishStatus).payload["event"] == "full_without_grasp_waypoint"
    assert machine.state is RouteState.LEFT_SCAN_LEG


def test_verification_uses_original_wrist_and_not_full_resumes_saved_endpoint():
    machine = make_machine()
    context = reach_verify(machine)

    assert context["mode"] == "verify_dual"
    assert context["wrist_camera"] == "left"
    assert context["timeout_sec"] == 6.0
    commands = machine.on_route_signal(
        {"check_id": context["check_id"], "decision": "not_full"}
    )

    assert only(commands, Navigate).goal_name == "waypoint_left_end"
    assert machine.state is RouteState.RESUME_SAVED_SCAN_LEG_GOAL


def test_verification_timeout_rejects_and_resumes_saved_endpoint():
    machine = make_machine()
    reach_verify(machine)

    commands = machine.on_verification_timeout()

    assert only(commands, Navigate).goal_name == "waypoint_left_end"
    assert machine.state is RouteState.RESUME_SAVED_SCAN_LEG_GOAL


def test_successful_mission_runs_dirty_place_clean_place_then_resumes():
    machine = make_machine()
    verify_full(machine)

    commands = machine.on_pipeline_status("succeeded", 13)
    assert only(commands, SetNavCmdEnabled).enabled is True
    assert only(commands, Navigate).goal_name == "dirty_car"
    assert machine.state is RouteState.NAVIGATE_DIRTY_CAR

    commands = machine.on_goal_reached()
    assert machine.state is RouteState.PLACE_DIRTY_CART
    assert only(commands, SetNavCmdEnabled).enabled is False
    assert isinstance(only(commands, RunPlace), RunPlace)

    commands = machine.on_place_result(True, "dirty cart placed")
    assert only(commands, SetNavCmdEnabled).enabled is True
    assert only(commands, Navigate).goal_name == "clean_car"
    assert machine.state is RouteState.NAVIGATE_CLEAN_CAR

    commands = machine.on_goal_reached()
    assert machine.state is RouteState.START_CLEAN_GRASP_PIPELINE
    assert isinstance(only(commands, StartPipeline), StartPipeline)
    machine.on_pipeline_start_requested("clean_cart", 20)
    machine.on_pipeline_start_result("clean_cart", True, "started")
    machine.on_pipeline_status("running", 21)

    commands = machine.on_pipeline_status("succeeded", 22)
    assert only(commands, SetNavCmdEnabled).enabled is True
    assert only(commands, Navigate).goal_name == "grasp_a"
    assert machine.state is RouteState.RETURN_TO_GRASP_A

    commands = machine.on_goal_reached()
    assert machine.state is RouteState.PLACE_CLEAN_CART
    assert only(commands, SetNavCmdEnabled).enabled is False
    assert isinstance(only(commands, RunPlace), RunPlace)

    commands = machine.on_place_result(True, "clean cart placed")
    assert only(commands, Navigate).goal_name == "waypoint_left_end"
    assert machine.state is RouteState.RESUME_SAVED_SCAN_LEG_GOAL

    commands = machine.on_goal_reached()
    assert only(commands, Navigate).goal_name == "waypoint_right"
    assert machine.state is RouteState.LEFT_TO_RIGHT_TRANSIT


def test_pipeline_or_place_failure_enters_fault_hold_with_fullness_disabled():
    machine = make_machine()
    verify_full(machine)

    commands = machine.on_pipeline_status("failed", 13)

    assert machine.state is RouteState.FAULT_HOLD
    assert only(commands, SetFullnessEnabled).enabled is False
    assert only(commands, SetNavCmdEnabled).enabled is False


def test_pipeline_start_rejection_enters_fault_hold():
    machine = make_machine()
    context = reach_verify(machine)
    machine.on_route_signal({"check_id": context["check_id"], "decision": "full"})

    commands = machine.on_pipeline_start_result("dirty_cart", False, "already running")

    assert machine.state is RouteState.FAULT_HOLD
    assert only(commands, SetFullnessEnabled).enabled is False


def test_pipeline_accepts_completion_only_after_current_request_running_generation():
    machine = make_machine()
    context = reach_verify(machine)
    machine.on_route_signal({"check_id": context["check_id"], "decision": "full"})

    assert machine.on_pipeline_status("running", 40) == []
    assert machine.on_pipeline_start_requested("dirty_cart", 50) == []
    assert machine.on_pipeline_status("running", 50) == []
    assert machine.on_pipeline_start_result("dirty_cart", True, "started")
    assert machine.on_pipeline_status("succeeded", 51) == []
    assert machine.on_pipeline_status("running", 52) == []

    commands = machine.on_pipeline_status("succeeded", 53)

    assert only(commands, Navigate).goal_name == "dirty_car"


def test_external_service_failure_enters_fault_hold_with_fullness_disabled():
    machine = make_machine()
    start_left_scan(machine)

    commands = machine.on_external_failure("mos_cart_fullness/set_enabled", "unavailable")

    assert machine.state is RouteState.FAULT_HOLD
    assert only(commands, SetFullnessEnabled).enabled is False


def test_fault_hold_ignores_later_external_failures_without_requeueing_actions():
    machine = make_machine()
    machine.on_external_failure("mos_cart_fullness/set_enabled", "unavailable")

    assert machine.on_external_failure("mos_cart_fullness/set_enabled", "unavailable") == []


def test_reset_fault_returns_to_wait_ready_without_resuming_motion():
    machine = make_machine()
    start_left_scan(machine)
    machine.on_external_failure("pipeline", "failed")

    commands = machine.reset_fault()

    assert machine.state is RouteState.WAIT_READY
    assert not any(isinstance(command, Navigate) for command in commands)
    assert only(commands, SetFullnessEnabled).enabled is False


def test_right_scan_uses_right_wrist_camera_only_after_transit():
    machine = make_machine()
    start_left_scan(machine)
    machine.on_goal_reached()
    assert machine.state is RouteState.LEFT_TO_RIGHT_TRANSIT
    machine.on_goal_reached()
    assert machine.state is RouteState.RIGHT_SCAN_LEG

    check_id = arm_scan_b(machine)
    commands = machine.on_pose(5.0, 0.0)

    assert check_id.startswith("epoch_2_right_scan_scan_b")
    assert commands == []
