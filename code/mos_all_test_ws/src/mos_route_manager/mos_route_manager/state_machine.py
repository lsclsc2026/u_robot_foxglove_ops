"""ROS-independent policy for the MOS patrol and cart-replacement mission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, TypeAlias

from .geometry import SegmentZone


class RouteState(str, Enum):
    WAIT_READY = "wait_ready"
    NAVIGATE_INITIAL_WAYPOINT = "navigate_initial_waypoint"
    LEFT_SCAN_LEG = "left_scan_leg"
    LEFT_TO_RIGHT_TRANSIT = "left_to_right_transit"
    RIGHT_SCAN_LEG = "right_scan_leg"
    RIGHT_TO_LEFT_TRANSIT = "right_to_left_transit"
    NAVIGATE_GRASP = "navigate_grasp"
    VERIFY_DUAL_AT_GRASP = "verify_dual_at_grasp"
    START_DIRTY_GRASP_PIPELINE = "start_dirty_grasp_pipeline"
    RUN_DIRTY_GRASP_PIPELINE = "run_dirty_grasp_pipeline"
    NAVIGATE_DIRTY_CAR = "navigate_dirty_car"
    PLACE_DIRTY_CART = "place_dirty_cart"
    NAVIGATE_CLEAN_CAR = "navigate_clean_car"
    START_CLEAN_GRASP_PIPELINE = "start_clean_grasp_pipeline"
    RUN_CLEAN_GRASP_PIPELINE = "run_clean_grasp_pipeline"
    RETURN_TO_GRASP_A = "return_to_grasp_a"
    PLACE_CLEAN_CART = "place_clean_cart"
    RESUME_SAVED_SCAN_LEG_GOAL = "resume_saved_scan_leg_goal"
    FAULT_HOLD = "fault_hold"


@dataclass(frozen=True)
class ScanZone:
    zone_id: str
    cart_slot: str
    pickup_enabled: bool
    grasp_waypoint: str
    corridor: SegmentZone


@dataclass(frozen=True)
class PatrolLeg:
    leg_id: str
    state: RouteState
    goal_name: str
    wrist_camera: str
    zone_ids: tuple[str, ...]

    @property
    def scans(self) -> bool:
        return self.wrist_camera in {"left", "right"}


@dataclass(frozen=True)
class CandidateMission:
    mission_id: int
    cart_slot: str
    origin_zone: str
    grasp_waypoint: str
    origin_leg_index: int
    origin_wrist_camera: str
    resume_goal: str


@dataclass(frozen=True)
class Navigate:
    goal_name: str


@dataclass(frozen=True)
class SetFullnessContext:
    payload: dict[str, Any] | str


@dataclass(frozen=True)
class ResetFullness:
    pass


@dataclass(frozen=True)
class SetFullnessEnabled:
    enabled: bool


@dataclass(frozen=True)
class SetNavCmdEnabled:
    enabled: bool


@dataclass(frozen=True)
class StartPipeline:
    purpose: str


@dataclass(frozen=True)
class RunPlace:
    purpose: str


@dataclass(frozen=True)
class PublishStatus:
    payload: dict[str, Any]


Command: TypeAlias = (
    Navigate
    | SetFullnessContext
    | ResetFullness
    | SetFullnessEnabled
    | SetNavCmdEnabled
    | StartPipeline
    | RunPlace
    | PublishStatus
)


class RouteManagerMachine:
    """Deterministic route policy driven by pose, result, arrival, and task events."""

    def __init__(
        self,
        *,
        patrol_legs: tuple[PatrolLeg, ...],
        scan_zones: dict[str, ScanZone],
        initial_waypoint: str,
        dirty_car_waypoint: str,
        clean_car_waypoint: str,
        verify_timeout_sec: float,
    ) -> None:
        if not patrol_legs:
            raise ValueError("at least one patrol leg is required")
        if verify_timeout_sec <= 0.0:
            raise ValueError("verify_timeout_sec must be positive")
        self._legs = patrol_legs
        self._zones = dict(scan_zones)
        self._initial_waypoint = initial_waypoint
        self._dirty_car_waypoint = dirty_car_waypoint
        self._clean_car_waypoint = clean_car_waypoint
        self.verify_timeout_sec = float(verify_timeout_sec)
        self._validate_configuration()

        self.state = RouteState.WAIT_READY
        self._leg_index: int | None = None
        self._epoch = 0
        self._checked_zone_ids: set[str] = set()
        self._skip_cart_slots: set[str] = set()
        self._active_zone_id = ""
        self._active_check_id = ""
        self._expected_goal_name = ""
        self._mission: CandidateMission | None = None
        self._next_mission_id = 1
        self._pipeline_request_status_sequence: int | None = None
        self._pipeline_running_status_sequence: int | None = None
        self._pipeline_succeeded_status_sequence: int | None = None

    @property
    def active_check_id(self) -> str:
        return self._active_check_id

    @property
    def expected_goal_name(self) -> str:
        return self._expected_goal_name

    @property
    def mission(self) -> CandidateMission | None:
        return self._mission

    def start(self) -> list[Command]:
        """Start safely at the first patrol waypoint; no scan is active yet."""
        if self.state is not RouteState.WAIT_READY:
            return []
        self.state = RouteState.NAVIGATE_INITIAL_WAYPOINT
        return [
            SetFullnessContext("off"),
            SetFullnessEnabled(False),
            SetNavCmdEnabled(True),
            self._navigate(self._initial_waypoint),
            self._status("patrol_starting"),
        ]

    def on_pose(self, x: float, y: float) -> list[Command]:
        """Arm and disarm a single scan context according to current map x/y."""
        if not self._in_scan_leg():
            return []

        commands: list[Command] = []
        if self._active_zone_id:
            active_zone = self._zones[self._active_zone_id]
            if active_zone.corridor.contains(x, y):
                return []
            self._checked_zone_ids.add(active_zone.zone_id)
            self._clear_active_check()
            commands.extend(
                [
                    SetFullnessContext("off"),
                    self._status("scan_abandoned", zone_id=active_zone.zone_id),
                ]
            )

        leg = self._current_leg()
        for zone_id in leg.zone_ids:
            zone = self._zones[zone_id]
            if zone_id in self._checked_zone_ids or zone.cart_slot in self._skip_cart_slots:
                continue
            if not zone.corridor.contains(x, y):
                continue
            self._active_zone_id = zone_id
            self._active_check_id = self._new_scan_check_id(leg, zone)
            commands.extend(
                [
                    SetFullnessContext(
                        {
                            "check_id": self._active_check_id,
                            "mode": "scan_wrist",
                            "waypoint": f"{leg.leg_id}:{leg.goal_name}",
                            "zone_id": zone.zone_id,
                            "cart_id": zone.cart_slot,
                            "wrist_camera": leg.wrist_camera,
                            "timeout_sec": 0.0,
                        }
                    ),
                    self._status("scan_started", zone_id=zone.zone_id),
                ]
            )
            break
        return commands

    def on_route_signal(self, payload: dict[str, Any]) -> list[Command]:
        """Accept only the active correlated fullness result."""
        if str(payload.get("check_id") or "") != self._active_check_id:
            return []
        decision = str(payload.get("decision") or "").strip().lower()
        if decision not in {"full", "not_full"}:
            return []

        if self.state is RouteState.VERIFY_DUAL_AT_GRASP:
            return self._handle_verify_decision(decision)
        if not self._in_scan_leg() or not self._active_zone_id:
            return []
        return self._handle_scan_decision(decision)

    def on_goal_reached(self) -> list[Command]:
        """Advance after the node has validated a matching arrival event."""
        if self.state is RouteState.NAVIGATE_INITIAL_WAYPOINT:
            return self._begin_leg(0)
        if self._in_patrol_state():
            return self._arrive_at_patrol_leg_goal()
        if self.state is RouteState.NAVIGATE_GRASP:
            return self._start_verification()
        if self.state is RouteState.NAVIGATE_DIRTY_CAR:
            self.state = RouteState.PLACE_DIRTY_CART
            return [
                SetNavCmdEnabled(False),
                RunPlace("dirty_cart"),
                self._status("place_dirty_cart"),
            ]
        if self.state is RouteState.NAVIGATE_CLEAN_CAR:
            self.state = RouteState.START_CLEAN_GRASP_PIPELINE
            self._reset_pipeline_tracking()
            return [StartPipeline("clean_cart"), self._status("run_clean_grasp_pipeline")]
        if self.state is RouteState.RETURN_TO_GRASP_A:
            self.state = RouteState.PLACE_CLEAN_CART
            return [
                SetNavCmdEnabled(False),
                RunPlace("clean_cart"),
                self._status("place_clean_cart"),
            ]
        if self.state is RouteState.RESUME_SAVED_SCAN_LEG_GOAL:
            if self._mission is None:
                return self._fault("missing mission while resuming patrol")
            origin_leg_index = self._mission.origin_leg_index
            self._mission = None
            return self._begin_leg((origin_leg_index + 1) % len(self._legs))
        return []

    def on_pipeline_status(self, status: str, sequence: int) -> list[Command]:
        status = str(status or "").strip().lower()
        active_states = {
            RouteState.START_DIRTY_GRASP_PIPELINE,
            RouteState.RUN_DIRTY_GRASP_PIPELINE,
            RouteState.START_CLEAN_GRASP_PIPELINE,
            RouteState.RUN_CLEAN_GRASP_PIPELINE,
        }
        if self.state not in active_states:
            return []
        if (
            self._pipeline_request_status_sequence is None
            or sequence <= self._pipeline_request_status_sequence
        ):
            return []
        if status == "failed":
            return self._fault("grasp pipeline failed")
        if status == "running":
            self._pipeline_running_status_sequence = sequence
            return []
        if status != "succeeded":
            return []
        if (
            self._pipeline_running_status_sequence is None
            or sequence <= self._pipeline_running_status_sequence
        ):
            return []
        self._pipeline_succeeded_status_sequence = sequence
        if self.state in {
            RouteState.START_DIRTY_GRASP_PIPELINE,
            RouteState.START_CLEAN_GRASP_PIPELINE,
        }:
            return []
        return self._complete_pipeline_success()

    def on_pipeline_start_requested(self, purpose: str, status_sequence: int) -> list[Command]:
        """Mark the local status sequence immediately before issuing `/start`."""
        expected_state = {
            "dirty_cart": RouteState.START_DIRTY_GRASP_PIPELINE,
            "clean_cart": RouteState.START_CLEAN_GRASP_PIPELINE,
        }.get(str(purpose))
        if expected_state is None or self.state is not expected_state:
            return []
        self._pipeline_request_status_sequence = int(status_sequence)
        return []

    def on_pipeline_start_result(
        self, purpose: str, success: bool, message: str
    ) -> list[Command]:
        """Accept a pipeline completion only after this start call was acknowledged."""
        expected_state = {
            "dirty_cart": RouteState.START_DIRTY_GRASP_PIPELINE,
            "clean_cart": RouteState.START_CLEAN_GRASP_PIPELINE,
        }.get(str(purpose))
        if expected_state is None or self.state is not expected_state:
            return []
        if not success:
            return self._fault(f"pipeline start failed: {message}")
        if self._pipeline_request_status_sequence is None:
            return self._fault("pipeline start acknowledged without request generation")
        self.state = (
            RouteState.RUN_DIRTY_GRASP_PIPELINE
            if purpose == "dirty_cart"
            else RouteState.RUN_CLEAN_GRASP_PIPELINE
        )
        commands: list[Command] = [
            self._status("pipeline_start_acknowledged", purpose=purpose)
        ]
        if self._pipeline_succeeded_status_sequence is not None:
            commands.extend(self._complete_pipeline_success())
        return commands

    def _complete_pipeline_success(self) -> list[Command]:
        if self.state is RouteState.RUN_DIRTY_GRASP_PIPELINE:
            self._reset_pipeline_tracking()
            self.state = RouteState.NAVIGATE_DIRTY_CAR
            return [
                SetNavCmdEnabled(True),
                self._navigate(self._dirty_car_waypoint),
                self._status("navigate_dirty_car"),
            ]
        if self.state is RouteState.RUN_CLEAN_GRASP_PIPELINE:
            self._reset_pipeline_tracking()
            if self._mission is None:
                return self._fault("missing mission after clean grasp")
            self.state = RouteState.RETURN_TO_GRASP_A
            return [
                SetNavCmdEnabled(True),
                self._navigate(self._mission.grasp_waypoint),
                self._status("return_to_grasp_waypoint"),
            ]
        return []

    def on_place_result(self, success: bool, message: str) -> list[Command]:
        if self.state not in {RouteState.PLACE_DIRTY_CART, RouteState.PLACE_CLEAN_CART}:
            return []
        if not success:
            return self._fault(f"place failed: {message}")
        if self.state is RouteState.PLACE_DIRTY_CART:
            self.state = RouteState.NAVIGATE_CLEAN_CAR
            return [
                SetNavCmdEnabled(True),
                self._navigate(self._clean_car_waypoint),
                self._status("navigate_clean_car"),
            ]
        if self._mission is None:
            return self._fault("missing mission after clean place")
        self.state = RouteState.RESUME_SAVED_SCAN_LEG_GOAL
        return [
            self._navigate(self._mission.resume_goal),
            self._status("resume_saved_scan_leg_goal"),
        ]

    def on_verification_timeout(self) -> list[Command]:
        if self.state is not RouteState.VERIFY_DUAL_AT_GRASP:
            return []
        return self._reject_candidate("verification_timeout")

    def on_external_failure(self, interface: str, message: str) -> list[Command]:
        """Move to the safe hold state when a required ROS service fails."""
        if self.state is RouteState.FAULT_HOLD:
            return []
        return self._fault(f"{interface}: {message}")

    def reset_fault(self) -> list[Command]:
        """Manually re-arm the policy without publishing a navigation goal."""
        if self.state is not RouteState.FAULT_HOLD:
            return []
        self.state = RouteState.WAIT_READY
        self._leg_index = None
        self._mission = None
        self._reset_pipeline_tracking()
        self._checked_zone_ids.clear()
        self._skip_cart_slots.clear()
        self._expected_goal_name = ""
        self._clear_active_check()
        return [
            SetFullnessContext("off"),
            SetFullnessEnabled(False),
            SetNavCmdEnabled(False),
            self._status("fault_reset_waiting_for_manual_start"),
        ]

    def _handle_scan_decision(self, decision: str) -> list[Command]:
        zone = self._zones[self._active_zone_id]
        self._checked_zone_ids.add(zone.zone_id)
        self._clear_active_check()
        if decision == "not_full":
            return [
                SetFullnessContext("off"),
                SetFullnessEnabled(True),
                self._status("scan_not_full", zone_id=zone.zone_id),
            ]
        if not zone.pickup_enabled or not zone.grasp_waypoint:
            return [
                SetFullnessContext("off"),
                SetFullnessEnabled(True),
                self._status("full_without_grasp_waypoint", zone_id=zone.zone_id),
            ]

        leg = self._current_leg()
        self._mission = CandidateMission(
            mission_id=self._next_mission_id,
            cart_slot=zone.cart_slot,
            origin_zone=zone.zone_id,
            grasp_waypoint=zone.grasp_waypoint,
            origin_leg_index=self._leg_index_or_raise(),
            origin_wrist_camera=leg.wrist_camera,
            resume_goal=leg.goal_name,
        )
        self._next_mission_id += 1
        self.state = RouteState.NAVIGATE_GRASP
        return [
            SetFullnessContext("off"),
            SetFullnessEnabled(False),
            SetNavCmdEnabled(False),
            self._navigate(zone.grasp_waypoint),
            self._status("candidate_full", zone_id=zone.zone_id),
        ]

    def _start_verification(self) -> list[Command]:
        if self._mission is None:
            return self._fault("missing mission at grasp waypoint")
        self.state = RouteState.VERIFY_DUAL_AT_GRASP
        self._active_zone_id = self._mission.origin_zone
        self._active_check_id = (
            f"mission_{self._mission.mission_id}_verify_{self._mission.cart_slot}"
        )
        context = {
            "check_id": self._active_check_id,
            "mode": "verify_dual",
            "waypoint": self._mission.grasp_waypoint,
            "zone_id": self._mission.origin_zone,
            "cart_id": self._mission.cart_slot,
            "wrist_camera": self._mission.origin_wrist_camera,
            "timeout_sec": self.verify_timeout_sec,
        }
        return [
            SetNavCmdEnabled(False),
            ResetFullness(),
            SetFullnessEnabled(True),
            SetFullnessContext(context),
            self._status("verify_dual_started", zone_id=self._mission.origin_zone),
        ]

    def _handle_verify_decision(self, decision: str) -> list[Command]:
        if decision == "full":
            self._clear_active_check()
            self.state = RouteState.START_DIRTY_GRASP_PIPELINE
            self._reset_pipeline_tracking()
            return [
                SetFullnessContext("off"),
                SetFullnessEnabled(False),
                SetNavCmdEnabled(False),
                StartPipeline("dirty_cart"),
                self._status("verify_full"),
            ]
        return self._reject_candidate("verification_not_full")

    def _reject_candidate(self, reason: str) -> list[Command]:
        if self._mission is None:
            return self._fault("missing mission while rejecting candidate")
        self._skip_cart_slots.add(self._mission.cart_slot)
        resume_goal = self._mission.resume_goal
        self._clear_active_check()
        self.state = RouteState.RESUME_SAVED_SCAN_LEG_GOAL
        return [
            SetFullnessContext("off"),
            SetFullnessEnabled(False),
            SetNavCmdEnabled(True),
            self._navigate(resume_goal),
            self._status(reason),
        ]

    def _arrive_at_patrol_leg_goal(self) -> list[Command]:
        if self._active_zone_id:
            self._checked_zone_ids.add(self._active_zone_id)
            self._clear_active_check()
        current_index = self._leg_index_or_raise()
        return self._begin_leg((current_index + 1) % len(self._legs))

    def _begin_leg(self, index: int) -> list[Command]:
        self._leg_index = index
        leg = self._current_leg()
        self.state = leg.state
        self._clear_active_check()
        commands: list[Command] = [SetFullnessContext("off")]
        if leg.scans:
            self._epoch += 1
            self._checked_zone_ids.clear()
            self._skip_cart_slots.clear()
            commands.extend([ResetFullness(), SetFullnessEnabled(True)])
        else:
            commands.append(SetFullnessEnabled(False))
        commands.append(SetNavCmdEnabled(True))
        commands.extend([self._navigate(leg.goal_name), self._status("patrol_leg_started")])
        return commands

    def _navigate(self, goal_name: str) -> Navigate:
        self._expected_goal_name = goal_name
        return Navigate(goal_name)

    def _fault(self, reason: str) -> list[Command]:
        self.state = RouteState.FAULT_HOLD
        self._reset_pipeline_tracking()
        self._clear_active_check()
        return [
            SetFullnessContext("off"),
            SetFullnessEnabled(False),
            SetNavCmdEnabled(False),
            self._status("fault_hold", error=reason),
        ]

    def _status(self, event: str, **extra: Any) -> PublishStatus:
        payload: dict[str, Any] = {
            "state": self.state.value,
            "event": event,
            "epoch": self._epoch,
            "active_zone": self._active_zone_id,
            "active_check_id": self._active_check_id,
            "expected_goal": self._expected_goal_name,
            "mission_id": self._mission.mission_id if self._mission else None,
            **extra,
        }
        return PublishStatus(payload)

    def _new_scan_check_id(self, leg: PatrolLeg, zone: ScanZone) -> str:
        return f"epoch_{self._epoch}_{leg.leg_id}_{zone.zone_id}"

    def _current_leg(self) -> PatrolLeg:
        return self._legs[self._leg_index_or_raise()]

    def _leg_index_or_raise(self) -> int:
        if self._leg_index is None:
            raise RuntimeError("no active patrol leg")
        return self._leg_index

    def _in_scan_leg(self) -> bool:
        return self._in_patrol_state() and self._current_leg().scans

    def _in_patrol_state(self) -> bool:
        return self.state in {
            RouteState.LEFT_SCAN_LEG,
            RouteState.LEFT_TO_RIGHT_TRANSIT,
            RouteState.RIGHT_SCAN_LEG,
            RouteState.RIGHT_TO_LEFT_TRANSIT,
        }

    def _clear_active_check(self) -> None:
        self._active_zone_id = ""
        self._active_check_id = ""

    def _reset_pipeline_tracking(self) -> None:
        self._pipeline_request_status_sequence = None
        self._pipeline_running_status_sequence = None
        self._pipeline_succeeded_status_sequence = None

    def _validate_configuration(self) -> None:
        for leg in self._legs:
            if leg.scans:
                for zone_id in leg.zone_ids:
                    if zone_id not in self._zones:
                        raise ValueError(f"unknown scan zone {zone_id!r} in leg {leg.leg_id!r}")
            elif leg.zone_ids:
                raise ValueError(f"transit leg {leg.leg_id!r} must not declare scan zones")
