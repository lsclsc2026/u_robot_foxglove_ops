"""ROS 2 adapter for the deterministic MOS route manager policy."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Callable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from mos_arm_controller.srv import RunTask

from .geometry import SegmentZone
from .goal_loader import GoalPose, load_goal_poses
from .state_machine import (
    Command,
    Navigate,
    PatrolLeg,
    PublishStatus,
    ResetFullness,
    RouteManagerMachine,
    RouteState,
    RunPlace,
    ScanZone,
    SetFullnessContext,
    SetFullnessEnabled,
    SetNavCmdEnabled,
    StartPipeline,
)


class MosRouteManagerNode(Node):
    """Global business-state node that drives Nav2 through NavigateToPose."""

    def __init__(self) -> None:
        super().__init__("mos_route_manager")
        self._declare_parameters()
        self._load_parameters()
        self._goals = self._load_required_goals()
        self.machine = self._build_machine()

        self._latest_pose: tuple[float, float, float, float] | None = None
        self._pending_commands: deque[Command] = deque()
        self._service_busy = False
        self._service_deadline = 0.0
        self._service_generation = 0
        self._verification_deadline = 0.0
        self._navigation_deadline = 0.0
        self._navigation_active = False
        self._navigation_generation = 0
        self._navigation_goal_handle = None
        self._navigation_send_pending = False
        self._navigation_goal_name = ""
        self._navigation_replace_goal: str | None = None
        self._navigation_cancel_generation: int | None = None
        self._navigation_cancel_acknowledged = False
        self._navigation_cancel_result_received = False
        self._navigation_cancel_for_fault = False
        self._navigation_cancel_deadline = 0.0
        self._pipeline_status_sequence = 0
        self._auto_start_timer = None

        self.fullness_command_pub = self.create_publisher(
            String, self.fullness_command_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.localization_sub = self.create_subscription(
            Odometry, self.localization_topic, self._on_localization, 20
        )
        self.fullness_signal_sub = self.create_subscription(
            String, self.fullness_signal_topic, self._on_fullness_signal, 10
        )
        self.pipeline_status_sub = self.create_subscription(
            String, self.pipeline_status_topic, self._on_pipeline_status, 10
        )

        self.navigation_client = ActionClient(
            self, NavigateToPose, self.navigation_action_name
        )
        self.pipeline_start_client = self.create_client(
            Trigger, self.pipeline_start_service
        )
        self.arm_run_task_client = self.create_client(
            RunTask, self.arm_run_task_service
        )
        self.fullness_reset_client = self.create_client(
            Trigger, self.fullness_reset_service
        )
        self.fullness_set_enabled_client = self.create_client(
            SetBool, self.fullness_set_enabled_service
        )
        self.nav_cmd_enabled_client = self.create_client(
            SetBool, self.nav_cmd_enabled_service
        )
        self.start_service = self.create_service(Trigger, "~/start", self._handle_start)
        self.reset_fault_service = self.create_service(
            Trigger, "~/reset_fault", self._handle_reset_fault
        )
        self.tick_timer = self.create_timer(0.1, self._on_tick)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.5, self._on_auto_start)

        self._publish_status(
            {
                "state": self.machine.state.value,
                "event": "ready_waiting_for_manual_start",
                "expected_goal": "",
            }
        )
        self.get_logger().info(
            "started mos_route_manager; wait for map localization, Nav2 Action, "
            "and services before calling /mos_route_manager/start"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "auto_start": False,
            "goal_file": "/opt/slam/config/target_goals.yaml",
            "global_frame": "map",
            "initial_waypoint": "waypoint_left",
            "dirty_car_waypoint": "dirty_car",
            "clean_car_waypoint": "clean_car",
            "required_goal_names": [
                "waypoint_left",
                "waypoint_left_end",
                "waypoint_right",
                "waypoint_right_end",
                "grasp_a",
                "dirty_car",
                "clean_car",
            ],
            "localization_topic": "/localization",
            "navigation_action_name": "/navigate_to_pose",
            "fullness_command_topic": "/cart_fullness/set_waypoint",
            "fullness_signal_topic": "/cart_fullness/route_signal",
            "pipeline_status_topic": "/mos_grasp_pipeline/status",
            "status_topic": "/mos_route_manager/status",
            "pipeline_start_service": "/mos_grasp_pipeline/start",
            "arm_run_task_service": "/mos_arm_controller/run_task",
            "fullness_reset_service": "/mos_cart_fullness/reset",
            "fullness_set_enabled_service": "/mos_cart_fullness/set_enabled",
            "nav_cmd_enabled_service": "/mos_arm_controller/set_nav_cmd_enabled",
            "verify_timeout_sec": 6.0,
            "service_timeout_sec": 15.0,
            "navigation_timeout_sec": 120.0,
            "navigation_cancel_timeout_sec": 5.0,
            "scan_half_width_m": 0.50,
            "left_scan_zone_ids": ["scan_a", "scan_b"],
            "right_scan_zone_ids": ["scan_b", "scan_a"],
            "scan_a_start_x": 0.6621,
            "scan_a_start_y": 0.2248,
            "scan_a_end_x": 3.3063,
            "scan_a_end_y": 0.8032,
            "scan_a_cart_slot": "cart_slot_1",
            "scan_a_pickup_enabled": True,
            "scan_a_grasp_waypoint": "grasp_a",
            "scan_b_start_x": 3.8406,
            "scan_b_start_y": 0.7794,
            "scan_b_end_x": 8.3138,
            "scan_b_end_y": 0.7939,
            "scan_b_cart_slot": "cart_slot_2",
            "scan_b_pickup_enabled": False,
            "scan_b_grasp_waypoint": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self.auto_start = bool(value("auto_start"))
        self.goal_file = Path(str(value("goal_file")))
        self.global_frame = str(value("global_frame"))
        self.initial_waypoint = str(value("initial_waypoint"))
        self.dirty_car_waypoint = str(value("dirty_car_waypoint"))
        self.clean_car_waypoint = str(value("clean_car_waypoint"))
        self.required_goal_names = tuple(str(name) for name in value("required_goal_names"))
        self.localization_topic = str(value("localization_topic"))
        self.navigation_action_name = str(value("navigation_action_name"))
        self.fullness_command_topic = str(value("fullness_command_topic"))
        self.fullness_signal_topic = str(value("fullness_signal_topic"))
        self.pipeline_status_topic = str(value("pipeline_status_topic"))
        self.status_topic = str(value("status_topic"))
        self.pipeline_start_service = str(value("pipeline_start_service"))
        self.arm_run_task_service = str(value("arm_run_task_service"))
        self.fullness_reset_service = str(value("fullness_reset_service"))
        self.fullness_set_enabled_service = str(value("fullness_set_enabled_service"))
        self.nav_cmd_enabled_service = str(value("nav_cmd_enabled_service"))
        self.verify_timeout_sec = float(value("verify_timeout_sec"))
        self.service_timeout_sec = float(value("service_timeout_sec"))
        self.navigation_timeout_sec = float(value("navigation_timeout_sec"))
        self.navigation_cancel_timeout_sec = float(
            value("navigation_cancel_timeout_sec")
        )
        self.scan_half_width_m = float(value("scan_half_width_m"))
        self.left_scan_zone_ids = tuple(str(item) for item in value("left_scan_zone_ids"))
        self.right_scan_zone_ids = tuple(str(item) for item in value("right_scan_zone_ids"))
        self._scan_parameters = {name: value(name) for name in (
            "scan_a_start_x", "scan_a_start_y", "scan_a_end_x", "scan_a_end_y",
            "scan_a_cart_slot", "scan_a_pickup_enabled", "scan_a_grasp_waypoint",
            "scan_b_start_x", "scan_b_start_y", "scan_b_end_x", "scan_b_end_y",
            "scan_b_cart_slot", "scan_b_pickup_enabled", "scan_b_grasp_waypoint",
        )}
        if self.global_frame != "map":
            raise ValueError("mos_route_manager requires global_frame=map")
        if not self.navigation_action_name:
            raise ValueError("navigation_action_name must not be empty")
        if self.service_timeout_sec <= 0.0:
            raise ValueError("service_timeout_sec must be positive")
        if self.navigation_timeout_sec <= 0.0:
            raise ValueError("navigation_timeout_sec must be positive")
        if self.navigation_cancel_timeout_sec <= 0.0:
            raise ValueError("navigation_cancel_timeout_sec must be positive")

    def _load_required_goals(self) -> dict[str, GoalPose]:
        goals = load_goal_poses(self.goal_file)
        missing = sorted(set(self.required_goal_names) - set(goals))
        if missing:
            raise ValueError(
                f"goal file {self.goal_file} is missing required goals: {', '.join(missing)}"
            )
        return goals

    def _build_machine(self) -> RouteManagerMachine:
        zones = {
            "scan_a": self._make_scan_zone("scan_a"),
            "scan_b": self._make_scan_zone("scan_b"),
        }
        legs = (
            PatrolLeg("left_scan", RouteState.LEFT_SCAN_LEG, "waypoint_left_end", "left", self.left_scan_zone_ids),
            PatrolLeg("left_to_right", RouteState.LEFT_TO_RIGHT_TRANSIT, "waypoint_right", "", ()),
            PatrolLeg("right_scan", RouteState.RIGHT_SCAN_LEG, "waypoint_right_end", "right", self.right_scan_zone_ids),
            PatrolLeg("right_to_left", RouteState.RIGHT_TO_LEFT_TRANSIT, "waypoint_left", "", ()),
        )
        return RouteManagerMachine(
            patrol_legs=legs,
            scan_zones=zones,
            initial_waypoint=self.initial_waypoint,
            dirty_car_waypoint=self.dirty_car_waypoint,
            clean_car_waypoint=self.clean_car_waypoint,
            verify_timeout_sec=self.verify_timeout_sec,
        )

    def _make_scan_zone(self, prefix: str) -> ScanZone:
        params = self._scan_parameters
        return ScanZone(
            zone_id=prefix,
            cart_slot=str(params[f"{prefix}_cart_slot"]),
            pickup_enabled=bool(params[f"{prefix}_pickup_enabled"]),
            grasp_waypoint=str(params[f"{prefix}_grasp_waypoint"]),
            corridor=SegmentZone(
                prefix,
                float(params[f"{prefix}_start_x"]),
                float(params[f"{prefix}_start_y"]),
                float(params[f"{prefix}_end_x"]),
                float(params[f"{prefix}_end_y"]),
                self.scan_half_width_m,
            ),
        )

    def _handle_start(self, _request: Trigger.Request, response: Trigger.Response):
        success, message = self._start_patrol()
        response.success = success
        response.message = message
        return response

    def _on_auto_start(self) -> None:
        success, _message = self._start_patrol()
        if success and self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None

    def _start_patrol(self) -> tuple[bool, str]:
        if self.machine.state is not RouteState.WAIT_READY:
            return False, f"route manager is already {self.machine.state.value}"
        if self._latest_pose is None:
            return False, "waiting for map-frame /localization"
        if self._navigation_busy():
            return False, "waiting for previous navigation Action to finish cancellation"
        missing_topics = self._missing_topic_endpoints()
        if missing_topics:
            return False, f"waiting for topic endpoints: {', '.join(missing_topics)}"
        missing = [
            label
            for label, client in self._required_clients()
            if not client.service_is_ready()
        ]
        if missing:
            return False, f"waiting for services: {', '.join(missing)}"
        if not self.navigation_client.server_is_ready():
            return False, f"waiting for Nav2 Action server: {self.navigation_action_name}"
        self._dispatch(self.machine.start())
        return True, "mos_route_manager patrol started"

    def _handle_reset_fault(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.machine.state is not RouteState.FAULT_HOLD:
            response.success = False
            response.message = "route manager is not in fault_hold"
            return response
        if self._navigation_busy():
            response.success = False
            response.message = "waiting for active Nav2 Action cancellation"
            return response
        self._dispatch(self.machine.reset_fault())
        response.success = True
        response.message = "fault reset; call /mos_route_manager/start to re-arm patrol"
        return response

    def _on_localization(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.global_frame:
            self.get_logger().warning(
                f"ignore localization frame={msg.header.frame_id!r}; expected {self.global_frame!r}"
            )
            return
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self._latest_pose = (pose.position.x, pose.position.y, pose.position.z, yaw)
        self._dispatch(self.machine.on_pose(pose.position.x, pose.position.y))

    def _on_fullness_signal(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning("ignore non-JSON cart fullness route signal")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("ignore non-object cart fullness route signal")
            return
        self._dispatch(self.machine.on_route_signal(payload))

    def _on_pipeline_status(self, msg: String) -> None:
        self._pipeline_status_sequence += 1
        self._dispatch(
            self.machine.on_pipeline_status(msg.data, self._pipeline_status_sequence)
        )

    def _on_tick(self) -> None:
        now = time.monotonic()
        if self._service_busy and now >= self._service_deadline:
            self._service_busy = False
            self._service_generation += 1
            self._dispatch(self.machine.on_external_failure("service", "response timeout"))
        if self._navigation_active and now >= self._navigation_deadline:
            self._navigation_active = False
            self._navigation_deadline = 0.0
            self._dispatch(self.machine.on_external_failure("navigation", "goal timeout"))
        if (
            self._navigation_cancel_generation is not None
            and now >= self._navigation_cancel_deadline
        ):
            self._dispatch(self.machine.on_external_failure("navigation", "cancel timeout"))
        if (
            self.machine.state is RouteState.VERIFY_DUAL_AT_GRASP
            and self._verification_deadline > 0.0
            and now >= self._verification_deadline
        ):
            self._verification_deadline = 0.0
            self._dispatch(self.machine.on_verification_timeout())

    def _dispatch(self, commands: list[Command]) -> None:
        if self.machine.state is RouteState.FAULT_HOLD:
            self._navigation_active = False
            self._navigation_deadline = 0.0
            self._navigation_replace_goal = None
            self._request_navigation_cancel(for_fault=True)
            if commands:
                self._pending_commands.clear()
                commands = [
                    command for command in commands if self._is_fault_command(command)
                ]
        self._pending_commands.extend(commands)
        self._run_next_command()

    def _run_next_command(self) -> None:
        if self._service_busy or not self._pending_commands:
            return
        command = self._pending_commands.popleft()
        if self.machine.state is RouteState.FAULT_HOLD and not self._is_fault_command(command):
            self._run_next_command()
            return
        if isinstance(command, Navigate):
            self._navigate_to_goal(command.goal_name)
            self._run_next_command()
            return
        if isinstance(command, SetFullnessContext):
            self._publish_fullness_context(command.payload)
            self._run_next_command()
            return
        if isinstance(command, PublishStatus):
            self._publish_status(command.payload)
            self._run_next_command()
            return
        if isinstance(command, ResetFullness):
            self._call_service(
                self.fullness_reset_client,
                Trigger.Request(),
                self.fullness_reset_service,
                self._continue_after_success,
            )
            return
        if isinstance(command, SetFullnessEnabled):
            request = SetBool.Request()
            request.data = command.enabled
            self._call_service(
                self.fullness_set_enabled_client,
                request,
                self.fullness_set_enabled_service,
                self._continue_after_success,
            )
            return
        if isinstance(command, SetNavCmdEnabled):
            request = SetBool.Request()
            request.data = command.enabled
            self._call_service(
                self.nav_cmd_enabled_client,
                request,
                self.nav_cmd_enabled_service,
                self._continue_after_success,
            )
            return
        if isinstance(command, StartPipeline):
            self.machine.on_pipeline_start_requested(
                command.purpose, self._pipeline_status_sequence
            )
            if self.machine.state is RouteState.FAULT_HOLD:
                return
            self._call_service(
                self.pipeline_start_client,
                Trigger.Request(),
                self.pipeline_start_service,
                lambda response: self._on_pipeline_start_response(command.purpose, response),
            )
            return
        if isinstance(command, RunPlace):
            self._call_service(
                self.arm_run_task_client,
                self._place_request(),
                self.arm_run_task_service,
                self._on_place_service_result,
            )
            return
        self._dispatch(self.machine.on_external_failure("command", str(command)))

    def _call_service(
        self,
        client,
        request,
        label: str,
        on_success: Callable[[object], None],
    ) -> None:
        if not client.service_is_ready():
            self._dispatch(self.machine.on_external_failure(label, "service unavailable"))
            return
        self._service_busy = True
        self._service_deadline = time.monotonic() + self.service_timeout_sec
        self._service_generation += 1
        generation = self._service_generation
        future = client.call_async(request)

        def completed(done_future) -> None:
            if generation != self._service_generation or not self._service_busy:
                return
            self._service_busy = False
            try:
                response = done_future.result()
            except Exception as exc:
                self._dispatch(self.machine.on_external_failure(label, str(exc)))
                return
            if not bool(getattr(response, "success", False)):
                self._dispatch(
                    self.machine.on_external_failure(
                        label, str(getattr(response, "message", "unsuccessful response"))
                    )
                )
                return
            on_success(response)

        future.add_done_callback(completed)

    def _continue_after_success(self, _response: object) -> None:
        self._run_next_command()

    def _on_pipeline_start_response(self, purpose: str, response: object) -> None:
        self._dispatch(
            self.machine.on_pipeline_start_result(
                purpose,
                bool(getattr(response, "success", False)),
                str(getattr(response, "message", "")),
            )
        )

    def _on_place_service_result(self, response: object) -> None:
        self._dispatch(
            self.machine.on_place_result(
                bool(getattr(response, "success", False)),
                str(getattr(response, "message", "")),
            )
        )

    def _navigate_to_goal(self, goal_name: str) -> None:
        if goal_name not in self._goals:
            self._dispatch(
                self.machine.on_external_failure("goal_file", f"unknown goal {goal_name}")
            )
            return
        if self._navigation_send_pending or self._navigation_goal_handle is not None:
            self._navigation_replace_goal = goal_name
            self._request_navigation_cancel(for_fault=False)
            return
        self._send_navigation_goal(goal_name)

    def _send_navigation_goal(self, goal_name: str) -> None:
        if self.machine.state is RouteState.FAULT_HOLD:
            return
        if not self.navigation_client.server_is_ready():
            self._dispatch(
                self.machine.on_external_failure(
                    self.navigation_action_name, "Action server unavailable"
                )
            )
            return
        goal = self._goals[goal_name]
        request = NavigateToPose.Goal()
        request.pose = self._pose_stamped(goal)
        self._navigation_generation += 1
        generation = self._navigation_generation
        self._navigation_goal_name = goal_name
        self._navigation_send_pending = True
        future = self.navigation_client.send_goal_async(request)
        future.add_done_callback(
            lambda done_future: self._on_navigation_goal_response(
                generation, goal_name, done_future
            )
        )
        self.get_logger().info(
            f"sent Nav2 goal request {goal_name} in {self.global_frame}"
        )

    def _on_navigation_goal_response(self, generation: int, goal_name: str, future) -> None:
        if generation != self._navigation_generation:
            return
        self._navigation_send_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._dispatch(
                self.machine.on_external_failure(self.navigation_action_name, str(exc))
            )
            return
        if not goal_handle.accepted:
            replacement = self._navigation_replace_goal
            self._navigation_replace_goal = None
            if replacement and self.machine.state is not RouteState.FAULT_HOLD:
                self._enable_nav_commands_then_send_goal(replacement)
                return
            self._dispatch(
                self.machine.on_external_failure(
                    self.navigation_action_name, f"goal rejected: {goal_name}"
                )
            )
            return

        self._navigation_goal_handle = goal_handle
        self._navigation_active = True
        self._navigation_deadline = time.monotonic() + self.navigation_timeout_sec
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._on_navigation_result(generation, done_future)
        )
        self.get_logger().info(f"Nav2 accepted goal {goal_name}")
        if self.machine.state is RouteState.FAULT_HOLD:
            self._request_navigation_cancel(for_fault=True)
        elif self._navigation_replace_goal is not None:
            self._request_navigation_cancel(for_fault=False)

    def _request_navigation_cancel(self, *, for_fault: bool) -> None:
        if self._navigation_goal_handle is None:
            return
        if self._navigation_cancel_generation == self._navigation_generation:
            self._navigation_cancel_for_fault = self._navigation_cancel_for_fault or for_fault
            return
        self._navigation_cancel_generation = self._navigation_generation
        self._navigation_cancel_acknowledged = False
        self._navigation_cancel_result_received = False
        self._navigation_cancel_for_fault = for_fault
        self._navigation_cancel_deadline = (
            time.monotonic() + self.navigation_cancel_timeout_sec
        )
        generation = self._navigation_generation
        future = self._navigation_goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda done_future: self._on_navigation_cancel_response(
                generation, done_future
            )
        )

    def _on_navigation_cancel_response(self, generation: int, future) -> None:
        if generation != self._navigation_cancel_generation:
            return
        try:
            response = future.result()
        except Exception as exc:
            if self.machine.state is not RouteState.FAULT_HOLD:
                self._dispatch(
                    self.machine.on_external_failure("navigation", f"cancel failed: {exc}")
                )
            return
        if not response.goals_canceling:
            if self.machine.state is not RouteState.FAULT_HOLD:
                self._dispatch(
                    self.machine.on_external_failure("navigation", "cancel rejected")
                )
            return
        self._navigation_cancel_acknowledged = True
        self._finish_navigation_cancel_if_ready()

    def _on_navigation_result(self, generation: int, future) -> None:
        if generation != self._navigation_generation:
            return
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._dispatch(
                self.machine.on_external_failure(self.navigation_action_name, str(exc))
            )
            return

        status = wrapped_result.status
        self._navigation_goal_handle = None
        self._navigation_active = False
        self._navigation_deadline = 0.0
        if status == GoalStatus.STATUS_SUCCEEDED:
            replacement = self._navigation_replace_goal
            cancel_pending = self._navigation_cancel_generation == generation
            self._navigation_cancel_generation = None
            self._navigation_cancel_deadline = 0.0
            self._navigation_cancel_acknowledged = False
            self._navigation_cancel_result_received = False
            self._navigation_cancel_for_fault = False
            self._navigation_replace_goal = None
            if cancel_pending and replacement and self.machine.state is not RouteState.FAULT_HOLD:
                self._enable_nav_commands_then_send_goal(replacement)
                return
            self._dispatch(self.machine.on_goal_reached())
            return
        if status == GoalStatus.STATUS_CANCELED:
            if self._navigation_cancel_generation == generation:
                self._navigation_cancel_result_received = True
                self._finish_navigation_cancel_if_ready()
                return
            if self.machine.state is RouteState.FAULT_HOLD:
                return
            self._dispatch(
                self.machine.on_external_failure("navigation", "goal canceled unexpectedly")
            )
            return
        if self.machine.state is RouteState.FAULT_HOLD:
            return
        status_name = "aborted" if status == GoalStatus.STATUS_ABORTED else str(status)
        self._dispatch(
            self.machine.on_external_failure("navigation", f"goal failed: {status_name}")
        )

    def _finish_navigation_cancel_if_ready(self) -> None:
        if not (
            self._navigation_cancel_acknowledged
            and self._navigation_cancel_result_received
        ):
            return
        for_fault = self._navigation_cancel_for_fault
        replacement = self._navigation_replace_goal
        self._navigation_cancel_generation = None
        self._navigation_cancel_acknowledged = False
        self._navigation_cancel_result_received = False
        self._navigation_cancel_for_fault = False
        self._navigation_cancel_deadline = 0.0
        self._navigation_replace_goal = None
        if for_fault or self.machine.state is RouteState.FAULT_HOLD:
            return
        if not replacement:
            self._dispatch(
                self.machine.on_external_failure("navigation", "missing replacement goal")
            )
            return
        self._enable_nav_commands_then_send_goal(replacement)

    def _enable_nav_commands_then_send_goal(self, goal_name: str) -> None:
        request = SetBool.Request()
        request.data = True
        self._call_service(
            self.nav_cmd_enabled_client,
            request,
            self.nav_cmd_enabled_service,
            lambda _response: self._on_nav_commands_enabled_for_goal(goal_name),
        )

    def _on_nav_commands_enabled_for_goal(self, goal_name: str) -> None:
        """Resume queued state-machine output after an intentional Nav2 preemption."""
        self._send_navigation_goal(goal_name)
        self._run_next_command()

    def _pose_stamped(self, goal: GoalPose) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame
        message.pose.position.x = goal.x
        message.pose.position.y = goal.y
        message.pose.position.z = goal.z
        message.pose.orientation.x = goal.qx
        message.pose.orientation.y = goal.qy
        message.pose.orientation.z = goal.qz
        message.pose.orientation.w = goal.qw
        return message

    def _publish_fullness_context(self, payload: dict | str) -> None:
        if isinstance(payload, dict):
            text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
            if payload.get("mode") == "verify_dual":
                self._verification_deadline = time.monotonic() + float(
                    payload["timeout_sec"]
                )
            else:
                self._verification_deadline = 0.0
        else:
            text = str(payload)
            if text == "off":
                self._verification_deadline = 0.0
        self.fullness_command_pub.publish(String(data=text))

    def _publish_status(self, payload: dict) -> None:
        message = {
            "node": "mos_route_manager",
            "state": self.machine.state.value,
            "expected_goal": self.machine.expected_goal_name,
            **payload,
        }
        self.status_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))

    def _place_request(self) -> RunTask.Request:
        request = RunTask.Request()
        request.task_name = "place"
        request.target_frame = "body_link"
        request.left_point = _zero_point_stamped("body_link")
        request.right_point = _zero_point_stamped("body_link")
        return request

    def _required_clients(self):
        return (
            (self.pipeline_start_service, self.pipeline_start_client),
            (self.arm_run_task_service, self.arm_run_task_client),
            (self.fullness_reset_service, self.fullness_reset_client),
            (self.fullness_set_enabled_service, self.fullness_set_enabled_client),
            (self.nav_cmd_enabled_service, self.nav_cmd_enabled_client),
        )

    def _missing_topic_endpoints(self) -> list[str]:
        checks = (
            (self.localization_topic, self.count_publishers(self.localization_topic)),
            (self.fullness_signal_topic, self.count_publishers(self.fullness_signal_topic)),
            (self.pipeline_status_topic, self.count_publishers(self.pipeline_status_topic)),
            (
                self.fullness_command_topic,
                self.count_subscribers(self.fullness_command_topic),
            ),
        )
        return [topic for topic, count in checks if count <= 0]

    def _navigation_busy(self) -> bool:
        return (
            self._navigation_send_pending
            or self._navigation_goal_handle is not None
            or self._navigation_cancel_generation is not None
        )

    @staticmethod
    def _is_fault_command(command: Command) -> bool:
        return (
            isinstance(command, PublishStatus)
            or isinstance(command, SetFullnessContext) and command.payload == "off"
            or isinstance(command, SetFullnessEnabled) and not command.enabled
            or isinstance(command, SetNavCmdEnabled) and not command.enabled
        )


def _zero_point_stamped(frame_id: str) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = 0.0
    point.point.y = 0.0
    point.point.z = 0.0
    return point


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MosRouteManagerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
