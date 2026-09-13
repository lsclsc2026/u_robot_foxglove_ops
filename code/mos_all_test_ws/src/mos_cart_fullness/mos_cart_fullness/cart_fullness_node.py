"""ROS2 node for wrist-camera 2D cart fullness route signals."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .cart_fullness import CartFullnessJudge
from .check_context import fuse_view_decisions, parse_check_command
from .multiview_state import (
    FrameFullnessEvaluator,
    FrameSnapshot,
    ItemCountFullnessEvaluator,
    LatestFrameStore,
    ViewEvaluation,
)
from .wrist_yolo_detector import WristYoloDetector


DETECTION_WINDOW_FRAMES = 10
SPECIAL_STATUS_EMPTY_MIN_FRAMES = 5
SPECIAL_STATUS_NO_CART_MIN_FRAMES = 6


def parse_mapping(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for raw_item in str(text or "").split(","):
        item = raw_item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"mapping item must look like key:value, got {item!r}")
        key, value = item.split(":", 1)
        result[key.strip()] = value.strip()
    return result


def parse_int_set(value) -> set[int]:
    if value is None:
        return set()
    if isinstance(value, str):
        items = value.split(",")
    elif isinstance(value, (list, tuple)):
        items = value
    else:
        items = [value]
    return {int(str(item).strip()) for item in items if str(item).strip()}


def extract_waypoint(data: str) -> str:
    text = str(data or "").strip()
    if not text:
        return ""
    try:
        payload = json.loads(text)
    except Exception:
        return text
    if isinstance(payload, dict):
        return str(payload.get("waypoint") or payload.get("data") or "").strip()
    return text


def build_result_payload(
    *,
    context,
    wrist: ViewEvaluation,
    head: ViewEvaluation | None,
    final_decision: str,
    full_signal: str,
    not_full_signal: str,
    full_route_step_delta: int,
    not_full_route_step_delta: int,
    reason: str = "",
) -> dict[str, Any]:
    primary = wrist.fullness_result or {}
    summary = {"full_frames": 0, "not_full_frames": 0, **(wrist.summary or {})}
    is_full = final_decision == "full"
    return {
        "waypoint": context.waypoint,
        "camera": context.wrist_camera,
        "cart_id": context.cart_id,
        "signal": full_signal if is_full else not_full_signal,
        "decision": final_decision,
        "route_step_delta": (
            full_route_step_delta if is_full else not_full_route_step_delta
        ),
        "reason": reason or wrist.reason,
        "occupancy_ratio": float(primary.get("occupancy_ratio", 0.0)),
        "used_item_count": int(primary.get("used_item_count", 0)),
        **summary,
        "check_id": context.check_id,
        "mode": context.mode,
        "zone_id": context.zone_id,
        "wrist_camera": context.wrist_camera,
        "wrist_decision": wrist.decision,
        "head_decision": head.decision if head is not None else "not_used",
        "wrist_used_item_count": int(primary.get("used_item_count", 0)),
        "head_used_item_count": int(
            (head.fullness_result or {}).get("used_item_count", 0)
        ) if head is not None else 0,
    }


@dataclass(frozen=True)
class EvaluatedView:
    snapshot: FrameSnapshot | None
    evaluation: ViewEvaluation


class CartFullnessNode(Node):
    def __init__(self):
        super().__init__("mos_cart_fullness")
        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge()
        self.frames = LatestFrameStore()
        self.frame_errors = {"left": "", "right": "", "head": ""}

        self.detector = WristYoloDetector(
            model_size=self.model_size,
            weights=self.weights,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
        )
        self.judge = CartFullnessJudge(
            full_threshold=self.full_threshold,
            count_threshold=self.count_threshold,
            count_operator=self.count_operator,
            item_class_ids=self.item_class_ids,
            cart_class_ids=self.cart_class_ids,
            min_item_score=self.min_item_score,
            min_cart_score=self.min_cart_score,
            x_margin_ratio=self.x_margin_ratio,
            y_start_ratio=self.y_start_ratio,
            y_end_ratio=self.y_end_ratio,
            min_item_roi_overlap=self.min_item_roi_overlap,
        )
        self.wrist_evaluator = FrameFullnessEvaluator.create_default(
            self.judge,
            decision_window=self.decision_window,
            min_full_frames=self.min_full_frames,
            min_not_full_frames=self.min_not_full_frames,
            empty_cart_class_ids=self.empty_cart_class_ids,
            loaded_cart_class_ids=self.loaded_cart_class_ids,
        )
        self.verify_wrist_evaluator = ItemCountFullnessEvaluator.create_default(
            self.judge,
            decision_window=self.decision_window,
            min_full_frames=self.min_full_frames,
            min_not_full_frames=self.min_not_full_frames,
            count_threshold=self.verify_wrist_item_count_threshold,
        )
        self.head_judge = CartFullnessJudge(
            full_threshold=self.head_full_threshold,
            count_threshold=self.head_count_threshold,
            count_operator=self.count_operator,
            item_class_ids=self.item_class_ids,
            cart_class_ids=self.cart_class_ids,
            min_item_score=self.head_min_item_score,
            min_cart_score=self.head_min_cart_score,
            x_margin_ratio=self.head_x_margin_ratio,
            y_start_ratio=self.head_y_start_ratio,
            y_end_ratio=self.head_y_end_ratio,
            min_item_roi_overlap=self.head_min_item_roi_overlap,
        )
        self.head_evaluator = FrameFullnessEvaluator.create_default(
            self.head_judge,
            decision_window=self.decision_window,
            min_full_frames=self.min_full_frames,
            min_not_full_frames=self.min_not_full_frames,
            empty_cart_class_ids=self.empty_cart_class_ids,
            loaded_cart_class_ids=self.loaded_cart_class_ids,
        )

        self.active_context = None
        self.context_started_monotonic = None
        self.decision_published = False
        self._reset_special_status_confirmation()
        self.completed_check_ids: set[str] = set()

        self.sub_waypoint = self.create_subscription(
            String,
            self.waypoint_topic,
            self._on_waypoint,
            10,
        )
        self.sub_left_rgb = self.create_subscription(
            Image,
            self.left_rgb_topic,
            self._on_left_rgb,
            qos_profile_sensor_data,
        )
        self.sub_right_rgb = self.create_subscription(
            Image,
            self.right_rgb_topic,
            self._on_right_rgb,
            qos_profile_sensor_data,
        )
        self.sub_head_rgb = self.create_subscription(
            Image,
            self.head_rgb_topic,
            self._on_head_rgb,
            qos_profile_sensor_data,
        )
        self.pub_route_signal = self.create_publisher(String, self.route_signal_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.srv_set_enabled = self.create_service(
            SetBool,
            "~/set_enabled",
            self._handle_set_enabled,
        )
        self.srv_reset = self.create_service(Trigger, "~/reset", self._handle_reset)
        self.timer = self.create_timer(self.camera_poll_seconds, self._tick)

        self.get_logger().info(
            f"started mos_cart_fullness enabled={self.enabled} "
            f"waypoint_topic={self.waypoint_topic} route_signal={self.route_signal_topic} "
            f"left_rgb_topic={self.left_rgb_topic} right_rgb_topic={self.right_rgb_topic} "
            f"head_rgb_topic={self.head_rgb_topic}"
        )

    def _declare_parameters(self):
        defaults = {
            "enabled": True,
            "camera_poll_seconds": 0.10,
            "left_rgb_topic": "/left_arm_camera/color/image_raw",
            "right_rgb_topic": "/right_arm_camera/color/image_raw",
            "head_rgb_topic": "/head_camera/color/image_raw",
            "max_frame_age_sec": 0.50,
            "max_pair_delta_sec": 0.15,
            "verify_timeout_sec": 5.0,
            "head_min_cart_score": 0.25,
            "head_min_item_score": 0.25,
            "head_full_threshold": 0.60,
            "head_count_threshold": 5,
            "head_x_margin_ratio": 0.15,
            "head_y_start_ratio": 0.45,
            "head_y_end_ratio": 0.90,
            "head_min_item_roi_overlap": 0.10,
            "verify_wrist_item_count_threshold": 1,
            "waypoint_topic": "/cart_fullness/set_waypoint",
            "route_signal_topic": "/cart_fullness/route_signal",
            "status_topic": "/cart_fullness/status",
            "waypoint_camera_map": "left_start:left,right_start:right",
            "waypoint_cart_map": "left_start:cart_left,right_start:cart_right",
            "weights": "",
            "model_size": "nano",
            "conf": 0.15,
            "iou": 0.45,
            "device": "cuda",
            "track": False,
            "cart_class_ids": "0,1,2,3,4,5",
            "empty_cart_class_ids": "0,1,2",
            "loaded_cart_class_ids": "3,4,5",
            "item_class_ids": "9,10",
            "min_cart_score": 0.25,
            "min_item_score": 0.25,
            "full_threshold": 0.60,
            "count_threshold": 5,
            "count_operator": ">=",
            "x_margin_ratio": 0.15,
            "y_start_ratio": 0.55,
            "y_end_ratio": 0.88,
            "min_item_roi_overlap": 0.10,
            "decision_window": 5,
            "min_full_frames": 3,
            "min_not_full_frames": 3,
            "full_signal": "continue_to_next_waypoint_for_grasp",
            "not_full_signal": "skip_next_waypoint",
            "full_route_step_delta": 1,
            "not_full_route_step_delta": 2,
            "publish_status": True,
            "disable_after_decision": True,
            "preview": False,
            "preview_after_decision": True,
            "preview_window": "mos_cart_fullness",
            "preview_wait_ms": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.enabled = bool(self.get_parameter("enabled").value)
        self.camera_poll_seconds = max(
            0.01,
            float(self.get_parameter("camera_poll_seconds").value),
        )
        self.left_rgb_topic = str(self.get_parameter("left_rgb_topic").value)
        self.right_rgb_topic = str(self.get_parameter("right_rgb_topic").value)
        self.head_rgb_topic = str(self.get_parameter("head_rgb_topic").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
        self.max_pair_delta_sec = float(self.get_parameter("max_pair_delta_sec").value)
        self.verify_timeout_sec = float(self.get_parameter("verify_timeout_sec").value)
        self.head_min_cart_score = float(self.get_parameter("head_min_cart_score").value)
        self.head_min_item_score = float(self.get_parameter("head_min_item_score").value)
        self.head_full_threshold = float(self.get_parameter("head_full_threshold").value)
        self.head_count_threshold = int(self.get_parameter("head_count_threshold").value)
        self.head_x_margin_ratio = float(self.get_parameter("head_x_margin_ratio").value)
        self.head_y_start_ratio = float(self.get_parameter("head_y_start_ratio").value)
        self.head_y_end_ratio = float(self.get_parameter("head_y_end_ratio").value)
        self.head_min_item_roi_overlap = float(
            self.get_parameter("head_min_item_roi_overlap").value
        )
        self.verify_wrist_item_count_threshold = int(
            self.get_parameter("verify_wrist_item_count_threshold").value
        )
        self.waypoint_topic = str(self.get_parameter("waypoint_topic").value)
        self.route_signal_topic = str(self.get_parameter("route_signal_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.waypoint_camera_map = parse_mapping(
            str(self.get_parameter("waypoint_camera_map").value)
        )
        self.waypoint_cart_map = parse_mapping(
            str(self.get_parameter("waypoint_cart_map").value)
        )
        self.weights = str(self.get_parameter("weights").value)
        self.model_size = str(self.get_parameter("model_size").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.device = str(self.get_parameter("device").value)
        self.track = bool(self.get_parameter("track").value)
        self.cart_class_ids = parse_int_set(self.get_parameter("cart_class_ids").value)
        self.empty_cart_class_ids = parse_int_set(
            self.get_parameter("empty_cart_class_ids").value
        )
        self.loaded_cart_class_ids = parse_int_set(
            self.get_parameter("loaded_cart_class_ids").value
        )
        self.item_class_ids = parse_int_set(self.get_parameter("item_class_ids").value)
        self.min_cart_score = float(self.get_parameter("min_cart_score").value)
        self.min_item_score = float(self.get_parameter("min_item_score").value)
        self.full_threshold = float(self.get_parameter("full_threshold").value)
        self.count_threshold = int(self.get_parameter("count_threshold").value)
        self.count_operator = str(self.get_parameter("count_operator").value)
        self.x_margin_ratio = float(self.get_parameter("x_margin_ratio").value)
        self.y_start_ratio = float(self.get_parameter("y_start_ratio").value)
        self.y_end_ratio = float(self.get_parameter("y_end_ratio").value)
        self.min_item_roi_overlap = float(
            self.get_parameter("min_item_roi_overlap").value
        )
        self.decision_window = int(self.get_parameter("decision_window").value)
        self.min_full_frames = int(self.get_parameter("min_full_frames").value)
        self.min_not_full_frames = int(self.get_parameter("min_not_full_frames").value)
        self.full_signal = str(self.get_parameter("full_signal").value)
        self.not_full_signal = str(self.get_parameter("not_full_signal").value)
        self.full_route_step_delta = int(
            self.get_parameter("full_route_step_delta").value
        )
        self.not_full_route_step_delta = int(
            self.get_parameter("not_full_route_step_delta").value
        )
        self.publish_status = bool(self.get_parameter("publish_status").value)
        self.disable_after_decision = bool(
            self.get_parameter("disable_after_decision").value
        )
        self.preview = bool(self.get_parameter("preview").value)
        self.preview_after_decision = bool(
            self.get_parameter("preview_after_decision").value
        )
        self.preview_window = str(self.get_parameter("preview_window").value)
        self.preview_wait_ms = max(1, int(self.get_parameter("preview_wait_ms").value))

    def _on_waypoint(self, msg: String):
        try:
            command = parse_check_command(
                msg.data,
                waypoint_camera_map=self.waypoint_camera_map,
                waypoint_cart_map=self.waypoint_cart_map,
                default_verify_timeout_sec=self.verify_timeout_sec,
            )
        except ValueError as exc:
            self._publish_status("invalid_command", reason=str(exc))
            return

        if command.action == "off":
            self._reset_active_state()
            self.enabled = False
            self._publish_status("disabled", reason="waypoint_disable")
            return
        if command.action == "reset":
            self._reset_active_state()
            getattr(self, "completed_check_ids", set()).clear()
            self._publish_status("reset", reason="waypoint_reset")
            return

        context = command.context
        if context is None:
            self._publish_status("invalid_command", reason="missing_context")
            return

        completed_check_ids = getattr(self, "completed_check_ids", set())
        if not context.legacy and context.check_id in completed_check_ids:
            self._publish_status(
                "duplicate_check_ignored",
                waypoint=context.waypoint,
                camera=context.wrist_camera,
                cart_id=context.cart_id,
                check_id=context.check_id,
                mode=context.mode,
                zone_id=context.zone_id,
                reason="check_already_completed",
            )
            return

        # Replace the context and all per-check state as one logical operation.
        self._reset_active_state()
        self.active_context = context
        self.context_started_monotonic = time.monotonic()
        self.decision_published = False
        self.enabled = True

    @property
    def active_waypoint(self) -> str:
        return self.active_context.waypoint if self.active_context else ""

    @property
    def active_camera(self) -> str:
        return self.active_context.wrist_camera if self.active_context else ""

    @property
    def active_cart_id(self) -> str:
        return self.active_context.cart_id if self.active_context else ""

    def _tick(self):
        started = time.monotonic()
        self._tick_impl()
        elapsed = time.monotonic() - started
        self.last_inference_duration_sec = elapsed

    def _tick_impl(self):
        context = self.active_context
        if context is None:
            return
        if self.decision_published and (
            getattr(self, "_detection_complete", False) or not self.enabled
        ):
            if self.preview and self.preview_after_decision:
                wrist_view = self._evaluate_view(
                    context.wrist_camera,
                    self._wrist_evaluator_for_context(context),
                )
                if context.mode == "scan_wrist":
                    self._show_view_preview(wrist_view, decision="published")
                else:
                    head_view = self._evaluate_synchronized_head(context, wrist_view)
                    self._show_dual_preview(
                        context,
                        wrist_view,
                        head_view,
                        fused_decision="published",
                    )
            return
        if not self.enabled:
            return
        if getattr(self, "_detection_complete", False):
            return

        if context.mode == "scan_wrist":
            wrist_view = self._evaluate_view(context.wrist_camera, self.wrist_evaluator)
            wrist = wrist_view.evaluation
            self._show_view_preview(wrist_view, decision=wrist.decision)
            frame_complete = self._advance_detection_frame()
            special_status_ready = self._should_publish_special_status(wrist.decision)
            self._finish_if_decided(context, wrist, None, wrist.decision)
            if special_status_ready:
                self._publish_checking_status(
                    context,
                    wrist,
                    status_decision=self._special_status_result,
                    status_reason=self._special_status_reason,
                    special_summary=self._special_status_summary(),
                )
            elif frame_complete:
                self._publish_pending_route_status()
            self._finish_detection_if_complete()
            return

        wrist_view, head_view = self._evaluate_dual_views(context)
        wrist = wrist_view.evaluation
        head = head_view.evaluation
        timed_out = self._check_timed_out(context)
        final = fuse_view_decisions(context.mode, wrist.decision, head.decision, timed_out)
        self._show_dual_preview(
            context,
            wrist_view,
            head_view,
            fused_decision=final or "unknown",
        )
        frame_complete = self._advance_detection_frame()
        special_status_ready = self._should_publish_special_status(wrist.decision)
        self._finish_if_decided(
            context,
            wrist,
            head,
            final,
            reason="verify_timeout" if timed_out and final == "not_full" else "",
        )
        if special_status_ready:
            self._publish_checking_status(
                context,
                wrist,
                head,
                status_decision=self._special_status_result,
                status_reason=self._special_status_reason,
                special_summary=self._special_status_summary(),
            )
        elif frame_complete:
            self._publish_pending_route_status()
        self._finish_detection_if_complete()

    @staticmethod
    def _unknown_evaluation(
        reason: str,
        detections: list[dict[str, Any]] | None = None,
        cart_candidates: list[dict[str, Any]] | None = None,
    ) -> ViewEvaluation:
        return ViewEvaluation(
            decision="unknown",
            reason=reason,
            detections=detections or [],
            cart_candidates=cart_candidates or [],
            target=None, stable_frames=0, fullness_result=None,
            summary={"full_frames": 0, "not_full_frames": 0},
        )

    def _evaluate_snapshot(
        self,
        camera: str,
        snapshot: FrameSnapshot | None,
        evaluator: FrameFullnessEvaluator,
        *,
        update_state: bool = True,
        reason: str = "",
    ) -> EvaluatedView:
        if snapshot is None or snapshot.frame is None or snapshot.frame.size == 0:
            return EvaluatedView(
                snapshot=None,
                evaluation=self._unknown_evaluation(
                    reason or self.frame_errors.get(camera) or "no_rgb_frame"
                ),
            )
        detections = self.detector.detect(snapshot.frame, track=False)
        if update_state:
            evaluation = evaluator.evaluate(detections)
        else:
            cart_candidates = (
                self._cart_candidates(detections)
                if hasattr(self, "judge")
                else []
            )
            evaluation = self._unknown_evaluation(
                reason or "unsynchronized_views",
                detections=detections,
                cart_candidates=cart_candidates,
            )
        return EvaluatedView(snapshot=snapshot, evaluation=evaluation)

    def _get_snapshot(self, camera: str) -> FrameSnapshot | None:
        return self.frames.get(
            camera, now_sec=time.monotonic(), max_age_sec=self.max_frame_age_sec
        )

    def _evaluate_view(self, camera: str, evaluator: FrameFullnessEvaluator):
        snapshot = self._get_snapshot(camera)
        return self._evaluate_snapshot(camera, snapshot, evaluator)

    def _evaluate_dual_views(self, context):
        wrist_snapshot = self._get_snapshot(context.wrist_camera)
        head_snapshot = self._get_snapshot("head")
        # Do not require the two cameras to share a timestamp. Each camera's
        # latest fresh frame is valid evidence for its own fullness decision.
        wrist_view = self._evaluate_snapshot(
            context.wrist_camera,
            wrist_snapshot,
            self._wrist_evaluator_for_context(context),
        )
        head_view = self._evaluate_snapshot(
            "head",
            head_snapshot,
            self.head_evaluator,
            reason="no_head_frame",
        )
        return wrist_view, head_view

    def _wrist_evaluator_for_context(self, context):
        if context.mode == "verify_dual":
            # The fallback keeps lightweight unit-test doubles and older
            # embedding code compatible; the real node always creates the
            # dedicated loose evaluator in __init__.
            return getattr(self, "verify_wrist_evaluator", self.wrist_evaluator)
        return self.wrist_evaluator

    def _evaluate_synchronized_head(self, context, wrist_view: EvaluatedView):
        """Evaluate the latest head frame without timestamp matching.

        The method name is retained for compatibility with existing tests and
        callers; dual-view decisions no longer require synchronization.
        """
        head_snapshot = self._get_snapshot("head")
        if head_snapshot is None or head_snapshot.frame is None or head_snapshot.frame.size == 0:
            return EvaluatedView(
                snapshot=head_snapshot,
                evaluation=self._unknown_evaluation("no_head_frame"),
            )
        detections = self.detector.detect(head_snapshot.frame, track=False)
        return EvaluatedView(
            snapshot=head_snapshot,
            evaluation=self.head_evaluator.evaluate(detections),
        )

    def _check_timed_out(self, context) -> bool:
        if context.timeout_sec <= 0.0 or self.context_started_monotonic is None:
            return False
        return time.monotonic() - self.context_started_monotonic >= context.timeout_sec

    def _finish_if_decided(self, context, wrist, head, decision: str, reason: str = ""):
        if decision not in {"full", "not_full"} or self.decision_published:
            return
        if not reason and context.mode == "scan_wrist":
            reason = (
                "frame_cart_full"
                if decision == "full"
                else "frame_cart_not_full"
            )
        if not reason and decision == "not_full" and head:
            reason = "dual_view_not_confirmed"
        payload = build_result_payload(
            context=context, wrist=wrist, head=head, final_decision=decision,
            full_signal=self.full_signal, not_full_signal=self.not_full_signal,
            full_route_step_delta=self.full_route_step_delta,
            not_full_route_step_delta=self.not_full_route_step_delta,
            reason=reason,
        )
        self.pub_route_signal.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.decision_published = True
        self._pending_route_status_payload = payload
        if not hasattr(self, "completed_check_ids"):
            self.completed_check_ids = set()
        self.completed_check_ids.add(context.check_id)

    def _publish_checking_status(
        self,
        context,
        wrist: ViewEvaluation,
        head=None,
        *,
        status_decision: str | None = None,
        status_reason: str | None = None,
        special_summary: dict[str, int] | None = None,
    ):
        primary = wrist.fullness_result or {}
        summary = {"full_frames": 0, "not_full_frames": 0, **(wrist.summary or {})}
        decision = wrist.decision if status_decision is None else status_decision
        reason = wrist.reason if status_reason is None else status_reason
        if status_reason is None and head is not None and head.reason in {
            "no_head_frame", "unsynchronized_views"
        }:
            reason = head.reason
        elif wrist.decision == "unknown" and wrist.fullness_result is not None:
            reason = "collecting_fullness_frames"
        self._publish_status(
            "checking",
            decision=decision,
            reason=reason,
            stable_frames=wrist.stable_frames,
            cart_candidates=len(wrist.cart_candidates),
            occupancy_ratio=float(primary.get("occupancy_ratio", 0.0)),
            used_item_count=int(primary.get("used_item_count", 0)),
            **summary,
            check_id=context.check_id,
            mode=context.mode,
            zone_id=context.zone_id,
            wrist_decision=wrist.decision,
            head_decision=head.decision if head is not None else "not_used",
            **(special_summary or {}),
        )

    def _reset_special_status_confirmation(self):
        self._special_status_history: list[str] = []
        self._special_status_result = ""
        self._special_status_reason = ""
        self._special_status_published = False
        self._detection_frames = 0
        self._detection_complete = False
        self._pending_route_status_payload = None

    def _should_publish_special_status(self, decision: str) -> bool:
        if getattr(self, "_special_status_published", False):
            return False

        history = getattr(self, "_special_status_history", [])
        if len(history) >= DETECTION_WINDOW_FRAMES:
            return False

        history.append(
            decision if decision in {"empty_cart", "no_cart"} else "other"
        )
        self._special_status_history = history
        if len(history) < DETECTION_WINDOW_FRAMES:
            return False

        empty_frames = history.count("empty_cart")
        no_cart_frames = history.count("no_cart")
        if empty_frames >= SPECIAL_STATUS_EMPTY_MIN_FRAMES:
            self._special_status_result = "empty_cart"
            self._special_status_reason = "empty_cart_confirmed_5_of_10"
        elif no_cart_frames >= SPECIAL_STATUS_NO_CART_MIN_FRAMES:
            self._special_status_result = "no_cart"
            self._special_status_reason = "no_cart_confirmed_6_of_10"
        else:
            return False

        self._special_status_published = True
        return True

    def _advance_detection_frame(self) -> bool:
        self._detection_frames = getattr(self, "_detection_frames", 0) + 1
        self._detection_complete = self._detection_frames >= DETECTION_WINDOW_FRAMES
        return self._detection_complete

    def _publish_pending_route_status(self):
        payload = getattr(self, "_pending_route_status_payload", None)
        if payload is None:
            return
        self._publish_status("route_signal_published", **payload)
        self._pending_route_status_payload = None

    def _finish_detection_if_complete(self):
        if self._detection_complete and self.disable_after_decision:
            self.enabled = False

    def _special_status_summary(self) -> dict[str, int]:
        history = getattr(self, "_special_status_history", [])
        return {
            "sampled_frames": len(history),
            "empty_cart_frames": history.count("empty_cart"),
            "no_cart_frames": history.count("no_cart"),
        }

    def _show_view_preview(self, view: EvaluatedView, *, decision: str):
        if view.snapshot is None:
            return
        result = view.evaluation
        self._show_preview(
            view.snapshot.frame,
            detections=result.detections,
            cart_candidates=result.cart_candidates,
            target=result.target,
            fullness_result=result.fullness_result,
            decision=decision,
            reason=result.reason,
            stable_frames=result.stable_frames,
            summary=result.summary,
        )

    def _cart_candidates(self, detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return self.judge.filter_detections_by_class(
            detections,
            self.cart_class_ids,
            min_score=self.min_cart_score,
        )

    def _show_preview(
        self,
        image,
        *,
        detections: list[dict[str, Any]],
        cart_candidates: list[dict[str, Any]],
        target: dict[str, Any] | None = None,
        fullness_result: dict[str, Any] | None = None,
        decision: str = "",
        reason: str = "",
        stable_frames: int = 0,
        summary: dict[str, int] | None = None,
    ):
        if not self.preview:
            return

        frame = self._render_preview_frame(
            image,
            detections=detections,
            cart_candidates=cart_candidates,
            target=target,
            fullness_result=fullness_result,
            decision=decision,
            reason=reason,
            stable_frames=stable_frames,
            summary=summary,
            view_label=self.active_camera or "wrist",
            mode=self.active_context.mode if self.active_context else "scan_wrist",
            check_id=self.active_context.check_id if self.active_context else "",
            wrist_decision=decision,
            head_decision="not_used",
            fused_decision=decision,
        )
        cv2.imshow(
            self._preview_window_name(self.active_camera or "wrist"),
            frame,
        )
        self._handle_preview_key()

    def _render_preview_frame(
        self,
        image,
        *,
        detections: list[dict[str, Any]],
        cart_candidates: list[dict[str, Any]],
        target: dict[str, Any] | None = None,
        fullness_result: dict[str, Any] | None = None,
        decision: str = "",
        reason: str = "",
        stable_frames: int = 0,
        summary: dict[str, int] | None = None,
        view_label: str = "",
        mode: str = "",
        check_id: str = "",
        wrist_decision: str = "",
        head_decision: str = "",
        fused_decision: str = "",
    ):
        frame = image.copy()

        item_class_ids = set(self.item_class_ids)
        cart_class_ids = set(self.cart_class_ids)

        for det in detections:
            class_id = int(det.get("class_id", -1))
            score = float(det.get("score", 0.0) or 0.0)
            label = f"{class_id}:{det.get('class_name', class_id)} {score:.2f}"
            color = (180, 180, 180)
            thickness = 1
            if class_id in item_class_ids:
                color = (0, 220, 0)
                thickness = 2
            elif class_id in cart_class_ids:
                color = (0, 165, 255)
                thickness = 2
            self._draw_bbox(frame, det.get("bbox_xyxy"), color, label, thickness)

        status, status_color = self._preview_status(
            decision=fused_decision or decision,
            fullness_result=fullness_result,
        )
        self._draw_status(frame, status, status_color)
        return frame

    def _show_dual_preview(
        self,
        context,
        wrist_view: EvaluatedView,
        head_view: EvaluatedView,
        *,
        fused_decision: str,
    ):
        if not self.preview:
            return

        wrist_result = wrist_view.evaluation
        if wrist_view.snapshot is not None:
            wrist_frame = self._render_preview_frame(
                wrist_view.snapshot.frame,
                detections=wrist_result.detections,
                cart_candidates=wrist_result.cart_candidates,
                target=wrist_result.target,
                fullness_result=wrist_result.fullness_result,
                decision=wrist_result.decision,
                reason=wrist_result.reason,
                stable_frames=wrist_result.stable_frames,
                summary=wrist_result.summary,
                view_label=f"wrist:{context.wrist_camera}",
                mode=context.mode,
                check_id=context.check_id,
                wrist_decision=wrist_result.decision,
                head_decision=head_view.evaluation.decision,
                fused_decision=fused_decision,
            )
            cv2.imshow(
                self._preview_window_name(f"wrist_{context.wrist_camera}"),
                wrist_frame,
            )

        head_result = head_view.evaluation
        if head_view.snapshot is not None:
            head_frame = self._render_preview_frame(
                head_view.snapshot.frame,
                detections=head_result.detections,
                cart_candidates=head_result.cart_candidates,
                target=head_result.target,
                fullness_result=head_result.fullness_result,
                decision=head_result.decision,
                reason=head_result.reason,
                stable_frames=head_result.stable_frames,
                summary=head_result.summary,
                view_label="head",
                mode=context.mode,
                check_id=context.check_id,
                wrist_decision=wrist_result.decision,
                head_decision=head_result.decision,
                fused_decision=fused_decision,
            )
            cv2.imshow(self._preview_window_name("head"), head_frame)
        self._handle_preview_key()

    def _preview_window_name(self, view_label: str) -> str:
        return f"{self.preview_window}/{view_label}"

    def _destroy_preview_windows(self):
        names = {
            self.preview_window,
            self._preview_window_name("wrist_left"),
            self._preview_window_name("wrist_right"),
            self._preview_window_name("head"),
        }
        for name in names:
            try:
                cv2.destroyWindow(name)
            except cv2.error:
                # OpenCV raises when a window was never created.
                pass

    def _handle_preview_key(self):
        key = cv2.waitKey(self.preview_wait_ms) & 0xFF
        if key in (ord("q"), 27):
            self.preview = False
            self._destroy_preview_windows()

    @staticmethod
    def _draw_bbox(frame, bbox, color, label: str, thickness: int = 2):
        if bbox is None:
            return
        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        if label:
            text_y = max(16, y1 - 6)
            cv2.putText(
                frame,
                str(label),
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

    @staticmethod
    def _preview_status(*, decision: str, fullness_result: dict[str, Any] | None):
        if decision == "published" and fullness_result is not None:
            decision = "full" if fullness_result.get("is_full", False) else "not_full"
        if decision == "full":
            return "FULL", (0, 220, 0)
        if decision == "not_full":
            return "NOT FULL", (0, 0, 255)
        return "CHECKING", (0, 215, 255)

    @staticmethod
    def _draw_status(frame, status: str, color):
        origin = (12, 34)
        cv2.putText(
            frame,
            status,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            status,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            color,
            2,
            cv2.LINE_AA,
        )

    def _publish_route_signal(
        self,
        decision: str,
        fullness_result: dict[str, Any],
        summary: dict[str, int],
    ):
        context = self.active_context
        wrist = ViewEvaluation(
            decision=decision,
            reason=("frame_cart_full" if decision == "full" else "frame_cart_not_full"),
            detections=[], cart_candidates=[], target=None, stable_frames=0,
            fullness_result=fullness_result, summary=summary,
        )
        payload = build_result_payload(
            context=context, wrist=wrist, head=None, final_decision=decision,
            full_signal=self.full_signal, not_full_signal=self.not_full_signal,
            full_route_step_delta=self.full_route_step_delta,
            not_full_route_step_delta=self.not_full_route_step_delta,
            reason=wrist.reason,
        )
        self.pub_route_signal.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self._pending_route_status_payload = payload
        if getattr(self, "_detection_complete", False):
            self._publish_pending_route_status()

    def _publish_status(self, state: str, **kwargs):
        if not self.publish_status:
            return
        payload = {
            "state": state,
            "waypoint": kwargs.pop("waypoint", self.active_waypoint),
            "camera": kwargs.pop("camera", self.active_camera),
            "cart_id": kwargs.pop("cart_id", self.active_cart_id),
            **kwargs,
        }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_left_rgb(self, msg: Image):
        self._on_rgb("left", msg)

    def _on_right_rgb(self, msg: Image):
        self._on_rgb("right", msg)

    def _on_head_rgb(self, msg: Image):
        self._on_rgb("head", msg)

    def _on_rgb(self, camera: str, msg: Image):
        received_sec = time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.frame_errors[camera] = "image_conversion_failed"
            self.get_logger().warn(f"{camera} RGB conversion failed: {exc}")
            return
        self.frame_errors[camera] = ""
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.frames.update(
            camera,
            image,
            stamp_sec=stamp_sec,
            received_sec=received_sec,
        )

    def _reset_active_state(self):
        self.active_context = None
        self.context_started_monotonic = None
        self.decision_published = False
        self._reset_special_status_confirmation()
        self.wrist_evaluator.reset()
        verify_wrist_evaluator = getattr(
            self, "verify_wrist_evaluator", self.wrist_evaluator
        )
        if verify_wrist_evaluator is not self.wrist_evaluator:
            verify_wrist_evaluator.reset()
        self.head_evaluator.reset()

    def _handle_set_enabled(self, request, response):
        if not request.data:
            self._reset_active_state()
            self.enabled = False
        else:
            self.enabled = True
        response.success = True
        response.message = (
            "mos_cart_fullness enabled"
            if self.enabled
            else "mos_cart_fullness disabled"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_reset(self, request, response):
        del request
        self._reset_active_state()
        getattr(self, "completed_check_ids", set()).clear()
        response.success = True
        response.message = "mos_cart_fullness reset"
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        if getattr(self, "preview", False):
            self._destroy_preview_windows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CartFullnessNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
