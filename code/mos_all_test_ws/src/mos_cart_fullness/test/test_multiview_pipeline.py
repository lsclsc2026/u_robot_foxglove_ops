import importlib
import json
from pathlib import Path
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mos_cart_fullness.cart_fullness_node import (
    CartFullnessNode,
    build_result_payload,
)
from mos_cart_fullness.check_context import CheckContext
from mos_cart_fullness.multiview_state import FrameSnapshot, ViewEvaluation


def evaluation(decision="unknown", *, used_item_count=0, summary=None):
    fullness = {
        "occupancy_ratio": 0.25 if used_item_count else 0.0,
        "used_item_count": used_item_count,
        "is_full": decision == "full",
    }
    return ViewEvaluation(
        decision=decision,
        reason="test_reason",
        detections=[],
        cart_candidates=[],
        target=None,
        stable_frames=3,
        fullness_result=fullness,
        summary=summary or {"full_frames": 3, "not_full_frames": 0},
    )


def verify_context():
    return CheckContext(
        check_id="trip_cart_b_verify",
        mode="verify_dual",
        waypoint="cart_b_grasp",
        zone_id="scan_cart_b",
        cart_id="cart_b",
        wrist_camera="left",
        timeout_sec=5.0,
    )


def test_verify_dual_publishes_full_only_for_two_full_views():
    payload = build_result_payload(
        context=verify_context(),
        wrist=evaluation("full", used_item_count=1),
        head=evaluation("full", used_item_count=1),
        final_decision="full",
        full_signal="continue_to_next_waypoint_for_grasp",
        not_full_signal="skip_next_waypoint",
        full_route_step_delta=1,
        not_full_route_step_delta=2,
    )

    assert payload["decision"] == "full"
    assert payload["signal"] == "continue_to_next_waypoint_for_grasp"
    assert payload["camera"] == "left"
    assert payload["check_id"] == "trip_cart_b_verify"
    assert payload["wrist_decision"] == "full"
    assert payload["head_decision"] == "full"
    assert payload["wrist_used_item_count"] == 1
    assert payload["head_used_item_count"] == 1


def test_verify_timeout_is_backward_compatible_not_full():
    payload = build_result_payload(
        context=verify_context(),
        wrist=evaluation("unknown", summary={"full_frames": 0, "not_full_frames": 0}),
        head=evaluation("unknown", summary={"full_frames": 0, "not_full_frames": 0}),
        final_decision="not_full",
        full_signal="continue_to_next_waypoint_for_grasp",
        not_full_signal="skip_next_waypoint",
        full_route_step_delta=1,
        not_full_route_step_delta=2,
        reason="verify_timeout",
    )

    assert payload["decision"] == "not_full"
    assert payload["signal"] == "skip_next_waypoint"
    assert payload["route_step_delta"] == 2
    assert "occupancy_ratio" in payload
    assert "used_item_count" in payload
    assert "full_frames" in payload
    assert "not_full_frames" in payload


def test_verify_dual_runs_wrist_then_head_with_track_false_and_publishes_once():
    node = CartFullnessNode.__new__(CartFullnessNode)
    node.active_context = verify_context()
    node.enabled = True
    node.decision_published = False
    node.preview = False
    node.preview_after_decision = True
    node.disable_after_decision = False
    node.max_frame_age_sec = 1.0
    node.max_pair_delta_sec = 0.1
    node.context_started_monotonic = 10.0
    node.verify_timeout_sec = 5.0
    node.frame_errors = {"left": "", "head": ""}
    node.detector_calls = []

    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshots = {
        "left": FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0),
        "head": FrameSnapshot(image, stamp_sec=20.05, received_sec=10.0),
    }

    class Frames:
        def get(self, camera, *, now_sec, max_age_sec):
            return snapshots[camera]

    class Detector:
        def detect(self, frame, *, track):
            node.detector_calls.append((frame, track))
            return []

    class Evaluator:
        def __init__(self, result):
            self.result = result

        def evaluate(self, detections):
            return self.result

    messages = []
    node.frames = Frames()
    node.detector = Detector()
    node.wrist_evaluator = Evaluator(evaluation("full", used_item_count=1))
    node.head_evaluator = Evaluator(evaluation("full", used_item_count=1))
    node.track = True
    node.full_signal = "continue_to_next_waypoint_for_grasp"
    node.not_full_signal = "skip_next_waypoint"
    node.full_route_step_delta = 1
    node.not_full_route_step_delta = 2
    node.pub_route_signal = SimpleNamespace(
        publish=lambda message: messages.append(json.loads(message.data))
    )
    node._show_preview = lambda *args, **kwargs: None
    node._publish_status = lambda *args, **kwargs: None

    for _ in range(10):
        node._tick()

    assert [track for _, track in node.detector_calls] == [False] * 20
    assert len(messages) == 1
    assert messages[0]["decision"] == "full"
    assert messages[0]["head_decision"] == "full"


def test_verify_wrist_accepts_one_item_without_cart_detection():
    from mos_cart_fullness.multiview_state import ItemCountFullnessEvaluator

    judge = SimpleNamespace(
        item_class_ids={9, 10},
        min_item_score=0.25,
        filter_detections_by_class=lambda detections, class_ids, min_score: [
            detection for detection in detections
            if detection.get("class_id") in set(class_ids)
        ],
    )
    evaluator = ItemCountFullnessEvaluator.create_default(
        judge,
        decision_window=1,
        min_full_frames=1,
        min_not_full_frames=1,
        count_threshold=1,
    )

    result = evaluator.evaluate([{"class_id": 9, "bbox_xyxy": [1, 1, 2, 2]}])

    assert result.decision == "full"
    assert result.fullness_result["used_item_count"] == 1
    assert result.fullness_result["is_full"] is True


def test_scan_dual_timeout_keeps_waiting_for_context_replacement():
    node = CartFullnessNode.__new__(CartFullnessNode)
    node.active_context = replace(verify_context(), mode="scan_dual", timeout_sec=0.01)
    node.enabled = True
    node.decision_published = False
    node.preview = False
    node.preview_after_decision = True
    node.max_frame_age_sec = 1.0
    node.max_pair_delta_sec = 0.1
    node.context_started_monotonic = 10.0
    node.frame_errors = {"left": "", "head": ""}
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: None)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node.pub_route_signal = SimpleNamespace(publish=lambda message: (_ for _ in ()).throw(AssertionError()))
    node._show_preview = lambda *args, **kwargs: None
    node._publish_status = lambda *args, **kwargs: None

    node._tick()

    assert node.decision_published is False


def _pipeline_node(context):
    node = CartFullnessNode.__new__(CartFullnessNode)
    node.active_context = context
    node.enabled = True
    node.decision_published = False
    node.preview = False
    node.preview_after_decision = True
    node.disable_after_decision = False
    node.max_frame_age_sec = 1.0
    node.max_pair_delta_sec = 0.1
    node.context_started_monotonic = 10.0
    node.frame_errors = {"left": "", "head": ""}
    node.full_signal = "continue_to_next_waypoint_for_grasp"
    node.not_full_signal = "skip_next_waypoint"
    node.full_route_step_delta = 1
    node.not_full_route_step_delta = 2
    node._publish_status = lambda *args, **kwargs: None
    return node


def test_head_uses_latest_snapshot_without_timestamp_matching():
    node = _pipeline_node(verify_context())
    image = np.ones((2, 2, 3), dtype=np.uint8)
    wrist_first = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    wrist_if_read_again = FrameSnapshot(image, stamp_sec=99.0, received_sec=10.0)
    head = FrameSnapshot(image, stamp_sec=99.0, received_sec=10.0)
    reads = []

    def get(camera, *, now_sec, max_age_sec):
        reads.append(camera)
        if camera == "left":
            return wrist_first if reads.count("left") == 1 else wrist_if_read_again
        return head

    node.frames = SimpleNamespace(get=get)
    node.detector = SimpleNamespace(detect=lambda image, *, track: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation("full"))
    node.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation("full"))

    wrist_view = node._evaluate_view("left", node.wrist_evaluator)
    head_view = node._evaluate_synchronized_head(node.active_context, wrist_view)

    assert reads == ["left", "head"]
    assert wrist_view.snapshot is wrist_first
    assert head_view.evaluation.decision == "full"


def test_verify_timeout_unknown_views_publishes_once():
    node = _pipeline_node(replace(verify_context(), timeout_sec=1.0))
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: None)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    messages = []
    node.pub_route_signal = SimpleNamespace(
        publish=lambda message: messages.append(json.loads(message.data))
    )
    node._show_preview = lambda *args, **kwargs: None
    node.context_started_monotonic = 10.0

    import mos_cart_fullness.cart_fullness_node as module
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: 20.0
    try:
        for _ in range(10):
            node._tick()
    finally:
        module.time.monotonic = original_monotonic

    assert len(messages) == 1
    assert messages[0]["decision"] == "not_full"


def test_scan_wrist_reason_identifies_frame_local_evidence():
    for decision, reason in (
        ("full", "frame_cart_full"),
        ("not_full", "frame_cart_not_full"),
    ):
        node = _pipeline_node(
            replace(verify_context(), mode="scan_wrist", timeout_sec=0.0)
        )
        messages = []
        node.pub_route_signal = SimpleNamespace(
            publish=lambda message: messages.append(json.loads(message.data))
        )
        node._finish_if_decided(
            node.active_context,
            evaluation(decision),
            None,
            decision,
        )
        assert messages[0]["reason"] == reason


def test_missing_view_does_not_block_available_view():
    node = _pipeline_node(replace(verify_context(), mode="scan_dual"))
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: None)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node.pub_route_signal = SimpleNamespace(
        publish=lambda message: (_ for _ in ()).throw(AssertionError())
    )
    node._show_preview = lambda *args, **kwargs: None

    node._tick()

    assert node.decision_published is False
    assert node._evaluate_view("left", node.wrist_evaluator).evaluation.decision == "unknown"

    wrist = FrameSnapshot(np.zeros((2, 2, 3), dtype=np.uint8), 10.0, 10.0)
    head = FrameSnapshot(np.zeros((2, 2, 3), dtype=np.uint8), 11.0, 10.0)
    node.frames = SimpleNamespace(
        get=lambda camera, **kwargs: wrist if camera == "left" else head
    )
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation("full"))
    node.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    wrist_view = node._evaluate_view("left", node.wrist_evaluator)
    head_view = node._evaluate_synchronized_head(node.active_context, wrist_view)
    assert head_view.evaluation.decision == "unknown"
    assert head_view.evaluation.reason == "test_reason"


def test_preview_uses_the_evaluated_snapshot_before_and_after_decision():
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    image = np.full((2, 2, 3), 7, dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    reads = []
    node.frames = SimpleNamespace(
        get=lambda camera, **kwargs: reads.append(camera) or snapshot
    )
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node._show_preview = lambda frame, **kwargs: reads.append(("preview", frame))

    node.preview = True
    node._tick()
    assert reads == ["left", ("preview", image)]

    reads.clear()
    node.decision_published = True
    node._tick()
    assert reads == ["left", ("preview", image)]


def _status_capture(node):
    statuses = []
    node.publish_status = True
    node.pub_status = SimpleNamespace(
        publish=lambda message: statuses.append(json.loads(message.data))
    )
    node._publish_status = CartFullnessNode._publish_status.__get__(node)
    return statuses


def test_missing_wrist_frame_does_not_publish_intermediate_status():
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    statuses = _status_capture(node)
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: None)
    node.frame_errors = {"left": ""}
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node._show_preview = lambda *args, **kwargs: None

    for _ in range(10):
        node._tick()

    assert statuses == []


def test_scan_no_cart_and_empty_cart_publish_status_without_route_signal():
    for decision in ("no_cart", "empty_cart"):
        node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
        statuses = _status_capture(node)
        image = np.ones((2, 2, 3), dtype=np.uint8)
        snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
        node.frames = SimpleNamespace(get=lambda *args, **kwargs: snapshot)
        node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
        node.wrist_evaluator = SimpleNamespace(
            evaluate=lambda detections, value=decision: evaluation(value)
        )
        routes = []
        node.pub_route_signal = SimpleNamespace(publish=lambda message: routes.append(message))
        node._show_preview = lambda *args, **kwargs: None

        for _ in range(9):
            node._tick()
        assert statuses == []

        node._tick()

        assert len(statuses) == 1
        assert statuses[0]["state"] == "checking"
        assert statuses[0]["decision"] == decision
        assert statuses[0]["wrist_decision"] == decision
        assert "signal" not in statuses[0]
        assert routes == []
        assert node.decision_published is False


def test_route_signal_keeps_existing_timing_but_status_waits_for_ten_frames():
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    statuses = _status_capture(node)
    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: snapshot)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(
        evaluate=lambda detections: evaluation("full", used_item_count=1)
    )
    routes = []
    node.pub_route_signal = SimpleNamespace(
        publish=lambda message: routes.append(json.loads(message.data))
    )
    node._show_preview = lambda *args, **kwargs: None

    for _ in range(3):
        node._tick()
    assert len(routes) == 1
    assert statuses == []

    for _ in range(7):
        node._tick()
    assert len(routes) == 1
    assert len(statuses) == 1
    assert statuses[0]["state"] == "route_signal_published"


@pytest.mark.parametrize(
    "decisions, expected_decision",
    [
        (["empty_cart"] * 10, "empty_cart"),
        (["no_cart"] * 10, "no_cart"),
        (["empty_cart"] * 5 + ["no_cart"] * 5, "empty_cart"),
        (["no_cart"] * 6 + ["empty_cart"] * 4, "no_cart"),
    ],
)
def test_scan_special_state_publishes_once_after_ten_frames(
    decisions, expected_decision
):
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    statuses = _status_capture(node)
    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: snapshot)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    empty_frame_count = decisions.count("empty_cart")
    no_cart_frame_count = decisions.count("no_cart")
    decisions = iter(decisions + [expected_decision])
    node.wrist_evaluator = SimpleNamespace(
        evaluate=lambda detections: evaluation(next(decisions))
    )
    node.pub_route_signal = SimpleNamespace(publish=lambda message: None)
    node._show_preview = lambda *args, **kwargs: None

    for frame_number in range(1, 12):
        node._tick()
        expected_status_count = 1 if frame_number >= 10 else 0
        assert len(statuses) == expected_status_count

    assert statuses[0]["decision"] == expected_decision
    assert statuses[0]["state"] == "checking"
    assert statuses[0]["sampled_frames"] == 10
    assert statuses[0]["empty_cart_frames"] == empty_frame_count
    assert statuses[0]["no_cart_frames"] == no_cart_frame_count


def test_scan_special_state_window_counts_across_decision_changes():
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    statuses = _status_capture(node)
    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    decisions = iter(["empty_cart"] * 5 + ["no_cart"] * 5)
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: snapshot)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(
        evaluate=lambda detections: evaluation(next(decisions))
    )
    node.pub_route_signal = SimpleNamespace(publish=lambda message: None)
    node._show_preview = lambda *args, **kwargs: None

    for _ in range(9):
        node._tick()
    assert statuses == []

    node._tick()
    assert len(statuses) == 1
    assert statuses[0]["decision"] == "empty_cart"


def test_special_state_confirmation_resets_for_a_new_check():
    node = CartFullnessNode.__new__(CartFullnessNode)
    node.wrist_evaluator = SimpleNamespace(reset=lambda: None)
    node.head_evaluator = SimpleNamespace(reset=lambda: None)

    for _ in range(9):
        assert node._should_publish_special_status("empty_cart") is False

    node._reset_active_state()

    for _ in range(9):
        assert node._should_publish_special_status("empty_cart") is False
    assert node._should_publish_special_status("empty_cart") is True


def test_detection_window_does_not_publish_intermediate_status():
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    statuses = _status_capture(node)
    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    node.frames = SimpleNamespace(get=lambda *args, **kwargs: snapshot)
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    node.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    node._show_preview = lambda *args, **kwargs: None

    for _ in range(10):
        node._tick()

    assert statuses == []

    dual = _pipeline_node(replace(verify_context(), mode="scan_dual"))
    dual_statuses = _status_capture(dual)
    dual.frames = SimpleNamespace(
        get=lambda camera, **kwargs: snapshot if camera == "left" else None
    )
    dual.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    dual.wrist_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    dual.head_evaluator = SimpleNamespace(evaluate=lambda detections: evaluation())
    dual._show_preview = lambda *args, **kwargs: None

    for _ in range(10):
        dual._tick()

    assert dual_statuses == []


def test_dual_cycle_evaluates_available_view_without_timestamp_matching():
    node = _pipeline_node(replace(verify_context(), mode="scan_dual"))
    image = np.ones((2, 2, 3), dtype=np.uint8)
    snapshot = FrameSnapshot(image, stamp_sec=20.0, received_sec=10.0)
    node.frames = SimpleNamespace(
        get=lambda camera, **kwargs: snapshot if camera == "left" else None
    )
    node.detector = SimpleNamespace(detect=lambda *args, **kwargs: [])
    calls = []
    node.wrist_evaluator = SimpleNamespace(
        evaluate=lambda detections: calls.append("wrist") or evaluation("full")
    )
    node.head_evaluator = SimpleNamespace(
        evaluate=lambda detections: calls.append("head") or evaluation("full")
    )
    messages = []
    node.pub_route_signal = SimpleNamespace(
        publish=lambda message: messages.append(json.loads(message.data))
    )
    node._show_preview = lambda *args, **kwargs: None

    node._tick()

    assert calls == ["wrist"]
    assert node.decision_published is True
    assert messages[0]["decision"] == "full"


def test_inference_overrun_is_reported_in_status():
    module = importlib.import_module("mos_cart_fullness.cart_fullness_node")
    node = _pipeline_node(replace(verify_context(), mode="scan_wrist"))
    node.camera_poll_seconds = 0.1
    node._tick_impl = lambda: None
    statuses = []
    node._publish_status = lambda state, **kwargs: statuses.append((state, kwargs))
    values = iter([1.0, 1.25])
    original_monotonic = module.time.monotonic
    module.time.monotonic = lambda: next(values)
    try:
        node._tick()
    finally:
        module.time.monotonic = original_monotonic

    assert statuses == []
