from pathlib import Path
import sys

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mos_cart_fullness.multiview_state import (
    FrameFullnessEvaluator,
    LatestFrameStore,
    frames_are_synchronized,
)


def test_frame_store_rejects_stale_snapshot():
    store = LatestFrameStore()
    store.update("head", np.ones((2, 2, 3), dtype=np.uint8), stamp_sec=10.0, received_sec=20.0)
    assert store.get("head", now_sec=20.4, max_age_sec=0.5) is not None
    assert store.get("head", now_sec=20.6, max_age_sec=0.5) is None


def test_frame_pair_requires_close_ros_timestamps():
    store = LatestFrameStore()
    frame = np.ones((2, 2, 3), dtype=np.uint8)
    store.update("left", frame, stamp_sec=10.00, received_sec=20.0)
    store.update("head", frame, stamp_sec=10.08, received_sec=20.0)
    assert frames_are_synchronized(store.peek("left"), store.peek("head"), 0.10)
    assert not frames_are_synchronized(store.peek("left"), store.peek("head"), 0.05)


class _FrameJudge:
    cart_class_ids = {0}
    item_class_ids = {9}
    min_cart_score = 0.0
    min_item_score = 0.0

    def filter_detections_by_class(self, detections, class_ids, *, min_score=None):
        allowed = set(class_ids)
        return [
            {
                "class_id": int(det["class_id"]),
                "score": float(det.get("score", 1.0)),
                "bbox_xyxy": list(det["bbox_xyxy"]),
            }
            for det in detections
            if int(det["class_id"]) in allowed
        ]

    def judge_cart(self, cart_bbox_xyxy, item_detections, *, cart_class_id=None):
        # Model a current-frame result so the test does not depend on IOU.
        x1, y1, x2, y2 = cart_bbox_xyxy
        used_items = []
        for item in item_detections:
            ix1, iy1, ix2, iy2 = item["bbox_xyxy"]
            cx = (ix1 + ix2) / 2.0
            cy = (iy1 + iy2) / 2.0
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                used_items.append(item)
        return {
            "is_full": bool(used_items),
            "occupancy_ratio": 0.5 if used_items else 0.0,
            "used_item_count": len(used_items),
            "cart_bbox_xyxy": list(cart_bbox_xyxy),
            "load_roi_bbox_xyxy": list(cart_bbox_xyxy),
            "used_items": used_items,
            "reason": "item_present" if used_items else "not_full",
        }


def _moving_frame(cart_x, item_x):
    return [
        {"class_id": 0, "score": 0.9, "bbox_xyxy": [cart_x, 0, cart_x + 100, 100]},
        {"class_id": 9, "score": 0.9, "bbox_xyxy": [item_x, 20, item_x + 20, 60]},
    ]


def _cart_detection(class_id, x=0):
    return {
        "class_id": class_id,
        "score": 0.9,
        "bbox_xyxy": [x, 0, x + 100, 100],
    }


def _classified_evaluator(*, decision_window=5, min_full=3, min_not_full=3):
    judge = _FrameJudge()
    judge.cart_class_ids = {0, 1, 2, 3, 4, 5}
    return FrameFullnessEvaluator.create_default(
        judge,
        decision_window=decision_window,
        min_full_frames=min_full,
        min_not_full_frames=min_not_full,
        empty_cart_class_ids={0, 1, 2},
        loaded_cart_class_ids={3, 4, 5},
    )


def test_classified_evaluator_reports_no_cart_immediately():
    result = _classified_evaluator().evaluate([])

    assert result.decision == "no_cart"
    assert result.reason == "no_cart_candidates"
    assert result.summary["frames"] == 0


def test_classified_evaluator_reports_empty_cart_immediately():
    result = _classified_evaluator().evaluate([_cart_detection(1)])

    assert result.decision == "empty_cart"
    assert result.reason == "empty_cart_detected"
    assert result.target["class_id"] == 1
    assert result.summary["frames"] == 0


def test_classified_evaluator_only_votes_for_loaded_cart_classes():
    evaluator = _classified_evaluator()
    loaded_cart = [_cart_detection(3)]

    assert evaluator.evaluate(loaded_cart).decision == "checking"
    assert evaluator.evaluate(loaded_cart).decision == "checking"
    assert evaluator.evaluate(loaded_cart).decision == "not_full"


def test_loaded_cart_takes_priority_over_empty_cart_in_same_frame():
    evaluator = _classified_evaluator(
        decision_window=1,
        min_full=1,
        min_not_full=1,
    )
    detections = [
        _cart_detection(0, x=0),
        _cart_detection(3, x=200),
        {"class_id": 9, "score": 0.9, "bbox_xyxy": [10, 20, 30, 60]},
    ]

    result = evaluator.evaluate(detections)

    assert result.decision == "not_full"
    assert result.target["class_id"] == 3
    assert [cart["class_id"] for cart in result.cart_candidates] == [3]


def test_frame_evaluator_uses_current_boxes_without_iou_locking():
    evaluator = FrameFullnessEvaluator.create_default(
        _FrameJudge(), decision_window=5, min_full_frames=3, min_not_full_frames=3
    )

    first = evaluator.evaluate(_moving_frame(0, 10))
    second = evaluator.evaluate(_moving_frame(200, 210))
    third = evaluator.evaluate(_moving_frame(400, 410))

    assert first.decision == "checking"
    assert second.decision == "checking"
    assert third.decision == "full"
    assert third.target["bbox_xyxy"] == [400.0, 0.0, 500.0, 100.0]


def test_frame_evaluator_marks_frame_full_when_any_current_cart_is_full():
    evaluator = FrameFullnessEvaluator.create_default(
        _FrameJudge(), decision_window=5, min_full_frames=3, min_not_full_frames=3
    )
    detections = [
        {"class_id": 0, "score": 0.9, "bbox_xyxy": [0, 0, 100, 100]},
        {"class_id": 0, "score": 0.9, "bbox_xyxy": [200, 0, 300, 100]},
        {"class_id": 9, "score": 0.9, "bbox_xyxy": [220, 20, 240, 60]},
    ]

    for _ in range(2):
        result = evaluator.evaluate(detections)
        assert result.decision == "checking"
    result = evaluator.evaluate(detections)

    assert result.decision == "full"
    assert result.fullness_result["used_item_count"] == 1
    assert result.target["bbox_xyxy"] == [200.0, 0.0, 300.0, 100.0]


def test_frame_evaluator_publishes_after_any_three_full_frames_in_window():
    evaluator = FrameFullnessEvaluator.create_default(
        _FrameJudge(), decision_window=5, min_full_frames=3, min_not_full_frames=3
    )
    full = _moving_frame(0, 10)
    not_full = [{"class_id": 0, "score": 0.9, "bbox_xyxy": [0, 0, 100, 100]}]

    assert evaluator.evaluate(full).decision == "checking"
    assert evaluator.evaluate(not_full).decision == "checking"
    assert evaluator.evaluate(full).decision == "checking"
    assert evaluator.evaluate(full).decision == "full"


def test_frame_store_returns_private_frame_copies():
    store = LatestFrameStore()
    frame = np.full((2, 2, 3), 7, dtype=np.uint8)
    store.update("left", frame, stamp_sec=1.0, received_sec=2.0)

    frame[:, :, :] = 1
    snapshot = store.peek("left")
    assert snapshot is not None
    assert int(snapshot.frame[0, 0, 0]) == 7

    snapshot.frame[:, :, :] = 2
    assert int(store.peek("left").frame[0, 0, 0]) == 7


def test_frame_store_ignores_none_and_empty_frames():
    store = LatestFrameStore()
    store.update("left", None, stamp_sec=1.0, received_sec=2.0)
    store.update("head", np.empty((0, 2, 3), dtype=np.uint8), stamp_sec=1.0, received_sec=2.0)
    assert store.peek("left") is None
    assert store.peek("head") is None
