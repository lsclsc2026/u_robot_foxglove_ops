"""Timestamped frame storage and independent per-view fullness state."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from .stability import FullnessDecisionWindow


@dataclass(frozen=True)
class FrameSnapshot:
    frame: Any
    stamp_sec: float
    received_sec: float


class LatestFrameStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._frames: dict[str, FrameSnapshot] = {}

    def update(self, camera: str, frame, *, stamp_sec: float, received_sec: float) -> None:
        if frame is None or frame.size == 0:
            return
        snapshot = FrameSnapshot(frame.copy(), float(stamp_sec), float(received_sec))
        with self._lock:
            self._frames[str(camera)] = snapshot

    def peek(self, camera: str) -> FrameSnapshot | None:
        with self._lock:
            snapshot = self._frames.get(str(camera))
            if snapshot is None:
                return None
            return FrameSnapshot(
                snapshot.frame.copy(), snapshot.stamp_sec, snapshot.received_sec
            )

    def get(self, camera: str, *, now_sec: float, max_age_sec: float) -> FrameSnapshot | None:
        snapshot = self.peek(camera)
        if snapshot is None:
            return None
        if float(now_sec) - snapshot.received_sec > max(0.0, float(max_age_sec)):
            return None
        return snapshot


def frames_are_synchronized(
    first: FrameSnapshot | None,
    second: FrameSnapshot | None,
    max_delta_sec: float,
) -> bool:
    if first is None or second is None:
        return False
    return abs(first.stamp_sec - second.stamp_sec) <= max(0.0, float(max_delta_sec))


@dataclass
class ViewEvaluation:
    decision: str
    reason: str
    detections: list[dict[str, Any]]
    cart_candidates: list[dict[str, Any]]
    target: dict[str, Any] | None
    stable_frames: int
    fullness_result: dict[str, Any] | None
    summary: dict[str, int]


class FrameFullnessEvaluator:
    """Evaluate every detected cart against the items in the same frame.

    This evaluator deliberately has no target tracker and no IOU state.  Cart
    and item boxes can move between frames; each call uses the boxes produced
    by that call.  A frame is full when at least one current cart is full.
    ``FullnessDecisionWindow`` then requires enough full frame results inside
    its configured window before the caller receives a stable ``full`` decision.
    """

    def __init__(
        self,
        judge,
        window,
        *,
        empty_cart_class_ids=None,
        loaded_cart_class_ids=None,
    ):
        self.judge = judge
        self.window = window
        self.empty_cart_class_ids = (
            None
            if empty_cart_class_ids is None
            else {int(value) for value in empty_cart_class_ids}
        )
        self.loaded_cart_class_ids = (
            None
            if loaded_cart_class_ids is None
            else {int(value) for value in loaded_cart_class_ids}
        )

    @classmethod
    def create_default(
        cls,
        judge,
        *,
        decision_window=5,
        min_full_frames=3,
        min_not_full_frames=3,
        empty_cart_class_ids=None,
        loaded_cart_class_ids=None,
    ):
        return cls(
            judge=judge,
            window=FullnessDecisionWindow(
                window=decision_window,
                min_full=min_full_frames,
                min_not_full=min_not_full_frames,
            ),
            empty_cart_class_ids=empty_cart_class_ids,
            loaded_cart_class_ids=loaded_cart_class_ids,
        )

    def reset(self) -> None:
        self.window.reset()

    @staticmethod
    def _result_rank(result: dict[str, Any]) -> tuple[float, int, float, float]:
        """Prefer a full result, then the strongest current-frame evidence."""
        return (
            1.0 if result.get("is_full", False) else 0.0,
            int(result.get("used_item_count", 0)),
            float(result.get("occupancy_ratio", 0.0)),
            float(result.get("cart_score") or 0.0),
        )

    def evaluate(self, detections: list[dict[str, Any]]) -> ViewEvaluation:
        carts = self.judge.filter_detections_by_class(
            detections,
            self.judge.cart_class_ids,
            min_score=self.judge.min_cart_score,
        )
        classify_cart_type = (
            self.empty_cart_class_ids is not None
            and self.loaded_cart_class_ids is not None
        )
        if classify_cart_type:
            loaded_carts = [
                cart
                for cart in carts
                if int(cart.get("class_id", -1)) in self.loaded_cart_class_ids
            ]
            empty_carts = [
                cart
                for cart in carts
                if int(cart.get("class_id", -1)) in self.empty_cart_class_ids
            ]
            if loaded_carts:
                carts = loaded_carts
            elif empty_carts:
                self.window.reset()
                target = max(
                    empty_carts,
                    key=lambda cart: float(cart.get("score") or 0.0),
                )
                return ViewEvaluation(
                    decision="empty_cart",
                    reason="empty_cart_detected",
                    detections=detections,
                    cart_candidates=empty_carts,
                    target=target,
                    stable_frames=0,
                    fullness_result=None,
                    summary=self.window.summary(),
                )
            else:
                carts = []
        if not carts:
            self.window.reset()
            return ViewEvaluation(
                decision="no_cart" if classify_cart_type else "unknown",
                reason="no_cart_candidates",
                detections=detections,
                cart_candidates=[],
                target=None,
                stable_frames=0,
                fullness_result=None,
                summary=self.window.summary(),
            )

        items = self.judge.filter_detections_by_class(
            detections,
            self.judge.item_class_ids,
            min_score=self.judge.min_item_score,
        )
        results: list[dict[str, Any]] = []
        for cart_index, cart in enumerate(carts):
            result = dict(
                self.judge.judge_cart(
                    cart["bbox_xyxy"],
                    items,
                    cart_class_id=cart.get("class_id"),
                )
            )
            result.setdefault("cart_bbox_xyxy", list(cart["bbox_xyxy"]))
            result["cart_index"] = cart_index
            result["cart_class_id"] = cart.get("class_id")
            result["cart_score"] = cart.get("score")
            results.append(result)

        selected = max(results, key=self._result_rank)
        selected_cart = carts[int(selected["cart_index"])]
        frame_is_full = any(bool(result.get("is_full", False)) for result in results)
        decision = self.window.add(frame_is_full)
        summary = self.window.summary()
        reason = "frame_cart_full" if frame_is_full else "frame_cart_not_full"
        return ViewEvaluation(
            decision=decision,
            reason=reason,
            detections=detections,
            cart_candidates=carts,
            target=selected_cart,
            stable_frames=int(summary.get("frames", 0)),
            fullness_result=selected,
            summary=summary,
        )


class ItemCountFullnessEvaluator:
    """Loose wrist-only evaluator used during grasp-point verification.

    At the grasp point the wrist camera may see the load item without seeing
    the complete cart. This evaluator therefore counts allowed item classes
    directly in the current frame and deliberately does not require a cart
    detection or an item-to-cart ROI association.
    """

    def __init__(self, judge, window, *, count_threshold: int = 1):
        self.judge = judge
        self.window = window
        self.count_threshold = max(1, int(count_threshold))

    @classmethod
    def create_default(
        cls,
        judge,
        *,
        decision_window=5,
        min_full_frames=3,
        min_not_full_frames=3,
        count_threshold=1,
    ):
        return cls(
            judge=judge,
            window=FullnessDecisionWindow(
                window=decision_window,
                min_full=min_full_frames,
                min_not_full=min_not_full_frames,
            ),
            count_threshold=count_threshold,
        )

    def reset(self) -> None:
        self.window.reset()

    def evaluate(self, detections: list[dict[str, Any]]) -> ViewEvaluation:
        items = self.judge.filter_detections_by_class(
            detections,
            self.judge.item_class_ids,
            min_score=self.judge.min_item_score,
        )
        frame_is_full = len(items) >= self.count_threshold
        decision = self.window.add(frame_is_full)
        summary = self.window.summary()
        result = {
            "is_full": frame_is_full,
            "occupancy_ratio": 0.0,
            "used_item_count": len(items),
            "count_threshold": self.count_threshold,
            "count_operator": ">=",
            "used_items": items,
            "reason": (
                f"item_count>={self.count_threshold}"
                if frame_is_full
                else "not_full"
            ),
        }
        return ViewEvaluation(
            decision=decision,
            reason=(
                "frame_item_count_full"
                if frame_is_full
                else "frame_item_count_not_full"
            ),
            detections=detections,
            cart_candidates=[],
            target=None,
            stable_frames=int(summary.get("frames", 0)),
            fullness_result=result,
            summary=summary,
        )
