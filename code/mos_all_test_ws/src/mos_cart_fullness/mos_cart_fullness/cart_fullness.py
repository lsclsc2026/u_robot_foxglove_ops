"""
治疗车满载判断工具。

这个模块用于根据治疗车框和车内物品框计算占据比例，并判断治疗车
是否达到满载条件。模块刻意保持无外部依赖，方便直接复制到机器人
集成代码或 ROS 包中使用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence


BBox = Sequence[float]
Detection = Mapping[str, Any]


DEFAULT_ITEM_CLASS_IDS = {9, 10}
DEFAULT_CART_CLASS_IDS = {0, 1, 2, 3, 4, 5}
DEFAULT_CART_CLASS_TO_HANDLE_CLASS_ID = {
    0: 6,
    1: 7,
    2: 8,
    3: 6,
    4: 7,
    5: 8,
}

# Empty and loaded detections of the same cart use the same load ROI.
# Classes 2/5 fall back to the generic ratios until their geometry is calibrated.
DEFAULT_CART_CLASS_TO_LOAD_ROI_PARAMS = {
    0: {"x_margin_ratio": 0.06, "y_start_ratio": 0.03, "y_end_ratio": 0.82},
    1: {"x_margin_ratio": 0.06, "y_start_ratio": 0.08, "y_end_ratio": 0.80},
    3: {"x_margin_ratio": 0.06, "y_start_ratio": 0.03, "y_end_ratio": 0.82},
    4: {"x_margin_ratio": 0.06, "y_start_ratio": 0.08, "y_end_ratio": 0.80},
}

DEFAULT_ASSOCIATION_TOP_EXTENSION_RATIO = 0.30


def _resolve_cart_load_roi_params(
    cart_class_id: int | None,
    cart_class_to_load_roi_params: Mapping[int, Mapping[str, float]],
    *,
    x_margin_ratio: float,
    y_start_ratio: float,
    y_end_ratio: float,
) -> tuple[float, float, float]:
    params = cart_class_to_load_roi_params.get(cart_class_id, {})
    return (
        float(params.get("x_margin_ratio", x_margin_ratio)),
        float(params.get("y_start_ratio", y_start_ratio)),
        float(params.get("y_end_ratio", y_end_ratio)),
    )


def _normalize_bbox_xyxy(bbox_xyxy: BBox) -> list[float]:
    """Return [x1, y1, x2, y2] with sorted coordinates."""
    if len(bbox_xyxy) != 4:
        raise ValueError(f"bbox_xyxy must have 4 values, got {len(bbox_xyxy)}")

    x1, y1, x2, y2 = (float(v) for v in bbox_xyxy)
    return [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]


def _bbox_area(bbox_xyxy: BBox) -> float:
    x1, y1, x2, y2 = _normalize_bbox_xyxy(bbox_xyxy)
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def _intersection_area(a_xyxy: BBox, b_xyxy: BBox) -> float:
    ax1, ay1, ax2, ay2 = _normalize_bbox_xyxy(a_xyxy)
    bx1, by1, bx2, by2 = _normalize_bbox_xyxy(b_xyxy)

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    return max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)


def compute_cart_load_roi(
    cart_bbox_xyxy: BBox,
    *,
    x_margin_ratio: float = 0.15,
    y_start_ratio: float = 0.55,
    y_end_ratio: float = 0.88,
) -> list[float]:
    """
    Estimate the cart load area from the whole-cart bbox.

    The default values target a two-layer metal treatment cart where items are
    usually placed on the lower tray. Tune these values after drawing
    load_roi_bbox_xyxy on real frames.
    """
    cart_x1, cart_y1, cart_x2, cart_y2 = _normalize_bbox_xyxy(cart_bbox_xyxy)
    cart_w = cart_x2 - cart_x1
    cart_h = cart_y2 - cart_y1

    if cart_w <= 0.0 or cart_h <= 0.0:
        return [cart_x1, cart_y1, cart_x1, cart_y1]

    x_margin_ratio = min(max(float(x_margin_ratio), 0.0), 0.49)
    y_start_ratio = min(max(float(y_start_ratio), 0.0), 1.0)
    y_end_ratio = min(max(float(y_end_ratio), 0.0), 1.0)
    if y_end_ratio < y_start_ratio:
        y_start_ratio, y_end_ratio = y_end_ratio, y_start_ratio

    return [
        cart_x1 + x_margin_ratio * cart_w,
        cart_y1 + y_start_ratio * cart_h,
        cart_x2 - x_margin_ratio * cart_w,
        cart_y1 + y_end_ratio * cart_h,
    ]


def compute_cart_item_association_roi(
    cart_bbox_xyxy: BBox,
    *,
    x_margin_ratio: float = 0.15,
    y_end_ratio: float = 0.88,
    top_extension_ratio: float = DEFAULT_ASSOCIATION_TOP_EXTENSION_RATIO,
) -> list[float]:
    """Return the region in which detected items can belong to a cart.

    This region extends above the cart bbox so packages on the top shelf can
    still be associated with the cart when their detection box protrudes up.
    """
    cart_x1, cart_y1, cart_x2, cart_y2 = _normalize_bbox_xyxy(cart_bbox_xyxy)
    cart_w = cart_x2 - cart_x1
    cart_h = cart_y2 - cart_y1
    if cart_w <= 0.0 or cart_h <= 0.0:
        return [cart_x1, cart_y1, cart_x1, cart_y1]

    x_margin_ratio = min(max(float(x_margin_ratio), 0.0), 0.49)
    y_end_ratio = min(max(float(y_end_ratio), 0.0), 1.0)
    top_extension_ratio = max(float(top_extension_ratio), 0.0)
    return [
        cart_x1 + x_margin_ratio * cart_w,
        cart_y1 - top_extension_ratio * cart_h,
        cart_x2 - x_margin_ratio * cart_w,
        cart_y1 + y_end_ratio * cart_h,
    ]


def _get_detection_bbox(det: Detection) -> list[float] | None:
    bbox = det.get("bbox_xyxy")
    if bbox is None:
        bbox = det.get("bbox")
    if bbox is None:
        return None
    return _normalize_bbox_xyxy(bbox)


def _get_detection_class_id(det: Detection) -> int | None:
    class_id = det.get("class_id")
    if class_id is None:
        class_id = det.get("cls")
    if class_id is None:
        return None
    return int(class_id)


def _score_is_valid(det: Detection, min_score: float | None) -> bool:
    if min_score is None:
        return True
    score = det.get("score")
    if score is None:
        score = det.get("confidence")
    if score is None:
        score = det.get("conf")
    return score is None or float(score) >= min_score


def judge_cart_fullness(
    cart_bbox_xyxy: BBox,
    item_detections: Iterable[Detection],
    *,
    full_threshold: float = 0.6,
    count_threshold: int = 5,
    item_class_ids: set[int] | frozenset[int] | None = None,
    count_operator: str = ">",
    min_item_score: float | None = None,
    x_margin_ratio: float = 0.15,
    y_start_ratio: float = 0.55,
    y_end_ratio: float = 0.88,
    association_top_extension_ratio: float = DEFAULT_ASSOCIATION_TOP_EXTENSION_RATIO,
    min_item_association_overlap: float = 0.1,
    min_item_roi_overlap: float | None = None,
) -> dict[str, Any]:
    """
    Judge whether one treatment cart is full enough to move.

    Args:
        cart_bbox_xyxy: Current cart bbox [x1, y1, x2, y2].
        item_detections: Detected load items. Each dict can contain:
            {"class_id": 9, "score": 0.87, "bbox_xyxy": [x1, y1, x2, y2]}.
        full_threshold: Full if occupancy_ratio >= this value.
        count_threshold: Full if used item count passes this threshold.
        item_class_ids: Allowed item classes. Defaults to {9, 10}.
        count_operator: ">" means count must be greater than count_threshold;
            ">=" means count_threshold itself is enough.
        min_item_score: Optional detection score filter.
        x_margin_ratio/y_start_ratio/y_end_ratio: Ratios used to estimate the
            load ROI from the whole-cart bbox.
        association_top_extension_ratio: How far the association ROI extends
            above the cart bbox, relative to cart height.
        min_item_association_overlap: Minimum item-area overlap with the
            association ROI. Item centers do not need to be inside the ROI.
        min_item_roi_overlap: Deprecated compatibility alias for
            min_item_association_overlap. When supplied, it takes precedence.

    Returns:
        A debug-friendly dict containing is_full, occupancy_ratio, used item
        count, thresholds, ROI bbox and reason.
    """
    allowed_class_ids = DEFAULT_ITEM_CLASS_IDS if item_class_ids is None else set(item_class_ids)
    cart_bbox = _normalize_bbox_xyxy(cart_bbox_xyxy)
    load_roi_bbox = compute_cart_load_roi(
        cart_bbox,
        x_margin_ratio=x_margin_ratio,
        y_start_ratio=y_start_ratio,
        y_end_ratio=y_end_ratio,
    )
    association_roi_bbox = compute_cart_item_association_roi(
        cart_bbox,
        x_margin_ratio=x_margin_ratio,
        y_end_ratio=y_end_ratio,
        top_extension_ratio=association_top_extension_ratio,
    )
    roi_area = _bbox_area(load_roi_bbox)
    effective_min_association_overlap = (
        float(min_item_association_overlap)
        if min_item_roi_overlap is None
        else float(min_item_roi_overlap)
    )

    used_items: list[dict[str, Any]] = []
    total_intersection_area = 0.0

    for det_index, det in enumerate(item_detections):
        class_id = _get_detection_class_id(det)
        if class_id not in allowed_class_ids:
            continue
        if not _score_is_valid(det, min_item_score):
            continue

        bbox = _get_detection_bbox(det)
        if bbox is None:
            continue

        inter_area = _intersection_area(bbox, load_roi_bbox)
        item_area = _bbox_area(bbox)
        if item_area <= 0.0:
            continue

        # Reject detections floating completely above the cart. The item does
        # not need to have its center inside the old LOAD_ROI anymore.
        if bbox[3] < cart_bbox[1]:
            continue

        association_intersection_area = _intersection_area(
            bbox, association_roi_bbox
        )
        item_association_overlap = association_intersection_area / item_area
        if item_association_overlap < effective_min_association_overlap:
            continue

        item_roi_overlap = inter_area / item_area

        total_intersection_area += inter_area
        used_items.append(
            {
                "index": det_index,
                "class_id": class_id,
                "score": det.get("score", det.get("confidence", det.get("conf"))),
                "bbox_xyxy": bbox,
                "intersection_area": inter_area,
                "item_roi_overlap": item_roi_overlap,
                "association_intersection_area": association_intersection_area,
                "item_association_overlap": item_association_overlap,
            }
        )

    occupancy_ratio = total_intersection_area / roi_area if roi_area > 0.0 else 0.0
    occupancy_ratio = min(max(occupancy_ratio, 0.0), 1.0)

    used_item_count = len(used_items)
    is_full_by_area = occupancy_ratio >= full_threshold
    if count_operator == ">=":
        is_full_by_count = used_item_count >= count_threshold
        count_reason = f"item_count>={count_threshold}"
    elif count_operator == ">":
        is_full_by_count = used_item_count > count_threshold
        count_reason = f"item_count>{count_threshold}"
    else:
        raise ValueError('count_operator must be ">" or ">="')

    if is_full_by_area:
        reason = f"occupancy_ratio>={full_threshold}"
    elif is_full_by_count:
        reason = count_reason
    else:
        reason = "not_full"

    return {
        "is_full": is_full_by_area or is_full_by_count,
        "occupancy_ratio": occupancy_ratio,
        "used_item_count": used_item_count,
        "full_threshold": full_threshold,
        "count_threshold": count_threshold,
        "count_operator": count_operator,
        "load_roi_bbox_xyxy": load_roi_bbox,
        "association_roi_bbox_xyxy": association_roi_bbox,
        "association_top_extension_ratio": association_top_extension_ratio,
        "min_item_association_overlap": effective_min_association_overlap,
        "cart_bbox_xyxy": cart_bbox,
        "used_items": used_items,
        "reason": reason,
    }


def filter_detections_by_class(
    detections: Iterable[Detection],
    class_ids: Iterable[int],
    *,
    min_score: float | None = None,
) -> list[dict[str, Any]]:
    """
    Keep detections whose class is in class_ids.

    This accepts dict-like detections with class_id/cls and bbox_xyxy/bbox keys.
    The returned dicts are normalized to class_id, score and bbox_xyxy.
    """
    allowed_class_ids = set(class_ids)
    filtered: list[dict[str, Any]] = []

    for det in detections:
        class_id = _get_detection_class_id(det)
        if class_id not in allowed_class_ids:
            continue
        if not _score_is_valid(det, min_score):
            continue

        bbox = _get_detection_bbox(det)
        if bbox is None:
            continue

        filtered.append(
            {
                "class_id": class_id,
                "score": det.get("score", det.get("confidence", det.get("conf"))),
                "bbox_xyxy": bbox,
            }
        )

    return filtered


def judge_carts_fullness(
    cart_detections: Iterable[Detection],
    item_detections: Iterable[Detection],
    *,
    cart_class_ids: set[int] | frozenset[int] | None = None,
    cart_class_to_handle_class_id: Mapping[int, int] | None = None,
    cart_class_to_load_roi_params: Mapping[int, Mapping[str, float]] | None = None,
    full_threshold: float = 0.6,
    count_threshold: int = 5,
    item_class_ids: set[int] | frozenset[int] | None = None,
    count_operator: str = ">",
    min_item_score: float | None = None,
    min_cart_score: float | None = None,
    x_margin_ratio: float = 0.15,
    y_start_ratio: float = 0.55,
    y_end_ratio: float = 0.88,
    association_top_extension_ratio: float = DEFAULT_ASSOCIATION_TOP_EXTENSION_RATIO,
    min_item_association_overlap: float = 0.1,
    min_item_roi_overlap: float | None = None,
) -> list[dict[str, Any]]:
    """
    Judge fullness for multiple carts in one frame.

    Each result contains the single-cart result plus cart_class_id,
    cart_score and handle_class_id. The caller can select full carts and send
    handle_class_id to the downstream grasping logic.
    """
    allowed_cart_class_ids = (
        DEFAULT_CART_CLASS_IDS if cart_class_ids is None else set(cart_class_ids)
    )
    handle_mapping = (
        DEFAULT_CART_CLASS_TO_HANDLE_CLASS_ID
        if cart_class_to_handle_class_id is None
        else cart_class_to_handle_class_id
    )
    load_roi_mapping = (
        DEFAULT_CART_CLASS_TO_LOAD_ROI_PARAMS
        if cart_class_to_load_roi_params is None
        else cart_class_to_load_roi_params
    )
    items = list(item_detections)
    results: list[dict[str, Any]] = []

    for cart_index, cart in enumerate(cart_detections):
        cart_class_id = _get_detection_class_id(cart)
        if cart_class_id not in allowed_cart_class_ids:
            continue
        if not _score_is_valid(cart, min_cart_score):
            continue

        cart_bbox = _get_detection_bbox(cart)
        if cart_bbox is None:
            continue

        cart_x_margin_ratio, cart_y_start_ratio, cart_y_end_ratio = (
            _resolve_cart_load_roi_params(
                cart_class_id,
                load_roi_mapping,
                x_margin_ratio=x_margin_ratio,
                y_start_ratio=y_start_ratio,
                y_end_ratio=y_end_ratio,
            )
        )

        result = judge_cart_fullness(
            cart_bbox_xyxy=cart_bbox,
            item_detections=items,
            full_threshold=full_threshold,
            count_threshold=count_threshold,
            item_class_ids=item_class_ids,
            count_operator=count_operator,
            min_item_score=min_item_score,
            x_margin_ratio=cart_x_margin_ratio,
            y_start_ratio=cart_y_start_ratio,
            y_end_ratio=cart_y_end_ratio,
            association_top_extension_ratio=association_top_extension_ratio,
            min_item_association_overlap=min_item_association_overlap,
            min_item_roi_overlap=min_item_roi_overlap,
        )
        result.update(
            {
                "cart_index": cart_index,
                "cart_class_id": cart_class_id,
                "cart_score": cart.get("score", cart.get("confidence", cart.get("conf"))),
                "handle_class_id": handle_mapping.get(cart_class_id),
                "load_roi_params": {
                    "x_margin_ratio": cart_x_margin_ratio,
                    "y_start_ratio": cart_y_start_ratio,
                    "y_end_ratio": cart_y_end_ratio,
                },
            }
        )
        results.append(result)

    return results


@dataclass
class CartFullnessJudge:
    """
    OOP interface for cart fullness judgment.

    Create one judge with stable thresholds and reuse it for every frame.
    """

    full_threshold: float = 0.6
    count_threshold: int = 5
    count_operator: str = ">"
    item_class_ids: set[int] | frozenset[int] | None = None
    cart_class_ids: set[int] | frozenset[int] | None = None
    cart_class_to_handle_class_id: Mapping[int, int] | None = None
    cart_class_to_load_roi_params: Mapping[int, Mapping[str, float]] | None = None
    min_item_score: float | None = None
    min_cart_score: float | None = None
    x_margin_ratio: float = 0.15
    y_start_ratio: float = 0.55
    y_end_ratio: float = 0.88
    association_top_extension_ratio: float = DEFAULT_ASSOCIATION_TOP_EXTENSION_RATIO
    min_item_association_overlap: float = 0.1
    min_item_roi_overlap: float | None = None

    def __post_init__(self) -> None:
        if self.count_operator not in {">", ">="}:
            raise ValueError('count_operator must be ">" or ">="')

        if self.item_class_ids is None:
            self.item_class_ids = set(DEFAULT_ITEM_CLASS_IDS)
        else:
            self.item_class_ids = set(self.item_class_ids)

        if self.cart_class_ids is None:
            self.cart_class_ids = set(DEFAULT_CART_CLASS_IDS)
        else:
            self.cart_class_ids = set(self.cart_class_ids)

        if self.cart_class_to_handle_class_id is None:
            self.cart_class_to_handle_class_id = dict(DEFAULT_CART_CLASS_TO_HANDLE_CLASS_ID)
        else:
            self.cart_class_to_handle_class_id = dict(self.cart_class_to_handle_class_id)

        if self.cart_class_to_load_roi_params is None:
            self.cart_class_to_load_roi_params = {
                class_id: dict(params)
                for class_id, params in DEFAULT_CART_CLASS_TO_LOAD_ROI_PARAMS.items()
            }
        else:
            self.cart_class_to_load_roi_params = {
                int(class_id): dict(params)
                for class_id, params in self.cart_class_to_load_roi_params.items()
            }

    def compute_load_roi(self, cart_bbox_xyxy: BBox) -> list[float]:
        """Estimate this judge's load ROI for one cart bbox."""
        return compute_cart_load_roi(
            cart_bbox_xyxy,
            x_margin_ratio=self.x_margin_ratio,
            y_start_ratio=self.y_start_ratio,
            y_end_ratio=self.y_end_ratio,
        )

    def filter_detections_by_class(
        self,
        detections: Iterable[Detection],
        class_ids: Iterable[int] | None = None,
        *,
        min_score: float | None = None,
    ) -> list[dict[str, Any]]:
        """Normalize and keep detections by class id."""
        if class_ids is None:
            class_ids = self.item_class_ids
        return filter_detections_by_class(
            detections=detections,
            class_ids=class_ids,
            min_score=min_score,
        )

    def judge_cart(
        self,
        cart_bbox_xyxy: BBox,
        item_detections: Iterable[Detection],
        *,
        cart_class_id: int | None = None,
    ) -> dict[str, Any]:
        """Judge one cart, optionally using its class-specific load ROI."""
        cart_x_margin_ratio, cart_y_start_ratio, cart_y_end_ratio = (
            _resolve_cart_load_roi_params(
                cart_class_id,
                self.cart_class_to_load_roi_params,
                x_margin_ratio=self.x_margin_ratio,
                y_start_ratio=self.y_start_ratio,
                y_end_ratio=self.y_end_ratio,
            )
        )
        result = judge_cart_fullness(
            cart_bbox_xyxy=cart_bbox_xyxy,
            item_detections=item_detections,
            full_threshold=self.full_threshold,
            count_threshold=self.count_threshold,
            item_class_ids=self.item_class_ids,
            count_operator=self.count_operator,
            min_item_score=self.min_item_score,
            x_margin_ratio=cart_x_margin_ratio,
            y_start_ratio=cart_y_start_ratio,
            y_end_ratio=cart_y_end_ratio,
            association_top_extension_ratio=self.association_top_extension_ratio,
            min_item_association_overlap=self.min_item_association_overlap,
            min_item_roi_overlap=self.min_item_roi_overlap,
        )
        result["cart_class_id"] = cart_class_id
        result["load_roi_params"] = {
            "x_margin_ratio": cart_x_margin_ratio,
            "y_start_ratio": cart_y_start_ratio,
            "y_end_ratio": cart_y_end_ratio,
        }
        return result

    def judge_carts(
        self,
        cart_detections: Iterable[Detection],
        item_detections: Iterable[Detection],
    ) -> list[dict[str, Any]]:
        """
        Judge fullness for multiple carts in one frame.

        Full cart results include handle_class_id for downstream grasping.
        """
        return judge_carts_fullness(
            cart_detections=cart_detections,
            item_detections=item_detections,
            cart_class_ids=self.cart_class_ids,
            cart_class_to_handle_class_id=self.cart_class_to_handle_class_id,
            cart_class_to_load_roi_params=self.cart_class_to_load_roi_params,
            full_threshold=self.full_threshold,
            count_threshold=self.count_threshold,
            item_class_ids=self.item_class_ids,
            count_operator=self.count_operator,
            min_item_score=self.min_item_score,
            min_cart_score=self.min_cart_score,
            x_margin_ratio=self.x_margin_ratio,
            y_start_ratio=self.y_start_ratio,
            y_end_ratio=self.y_end_ratio,
            association_top_extension_ratio=self.association_top_extension_ratio,
            min_item_association_overlap=self.min_item_association_overlap,
            min_item_roi_overlap=self.min_item_roi_overlap,
        )

    def is_cart_full(
        self,
        cart_bbox_xyxy: BBox,
        item_bboxes_xyxy: Sequence[BBox],
        item_class_ids: Sequence[int],
    ) -> bool:
        """Minimal bool method for callers that do not need debug details."""
        item_detections = [
            {"class_id": class_id, "bbox_xyxy": bbox}
            for bbox, class_id in zip(item_bboxes_xyxy, item_class_ids)
        ]
        result = self.judge_cart(
            cart_bbox_xyxy=cart_bbox_xyxy,
            item_detections=item_detections,
        )
        return bool(result["is_full"])


def is_cart_full(
    cart_bbox_xyxy: BBox,
    item_bboxes_xyxy: Sequence[BBox],
    item_class_ids: Sequence[int],
    *,
    full_threshold: float = 0.6,
    count_threshold: int = 5,
) -> bool:
    """
    Minimal bool interface for callers that do not need debug details.
    """
    item_detections = [
        {"class_id": class_id, "bbox_xyxy": bbox}
        for bbox, class_id in zip(item_bboxes_xyxy, item_class_ids)
    ]
    result = judge_cart_fullness(
        cart_bbox_xyxy=cart_bbox_xyxy,
        item_detections=item_detections,
        full_threshold=full_threshold,
        count_threshold=count_threshold,
    )
    return bool(result["is_full"])


if __name__ == "__main__":
    judge = CartFullnessJudge(full_threshold=0.6, count_threshold=1)
    demo_result = judge.judge_cart(
        cart_bbox_xyxy=[405, 180, 905, 625],
        item_detections=[
            {"class_id": 9, "score": 0.91, "bbox_xyxy": [590, 455, 735, 545]},
        ],
    )
    print(demo_result)
