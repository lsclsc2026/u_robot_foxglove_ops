from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mos_cart_fullness.cart_fullness import judge_cart_fullness


def test_top_shelf_item_is_associated_without_center_in_load_roi():
    result = judge_cart_fullness(
        [100, 100, 500, 500],
        [
            {
                "class_id": 10,
                "score": 0.9,
                # The center is above the load ROI, but the bottom overlaps the
                # cart association region and the item reaches the cart top.
                "bbox_xyxy": [220, 40, 320, 140],
            }
        ],
        full_threshold=2.0,
        count_threshold=5,
        count_operator=">",
    )

    assert result["used_item_count"] == 1
    assert result["used_items"][0]["class_id"] == 10
    assert result["is_full"] is False


def test_six_allowed_items_satisfy_greater_than_five_count_rule():
    items = [
        {
            "class_id": 9 if index % 2 == 0 else 10,
            "score": 0.9,
            "bbox_xyxy": [150 + index * 45, 330, 180 + index * 45, 390],
        }
        for index in range(6)
    ]

    result = judge_cart_fullness(
        [100, 100, 500, 500],
        items,
        full_threshold=2.0,
        count_threshold=5,
        count_operator=">",
    )

    assert result["used_item_count"] == 6
    assert result["is_full"] is True
    assert result["reason"] == "item_count>5"
