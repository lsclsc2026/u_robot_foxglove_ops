import importlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "mos_cart_fullness" / "cart_fullness_node.py"


def node_module():
    return importlib.import_module("mos_cart_fullness.cart_fullness_node")


def bare_node():
    return node_module().CartFullnessNode.__new__(node_module().CartFullnessNode)


def load_params():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "cart_fullness.yaml").read_text(encoding="utf-8")
    )
    return config["mos_cart_fullness"]["ros__parameters"]


def test_three_rgb_topics_are_configured():
    params = load_params()
    assert params["left_rgb_topic"] == "/left_arm_camera/color/image_raw"
    assert params["right_rgb_topic"] == "/right_arm_camera/color/image_raw"
    assert params["head_rgb_topic"] == "/head_camera/color/image_raw"


def test_node_subscribes_head_without_mos_sdk():
    source = NODE_PATH.read_text(encoding="utf-8")
    assert "self.sub_head_rgb" in source
    assert "self._on_head_rgb" in source
    assert "MosSensorAdapter" not in source
    assert "get_image_data" not in source


def test_readme_documents_all_modes_and_head_topic():
    text = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    assert "scan_wrist" in text
    assert "scan_dual" in text
    assert "verify_dual" in text
    assert "/head_camera/color/image_raw" in text
    assert "any available" in text


def test_node_builds_dual_preview_without_controlling_inference():
    source = NODE_PATH.read_text(encoding="utf-8")
    assert "_preview_window_name" in source
    assert "cv2.hconcat" not in source
    assert "wrist_decision" in source
    assert "head_decision" in source


def test_preview_does_not_draw_derived_roi_boxes_or_opaque_overlay():
    source = NODE_PATH.read_text(encoding="utf-8")
    assert "CURRENT_CART" not in source
    assert "LOAD_ROI" not in source
    assert "USED_ITEM" not in source
    assert "self._draw_overlay(frame, lines)" not in source
    assert "_draw_status" in source


def test_dual_preview_handles_missing_head_and_waits_once(monkeypatch):
    module = node_module()
    state = importlib.import_module("mos_cart_fullness.multiview_state")
    node = bare_node()
    node.preview = True
    node.preview_wait_ms = 1
    node.preview_window = "test"
    node.item_class_ids = {9}
    node.active_context = SimpleNamespace(
        mode="verify_dual", check_id="check-1", wrist_camera="left"
    )
    wrist_frame = np.zeros((20, 40, 3), dtype=np.uint8)
    wrist_eval = state.ViewEvaluation(
        decision="full", reason="full", detections=[], cart_candidates=[],
        target=None, stable_frames=2, fullness_result=None,
        summary={"full_frames": 2, "not_full_frames": 0},
    )
    head_eval = state.ViewEvaluation(
        decision="unknown", reason="no_head_frame", detections=[],
        cart_candidates=[], target=None, stable_frames=0, fullness_result=None,
        summary={"full_frames": 0, "not_full_frames": 0},
    )
    wrist_view = module.EvaluatedView(
        state.FrameSnapshot(wrist_frame, 1.0, 2.0), wrist_eval
    )
    head_view = module.EvaluatedView(None, head_eval)
    rendered = []
    shown = []
    waited = []
    node._render_preview_frame = lambda image, **kwargs: rendered.append(image) or image
    monkeypatch.setattr(module.cv2, "imshow", lambda name, image: shown.append((name, image)))
    monkeypatch.setattr(module.cv2, "waitKey", lambda wait_ms: waited.append(wait_ms) or -1)

    node._show_dual_preview(
        node.active_context, wrist_view, head_view, fused_decision="unknown"
    )

    assert len(rendered) == 1
    assert [name for name, _ in shown] == ["test/wrist_left"]
    assert shown[0][1].shape == (20, 40, 3)
    assert waited == [1]


def test_single_preview_q_closes_window(monkeypatch):
    module = node_module()
    node = bare_node()
    node.preview = True
    node.preview_wait_ms = 1
    node.preview_window = "test"
    destroyed = []
    monkeypatch.setattr(module.cv2, "waitKey", lambda wait_ms: ord("q"))
    monkeypatch.setattr(module.cv2, "destroyWindow", lambda name: destroyed.append(name))

    node._handle_preview_key()

    assert node.preview is False
    assert set(destroyed) == {
        "test",
        "test/wrist_left",
        "test/wrist_right",
        "test/head",
    }


def test_dual_after_decision_previews_without_publishing_route():
    module = node_module()
    state = importlib.import_module("mos_cart_fullness.multiview_state")
    node = bare_node()
    node.active_context = SimpleNamespace(
        mode="verify_dual", wrist_camera="left", check_id="check-1"
    )
    node.decision_published = True
    node.preview = True
    node.preview_after_decision = True
    node.enabled = False
    evaluation = state.ViewEvaluation(
        decision="full", reason="full", detections=[], cart_candidates=[],
        target=None, stable_frames=2, fullness_result=None,
        summary={"full_frames": 2, "not_full_frames": 0},
    )
    view = module.EvaluatedView(None, evaluation)
    calls = []
    routes = []
    node.wrist_evaluator = object()
    node._evaluate_view = lambda camera, evaluator: calls.append(camera) or view
    node._evaluate_synchronized_head = lambda context, wrist: calls.append("head") or view
    node._show_dual_preview = lambda *args, **kwargs: calls.append("preview")
    node.pub_route_signal = SimpleNamespace(publish=lambda message: routes.append(message))

    node._tick()

    assert calls == ["left", "head", "preview"]
    assert routes == []


def test_multiview_parameters_are_declared():
    params = load_params()
    assert params["max_frame_age_sec"] > 0.0
    assert params["max_pair_delta_sec"] > 0.0
    assert params["verify_timeout_sec"] > 0.0
    assert "head_y_start_ratio" in params
    assert "head_y_end_ratio" in params


def test_cart_semantic_class_parameters_are_configured():
    params = load_params()

    assert params["empty_cart_class_ids"] == "0,1,2"
    assert params["loaded_cart_class_ids"] == "3,4,5"


def test_node_uses_task_two_timestamped_store_and_context_lifecycle():
    source = NODE_PATH.read_text(encoding="utf-8")
    assert "from .multiview_state import" in source
    assert "time.monotonic()" in source
    assert "msg.header.stamp" in source
    assert "max_age_sec=self.max_frame_age_sec" in source
    assert "self.active_context" in source
    assert "self.wrist_evaluator.reset()" in source
    assert "self.head_evaluator.reset()" in source


def test_on_rgb_records_receipt_before_bridge_and_stores_ros_timestamp(monkeypatch):
    module = node_module()
    node = bare_node()
    calls = []
    image = np.ones((2, 2, 3), dtype=np.uint8)

    class Bridge:
        def imgmsg_to_cv2(self, msg, desired_encoding):
            calls.append("bridge")
            assert calls == ["monotonic", "bridge"]
            assert desired_encoding == "bgr8"
            return image

    class Frames:
        def update(self, camera, frame, *, stamp_sec, received_sec):
            calls.append((camera, frame, stamp_sec, received_sec))

    node.bridge = Bridge()
    node.frames = Frames()
    node.frame_errors = {"left": ""}
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    monkeypatch.setattr(module.time, "monotonic", lambda: calls.append("monotonic") or 12.5)
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=250000000))
    )

    node._on_rgb("left", msg)

    assert calls[0:2] == ["monotonic", "bridge"]
    assert calls[2][0] == "left"
    assert calls[2][1] is image
    assert calls[2][2:] == (7.25, 12.5)


def test_wrist_tick_unpacks_snapshot_frame_and_passes_max_age_to_store(monkeypatch):
    module = node_module()
    state = importlib.import_module("mos_cart_fullness.multiview_state")
    node = bare_node()
    frame = np.ones((2, 2, 3), dtype=np.uint8)
    seen = {}

    class Frames:
        def get(self, camera, *, now_sec, max_age_sec):
            seen.update(camera=camera, now_sec=now_sec, max_age_sec=max_age_sec)
            return state.FrameSnapshot(frame=frame, stamp_sec=1.0, received_sec=now_sec)

    node.active_context = SimpleNamespace(
        check_id="legacy:left_start",
        mode="scan_wrist",
        zone_id="left_start",
        wrist_camera="left",
    )
    node.enabled = True
    node.decision_published = False
    node.preview = False
    node.max_frame_age_sec = 0.4
    node.frames = Frames()
    node.frame_errors = {"left": ""}
    node.detector = SimpleNamespace(detect=lambda image, track: [])
    node.wrist_evaluator = SimpleNamespace(
        evaluate=lambda detections: state.ViewEvaluation(
            decision="unknown",
            reason="no_cart",
            detections=detections,
            cart_candidates=[],
            target=None,
            stable_frames=0,
            fullness_result=None,
            summary={"full_frames": 0, "not_full_frames": 0},
        )
    )
    node.judge = SimpleNamespace(filter_detections_by_class=lambda *args, **kwargs: [])
    node.cart_class_ids = {0}
    node.item_class_ids = {9}
    node.min_cart_score = 0.1
    node.min_item_score = 0.1
    node.track = False
    node._show_preview = lambda *args, **kwargs: None
    node._publish_status = lambda *args, **kwargs: None
    monkeypatch.setattr(module.time, "monotonic", lambda: 9.0)

    node._tick()

    assert seen == {"camera": "left", "now_sec": 9.0, "max_age_sec": 0.4}


def test_start_replaces_context_and_resets_both_evaluators(monkeypatch):
    node = bare_node()
    resets = []
    node.active_context = SimpleNamespace(check_id="old")
    node.waypoint_camera_map = {"new": "right"}
    node.waypoint_cart_map = {"new": "cart-new"}
    node.verify_timeout_sec = 5.0
    node.wrist_evaluator = SimpleNamespace(reset=lambda: resets.append("wrist"))
    node.head_evaluator = SimpleNamespace(reset=lambda: resets.append("head"))
    node.enabled = False
    node.decision_published = True
    statuses = []
    node._publish_status = lambda state, **kwargs: statuses.append((state, kwargs))
    monkeypatch.setattr(node_module().time, "monotonic", lambda: 23.0)

    node._on_waypoint(SimpleNamespace(data=json.dumps({
        "check_id": "check-2",
        "mode": "scan_wrist",
        "waypoint": "new",
        "wrist_camera": "right",
    })))

    assert node.active_context.check_id == "check-2"
    assert node.active_context.wrist_camera == "right"
    assert resets == ["wrist", "head"]
    assert node.context_started_monotonic == 23.0
    assert node.decision_published is False
    assert node.enabled is True
    assert statuses == []


@pytest.mark.parametrize("command", ["off", "reset"])
def test_off_and_reset_clear_active_state(command):
    node = bare_node()
    resets = []
    node.active_context = SimpleNamespace(check_id="check-1")
    node.context_started_monotonic = 11.0
    node.decision_published = True
    node.enabled = True
    node.wrist_evaluator = SimpleNamespace(reset=lambda: resets.append("wrist"))
    node.head_evaluator = SimpleNamespace(reset=lambda: resets.append("head"))
    node._publish_status = lambda *args, **kwargs: None
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)

    if command == "off":
        node.waypoint_camera_map = {}
        node.waypoint_cart_map = {}
        node.verify_timeout_sec = 5.0
        node._on_waypoint(SimpleNamespace(data=command))
    else:
        node._handle_reset(SimpleNamespace(), SimpleNamespace())

    assert node.active_context is None
    assert node.context_started_monotonic is None
    assert node.decision_published is False
    assert resets == ["wrist", "head"]
    if command == "off":
        assert node.enabled is False


def test_set_enabled_false_clears_active_context_and_both_views():
    node = bare_node()
    resets = []
    node.active_context = SimpleNamespace(check_id="check-1")
    node.context_started_monotonic = 11.0
    node.decision_published = True
    node.enabled = True
    node.completed_check_ids = {"check-1"}
    node.wrist_evaluator = SimpleNamespace(reset=lambda: resets.append("wrist"))
    node.head_evaluator = SimpleNamespace(reset=lambda: resets.append("head"))
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    response = SimpleNamespace()

    node._handle_set_enabled(SimpleNamespace(data=False), response)

    assert response.success is True
    assert node.enabled is False
    assert node.active_context is None
    assert node.context_started_monotonic is None
    assert node.decision_published is False
    assert resets == ["wrist", "head"]
    assert node.completed_check_ids == {"check-1"}


def test_completed_extended_check_id_is_idempotently_ignored():
    node = bare_node()
    node.active_context = SimpleNamespace(check_id="old")
    node.completed_check_ids = {"check-1"}
    node.waypoint_camera_map = {}
    node.waypoint_cart_map = {}
    node.verify_timeout_sec = 5.0
    statuses = []
    node._publish_status = lambda state, **kwargs: statuses.append((state, kwargs))

    node._on_waypoint(SimpleNamespace(data=json.dumps({
        "check_id": "check-1",
        "mode": "verify_dual",
        "waypoint": "cart_a_grasp",
        "zone_id": "scan_cart_a",
        "cart_id": "cart_a",
        "wrist_camera": "left",
        "timeout_sec": 5.0,
    })))

    assert node.active_context.check_id == "old"
    assert statuses[0][0] == "duplicate_check_ignored"


def test_waypoint_reset_clears_completed_check_ids():
    node = bare_node()
    node.active_context = SimpleNamespace(check_id="old")
    node.completed_check_ids = {"check-1"}
    node.waypoint_camera_map = {}
    node.waypoint_cart_map = {}
    node.verify_timeout_sec = 5.0
    node.wrist_evaluator = SimpleNamespace(reset=lambda: None)
    node.head_evaluator = SimpleNamespace(reset=lambda: None)
    node._publish_status = lambda *args, **kwargs: None

    node._on_waypoint(SimpleNamespace(data="reset"))

    assert node.active_context is None
    assert node.completed_check_ids == set()
