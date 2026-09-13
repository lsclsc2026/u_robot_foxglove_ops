#!/usr/bin/env python3
"""ROS2 dummy node for testing cart fullness detection without hardware."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

# from .cart_fullness import CartFullnessJudge
# from .first_cart_tracker import FirstStableCartTracker
# from .stability import FullnessDecisionWindow
# from .wrist_yolo_detector import WristYoloDetector


CAMERA_ENUM_NAMES = {
    "left": "LEFT_ARM_CAMERA",
    "right": "RIGHT_ARM_CAMERA",
}


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


class DummyCartFullnessNode(Node):
    def __init__(self):
        super().__init__("dummy_mos_cart_fullness")
        print("【DUMMY】初始化 DummyCartFullnessNode")
        self._declare_parameters()
        self._load_parameters()

        # try:
        #     from mos_sdk import CameraType, MosSensorAdapter
        # except ImportError as exc:
        #     raise RuntimeError(
        #         "mos_sdk is unavailable. Run this node inside the Lumos/MOS SDK environment."
        #     ) from exc
        print("【DUMMY】跳过 mos_sdk 导入")

        # self.camera_types = {
        #     name: getattr(CameraType, enum_name)
        #     for name, enum_name in CAMERA_ENUM_NAMES.items()
        # }
        # self.sensor = MosSensorAdapter(self.config)
        # if not self.sensor.start():
        #     raise RuntimeError("MosSensorAdapter.start() failed")
        # if self.warmup_seconds > 0.0:
        #     time.sleep(self.warmup_seconds)
        print(f"【DUMMY】跳过 MosSensorAdapter，config={self.config}, warmup={self.warmup_seconds}s")
        self.camera_types = CAMERA_ENUM_NAMES
        self.sensor = None

        # self.detector = WristYoloDetector(
        #     model_size=self.model_size,
        #     weights=self.weights,
        #     conf=self.conf,
        #     iou=self.iou,
        #     device=self.device,
        # )
        print(f"【DUMMY】跳过 WristYoloDetector，model_size={self.model_size}, device={self.device}")
        self.detector = None

        # self.judge = CartFullnessJudge(
        #     full_threshold=self.full_threshold,
        #     count_threshold=self.count_threshold,
        #     count_operator=self.count_operator,
        #     item_class_ids=self.item_class_ids,
        #     cart_class_ids=self.cart_class_ids,
        #     min_item_score=self.min_item_score,
        #     min_cart_score=self.min_cart_score,
        #     x_margin_ratio=self.x_margin_ratio,
        #     y_start_ratio=self.y_start_ratio,
        #     y_end_ratio=self.y_end_ratio,
        #     min_item_roi_overlap=self.min_item_roi_overlap,
        # )
        print(f"【DUMMY】跳过 CartFullnessJudge，full_threshold={self.full_threshold}")
        self.judge = None

        # self.tracker = FirstStableCartTracker(
        #     lock_frames=self.lock_frames,
        #     lost_frames=self.target_lost_frames,
        #     min_iou=self.min_lock_iou,
        # )
        print(f"【DUMMY】跳过 FirstStableCartTracker，lock_frames={self.lock_frames}")
        self.tracker = None

        # self.decision_window_state = FullnessDecisionWindow(
        #     window=self.decision_window,
        #     min_full=self.min_full_frames,
        #     min_not_full=self.min_not_full_frames,
        # )
        print(f"【DUMMY】跳过 FullnessDecisionWindow，window={self.decision_window}")
        self.decision_window_state = None

        self.active_waypoint = ""
        self.active_camera = ""
        self.active_cart_id = ""
        self.decision_published = False

        self.sub_waypoint = self.create_subscription(
            String,
            self.waypoint_topic,
            self._on_waypoint,
            10,
        )
        print(f"【DUMMY】订阅话题: {self.waypoint_topic}")
        self.pub_route_signal = self.create_publisher(String, self.route_signal_topic, 10)
        print(f"【DUMMY】创建发布器: {self.route_signal_topic}")
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        print(f"【DUMMY】创建发布器: {self.status_topic}")
        self.srv_set_enabled = self.create_service(
            SetBool,
            "~/set_enabled",
            self._handle_set_enabled,
        )
        self.srv_reset = self.create_service(Trigger, "~/reset", self._handle_reset)
        self.timer = self.create_timer(self.camera_poll_seconds, self._tick)

        self.get_logger().info(
            f"【DUMMY】启动 cart_fullness 节点 enabled={self.enabled} "
            f"waypoint_topic={self.waypoint_topic} route_signal={self.route_signal_topic}"
        )
        
        # DUMMY: 模拟帧计数用于生成不同的满度判定
        self.dummy_frame_count = 0

    def _declare_parameters(self):
        defaults = {
            "enabled": True,
            "config": "/opt/lumos/config/mos_sensor/example.yaml",
            "camera_poll_seconds": 0.50,  # DUMMY: 0.5秒一次
            "warmup_seconds": 0.5,
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
            "item_class_ids": "9,10",
            "min_cart_score": 0.25,
            "min_item_score": 0.25,
            "full_threshold": 0.45,
            "count_threshold": 4,
            "count_operator": ">=",
            "x_margin_ratio": 0.15,
            "y_start_ratio": 0.55,
            "y_end_ratio": 0.88,
            "min_item_roi_overlap": 0.10,
            "lock_frames": 2,
            "target_lost_frames": 3,
            "min_lock_iou": 0.30,
            "decision_window": 5,
            "min_full_frames": 3,
            "min_not_full_frames": 3,
            "full_signal": "continue_to_next_waypoint_for_grasp",
            "not_full_signal": "skip_next_waypoint",
            "full_route_step_delta": 1,
            "not_full_route_step_delta": 2,
            "publish_status": True,
            "disable_after_decision": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.enabled = bool(self.get_parameter("enabled").value)
        self.config = str(self.get_parameter("config").value)
        self.camera_poll_seconds = max(
            0.01,
            float(self.get_parameter("camera_poll_seconds").value),
        )
        self.warmup_seconds = max(0.0, float(self.get_parameter("warmup_seconds").value))
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
        self.lock_frames = int(self.get_parameter("lock_frames").value)
        self.target_lost_frames = int(self.get_parameter("target_lost_frames").value)
        self.min_lock_iou = float(self.get_parameter("min_lock_iou").value)
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

    def _on_waypoint(self, msg: String):
        print(f"【DUMMY】收到 waypoint 消息: {msg.data}")
        waypoint = extract_waypoint(msg.data)
        if waypoint in {"", "off", "disable", "disabled"}:
            self._reset_active_state()
            self.enabled = False
            self._publish_status("disabled", reason="waypoint_disable")
            print("【DUMMY】waypoint 禁用，节点停止检测")
            return
        if waypoint == "reset":
            self._reset_active_state()
            self._publish_status("reset", reason="waypoint_reset")
            print("【DUMMY】waypoint 重置")
            return

        camera = self.waypoint_camera_map.get(waypoint)
        if camera not in self.camera_types:
            self._publish_status(
                "unsupported_waypoint",
                waypoint=waypoint,
                reason="no_camera_mapping",
            )
            print(f"【DUMMY】不支持的 waypoint: {waypoint}, 无相机映射")
            return

        self._reset_active_state()
        self.active_waypoint = waypoint
        self.active_camera = camera
        self.active_cart_id = self.waypoint_cart_map.get(waypoint, waypoint)
        self.enabled = True
        self._publish_status(
            "checking",
            waypoint=waypoint,
            camera=camera,
            cart_id=self.active_cart_id,
            reason="waypoint_accepted",
        )
        print(f"【DUMMY】开始检测: waypoint={waypoint}, camera={camera}, cart_id={self.active_cart_id}")

    def _tick(self):
        if not self.enabled or not self.active_camera or self.decision_published:
            return

        self.dummy_frame_count += 1
        # 只在第一帧打印
        if self.dummy_frame_count == 1:
            print(f"【DUMMY】开始检测: camera={self.active_camera}, 需要 3 帧稳定")

        # camera_type = self.camera_types[self.active_camera]
        # rgb = self.sensor.get_image_data(camera_type)
        # if rgb is None:
        #     self._publish_status("checking", reason="no_rgb_frame")
        #     return
        rgb = True

        # image = rgb.convert_to_cv2_mat()
        # if image is None or image.size == 0:
        #     self._publish_status("checking", reason="empty_rgb_frame")
        #     return
        # image = np.zeros((480, 640, 3), dtype=np.uint8)

        # detections = self.detector.detect(image, track=self.track)
        # cart_candidates = self._cart_candidates(detections)
        # tracking = self.tracker.update(cart_candidates)
        # if tracking.target is None or not tracking.locked:
        #     self.decision_window_state.reset()
        #     self._publish_status(
        #         "checking",
        #         reason=tracking.reason,
        #         stable_frames=tracking.stable_frames,
        #         cart_candidates=len(cart_candidates),
        #     )
        #     return
        
        # 模拟经过足够帧数后判定稳定
        if self.dummy_frame_count < 3:
            self._publish_status(
                "checking",
                reason="waiting_for_stable_cart",
                stable_frames=self.dummy_frame_count,
                cart_candidates=1,
            )
            return

        # items = self.judge.filter_detections_by_class(
        #     detections,
        #     self.item_class_ids,
        #     min_score=self.min_item_score,
        # )
        # result = self.judge.judge_cart(
        #     tracking.target["bbox_xyxy"],
        #     items,
        # )
        # decision = self.decision_window_state.add(bool(result["is_full"]))
        # summary = self.decision_window_state.summary()
        print("【DUMMY】跳过满度判定，生成硬编码结果")
        
        # DUMMY: 根据 waypoint 生成不同的满度判定
        # left_start -> not_full (空车)
        # right_start -> full (满车)
        if "left" in self.active_waypoint:
            is_full = False
            decision = "not_full"
            occupancy_ratio = 0.25
            used_item_count = 2
        else:
            is_full = True
            decision = "full"
            occupancy_ratio = 0.68
            used_item_count = 7
        
        result = {
            "is_full": is_full,
            "occupancy_ratio": occupancy_ratio,
            "used_item_count": used_item_count,
        }
        summary = {
            "full_count": 5 if is_full else 1,
            "not_full_count": 0 if is_full else 4,
            "window_size": 5,
        }
        
        print(f"【DUMMY】判定结果: decision={decision}, occupancy={occupancy_ratio:.2f}, items={used_item_count}")

        # if decision in {"full", "not_full"}:
        #     self._publish_route_signal(decision, result, summary)
        #     self.decision_published = True
        #     if self.disable_after_decision:
        #         self.enabled = False
        #     return
        self._publish_route_signal(decision, result, summary)
        self.decision_published = True
        if self.disable_after_decision:
            self.enabled = False
        print(f"【DUMMY】已发布路由信号: {decision}")
        return

        # self._publish_status(
        #     "checking",
        #     reason="collecting_fullness_frames",
        #     cart_id=self.active_cart_id,
        #     camera=self.active_camera,
        #     is_full=bool(result["is_full"]),
        #     occupancy_ratio=float(result.get("occupancy_ratio", 0.0)),
        #     used_item_count=int(result.get("used_item_count", 0)),
        #     **summary,
        # )

    def _cart_candidates(self, detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
        # return self.judge.filter_detections_by_class(
        #     detections,
        #     self.cart_class_ids,
        #     min_score=self.min_cart_score,
        # )
        print("【DUMMY】跳过 _cart_candidates")
        return []

    def _publish_route_signal(
        self,
        decision: str,
        fullness_result: dict[str, Any],
        summary: dict[str, int],
    ):
        is_full = decision == "full"
        payload = {
            "waypoint": self.active_waypoint,
            "camera": self.active_camera,
            "cart_id": self.active_cart_id,
            "signal": self.full_signal if is_full else self.not_full_signal,
            "decision": decision,
            "route_step_delta": (
                self.full_route_step_delta if is_full else self.not_full_route_step_delta
            ),
            "reason": (
                "first_stable_cart_full"
                if is_full
                else "first_stable_cart_not_full"
            ),
            "occupancy_ratio": float(fullness_result.get("occupancy_ratio", 0.0)),
            "used_item_count": int(fullness_result.get("used_item_count", 0)),
            **summary,
        }
        self.pub_route_signal.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self._publish_status("route_signal_published", **payload)
        print(f"【DUMMY】发布路由信号: {json.dumps(payload, ensure_ascii=False, indent=2)}")

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

    def _reset_active_state(self):
        self.active_waypoint = ""
        self.active_camera = ""
        self.active_cart_id = ""
        self.decision_published = False
        # self.tracker.reset()
        # self.decision_window_state.reset()
        self.dummy_frame_count = 0
        print("【DUMMY】重置活动状态")

    def _handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = (
            "dummy_cart_fullness enabled"
            if self.enabled
            else "dummy_cart_fullness disabled"
        )
        print(f"【DUMMY】服务调用: {response.message}")
        self.get_logger().info(response.message)
        return response

    def _handle_reset(self, request, response):
        del request
        self._reset_active_state()
        response.success = True
        response.message = "dummy_cart_fullness reset"
        print(f"【DUMMY】服务调用: {response.message}")
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        # if hasattr(self, "sensor") and self.sensor is not None:
        #     self.sensor.stop()
        print("【DUMMY】跳过 sensor.stop()")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DummyCartFullnessNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
