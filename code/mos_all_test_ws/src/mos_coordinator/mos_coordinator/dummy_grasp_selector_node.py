#!/usr/bin/env python3
"""ROS2 dummy node for testing grasp selector without hardware."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

try:
    from mos_3d_object.msg import DetectedObject3DArray
    from mos_grasp_selector.msg import GraspPoint, GraspTask, GraspTaskArray
except ImportError:
    print("【DUMMY】警告: mos_3d_object.msg 或 mos_grasp_selector.msg 无法导入")
    DetectedObject3DArray = None
    GraspPoint = None
    GraspTask = None
    GraspTaskArray = None

# try:
#     from ament_index_python.packages import get_package_share_directory
# except ImportError:
#     get_package_share_directory = None


def finite_xyz(point):
    return (
        math.isfinite(float(point.x))
        and math.isfinite(float(point.y))
        and math.isfinite(float(point.z))
    )


def point_xyz(point):
    return (float(point.x), float(point.y), float(point.z))


def distance_from_origin(point):
    x, y, z = point_xyz(point)
    return math.sqrt(x * x + y * y + z * z)


def median(values):
    ordered = sorted(float(value) for value in values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def mean(values):
    values = [float(value) for value in values]
    return sum(values) / len(values)


def axis_std(points, axis):
    values = [point[axis] for point in points]
    mean_val = sum(values) / len(values)
    variance = sum((value - mean_val) ** 2 for value in values) / len(values)
    return math.sqrt(variance)


def axis_range(points, axis):
    values = [point[axis] for point in points]
    return max(values) - min(values)


def tuple_finite(xyz):
    return all(math.isfinite(float(value)) for value in xyz)


def point_distance(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def normalize_int_list(value):
    if value is None:
        return []
    if isinstance(value, str):
        items = value.split(",")
    elif isinstance(value, (list, tuple)):
        items = value
    else:
        items = [value]
    result = []
    for item in items:
        text = str(item).strip()
        if text:
            result.append(int(text))
    return result


def load_yaml_file(path):
    # try:
    #     import yaml
    # except ImportError as exc:
    #     raise RuntimeError("PyYAML is required to load grasp selector rules") from exc
    # with Path(path).open("r", encoding="utf-8") as file:
    #     data = yaml.safe_load(file)
    # return data or {}
    print(f"【DUMMY】跳过 load_yaml_file({path})")
    return {}


def as_dict(value):
    return value if isinstance(value, dict) else {}


def normalize_class_map(raw_map):
    result = {}
    for key, value in as_dict(raw_map).items():
        item = as_dict(value)
        result[int(key)] = {
            "yolo_name": str(item.get("yolo_name", "")),
            "zh_name": str(item.get("zh_name", "")),
            "type": str(item.get("type", "")),
        }
    return result


class DummyGraspSelector(Node):
    def __init__(self):
        super().__init__("dummy_mos_grasp_selector")
        print("【DUMMY】初始化 DummyGraspSelector")
        self._declare_parameters()
        self._load_parameters()

        self.history = deque(maxlen=self.stability_window)
        self.sub_detections = self.create_subscription(
            DetectedObject3DArray,
            self.detections_topic,
            self._on_detections,
            10,
        )
        print(f"【DUMMY】订阅话题: {self.detections_topic}")
        self.pub_tasks = self.create_publisher(
            GraspTaskArray, self.grasp_tasks_topic, 10
        )
        print(f"【DUMMY】创建发布器: {self.grasp_tasks_topic}")
        self.pub_json = None
        if self.publish_json_debug:
            self.pub_json = self.create_publisher(
                String, self.grasp_tasks_json_topic, 10
            )
            print(f"【DUMMY】创建发布器: {self.grasp_tasks_json_topic}")
        self.srv_set_enabled = self.create_service(
            SetBool, "~/set_enabled", self._handle_set_enabled
        )
        self.srv_reset = self.create_service(Trigger, "~/reset", self._handle_reset)

        self.get_logger().info(
            f"【DUMMY】启动 grasp selector task={self.task_name} "
            f"input={self.detections_topic} output={self.grasp_tasks_topic} "
            f"enabled={self.enabled}"
        )
        
        # DUMMY: 跟踪用于生成稳定结果
        self.dummy_detection_count = 0

    def _declare_parameters(self):
        defaults = {
            "detections_topic": "/vision/detections_3d",
            "grasp_tasks_topic": "/vision/grasp_tasks",
            "grasp_tasks_json_topic": "/vision/grasp_tasks_json",
            "publish_json_debug": True,
            "publish_empty_tasks": True,
            "publish_once_when_ready": True,
            "enabled": True,  # DUMMY: 默认启用
            "active_task": "loaded_treatment_cart_lift",
            "rules_file": "config/grasp_selector_rules.yaml",
            "class_map_file": "config/class_map.yaml",
            "task_name": "loaded_treatment_cart_lift",
            "grasp_mode": "dual_arm_sync",
            "target_frame": "body_link",
            "require_trigger": False,
            "trigger_class_id": 1,
            "trigger_class_ids": [0, 1],
            "trigger_class_name": "金属治疗车",
            "target_class_id": 2,
            "target_class_name": "治疗车推车把手",
            "required_points": 2,
            "min_trigger_score": 0.0,
            "min_target_score": 0.0,
            "max_target_distance_m": 0.0,
            "use_stability": True,
            "stability_window": 10,
            "min_stable_frames": 5,
            "position_filter": "median",
            "outlier_distance_m": 0.08,
            "max_position_std_m": 0.03,
            "max_position_range_m": 0.06,
            "positive_y_role": "left_handle",
            "negative_y_role": "right_handle",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.detections_topic = self.get_parameter("detections_topic").value
        self.grasp_tasks_topic = self.get_parameter("grasp_tasks_topic").value
        self.grasp_tasks_json_topic = self.get_parameter(
            "grasp_tasks_json_topic"
        ).value
        self.publish_json_debug = bool(
            self.get_parameter("publish_json_debug").value
        )
        self.publish_empty_tasks = bool(
            self.get_parameter("publish_empty_tasks").value
        )
        self.publish_once_when_ready = bool(
            self.get_parameter("publish_once_when_ready").value
        )
        self.enabled = bool(self.get_parameter("enabled").value)
        self.active_task = str(self.get_parameter("active_task").value)
        self.rules_file = str(self.get_parameter("rules_file").value)
        self.class_map_file = str(self.get_parameter("class_map_file").value)
        self.task_name = self.get_parameter("task_name").value
        self.grasp_mode = self.get_parameter("grasp_mode").value
        self.target_frame = self.get_parameter("target_frame").value
        self.require_trigger = bool(self.get_parameter("require_trigger").value)
        self.trigger_class_id = int(self.get_parameter("trigger_class_id").value)
        self.trigger_class_ids = normalize_int_list(
            self.get_parameter("trigger_class_ids").value
        )
        if not self.trigger_class_ids:
            self.trigger_class_ids = [self.trigger_class_id]
        self.trigger_class_name = self.get_parameter("trigger_class_name").value
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = self.get_parameter("target_class_name").value
        self.required_points = max(
            1, int(self.get_parameter("required_points").value)
        )
        self.min_trigger_score = float(
            self.get_parameter("min_trigger_score").value
        )
        self.min_target_score = float(self.get_parameter("min_target_score").value)
        max_distance = float(self.get_parameter("max_target_distance_m").value)
        self.max_target_distance_m = max_distance if max_distance > 0.0 else None
        self.use_stability = bool(self.get_parameter("use_stability").value)
        self.stability_window = max(
            1, int(self.get_parameter("stability_window").value)
        )
        self.min_stable_frames = max(
            1, int(self.get_parameter("min_stable_frames").value)
        )
        if self.min_stable_frames > self.stability_window:
            self.get_logger().warn(
                f"min_stable_frames ({self.min_stable_frames}) > stability_window "
                f"({self.stability_window}); adjusting to {self.stability_window}"
            )
            self.min_stable_frames = self.stability_window
        self.max_position_std_m = max(
            0.0, float(self.get_parameter("max_position_std_m").value)
        )
        self.position_filter = str(self.get_parameter("position_filter").value).lower()
        if self.position_filter not in {"median", "mean"}:
            self.get_logger().warn(
                f"unsupported position_filter={self.position_filter}; using median"
            )
            self.position_filter = "median"
        self.outlier_distance_m = max(
            0.0, float(self.get_parameter("outlier_distance_m").value)
        )
        self.max_position_range_m = max(
            0.0, float(self.get_parameter("max_position_range_m").value)
        )
        self.positive_y_role = self.get_parameter("positive_y_role").value
        self.negative_y_role = self.get_parameter("negative_y_role").value
        self.role_mode = "split_by_y" if self.required_points == 2 else "single"
        self.single_role = "pick_point"
        self.selection_mode = "nearest"
        self.trigger_mode = "any" if self.require_trigger else "none"
        self.position_mode = "detected"
        self.position_offset = (0.0, 0.0, 0.0)
        self.class_map = {}
        self.task_rules = {}
        self.current_rule = {}
        # self._load_rule_files()
        # self._apply_active_task()
        print("【DUMMY】跳过 _load_rule_files 和 _apply_active_task")
        self.ready_task_published = False
        self._enabled_start_time = time.perf_counter() if self.enabled else None
        self._first_candidate_time = None

    def _load_rule_files(self):
        # self.class_map = {}
        # self.task_rules = {}
        # class_map_path = self._resolve_config_path(self.class_map_file)
        # if class_map_path is not None:
        #     try:
        #         data = load_yaml_file(class_map_path)
        #         self.class_map = normalize_class_map(data.get("class_map"))
        #     except Exception as exc:
        #         self.get_logger().warn(f"failed to load class_map from {class_map_path}: {exc}")
        # rules_path = self._resolve_config_path(self.rules_file)
        # if rules_path is None:
        #     self.get_logger().warn(
        #         f"rules_file={self.rules_file} not found; "
        #         f"using only direct parameters"
        #     )
        #     return
        # try:
        #     data = load_yaml_file(rules_path)
        # except Exception as exc:
        #     self.get_logger().warn(
        #         f"failed to load rules from {rules_path}: {exc}; using direct parameters"
        #     )
        #     return
        # if not self.class_map:
        #     self.class_map = normalize_class_map(data.get("class_map"))
        # self.task_rules = as_dict(data.get("tasks"))
        print("【DUMMY】跳过 _load_rule_files")
        pass

    def _resolve_config_path(self, value):
        # path = Path(str(value))
        # if path.is_absolute() and path.exists():
        #     return path
        # if path.exists():
        #     return path
        # candidates = []
        # if get_package_share_directory is not None:
        #     try:
        #         pkg_share = Path(get_package_share_directory("mos_grasp_selector"))
        #         candidates.append(pkg_share / path)
        #         candidates.append(pkg_share.parent / path)
        #     except Exception:
        #         pass
        # candidates.append(Path.cwd() / path)
        # for candidate in candidates:
        #     if candidate.exists():
        #         return candidate
        print(f"【DUMMY】跳过 _resolve_config_path({value})")
        return None

    def _apply_active_task(self):
        # rule = as_dict(self.task_rules.get(self.active_task))
        # if not rule:
        #     self.get_logger().warn(
        #         f"active_task={self.active_task} not found in rules; "
        #         f"using only direct parameters"
        #     )
        #     return
        # self.current_rule = rule
        # self.task_name = str(rule.get("task_name", self.active_task))
        # self.grasp_mode = str(rule.get("grasp_mode", self.grasp_mode))
        # self.target_class_id = int(rule.get("target_class_id", self.target_class_id))
        # self.target_class_name = self._class_display_name(
        #     self.target_class_id,
        #     str(rule.get("target_class_name", self.target_class_name)),
        # )
        # self.required_points = max(1, int(rule.get("required_points", self.required_points)))
        # self.publish_once_when_ready = bool(
        #     rule.get("publish_once_when_ready", self.publish_once_when_ready)
        # )
        # trigger = as_dict(rule.get("trigger"))
        # self.trigger_mode = str(trigger.get("mode", "none")).lower()
        # self.trigger_class_ids = normalize_int_list(trigger.get("class_ids", []))
        # self.require_trigger = self.trigger_mode in {"any", "all"}
        # selection = as_dict(rule.get("selection"))
        # self.selection_mode = str(selection.get("mode", "nearest")).lower()
        # max_distance = float(selection.get("max_distance_m", 0.0))
        # self.max_target_distance_m = max_distance if max_distance > 0.0 else None
        # self.min_target_score = float(selection.get("min_score", self.min_target_score))
        # role = as_dict(rule.get("role"))
        # self.role_mode = str(role.get("mode", self.role_mode)).lower()
        # self.negative_y_role = str(role.get("negative_y", self.negative_y_role))
        # self.positive_y_role = str(role.get("positive_y", self.positive_y_role))
        # self.single_role = str(role.get("name", self.single_role))
        # position = as_dict(rule.get("position"))
        # self.position_mode = str(position.get("mode", "detected")).lower()
        # self.position_offset = (
        #     float(position.get("offset_x", 0.0)),
        #     float(position.get("offset_y", 0.0)),
        #     float(position.get("offset_z", 0.0)),
        # )
        # stability = as_dict(rule.get("stability"))
        # if stability:
        #     self.use_stability = bool(stability.get("enabled", self.use_stability))
        #     self.stability_window = max(1, int(stability.get("window", self.stability_window)))
        #     self.min_stable_frames = max(
        #         1, int(stability.get("min_frames", self.min_stable_frames))
        #     )
        #     if self.min_stable_frames > self.stability_window:
        #         self.min_stable_frames = self.stability_window
        #     self.max_position_std_m = max(
        #         0.0, float(stability.get("max_std_m", self.max_position_std_m))
        #     )
        #     self.position_filter = str(stability.get("filter", self.position_filter)).lower()
        #     if self.position_filter not in {"median", "mean"}:
        #         self.position_filter = "median"
        #     self.outlier_distance_m = max(
        #         0.0, float(stability.get("outlier_distance_m", self.outlier_distance_m))
        #     )
        #     self.max_position_range_m = max(
        #         0.0, float(stability.get("max_range_m", self.max_position_range_m))
        #     )
        # self.get_logger().info(
        #     f"active_task={self.task_name} target_class_id={self.target_class_id} "
        #     f"required_points={self.required_points} selection={self.selection_mode} "
        #     f"role_mode={self.role_mode}"
        # )
        print("【DUMMY】跳过 _apply_active_task")
        pass

    def _class_display_name(self, class_id, fallback=""):
        item = self.class_map.get(int(class_id), {})
        return str(item.get("zh_name") or fallback or item.get("yolo_name") or class_id)

    def _on_detections(self, msg):
        # 只在关键时刻打印
        output = GraspTaskArray()
        output.header = msg.header

        if not self.enabled:
            self._publish_empty_or_status(output, "disabled")
            return

        self.dummy_detection_count += 1
        total_t0 = time.perf_counter()
        
        # select_t0 = time.perf_counter()
        # candidate, reason = self._select_candidate(msg)
        # select_t1 = time.perf_counter()
        # if candidate is None:
        #     self._publish_empty_or_status(output, reason)
        #     self.get_logger().info(
        #         f"TIMING grasp_selector no_candidate reason={reason} "
        #         f"detections={len(msg.detections)} "
        #         f"select_ms={(select_t1 - select_t0) * 1000.0:.1f}"
        #     )
        #     return
        candidate = self._generate_dummy_candidate(msg)
        if candidate is None:
            self._publish_empty_or_status(output, "no_handles_detected")
            return

        # now = time.perf_counter()
        # if self._first_candidate_time is None:
        #     self._first_candidate_time = now
        #     self.get_logger().info(
        #         f"first valid candidate after {now - self._enabled_start_time:.2f}s"
        #     )
        # else:
        #     elapsed = now - self._first_candidate_time
        #     self.get_logger().info(
        #         f"valid candidate at {elapsed:.2f}s since first"
        #     )

        self.history.append(candidate)
        if self.publish_once_when_ready and self.ready_task_published:
            return

        # stability_t0 = time.perf_counter()
        # ready, quality, reason, stable_points = self._evaluate_stability()
        # stability_t1 = time.perf_counter()
        # 移除冗余输出
        if self.dummy_detection_count < 3:
            self._publish_empty_or_status(output, "collecting_samples", quality="unstable")
            if self.dummy_detection_count == 1:
                print(f"【DUMMY】开始收集样本: 需要 {self.min_stable_frames} 帧")
            return
        
        ready = True
        quality = "ok"
        reason = "stable"
        stable_points = candidate["points"]
        
        if ready:
            task = self._build_dummy_task(msg, stable_points)
            output.tasks = [task]
            
            self.pub_tasks.publish(output)
            print(f"【DUMMY】发布抓取任务: {task.task_name} ({task.grasp_mode}), {len(task.points)}个点")
            
            if self.publish_once_when_ready:
                self.ready_task_published = True
            
            # self.get_logger().info(
            #     f"TIMING grasp_selector_summary ready=true quality={quality} "
            #     f"detections={len(msg.detections)} history={len(self.history)} "
            #     f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
            #     f"stability_ms={(stability_t1 - stability_t0) * 1000.0:.1f} "
            #     f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
            #     f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
            # )
            return

        # publish_t0 = time.perf_counter()
        # self._publish_empty_or_status(output, reason, quality=quality)
        # publish_t1 = time.perf_counter()
        # self.get_logger().info(
        #     f"TIMING grasp_selector_summary ready=false reason={reason} quality={quality} "
        #     f"detections={len(msg.detections)} history={len(self.history)} "
        #     f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
        #     f"stability_ms={(stability_t1 - stability_t0) * 1000.0:.1f} "
        #     f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
        #     f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
        # )

    def _generate_dummy_candidate(self, msg):
        """生成硬编码的把手候选"""
        # 模拟检测到两个把手（左右）
        # 假设接收到的 3D 检测消息包含购物车，把手位于左右两侧
        frame = msg.header.frame_id
        stamp = msg.header.stamp
        
        # 硬编码: 左把手在 y=0.3, 右把手在 y=-0.3，都在前方 1.5m
        left_handle = {
            "class_id": self.target_class_id,
            "score": 0.9,
            "distance": 1.52,
            "xyz": (1.5, 0.3, 0.8),  # x前, y左, z高
            "frame": frame,
            "stamp": stamp,
        }
        right_handle = {
            "class_id": self.target_class_id,
            "score": 0.88,
            "distance": 1.51,
            "xyz": (1.5, -0.3, 0.8),
            "frame": frame,
            "stamp": stamp,
        }
        
        candidate = {
            "points": {
                self.positive_y_role: left_handle,  # "left_handle"
                self.negative_y_role: right_handle,  # "right_handle"
            },
            "trigger": None,
            "frame": frame,
            "stamp": stamp,
        }
        
        # print(f"【DUMMY】生成候选: 左把手 {left_handle['xyz']}, 右把手 {right_handle['xyz']}")
        return candidate

    def _build_dummy_task(self, msg, stable_points):
        """构建硬编码的抓取任务"""
        task = GraspTask()
        task.header = msg.header
        task.task_name = self.task_name
        task.grasp_mode = self.grasp_mode
        task.target_frame = self.target_frame
        task.ready = True  # 标记任务已就绪
        task.quality = "ok"
        task.reason = "stable"
        task.target_class_id = 0
        task.target_class_name = "loaded_treatment_cart"
        task.required_points = 2
        
        grasp_points = []
        for role, item in stable_points.items():
            gp = GraspPoint()
            gp.header.stamp = item["stamp"]
            gp.header.frame_id = item["frame"]
            gp.role = role
            gp.rank = 0
            gp.class_id = 0
            gp.class_name = "loaded_treatment_cart"
            gp.source_object_id = 0
            gp.source_score = 0.85
            gp.point = PointStamped()
            gp.point.header.frame_id = item["frame"]
            gp.point.header.stamp = item["stamp"]
            gp.point.point.x = float(item["xyz"][0])
            gp.point.point.y = float(item["xyz"][1])
            gp.point.point.z = float(item["xyz"][2])
            gp.distance_m = float(sum(v**2 for v in item["xyz"])**0.5)
            grasp_points.append(gp)
        
        task.points = grasp_points
        return task

    def _select_candidate(self, msg):
        # frame = self._target_frame(msg)
        # trigger_candidates = [
        #     detection
        #     for detection in msg.detections
        #     if self._is_valid_trigger_detection(detection, frame)
        # ]
        # trigger_ok, trigger_reason = self._trigger_satisfied(trigger_candidates)
        # if not trigger_ok:
        #     return None, trigger_reason
        # target_candidates = [
        #     detection
        #     for detection in msg.detections
        #     if self._is_valid_detection(
        #         detection,
        #         self.target_class_id,
        #         self.min_target_score,
        #         frame,
        #     )
        # ]
        # if len(target_candidates) < self.required_points:
        #     return None, f"insufficient_targets:{len(target_candidates)}<{self.required_points}"
        # ranked_targets = self._rank_targets(target_candidates)
        # if self.max_target_distance_m is not None:
        #     ranked_targets = [item for item in ranked_targets if item[0] <= self.max_target_distance_m]
        # if len(ranked_targets) < self.required_points:
        #     return None, f"insufficient_targets_in_range:{len(ranked_targets)}<{self.required_points}"
        # selected = ranked_targets[: self.required_points]
        # role_items = self._assign_roles(selected)
        # return {
        #     "points": role_items,
        #     "trigger": self._best_trigger(trigger_candidates) if trigger_candidates else None,
        #     "frame": frame,
        #     "stamp": msg.header.stamp,
        # }, "ok"
        print("【DUMMY】跳过 _select_candidate")
        return None, "dummy"

    def _trigger_satisfied(self, trigger_candidates):
        # if self.trigger_mode in {"", "none", "false"}:
        #     return True, "no_trigger_required"
        # if self.trigger_mode == "any":
        #     return len(trigger_candidates) > 0, (
        #         "trigger_ok" if trigger_candidates else "no_trigger"
        #     )
        # if self.trigger_mode == "all":
        #     required_count = len(self.trigger_class_ids)
        #     if len(trigger_candidates) < required_count:
        #         return False, f"insufficient_triggers:{len(trigger_candidates)}<{required_count}"
        #     return True, "all_triggers_ok"
        # return False, f"unknown_trigger_mode:{self.trigger_mode}"
        print("【DUMMY】跳过 _trigger_satisfied")
        return True, "dummy"

    def _rank_targets(self, target_candidates):
        # items = [
        #     (distance_from_origin(detection.position_target.point), detection)
        #     for detection in target_candidates
        # ]
        # if self.selection_mode == "highest_score":
        #     return sorted(items, key=lambda item: item[1].score, reverse=True)
        # if self.selection_mode == "nearest":
        #     return sorted(items, key=lambda item: item[0])
        # self.get_logger().warn(
        #     f"unsupported selection_mode={self.selection_mode}; using nearest"
        # )
        # return sorted(items, key=lambda item: item[0])
        print("【DUMMY】跳过 _rank_targets")
        return []

    @staticmethod
    def _best_trigger(trigger_candidates):
        # return max(trigger_candidates, key=lambda d: d.score)
        print("【DUMMY】跳过 _best_trigger")
        return None

    def _is_valid_detection(self, detection, class_id, min_score, frame):
        # if int(detection.class_id) != int(class_id):
        #     return False
        # return self._is_valid_position_detection(detection, min_score, frame)
        print("【DUMMY】跳过 _is_valid_detection")
        return False

    def _is_valid_trigger_detection(self, detection, frame):
        # if int(detection.class_id) not in self.trigger_class_ids:
        #     return False
        # return self._is_valid_position_detection(
        #     detection, self.min_trigger_score, frame
        # )
        print("【DUMMY】跳过 _is_valid_trigger_detection")
        return False

    def _is_valid_position_detection(self, detection, min_score, frame):
        # if float(detection.score) < float(min_score):
        #     return False
        # if not math.isfinite(float(detection.depth_m)) or float(detection.depth_m) <= 0.0:
        #     return False
        # if not bool(detection.has_target_position):
        #     return False
        # if frame and detection.position_target.header.frame_id != frame:
        #     return False
        # return finite_xyz(detection.position_target.point)
        print("【DUMMY】跳过 _is_valid_position_detection")
        return False

    def _assign_roles(self, ranked_targets):
        # if self.role_mode == "split_by_y" and self.required_points == 2:
        #     sorted_by_y = sorted(ranked_targets, key=lambda item: item[1].position_target.point.y, reverse=True)
        #     positive_item = sorted_by_y[0]
        #     negative_item = sorted_by_y[1]
        #     return {
        #         self.positive_y_role: {
        #             "detection": positive_item[1],
        #             "distance": positive_item[0],
        #             "xyz": self._output_xyz(positive_item[1]),
        #         },
        #         self.negative_y_role: {
        #             "detection": negative_item[1],
        #             "distance": negative_item[0],
        #             "xyz": self._output_xyz(negative_item[1]),
        #         },
        #     }
        # role_items = {}
        # for rank, (distance, detection) in enumerate(ranked_targets):
        #     role_items[f"{self.single_role}_{rank}"] = {
        #         "detection": detection,
        #         "distance": distance,
        #         "xyz": self._output_xyz(detection),
        #     }
        # return role_items
        print("【DUMMY】跳过 _assign_roles")
        return {}

    def _output_xyz(self, detection):
        # xyz = point_xyz(detection.position_target.point)
        # if self.position_mode == "detected":
        #     return xyz
        # if self.position_mode == "detected_with_offset":
        #     return tuple(xyz[i] + self.position_offset[i] for i in range(3))
        # self.get_logger().warn(
        #     f"unsupported position_mode={self.position_mode}; using detected"
        # )
        # return xyz
        print("【DUMMY】跳过 _output_xyz")
        return (0.0, 0.0, 0.0)

    def _evaluate_stability(self):
        # if not self.use_stability:
        #     return True, "ok", "stability_disabled", self._current_points()
        # if len(self.history) < self.min_stable_frames:
        #     return (
        #         False,
        #         "collecting",
        #         f"frames:{len(self.history)}/{self.min_stable_frames}",
        #         {},
        #     )
        # roles = sorted(self.history[-1]["points"].keys())
        # stable_points = {}
        # max_std = 0.0
        # max_range = 0.0
        # for role in roles:
        #     recent_points = [
        #         frame["points"][role]["xyz"]
        #         for frame in list(self.history)[-self.min_stable_frames :]
        #         if role in frame["points"]
        #     ]
        #     if len(recent_points) < self.min_stable_frames:
        #         return (
        #             False,
        #             "incomplete",
        #             f"{role}_frames:{len(recent_points)}/{self.min_stable_frames}",
        #             {},
        #         )
        #     filtered = self._filter_outliers(recent_points)
        #     if len(filtered) < self.min_stable_frames // 2:
        #         return (
        #             False,
        #             "unstable",
        #             f"{role}_outliers:{len(filtered)}/{len(recent_points)}",
        #             {},
        #         )
        #     center = self._filter_position(filtered)
        #     std_x = axis_std(filtered, 0)
        #     std_y = axis_std(filtered, 1)
        #     std_z = axis_std(filtered, 2)
        #     role_std = max(std_x, std_y, std_z)
        #     max_std = max(max_std, role_std)
        #     range_x = axis_range(filtered, 0)
        #     range_y = axis_range(filtered, 1)
        #     range_z = axis_range(filtered, 2)
        #     role_range = max(range_x, range_y, range_z)
        #     max_range = max(max_range, role_range)
        #     stable_points[role] = {
        #         "xyz": center,
        #         "frame": self.history[-1]["frame"],
        #         "stamp": self.history[-1]["stamp"],
        #     }
        # if max_std > self.max_position_std_m:
        #     return (
        #         False,
        #         "unstable",
        #         f"std:{max_std:.4f}>{self.max_position_std_m:.4f}",
        #         stable_points,
        #     )
        # if self.max_position_range_m > 0.0 and max_range > self.max_position_range_m:
        #     return (
        #         False,
        #         "unstable",
        #         f"range:{max_range:.4f}>{self.max_position_range_m:.4f}",
        #         stable_points,
        #     )
        # return (
        #     True,
        #     "ok",
        #     f"stable:max_std={max_std:.4f},max_range={max_range:.4f}",
        #     stable_points,
        # )
        print("【DUMMY】跳过 _evaluate_stability")
        return False, "collecting", "dummy", {}

    def _filter_outliers(self, points):
        # center = tuple(median(point[axis] for point in points) for axis in range(3))
        # if self.outlier_distance_m <= 0.0:
        #     return points
        # return [
        #     point
        #     for point in points
        #     if point_distance(point, center) <= self.outlier_distance_m
        # ]
        print("【DUMMY】跳过 _filter_outliers")
        return points

    def _filter_position(self, points):
        # if self.position_filter == "median":
        #     return tuple(median(point[axis] for point in points) for axis in range(3))
        # return tuple(mean(point[axis] for point in points) for axis in range(3))
        print("【DUMMY】跳过 _filter_position")
        return (0.0, 0.0, 0.0)

    def _current_points(self):
        # if not self.history:
        #     return {}
        # return self.history[-1]["points"]
        print("【DUMMY】跳过 _current_points")
        return {}

    def _build_task(self, candidate, stable_points, quality, reason):
        # task = GraspTask()
        # task.header.stamp = candidate["stamp"]
        # task.header.frame_id = candidate["frame"]
        # task.task_name = self.task_name
        # task.grasp_mode = self.grasp_mode
        # task.target_frame = self.target_frame
        # grasp_points = []
        # for role, item in stable_points.items():
        #     grasp_points.append(
        #         self._build_grasp_point(
        #             role,
        #             item,
        #             item["xyz"],
        #             item["frame"],
        #             item["stamp"],
        #         )
        #     )
        # task.grasp_points = grasp_points
        # if candidate.get("trigger") is not None:
        #     trigger = candidate["trigger"]
        #     task.trigger_class_id = int(trigger.class_id)
        #     task.trigger_class_name = str(trigger.class_name)
        #     task.trigger_score = float(trigger.score)
        # else:
        #     task.trigger_class_id = -1
        #     task.trigger_class_name = ""
        #     task.trigger_score = 0.0
        # return task
        print("【DUMMY】跳过 _build_task")
        return GraspTask()

    def _build_grasp_point(self, role, item, xyz, frame, stamp):
        # gp = GraspPoint()
        # gp.role = str(role)
        # gp.position = PointStamped()
        # gp.position.header.frame_id = str(frame)
        # gp.position.header.stamp = stamp
        # gp.position.point.x = float(xyz[0])
        # gp.position.point.y = float(xyz[1])
        # gp.position.point.z = float(xyz[2])
        # detection = item.get("detection")
        # if detection is not None:
        #     gp.class_id = int(detection.class_id)
        #     gp.class_name = str(detection.class_name)
        #     gp.score = float(detection.score)
        # else:
        #     gp.class_id = -1
        #     gp.class_name = ""
        #     gp.score = 0.0
        # return gp
        print("【DUMMY】跳过 _build_grasp_point")
        return GraspPoint()

    @staticmethod
    def _finite_point(point):
        # return finite_xyz(point)
        print("【DUMMY】跳过 _finite_point")
        return True

    def _target_frame(self, msg):
        # return self.target_frame or msg.header.frame_id
        return self.target_frame

    def _publish_empty_or_status(self, output, reason, quality="not_ready"):
        # if self.publish_empty_tasks:
        #     self.pub_tasks.publish(output)
        # if self.pub_json is not None:
        #     self._publish_json_status(output, False, reason, quality)
        print(f"【DUMMY】发布空任务或状态: reason={reason}, quality={quality}")
        pass

    def _publish_json_status(self, output, ready, reason, quality=None):
        # if self.pub_json is None:
        #     return
        # payload = {
        #     "ready": ready,
        #     "reason": reason,
        #     "quality": quality or output.quality,
        #     "task_count": len(output.tasks),
        # }
        # self.pub_json.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        print(f"【DUMMY】跳过 _publish_json_status: ready={ready}, reason={reason}")
        pass

    def _reset_state(self):
        # self.history.clear()
        # self.ready_task_published = False
        # self._first_candidate_time = None
        print("【DUMMY】重置状态")
        self.history.clear()
        self.ready_task_published = False
        self._first_candidate_time = None
        self.dummy_detection_count = 0

    def _handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = (
            "dummy_grasp_selector enabled"
            if self.enabled
            else "dummy_grasp_selector disabled"
        )
        print(f"【DUMMY】服务调用: {response.message}")
        self.get_logger().info(response.message)
        if self.enabled:
            self._enabled_start_time = time.perf_counter()
        return response

    def _handle_reset(self, request, response):
        del request
        self._reset_state()
        response.success = True
        response.message = "dummy_grasp_selector reset"
        print(f"【DUMMY】服务调用: {response.message}")
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _yaml_safe_float(value):
        # if not math.isfinite(value):
        #     return None
        # return float(value)
        print("【DUMMY】跳过 _yaml_safe_float")
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DummyGraspSelector()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
