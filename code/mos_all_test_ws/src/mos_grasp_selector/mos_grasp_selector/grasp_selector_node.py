"""ROS2 node for selecting grasp tasks from MOS 3D detections."""

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

from mos_3d_object.msg import DetectedObject3DArray
from mos_grasp_selector.msg import GraspPoint, GraspTask, GraspTaskArray

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


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
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
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
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load grasp selector rules") from exc

    with Path(path).open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
    return data or {}


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


class MosGraspSelector(Node):
    def __init__(self):
        super().__init__("mos_grasp_selector")
        self._declare_parameters()
        self._load_parameters()

        self.history = deque(maxlen=self.stability_window)
        self.sub_detections = self.create_subscription(
            DetectedObject3DArray,
            self.detections_topic,
            self._on_detections,
            10,
        )
        self.pub_tasks = self.create_publisher(
            GraspTaskArray, self.grasp_tasks_topic, 10
        )
        self.pub_json = None
        if self.publish_json_debug:
            self.pub_json = self.create_publisher(String, self.grasp_tasks_json_topic, 10)
        self.srv_set_enabled = self.create_service(
            SetBool, "~/set_enabled", self._handle_set_enabled
        )
        self.srv_reset = self.create_service(Trigger, "~/reset", self._handle_reset)

        self.get_logger().info(
            f"started grasp selector task={self.task_name} "
            f"input={self.detections_topic} output={self.grasp_tasks_topic} "
            f"enabled={self.enabled}"
        )

    def _declare_parameters(self):
        defaults = {
            "detections_topic": "/vision/detections_3d",
            "grasp_tasks_topic": "/vision/grasp_tasks",
            "grasp_tasks_json_topic": "/vision/grasp_tasks_json",
            "publish_json_debug": True,
            "publish_empty_tasks": True,
            "publish_once_when_ready": True,
            "enabled": False,
            "active_task": "treatment_cart_A_push",
            "rules_file": "config/grasp_selector_rules.yaml",
            "class_map_file": "config/class_map.yaml",
            "task_name": "treatment_cart_A_push",
            "grasp_mode": "dual_arm_sync",
            "target_frame": "body_link",
            "require_trigger": False,
            "trigger_class_id": 3,
            "trigger_class_ids": [0, 1, 2, 3, 4, 5],
            "trigger_class_name": "治疗车 A/B/C",
            "target_class_id": 6,
            "target_class_ids": [6, 7, 8],
            "target_class_name": "A型车把手",
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
        self.target_class_ids = normalize_int_list(
            self.get_parameter("target_class_ids").value
        )
        if not self.target_class_ids:
            self.target_class_ids = [self.target_class_id]
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
                "min_stable_frames is larger than stability_window; "
                "using stability_window instead"
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
        self._load_rule_files()
        self._apply_active_task()
        self.ready_task_published = False
        self._enabled_start_time = time.perf_counter() if self.enabled else None
        self._first_candidate_time = None

    def _load_rule_files(self):
        self.class_map = {}
        self.task_rules = {}

        class_map_path = self._resolve_config_path(self.class_map_file)
        if class_map_path is not None:
            try:
                self.class_map = normalize_class_map(
                    load_yaml_file(class_map_path).get("class_map", {})
                )
            except Exception as exc:
                self.get_logger().warn(f"failed to load class_map_file: {exc}")

        rules_path = self._resolve_config_path(self.rules_file)
        if rules_path is None:
            self.get_logger().warn(
                f"grasp selector rules file not found: {self.rules_file}; "
                "using legacy parameters"
            )
            return

        try:
            data = load_yaml_file(rules_path)
        except Exception as exc:
            self.get_logger().warn(
                f"failed to load grasp selector rules: {exc}; using legacy parameters"
            )
            return

        if not self.class_map:
            self.class_map = normalize_class_map(data.get("class_map", {}))
        self.task_rules = as_dict(data.get("tasks"))

    def _resolve_config_path(self, value):
        path = Path(str(value))
        if path.is_absolute() and path.exists():
            return path
        if path.exists():
            return path

        candidates = []
        if get_package_share_directory is not None:
            try:
                candidates.append(
                    Path(get_package_share_directory("mos_grasp_selector")) / path
                )
            except Exception:
                pass
        candidates.append(Path.cwd() / path)

        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    def _apply_active_task(self):
        rule = as_dict(self.task_rules.get(self.active_task))
        if not rule:
            if self.task_rules:
                self.get_logger().warn(
                    f"active_task={self.active_task} not found; using legacy parameters"
                )
            return

        self.current_rule = rule
        self.task_name = str(rule.get("task_name", self.active_task))
        self.grasp_mode = str(rule.get("grasp_mode", self.grasp_mode))
        rule_target_class_ids = normalize_int_list(rule.get("target_class_ids", []))
        if rule_target_class_ids:
            self.target_class_ids = rule_target_class_ids
        else:
            self.target_class_ids = [
                int(rule.get("target_class_id", self.target_class_id))
            ]
        self.target_class_id = int(
            rule.get("target_class_id", self.target_class_ids[0])
        )
        self.target_class_name = str(
            rule.get("target_class_name", self.target_class_name)
        )
        self.required_points = max(1, int(rule.get("required_points", self.required_points)))
        self.publish_once_when_ready = bool(
            rule.get("publish_once_when_ready", self.publish_once_when_ready)
        )

        trigger = as_dict(rule.get("trigger"))
        self.trigger_mode = str(trigger.get("mode", "none")).lower()
        self.trigger_class_ids = normalize_int_list(trigger.get("class_ids", []))
        self.require_trigger = self.trigger_mode in {"any", "all"}

        selection = as_dict(rule.get("selection"))
        self.selection_mode = str(selection.get("mode", "nearest")).lower()
        max_distance = float(selection.get("max_distance_m", 0.0))
        self.max_target_distance_m = max_distance if max_distance > 0.0 else None
        self.min_target_score = float(selection.get("min_score", self.min_target_score))

        role = as_dict(rule.get("role"))
        self.role_mode = str(role.get("mode", self.role_mode)).lower()
        self.negative_y_role = str(role.get("negative_y", self.negative_y_role))
        self.positive_y_role = str(role.get("positive_y", self.positive_y_role))
        self.single_role = str(role.get("name", self.single_role))

        position = as_dict(rule.get("position"))
        self.position_mode = str(position.get("mode", "detected")).lower()
        self.position_offset = (
            float(position.get("offset_x", 0.0)),
            float(position.get("offset_y", 0.0)),
            float(position.get("offset_z", 0.0)),
        )

        stability = as_dict(rule.get("stability"))
        if stability:
            self.use_stability = bool(stability.get("enabled", self.use_stability))
            self.stability_window = max(
                1, int(stability.get("window", self.stability_window))
            )
            self.min_stable_frames = max(
                1, int(stability.get("min_frames", self.min_stable_frames))
            )
            if self.min_stable_frames > self.stability_window:
                self.min_stable_frames = self.stability_window
            self.position_filter = str(
                stability.get("filter", self.position_filter)
            ).lower()
            self.outlier_distance_m = max(
                0.0, float(stability.get("outlier_distance_m", self.outlier_distance_m))
            )
            self.max_position_std_m = max(
                0.0, float(stability.get("max_std_m", self.max_position_std_m))
            )
            self.max_position_range_m = max(
                0.0, float(stability.get("max_range_m", self.max_position_range_m))
            )
            self.history = deque(maxlen=self.stability_window)

        self.get_logger().info(
            f"active_task={self.task_name} target_class_id={self.target_class_id} "
            f"target_class_ids={self.target_class_ids} "
            f"required_points={self.required_points} selection={self.selection_mode} "
            f"role_mode={self.role_mode}"
        )

    def _class_display_name(self, class_id, fallback=""):
        item = self.class_map.get(int(class_id), {})
        return str(item.get("zh_name") or fallback or item.get("yolo_name") or class_id)

    def _on_detections(self, msg):
        output = GraspTaskArray()
        output.header = msg.header

        if not self.enabled:
            return

        total_t0 = time.perf_counter()
        select_t0 = time.perf_counter()
        candidate, reason = self._select_candidate(msg)
        select_t1 = time.perf_counter()
        if candidate is None:
            self.history.clear()
            self.ready_task_published = False
            self._first_candidate_time = None
            publish_t0 = time.perf_counter()
            self._publish_empty_or_status(output, reason)
            publish_t1 = time.perf_counter()
            self.get_logger().info(
                f"TIMING grasp_selector_summary ready=false reason={reason} "
                f"detections={len(msg.detections)} history={len(self.history)} "
                f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
                f"stability_ms=0.0 "
                f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
                f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
            )
            return

        now = time.perf_counter()
        if self._first_candidate_time is None:
            self._first_candidate_time = now
            enabled_elapsed = (
                now - self._enabled_start_time
                if self._enabled_start_time is not None
                else -1.0
            )
            self.get_logger().info(
                f"TIMING grasp_selector_first_candidate "
                f"since_enabled_sec={enabled_elapsed:.3f} "
                f"detections={len(msg.detections)}"
            )

        candidate_class_id = int(candidate.get("target_class_id", self.target_class_id))
        if (
            not self.ready_task_published
            and self.history
            and int(self.history[-1].get("target_class_id", candidate_class_id))
            != candidate_class_id
        ):
            # Never stabilize a pair by mixing points from different handle
            # classes when the automatic task switches candidate type.
            self.history.clear()
            self._first_candidate_time = now
        self.history.append(candidate)
        if self.publish_once_when_ready and self.ready_task_published:
            publish_t0 = time.perf_counter()
            self._publish_json_status(
                output,
                ready=True,
                reason="ready_task_already_published",
                quality="latched",
            )
            publish_t1 = time.perf_counter()
            self.get_logger().info(
                f"TIMING grasp_selector_summary ready=true reason=ready_task_already_published "
                f"quality=latched detections={len(msg.detections)} history={len(self.history)} "
                f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
                f"stability_ms=0.0 "
                f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
                f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
            )
            return

        stability_t0 = time.perf_counter()
        ready, quality, reason, stable_points = self._evaluate_stability()
        stability_t1 = time.perf_counter()
        if ready:
            output.tasks = [
                self._build_task(candidate, stable_points, quality, reason)
            ]
            publish_t0 = time.perf_counter()
            self.pub_tasks.publish(output)
            self.ready_task_published = True
            self._publish_json_status(
                output, ready=True, reason=reason, quality=quality
            )
            publish_t1 = time.perf_counter()
            ready_since_enabled = (
                publish_t1 - self._enabled_start_time
                if self._enabled_start_time is not None
                else -1.0
            )
            ready_since_first_candidate = (
                publish_t1 - self._first_candidate_time
                if self._first_candidate_time is not None
                else -1.0
            )
            self.get_logger().info(
                f"TIMING grasp_selector_ready "
                f"since_enabled_sec={ready_since_enabled:.3f} "
                f"since_first_candidate_sec={ready_since_first_candidate:.3f} "
                f"history={len(self.history)} reason={reason} quality={quality}"
            )
            self.get_logger().info(
                f"TIMING grasp_selector_summary ready=true reason={reason} quality={quality} "
                f"detections={len(msg.detections)} history={len(self.history)} "
                f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
                f"stability_ms={(stability_t1 - stability_t0) * 1000.0:.1f} "
                f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
                f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
            )
            self._first_candidate_time = None
            return

        publish_t0 = time.perf_counter()
        self._publish_empty_or_status(output, reason, quality=quality)
        publish_t1 = time.perf_counter()
        self.get_logger().info(
            f"TIMING grasp_selector_summary ready=false reason={reason} quality={quality} "
            f"detections={len(msg.detections)} history={len(self.history)} "
            f"select_ms={(select_t1 - select_t0) * 1000.0:.1f} "
            f"stability_ms={(stability_t1 - stability_t0) * 1000.0:.1f} "
            f"publish_ms={(publish_t1 - publish_t0) * 1000.0:.1f} "
            f"total_ms={(publish_t1 - total_t0) * 1000.0:.1f}"
        )

    def _select_candidate(self, msg):
        frame = self._target_frame(msg)
        trigger_candidates = [
            detection
            for detection in msg.detections
            if self._is_valid_trigger_detection(detection, frame)
        ]
        trigger_ok, trigger_reason = self._trigger_satisfied(trigger_candidates)
        if not trigger_ok:
            return None, trigger_reason

        saw_target_points = False
        saw_too_far_targets = False
        pair_candidates = []
        for target_class_id in self.target_class_ids:
            target_candidates = [
                detection
                for detection in msg.detections
                if self._is_valid_detection(
                    detection,
                    target_class_id,
                    self.min_target_score,
                    frame,
                )
            ]
            if target_candidates:
                saw_target_points = True
            if len(target_candidates) < self.required_points:
                continue

            ranked_targets = self._rank_targets(target_candidates)
            if self.max_target_distance_m is not None:
                ranked_targets = [
                    item
                    for item in ranked_targets
                    if item[0] <= self.max_target_distance_m
                ]
            if len(ranked_targets) < self.required_points:
                saw_too_far_targets = True
                continue

            selected = ranked_targets[: self.required_points]
            role_items = self._assign_roles(selected)
            pair_distance = sum(item[0] for item in selected)
            pair_max_distance = max(item[0] for item in selected)
            pair_score = sum(float(item[1].score) for item in selected)
            if self.selection_mode == "highest_score":
                pair_key = (-pair_score, pair_max_distance, pair_distance)
            else:
                pair_key = (pair_distance, pair_max_distance, -pair_score)
            pair_candidates.append(
                (
                    pair_key,
                    {
                        "header": msg.header,
                        "frame": frame,
                        "trigger": self._best_trigger(trigger_candidates),
                        "points": role_items,
                        "target_class_id": int(target_class_id),
                        "target_class_name": self._class_display_name(target_class_id),
                    },
                )
            )

        if pair_candidates:
            # Class IDs are an eligibility set, not a priority order. If more
            # than one class has a pair, select the best geometric/score pair.
            _, candidate = min(pair_candidates, key=lambda item: item[0])
            return candidate, "candidate_selected"

        if saw_too_far_targets:
            return None, "target_points_too_far"
        if saw_target_points:
            return None, "insufficient_target_points"
        return None, "no_target_class_points"

    def _trigger_satisfied(self, trigger_candidates):
        if self.trigger_mode in {"", "none", "false"}:
            return True, "trigger_disabled"
        if self.trigger_mode == "any":
            if trigger_candidates:
                return True, "trigger_any_detected"
            return False, "trigger_not_detected"
        if self.trigger_mode == "all":
            detected = {int(detection.class_id) for detection in trigger_candidates}
            missing = [
                class_id for class_id in self.trigger_class_ids if class_id not in detected
            ]
            if not missing:
                return True, "trigger_all_detected"
            return False, f"trigger_missing:{missing}"
        return False, f"unsupported_trigger_mode:{self.trigger_mode}"

    def _rank_targets(self, target_candidates):
        items = [
            (distance_from_origin(detection.position_target.point), detection)
            for detection in target_candidates
        ]
        if self.selection_mode == "highest_score":
            return sorted(items, key=lambda item: float(item[1].score), reverse=True)
        if self.selection_mode == "nearest":
            return sorted(items, key=lambda item: item[0])
        self.get_logger().warn(
            f"unsupported selection_mode={self.selection_mode}; using nearest"
        )
        return sorted(items, key=lambda item: item[0])

    @staticmethod
    def _best_trigger(trigger_candidates):
        if not trigger_candidates:
            return None
        return max(trigger_candidates, key=lambda detection: float(detection.score))

    def _is_valid_detection(self, detection, class_id, min_score, frame):
        if int(detection.class_id) != int(class_id):
            return False
        return self._is_valid_position_detection(detection, min_score, frame)

    def _is_valid_trigger_detection(self, detection, frame):
        if int(detection.class_id) not in self.trigger_class_ids:
            return False
        return self._is_valid_position_detection(
            detection, self.min_trigger_score, frame
        )

    def _is_valid_position_detection(self, detection, min_score, frame):
        if float(detection.score) < float(min_score):
            return False
        if not math.isfinite(float(detection.depth_m)) or float(detection.depth_m) <= 0.0:
            return False
        if not bool(detection.has_target_position):
            return False
        if frame and detection.position_target.header.frame_id != frame:
            return False
        return finite_xyz(detection.position_target.point)

    def _assign_roles(self, ranked_targets):
        if self.role_mode == "split_by_y" and self.required_points == 2:
            with_rank = [
                {
                    "detection": detection,
                    "distance": distance,
                    "rank": rank,
                    "xyz": self._output_xyz(detection),
                }
                for rank, (distance, detection) in enumerate(ranked_targets)
            ]
            by_y = sorted(with_rank, key=lambda item: item["xyz"][1])
            by_y[0]["role"] = self.negative_y_role
            by_y[1]["role"] = self.positive_y_role
            return {item["role"]: item for item in by_y}

        role_items = {}
        for rank, (distance, detection) in enumerate(ranked_targets):
            role = self.single_role if self.role_mode == "single" else f"target_{rank}"
            role_items[role] = {
                "role": role,
                "detection": detection,
                "distance": distance,
                "rank": rank,
                "xyz": self._output_xyz(detection),
            }
        return role_items

    def _output_xyz(self, detection):
        xyz = point_xyz(detection.position_target.point)
        if self.position_mode == "detected":
            return xyz
        if self.position_mode == "detected_with_offset":
            return tuple(xyz[index] + self.position_offset[index] for index in range(3))
        self.get_logger().warn(
            f"unsupported position_mode={self.position_mode}; using detected"
        )
        return xyz

    def _evaluate_stability(self):
        if not self.use_stability:
            return True, "ok", "stability_disabled", self._current_points()

        if len(self.history) < self.min_stable_frames:
            return (
                False,
                "collecting",
                f"collecting_stable_frames:{len(self.history)}/{self.min_stable_frames}",
                None,
            )

        roles = sorted(self.history[-1]["points"].keys())
        stable_points = {}
        max_std = 0.0
        max_range = 0.0
        for role in roles:
            points = [entry["points"][role]["xyz"] for entry in self.history]
            filtered_points, outlier_count = self._filter_outliers(points)
            if len(filtered_points) < self.min_stable_frames:
                return (
                    False,
                    "collecting",
                    f"insufficient_filtered_frames:{len(filtered_points)}/{self.min_stable_frames}",
                    None,
                )

            if any(not tuple_finite(point) for point in filtered_points):
                return False, "invalid", "non_finite_filtered_point", None

            axis_stds = [axis_std(filtered_points, axis) for axis in range(3)]
            axis_ranges = [axis_range(filtered_points, axis) for axis in range(3)]
            role_std = max(axis_stds)
            role_range = max(axis_ranges)
            max_std = max(max_std, role_std)
            max_range = max(max_range, role_range)
            xyz = self._filter_position(filtered_points)
            if not tuple_finite(xyz):
                return False, "invalid", "non_finite_stable_point", None

            stable_points[role] = {
                "xyz": xyz,
                "std_m": role_std,
                "range_m": role_range,
                "outlier_count": outlier_count,
            }

        if max_std > self.max_position_std_m:
            return (
                False,
                "unstable",
                f"position_std_too_large:{max_std:.4f}>{self.max_position_std_m:.4f}",
                stable_points,
            )

        if self.max_position_range_m > 0.0 and max_range > self.max_position_range_m:
            return (
                False,
                "unstable",
                f"position_range_too_large:{max_range:.4f}>{self.max_position_range_m:.4f}",
                stable_points,
            )

        return (
            True,
            "ok",
            f"stable:max_std={max_std:.4f},max_range={max_range:.4f}",
            stable_points,
        )

    def _filter_outliers(self, points):
        center = tuple(median(point[axis] for point in points) for axis in range(3))
        if self.outlier_distance_m <= 0.0:
            return points, 0

        filtered = [
            point
            for point in points
            if point_distance(point, center) <= self.outlier_distance_m
        ]
        return filtered, len(points) - len(filtered)

    def _filter_position(self, points):
        reducer = median if self.position_filter == "median" else mean
        return tuple(reducer(point[axis] for point in points) for axis in range(3))

    def _current_points(self):
        current = self.history[-1]
        return {
            role: {"xyz": item["xyz"], "std_m": 0.0}
            for role, item in current["points"].items()
        }

    def _build_task(self, candidate, stable_points, quality, reason):
        header = candidate["header"]
        trigger = candidate["trigger"]

        task = GraspTask()
        task.header = header
        task.task_name = self.task_name
        task.grasp_mode = self.grasp_mode
        task.target_frame = candidate["frame"]
        task.ready = True
        task.quality = quality
        task.reason = reason
        if trigger is None:
            # 未启用触发类时，trigger 字段作为占位信息。
            task.trigger_class_id = -1
            task.trigger_class_name = "unused"
            task.trigger_object_id = -1
            task.trigger_score = 0.0
            task.trigger_point = self._make_point(
                (0.0, 0.0, 0.0), candidate["frame"], header.stamp
            )
        else:
            task.trigger_class_id = int(trigger.class_id)
            task.trigger_class_name = str(trigger.class_name or self.trigger_class_name)
            task.trigger_object_id = int(trigger.object_id)
            task.trigger_score = float(trigger.score)
            task.trigger_point = self._make_point(
                point_xyz(trigger.position_target.point), candidate["frame"], header.stamp
            )
        target_class_id = int(candidate.get("target_class_id", self.target_class_id))
        target_class_name = str(
            candidate.get(
                "target_class_name",
                self._class_display_name(target_class_id, self.target_class_name),
            )
        )
        task.target_class_id = target_class_id
        task.target_class_name = target_class_name
        task.required_points = self.required_points

        points = []
        for role, item in sorted(candidate["points"].items()):
            stable = stable_points[role]
            points.append(
                self._build_grasp_point(
                    role,
                    item,
                    stable["xyz"],
                    candidate["frame"],
                    header.stamp,
                    target_class_id=target_class_id,
                    target_class_name=target_class_name,
                )
            )
        task.points = points
        return task

    def _build_grasp_point(
        self,
        role,
        item,
        xyz,
        frame,
        stamp,
        *,
        target_class_id=None,
        target_class_name="",
    ):
        detection = item["detection"]
        point = GraspPoint()
        point.header.stamp = stamp
        point.header.frame_id = frame
        point.role = role
        point.rank = int(item["rank"])
        point.class_id = int(
            self.target_class_id if target_class_id is None else target_class_id
        )
        point.class_name = target_class_name or self.target_class_name or str(
            detection.class_name
        )
        point.source_object_id = int(detection.object_id)
        point.source_score = float(detection.score)
        point.point = self._make_point(xyz, frame, stamp)
        point.distance_m = math.sqrt(sum(float(value) ** 2 for value in xyz))
        return point

    @staticmethod
    def _make_point(xyz, frame, stamp):
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = str(frame or "")
        point.point.x = float(xyz[0])
        point.point.y = float(xyz[1])
        point.point.z = float(xyz[2])
        return point

    def _target_frame(self, msg):
        return str(self.target_frame or msg.target_frame or msg.header.frame_id or "")

    def _publish_empty_or_status(self, output, reason, quality="not_ready"):
        if self.publish_empty_tasks:
            self.pub_tasks.publish(output)
        self._publish_json_status(output, ready=False, reason=reason, quality=quality)

    def _publish_json_status(self, output, ready, reason, quality=None):
        if self.pub_json is None:
            return
        payload = {
            "ready": bool(ready),
            "quality": quality,
            "reason": reason,
            "history_size": len(self.history),
            "task_count": len(output.tasks),
            "tasks": [self._task_to_dict(task) for task in output.tasks],
        }
        self.pub_json.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _reset_state(self):
        self.history.clear()
        self.ready_task_published = False
        self._first_candidate_time = None
        self._enabled_start_time = time.perf_counter() if self.enabled else None

    def _handle_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        self._reset_state()
        state = "enabled" if self.enabled else "disabled"
        response.success = True
        response.message = f"mos_grasp_selector {state}"
        self.get_logger().info(response.message)
        self.get_logger().info(
            f"TIMING grasp_selector_enabled enabled={str(self.enabled).lower()}"
        )
        return response

    def _handle_reset(self, request, response):
        del request
        self._reset_state()
        response.success = True
        response.message = "mos_grasp_selector reset"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _task_to_dict(task):
        return {
            "task_name": task.task_name,
            "grasp_mode": task.grasp_mode,
            "target_frame": task.target_frame,
            "ready": bool(task.ready),
            "quality": task.quality,
            "reason": task.reason,
            "trigger_class_id": int(task.trigger_class_id),
            "target_class_id": int(task.target_class_id),
            "points": [
                {
                    "role": point.role,
                    "rank": int(point.rank),
                    "class_id": int(point.class_id),
                    "source_object_id": int(point.source_object_id),
                    "score": float(point.source_score),
                    "x": float(point.point.point.x),
                    "y": float(point.point.point.y),
                    "z": float(point.point.point.z),
                    "distance_m": float(point.distance_m),
                }
                for point in task.points
            ],
        }


def main(args=None):
    rclpy.init(args=args)
    node = MosGraspSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
