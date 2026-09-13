#!/usr/bin/env python3
"""Publish one bounded, read-only MOS status message for Foxglove."""

from __future__ import annotations

import os
import platform
import shutil
import socket
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


RGB_TOPICS = (
    "/head_camera/color/image_raw",
    "/left_arm_camera/color/image_raw",
    "/right_arm_camera/color/image_raw",
)
DEPTH_TOPICS = (
    "/head_camera/depth/image_raw",
    "/left_arm_camera/depth/image_raw",
    "/right_arm_camera/depth/image_raw",
)
LIDAR_TOPICS = ("/livox/lidar",)
IMU_TOPICS = ("/livox/imu",)
MAPPING_TOPICS = (
    "/cloud_registered",
    "/cloud_map",
    "/projected_map",
    "/octomap_full",
)
ODOMETRY_TOPICS = ("/odom", "/Odometry")
LOCALIZATION_TOPICS = ("/localization", "/amcl_pose")
NAVIGATION_TOPICS = (
    "/plan",
    "/global_plan",
    "/local_plan",
    "/goal_pose",
    "/navigate_to_pose/_action/status",
)

MANAGED_COMPONENTS = (
    ("vision", "视觉"),
    ("navigation", "导航"),
    ("image_relay", "图像"),
    ("status_logger", "状态"),
    ("bridge", "Bridge"),
)
RUNTIME_DIR = Path("/run/mos_fleet_monitor")

IMAGE_TOPICS = (
    ("head", "头部", "/foxglove/head_camera/image/compressed"),
    ("left", "左臂", "/foxglove/left_arm_camera/image/compressed"),
    ("right", "右臂", "/foxglove/right_arm_camera/image/compressed"),
)


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError, TypeError):
        return None


def _format_duration(total_seconds: float) -> str:
    seconds = max(0, int(total_seconds))
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, _ = divmod(seconds, 60)
    parts: list[str] = []
    if days:
        parts.append(f"{days}天")
    if hours or days:
        parts.append(f"{hours}小时")
    parts.append(f"{minutes}分钟")
    return " ".join(parts)


def _read_uptime() -> str:
    value = _read_text(Path("/proc/uptime"))
    if not value:
        return "不可用"
    try:
        return _format_duration(float(value.split()[0]))
    except (ValueError, IndexError):
        return "不可用"


def _read_memory_percent() -> float | None:
    values: dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            name, raw_value = line.split(":", 1)
            values[name] = int(raw_value.strip().split()[0])
        total = values["MemTotal"]
        available = values["MemAvailable"]
        return (total - available) / total * 100.0
    except (OSError, UnicodeError, ValueError, KeyError, ZeroDivisionError):
        return None


def _read_disk_percent() -> float | None:
    try:
        usage = shutil.disk_usage("/")
        return usage.used / usage.total * 100.0
    except (OSError, ZeroDivisionError):
        return None


def _read_load_average() -> float | None:
    try:
        return os.getloadavg()[0]
    except (AttributeError, OSError):
        return None


def _read_temperatures() -> dict[str, float]:
    temperatures: dict[str, float] = {}
    try:
        zones = list(Path("/sys/class/thermal").glob("thermal_zone*"))
    except OSError:
        return temperatures
    for zone in zones:
        sensor_type = _read_text(zone / "type")
        raw_temp = _read_text(zone / "temp")
        if not sensor_type or not raw_temp:
            continue
        try:
            value = float(raw_temp)
        except ValueError:
            continue
        if abs(value) > 1000:
            value /= 1000.0
        temperatures[sensor_type.lower()] = value
    return temperatures


def _pick_temperature(
    temperatures: dict[str, float], prefix: str
) -> float | None:
    values = [
        value for name, value in temperatures.items() if name.startswith(prefix)
    ]
    return max(values) if values else None


def _format_number(value: float | None, suffix: str = "") -> str:
    return "不可用" if value is None else f"{value:.1f}{suffix}"


def _format_memory(megabytes: float) -> str:
    if megabytes >= 1024:
        return f"{megabytes / 1024:.1f}GB"
    return f"{megabytes:.0f}MB"


def _key_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


def _diagnostic_status(
    name: str,
    message: str,
    values: list[tuple[str, object]],
    *,
    level: int = DiagnosticStatus.OK,
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.level = level
    status.name = f"MOS/{name}"
    status.message = message
    status.hardware_id = socket.gethostname()
    status.values = [_key_value(key, value) for key, value in values]
    return status


def _read_system_cpu() -> tuple[int, int] | None:
    value = _read_text(Path("/proc/stat"))
    if not value:
        return None
    try:
        fields = [int(item) for item in value.splitlines()[0].split()[1:]]
        total = sum(fields)
        idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
        return total, idle
    except (ValueError, IndexError):
        return None


def _read_process_stat(pid: int) -> tuple[str, int, int, int] | None:
    """Return state, pgrp, CPU ticks and RSS pages."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw[raw.rfind(")") + 2 :].split()
        return fields[0], int(fields[2]), int(fields[11]) + int(fields[12]), int(fields[21])
    except (OSError, UnicodeError, ValueError, IndexError):
        return None


class ResourceSampler:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest: dict[str, Any] = {
            "system_cpu_percent": None,
            "components": {},
        }
        self.previous_system: tuple[int, int] | None = None
        self.previous_component_ticks: dict[str, int] = {}
        self.previous_time: float | None = None
        self.clock_ticks = float(os.sysconf("SC_CLK_TCK"))
        self.page_size = float(os.sysconf("SC_PAGE_SIZE"))
        self.thread = threading.Thread(
            target=self._run, name="mos_resource_sampler", daemon=True
        )
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "system_cpu_percent": self.latest["system_cpu_percent"],
                "components": {
                    key: dict(value)
                    for key, value in self.latest["components"].items()
                },
            }

    def _run(self) -> None:
        while not self.stop_event.is_set():
            self._sample()
            self.stop_event.wait(2.0)

    def _sample(self) -> None:
        now = time.monotonic()
        system = _read_system_cpu()
        system_percent = None
        if system is not None and self.previous_system is not None:
            total_delta = system[0] - self.previous_system[0]
            idle_delta = system[1] - self.previous_system[1]
            if total_delta > 0:
                system_percent = max(
                    0.0, min(100.0, (total_delta - idle_delta) / total_delta * 100.0)
                )
        self.previous_system = system

        pgrp_by_component: dict[str, int] = {}
        for component, _label in MANAGED_COMPONENTS:
            value = _read_text(RUNTIME_DIR / f"{component}.pid")
            try:
                leader = int(value) if value else 0
                pgrp_by_component[component] = os.getpgid(leader)
            except (ValueError, OSError, ProcessLookupError):
                continue

        totals = {
            component: {"ticks": 0, "rss_pages": 0}
            for component in pgrp_by_component
        }
        pgrp_to_component = {
            pgrp: component for component, pgrp in pgrp_by_component.items()
        }
        try:
            process_entries = list(Path("/proc").iterdir())
        except OSError:
            process_entries = []
        for entry in process_entries:
            if not entry.name.isdigit():
                continue
            stat = _read_process_stat(int(entry.name))
            if stat is None or stat[0] == "Z":
                continue
            component = pgrp_to_component.get(stat[1])
            if component is None:
                continue
            totals[component]["ticks"] += stat[2]
            totals[component]["rss_pages"] += stat[3]

        elapsed = None if self.previous_time is None else now - self.previous_time
        components: dict[str, dict[str, float | None]] = {}
        for component, values in totals.items():
            old_ticks = self.previous_component_ticks.get(component)
            cpu_percent = None
            if old_ticks is not None and elapsed is not None and elapsed > 0:
                cpu_percent = max(
                    0.0,
                    (values["ticks"] - old_ticks)
                    / self.clock_ticks
                    / elapsed
                    * 100.0,
                )
            components[component] = {
                "cpu_percent": cpu_percent,
                "memory_mb": values["rss_pages"] * self.page_size / 1024 / 1024,
            }
        self.previous_component_ticks = {
            component: int(values["ticks"])
            for component, values in totals.items()
        }
        self.previous_time = now
        with self.lock:
            self.latest = {
                "system_cpu_percent": system_percent,
                "components": components,
            }


class MosStatusLogger(Node):
    def __init__(self) -> None:
        super().__init__("mos_status_logger")
        self.declare_parameter("interval_seconds", 2.0)
        self.declare_parameter("graph_interval_seconds", 10.0)
        self.interval = max(
            1.0, float(self.get_parameter("interval_seconds").value)
        )
        self.graph_interval = max(
            self.interval,
            float(self.get_parameter("graph_interval_seconds").value),
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            String, "/mos/monitor/status_log", output_qos
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/mos/monitor/diagnostics", output_qos
        )

        self._image_lock = threading.Lock()
        self._image_times: dict[str, deque[float]] = {
            name: deque(maxlen=300) for name, _label, _topic in IMAGE_TOPICS
        }
        self._image_age: dict[str, float | None] = {
            name: None for name, _label, _topic in IMAGE_TOPICS
        }
        self._image_subscriptions = []
        for name, _label, topic in IMAGE_TOPICS:
            subscription = self.create_subscription(
                CompressedImage,
                topic,
                lambda message, camera=name: self._on_image(camera, message),
                sensor_qos,
            )
            self._image_subscriptions.append(subscription)

        self._graph_lock = threading.Lock()
        self._graph_topics: set[str] = set()
        self._graph_nodes: list[tuple[str, str]] = []
        self._graph_ready = False
        self._stop_event = threading.Event()
        self._graph_thread = threading.Thread(
            target=self._refresh_graph_loop,
            name="mos_status_graph_reader",
            daemon=True,
        )
        self._graph_thread.start()
        self._resources = ResourceSampler()

        self.timer = self.create_timer(self.interval, self.publish_status)
        self.initial_refresh_timer = self.create_timer(
            3.0, self._publish_initial_refresh
        )
        self.publish_status()

    def stop(self) -> None:
        self._stop_event.set()
        self._resources.stop()

    def _publish_initial_refresh(self) -> None:
        self.initial_refresh_timer.cancel()
        self.publish_status()

    def _on_image(self, camera: str, message: CompressedImage) -> None:
        now_monotonic = time.monotonic()
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = None
        if stamp_ns > 0:
            candidate = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if 0.0 <= candidate <= 60.0:
                age = candidate
        with self._image_lock:
            self._image_times[camera].append(now_monotonic)
            self._image_age[camera] = age

    def _image_metrics(self) -> dict[str, dict[str, float | None]]:
        now = time.monotonic()
        result: dict[str, dict[str, float | None]] = {}
        with self._image_lock:
            for name, label, _topic in IMAGE_TOPICS:
                timestamps = [
                    value for value in self._image_times[name] if now - value <= 10.0
                ]
                age = self._image_age[name]
                fps = None
                if len(timestamps) >= 2:
                    fps = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
                result[name] = {"label": label, "fps": fps, "age": age}
        return result

    @staticmethod
    def _image_summary(metrics: dict[str, dict[str, Any]]) -> str:
        parts = []
        for name, label, _topic in IMAGE_TOPICS:
            values = metrics[name]
            fps = values["fps"]
            age = values["age"]
            if fps is None:
                parts.append(f"{label}无画面")
                continue
            age_text = "" if age is None else f"/{age:.2f}s"
            parts.append(f"{label}{fps:.1f}FPS{age_text}")
        return "｜".join(parts)

    def _refresh_graph_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                topics = {
                    name for name, _types in self.get_topic_names_and_types()
                }
                nodes = list(self.get_node_names_and_namespaces())
                with self._graph_lock:
                    self._graph_topics = topics
                    self._graph_nodes = nodes
                    self._graph_ready = True
            except Exception:
                pass
            # Reading the ROS graph is more expensive than publishing cached
            # status. Keep it independent from the smooth UI refresh rate.
            self._stop_event.wait(self.graph_interval)

    @staticmethod
    def _registered_count(
        topics: set[str], expected: tuple[str, ...]
    ) -> int:
        return sum(topic in topics for topic in expected)

    def _component_resource_summary(self) -> tuple[str, str]:
        snapshot = self._resources.snapshot()
        components = snapshot["components"]
        cpu_parts = []
        memory_parts = []
        for component, label in MANAGED_COMPONENTS:
            values = components.get(component)
            if values is None:
                continue
            cpu = values.get("cpu_percent")
            cpu_parts.append(
                f"{label} {'采样中' if cpu is None else f'{cpu:.1f}%'}"
            )
            memory_parts.append(
                f"{label} {_format_memory(float(values['memory_mb']))}"
            )
        return (
            "｜".join(cpu_parts) if cpu_parts else "无受管组件",
            "｜".join(memory_parts) if memory_parts else "无受管组件",
        )

    def publish_status(self) -> None:
        with self._graph_lock:
            topics = set(self._graph_topics)
            nodes = list(self._graph_nodes)
            graph_ready = self._graph_ready

        node_name_text = " ".join(name.lower() for name, _namespace in nodes)
        temperatures = _read_temperatures()
        rgb_count = self._registered_count(topics, RGB_TOPICS)
        depth_count = self._registered_count(topics, DEPTH_TOPICS)
        lidar_registered = any(topic in topics for topic in LIDAR_TOPICS)
        imu_registered = any(topic in topics for topic in IMU_TOPICS)
        mapping_active = any(topic in topics for topic in MAPPING_TOPICS)
        odometry_active = any(topic in topics for topic in ODOMETRY_TOPICS)
        localization_active = any(topic in topics for topic in LOCALIZATION_TOPICS)
        navigation_active = any(topic in topics for topic in NAVIGATION_TOPICS) or any(
            marker in node_name_text
            for marker in ("bt_navigator", "planner_server", "controller_server")
        )

        if graph_ready:
            ros_summary = f"节点 {len(nodes)}｜话题 {len(topics)}"
            sensor_summary = (
                f"RGB {rgb_count}/{len(RGB_TOPICS)}｜"
                f"深度 {depth_count}/{len(DEPTH_TOPICS)}｜"
                f"雷达{'有' if lidar_registered else '无'}｜"
                f"IMU{'有' if imu_registered else '无'}"
            )
            navigation_summary = (
                f"建图{'运行' if mapping_active else '停止'}｜"
                f"里程计{'运行' if odometry_active else '停止'}｜"
                f"定位{'运行' if localization_active else '停止'}｜"
                f"导航{'运行' if navigation_active else '停止'}"
            )
        else:
            ros_summary = sensor_summary = navigation_summary = "状态采集中"

        resource_snapshot = self._resources.snapshot()
        system_cpu = resource_snapshot["system_cpu_percent"]
        load_average = _read_load_average()
        memory_percent = _read_memory_percent()
        disk_percent = _read_disk_percent()
        cpu_temperature = _pick_temperature(temperatures, "cpu")
        gpu_temperature = _pick_temperature(temperatures, "gpu")
        soc_temperature = _pick_temperature(temperatures, "soc")
        image_metrics = self._image_metrics()
        image_summary = self._image_summary(image_metrics)
        component_cpu, component_memory = self._component_resource_summary()

        resource_warning = any(
            value is not None and value >= threshold
            for value, threshold in (
                (system_cpu, 90.0),
                (memory_percent, 90.0),
                (disk_percent, 90.0),
                (cpu_temperature, 85.0),
                (gpu_temperature, 85.0),
                (soc_temperature, 85.0),
            )
        )
        sensor_warning = graph_ready and (
            rgb_count < len(RGB_TOPICS)
            or depth_count < len(DEPTH_TOPICS)
            or not lidar_registered
            or not imu_registered
        )
        overall_warning = resource_warning or sensor_warning
        overall_text = "警告" if overall_warning else "正常"

        # The legacy String topic is intentionally compact. Foxglove renders
        # newlines in std_msgs/String as escaped text, so detailed information
        # belongs in the structured DiagnosticArray below.
        report = " ◆ ".join(
            (
                f"【{overall_text}】{datetime.now().astimezone():%H:%M:%S}",
                (
                    f"CPU {_format_number(system_cpu, '%')} "
                    f"内存 {_format_number(memory_percent, '%')} "
                    f"磁盘 {_format_number(disk_percent, '%')}"
                ),
                f"温度 CPU {_format_number(cpu_temperature, '℃')} GPU {_format_number(gpu_temperature, '℃')}",
                f"导航 {navigation_summary}",
                f"图像 {image_summary}",
            )
        )

        message = String()
        message.data = report
        self.publisher.publish(message)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [
            _diagnostic_status(
                "总览",
                overall_text,
                [
                    ("更新时间", f"{datetime.now().astimezone():%Y-%m-%d %H:%M:%S %Z}"),
                    ("主机", socket.gethostname()),
                    ("系统", f"{platform.system()} {platform.release()}"),
                    ("运行时长", _read_uptime()),
                ],
                level=DiagnosticStatus.WARN if overall_warning else DiagnosticStatus.OK,
            ),
            _diagnostic_status(
                "核心资源",
                "需要关注" if resource_warning else "正常",
                [
                    ("CPU使用率", _format_number(system_cpu, "%")),
                    ("1分钟负载", _format_number(load_average)),
                    ("内存使用率", _format_number(memory_percent, "%")),
                    ("磁盘使用率", _format_number(disk_percent, "%")),
                    ("CPU温度", _format_number(cpu_temperature, "℃")),
                    ("GPU温度", _format_number(gpu_temperature, "℃")),
                    ("SoC温度", _format_number(soc_temperature, "℃")),
                ],
                level=DiagnosticStatus.WARN if resource_warning else DiagnosticStatus.OK,
            ),
            _diagnostic_status(
                "导航链路",
                navigation_summary,
                [
                    ("建图", "运行" if mapping_active else "停止"),
                    ("里程计", "运行" if odometry_active else "停止"),
                    ("定位", "运行" if localization_active else "停止"),
                    ("导航", "运行" if navigation_active else "停止"),
                ],
            ),
            _diagnostic_status(
                "传感器",
                "缺失" if sensor_warning else "正常",
                [
                    ("RGB相机", f"{rgb_count}/{len(RGB_TOPICS)}"),
                    ("深度相机", f"{depth_count}/{len(DEPTH_TOPICS)}"),
                    ("雷达", "已注册" if lidar_registered else "未注册"),
                    ("IMU", "已注册" if imu_registered else "未注册"),
                ],
                level=DiagnosticStatus.WARN if sensor_warning else DiagnosticStatus.OK,
            ),
            _diagnostic_status(
                "图像预览",
                image_summary,
                [
                    item
                    for name, label, _topic in IMAGE_TOPICS
                    for item in (
                        (f"{label}帧率", _format_number(image_metrics[name]["fps"], " FPS")),
                        (f"{label}帧龄", _format_number(image_metrics[name]["age"], " s")),
                    )
                ],
            ),
            _diagnostic_status(
                "组件资源",
                "实时采样",
                [
                    ("组件CPU（单核=100%）", component_cpu),
                    ("组件内存", component_memory),
                ],
            ),
            _diagnostic_status(
                "ROS 2",
                ros_summary,
                [
                    ("节点数", len(nodes) if graph_ready else "采集中"),
                    ("话题数", len(topics) if graph_ready else "采集中"),
                ],
            ),
        ]
        self.diagnostics_publisher.publish(diagnostics)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MosStatusLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
