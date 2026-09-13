from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Callable

from .alarm import alarm_to_json, make_alarm
from .config import load_yaml, runtime_path
from .protection import load_state as load_protection_state
from .storage import atomic_write_json, now_iso, read_json
from .system_reader import default_gateway, disk_info, iter_files, ping, run_readonly

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
except ImportError:
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]


class Watchdog(Node):
    def __init__(self) -> None:
        super().__init__("mos_ops_watchdog")
        self.config = load_yaml("watchdog.yaml")
        self.started_monotonic = time.monotonic()
        self.startup_grace = float(self.config.get("startup_grace_seconds", 20))
        self.topic_seen: dict[str, float] = {}
        self.conditions: dict[str, dict[str, Any]] = {}
        self.subscriptions_dynamic: list[Any] = []
        self.last_motion_position: tuple[float, float] | None = None
        self.motion_reference_position: tuple[float, float] | None = None
        self.motion_started: float | None = None
        self.commanded_moving = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.alarm_publisher = self.create_publisher(
            String, "/mos/ops/alarm_input", qos
        )
        self._create_topic_watchers()
        self._create_motion_watchers()
        self._initialize_file_watch()
        interval = max(1.0, float(self.config.get("interval_seconds", 5)))
        self.timer = self.create_timer(interval, self._check)

    def _create_topic_watchers(self) -> None:
        topics = self.config.get("topics", {})
        if not isinstance(topics, dict):
            return
        for topic, settings in topics.items():
            if not isinstance(settings, dict):
                continue
            type_name = settings.get("type")
            try:
                message_type = get_message(str(type_name))
                subscription = self.create_subscription(
                    message_type,
                    str(topic),
                    lambda _message, name=str(topic): self.topic_seen.__setitem__(
                        name, time.monotonic()
                    ),
                    qos_profile_sensor_data,
                )
                self.subscriptions_dynamic.append(subscription)
            except Exception as exc:
                self.get_logger().warning(
                    f"无法订阅 {topic} ({type_name})，将按缺失处理：{exc}"
                )

    def _create_motion_watchers(self) -> None:
        settings = self.config.get("motion_stall", {})
        if not isinstance(settings, dict) or not settings.get("enabled", True):
            return
        try:
            self.cmd_subscription = self.create_subscription(
                Twist,
                str(settings.get("cmd_topic", "/cmd_vel")),
                self._on_cmd_vel,
                qos_profile_sensor_data,
            )
            self.odom_subscription = self.create_subscription(
                Odometry,
                str(settings.get("odometry_topic", "/Odometry")),
                self._on_odometry,
                qos_profile_sensor_data,
            )
        except Exception as exc:
            self.get_logger().warning(f"无法创建运动卡死检测订阅：{exc}")

    def _on_cmd_vel(self, message: Any) -> None:
        settings = self.config.get("motion_stall", {})
        threshold = float(settings.get("command_threshold", 0.03))
        moving = (
            abs(float(message.linear.x)) >= threshold
            or abs(float(message.linear.y)) >= threshold
            or abs(float(message.angular.z)) >= threshold
        )
        if moving and not self.commanded_moving:
            self.motion_started = time.monotonic()
            self.motion_reference_position = self.last_motion_position
        if not moving:
            self.motion_started = None
            self.motion_reference_position = self.last_motion_position
        self.commanded_moving = moving

    def _on_odometry(self, message: Any) -> None:
        position = message.pose.pose.position
        self.last_motion_position = (float(position.x), float(position.y))

    def _initialize_file_watch(self) -> None:
        roots = [str(item) for item in self.config.get("file_watch", [])]
        files, _errors, _sizes = iter_files(roots, max_files_per_directory=1000)
        baseline_path = runtime_path("watchdog_file_baseline.json")
        previous = read_json(baseline_path, {})
        known = previous.get("files", {}) if isinstance(previous, dict) else {}
        self.new_crash_files = {
            path for path in files if path not in known
        } if known else set()
        atomic_write_json(
            baseline_path,
            {"schema_version": 1, "updated_at": now_iso(), "files": files},
        )

    def _publish_alarm(self, alarm: dict[str, Any]) -> None:
        message = String()
        message.data = alarm_to_json(alarm)
        self.alarm_publisher.publish(message)

    def _condition(
        self,
        alarm_id: str,
        active: bool,
        *,
        severity: str,
        item: str,
        advice: str,
        onsite_required: bool = False,
        details: dict[str, Any] | None = None,
    ) -> None:
        protection = load_protection_state().get("state", "NORMAL")
        alarm = make_alarm(
            alarm_id,
            active=active,
            severity=severity,
            source="watchdog",
            item=item,
            protection_state=str(protection),
            advice=advice,
            onsite_required=onsite_required,
            details=details,
        )
        semantic = json.dumps(
            {
                key: value
                for key, value in alarm.items()
                if key not in ("timestamp", "details")
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        old = self.conditions.get(alarm_id)
        if old and old["semantic"] == semantic:
            return
        # 启动后第一次观察到“正常”无需发送清除消息。
        if not active and old is None:
            self.conditions[alarm_id] = {"active": False, "semantic": semantic}
            return
        self.conditions[alarm_id] = {"active": active, "semantic": semantic}
        self._publish_alarm(alarm)

    def _check_topics(self, graph_topics: set[str], now: float) -> None:
        in_grace = now - self.started_monotonic < self.startup_grace
        topics = self.config.get("topics", {})
        if not isinstance(topics, dict):
            return
        for topic, settings in topics.items():
            if not isinstance(settings, dict):
                continue
            required = bool(settings.get("required", False))
            severity = str(settings.get("severity", "warning"))
            max_age = float(settings.get("max_age_seconds", 5))
            last_seen = self.topic_seen.get(str(topic))
            registered = str(topic) in graph_topics
            expected_now = required or registered or last_seen is not None
            stale = (
                not in_grace
                and expected_now
                and (
                    not registered
                    or last_seen is None
                    or now - last_seen > max_age
                )
            )
            age = None if last_seen is None else round(now - last_seen, 2)
            self._condition(
                f"topic_stale:{topic}",
                stale,
                severity=severity,
                item=f"关键话题 {topic} 未持续收到数据",
                advice="检查对应传感器、发布节点和 ROS_DOMAIN_ID",
                onsite_required=False,
                details={"registered": registered, "age_seconds": age},
            )

    def _check_nodes(self, graph_nodes: set[str], now: float) -> None:
        if now - self.started_monotonic < self.startup_grace:
            return
        required = self.config.get("critical_nodes", {}).get("required", [])
        for node in required:
            missing = str(node) not in graph_nodes
            self._condition(
                f"node_missing:{node}",
                missing,
                severity="critical",
                item=f"关键 ROS 2 节点 {node} 不存在",
                advice="查看对应 launch 日志；不要由 Watchdog 自动重启",
            )

    def _check_disk(self) -> None:
        disk = disk_info("/")
        if not disk.get("ok"):
            self._condition(
                "disk_read_failed",
                True,
                severity="warning",
                item="无法读取根分区使用率",
                advice="人工执行 mos-maintain 查看详细错误",
                details=disk,
            )
            return
        self._condition(
            "disk_read_failed",
            False,
            severity="warning",
            item="根分区读取恢复",
            advice="无需操作",
        )
        percent = float(disk["used_percent"])
        critical = float(self.config.get("disk_critical_percent", 95))
        warning = float(self.config.get("disk_warning_percent", 85))
        self._condition(
            "disk_usage",
            percent >= warning,
            severity="critical" if percent >= critical else "warning",
            item="根分区使用率超过告警阈值",
            advice="检查大文件和日志；维护脚本只读，不会自动删除",
            onsite_required=percent >= critical,
            details=disk,
        )

    def _check_processes(self) -> None:
        if time.monotonic() - self.started_monotonic < self.startup_grace:
            return
        markers = [
            str(item)
            for item in self.config.get("critical_process_markers", [])
            if str(item)
        ]
        if not markers:
            return
        result = run_readonly(["ps", "-eo", "args="], timeout=3)
        output = result.get("stdout", "")
        for marker in markers:
            self._condition(
                f"process_missing:{marker}",
                not result.get("ok") or marker not in output,
                severity="critical",
                item=f"关键进程标记 {marker} 不存在",
                advice="检查受管启动器和对应日志；Watchdog 不自动重启",
                details={"ps_error": result.get("stderr", "")},
            )

    def _check_files(self) -> None:
        roots = [str(item) for item in self.config.get("file_watch", [])]
        files, errors, _sizes = iter_files(roots, max_files_per_directory=1000)
        current = set(files)
        self.new_crash_files.intersection_update(current)
        baseline_path = runtime_path("watchdog_file_baseline.json")
        baseline = read_json(baseline_path, {})
        known = set(baseline.get("files", {})) if isinstance(baseline, dict) else set()
        self.new_crash_files.update(current - known)
        atomic_write_json(
            baseline_path,
            {"schema_version": 1, "updated_at": now_iso(), "files": files},
        )
        paths = sorted(self.new_crash_files)
        self._condition(
            "new_crash_files",
            bool(paths),
            severity="critical",
            item=f"发现 {len(paths)} 个新 core/crash 文件",
            advice="运行 mos-maintain 记录详情并分析崩溃原因",
            details={"files": paths[:20], "scan_errors": errors},
        )

    def _check_network(self) -> None:
        settings = self.config.get("network", {})
        timeout = int(settings.get("ping_timeout_seconds", 2))
        if settings.get("gateway_enabled", True):
            gateway = default_gateway()
            host = gateway.get("gateway")
            result = ping(str(host), timeout) if host else {"ok": False}
            self._condition(
                "gateway_unreachable",
                not bool(result.get("ok")),
                severity="warning",
                item=f"默认网关 {host or '未知'} 不可达",
                advice="检查网线、交换机、IP 和默认路由",
                details={"gateway": gateway, "ping": result},
            )
        remote = str(settings.get("remote_host", "")).strip()
        if remote:
            result = ping(remote, timeout)
            self._condition(
                "remote_unreachable",
                not bool(result.get("ok")),
                severity="warning",
                item=f"远程运维地址 {remote} 不可达",
                advice="检查远程网络链路；不自动修改网络参数",
                details=result,
            )

    def _check_motion(self, now: float) -> None:
        settings = self.config.get("motion_stall", {})
        if not settings.get("enabled", True):
            return
        timeout = float(settings.get("timeout_seconds", 8))
        threshold = float(settings.get("movement_threshold_meters", 0.02))
        stalled = False
        distance = None
        if (
            self.commanded_moving
            and self.motion_started is not None
            and now - self.motion_started >= timeout
            and self.motion_reference_position is not None
            and self.last_motion_position is not None
        ):
            distance = math.dist(
                self.motion_reference_position, self.last_motion_position
            )
            stalled = distance < threshold
            if not stalled:
                self.motion_started = now
                self.motion_reference_position = self.last_motion_position
        self._condition(
            "motion_stall",
            stalled,
            severity="critical",
            item=f"持续有速度指令但 {timeout:g} 秒内位移不足",
            advice="人工确认障碍、定位和底盘状态，必要时执行 P1 base/navigation",
            onsite_required=True,
            details={"observed_distance_meters": distance},
        )

    def _check_protection(self) -> None:
        state = load_protection_state()
        level = str(state.get("state", "NORMAL"))
        active = level != "NORMAL"
        complete = bool(state.get("protection_complete", True))
        self._condition(
            "protection_state",
            active,
            severity={"P2": "warning", "P1": "critical", "P0": "emergency"}.get(
                level, "warning"
            ),
            item=(
                f"当前处于 {level} 保护"
                + ("" if complete else "，且部分保护动作未确认成功")
            ),
            advice="按保护状态文件和现场检查结果决定是否恢复",
            onsite_required=level == "P0",
            details={"state": state},
        )

    def _check(self) -> None:
        now = time.monotonic()
        try:
            graph_topics = {name for name, _types in self.get_topic_names_and_types()}
            graph_nodes = {
                f"{namespace.rstrip('/')}/{name}".replace("//", "/")
                for name, namespace in self.get_node_names_and_namespaces()
            }
            self._check_topics(graph_topics, now)
            self._check_nodes(graph_nodes, now)
        except Exception as exc:
            self.get_logger().warning(f"ROS Graph 检查失败：{exc}")
        checks: tuple[Callable[[], None], ...] = (
            self._check_disk,
            self._check_processes,
            self._check_files,
            self._check_network,
            lambda: self._check_motion(now),
            self._check_protection,
        )
        for check in checks:
            try:
                check()
            except Exception as exc:
                self.get_logger().warning(f"Watchdog 子检查失败：{exc}")
        atomic_write_json(
            runtime_path("watchdog_state.json"),
            {
                "schema_version": 1,
                "updated_at": now_iso(),
                "auto_protect_enabled": bool(
                    self.config.get("auto_protect", {}).get("enabled", False)
                ),
                "conditions": self.conditions,
                "note": "Watchdog 只发布告警，不执行控制命令。",
            },
        )


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("当前 Python 环境未安装 ROS 2 rclpy")
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
