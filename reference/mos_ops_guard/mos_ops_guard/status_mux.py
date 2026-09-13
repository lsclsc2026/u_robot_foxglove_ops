from __future__ import annotations

import json
from typing import Any

from .alarm import format_alarm, highest_alarm, parse_alarm
from .config import runtime_path
from .storage import atomic_write_json, now_iso, read_json

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String
except ImportError:  # 允许在没有 ROS 2 的开发机上导入并测试纯逻辑。
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]


def alarm_key(alarm: dict[str, Any]) -> str:
    return f"{alarm.get('source', 'unknown')}:{alarm.get('alarm_id', 'unknown')}"


def _semantic_alarm(alarm: dict[str, Any]) -> str:
    value = {
        key: item
        for key, item in alarm.items()
        if key not in ("timestamp", "details")
    }
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


class AlarmLatch:
    def __init__(self, initial: dict[str, dict[str, Any]] | None = None) -> None:
        self.active: dict[str, dict[str, Any]] = initial or {}

    def update(self, alarm: dict[str, Any]) -> tuple[bool, bool]:
        """Return (changed, became_clear)."""
        key = alarm_key(alarm)
        was_active = bool(self.active)
        old = self.active.get(key)
        if alarm.get("active"):
            if old and _semantic_alarm(old) == _semantic_alarm(alarm):
                return False, False
            self.active[key] = alarm
            return True, False
        if key not in self.active:
            return False, False
        del self.active[key]
        return True, was_active and not self.active


class StatusMux(Node):
    def __init__(self) -> None:
        super().__init__("mos_ops_status_mux")
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state = read_json(runtime_path("latest_alarm.json"), {})
        initial = state.get("active_alarms", {}) if isinstance(state, dict) else {}
        if not isinstance(initial, dict):
            initial = {}
        self.latch = AlarmLatch(initial)
        self.latest_normal = ""
        self.status_publisher = self.create_publisher(
            String, "/mos/monitor/status_log", output_qos
        )
        self.alarm_publisher = self.create_publisher(
            String, "/mos/monitor/alarm", output_qos
        )
        self.create_subscription(
            String, "/mos/monitor/status_normal", self._on_normal, output_qos
        )
        self.create_subscription(
            String, "/mos/ops/alarm_input", self._on_alarm, input_qos
        )
        self.initial_timer = self.create_timer(1.0, self._publish_initial_state)

    def _publish(self, publisher: Any, content: str) -> None:
        message = String()
        message.data = content
        publisher.publish(message)

    def _persist(self) -> None:
        highest = highest_alarm(self.latch.active.values())
        atomic_write_json(
            runtime_path("latest_alarm.json"),
            {
                "schema_version": 1,
                "active": bool(self.latch.active),
                "active_count": len(self.latch.active),
                "highest_alarm": highest,
                "active_alarms": self.latch.active,
                "updated_at": now_iso(),
            },
        )

    def _publish_current_alarm(self) -> None:
        highest = highest_alarm(self.latch.active.values())
        if highest is None:
            return
        text = format_alarm(highest, len(self.latch.active))
        self._publish(self.status_publisher, text)
        self._publish(self.alarm_publisher, text)

    def _publish_initial_state(self) -> None:
        self.initial_timer.cancel()
        if self.latch.active:
            self._publish_current_alarm()

    def _on_normal(self, message: Any) -> None:
        self.latest_normal = message.data
        if not self.latch.active:
            self._publish(self.status_publisher, message.data)

    def _on_alarm(self, message: Any) -> None:
        try:
            alarm = parse_alarm(message.data)
        except (ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"忽略无效告警消息：{exc}")
            return
        changed, became_clear = self.latch.update(alarm)
        if not changed:
            return
        self._persist()
        if self.latch.active:
            self._publish_current_alarm()
            return
        if became_clear:
            recovery = "\n".join(
                (
                    "【MOS异常已解除】",
                    f"时间：{now_iso()}",
                    f"来源：{alarm.get('source', '未知')}",
                    f"已解除：{alarm.get('item', alarm.get('alarm_id', '未知'))}",
                    "状态：恢复正常摘要转发，机器人不会自动恢复旧任务。",
                )
            )
            self._publish(self.status_publisher, recovery)
            self._publish(self.alarm_publisher, recovery)
            if self.latest_normal:
                self._publish(self.status_publisher, self.latest_normal)


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("当前 Python 环境未安装 ROS 2 rclpy")
    rclpy.init(args=args)
    node = StatusMux()
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
