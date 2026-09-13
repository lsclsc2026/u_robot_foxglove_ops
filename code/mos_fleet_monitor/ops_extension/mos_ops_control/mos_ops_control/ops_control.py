#!/usr/bin/env python3
"""Foxglove teleop gate with optional isolation of existing motion sources."""

from __future__ import annotations

import math
import os
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class OpsControl(Node):
    MOTION_SOURCE_PREFIXES = (
        b"bash /root/work_space/mos_fleet_monitor/bin/_supervise_component.sh navigation",
        b"/usr/bin/python3 /opt/ros/humble/bin/ros2 launch lumos_nav nav2_fastlio_bringup.launch.py",
        b"/bin/bash /usr/local/bin/mos_slam_mapping",
        b"/bin/bash /usr/local/bin/mos_slam_keyboard_teleop",
        b"/usr/bin/python3 /opt/ros/humble/bin/ros2 run lumos_nav keyboard_teleop_node",
        b"/opt/slam/apps/install/lib/lumos_nav/keyboard_teleop_node",
        b"/opt/slam/apps/install/lib/fast_gicp/gicp_localization",
        b"/opt/slam/apps/install/lib/fast_lio/fastlio_mapping",
        b"/tmp/mos_all_test_ws/install/nav2_controller/lib/nav2_controller/controller_server",
        b"/tmp/mos_all_test_ws/install/nav2_planner/lib/nav2_planner/planner_server",
        b"/tmp/mos_all_test_ws/install/nav2_bt_navigator/lib/nav2_bt_navigator/bt_navigator",
        b"/tmp/mos_all_test_ws/install/nav2_velocity_smoother/lib/nav2_velocity_smoother/velocity_smoother",
    )

    def __init__(self) -> None:
        super().__init__("mos_ops_control")
        self.declare_parameter("input_topic", "/foxglove/teleop/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "/foxglove/ops/teleop_enable")
        self.declare_parameter("disable_topic", "/foxglove/ops/teleop_disable")
        self.declare_parameter("status_topic", "/foxglove/ops/status")
        self.declare_parameter(
            "teleop_state_file", "/run/mos_fleet_monitor/teleop_enabled"
        )
        self.declare_parameter(
            "frozen_groups_file", "/run/mos_fleet_monitor/ops_frozen_groups"
        )
        self.declare_parameter("max_linear_x", 0.10)
        self.declare_parameter("max_angular_z", 0.30)
        self.declare_parameter("command_timeout", 0.50)

        self._state_file = Path(str(self.get_parameter("teleop_state_file").value))
        self._groups_file = Path(str(self.get_parameter("frozen_groups_file").value))
        self._max_linear = float(self.get_parameter("max_linear_x").value)
        self._max_angular = float(self.get_parameter("max_angular_z").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        for name, value in (
            ("max_linear_x", self._max_linear),
            ("max_angular_z", self._max_angular),
            ("command_timeout", self._command_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

        self._enabled = False
        self._frozen_groups: set[int] = set()
        self._last_command_at = 0.0
        self._last_graph_check_at = 0.0
        self._status = "摇操已锁定"

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos
        )
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_command, 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self._on_enable,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("disable_topic").value),
            self._on_disable,
            10,
        )
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.50, self._publish_status)

        self._recover_stale_freeze()
        self._write_enabled(False)
        self._send_stop(repeat=8)
        self._publish_status()
        self.get_logger().info("Foxglove摇操解锁控制已启动")

    def _write_enabled(self, enabled: bool) -> None:
        self._state_file.write_text("1\n" if enabled else "0\n", encoding="utf-8")

    def _set_status(self, text: str) -> None:
        self._status = text
        self._publish_status()

    def _motion_source_groups(self) -> set[int]:
        groups: set[int] = set()
        own_group = os.getpgrp()
        for proc_dir in Path("/proc").iterdir():
            if not proc_dir.name.isdigit():
                continue
            pid = int(proc_dir.name)
            try:
                cmdline = (proc_dir / "cmdline").read_bytes().replace(b"\0", b" ").strip()
                if not any(cmdline.startswith(prefix) for prefix in self.MOTION_SOURCE_PREFIXES):
                    continue
                pgid = os.getpgid(pid)
            except OSError:
                continue
            if pgid > 1 and pgid != own_group:
                groups.add(pgid)
        return groups

    def _save_frozen_groups(self) -> None:
        text = "".join(f"{pgid}\n" for pgid in sorted(self._frozen_groups))
        self._groups_file.write_text(text, encoding="utf-8")

    def _recover_stale_freeze(self) -> None:
        try:
            groups = {
                int(line)
                for line in self._groups_file.read_text(encoding="utf-8").splitlines()
                if line.strip()
            }
        except (OSError, ValueError):
            groups = set()
        for pgid in groups:
            if pgid > 1:
                try:
                    os.killpg(pgid, signal.SIGCONT)
                except OSError:
                    pass
        self._groups_file.unlink(missing_ok=True)

    def _freeze_motion_sources(self) -> bool:
        groups = self._motion_source_groups()
        stopped: set[int] = set()
        try:
            for pgid in groups:
                os.killpg(pgid, signal.SIGSTOP)
                stopped.add(pgid)
        except OSError:
            for pgid in stopped:
                try:
                    os.killpg(pgid, signal.SIGCONT)
                except OSError:
                    pass
            self._set_status("解锁失败：无法隔离原运动控制")
            return False
        self._frozen_groups = stopped
        self._save_frozen_groups()
        return True

    def _resume_motion_sources(self) -> None:
        for pgid in self._frozen_groups:
            try:
                os.killpg(pgid, signal.SIGCONT)
            except OSError:
                pass
        self._frozen_groups.clear()
        self._groups_file.unlink(missing_ok=True)

    def _on_enable(self, _message: Bool) -> None:
        if self._enabled:
            if self._frozen_groups:
                self._set_status("摇操已解锁（原运动已暂停）")
            else:
                self._set_status("摇操已解锁（未检测到导航）")
            return
        if not self._freeze_motion_sources():
            self._enabled = False
            self._write_enabled(False)
            self._send_stop(repeat=10)
            return
        self._send_stop(repeat=10)
        self._enabled = True
        self._write_enabled(True)
        if self._frozen_groups:
            self._set_status("摇操已解锁（原运动已暂停）")
        else:
            self._set_status("摇操已解锁（未检测到导航）")

    def _on_disable(self, _message: Bool) -> None:
        self._enabled = False
        self._last_command_at = 0.0
        self._write_enabled(False)
        self._send_stop(repeat=10)
        had_sources = bool(self._frozen_groups)
        self._resume_motion_sources()
        self._send_stop(repeat=5)
        if had_sources:
            self._set_status("摇操已锁定（原运动已恢复）")
        else:
            self._set_status("摇操已锁定")

    def _on_command(self, message: Twist) -> None:
        if not self._enabled:
            return
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._on_disable(Bool())
            self._set_status("摇操已锁定：收到无效速度指令")
            return
        command = Twist()
        command.linear.x = max(-self._max_linear, min(self._max_linear, linear))
        command.angular.z = max(-self._max_angular, min(self._max_angular, angular))
        self._cmd_pub.publish(command)
        self._last_command_at = time.monotonic()
        if abs(command.linear.x) > 1e-6 or abs(command.angular.z) > 1e-6:
            self._set_status("人工摇操中")
        else:
            self._set_status("摇操已解锁（已停车）")

    def _send_stop(self, repeat: int = 1) -> None:
        for _ in range(repeat):
            self._cmd_pub.publish(Twist())

    def _watchdog(self) -> None:
        now = time.monotonic()
        if now - self._last_graph_check_at >= 1.0:
            self._last_graph_check_at = now
            controller_present = any(
                endpoint.node_name == "mos_hardware_controller"
                for endpoint in self.get_subscriptions_info_by_topic("/cmd_vel")
            )
            if self._enabled and not controller_present:
                self._on_disable(Bool())
                self._set_status("摇操已锁定：底盘控制器已断开")
                return
        if (
            self._enabled
            and self._last_command_at > 0.0
            and now - self._last_command_at > self._command_timeout
        ):
            self._send_stop(repeat=5)
            self._last_command_at = 0.0
            self._set_status("摇操已解锁（已自动停车）")

    def _publish_status(self) -> None:
        message = String()
        message.data = self._status
        self._status_pub.publish(message)

    def shutdown(self) -> None:
        self._enabled = False
        self._write_enabled(False)
        if rclpy.ok():
            self._send_stop(repeat=10)
        self._resume_motion_sources()


def main() -> None:
    rclpy.init()
    node = OpsControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
