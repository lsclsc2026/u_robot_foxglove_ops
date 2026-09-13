#!/usr/bin/env python3
"""Guarded Foxglove teleoperation for the Lumos differential-drive base.

The node starts stationary and requires a short-lived arming message before it
accepts velocity commands.  It talks to MosHal directly so navigation can keep
publishing maps and point clouds without its /cmd_vel output reaching the base.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from mos_hal import ChasisControlMode, JointCmd, MosHal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class FoxgloveTeleopGuard(Node):
    """Apply arming, speed limits and a command watchdog before MosHal."""

    def __init__(self) -> None:
        super().__init__("foxglove_teleop_guard")

        self.declare_parameter("input_topic", "/foxglove/teleop/cmd_vel")
        self.declare_parameter("enable_topic", "/foxglove/teleop/enable")
        self.declare_parameter("status_topic", "/foxglove/teleop/status")
        self.declare_parameter("max_linear_x", 0.10)
        self.declare_parameter("max_angular_z", 0.30)
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("arm_timeout", 30.0)
        self.declare_parameter("wheelbase", 0.54474)
        self.declare_parameter("inverse_wheel_radius", 11.11)
        self.declare_parameter("hal_config", "/opt/lumos/config/mos_hal/mos_p15.yaml")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._enable_topic = str(self.get_parameter("enable_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._max_linear = float(self.get_parameter("max_linear_x").value)
        self._max_angular = float(self.get_parameter("max_angular_z").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._arm_timeout = float(self.get_parameter("arm_timeout").value)
        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._inverse_wheel_radius = float(
            self.get_parameter("inverse_wheel_radius").value
        )
        hal_config = str(self.get_parameter("hal_config").value)

        for name, value in (
            ("max_linear_x", self._max_linear),
            ("max_angular_z", self._max_angular),
            ("command_timeout", self._command_timeout),
            ("arm_timeout", self._arm_timeout),
            ("wheelbase", self._wheelbase),
            ("inverse_wheel_radius", self._inverse_wheel_radius),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

        self._armed_until = 0.0
        self._last_command_at = 0.0
        self._last_graph_check_at = 0.0
        self._moving = False
        self._fault = ""
        self._state = "DISARMED"
        self._hal_started = False

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            String, self._status_topic, status_qos
        )
        self.create_subscription(Bool, self._enable_topic, self._on_enable, 1)
        self.create_subscription(Twist, self._input_topic, self._on_command, 1)

        self._hal = MosHal(hal_config)
        self._hal.set_chasis_control_mode(ChasisControlMode.PI)
        if not self._hal.start():
            raise RuntimeError("MosHal.start() failed")
        self._hal_started = True
        self._send_stop()

        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.50, self._publish_status)
        self.get_logger().info(
            "Foxglove teleop ready but DISARMED; "
            f"limits={self._max_linear:.2f}m/s,{self._max_angular:.2f}rad/s "
            f"watchdog={self._command_timeout:.2f}s lease={self._arm_timeout:.1f}s"
        )
        self._publish_status()

    def _cmd_vel_subscriber_conflicts(self) -> list[str]:
        conflicts = []
        for endpoint in self.get_subscriptions_info_by_topic("/cmd_vel"):
            namespace = endpoint.node_namespace.rstrip("/")
            full_name = f"{namespace}/{endpoint.node_name}" or "/"
            conflicts.append(full_name)
        return sorted(set(conflicts))

    def _on_enable(self, message: Bool) -> None:
        now = time.monotonic()
        if not message.data:
            self._disarm("DISARMED")
            return

        conflicts = self._cmd_vel_subscriber_conflicts()
        if conflicts:
            self._fault = "existing /cmd_vel subscriber(s): " + ", ".join(conflicts)
            self._disarm("CONTROL_CONFLICT")
            self.get_logger().error(
                "Refusing teleop arm because another chassis controller is active: "
                + ", ".join(conflicts)
            )
            return

        self._fault = ""
        self._armed_until = now + self._arm_timeout
        self._last_command_at = 0.0
        self._state = "ARMED_WAITING_FOR_COMMAND"
        self._send_stop()
        self.get_logger().warn(
            f"Teleop armed for at most {self._arm_timeout:.1f}s; robot remains stopped "
            "until fresh commands arrive"
        )
        self._publish_status()

    def _on_command(self, message: Twist) -> None:
        now = time.monotonic()
        if now >= self._armed_until:
            if self._moving:
                self._send_stop()
            return

        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._fault = "non-finite velocity command rejected"
            self._disarm("INVALID_COMMAND")
            return

        linear = max(-self._max_linear, min(self._max_linear, linear))
        angular = max(-self._max_angular, min(self._max_angular, angular))
        self._send_velocity(linear, angular)
        self._last_command_at = now
        self._moving = abs(linear) > 1e-6 or abs(angular) > 1e-6
        self._state = "ACTIVE" if self._moving else "ARMED_STOPPED"

    def _send_velocity(self, linear_x: float, angular_z: float) -> None:
        left_linear = linear_x - 0.5 * self._wheelbase * angular_z
        right_linear = linear_x + 0.5 * self._wheelbase * angular_z

        command = JointCmd()
        command.name = ["right_wheel_joint", "left_wheel_joint"]
        command.velocity = [
            -self._inverse_wheel_radius * right_linear,
            self._inverse_wheel_radius * left_linear,
        ]
        command.position = [0.0, 0.0]
        command.torque = [0.0, 0.0]
        self._hal.setRobotCmd(command)

    def _send_stop(self) -> None:
        if not self._hal_started:
            return
        self._send_velocity(0.0, 0.0)
        self._moving = False

    def _disarm(self, state: str) -> None:
        self._send_stop()
        self._armed_until = 0.0
        self._last_command_at = 0.0
        self._state = state
        self._publish_status()

    def _watchdog(self) -> None:
        now = time.monotonic()
        if self._armed_until > 0.0 and now >= self._armed_until:
            self._fault = "arming lease expired"
            self._disarm("LEASE_EXPIRED")
            return

        if (
            self._armed_until > now
            and self._last_command_at > 0.0
            and now - self._last_command_at > self._command_timeout
        ):
            self._fault = "command watchdog expired"
            self._send_stop()
            self._last_command_at = 0.0
            self._state = "COMMAND_TIMEOUT"
            self._publish_status()

        if now - self._last_graph_check_at >= 1.0:
            self._last_graph_check_at = now
            conflicts = self._cmd_vel_subscriber_conflicts()
            if conflicts and self._armed_until > now:
                self._fault = "existing /cmd_vel subscriber(s): " + ", ".join(conflicts)
                self._disarm("CONTROL_CONFLICT")

    def _publish_status(self) -> None:
        now = time.monotonic()
        payload: dict[str, Any] = {
            "state": self._state,
            "armed": self._armed_until > now,
            "lease_remaining_sec": round(max(0.0, self._armed_until - now), 1),
            "moving": self._moving,
            "max_linear_x": self._max_linear,
            "max_angular_z": self._max_angular,
            "command_timeout_sec": self._command_timeout,
            "fault": self._fault,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_publisher.publish(message)

    def shutdown(self) -> None:
        try:
            self._disarm("SHUTDOWN")
            for _ in range(2):
                time.sleep(0.05)
                self._send_stop()
        finally:
            if self._hal_started:
                self._hal.stop()
                self._hal_started = False


def main() -> None:
    rclpy.init()
    node: FoxgloveTeleopGuard | None = None
    try:
        node = FoxgloveTeleopGuard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
