from __future__ import annotations

import os
import re
import shutil
import subprocess
import time
from typing import Any

from .system_reader import run_readonly


AVERAGE_RATE_RE = re.compile(r"average rate:\s*([0-9.]+)")


def ros_available() -> bool:
    return shutil.which("ros2") is not None


def ros_environment() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "111")
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    return env


def list_nodes(timeout: float = 5.0) -> dict[str, Any]:
    if not ros_available():
        return {"ok": False, "nodes": [], "error": "找不到 ros2 命令"}
    result = run_readonly(["ros2", "node", "list"], timeout=timeout, env=ros_environment())
    return {
        "ok": result["ok"],
        "nodes": sorted(set(result["stdout"].splitlines())) if result["ok"] else [],
        "error": result["stderr"] if not result["ok"] else "",
    }


def list_topics(timeout: float = 5.0) -> dict[str, Any]:
    if not ros_available():
        return {"ok": False, "topics": {}, "error": "找不到 ros2 命令"}
    result = run_readonly(
        ["ros2", "topic", "list", "-t"], timeout=timeout, env=ros_environment()
    )
    topics: dict[str, list[str]] = {}
    if result["ok"]:
        for line in result["stdout"].splitlines():
            if " [" in line and line.endswith("]"):
                name, raw_types = line.split(" [", 1)
                topics[name.strip()] = [
                    item.strip() for item in raw_types[:-1].split(",") if item.strip()
                ]
            elif line.strip():
                topics[line.strip()] = []
    return {
        "ok": result["ok"],
        "topics": topics,
        "error": result["stderr"] if not result["ok"] else "",
    }


def _topic_rate_rclpy(
    topic: str, type_names: list[str], sample_seconds: float
) -> dict[str, Any]:
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        return {"ok": False, "topic": topic, "hz": None, "error": str(exc)}

    # 保留 ROS Graph 返回的类型顺序；同名多类型话题中，首项通常是当前业务链路。
    ordered_types = list(dict.fromkeys(type_names))
    message_type = None
    selected_type = ""
    errors: list[str] = []
    for type_name in ordered_types:
        try:
            message_type = get_message(type_name)
            selected_type = type_name
            break
        except (ImportError, AttributeError, ModuleNotFoundError, ValueError) as exc:
            errors.append(f"{type_name}: {exc}")
    if message_type is None:
        return {
            "ok": False,
            "topic": topic,
            "hz": None,
            "error": "无法加载消息类型；" + "; ".join(errors),
        }

    timestamps: list[float] = []
    node = None
    was_initialized = rclpy.ok()
    try:
        if not was_initialized:
            rclpy.init(args=[])
        node = rclpy.create_node(f"mos_maintenance_rate_{os.getpid()}")
        subscription = node.create_subscription(
            message_type,
            topic,
            lambda _message: timestamps.append(time.monotonic()),
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + max(1.0, sample_seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node,
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
            )
        # 保持引用至采样结束。
        del subscription
    except Exception as exc:
        return {
            "ok": False,
            "topic": topic,
            "hz": None,
            "type": selected_type,
            "error": str(exc),
        }
    finally:
        if node is not None:
            node.destroy_node()
        if not was_initialized and rclpy.ok():
            rclpy.shutdown()

    if len(timestamps) >= 2 and timestamps[-1] > timestamps[0]:
        hz = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
        return {
            "ok": True,
            "topic": topic,
            "hz": round(hz, 3),
            "samples": len(timestamps),
            "type": selected_type,
            "sample_seconds": sample_seconds,
            "error": "",
        }
    return {
        "ok": False,
        "topic": topic,
        "hz": None,
        "samples": len(timestamps),
        "type": selected_type,
        "sample_seconds": sample_seconds,
        "error": "采样窗口内消息不足",
    }


def topic_rate(
    topic: str,
    sample_seconds: float = 4.0,
    type_names: list[str] | None = None,
) -> dict[str, Any]:
    """Sample `ros2 topic hz`; timeout is expected after the sampling window."""
    if type_names:
        return _topic_rate_rclpy(topic, type_names, sample_seconds)
    if not ros_available():
        return {"ok": False, "topic": topic, "hz": None, "error": "找不到 ros2 命令"}
    argv = ["ros2", "topic", "hz", topic, "--window", "20"]
    # ros2 topic hz 在非终端管道中会块缓冲；stdbuf 确保超时终止前的统计可读取。
    if shutil.which("stdbuf"):
        argv = ["stdbuf", "-oL", *argv]
    try:
        result = subprocess.run(
            argv,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=max(1.0, sample_seconds),
            check=False,
            env=ros_environment(),
        )
        output = result.stdout
        error = result.stderr.strip()
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        raw_error = exc.stderr or ""
        if isinstance(raw_error, bytes):
            raw_error = raw_error.decode(errors="replace")
        error = raw_error.strip()
    except OSError as exc:
        return {"ok": False, "topic": topic, "hz": None, "error": str(exc)}
    matches = AVERAGE_RATE_RE.findall(output)
    hz = float(matches[-1]) if matches else None
    return {
        "ok": hz is not None,
        "topic": topic,
        "hz": hz,
        "sample_seconds": sample_seconds,
        "error": "" if hz is not None else (error or "采样窗口内未收到频率统计"),
    }


def graph_snapshot(timeout: float = 5.0) -> dict[str, Any]:
    return {"nodes": list_nodes(timeout), "topics": list_topics(timeout)}
