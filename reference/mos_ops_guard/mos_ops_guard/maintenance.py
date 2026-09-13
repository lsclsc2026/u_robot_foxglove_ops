from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any

from .config import load_yaml, runtime_path
from .protection import load_state as load_protection_state
from .ros_reader import graph_snapshot, topic_rate
from .storage import atomic_write_json, atomic_write_text, now_iso, read_json
from .system_reader import (
    host_snapshot,
    interface_speed,
    iter_files,
    network_interfaces,
    ping,
    port_open,
    process_snapshot,
)


GRADE_RANK = {"normal": 0, "warning": 1, "abnormal": 2}
GRADE_CN = {"normal": "正常", "warning": "警告", "abnormal": "异常"}


def _human_bytes(value: int | float | None) -> str:
    if value is None:
        return "不可用"
    number = float(value)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if abs(number) < 1024 or unit == "TiB":
            return f"{number:.1f} {unit}"
        number /= 1024
    return f"{number:.1f} TiB"


def _duration(value: float | None) -> str:
    if value is None:
        return "不可用"
    seconds = max(0, int(value))
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, _ = divmod(seconds, 60)
    return f"{days}天 {hours}小时 {minutes}分钟"


class ResultBuilder:
    def __init__(self) -> None:
        self.grade = "normal"
        self.findings: list[dict[str, str]] = []

    def add(self, grade: str, item: str, detail: str) -> None:
        if grade not in GRADE_RANK:
            raise ValueError(grade)
        if GRADE_RANK[grade] > GRADE_RANK[self.grade]:
            self.grade = grade
        self.findings.append({"grade": grade, "item": item, "detail": detail})


def _network_check(config: dict[str, Any]) -> dict[str, Any]:
    settings = config.get("network", {})
    snapshot = network_interfaces()
    gateway = snapshot["gateway"]
    gateway_host = gateway.get("gateway")
    timeout = int(settings.get("connect_timeout_seconds", 1))
    remote = str(settings.get("remote_host", "")).strip()
    snapshot.update(
        {
            "gateway_ping": ping(str(gateway_host), timeout)
            if gateway_host
            else {"ok": False, "reason": "无默认网关"},
            "remote_ping": ping(remote, timeout) if remote else {"skipped": True},
            "remote_host": remote,
            "ssh_port_open": port_open(
                "127.0.0.1", int(settings.get("ssh_port", 22)), timeout
            ),
            "foxglove_port_open": port_open(
                "127.0.0.1", int(settings.get("foxglove_port", 8765)), timeout
            ),
            "interface_speed_mbps": interface_speed(gateway.get("interface")),
        }
    )
    return snapshot


def _disk_files_check(config: dict[str, Any]) -> dict[str, Any]:
    roots = [str(item) for item in config.get("scan_directories", [])]
    current, errors, directory_sizes = iter_files(
        roots,
        max_files_per_directory=int(config.get("max_files_per_directory", 2000)),
    )
    baseline_path = runtime_path("maintenance_baseline.json")
    baseline = read_json(baseline_path, {})
    previous = baseline.get("files", {}) if isinstance(baseline, dict) else {}
    previous_sizes = (
        baseline.get("directory_sizes", {}) if isinstance(baseline, dict) else {}
    )
    first_run = not isinstance(previous, dict) or not previous
    if first_run:
        previous = {}
    new_paths = [] if first_run else sorted(set(current) - set(previous))
    changed_paths = (
        []
        if first_run
        else sorted(
            path
            for path in set(current) & set(previous)
            if current[path] != previous[path]
        )
    )
    removed_paths = [] if first_run else sorted(set(previous) - set(current))
    large_limit = int(config.get("large_file_bytes", 1024**3))
    large_files = [
        {"path": path, **metadata}
        for path, metadata in current.items()
        if int(metadata.get("size_bytes", 0)) >= large_limit
    ]
    crash_suffixes = (".crash", ".core", ".dmp")
    crash_markers = ("/var/crash/", "/var/lib/apport/coredump/")
    current_crash_files = [
        path
        for path in current
        if path.endswith(crash_suffixes) or any(marker in path for marker in crash_markers)
    ]
    new_crash_files = [path for path in new_paths if path in current_crash_files]
    growth = {
        root: size - int(previous_sizes.get(root, 0))
        for root, size in directory_sizes.items()
    }
    return {
        "first_run": first_run,
        "files": current,
        "scan_errors": errors,
        "directory_sizes": directory_sizes,
        "directory_growth_bytes": growth,
        "new_files": [
            {"path": path, **current[path]} for path in new_paths[:200]
        ],
        "changed_files": changed_paths[:200],
        "removed_files": removed_paths[:200],
        "large_files": sorted(
            large_files, key=lambda item: int(item["size_bytes"]), reverse=True
        )[:100],
        "current_crash_files": current_crash_files[:100],
        "new_crash_files": new_crash_files[:100],
    }


def _ros_check(config: dict[str, Any]) -> dict[str, Any]:
    graph = graph_snapshot(timeout=float(config.get("command_timeout_seconds", 8)))
    nodes = graph["nodes"].get("nodes", [])
    topics = graph["topics"].get("topics", {})
    required_nodes = [str(item) for item in config.get("critical_nodes", [])]
    required_topics = [str(item) for item in config.get("critical_topics", [])]
    frequency_targets = [
        str(item)
        for item in config.get("frequency_topics", [])
        if str(item) in topics
    ]
    rates: dict[str, Any] = {}
    sample_seconds = float(config.get("topic_sample_seconds", 4))
    # 逐个采样，避免同时订阅多路图像和点云给 Jetson 带来瞬时负载。
    for topic in frequency_targets:
        try:
            rates[topic] = topic_rate(topic, sample_seconds, topics.get(topic, []))
        except Exception as exc:
            rates[topic] = {
                "ok": False,
                "topic": topic,
                "hz": None,
                "error": str(exc),
            }
    return {
        "graph": graph,
        "missing_nodes": [name for name in required_nodes if name not in nodes],
        "missing_topics": [name for name in required_topics if name not in topics],
        "topic_rates": rates,
    }


def _vision_duplicates(
    processes: dict[str, Any], markers: list[str]
) -> dict[str, Any]:
    matching = []
    for line in processes.get("python_ros_launch", []):
        if any(marker.lower() in line.lower() for marker in markers):
            matching.append(line)
    return {"count": len(matching), "processes": matching}


def _latest_alarm() -> dict[str, Any]:
    value = read_json(runtime_path("latest_alarm.json"), {})
    return value if isinstance(value, dict) else {}


def collect() -> tuple[dict[str, Any], str]:
    config = load_yaml("maintenance.yaml")
    started = time.monotonic()
    builder = ResultBuilder()
    sections: dict[str, Any] = {}

    checks = {
        "host": lambda: host_snapshot(),
        "processes": lambda: process_snapshot(limit=10),
        "network": lambda: _network_check(config),
        "files": lambda: _disk_files_check(config),
        "ros": lambda: _ros_check(config),
    }
    # 主机/进程/网络/文件彼此独立；ROS 频率检查也可并行，降低总耗时。
    with ThreadPoolExecutor(max_workers=len(checks)) as executor:
        futures = {executor.submit(function): name for name, function in checks.items()}
        for future in as_completed(futures):
            name = futures[future]
            try:
                sections[name] = future.result()
            except Exception as exc:
                sections[name] = {"error": str(exc)}
                builder.add("warning", f"{name} 检查失败", str(exc))

    host = sections.get("host", {})
    root_disk = host.get("root_disk", {})
    if root_disk.get("ok"):
        disk_percent = float(root_disk.get("used_percent", 0))
        if disk_percent >= 95:
            builder.add("abnormal", "根分区", f"使用率 {disk_percent:.1f}%")
        elif disk_percent >= 85:
            builder.add("warning", "根分区", f"使用率 {disk_percent:.1f}%")
    else:
        builder.add("warning", "根分区", "无法读取使用率")

    network = sections.get("network", {})
    if not network.get("gateway_ping", {}).get("ok"):
        builder.add("warning", "默认网关", "不可达或没有默认网关")
    if network.get("remote_host") and not network.get("remote_ping", {}).get("ok"):
        builder.add("warning", "远程运维网络", "配置的远程地址不可达")
    if not network.get("ssh_port_open", False):
        builder.add("warning", "SSH", "本机 SSH 端口未监听")
    if not network.get("foxglove_port_open", False):
        builder.add("warning", "Foxglove", "本机 8765 端口未监听")

    file_data = sections.get("files", {})
    if file_data.get("new_crash_files"):
        builder.add(
            "abnormal",
            "新增崩溃文件",
            f"{len(file_data['new_crash_files'])} 个",
        )
    elif file_data.get("current_crash_files"):
        builder.add(
            "warning",
            "现存崩溃文件",
            f"{len(file_data['current_crash_files'])} 个（本次非新增）",
        )
    if file_data.get("large_files"):
        builder.add(
            "warning", "大文件", f"{len(file_data['large_files'])} 个超过阈值"
        )
    truncated_errors = [
        item for item in file_data.get("scan_errors", []) if "文件过多，已截断" in item
    ]
    if truncated_errors:
        builder.add("warning", "目录扫描", "部分目录达到文件数量上限")

    ros = sections.get("ros", {})
    graph = ros.get("graph", {})
    if not graph.get("nodes", {}).get("ok") or not graph.get("topics", {}).get("ok"):
        builder.add("abnormal", "ROS 2 Graph", "节点或话题列表读取失败")
    if ros.get("missing_nodes"):
        builder.add(
            "abnormal", "关键节点缺失", ", ".join(ros["missing_nodes"])
        )
    if ros.get("missing_topics"):
        builder.add(
            "abnormal", "关键话题缺失", ", ".join(ros["missing_topics"])
        )
    no_rate = [
        topic for topic, result in ros.get("topic_rates", {}).items() if not result["ok"]
    ]
    if no_rate:
        builder.add("abnormal", "话题断流", ", ".join(no_rate))

    processes = sections.get("processes", {})
    vision = _vision_duplicates(
        processes, [str(item) for item in config.get("vision_process_markers", [])]
    )
    sections["vision"] = vision
    if vision["count"] > 1:
        builder.add(
            "abnormal", "Vision 重复实例", f"发现 {vision['count']} 个匹配进程"
        )

    protection = load_protection_state()
    latest_alarm = _latest_alarm()
    sections["protection"] = protection
    sections["latest_alarm"] = latest_alarm
    if protection.get("state") != "NORMAL":
        builder.add("abnormal", "保护状态", str(protection.get("state")))
    if latest_alarm.get("active"):
        builder.add(
            "abnormal",
            "当前告警",
            f"{latest_alarm.get('active_count', 1)} 项活动告警",
        )

    if not builder.findings:
        builder.add("normal", "总体", "未发现达到阈值的问题")

    completed = now_iso()
    report = {
        "schema_version": 1,
        "generated_at": completed,
        "elapsed_seconds": round(time.monotonic() - started, 2),
        "grade": builder.grade,
        "grade_cn": GRADE_CN[builder.grade],
        "findings": builder.findings,
        "sections": sections,
    }
    text = render_text(report)
    atomic_write_json(runtime_path("latest_maintenance.json"), report)
    atomic_write_text(runtime_path("latest_maintenance.txt"), text + "\n")
    if "files" in sections and "files" in sections["files"]:
        atomic_write_json(
            runtime_path("maintenance_baseline.json"),
            {
                "schema_version": 1,
                "updated_at": completed,
                "files": sections["files"]["files"],
                "directory_sizes": sections["files"]["directory_sizes"],
            },
        )
        # 报告不需要重复包含完整基线，降低 latest JSON 体积。
        report["sections"]["files"].pop("files", None)
        atomic_write_json(runtime_path("latest_maintenance.json"), report)
    if config.get("publish_report", False):
        report["publish_error"] = _publish_report_once(
            str(config.get("report_topic", "/mos/monitor/maintenance_report")), text
        )
        atomic_write_json(runtime_path("latest_maintenance.json"), report)
    return report, text


def render_text(report: dict[str, Any]) -> str:
    sections = report["sections"]
    host = sections.get("host", {})
    memory = host.get("memory", {})
    disk = host.get("root_disk", {})
    network = sections.get("network", {})
    ros = sections.get("ros", {})
    graph = ros.get("graph", {})
    nodes = graph.get("nodes", {}).get("nodes", [])
    topics = graph.get("topics", {}).get("topics", {})
    files = sections.get("files", {})
    lines = [
        "【MOS 一键维护检查】",
        f"结果：{report['grade_cn']}",
        f"时间：{report['generated_at']}",
        f"耗时：{report['elapsed_seconds']:.2f} 秒",
        "",
        "发现项：",
    ]
    for finding in report["findings"]:
        lines.append(
            f"- [{GRADE_CN[finding['grade']]}] {finding['item']}：{finding['detail']}"
        )
    lines.extend(
        (
            "",
            "主机：",
            (
                f"- {host.get('hostname', '不可用')}｜{host.get('platform', '不可用')}"
                f"｜运行 {_duration(host.get('uptime_seconds'))}"
            ),
            (
                f"- 负载 {host.get('load_average', [])}｜CPU {host.get('cpu_count', '不可用')} 核"
                f"｜内存 {_human_bytes(memory.get('used_bytes'))}/"
                f"{_human_bytes(memory.get('total_bytes'))}"
            ),
            (
                f"- 根分区 {_human_bytes(disk.get('used_bytes'))}/"
                f"{_human_bytes(disk.get('total_bytes'))}（{disk.get('used_percent', '不可用')}%）"
            ),
            f"- 温度：{json.dumps(host.get('temperatures', []), ensure_ascii=False)}",
            "",
            "网络：",
            (
                f"- 网关 {network.get('gateway', {}).get('gateway', '无')}"
                f"（{'可达' if network.get('gateway_ping', {}).get('ok') else '不可达'}）"
                f"｜链路 {network.get('interface_speed_mbps') or '未知'} Mbps"
            ),
            (
                f"- SSH 22：{'监听' if network.get('ssh_port_open') else '未监听'}"
                f"｜Foxglove 8765：{'监听' if network.get('foxglove_port_open') else '未监听'}"
            ),
            "",
            "ROS 2：",
            f"- 节点 {len(nodes)}｜话题 {len(topics)}",
            f"- 缺失节点：{', '.join(ros.get('missing_nodes', [])) or '无'}",
            f"- 缺失话题：{', '.join(ros.get('missing_topics', [])) or '无'}",
        )
    )
    for topic, rate in sorted(ros.get("topic_rates", {}).items()):
        value = f"{rate['hz']:.2f} Hz" if rate.get("hz") is not None else "未收到数据"
        lines.append(f"- {topic}：{value}")
    lines.extend(
        (
            "",
            "文件：",
            f"- 首次建立基线：{'是' if files.get('first_run') else '否'}",
            f"- 新增文件：{len(files.get('new_files', []))}",
            f"- 新增 core/crash：{len(files.get('new_crash_files', []))}",
            f"- 超阈值大文件：{len(files.get('large_files', []))}",
            "",
            "保护与告警：",
            f"- 保护状态：{sections.get('protection', {}).get('state', '未知')}",
            (
                f"- 活动告警："
                f"{sections.get('latest_alarm', {}).get('active_count', 0)} 项"
            ),
            "",
            "说明：本检查全程只读；除覆盖本运维库 runtime 下的报告和基线外，"
            "不会修改、删除或重启生产内容。",
        )
    )
    return "\n".join(lines)


def _publish_report_once(topic: str, content: str) -> str | None:
    try:
        import rclpy
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        was_initialized = rclpy.ok()
        if not was_initialized:
            rclpy.init(args=None)
        node = rclpy.create_node(f"mos_maintenance_report_{os.getpid()}")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        publisher = node.create_publisher(String, topic, qos)
        message = String()
        message.data = content
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.3)
        node.destroy_node()
        if not was_initialized and rclpy.ok():
            rclpy.shutdown()
        return None
    except Exception as exc:
        return str(exc)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="只读执行一次 MOS 维护检查")
    parser.add_argument(
        "--strict", action="store_true", help="警告返回 1、异常返回 2"
    )
    parser.add_argument("--json", action="store_true", help="终端输出 JSON")
    args = parser.parse_args(argv)
    try:
        report, text = collect()
    except Exception as exc:
        print(f"维护检查无法启动：{exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        if args.json
        else text
    )
    return GRADE_RANK[report["grade"]] if args.strict else 0


if __name__ == "__main__":
    raise SystemExit(main())
