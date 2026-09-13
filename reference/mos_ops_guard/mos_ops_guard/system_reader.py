from __future__ import annotations

import os
import platform
import shutil
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Iterable


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError, TypeError):
        return None


def run_readonly(
    argv: list[str], *, timeout: float = 5.0, env: dict[str, str] | None = None
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        result = subprocess.run(
            argv,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
            env=env,
        )
        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "returncode": None,
            "stdout": (exc.stdout or "").strip()
            if isinstance(exc.stdout, str)
            else "",
            "stderr": f"命令超时（{timeout:g}秒）",
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }
    except (OSError, ValueError) as exc:
        return {
            "ok": False,
            "returncode": None,
            "stdout": "",
            "stderr": str(exc),
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }


def uptime_seconds() -> float | None:
    raw = read_text(Path("/proc/uptime"))
    try:
        return float(raw.split()[0]) if raw else None
    except (ValueError, IndexError):
        return None


def memory_info() -> dict[str, Any]:
    values: dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            name, raw = line.split(":", 1)
            values[name] = int(raw.strip().split()[0]) * 1024
        total = values["MemTotal"]
        available = values["MemAvailable"]
        used = total - available
        return {
            "ok": True,
            "total_bytes": total,
            "used_bytes": used,
            "available_bytes": available,
            "used_percent": round(used / total * 100, 1),
        }
    except (OSError, UnicodeError, ValueError, KeyError, ZeroDivisionError) as exc:
        return {"ok": False, "error": str(exc)}


def disk_info(path: str = "/") -> dict[str, Any]:
    try:
        usage = shutil.disk_usage(path)
        return {
            "ok": True,
            "path": path,
            "total_bytes": usage.total,
            "used_bytes": usage.used,
            "free_bytes": usage.free,
            "used_percent": round(usage.used / usage.total * 100, 1),
        }
    except (OSError, ZeroDivisionError) as exc:
        return {"ok": False, "path": path, "error": str(exc)}


def temperatures() -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        name = read_text(zone / "type")
        raw = read_text(zone / "temp")
        if not name or raw is None:
            continue
        try:
            value = float(raw)
            if abs(value) > 1000:
                value /= 1000
            result.append({"sensor": name, "celsius": round(value, 1)})
        except ValueError:
            continue
    return result


def host_snapshot() -> dict[str, Any]:
    try:
        load = [round(item, 2) for item in os.getloadavg()]
    except OSError:
        load = []
    return {
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "kernel": platform.release(),
        "cpu_count": os.cpu_count(),
        "load_average": load,
        "uptime_seconds": uptime_seconds(),
        "memory": memory_info(),
        "root_disk": disk_info("/"),
        "temperatures": temperatures(),
    }


def process_snapshot(limit: int = 10) -> dict[str, Any]:
    count_result = run_readonly(["ps", "-e", "--no-headers"], timeout=3)
    cpu_result = run_readonly(
        ["ps", "-eo", "pid,comm,%cpu,%mem,args", "--sort=-%cpu"], timeout=3
    )
    mem_result = run_readonly(
        ["ps", "-eo", "pid,comm,%cpu,%mem,args", "--sort=-%mem"], timeout=3
    )

    def rows(value: dict[str, Any]) -> list[str]:
        lines = value.get("stdout", "").splitlines()
        return lines[: limit + 1] if lines else []

    all_processes = count_result.get("stdout", "").splitlines()
    interesting = [
        line.strip()
        for line in all_processes
        if any(marker in line.lower() for marker in ("python", "ros2", "launch"))
    ]
    return {
        "count": len(all_processes) if count_result["ok"] else None,
        "top_cpu": rows(cpu_result),
        "top_memory": rows(mem_result),
        "python_ros_launch": interesting,
        "errors": [
            value["stderr"]
            for value in (count_result, cpu_result, mem_result)
            if not value["ok"] and value["stderr"]
        ],
    }


def default_gateway() -> dict[str, Any]:
    result = run_readonly(["ip", "-j", "route", "show", "default"], timeout=3)
    if result["ok"]:
        try:
            import json

            rows = json.loads(result["stdout"])
            if rows:
                row = rows[0]
                return {
                    "ok": True,
                    "gateway": row.get("gateway"),
                    "interface": row.get("dev"),
                }
        except (ValueError, TypeError, KeyError):
            pass
    return {"ok": False, "gateway": None, "interface": None, "error": result["stderr"]}


def network_interfaces() -> dict[str, Any]:
    addresses = run_readonly(["ip", "-j", "address", "show"], timeout=3)
    links = run_readonly(["ip", "-j", "link", "show"], timeout=3)
    return {
        "addresses_json": addresses.get("stdout", ""),
        "links_json": links.get("stdout", ""),
        "gateway": default_gateway(),
        "errors": [
            result["stderr"]
            for result in (addresses, links)
            if not result["ok"] and result["stderr"]
        ],
    }


def ping(host: str, timeout_seconds: int = 2) -> dict[str, Any]:
    if not host:
        return {"ok": False, "skipped": True, "reason": "未配置地址"}
    result = run_readonly(
        ["ping", "-n", "-c", "2", "-W", str(max(1, timeout_seconds)), host],
        timeout=max(3, timeout_seconds * 3),
    )
    return {
        "ok": result["ok"],
        "host": host,
        "output": result["stdout"] or result["stderr"],
    }


def port_open(host: str, port: int, timeout_seconds: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout_seconds):
            return True
    except (OSError, ValueError):
        return False


def interface_speed(interface: str | None) -> int | None:
    if not interface:
        return None
    raw = read_text(Path("/sys/class/net") / interface / "speed")
    try:
        return int(raw) if raw is not None else None
    except ValueError:
        return None


def iter_files(
    roots: Iterable[str], *, max_files_per_directory: int = 2000
) -> tuple[dict[str, dict[str, Any]], list[str], dict[str, int]]:
    files: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    directory_sizes: dict[str, int] = {}
    for raw_root in roots:
        root = Path(raw_root).expanduser()
        total = 0
        seen = 0
        try:
            exists = root.exists()
        except OSError as exc:
            errors.append(f"无法访问目录 {root}：{exc}")
            directory_sizes[str(root)] = 0
            continue
        if not exists:
            errors.append(f"目录不存在：{root}")
            directory_sizes[str(root)] = 0
            continue
        try:
            iterator = root.rglob("*")
            for path in iterator:
                if seen >= max_files_per_directory:
                    errors.append(f"目录文件过多，已截断：{root}")
                    break
                try:
                    if not path.is_file():
                        continue
                    stat = path.stat()
                except OSError as exc:
                    errors.append(f"无法读取 {path}：{exc}")
                    continue
                seen += 1
                total += stat.st_size
                files[str(path)] = {
                    "size_bytes": stat.st_size,
                    "mtime": stat.st_mtime,
                }
        except OSError as exc:
            errors.append(f"无法扫描 {root}：{exc}")
        directory_sizes[str(root)] = total
    return files, errors, directory_sizes
