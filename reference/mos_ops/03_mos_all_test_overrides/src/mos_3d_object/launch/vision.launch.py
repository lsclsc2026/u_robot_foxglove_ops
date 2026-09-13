"""Single-instance guard for the original MOS vision launch file."""

import atexit
import fcntl
import os
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


LOCK_PATH = Path("/run/lock/mos_vision.lock")
PID_PATH = Path("/run/mos_vision.pid")
STATUS_PATH = Path("/run/mos_vision.status")
ORIGINAL_LAUNCH = Path(__file__).with_name("vision_impl.launch.py")

_lock_file = None


def _remove_runtime_files():
    current_pid = str(os.getpid())

    for path in (PID_PATH, STATUS_PATH):
        try:
            if path.exists() and current_pid in path.read_text(encoding="utf-8"):
                path.unlink()
        except OSError:
            pass


def _acquire_single_instance_lock():
    global _lock_file

    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    _lock_file = LOCK_PATH.open("a+", encoding="utf-8")

    try:
        fcntl.flock(_lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        _lock_file.close()
        _lock_file = None
        message = "禁止打开多个 vision.launch.py 文件：视觉节点已在运行。"
        print(f"\n[禁止] {message}\n", flush=True)
        raise RuntimeError(message) from exc

    pid = os.getpid()
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")

    _lock_file.seek(0)
    _lock_file.truncate()
    _lock_file.write(f"{pid}\n")
    _lock_file.flush()

    PID_PATH.write_text(f"{pid}\n", encoding="utf-8")
    STATUS_PATH.write_text(
        f"running pid={pid} started_at={started_at}\n",
        encoding="utf-8",
    )
    atexit.register(_remove_runtime_files)


def generate_launch_description():
    _acquire_single_instance_lock()

    if not ORIGINAL_LAUNCH.is_file():
        raise RuntimeError(f"找不到原始视觉启动文件：{ORIGINAL_LAUNCH}")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ORIGINAL_LAUNCH))
            )
        ]
    )
