"""Single-instance guard for the original MOS vision launch file."""

import atexit
import os
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


LOCK_DIR = Path("/run/lock/mos_vision.instance")
LOCK_PID_PATH = LOCK_DIR / "pid"
PID_PATH = Path("/run/mos_vision.pid")
STATUS_PATH = Path("/run/mos_vision.status")
ORIGINAL_LAUNCH = Path(__file__).with_name("vision_impl.launch.py")

_owns_lock = False


def _process_is_live_vision(pid: int) -> bool:
    proc = Path(f"/proc/{pid}")
    if not proc.is_dir():
        return False
    try:
        command = (proc / "cmdline").read_bytes().replace(b"\0", b" ")
        state = (proc / "stat").read_text(encoding="utf-8").split()[2]
    except (OSError, UnicodeError, IndexError):
        return False
    return state != "Z" and b"mos_3d_object" in command and b"vision.launch.py" in command


def _read_lock_pid() -> int | None:
    for path in (LOCK_PID_PATH, PID_PATH):
        try:
            value = int(path.read_text(encoding="utf-8").strip())
            if value > 0:
                return value
        except (OSError, UnicodeError, ValueError):
            continue
    return None


def _remove_lock_dir_if_owned(pid: int) -> None:
    try:
        lock_pid = int(LOCK_PID_PATH.read_text(encoding="utf-8").strip())
    except (OSError, UnicodeError, ValueError):
        lock_pid = None
    if lock_pid != pid:
        return
    try:
        LOCK_PID_PATH.unlink(missing_ok=True)
        LOCK_DIR.rmdir()
    except OSError:
        pass


def _remove_runtime_files() -> None:
    pid = os.getpid()
    current_pid = str(pid)
    for path in (PID_PATH, STATUS_PATH):
        try:
            if path.exists() and current_pid in path.read_text(encoding="utf-8"):
                path.unlink()
        except (OSError, UnicodeError):
            pass
    if _owns_lock:
        _remove_lock_dir_if_owned(pid)


def _acquire_single_instance_lock() -> None:
    global _owns_lock

    LOCK_DIR.parent.mkdir(parents=True, exist_ok=True)
    for _attempt in range(3):
        try:
            LOCK_DIR.mkdir(mode=0o755)
            _owns_lock = True
            break
        except FileExistsError:
            existing_pid = _read_lock_pid()
            if existing_pid is not None and _process_is_live_vision(existing_pid):
                message = "禁止打开多个 vision.launch.py 文件：视觉节点已在运行。"
                print(f"\n[禁止] {message}\n", flush=True)
                raise RuntimeError(message)

            # The previous process no longer exists. Remove only this guard's
            # own stale PID file/directory, then retry the atomic mkdir.
            try:
                LOCK_PID_PATH.unlink(missing_ok=True)
                LOCK_DIR.rmdir()
            except OSError:
                continue

    if not _owns_lock:
        raise RuntimeError(f"无法取得 Vision 单实例目录锁：{LOCK_DIR}")

    pid = os.getpid()
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    LOCK_PID_PATH.write_text(f"{pid}\n", encoding="utf-8")
    PID_PATH.write_text(f"{pid}\n", encoding="utf-8")
    STATUS_PATH.write_text(
        f"running pid={pid} started_at={started_at}\n", encoding="utf-8"
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

