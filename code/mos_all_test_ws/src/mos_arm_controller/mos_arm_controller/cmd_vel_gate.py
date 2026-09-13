"""Small, ROS-independent safety gate for externally sourced base velocity."""

from __future__ import annotations


class NavCommandGate:
    def __init__(self, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self.timeout_sec = float(timeout_sec)
        self._enabled = False
        self._last_command_sec: float | None = None
        self._stop_pending = False

    def set_enabled(self, enabled: bool, *, now_sec: float) -> None:
        self._enabled = bool(enabled)
        self._last_command_sec = None
        self._stop_pending = True

    def record_command(self, *, now_sec: float) -> None:
        self._last_command_sec = float(now_sec)
        self._stop_pending = False

    def accepts_command(self, *, now_sec: float) -> bool:
        del now_sec
        return self._enabled

    def needs_stop(self, *, now_sec: float) -> bool:
        if self._stop_pending:
            return True
        if not self._enabled or self._last_command_sec is None:
            return False
        return float(now_sec) - self._last_command_sec > self.timeout_sec

    def mark_stopped(self) -> None:
        self._stop_pending = False
        self._last_command_sec = None
