"""Small stability windows for fullness decisions."""

from __future__ import annotations

from collections import deque


class FullnessDecisionWindow:
    """Return full/not_full after enough frame decisions in the window."""

    def __init__(self, *, window: int = 5, min_full: int = 3, min_not_full: int = 3):
        self.window = max(1, int(window))
        self.min_full = max(1, int(min_full))
        self.min_not_full = max(1, int(min_not_full))
        self.history = deque(maxlen=self.window)

    def reset(self):
        self.history.clear()

    def add(self, is_full: bool) -> str:
        self.history.append(bool(is_full))
        full_frames = sum(1 for value in self.history if value)
        not_full_frames = sum(1 for value in self.history if not value)
        if full_frames >= self.min_full:
            return "full"
        if not_full_frames >= self.min_not_full:
            return "not_full"
        return "checking"

    def summary(self) -> dict[str, int]:
        return {
            "window": self.window,
            "frames": len(self.history),
            "full_frames": sum(1 for value in self.history if value),
            "not_full_frames": sum(1 for value in self.history if not value),
        }
