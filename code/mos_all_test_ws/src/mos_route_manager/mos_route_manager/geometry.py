"""Map-frame x/y geometry used to arm fullness scan zones."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite


@dataclass(frozen=True)
class SegmentZone:
    """A finite-width corridor around one map-frame line segment."""

    zone_id: str
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    half_width_m: float

    def __post_init__(self) -> None:
        values = (
            self.start_x,
            self.start_y,
            self.end_x,
            self.end_y,
            self.half_width_m,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("zone coordinates and width must be finite")
        if self.half_width_m < 0.0:
            raise ValueError("zone half width must be non-negative")
        if self._length_squared <= 0.0:
            raise ValueError("zone segment must have non-zero length")

    @property
    def _length_squared(self) -> float:
        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        return dx * dx + dy * dy

    def contains(self, x: float, y: float) -> bool:
        """Return whether a map-frame x/y point lies inside this corridor."""
        x = float(x)
        y = float(y)
        if not isfinite(x) or not isfinite(y):
            return False

        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        t = ((x - self.start_x) * dx + (y - self.start_y) * dy) / self._length_squared
        if not 0.0 <= t <= 1.0:
            return False

        closest_x = self.start_x + t * dx
        closest_y = self.start_y + t * dy
        return hypot(x - closest_x, y - closest_y) <= self.half_width_m
