from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Sequence

from src.airspace.geometry import Geometry


@dataclass(frozen=True)
class WindZone:
    """A spatial and optionally temporal region with a constant wind vector."""

    wind_id: str
    geometry: Geometry
    velocity: tuple[float, float, float] | Sequence[float]
    active_from: float | None = None
    active_until: float | None = None
    name: str | None = None

    def __post_init__(self):
        velocity = tuple(float(component) for component in self.velocity)
        if len(velocity) != 3:
            raise ValueError("wind velocity must have exactly three components")
        if not all(isfinite(component) for component in velocity):
            raise ValueError("wind velocity components must be finite")
        if (
            self.active_from is not None
            and self.active_until is not None
            and self.active_until < self.active_from
        ):
            raise ValueError("active_until cannot be earlier than active_from")

        object.__setattr__(self, "velocity", velocity)

    def is_active_at(self, timestamp: float) -> bool:
        if self.active_from is not None and timestamp < self.active_from:
            return False
        if self.active_until is not None and timestamp > self.active_until:
            return False
        return True

    def affects(self, position, timestamp: float) -> bool:
        return self.is_active_at(timestamp) and self.geometry.contains(position)
