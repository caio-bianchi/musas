from dataclasses import dataclass
from typing import Sequence

from musas.airspace.geometry import Geometry

#TODO: In future, create an abstract class for Velocity, so it can be represent some non 3-dimensional velocity, like velocity in graphs or airways airspaces.

@dataclass
class WindZone:
    """A spatial and optionally temporal region with a constant wind vector."""

    wind_id: str
    geometry: Geometry
    velocity: Sequence[float]
    active_from: float | None = None
    active_until: float | None = None
    name: str | None = None

    def __post_init__(self) -> None:
        if (
            self.active_from is not None
            and self.active_until is not None
            and self.active_until < self.active_from
        ):
            raise ValueError("active_until cannot be earlier than active_from")

    def is_active_at(self, timestamp: float) -> bool:
        if self.active_from is not None and timestamp < self.active_from:
            return False
        if self.active_until is not None and timestamp > self.active_until:
            return False
        return True

    def affects(self, position: Sequence[float], timestamp: float) -> bool:
        return self.is_active_at(timestamp) and self.geometry.contains(position)
