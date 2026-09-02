from dataclasses import dataclass

from src.airspace.geometry import Geometry


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    geometry: Geometry
    name: str | None = None

    def contains(self, position) -> bool:
        return self.geometry.contains(position)


@dataclass(frozen=True)
class Geofence:
    """A static or temporally activated KEEP_OUT region."""

    geofence_id: str
    geometry: Geometry
    active_from: float | None = None
    active_until: float | None = None
    name: str | None = None

    def is_active_at(self, timestamp: float) -> bool:
        if self.active_from is not None and timestamp < self.active_from:
            return False
        if self.active_until is not None and timestamp > self.active_until:
            return False

        return True

    def prohibits(self, position, timestamp: float) -> bool:
        return self.is_active_at(timestamp) and self.geometry.contains(position)
