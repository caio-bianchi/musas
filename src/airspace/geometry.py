from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Protocol, Sequence


class Geometry(Protocol):
    """Spatial region that can test whether it contains a 3-D position."""

    def contains(self, position) -> bool:
        ...


@dataclass(frozen=True)
class Box3D:
    minimum_x: float
    maximum_x: float
    minimum_y: float
    maximum_y: float
    minimum_z: float
    maximum_z: float

    def contains(self, position) -> bool:
        x, y, z = position[0], position[1], position[2]
        return (
            self.minimum_x <= x <= self.maximum_x
            and self.minimum_y <= y <= self.maximum_y
            and self.minimum_z <= z <= self.maximum_z
        )


@dataclass(frozen=True)
class Cylinder3D:
    center_x: float
    center_y: float
    radius: float
    minimum_z: float
    maximum_z: float

    def contains(self, position) -> bool:
        x, y, z = position[0], position[1], position[2]
        horizontal_distance = hypot(x - self.center_x, y - self.center_y)
        return (
            horizontal_distance <= self.radius
            and self.minimum_z <= z <= self.maximum_z
        )
