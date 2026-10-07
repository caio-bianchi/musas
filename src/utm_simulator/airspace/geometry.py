from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import hypot
from typing import Sequence

#TODO: In future, create an abstract class for Position, so it can be represent some non 3-dimensional positions, like graph nodes.


class Geometry(ABC):
    """Spatial region that can test whether it contains a 3-D position."""

    def contains(self, position: Sequence[float]) -> bool:
        ...


@dataclass(frozen=True)
class Box3D(Geometry):
    minimum_x: float
    maximum_x: float
    minimum_y: float
    maximum_y: float
    minimum_z: float
    maximum_z: float

    @abstractmethod
    def contains(self, position: Sequence[float]) -> bool:
        x, y, z = position
        return (
            self.minimum_x <= x <= self.maximum_x
            and self.minimum_y <= y <= self.maximum_y
            and self.minimum_z <= z <= self.maximum_z
        )


@dataclass(frozen=True)
class Cylinder3D(Geometry):
    center_x: float
    center_y: float
    radius: float
    minimum_z: float
    maximum_z: float

    @abstractmethod
    def contains(self, position: Sequence[float]) -> bool:
        x, y, z = position
        horizontal_distance = hypot(x - self.center_x, y - self.center_y)
        return (
            horizontal_distance <= self.radius
            and self.minimum_z <= z <= self.maximum_z
        )
