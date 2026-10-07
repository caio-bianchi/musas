from __future__ import annotations
from dataclasses import dataclass, field

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    t: float

    def __post_init__(self) -> None:
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)
        self.t = float(self.t)

    def position(self) -> list[float]:
        return [self.x, self.y, self.z]

    def timestamp(self) -> float:
        return self.t

    def __str__(self) -> str:
        return f"[{self.x}, {self.y}, {self.z}, {self.t}]"

    def __lt__(self, other: Waypoint) -> bool:
        return self.t < other.t


class Conflict():
    """
    Base class for conflict representations.

    Users may define their own conflict types by subclassing this class.
    """

@dataclass
class FlightPlan:
    waypoint_list: list[Waypoint] = field(default_factory=list)

    def __post_init__(self) -> None: # sort waypoint list (w/ respect to timestamps)
        self.waypoint_list.sort()

    def add_waypoint(self, waypoint: Waypoint) -> None:
        self.waypoint_list.append(waypoint)
        self.waypoint_list.sort()

@dataclass
class FlightPlanUpdate:
    """New flight plan to be issued."""
    uav_name: str
    flight_plan: FlightPlan


