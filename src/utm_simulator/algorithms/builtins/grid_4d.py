"""Shared data structures for the 4-D grid conflict-management method."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from itertools import product
import math

from utm_simulator.domain.flight import Conflict
from utm_simulator.domain.tracking import TrackPrediction


WaypointKey = tuple[str, int]
CellIndex = tuple[int, int, int, int]


@dataclass
class GridWaypoint:
    """One future track waypoint stored in the sparse 4-D grid."""

    uav_name: str
    waypoint_index: int
    position: list[float]
    timestamp: float

    @property
    def key(self) -> WaypointKey:
        return self.uav_name, self.waypoint_index


@dataclass(frozen=True)
class Grid4DConflict(Conflict):
    """Conflict ``((i, k), (j, l))`` from Equation 9 of the paper."""

    first_uav: str
    first_waypoint_index: int
    second_uav: str
    second_waypoint_index: int
    first_position: tuple[float, float, float]
    second_position: tuple[float, float, float]
    first_timestamp: float
    second_timestamp: float
    distance: float
    time_difference: float

    @property
    def uav_names(self) -> tuple[str, str]:
        return self.first_uav, self.second_uav

    @property
    def waypoint_keys(self) -> tuple[WaypointKey, WaypointKey]:
        return (
            (self.first_uav, self.first_waypoint_index),
            (self.second_uav, self.second_waypoint_index),
        )


class Sparse4DGrid:
    """Sparse grid whose cells have spatial side ``delta`` and duration ``tau``."""

    _NEIGHBOR_OFFSETS = tuple(product((-1, 0, 1), repeat=4))

    def __init__(self, separation: float, time_step: float) -> None:
        if separation <= 0:
            raise ValueError("separation must be greater than zero")
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        self.separation = float(separation)
        self.time_step = float(time_step)
        self._cells: dict[
            CellIndex,
            dict[WaypointKey, GridWaypoint],
        ] = defaultdict(dict)
        self._locations: dict[WaypointKey, CellIndex] = {}

    def cell_index(self, waypoint: GridWaypoint) -> CellIndex:
        x, y, z = waypoint.position
        return (
            math.floor(x / self.separation),
            math.floor(y / self.separation),
            math.floor(z / self.separation),
            math.floor(waypoint.timestamp / self.time_step),
        )

    def insert(self, waypoint: GridWaypoint) -> None:
        cell = self.cell_index(waypoint)
        self._cells[cell][waypoint.key] = waypoint
        self._locations[waypoint.key] = cell

    def remove(self, waypoint: GridWaypoint) -> None:
        cell = self._locations.pop(waypoint.key, None)
        if cell is None:
            return
        occupants = self._cells[cell]
        occupants.pop(waypoint.key, None)
        if not occupants:
            del self._cells[cell]

    def neighbors(self, waypoint: GridWaypoint):
        cell = self.cell_index(waypoint)
        for offset in self._NEIGHBOR_OFFSETS:
            neighbor_cell = tuple(
                coordinate + delta
                for coordinate, delta in zip(cell, offset)
            )
            yield from self._cells.get(neighbor_cell, {}).values()

    def conflicts_for(self, waypoint: GridWaypoint) -> list[Grid4DConflict]:
        conflicts = []
        for other in self.neighbors(waypoint):
            if other.uav_name == waypoint.uav_name:
                continue
            conflict = self.make_conflict(waypoint, other)
            if conflict is not None:
                conflicts.append(conflict)
        return conflicts

    def make_conflict(
        self,
        first: GridWaypoint,
        second: GridWaypoint,
    ) -> Grid4DConflict | None:
        distance = math.dist(first.position, second.position)
        time_difference = abs(first.timestamp - second.timestamp)
        if distance > self.separation or time_difference > self.time_step:
            return None

        if first.key > second.key:
            first, second = second, first
        return Grid4DConflict(
            first_uav=first.uav_name,
            first_waypoint_index=first.waypoint_index,
            second_uav=second.uav_name,
            second_waypoint_index=second.waypoint_index,
            first_position=tuple(first.position),
            second_position=tuple(second.position),
            first_timestamp=first.timestamp,
            second_timestamp=second.timestamp,
            distance=distance,
            time_difference=time_difference,
        )


def track_waypoints(
    tracks: dict[str, TrackPrediction],
) -> dict[WaypointKey, GridWaypoint]:
    """Convert future predicted states into the paper's 4-D waypoints.

    The estimated state is deliberately excluded: it represents the current
    physical position and cannot be displaced by a new tactical flight plan.
    """
    return {
        (uav_name, index): GridWaypoint(
            uav_name=uav_name,
            waypoint_index=index,
            position=list(state.position),
            timestamp=float(state.timestamp),
        )
        for uav_name, track in tracks.items()
        for index, state in enumerate(track.predicted_states)
    }


def conflict_signature(
    conflict: Grid4DConflict,
) -> tuple[WaypointKey, WaypointKey]:
    return tuple(sorted(conflict.waypoint_keys))
