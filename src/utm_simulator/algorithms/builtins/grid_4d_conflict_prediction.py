"""Conflict detection based on the 4-D grid proposed by Acevedo et al."""

from __future__ import annotations

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.algorithms.builtins.grid_4d import (
    Grid4DConflict,
    Sparse4DGrid,
    conflict_signature,
    track_waypoints,
)
from utm_simulator.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from utm_simulator.domain.tracking import TrackPrediction


class Grid4DConflictPrediction(TacticalConflictPredictionAlgorithm):
    """Detect spherical-separation conflicts in neighboring 4-D cells."""

    def __init__(
        self,
        minimum_separation: float,
        time_step: float,
    ) -> None:
        self.minimum_separation = float(minimum_separation)
        self.time_step = float(time_step)
        if self.minimum_separation <= 0:
            raise ValueError("minimum_separation must be greater than zero")
        if self.time_step <= 0:
            raise ValueError("time_step must be greater than zero")

    def predict(
        self,
        updated_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> list[Grid4DConflict]:
        """Return conflicts involving the track that triggered the TD cycle."""
        return [
            conflict
            for conflict in self.detect_all(tracks)
            if updated_track.uav_name in conflict.uav_names
        ]

    def detect_all(
        self,
        tracks: dict[str, TrackPrediction],
    ) -> list[Grid4DConflict]:
        """Detect all conflicts in a complete track snapshot."""
        grid = Sparse4DGrid(
            separation=self.minimum_separation,
            time_step=self.time_step,
        )
        conflicts: dict[tuple, Grid4DConflict] = {}
        waypoints = sorted(
            track_waypoints(tracks).values(),
            key=lambda waypoint: (
                waypoint.timestamp,
                waypoint.uav_name,
                waypoint.waypoint_index,
            ),
        )
        for waypoint in waypoints:
            for conflict in grid.conflicts_for(waypoint):
                conflicts[conflict_signature(conflict)] = conflict
            grid.insert(waypoint)
        return list(conflicts.values())
