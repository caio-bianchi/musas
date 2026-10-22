from __future__ import annotations

from math import dist, hypot

from musas.airspace.airspace import Airspace
from musas.airspace.geometry import Box3D, Cylinder3D, Geometry
from musas.algorithms.builtins.conflict_prediction import (
    MinimumSeparationConflictPrediction,
)
from musas.algorithms.builtins.conflicts import StrategicAirspaceConflict
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.domain.flight import FlightPlan, Waypoint


class DetourPathPlanner:
    """Plan a short spatial detour around known airspace restrictions.

    For each detected obstacle or active geofence, the planner creates lateral
    candidates on both sides of the restriction and vertical candidates above
    and below it. Candidates are recursively checked against the airspace, and
    the shortest valid route is returned.

    The built-in planner supports :class:`Box3D` and :class:`Cylinder3D`.
    Traffic is intentionally ignored here; it is handled later by strategic
    time shifting.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: (
            ConflictPredictionAlgorithm | None
        ) = None,
        lateral_clearance: float = 10.0,
        vertical_clearance: float = 10.0,
        maximum_maneuvers: int = 4,
        allow_lateral: bool = True,
        allow_vertical: bool = True,
    ) -> None:
        if lateral_clearance <= 0:
            raise ValueError("lateral_clearance must be greater than zero")
        if vertical_clearance <= 0:
            raise ValueError("vertical_clearance must be greater than zero")
        if maximum_maneuvers <= 0:
            raise ValueError("maximum_maneuvers must be greater than zero")
        if not allow_lateral and not allow_vertical:
            raise ValueError(
                "at least one path-planning maneuver must be enabled"
            )

        self.conflict_prediction_algorithm = (
            conflict_prediction_algorithm
            if conflict_prediction_algorithm is not None
            else MinimumSeparationConflictPrediction()
        )
        self.lateral_clearance = float(lateral_clearance)
        self.vertical_clearance = float(vertical_clearance)
        self.maximum_maneuvers = int(maximum_maneuvers)
        self.allow_lateral = allow_lateral
        self.allow_vertical = allow_vertical

    def plan(
        self,
        flight_plan: FlightPlan,
        airspace: Airspace,
    ) -> FlightPlan | None:
        """Return the shortest airspace-free candidate, if one is found."""
        if len(flight_plan.waypoint_list) < 2:
            return None
        return self._search(
            flight_plan,
            airspace,
            remaining_maneuvers=self.maximum_maneuvers,
        )

    def _search(
        self,
        flight_plan: FlightPlan,
        airspace: Airspace,
        remaining_maneuvers: int,
    ) -> FlightPlan | None:
        conflicts = self.conflict_prediction_algorithm.predict(
            proposed_flight_plan=flight_plan,
            approved_flight_plans={},
            airspace=airspace,
        )
        airspace_conflicts = [
            conflict
            for conflict in conflicts
            if isinstance(conflict, StrategicAirspaceConflict)
        ]
        if not airspace_conflicts:
            return flight_plan
        if remaining_maneuvers == 0:
            return None

        first_conflict = min(
            airspace_conflicts,
            key=lambda conflict: conflict.timestamp,
        )
        candidates = self._maneuver_candidates(flight_plan, first_conflict)
        solutions: list[FlightPlan] = []
        for candidate in candidates:
            solution = self._search(
                candidate,
                airspace,
                remaining_maneuvers=remaining_maneuvers - 1,
            )
            if solution is not None:
                solutions.append(solution)

        if not solutions:
            return None
        return min(solutions, key=self._path_length)

    def _maneuver_candidates(
        self,
        flight_plan: FlightPlan,
        conflict: StrategicAirspaceConflict,
    ) -> list[FlightPlan]:
        waypoints = flight_plan.waypoint_list
        segment_index = self._segment_at(waypoints, conflict.timestamp)
        before_index, after_index = self._outer_waypoint_indices(
            waypoints,
            segment_index,
            conflict.restriction.geometry,
        )
        if before_index == after_index:
            return []

        before = waypoints[before_index]
        after = waypoints[after_index]
        candidates: list[FlightPlan] = []

        if self.allow_lateral:
            for side in (-1.0, 1.0):
                positions = self._lateral_positions(
                    before,
                    after,
                    conflict.restriction.geometry,
                    side,
                )
                candidate = self._candidate_plan(
                    flight_plan,
                    before_index,
                    after_index,
                    positions,
                )
                if candidate is not None:
                    candidates.append(candidate)

        if self.allow_vertical:
            for altitude in self._vertical_altitudes(
                conflict.restriction.geometry
            ):
                positions = [
                    [before.x, before.y, altitude],
                    [after.x, after.y, altitude],
                ]
                candidate = self._candidate_plan(
                    flight_plan,
                    before_index,
                    after_index,
                    positions,
                )
                if candidate is not None:
                    candidates.append(candidate)

        return candidates

    def _lateral_positions(
        self,
        before: Waypoint,
        after: Waypoint,
        geometry: Geometry,
        side: float,
    ) -> list[list[float]]:
        delta_x = after.x - before.x
        delta_y = after.y - before.y
        horizontal_length = hypot(delta_x, delta_y)
        if horizontal_length == 0:
            return []

        direction = (delta_x / horizontal_length, delta_y / horizontal_length)
        perpendicular = (-direction[1] * side, direction[0] * side)

        if isinstance(geometry, Cylinder3D):
            center_x, center_y = geometry.center_x, geometry.center_y
            longitudinal_extent = geometry.radius
            lateral_extent = geometry.radius
        elif isinstance(geometry, Box3D):
            center_x = (geometry.minimum_x + geometry.maximum_x) / 2.0
            center_y = (geometry.minimum_y + geometry.maximum_y) / 2.0
            half_x = (geometry.maximum_x - geometry.minimum_x) / 2.0
            half_y = (geometry.maximum_y - geometry.minimum_y) / 2.0
            longitudinal_extent = (
                abs(direction[0]) * half_x
                + abs(direction[1]) * half_y
            )
            lateral_extent = (
                abs(perpendicular[0]) * half_x
                + abs(perpendicular[1]) * half_y
            )
        else:
            return []

        longitudinal_extent += self.lateral_clearance
        lateral_extent += self.lateral_clearance
        return [
            [
                center_x - direction[0] * longitudinal_extent
                + perpendicular[0] * lateral_extent,
                center_y - direction[1] * longitudinal_extent
                + perpendicular[1] * lateral_extent,
                before.z,
            ],
            [
                center_x + direction[0] * longitudinal_extent
                + perpendicular[0] * lateral_extent,
                center_y + direction[1] * longitudinal_extent
                + perpendicular[1] * lateral_extent,
                after.z,
            ],
        ]

    def _vertical_altitudes(self, geometry: Geometry) -> tuple[float, ...]:
        if not isinstance(geometry, (Box3D, Cylinder3D)):
            return ()
        return (
            geometry.maximum_z + self.vertical_clearance,
            geometry.minimum_z - self.vertical_clearance,
        )

    def _candidate_plan(
        self,
        original: FlightPlan,
        before_index: int,
        after_index: int,
        detour_positions: list[list[float]],
    ) -> FlightPlan | None:
        if not detour_positions:
            return None
        positions = [
            waypoint.position()
            for waypoint in original.waypoint_list[: before_index + 1]
        ]
        positions.extend(detour_positions)
        positions.extend(
            waypoint.position()
            for waypoint in original.waypoint_list[after_index:]
        )
        return self._plan_at_original_speed(original, positions)

    @staticmethod
    def _segment_at(waypoints: list[Waypoint], timestamp: float) -> int:
        for index, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
            if start.t <= timestamp <= end.t:
                return index
        return max(0, len(waypoints) - 2)

    @staticmethod
    def _outer_waypoint_indices(
        waypoints: list[Waypoint],
        segment_index: int,
        geometry: Geometry,
    ) -> tuple[int, int]:
        inside_indices = [
            index
            for index, waypoint in enumerate(waypoints)
            if geometry.contains(waypoint.position())
        ]
        if not inside_indices:
            return segment_index, segment_index + 1
        return (
            max(0, min(inside_indices) - 1),
            min(len(waypoints) - 1, max(inside_indices) + 1),
        )

    @staticmethod
    def _plan_at_original_speed(
        original: FlightPlan,
        positions: list[list[float]],
    ) -> FlightPlan | None:
        original_distance = DetourPathPlanner._path_length(original)
        original_duration = (
            original.waypoint_list[-1].t
            - original.waypoint_list[0].t
        )
        if original_distance <= 0 or original_duration <= 0:
            return None

        speed = original_distance / original_duration
        timestamp = original.waypoint_list[0].t
        waypoints = [Waypoint(*positions[0], timestamp)]
        for start, end in zip(positions, positions[1:]):
            segment_length = dist(start, end)
            if segment_length == 0:
                continue
            timestamp += segment_length / speed
            waypoints.append(Waypoint(*end, timestamp))
        return FlightPlan(
            waypoint_list=waypoints,
            priority=original.priority,
        )

    @staticmethod
    def _path_length(flight_plan: FlightPlan) -> float:
        return sum(
            dist(start.position(), end.position())
            for start, end in zip(
                flight_plan.waypoint_list,
                flight_plan.waypoint_list[1:],
            )
        )
