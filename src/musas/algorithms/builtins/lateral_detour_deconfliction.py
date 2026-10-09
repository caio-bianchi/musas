from __future__ import annotations

import math

from musas.airspace.airspace import Airspace
from musas.airspace.geometry import Box3D, Cylinder3D
from musas.algorithms.builtins.conflict_prediction import (
    MinimumSeparationConflictPrediction,
)
from musas.algorithms.builtins.conflicts import StrategicAirspaceConflict
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from musas.domain.flight import (
    Conflict,
    FlightPlan,
    FlightPlanUpdate,
    Waypoint,
)
from musas.domain.message_types import StrategicDecision


class LateralDetourDeconfliction(StrategicDeconflictionAlgorithm):
    """Create lateral detours around known obstacles and geofences.

    The algorithm searches both sides of each conflicting restriction. For a
    cylinder, the detour clears its radius; for a box, it clears the projected
    half-width in the direction perpendicular to the affected route segment.
    Every candidate is checked again by the supplied conflict predictor.

    This algorithm intentionally resolves only ``StrategicAirspaceConflict``
    instances. Traffic conflicts should be handled by a temporal, vertical or
    other traffic-specific algorithm.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: ConflictPredictionAlgorithm | None = (
            None
        ),
        lateral_clearance: float = 10.0,
        maximum_detours: int = 4,
    ) -> None:
        if lateral_clearance < 0:
            raise ValueError("lateral_clearance cannot be negative")
        if maximum_detours <= 0:
            raise ValueError("maximum_detours must be greater than zero")
        if (
            conflict_prediction_algorithm is not None
            and not isinstance(
                conflict_prediction_algorithm,
                ConflictPredictionAlgorithm,
            )
        ):
            raise TypeError(
                "conflict_prediction_algorithm must inherit from "
                "ConflictPredictionAlgorithm"
            )
        self.conflict_prediction_algorithm = (
            conflict_prediction_algorithm
            if conflict_prediction_algorithm is not None
            else MinimumSeparationConflictPrediction()
        )
        self.lateral_clearance = float(lateral_clearance)
        self.maximum_detours = int(maximum_detours)

    def deconflict(
        self,
        proposed_flight_plan: FlightPlan,
        conflicts: list[Conflict],
        inactive_flight_plans: dict[str, FlightPlan],
        active_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> tuple[
        StrategicDecision,
        FlightPlan | None,
        list[FlightPlanUpdate],
    ]:
        if not conflicts:
            return StrategicDecision.APPROVE, None, []
        if any(
            not isinstance(conflict, StrategicAirspaceConflict)
            for conflict in conflicts
        ):
            return StrategicDecision.REJECT, None, []

        approved_flight_plans = {
            **inactive_flight_plans,
            **active_flight_plans,
        }
        candidate = self._search(
            proposed_flight_plan,
            approved_flight_plans,
            airspace,
            remaining_detours=self.maximum_detours,
            allow_other_conflicts=False,
        )
        if candidate is None:
            return StrategicDecision.REJECT, None, []
        return StrategicDecision.COUNTER_PROPOSAL, candidate, []

    def find_airspace_free_plan(
        self,
        proposed_flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> FlightPlan | None:
        """Return a plan free of known airspace conflicts.

        Traffic conflicts may remain in the returned plan. This method is
        intended for composition with a traffic deconfliction algorithm.
        """
        return self._search(
            proposed_flight_plan,
            approved_flight_plans,
            airspace,
            remaining_detours=self.maximum_detours,
            allow_other_conflicts=True,
        )

    def _search(
        self,
        flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
        remaining_detours: int,
        allow_other_conflicts: bool,
    ) -> FlightPlan | None:
        conflicts = self.conflict_prediction_algorithm.predict(
            proposed_flight_plan=flight_plan,
            approved_flight_plans=approved_flight_plans,
            airspace=airspace,
        )
        if not conflicts:
            return flight_plan

        airspace_conflicts = [
            conflict
            for conflict in conflicts
            if isinstance(conflict, StrategicAirspaceConflict)
        ]
        if not airspace_conflicts:
            return flight_plan if allow_other_conflicts else None
        if remaining_detours <= 0:
            return None
        if (
            not allow_other_conflicts
            and len(airspace_conflicts) != len(conflicts)
        ):
            return None

        conflict = min(
            airspace_conflicts,
            key=lambda item: item.timestamp,
        )
        for side in (-1.0, 1.0):
            candidate = self._detour_candidate(
                flight_plan,
                conflict,
                side,
            )
            if candidate is None:
                continue
            result = self._search(
                candidate,
                approved_flight_plans,
                airspace,
                remaining_detours=remaining_detours - 1,
                allow_other_conflicts=allow_other_conflicts,
            )
            if result is not None:
                return result
        return None

    def _detour_candidate(
        self,
        flight_plan: FlightPlan,
        conflict: StrategicAirspaceConflict,
        side: float,
    ) -> FlightPlan | None:
        waypoints = flight_plan.waypoint_list
        if len(waypoints) < 2:
            return None

        segment_index = self._segment_at(
            waypoints,
            conflict.timestamp,
        )
        geometry = conflict.restriction.geometry
        first_removed = segment_index + 1
        last_removed = segment_index
        for index, waypoint in enumerate(waypoints[1:-1], start=1):
            if geometry.contains(waypoint.position()):
                first_removed = min(first_removed, index)
                last_removed = max(last_removed, index)

        if last_removed < first_removed:
            before_index = segment_index
            after_index = segment_index + 1
        else:
            before_index = max(0, first_removed - 1)
            after_index = min(len(waypoints) - 1, last_removed + 1)
        if before_index == after_index:
            return None

        before = waypoints[before_index]
        after = waypoints[after_index]
        direction = self._horizontal_direction(before, after)
        if direction is None:
            return None
        perpendicular = (-direction[1] * side, direction[0] * side)
        detour_positions = self._detour_positions(
            geometry,
            direction,
            perpendicular,
            before.z,
            after.z,
        )
        if not detour_positions:
            return None

        positions = [
            waypoint.position()
            for waypoint in waypoints[: before_index + 1]
        ]
        positions.extend(detour_positions)
        positions.extend(
            waypoint.position()
            for waypoint in waypoints[after_index:]
        )
        return self._plan_at_original_speed(flight_plan, positions)

    def _detour_positions(
        self,
        geometry,
        direction: tuple[float, float],
        perpendicular: tuple[float, float],
        start_altitude: float,
        end_altitude: float,
    ) -> list[list[float]]:
        if isinstance(geometry, Cylinder3D):
            center_x = geometry.center_x
            center_y = geometry.center_y
            longitudinal_clearance = (
                geometry.radius + self.lateral_clearance
            )
            lateral_clearance = geometry.radius + self.lateral_clearance
        elif isinstance(geometry, Box3D):
            center_x = (geometry.minimum_x + geometry.maximum_x) / 2.0
            center_y = (geometry.minimum_y + geometry.maximum_y) / 2.0
            half_x = (geometry.maximum_x - geometry.minimum_x) / 2.0
            half_y = (geometry.maximum_y - geometry.minimum_y) / 2.0
            longitudinal_clearance = (
                abs(direction[0]) * half_x
                + abs(direction[1]) * half_y
                + self.lateral_clearance
            )
            lateral_clearance = (
                abs(perpendicular[0]) * half_x
                + abs(perpendicular[1]) * half_y
                + self.lateral_clearance
            )
        else:
            return []

        return [
            [
                center_x - direction[0] * longitudinal_clearance
                + perpendicular[0] * lateral_clearance,
                center_y - direction[1] * longitudinal_clearance
                + perpendicular[1] * lateral_clearance,
                start_altitude,
            ],
            [
                center_x + direction[0] * longitudinal_clearance
                + perpendicular[0] * lateral_clearance,
                center_y + direction[1] * longitudinal_clearance
                + perpendicular[1] * lateral_clearance,
                end_altitude,
            ],
        ]

    @staticmethod
    def _segment_at(
        waypoints: list[Waypoint],
        timestamp: float,
    ) -> int:
        for index, (start, end) in enumerate(
            zip(waypoints, waypoints[1:])
        ):
            if start.t <= timestamp <= end.t:
                return index
        return max(0, len(waypoints) - 2)

    @staticmethod
    def _horizontal_direction(
        start: Waypoint,
        end: Waypoint,
    ) -> tuple[float, float] | None:
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        length = math.hypot(delta_x, delta_y)
        if length == 0:
            return None
        return delta_x / length, delta_y / length

    @staticmethod
    def _plan_at_original_speed(
        original: FlightPlan,
        positions: list[list[float]],
    ) -> FlightPlan | None:
        original_waypoints = original.waypoint_list
        original_distance = sum(
            math.dist(start.position(), end.position())
            for start, end in zip(
                original_waypoints,
                original_waypoints[1:],
            )
        )
        original_duration = (
            original_waypoints[-1].t - original_waypoints[0].t
        )
        if original_distance <= 0 or original_duration <= 0:
            return None

        speed = original_distance / original_duration
        timestamp = original_waypoints[0].t
        new_waypoints = [Waypoint(*positions[0], timestamp)]
        for start, end in zip(positions, positions[1:]):
            timestamp += math.dist(start, end) / speed
            new_waypoints.append(Waypoint(*end, timestamp))
        return FlightPlan(new_waypoints)
