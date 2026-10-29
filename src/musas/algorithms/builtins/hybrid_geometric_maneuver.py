from __future__ import annotations

from typing import Sequence

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins.conflicts import (
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
)
from musas.algorithms.builtins.tactical_deconfliction import (
    GeometricManeuverDeconfliction,
)
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.domain.flight import Conflict, FlightPlan
from musas.domain.tracking import TrackPrediction


class HybridGeometricManeuver(GeometricManeuverDeconfliction):
    """Use vertical traffic maneuvers and lateral airspace detours.

    A traffic conflict produces only vertical candidates, while an airspace
    conflict produces only lateral candidates.  The inherited validation
    stage checks every candidate against the other predicted tracks and all
    known obstacles and active geofences before issuing an update.

    When an active geofence already contains the UAV, lateral escape and
    lateral corridor candidates are considered.  A candidate is accepted
    only if it exits the geofence without subsequently re-entering it.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: (
            TacticalConflictPredictionAlgorithm | None
        ) = None,
        minimum_separation_distance: float = 30.0,
        vertical_offset: float = 40.0,
        airspace_clearance: float = 10.0,
        maneuver_duration: float = 5.0,
        recovery_duration: float = 8.0,
        minimum_speed: float = 1.0,
        maximum_speed: float = 25.0,
        cooldown: float = 5.0,
        maximum_escape_distance: float = 500.0,
    ) -> None:
        super().__init__(
            conflict_prediction_algorithm=conflict_prediction_algorithm,
            minimum_separation_distance=minimum_separation_distance,
            lateral_offset=airspace_clearance,
            vertical_offset=vertical_offset,
            airspace_clearance=airspace_clearance,
            maneuver_duration=maneuver_duration,
            recovery_duration=recovery_duration,
            minimum_speed=minimum_speed,
            maximum_speed=maximum_speed,
            speed_factors=(1.0,),
            cooldown=cooldown,
            maximum_escape_distance=maximum_escape_distance,
            allow_lateral=True,
            allow_vertical=True,
        )

    def _candidate_plans(
        self,
        conflict: Conflict,
        track: TrackPrediction,
        original_plan: FlightPlan,
    ) -> list[FlightPlan]:
        if isinstance(conflict, TacticalTrafficConflict):
            return self._vertical_traffic_candidates(
                conflict,
                track,
                original_plan,
            )
        if isinstance(conflict, TacticalAirspaceConflict):
            return self._lateral_airspace_candidates(
                conflict,
                track,
                original_plan,
            )
        return []

    def _vertical_traffic_candidates(
        self,
        conflict: TacticalTrafficConflict,
        track: TrackPrediction,
        original_plan: FlightPlan,
    ) -> list[FlightPlan]:
        candidates = super()._candidate_plans(
            conflict,
            track,
            original_plan,
        )
        current_altitude = float(track.estimated_state.position[2])
        return [
            candidate
            for candidate in candidates
            if self._first_target_changes_altitude(
                candidate,
                current_altitude,
            )
        ]

    def _lateral_airspace_candidates(
        self,
        conflict: TacticalAirspaceConflict,
        track: TrackPrediction,
        original_plan: FlightPlan,
    ) -> list[FlightPlan]:
        state = track.estimated_state
        current = [float(value) for value in state.position]
        current_time = float(state.timestamp)
        direction = self._horizontal_direction(state.velocity)
        if direction is None:
            destination = original_plan.waypoint_list[-1].position()
            direction = self._horizontal_direction([
                destination[0] - current[0],
                destination[1] - current[1],
                0.0,
            ])
        if direction is None:
            direction = (1.0, 0.0)

        restriction = conflict.restriction
        routes: list[Sequence[Sequence[float]]] = list(
            self._restriction_detour_routes(
                current,
                restriction.geometry,
                direction,
            )
        )

        if self._restriction_contains(
            restriction,
            current,
            current_time,
        ):
            routes.extend(
                [target]
                for target in self._escape_targets(
                    current,
                    current_time,
                    restriction,
                    direction,
                )
            )

        candidates = self._plans_for_routes(
            original_plan,
            current,
            current_time,
            routes,
        )
        current_altitude = current[2]
        return [
            candidate
            for candidate in candidates
            if self._first_target_preserves_altitude(
                candidate,
                current_altitude,
            )
        ]

    def _candidate_is_valid(
        self,
        uav_name: str,
        candidate_plan: FlightPlan,
        candidate_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> bool:
        """Validate only conflicts involving the maneuvering UAV.

        Running the global predictor here would repeatedly evaluate every
        unrelated pair of tracks for each maneuver candidate.  The two
        focused checks below cover the same relevant constraints: separation
        from every other track and avoidance of every known restriction.
        """
        return self._traffic_recovery_is_valid(
            uav_name,
            candidate_track,
            tracks,
        ) and self._airspace_recovery_is_valid(
            candidate_track,
            candidate_plan,
            airspace,
        )

    @staticmethod
    def _first_target_changes_altitude(
        candidate: FlightPlan,
        current_altitude: float,
    ) -> bool:
        return (
            len(candidate.waypoint_list) > 1
            and abs(
                candidate.waypoint_list[1].z - current_altitude
            ) > 1e-9
        )

    @staticmethod
    def _first_target_preserves_altitude(
        candidate: FlightPlan,
        current_altitude: float,
    ) -> bool:
        return (
            len(candidate.waypoint_list) > 1
            and abs(
                candidate.waypoint_list[1].z - current_altitude
            ) <= 1e-9
        )
