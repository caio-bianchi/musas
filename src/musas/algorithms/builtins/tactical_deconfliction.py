from __future__ import annotations

from math import dist
from typing import Sequence

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins.conflicts import (
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
)
from musas.algorithms.builtins._tactical_maneuver_core import (
    _GeometricManeuverCore,
)
from musas.algorithms.builtins._track_comparison import (
    IndexedTrack,
    aligned_separations,
    bounding_box_gap,
)
from musas.algorithms.builtins._trajectory import position_at, velocity_at
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.tracking import PredictedState, TrackPrediction


# A candidate must improve the baseline separation by more than this amount.
_SEPARATION_TOLERANCE = 1e-9


class HybridGeometricManeuver(_GeometricManeuverCore):
    """Use vertical traffic maneuvers and lateral airspace detours.

    A traffic conflict produces only vertical candidates, while an airspace
    conflict produces only lateral candidates.  The inherited validation
    stage checks every candidate against the other predicted tracks and all
    known obstacles and active geofences before issuing an update.

    When a restriction already contains the UAV, the maneuver starts at the
    nearest lateral exit and, when necessary, continues around the protected
    boundary before rejoining the original route. A candidate is accepted
    only if it exits the restriction without subsequently re-entering it.
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
            vertical_offset=vertical_offset,
            airspace_clearance=airspace_clearance,
            maneuver_duration=maneuver_duration,
            recovery_duration=recovery_duration,
            minimum_speed=minimum_speed,
            maximum_speed=maximum_speed,
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

        conflict_position = (
            conflict.position
            if conflict.uav_name == track.uav_name
            else conflict.other_position
        )
        longitudinal_extent = self.minimum_separation_distance
        routes = []
        for vertical_direction in (-1.0, 1.0):
            altitude = (
                current[2]
                + vertical_direction * self.vertical_offset
            )
            routes.append([
                [
                    conflict_position[0]
                    - direction[0] * longitudinal_extent,
                    conflict_position[1]
                    - direction[1] * longitudinal_extent,
                    altitude,
                ],
                [
                    conflict_position[0]
                    + direction[0] * longitudinal_extent,
                    conflict_position[1]
                    + direction[1] * longitudinal_extent,
                    altitude,
                ],
            ])

        return self._plans_for_routes(
            original_plan,
            current,
            current_time,
            routes,
        )

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
        inside_restriction = self._restriction_contains(
            restriction,
            current,
            current_time,
        )
        if inside_restriction:
            escape_targets = [
                target
                for target in self._escape_targets(
                    current,
                    current_time,
                    restriction,
                    direction,
                )
                if abs(target[2] - current[2]) <= 1e-9
            ]
            if not escape_targets:
                return []
            shortest_distance = min(
                dist(current, target)
                for target in escape_targets
            )
            nearest_exits = [
                target
                for target in escape_targets
                if abs(dist(current, target) - shortest_distance) <= 1e-9
            ]
            detours = [
                detour
                for detour in self._restriction_detour_routes(
                    current,
                    restriction.geometry,
                    direction,
                )
                if all(
                    abs(target[2] - current[2]) <= 1e-9
                    for target in detour
                )
            ]
            routes: list[Sequence[Sequence[float]]] = []
            for exit_target in nearest_exits:
                routes.append([exit_target])
                routes.extend(
                    [exit_target, *detour]
                    for detour in detours
                )
        else:
            routes = list(
                self._restriction_detour_routes(
                    current,
                    restriction.geometry,
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


class ExtendedHybridGeometricManeuver(HybridGeometricManeuver):
    """Search a richer set of tactical maneuvers under disturbances.

    Traffic conflicts are evaluated using several vertical and lateral
    candidates. Candidate tracks retain the velocity disturbance inferred
    from the current estimate, use an additional separation margin, and may
    bypass the regular cooldown once while the same imminent conflict
    persists. A candidate must improve the maneuvering UAV's minimum
    predicted separation. Airspace conflicts retain the inherited lateral
    escape and detour behavior.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: (
            TacticalConflictPredictionAlgorithm | None
        ) = None,
        minimum_separation_distance: float = 30.0,
        vertical_offsets: Sequence[float] = (40.0, 60.0, 80.0),
        lateral_offsets: Sequence[float] = (45.0, 70.0),
        desired_separation_factor: float = 1.5,
        emergency_time_to_conflict: float = 6.0,
        airspace_clearance: float = 10.0,
        maneuver_duration: float = 5.0,
        recovery_duration: float = 8.0,
        minimum_speed: float = 1.0,
        maximum_speed: float = 25.0,
        cooldown: float = 5.0,
        maximum_escape_distance: float = 500.0,
    ) -> None:
        vertical_offsets = tuple(float(value) for value in vertical_offsets)
        lateral_offsets = tuple(float(value) for value in lateral_offsets)
        if not vertical_offsets or any(value <= 0 for value in vertical_offsets):
            raise ValueError("vertical_offsets must contain positive values")
        if not lateral_offsets or any(value <= 0 for value in lateral_offsets):
            raise ValueError("lateral_offsets must contain positive values")
        if desired_separation_factor < 1.0:
            raise ValueError(
                "desired_separation_factor cannot be smaller than one"
            )
        if emergency_time_to_conflict < 0:
            raise ValueError(
                "emergency_time_to_conflict cannot be negative"
            )

        super().__init__(
            conflict_prediction_algorithm=conflict_prediction_algorithm,
            minimum_separation_distance=minimum_separation_distance,
            vertical_offset=vertical_offsets[0],
            airspace_clearance=airspace_clearance,
            maneuver_duration=maneuver_duration,
            recovery_duration=recovery_duration,
            minimum_speed=minimum_speed,
            maximum_speed=maximum_speed,
            cooldown=cooldown,
            maximum_escape_distance=maximum_escape_distance,
        )
        self.vertical_offsets = vertical_offsets
        self.lateral_offsets = lateral_offsets
        self.desired_separation = (
            float(desired_separation_factor)
            * self.minimum_separation_distance
        )
        self.emergency_time_to_conflict = float(
            emergency_time_to_conflict
        )
        self._emergency_bypassed_conflicts: set[tuple[str, str]] = set()

    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[FlightPlanUpdate]:
        order = {
            name: index
            for index, name in enumerate(flight_plans)
        }
        current_emergency_conflicts: set[tuple[str, str]] = set()
        for conflict in conflicts:
            if not isinstance(conflict, TacticalTrafficConflict):
                continue
            conflict_key = tuple(sorted((
                conflict.uav_name,
                conflict.other_uav,
            )))
            uav_name = self._maneuvering_uav(
                conflict,
                flight_plans,
                order,
            )
            track = tracks.get(uav_name) if uav_name is not None else None
            if track is None:
                continue
            time_to_conflict = (
                float(conflict.timestamp)
                - float(track.estimated_state.timestamp)
            )
            if not (
                0.0
                <= time_to_conflict
                <= self.emergency_time_to_conflict
            ):
                continue
            current_emergency_conflicts.add(conflict_key)
            if conflict_key not in self._emergency_bypassed_conflicts:
                self._last_update_at.pop(uav_name, None)
                self._emergency_bypassed_conflicts.add(conflict_key)

        self._emergency_bypassed_conflicts.intersection_update(
            current_emergency_conflicts
        )

        return super().deconflict(
            conflicts,
            tracks,
            flight_plans,
            airspace,
        )

    def _candidate_plans(
        self,
        conflict: Conflict,
        track: TrackPrediction,
        original_plan: FlightPlan,
    ) -> list[FlightPlan]:
        if not isinstance(conflict, TacticalTrafficConflict):
            return super()._candidate_plans(
                conflict,
                track,
                original_plan,
            )

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

        conflict_position = (
            conflict.position
            if conflict.uav_name == track.uav_name
            else conflict.other_position
        )
        extent = self.desired_separation
        routes: list[list[list[float]]] = []

        for offset in self.vertical_offsets:
            for sign in (-1.0, 1.0):
                altitude = current[2] + sign * offset
                routes.append([
                    [
                        conflict_position[0] - direction[0] * extent,
                        conflict_position[1] - direction[1] * extent,
                        altitude,
                    ],
                    [
                        conflict_position[0] + direction[0] * extent,
                        conflict_position[1] + direction[1] * extent,
                        altitude,
                    ],
                ])

        perpendicular = (-direction[1], direction[0])
        for offset in self.lateral_offsets:
            for sign in (-1.0, 1.0):
                routes.append([
                    [
                        conflict_position[0]
                        + sign * perpendicular[0] * offset,
                        conflict_position[1]
                        + sign * perpendicular[1] * offset,
                        current[2],
                    ],
                    [
                        conflict_position[0] + direction[0] * extent
                        + sign * perpendicular[0] * offset,
                        conflict_position[1] + direction[1] * extent
                        + sign * perpendicular[1] * offset,
                        current[2],
                    ],
                ])

        return self._plans_for_routes(
            original_plan,
            current,
            current_time,
            routes,
        )

    def _resolve_for_uav(
        self,
        uav_name: str,
        conflict: Conflict,
        tracks: dict[str, TrackPrediction],
        original_plan: FlightPlan,
        airspace: Airspace,
    ) -> tuple[FlightPlan, TrackPrediction] | None:
        track = tracks[uav_name]
        other_tracks = self._indexed_other_tracks(uav_name, tracks)
        baseline_separation = self._minimum_predicted_separation(
            IndexedTrack.from_track(track),
            other_tracks,
        )
        # Tracks farther than this distance from a candidate cannot affect
        # its validity, its improvement test or its risk term. See
        # _candidate_traffic_separation.
        relevance_distance = max(
            self.desired_separation,
            baseline_separation + _SEPARATION_TOLERANCE,
        )

        valid: list[tuple[float, FlightPlan, TrackPrediction]] = []
        for candidate in self._candidate_plans(
            conflict,
            track,
            original_plan,
        ):
            candidate_track = self._disturbance_aware_track(
                track,
                original_plan,
                candidate,
            )
            separation = self._candidate_traffic_separation(
                IndexedTrack.from_track(candidate_track),
                other_tracks,
                relevance_distance,
            )
            if separation is None:
                continue
            if not self._airspace_recovery_is_valid(
                candidate_track,
                candidate,
                airspace,
            ):
                continue
            if (
                isinstance(conflict, TacticalTrafficConflict)
                and separation <= baseline_separation + _SEPARATION_TOLERANCE
            ):
                continue
            risk = max(0.0, self.desired_separation - separation)
            valid.append((
                self._candidate_cost(candidate, original_plan)
                + 2.0 * risk,
                candidate,
                candidate_track,
            ))

        if not valid:
            return None
        _, flight_plan, candidate_track = min(
            valid,
            key=lambda item: item[0],
        )
        return flight_plan, candidate_track

    def _disturbance_aware_track(
        self,
        original_track: TrackPrediction,
        original_plan: FlightPlan,
        candidate_plan: FlightPlan,
    ) -> TrackPrediction:
        start_time = float(original_track.estimated_state.timestamp)
        nominal_velocity = velocity_at(original_plan, start_time)
        disturbance = [
            float(observed) - float(nominal)
            for observed, nominal in zip(
                original_track.estimated_state.velocity,
                nominal_velocity,
            )
        ]

        timestamps = {
            float(state.timestamp)
            for state in original_track.predicted_states
            if state.timestamp > start_time
        }
        timestamps.update(
            waypoint.t
            for waypoint in candidate_plan.waypoint_list[1:]
            if waypoint.t > start_time
        )
        predicted_states = tuple(
            PredictedState(
                position=[
                    coordinate
                    + disturbance[axis] * (timestamp - start_time)
                    for axis, coordinate in enumerate(
                        position_at(candidate_plan, timestamp)
                    )
                ],
                velocity=[
                    component + disturbance[axis]
                    for axis, component in enumerate(
                        velocity_at(candidate_plan, timestamp)
                    )
                ],
                timestamp=timestamp,
            )
            for timestamp in sorted(timestamps)
        )
        return TrackPrediction(
            uav_name=original_track.uav_name,
            estimated_state=original_track.estimated_state,
            predicted_states=predicted_states,
            generated_at=original_track.generated_at,
        )

    def _candidate_traffic_separation(
        self,
        candidate: IndexedTrack,
        other_tracks: dict[str, IndexedTrack],
        relevance_distance: float,
    ) -> float | None:
        """Check a candidate against traffic in a single pass.

        Returns ``None`` when the candidate breaks the recovery rule (see
        :meth:`_recovers_desired_separation`) with any other track. Otherwise,
        returns the minimum future separation from the tracks that lie within
        ``relevance_distance``, or infinity if there are none.

        Skipping a track whose bounding box is farther than
        ``relevance_distance`` does not change any decision, because all of
        its separations exceed that distance and therefore:

        * exceed ``desired_separation``, so the recovery rule holds and the
          risk term ``desired_separation - separation`` is not positive;
        * exceed the baseline separation, so the track cannot make the
          candidate fail the improvement test.
        """
        maximum_time_difference = self._maximum_time_difference()
        minimum = float("inf")
        for other in other_tracks.values():
            if bounding_box_gap(candidate, other) > relevance_distance:
                continue
            separations = aligned_separations(
                candidate,
                other,
                maximum_time_difference,
            )
            if not separations:
                continue
            if not self._recovers_desired_separation(separations):
                return None
            minimum = min(minimum, *self._future(separations))
        return minimum

    def _recovers_desired_separation(self, separations: list[float]) -> bool:
        """Apply the recovery rule to the separations from one other track.

        If the UAVs start closer than ``desired_separation``, they must be
        at least that far apart at the end of the prediction. Otherwise, they
        must remain at least that far apart throughout the prediction.
        """
        future = self._future(separations)
        if separations[0] < self.desired_separation:
            return future[-1] >= self.desired_separation
        return min(future) >= self.desired_separation

    def _minimum_predicted_separation(
        self,
        track: IndexedTrack,
        other_tracks: dict[str, IndexedTrack],
    ) -> float:
        """Return the minimum future separation from every other track."""
        maximum_time_difference = self._maximum_time_difference()
        minimum = float("inf")
        for other in other_tracks.values():
            separations = aligned_separations(
                track,
                other,
                maximum_time_difference,
            )
            if separations:
                minimum = min(minimum, *self._future(separations))
        return minimum

    def _traffic_recovery_is_valid(
        self,
        uav_name: str,
        candidate_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
    ) -> bool:
        """Validation hook inherited from HybridGeometricManeuver."""
        return self._candidate_traffic_separation(
            IndexedTrack.from_track(candidate_track),
            self._indexed_other_tracks(uav_name, tracks),
            self.desired_separation,
        ) is not None

    @staticmethod
    def _indexed_other_tracks(
        uav_name: str,
        tracks: dict[str, TrackPrediction],
    ) -> dict[str, IndexedTrack]:
        return {
            other_name: IndexedTrack.from_track(other_track)
            for other_name, other_track in tracks.items()
            if other_name != uav_name
        }

    @staticmethod
    def _future(separations: list[float]) -> list[float]:
        """Drop the current separation, unless it is the only one."""
        return separations[1:] or separations
