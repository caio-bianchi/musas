from __future__ import annotations

from math import dist, hypot
from typing import Sequence

from musas.airspace.airspace import Airspace
from musas.airspace.geometry import Box3D, Cylinder3D, Geometry
from musas.airspace.restrictions import Geofence, Obstacle
from musas.algorithms.builtins._trajectory import position_at, velocity_at
from musas.algorithms.builtins.conflict_prediction import (
    MinimumSeparationTacticalConflictPrediction,
)
from musas.algorithms.builtins.conflicts import (
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
)
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from musas.domain.flight import (
    Conflict,
    FlightPlan,
    FlightPlanUpdate,
    Waypoint,
)
from musas.domain.tracking import PredictedState, TrackPrediction


AirspaceRestriction = Obstacle | Geofence


class GeometricManeuverDeconfliction(TacticalDeconflictionAlgorithm):
    """Resolve short-term conflicts with deterministic geometric maneuvers.

    The algorithm tests lateral, vertical, and speed-change candidates. A
    candidate is accepted only if the maneuvering UAV is separated from every
    other track and avoids every known obstacle and active geofence.

    For traffic conflicts, the lower-priority flight plan maneuvers. Ties are
    resolved in FCFS order: the plan appearing later in ``flight_plans``
    maneuvers. For airspace conflicts, the affected UAV always maneuvers.

    If a geofence is activated around a UAV, its current prohibited state is
    temporarily tolerated while candidate escape routes are evaluated. The
    selected route must leave the geofence and must not reenter it.
    """

    def __init__(
        self,
        conflict_prediction_algorithm: (
            TacticalConflictPredictionAlgorithm | None
        ) = None,
        minimum_separation_distance: float = 30.0,
        lateral_offset: float = 40.0,
        vertical_offset: float = 40.0,
        airspace_clearance: float = 10.0,
        maneuver_duration: float = 5.0,
        recovery_duration: float = 8.0,
        minimum_speed: float = 1.0,
        maximum_speed: float = 25.0,
        speed_factors: Sequence[float] = (0.6, 1.4),
        cooldown: float = 5.0,
        maximum_escape_distance: float = 500.0,
    ) -> None:
        if minimum_separation_distance <= 0:
            raise ValueError(
                "minimum_separation_distance must be greater than zero"
            )
        if lateral_offset <= 0 or vertical_offset <= 0:
            raise ValueError("maneuver offsets must be greater than zero")
        if airspace_clearance <= 0:
            raise ValueError("airspace_clearance must be greater than zero")
        if maneuver_duration <= 0 or recovery_duration <= 0:
            raise ValueError("maneuver durations must be greater than zero")
        if minimum_speed <= 0 or maximum_speed < minimum_speed:
            raise ValueError("invalid tactical speed limits")
        if not speed_factors or any(factor <= 0 for factor in speed_factors):
            raise ValueError("speed_factors must contain positive values")
        if cooldown < 0:
            raise ValueError("cooldown cannot be negative")
        if maximum_escape_distance <= 0:
            raise ValueError(
                "maximum_escape_distance must be greater than zero"
            )

        self.minimum_separation_distance = float(
            minimum_separation_distance
        )
        self.conflict_prediction_algorithm = (
            conflict_prediction_algorithm
            if conflict_prediction_algorithm is not None
            else MinimumSeparationTacticalConflictPrediction(
                minimum_separation_distance=minimum_separation_distance,
            )
        )
        self.lateral_offset = float(lateral_offset)
        self.vertical_offset = float(vertical_offset)
        self.airspace_clearance = float(airspace_clearance)
        self.maneuver_duration = float(maneuver_duration)
        self.recovery_duration = float(recovery_duration)
        self.minimum_speed = float(minimum_speed)
        self.maximum_speed = float(maximum_speed)
        self.speed_factors = tuple(float(item) for item in speed_factors)
        self.cooldown = float(cooldown)
        self.maximum_escape_distance = float(maximum_escape_distance)
        self._last_update_at: dict[str, float] = {}
        self._escape_maneuver_until: dict[str, float] = {}

    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[FlightPlanUpdate]:
        if not conflicts:
            return []

        working_tracks = dict(tracks)
        order = {
            uav_name: index
            for index, uav_name in enumerate(flight_plans)
        }
        updates: dict[str, FlightPlanUpdate] = {}

        for conflict in sorted(
            conflicts,
            key=lambda item: float(getattr(item, "timestamp", 0.0)),
        ):
            uav_name = self._maneuvering_uav(
                conflict,
                flight_plans,
                order,
            )
            if (
                uav_name is None
                or uav_name in updates
                or uav_name not in working_tracks
                or uav_name not in flight_plans
            ):
                continue

            track = working_tracks[uav_name]
            current_time = float(track.estimated_state.timestamp)
            escaping = self._is_inside_conflicting_restriction(
                uav_name,
                conflict,
                track.estimated_state.position,
                current_time,
            )
            last_update = self._last_update_at.get(uav_name)
            if (
                escaping
                and current_time
                < self._escape_maneuver_until.get(uav_name, float("-inf"))
            ):
                continue
            if (
                not escaping
                and last_update is not None
                and current_time - last_update < self.cooldown
            ):
                continue

            result = self._resolve_for_uav(
                uav_name,
                conflict,
                working_tracks,
                flight_plans[uav_name],
                airspace,
            )
            if result is None:
                continue

            flight_plan, updated_track = result
            updates[uav_name] = FlightPlanUpdate(uav_name, flight_plan)
            working_tracks[uav_name] = updated_track
            self._last_update_at[uav_name] = current_time
            if escaping:
                self._escape_maneuver_until[uav_name] = (
                    flight_plan.waypoint_list[1].t
                )

        return list(updates.values())

    def _resolve_for_uav(
        self,
        uav_name: str,
        conflict: Conflict,
        tracks: dict[str, TrackPrediction],
        original_plan: FlightPlan,
        airspace: Airspace,
    ) -> tuple[FlightPlan, TrackPrediction] | None:
        track = tracks[uav_name]
        candidates = self._candidate_plans(
            conflict,
            track,
            original_plan,
        )
        valid: list[tuple[float, FlightPlan, TrackPrediction]] = []
        for candidate in candidates:
            candidate_track = self._track_for_plan(track, candidate)
            if self._candidate_is_valid(
                uav_name,
                candidate_track,
                tracks,
                airspace,
            ):
                valid.append(
                    (
                        self._candidate_cost(candidate, original_plan),
                        candidate,
                        candidate_track,
                    )
                )

        if not valid:
            return None
        _, flight_plan, candidate_track = min(
            valid,
            key=lambda item: item[0],
        )
        return flight_plan, candidate_track

    def _candidate_plans(
        self,
        conflict: Conflict,
        track: TrackPrediction,
        original_plan: FlightPlan,
    ) -> list[FlightPlan]:
        state = track.estimated_state
        current = [float(value) for value in state.position]
        current_time = float(state.timestamp)
        direction = self._horizontal_direction(state.velocity)
        if direction is None:
            goal = original_plan.waypoint_list[-1].position()
            direction = self._horizontal_direction(
                [goal[0] - current[0], goal[1] - current[1], 0.0]
            )
        if direction is None:
            direction = (1.0, 0.0)

        targets: list[list[float]] = []
        restriction = (
            conflict.restriction
            if isinstance(conflict, TacticalAirspaceConflict)
            else None
        )
        if restriction is not None:
            if self._restriction_contains(
                restriction,
                current,
                current_time,
            ):
                targets.extend(
                    self._escape_targets(
                        current,
                        current_time,
                        restriction,
                        direction,
                    )
                )
            else:
                targets.extend(
                    self._restriction_detour_targets(
                        current,
                        restriction.geometry,
                        direction,
                    )
                )

        forward_distance = self.maximum_speed * self.maneuver_duration
        perpendicular = (-direction[1], direction[0])
        for side in (-1.0, 1.0):
            targets.append([
                current[0] + direction[0] * forward_distance
                + side * perpendicular[0] * self.lateral_offset,
                current[1] + direction[1] * forward_distance
                + side * perpendicular[1] * self.lateral_offset,
                current[2],
            ])
        for vertical_direction in (-1.0, 1.0):
            targets.append([
                current[0] + direction[0] * forward_distance,
                current[1] + direction[1] * forward_distance,
                current[2] + vertical_direction * self.vertical_offset,
            ])

        current_speed = self._norm(state.velocity)
        nominal_speed = min(
            self.maximum_speed,
            max(self.minimum_speed, current_speed),
        )
        for factor in self.speed_factors:
            distance_at_speed = (
                min(
                    self.maximum_speed,
                    max(self.minimum_speed, nominal_speed * factor),
                )
                * self.maneuver_duration
            )
            targets.append([
                current[0] + direction[0] * distance_at_speed,
                current[1] + direction[1] * distance_at_speed,
                current[2],
            ])

        plans: list[FlightPlan] = []
        seen_targets: set[tuple[float, float, float]] = set()
        for target in targets:
            target_key = tuple(round(value, 6) for value in target)
            if target_key in seen_targets or dist(current, target) <= 1e-9:
                continue
            seen_targets.add(target_key)
            candidate = self._build_candidate_plan(
                original_plan,
                current,
                current_time,
                target,
            )
            if candidate is not None:
                plans.append(candidate)
        return plans

    def _build_candidate_plan(
        self,
        original_plan: FlightPlan,
        current: list[float],
        current_time: float,
        maneuver_target: list[float],
    ) -> FlightPlan | None:
        maneuver_travel_time = max(
            self.maneuver_duration,
            dist(current, maneuver_target) / self.maximum_speed,
        )
        maneuver_time = current_time + maneuver_travel_time
        desired_rejoin_time = maneuver_time + self.recovery_duration
        original_end_time = original_plan.waypoint_list[-1].t
        sampled_rejoin_time = min(desired_rejoin_time, original_end_time)
        rejoin_position = position_at(original_plan, sampled_rejoin_time)
        recovery_time = max(
            self.recovery_duration,
            dist(maneuver_target, rejoin_position) / self.maximum_speed,
        )
        rejoin_time = maneuver_time + recovery_time

        waypoints = [
            Waypoint(*current, current_time),
            Waypoint(*maneuver_target, maneuver_time),
            Waypoint(*rejoin_position, rejoin_time),
        ]
        waypoints.extend(
            Waypoint(item.x, item.y, item.z, item.t)
            for item in original_plan.waypoint_list
            if item.t > rejoin_time
        )
        waypoints = self._without_duplicate_waypoints(waypoints)
        if len(waypoints) < 2:
            return None
        return FlightPlan(
            waypoint_list=waypoints,
            priority=original_plan.priority,
        )

    def _track_for_plan(
        self,
        original_track: TrackPrediction,
        flight_plan: FlightPlan,
    ) -> TrackPrediction:
        start_time = float(original_track.estimated_state.timestamp)
        timestamps = {
            float(state.timestamp)
            for state in original_track.predicted_states
            if state.timestamp > start_time
        }
        timestamps.update(
            waypoint.t
            for waypoint in flight_plan.waypoint_list[1:]
            if waypoint.t > start_time
        )
        predicted_states = tuple(
            PredictedState(
                position=position_at(flight_plan, timestamp),
                velocity=velocity_at(flight_plan, timestamp),
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

    def _candidate_is_valid(
        self,
        uav_name: str,
        candidate_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> bool:
        working_tracks = {**tracks, uav_name: candidate_track}
        conflicts = self.conflict_prediction_algorithm.predict(
            tracks=working_tracks,
            airspace=airspace,
        )
        for conflict in conflicts:
            if not self._involves(conflict, uav_name):
                continue
            if isinstance(conflict, TacticalTrafficConflict):
                other_name = (
                    conflict.other_uav
                    if conflict.uav_name == uav_name
                    else conflict.uav_name
                )
                other_track = tracks.get(other_name)
                if (
                    other_track is None
                    or not self._starts_in_loss_of_separation(
                        candidate_track,
                        other_track,
                    )
                ):
                    return False
            elif isinstance(conflict, TacticalAirspaceConflict):
                if not self._restriction_contains(
                    conflict.restriction,
                    candidate_track.estimated_state.position,
                    candidate_track.estimated_state.timestamp,
                ):
                    return False
            else:
                return False

        if not self._traffic_recovery_is_valid(
            uav_name,
            candidate_track,
            tracks,
        ):
            return False
        return self._airspace_recovery_is_valid(candidate_track, airspace)

    def _traffic_recovery_is_valid(
        self,
        uav_name: str,
        candidate_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
    ) -> bool:
        candidate_states = self._states(candidate_track)
        maximum_time_difference = float(getattr(
            self.conflict_prediction_algorithm,
            "maximum_time_difference",
            0.5,
        ))
        minimum_separation = float(getattr(
            self.conflict_prediction_algorithm,
            "minimum_separation_distance",
            self.minimum_separation_distance,
        ))

        for other_name, other_track in tracks.items():
            if other_name == uav_name:
                continue
            other_states = self._states(other_track)
            if not other_states:
                continue
            separations: list[float] = []
            for state in candidate_states:
                other = min(
                    other_states,
                    key=lambda item: abs(item.timestamp - state.timestamp),
                )
                if (
                    abs(other.timestamp - state.timestamp)
                    <= maximum_time_difference
                ):
                    separations.append(dist(state.position, other.position))
            if not separations:
                continue

            starts_inside = separations[0] < minimum_separation
            if starts_inside:
                if (
                    not any(
                        value >= minimum_separation
                        for value in separations[1:]
                    )
                    or separations[-1] < minimum_separation
                ):
                    return False
            elif any(
                value < minimum_separation
                for value in separations[1:]
            ):
                return False
        return True

    def _airspace_recovery_is_valid(
        self,
        candidate_track: TrackPrediction,
        airspace: Airspace,
    ) -> bool:
        states = self._states(candidate_track)
        restrictions: tuple[AirspaceRestriction, ...] = (
            *airspace.obstacles.values(),
            *airspace.geofences.values(),
        )
        for restriction in restrictions:
            prohibited = [
                self._restriction_contains(
                    restriction,
                    state.position,
                    state.timestamp,
                )
                for state in states
            ]
            if not prohibited[0]:
                if any(prohibited[1:]):
                    return False
                continue

            try:
                first_clear = prohibited.index(False, 1)
            except ValueError:
                return False
            if any(prohibited[first_clear + 1:]):
                return False
        return True

    def _escape_targets(
        self,
        current: list[float],
        timestamp: float,
        restriction: AirspaceRestriction,
        direction: tuple[float, float],
    ) -> list[list[float]]:
        geometry = restriction.geometry
        targets: list[list[float]] = []
        clearance = self.airspace_clearance
        if isinstance(geometry, Cylinder3D):
            radial_x = current[0] - geometry.center_x
            radial_y = current[1] - geometry.center_y
            radial_norm = hypot(radial_x, radial_y)
            if radial_norm <= 1e-9:
                radial_x, radial_y = direction
                radial_norm = 1.0
            targets.extend([
                [
                    geometry.center_x
                    + radial_x / radial_norm
                    * (geometry.radius + clearance),
                    geometry.center_y
                    + radial_y / radial_norm
                    * (geometry.radius + clearance),
                    current[2],
                ],
                [current[0], current[1], geometry.maximum_z + clearance],
                [current[0], current[1], geometry.minimum_z - clearance],
            ])
        elif isinstance(geometry, Box3D):
            targets.extend([
                [geometry.minimum_x - clearance, current[1], current[2]],
                [geometry.maximum_x + clearance, current[1], current[2]],
                [current[0], geometry.minimum_y - clearance, current[2]],
                [current[0], geometry.maximum_y + clearance, current[2]],
                [current[0], current[1], geometry.minimum_z - clearance],
                [current[0], current[1], geometry.maximum_z + clearance],
            ])
        else:
            targets.extend(
                self._sampled_escape_targets(
                    current,
                    timestamp,
                    restriction,
                )
            )
        return [
            target
            for target in targets
            if not self._restriction_contains(
                restriction,
                target,
                timestamp,
            )
        ]

    def _sampled_escape_targets(
        self,
        current: list[float],
        timestamp: float,
        restriction: AirspaceRestriction,
    ) -> list[list[float]]:
        directions = (
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        )
        results = []
        for direction in directions:
            distance_from_start = self.airspace_clearance
            while distance_from_start <= self.maximum_escape_distance:
                candidate = [
                    current[axis] + direction[axis] * distance_from_start
                    for axis in range(3)
                ]
                if not self._restriction_contains(
                    restriction,
                    candidate,
                    timestamp,
                ):
                    results.append(candidate)
                    break
                distance_from_start += self.airspace_clearance
        return results

    def _restriction_detour_targets(
        self,
        current: list[float],
        geometry: Geometry,
        direction: tuple[float, float],
    ) -> list[list[float]]:
        perpendicular = (-direction[1], direction[0])
        if isinstance(geometry, Cylinder3D):
            center_x, center_y = geometry.center_x, geometry.center_y
            lateral_extent = geometry.radius + self.airspace_clearance
        elif isinstance(geometry, Box3D):
            center_x = (geometry.minimum_x + geometry.maximum_x) / 2.0
            center_y = (geometry.minimum_y + geometry.maximum_y) / 2.0
            half_x = (geometry.maximum_x - geometry.minimum_x) / 2.0
            half_y = (geometry.maximum_y - geometry.minimum_y) / 2.0
            lateral_extent = (
                abs(perpendicular[0]) * half_x
                + abs(perpendicular[1]) * half_y
                + self.airspace_clearance
            )
        else:
            return []

        return [
            [
                center_x + side * perpendicular[0] * lateral_extent,
                center_y + side * perpendicular[1] * lateral_extent,
                current[2],
            ]
            for side in (-1.0, 1.0)
        ] + [
            [
                current[0],
                current[1],
                geometry.maximum_z + self.airspace_clearance,
            ],
            [
                current[0],
                current[1],
                geometry.minimum_z - self.airspace_clearance,
            ],
        ]

    @staticmethod
    def _maneuvering_uav(
        conflict: Conflict,
        flight_plans: dict[str, FlightPlan],
        order: dict[str, int],
    ) -> str | None:
        if isinstance(conflict, TacticalAirspaceConflict):
            return conflict.uav_name
        if not isinstance(conflict, TacticalTrafficConflict):
            return None

        first, second = conflict.uav_name, conflict.other_uav
        first_plan = flight_plans.get(first)
        second_plan = flight_plans.get(second)
        first_priority = first_plan.priority if first_plan is not None else 0
        second_priority = (
            second_plan.priority if second_plan is not None else 0
        )
        if first_priority < second_priority:
            return first
        if second_priority < first_priority:
            return second
        first_order = order.get(first, -1)
        second_order = order.get(second, -1)
        if first_order != second_order:
            return first if first_order > second_order else second
        return max(first, second)

    @staticmethod
    def _is_inside_conflicting_restriction(
        uav_name: str,
        conflict: Conflict,
        position: Sequence[float],
        timestamp: float,
    ) -> bool:
        return (
            isinstance(conflict, TacticalAirspaceConflict)
            and conflict.uav_name == uav_name
            and GeometricManeuverDeconfliction._restriction_contains(
                conflict.restriction,
                position,
                timestamp,
            )
        )

    @staticmethod
    def _restriction_contains(
        restriction: AirspaceRestriction,
        position: Sequence[float],
        timestamp: float,
    ) -> bool:
        if isinstance(restriction, Geofence):
            return restriction.prohibits(position, timestamp)
        return restriction.contains(position)

    @staticmethod
    def _involves(conflict: Conflict, uav_name: str) -> bool:
        if isinstance(conflict, TacticalTrafficConflict):
            return uav_name in (conflict.uav_name, conflict.other_uav)
        if isinstance(conflict, TacticalAirspaceConflict):
            return conflict.uav_name == uav_name
        return False

    def _starts_in_loss_of_separation(
        self,
        first: TrackPrediction,
        second: TrackPrediction,
    ) -> bool:
        minimum_separation = float(getattr(
            self.conflict_prediction_algorithm,
            "minimum_separation_distance",
            self.minimum_separation_distance,
        ))
        return (
            dist(
                first.estimated_state.position,
                second.estimated_state.position,
            )
            < minimum_separation
        )

    @staticmethod
    def _states(track: TrackPrediction) -> tuple[PredictedState, ...]:
        return (track.estimated_state, *track.predicted_states)

    @staticmethod
    def _horizontal_direction(
        vector: Sequence[float],
    ) -> tuple[float, float] | None:
        length = hypot(float(vector[0]), float(vector[1]))
        if length <= 1e-9:
            return None
        return float(vector[0]) / length, float(vector[1]) / length

    @staticmethod
    def _norm(vector: Sequence[float]) -> float:
        return sum(float(value) ** 2 for value in vector) ** 0.5

    @staticmethod
    def _without_duplicate_waypoints(
        waypoints: list[Waypoint],
    ) -> list[Waypoint]:
        result: list[Waypoint] = []
        for waypoint in waypoints:
            if (
                result
                and (
                    waypoint.t <= result[-1].t
                    or dist(
                        waypoint.position(),
                        result[-1].position(),
                    ) <= 1e-9
                )
            ):
                continue
            result.append(waypoint)
        return result

    @staticmethod
    def _candidate_cost(
        candidate: FlightPlan,
        original: FlightPlan,
    ) -> float:
        candidate_distance = sum(
            dist(start.position(), end.position())
            for start, end in zip(
                candidate.waypoint_list,
                candidate.waypoint_list[1:],
            )
        )
        original_end = original.waypoint_list[-1].t
        delay = max(0.0, candidate.waypoint_list[-1].t - original_end)
        return candidate_distance + delay
