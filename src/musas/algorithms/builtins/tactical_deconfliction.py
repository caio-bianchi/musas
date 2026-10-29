from __future__ import annotations

from collections.abc import Iterable
from math import ceil, dist, hypot
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

    For traffic conflicts, the algorithm tests lateral, vertical, and
    speed-change candidates. For airspace conflicts, it builds lateral and
    vertical routes around the restriction with the configured clearance.
    A candidate is accepted only if the maneuvering UAV is separated from
    every other track and every route segment avoids known restrictions.

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
        allow_lateral: bool = True,
        allow_vertical: bool = True,
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
        if not allow_lateral and not allow_vertical:
            raise ValueError(
                "at least one geometric maneuver must be enabled"
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
        self.allow_lateral = allow_lateral
        self.allow_vertical = allow_vertical
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
                if not (
                    isinstance(conflict, TacticalAirspaceConflict)
                    and not self._flight_plan_avoids_airspace(
                        flight_plans[uav_name],
                        current_time,
                        airspace,
                    )
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
                candidate,
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
                targets = self._escape_targets(
                    current,
                    current_time,
                    restriction,
                    direction,
                )
                return self._plans_for_routes(
                    original_plan,
                    current,
                    current_time,
                    ([target] for target in targets),
                )
            return self._plans_for_routes(
                original_plan,
                current,
                current_time,
                self._restriction_detour_routes(
                    current,
                    restriction.geometry,
                    direction,
                ),
            )

        routes: list[list[list[float]]] = []
        perpendicular = (-direction[1], direction[0])
        if isinstance(conflict, TacticalTrafficConflict):
            conflict_position = (
                conflict.position
                if conflict.uav_name == track.uav_name
                else conflict.other_position
            )
        else:
            forward_distance = self.maximum_speed * self.maneuver_duration
            conflict_position = (
                current[0] + direction[0] * forward_distance,
                current[1] + direction[1] * forward_distance,
                current[2],
            )
        longitudinal_extent = self.minimum_separation_distance

        if self.allow_lateral:
            for side in (-1.0, 1.0):
                routes.append([
                    [
                        conflict_position[0]
                        - direction[0] * longitudinal_extent
                        + side * perpendicular[0] * self.lateral_offset,
                        conflict_position[1]
                        - direction[1] * longitudinal_extent
                        + side * perpendicular[1] * self.lateral_offset,
                        current[2],
                    ],
                    [
                        conflict_position[0]
                        + direction[0] * longitudinal_extent
                        + side * perpendicular[0] * self.lateral_offset,
                        conflict_position[1]
                        + direction[1] * longitudinal_extent
                        + side * perpendicular[1] * self.lateral_offset,
                        current[2],
                    ],
                ])
        if self.allow_vertical:
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
            routes.append([[
                current[0] + direction[0] * distance_at_speed,
                current[1] + direction[1] * distance_at_speed,
                current[2],
            ]])

        return self._plans_for_routes(
            original_plan,
            current,
            current_time,
            routes,
        )

    def _plans_for_routes(
        self,
        original_plan: FlightPlan,
        current: list[float],
        current_time: float,
        routes: Iterable[Sequence[Sequence[float]]],
    ) -> list[FlightPlan]:
        plans: list[FlightPlan] = []
        seen_routes: set[tuple[float, ...]] = set()
        for route in routes:
            targets = [list(target) for target in route]
            if not targets or dist(current, targets[0]) <= 1e-9:
                continue
            route_key = tuple(
                round(value, 6)
                for target in targets
                for value in target
            )
            if route_key in seen_routes:
                continue
            seen_routes.add(route_key)
            candidate = self._build_candidate_plan(
                original_plan,
                current,
                current_time,
                targets,
            )
            if candidate is not None:
                plans.append(candidate)
        return plans

    def _build_candidate_plan(
        self,
        original_plan: FlightPlan,
        current: list[float],
        current_time: float,
        maneuver_targets: list[list[float]],
    ) -> FlightPlan | None:
        if not maneuver_targets:
            return None

        maneuver_waypoints: list[Waypoint] = []
        previous = current
        maneuver_time = current_time
        for index, target in enumerate(maneuver_targets):
            travel_time = dist(previous, target) / self.maximum_speed
            if index == 0:
                travel_time = max(self.maneuver_duration, travel_time)
            maneuver_time += travel_time
            maneuver_waypoints.append(Waypoint(*target, maneuver_time))
            previous = target

        desired_rejoin_time = maneuver_time + self.recovery_duration
        original_end_time = original_plan.waypoint_list[-1].t
        sampled_rejoin_time = min(desired_rejoin_time, original_end_time)
        rejoin_position = position_at(original_plan, sampled_rejoin_time)
        recovery_time = max(
            self.recovery_duration,
            dist(previous, rejoin_position) / self.maximum_speed,
        )
        rejoin_time = maneuver_time + recovery_time

        waypoints = [
            Waypoint(*current, current_time),
            *maneuver_waypoints,
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
        candidate_plan: FlightPlan,
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
        return self._airspace_recovery_is_valid(
            candidate_track,
            candidate_plan,
            airspace,
        )

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
        candidate_plan: FlightPlan,
        airspace: Airspace,
    ) -> bool:
        restrictions: tuple[AirspaceRestriction, ...] = (
            *airspace.obstacles.values(),
            *airspace.geofences.values(),
        )
        for restriction in restrictions:
            if not self._route_avoids_restriction(
                candidate_plan,
                restriction,
                allow_initial_escape=self._restriction_contains_with_clearance(
                    restriction,
                    candidate_track.estimated_state.position,
                    candidate_track.estimated_state.timestamp,
                ),
            ):
                return False
        return True

    def _flight_plan_avoids_airspace(
        self,
        flight_plan: FlightPlan,
        current_time: float,
        airspace: Airspace,
    ) -> bool:
        remaining_plan = self._remaining_plan(flight_plan, current_time)
        start = remaining_plan.waypoint_list[0]
        return all(
            self._route_avoids_restriction(
                remaining_plan,
                restriction,
                allow_initial_escape=self._restriction_contains_with_clearance(
                    restriction,
                    start.position(),
                    start.t,
                ),
            )
            for restriction in (
                *airspace.obstacles.values(),
                *airspace.geofences.values(),
            )
        )

    @staticmethod
    def _remaining_plan(
        flight_plan: FlightPlan,
        current_time: float,
    ) -> FlightPlan:
        return FlightPlan(
            waypoint_list=[
                Waypoint(*position_at(flight_plan, current_time), current_time),
                *(
                    Waypoint(item.x, item.y, item.z, item.t)
                    for item in flight_plan.waypoint_list
                    if item.t > current_time
                ),
            ],
            priority=flight_plan.priority,
        )

    def _route_avoids_restriction(
        self,
        flight_plan: FlightPlan,
        restriction: AirspaceRestriction,
        allow_initial_escape: bool,
    ) -> bool:
        waypoints = flight_plan.waypoint_list
        if len(waypoints) < 2:
            return not self._restriction_contains_with_clearance(
                restriction,
                waypoints[0].position(),
                waypoints[0].t,
            )

        first_segment = 0
        if allow_initial_escape:
            first_clear = next(
                (
                    index
                    for index, waypoint in enumerate(waypoints[1:], 1)
                    if not self._restriction_contains_with_clearance(
                        restriction,
                        waypoint.position(),
                        waypoint.t,
                    )
                ),
                None,
            )
            if first_clear is None:
                return False
            first_segment = first_clear

        return not any(
            self._segment_intersects_restriction(start, end, restriction)
            for start, end in zip(
                waypoints[first_segment:],
                waypoints[first_segment + 1:],
            )
        )

    def _segment_intersects_restriction(
        self,
        start: Waypoint,
        end: Waypoint,
        restriction: AirspaceRestriction,
    ) -> bool:
        start_time = start.t
        end_time = end.t
        if end_time <= start_time:
            return self._restriction_contains_with_clearance(
                restriction,
                start.position(),
                start_time,
            )

        active_start = start_time
        active_end = end_time
        if isinstance(restriction, Geofence):
            if restriction.active_from is not None:
                active_start = max(active_start, restriction.active_from)
            if restriction.active_until is not None:
                active_end = min(active_end, restriction.active_until)
            if active_end < active_start:
                return False

        duration = end_time - start_time
        start_fraction = (active_start - start_time) / duration
        end_fraction = (active_end - start_time) / duration
        start_position = self._interpolate_segment(
            start.position(),
            end.position(),
            start_fraction,
        )
        end_position = self._interpolate_segment(
            start.position(),
            end.position(),
            end_fraction,
        )
        return self._segment_intersects_geometry(
            start_position,
            end_position,
            restriction.geometry,
        )

    def _segment_intersects_geometry(
        self,
        start: Sequence[float],
        end: Sequence[float],
        geometry: Geometry,
    ) -> bool:
        clearance = self.airspace_clearance
        if isinstance(geometry, Box3D):
            minimums = (
                geometry.minimum_x - clearance,
                geometry.minimum_y - clearance,
                geometry.minimum_z - clearance,
            )
            maximums = (
                geometry.maximum_x + clearance,
                geometry.maximum_y + clearance,
                geometry.maximum_z + clearance,
            )
            return self._segment_intersects_box(
                start,
                end,
                minimums,
                maximums,
            )
        if isinstance(geometry, Cylinder3D):
            return self._segment_intersects_cylinder(
                start,
                end,
                geometry.center_x,
                geometry.center_y,
                geometry.radius + clearance,
                geometry.minimum_z - clearance,
                geometry.maximum_z + clearance,
            )

        segment_length = dist(start, end)
        sample_step = max(clearance / 2.0, 1e-3)
        sample_count = max(1, ceil(segment_length / sample_step))
        return any(
            geometry.contains(
                self._interpolate_segment(start, end, index / sample_count)
            )
            for index in range(sample_count + 1)
        )

    def _restriction_contains_with_clearance(
        self,
        restriction: AirspaceRestriction,
        position: Sequence[float],
        timestamp: float,
    ) -> bool:
        if (
            isinstance(restriction, Geofence)
            and not restriction.is_active_at(timestamp)
        ):
            return False
        geometry = restriction.geometry
        clearance = self.airspace_clearance
        x, y, z = (float(value) for value in position)
        if isinstance(geometry, Box3D):
            return (
                geometry.minimum_x - clearance
                <= x
                <= geometry.maximum_x + clearance
                and geometry.minimum_y - clearance
                <= y
                <= geometry.maximum_y + clearance
                and geometry.minimum_z - clearance
                <= z
                <= geometry.maximum_z + clearance
            )
        if isinstance(geometry, Cylinder3D):
            return (
                hypot(x - geometry.center_x, y - geometry.center_y)
                <= geometry.radius + clearance
                and geometry.minimum_z - clearance
                <= z
                <= geometry.maximum_z + clearance
            )
        return geometry.contains(position)

    @staticmethod
    def _segment_intersects_box(
        start: Sequence[float],
        end: Sequence[float],
        minimums: Sequence[float],
        maximums: Sequence[float],
    ) -> bool:
        lower, upper = 0.0, 1.0
        for initial, final, minimum, maximum in zip(
            start,
            end,
            minimums,
            maximums,
        ):
            change = final - initial
            if abs(change) <= 1e-12:
                if initial < minimum or initial > maximum:
                    return False
                continue
            first = (minimum - initial) / change
            second = (maximum - initial) / change
            entry, exit_ = sorted((first, second))
            lower = max(lower, entry)
            upper = min(upper, exit_)
            if lower > upper:
                return False
        return True

    @staticmethod
    def _segment_intersects_cylinder(
        start: Sequence[float],
        end: Sequence[float],
        center_x: float,
        center_y: float,
        radius: float,
        minimum_z: float,
        maximum_z: float,
    ) -> bool:
        lower, upper = 0.0, 1.0
        delta_z = end[2] - start[2]
        if abs(delta_z) <= 1e-12:
            if start[2] < minimum_z or start[2] > maximum_z:
                return False
        else:
            first = (minimum_z - start[2]) / delta_z
            second = (maximum_z - start[2]) / delta_z
            entry, exit_ = sorted((first, second))
            lower = max(lower, entry)
            upper = min(upper, exit_)
            if lower > upper:
                return False

        start_x = start[0] - center_x
        start_y = start[1] - center_y
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        squared_speed = delta_x * delta_x + delta_y * delta_y
        if squared_speed <= 1e-12:
            closest = lower
        else:
            closest = -(
                start_x * delta_x + start_y * delta_y
            ) / squared_speed
            closest = min(upper, max(lower, closest))
        x = start_x + closest * delta_x
        y = start_y + closest * delta_y
        return x * x + y * y <= radius * radius

    @staticmethod
    def _interpolate_segment(
        start: Sequence[float],
        end: Sequence[float],
        fraction: float,
    ) -> list[float]:
        return [
            initial + fraction * (final - initial)
            for initial, final in zip(start, end)
        ]

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
        escape_extent = clearance + max(1e-3, clearance * 0.01)
        if isinstance(geometry, Cylinder3D):
            radial_x = current[0] - geometry.center_x
            radial_y = current[1] - geometry.center_y
            radial_norm = hypot(radial_x, radial_y)
            if radial_norm <= 1e-9:
                radial_x, radial_y = direction
                radial_norm = 1.0
            if self.allow_lateral:
                targets.append([
                    geometry.center_x
                    + radial_x / radial_norm
                    * (geometry.radius + escape_extent),
                    geometry.center_y
                    + radial_y / radial_norm
                    * (geometry.radius + escape_extent),
                    current[2],
                ])
            if self.allow_vertical:
                targets.extend([
                    [current[0], current[1], geometry.maximum_z + escape_extent],
                    [current[0], current[1], geometry.minimum_z - escape_extent],
                ])
        elif isinstance(geometry, Box3D):
            if self.allow_lateral:
                targets.extend([
                    [geometry.minimum_x - escape_extent, current[1], current[2]],
                    [geometry.maximum_x + escape_extent, current[1], current[2]],
                    [current[0], geometry.minimum_y - escape_extent, current[2]],
                    [current[0], geometry.maximum_y + escape_extent, current[2]],
                ])
            if self.allow_vertical:
                targets.extend([
                    [current[0], current[1], geometry.minimum_z - escape_extent],
                    [current[0], current[1], geometry.maximum_z + escape_extent],
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
        directions: list[tuple[float, float, float]] = []
        if self.allow_lateral:
            directions.extend([
                (1.0, 0.0, 0.0),
                (-1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, -1.0, 0.0),
            ])
        if self.allow_vertical:
            directions.extend([
                (0.0, 0.0, 1.0),
                (0.0, 0.0, -1.0),
            ])
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

    def _restriction_detour_routes(
        self,
        current: list[float],
        geometry: Geometry,
        direction: tuple[float, float],
    ) -> list[list[list[float]]]:
        perpendicular = (-direction[1], direction[0])
        clearance = self.airspace_clearance
        # Keep detour segments just beyond the configured clearance boundary,
        # so numerical equality cannot be interpreted as an intersection.
        padding = max(1e-3, clearance * 0.01)
        if isinstance(geometry, Cylinder3D):
            center_x, center_y = geometry.center_x, geometry.center_y
            longitudinal_extent = geometry.radius + clearance + padding
            lateral_extent = geometry.radius + clearance + padding
        elif isinstance(geometry, Box3D):
            center_x = (geometry.minimum_x + geometry.maximum_x) / 2.0
            center_y = (geometry.minimum_y + geometry.maximum_y) / 2.0
            half_x = (geometry.maximum_x - geometry.minimum_x) / 2.0
            half_y = (geometry.maximum_y - geometry.minimum_y) / 2.0
            longitudinal_extent = (
                abs(direction[0]) * half_x
                + abs(direction[1]) * half_y
                + clearance
                + padding
            )
            lateral_extent = (
                abs(perpendicular[0]) * half_x
                + abs(perpendicular[1]) * half_y
                + clearance
                + padding
            )
        else:
            return []

        routes = []
        if self.allow_lateral:
            for side in (-1.0, 1.0):
                routes.append([
                    [
                        center_x - direction[0] * longitudinal_extent
                        + side * perpendicular[0] * lateral_extent,
                        center_y - direction[1] * longitudinal_extent
                        + side * perpendicular[1] * lateral_extent,
                        current[2],
                    ],
                    [
                        center_x + direction[0] * longitudinal_extent
                        + side * perpendicular[0] * lateral_extent,
                        center_y + direction[1] * longitudinal_extent
                        + side * perpendicular[1] * lateral_extent,
                        current[2],
                    ],
                ])

        if self.allow_vertical:
            routes.extend([
                [
                    [
                        center_x - direction[0] * longitudinal_extent,
                        center_y - direction[1] * longitudinal_extent,
                        geometry.maximum_z + clearance + padding,
                    ],
                    [
                        center_x + direction[0] * longitudinal_extent,
                        center_y + direction[1] * longitudinal_extent,
                        geometry.maximum_z + clearance + padding,
                    ],
                ],
                [
                    [
                        center_x - direction[0] * longitudinal_extent,
                        center_y - direction[1] * longitudinal_extent,
                        geometry.minimum_z - clearance - padding,
                    ],
                    [
                        center_x + direction[0] * longitudinal_extent,
                        center_y + direction[1] * longitudinal_extent,
                        geometry.minimum_z - clearance - padding,
                    ],
                ],
            ])
        return routes

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
