from __future__ import annotations

import math

from musas.airspace.airspace import Airspace
from musas.airspace.geometry import Box3D, Cylinder3D
from musas.algorithms.builtins.conflicts import TacticalAirspaceConflict
from musas.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from musas.domain.flight import (
    Conflict,
    FlightPlan,
    FlightPlanUpdate,
    Waypoint,
)
from musas.domain.tracking import TrackPrediction


class LateralManeuverDeconfliction(TacticalDeconflictionAlgorithm):
    """Resolve tactical airspace conflicts with horizontal maneuvers.

    A restriction ahead is bypassed with two lateral waypoints. If a dynamic
    restriction appears around the UAV, the first waypoint is the nearest
    horizontal exit from its geometry. Every maneuver is expressed as a
    flight-plan update and preserves the current altitude.

    Traffic conflicts are intentionally ignored; they require a traffic-
    specific tactical policy such as ``VerticalManeuverDeconfliction``.
    """

    def __init__(
        self,
        lateral_clearance: float = 10.0,
        cooldown: float = 10.0,
    ) -> None:
        if lateral_clearance < 0:
            raise ValueError("lateral_clearance cannot be negative")
        if cooldown < 0:
            raise ValueError("cooldown cannot be negative")
        self.lateral_clearance = float(lateral_clearance)
        self.cooldown = float(cooldown)
        self._cooldown_until: dict[str, float] = {}

    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[FlightPlanUpdate]:
        del airspace
        updates: list[FlightPlanUpdate] = []
        updated_uavs: set[str] = set()

        airspace_conflicts = sorted(
            (
                conflict
                for conflict in conflicts
                if isinstance(conflict, TacticalAirspaceConflict)
            ),
            key=lambda conflict: conflict.timestamp,
        )
        for conflict in airspace_conflicts:
            uav_name = conflict.uav_name
            if (
                uav_name in updated_uavs
                or uav_name not in tracks
                or uav_name not in flight_plans
            ):
                continue

            track = tracks[uav_name]
            now = float(track.generated_at)
            if now < self._cooldown_until.get(uav_name, float("-inf")):
                continue

            updated_plan = self._updated_plan(
                conflict,
                track,
                flight_plans[uav_name],
            )
            if updated_plan is None:
                continue

            updates.append(FlightPlanUpdate(uav_name, updated_plan))
            updated_uavs.add(uav_name)
            self._cooldown_until[uav_name] = now + self.cooldown

        return updates

    def _updated_plan(
        self,
        conflict: TacticalAirspaceConflict,
        track: TrackPrediction,
        flight_plan: FlightPlan,
    ) -> FlightPlan | None:
        if not flight_plan.waypoint_list:
            return None

        current_state = track.estimated_state
        current = list(current_state.position)
        destination = flight_plan.waypoint_list[-1]
        destination_position = destination.position()
        direction = self._horizontal_direction(current, destination_position)
        if direction is None:
            direction = self._horizontal_direction(
                [0.0, 0.0, 0.0],
                current_state.velocity,
            )
        if direction is None:
            return None

        geometry = conflict.restriction.geometry
        if geometry.contains(current):
            maneuver_positions = self._exit_positions(
                geometry,
                current,
                direction,
            )
        else:
            maneuver_positions = self._detour_positions(
                geometry,
                current,
                destination_position,
                direction,
            )
        if not maneuver_positions:
            return None

        positions = [current, *maneuver_positions, destination_position]
        return self._timed_plan(
            positions,
            now=max(float(track.generated_at), float(current_state.timestamp)),
            original_plan=flight_plan,
            observed_velocity=current_state.velocity,
        )

    def _exit_positions(
        self,
        geometry,
        current: list[float],
        direction: tuple[float, float],
    ) -> list[list[float]]:
        perpendicular = (-direction[1], direction[0])

        if isinstance(geometry, Cylinder3D):
            distance = geometry.radius + self.lateral_clearance
            candidates = []
            for side in (-1.0, 1.0):
                lateral = (
                    perpendicular[0] * distance * side,
                    perpendicular[1] * distance * side,
                )
                candidates.append([
                    [
                        geometry.center_x + lateral[0],
                        geometry.center_y + lateral[1],
                        current[2],
                    ],
                    [
                        geometry.center_x + direction[0] * distance
                        + lateral[0],
                        geometry.center_y + direction[1] * distance
                        + lateral[1],
                        current[2],
                    ],
                ])
            return min(
                candidates,
                key=lambda points: math.dist(current, points[0]),
            )

        if isinstance(geometry, Box3D):
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
            candidates = []
            for side in (-1.0, 1.0):
                lateral = (
                    perpendicular[0] * lateral_clearance * side,
                    perpendicular[1] * lateral_clearance * side,
                )
                candidates.append([
                    [
                        center_x + lateral[0],
                        center_y + lateral[1],
                        current[2],
                    ],
                    [
                        center_x + direction[0] * longitudinal_clearance
                        + lateral[0],
                        center_y + direction[1] * longitudinal_clearance
                        + lateral[1],
                        current[2],
                    ],
                ])
            return min(
                candidates,
                key=lambda points: math.dist(current, points[0]),
            )
        return []

    def _detour_positions(
        self,
        geometry,
        current: list[float],
        destination: list[float],
        direction: tuple[float, float],
    ) -> list[list[float]]:
        perpendicular = (-direction[1], direction[0])

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

        candidates = []
        for side in (-1.0, 1.0):
            lateral = (
                perpendicular[0] * lateral_clearance * side,
                perpendicular[1] * lateral_clearance * side,
            )
            candidates.append([
                [
                    center_x - direction[0] * longitudinal_clearance
                    + lateral[0],
                    center_y - direction[1] * longitudinal_clearance
                    + lateral[1],
                    current[2],
                ],
                [
                    center_x + direction[0] * longitudinal_clearance
                    + lateral[0],
                    center_y + direction[1] * longitudinal_clearance
                    + lateral[1],
                    current[2],
                ],
            ])

        return min(
            candidates,
            key=lambda points: (
                math.dist(current, points[0])
                + math.dist(points[0], points[1])
                + math.dist(points[1], destination)
            ),
        )

    @staticmethod
    def _horizontal_direction(
        start,
        end,
    ) -> tuple[float, float] | None:
        delta_x = float(end[0]) - float(start[0])
        delta_y = float(end[1]) - float(start[1])
        length = math.hypot(delta_x, delta_y)
        if length <= 1e-9:
            return None
        return delta_x / length, delta_y / length

    @staticmethod
    def _timed_plan(
        positions: list[list[float]],
        now: float,
        original_plan: FlightPlan,
        observed_velocity,
    ) -> FlightPlan | None:
        speed = math.sqrt(
            sum(float(component) ** 2 for component in observed_velocity)
        )
        if speed <= 1e-9:
            waypoints = original_plan.waypoint_list
            distance = sum(
                math.dist(start.position(), end.position())
                for start, end in zip(waypoints, waypoints[1:])
            )
            duration = waypoints[-1].t - waypoints[0].t
            if distance <= 0 or duration <= 0:
                return None
            speed = distance / duration

        segment_distances = [
            math.dist(start, end)
            for start, end in zip(positions, positions[1:])
        ]
        natural_duration = sum(segment_distances) / speed
        destination_time = max(
            original_plan.waypoint_list[-1].t,
            now + natural_duration,
        )
        available_duration = destination_time - now
        total_distance = sum(segment_distances)
        if total_distance <= 0 or available_duration <= 0:
            return None

        timestamp = now
        new_waypoints = [Waypoint(*positions[0], timestamp)]
        for end, segment_distance in zip(
            positions[1:],
            segment_distances,
        ):
            timestamp += available_duration * segment_distance / total_distance
            new_waypoints.append(Waypoint(*end, timestamp))
        return FlightPlan(new_waypoints)
