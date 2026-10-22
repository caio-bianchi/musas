from __future__ import annotations

from itertools import combinations
from math import dist

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins._trajectory import position_at, sample_times
from musas.algorithms.builtins.conflicts import (
    StrategicAirspaceConflict,
    StrategicTrafficConflict,
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
)
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.domain.flight import Conflict, FlightPlan
from musas.domain.tracking import PredictedState, TrackPrediction


class MinimumSeparationConflictPrediction(ConflictPredictionAlgorithm):
    """Predict strategic conflicts by sampling 4-D flight plans.

    Separation is spherical: a traffic conflict exists whenever the Euclidean
    distance between two sampled positions is smaller than
    ``minimum_separation_distance``. The first conflict with each aircraft or
    airspace restriction is returned.
    """

    def __init__(
        self,
        minimum_separation_distance: float = 30.0,
        sampling_interval: float = 0.5,
    ) -> None:
        if minimum_separation_distance <= 0:
            raise ValueError(
                "minimum_separation_distance must be greater than zero"
            )
        if sampling_interval <= 0:
            raise ValueError("sampling_interval must be greater than zero")
        self.minimum_separation_distance = float(
            minimum_separation_distance
        )
        self.sampling_interval = float(sampling_interval)

    def predict(
        self,
        proposed_flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[Conflict]:
        self._require_waypoints(proposed_flight_plan)
        conflicts: list[Conflict] = []

        for uav_name, approved_plan in approved_flight_plans.items():
            self._require_waypoints(approved_plan)
            conflict = self._traffic_conflict(
                proposed_flight_plan,
                approved_plan,
                uav_name,
            )
            if conflict is not None:
                conflicts.append(conflict)

        conflicts.extend(
            self._airspace_conflicts(proposed_flight_plan, airspace)
        )
        return conflicts

    def _traffic_conflict(
        self,
        proposed_plan: FlightPlan,
        approved_plan: FlightPlan,
        approved_uav: str,
    ) -> StrategicTrafficConflict | None:
        start = max(
            proposed_plan.waypoint_list[0].t,
            approved_plan.waypoint_list[0].t,
        )
        end = min(
            proposed_plan.waypoint_list[-1].t,
            approved_plan.waypoint_list[-1].t,
        )
        if end < start:
            return None

        for timestamp in sample_times(
            start,
            end,
            self.sampling_interval,
        ):
            proposed_position = position_at(proposed_plan, timestamp)
            approved_position = position_at(approved_plan, timestamp)
            separation = dist(proposed_position, approved_position)
            if separation < self.minimum_separation_distance:
                return StrategicTrafficConflict(
                    timestamp=timestamp,
                    conflicting_uav=approved_uav,
                    proposed_position=tuple(proposed_position),
                    conflicting_position=tuple(approved_position),
                    separation_distance=separation,
                )
        return None

    def _airspace_conflicts(
        self,
        flight_plan: FlightPlan,
        airspace: Airspace,
    ) -> list[StrategicAirspaceConflict]:
        conflicts: list[StrategicAirspaceConflict] = []
        detected_restrictions: set[tuple[str, str]] = set()
        start = flight_plan.waypoint_list[0].t
        end = flight_plan.waypoint_list[-1].t

        for timestamp in sample_times(start, end, self.sampling_interval):
            position = position_at(flight_plan, timestamp)
            for obstacle in airspace.obstacles.values():
                key = ("obstacle", obstacle.obstacle_id)
                if key not in detected_restrictions and obstacle.contains(
                    position
                ):
                    conflicts.append(StrategicAirspaceConflict(
                        timestamp=timestamp,
                        position=tuple(position),
                        restriction=obstacle,
                    ))
                    detected_restrictions.add(key)

            for geofence in airspace.geofences.values():
                key = ("geofence", geofence.geofence_id)
                if key not in detected_restrictions and geofence.prohibits(
                    position,
                    timestamp,
                ):
                    conflicts.append(StrategicAirspaceConflict(
                        timestamp=timestamp,
                        position=tuple(position),
                        restriction=geofence,
                    ))
                    detected_restrictions.add(key)
        return conflicts

    @staticmethod
    def _require_waypoints(flight_plan: FlightPlan) -> None:
        if not flight_plan.waypoint_list:
            raise ValueError("flight plan must contain at least one waypoint")


class MinimumSeparationTacticalConflictPrediction(
    TacticalConflictPredictionAlgorithm
):
    """Predict spherical separation and airspace conflicts from tracks."""

    def __init__(
        self,
        minimum_separation_distance: float = 30.0,
        maximum_time_difference: float = 0.5,
    ) -> None:
        if minimum_separation_distance <= 0:
            raise ValueError(
                "minimum_separation_distance must be greater than zero"
            )
        if maximum_time_difference < 0:
            raise ValueError("maximum_time_difference cannot be negative")
        self.minimum_separation_distance = float(
            minimum_separation_distance
        )
        self.maximum_time_difference = float(maximum_time_difference)

    def predict(
        self,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> list[Conflict]:
        conflicts: list[Conflict] = []
        track_states = {
            name: self._states(track)
            for name, track in tracks.items()
        }

        for uav_name in sorted(tracks):
            conflicts.extend(
                self._airspace_conflicts(
                    uav_name,
                    track_states[uav_name],
                    airspace,
                )
            )

        for uav_name, other_name in combinations(sorted(tracks), 2):
            conflict = self._traffic_conflict(
                uav_name,
                track_states[uav_name],
                other_name,
                track_states[other_name],
            )
            if conflict is not None:
                conflicts.append(conflict)
        return conflicts

    def _traffic_conflict(
        self,
        uav_name: str,
        states: tuple[PredictedState, ...],
        other_name: str,
        other_states: tuple[PredictedState, ...],
    ) -> TacticalTrafficConflict | None:
        if not other_states:
            return None
        for state in states:
            other = min(
                other_states,
                key=lambda candidate: abs(
                    candidate.timestamp - state.timestamp
                ),
            )
            if (
                abs(other.timestamp - state.timestamp)
                > self.maximum_time_difference
            ):
                continue
            separation = dist(state.position, other.position)
            if separation < self.minimum_separation_distance:
                return TacticalTrafficConflict(
                    timestamp=state.timestamp,
                    uav_name=uav_name,
                    other_uav=other_name,
                    position=tuple(state.position),
                    other_position=tuple(other.position),
                    separation_distance=separation,
                )
        return None

    @staticmethod
    def _airspace_conflicts(
        uav_name: str,
        states: tuple[PredictedState, ...],
        airspace: Airspace,
    ) -> list[TacticalAirspaceConflict]:
        conflicts: list[TacticalAirspaceConflict] = []
        detected_restrictions: set[tuple[str, str]] = set()
        for state in states:
            for obstacle in airspace.obstacles.values():
                key = ("obstacle", obstacle.obstacle_id)
                if key not in detected_restrictions and obstacle.contains(
                    state.position
                ):
                    conflicts.append(TacticalAirspaceConflict(
                        timestamp=state.timestamp,
                        uav_name=uav_name,
                        position=tuple(state.position),
                        restriction=obstacle,
                    ))
                    detected_restrictions.add(key)

            for geofence in airspace.geofences.values():
                key = ("geofence", geofence.geofence_id)
                if key not in detected_restrictions and geofence.prohibits(
                    state.position,
                    state.timestamp,
                ):
                    conflicts.append(TacticalAirspaceConflict(
                        timestamp=state.timestamp,
                        uav_name=uav_name,
                        position=tuple(state.position),
                        restriction=geofence,
                    ))
                    detected_restrictions.add(key)
        return conflicts

    @staticmethod
    def _states(track: TrackPrediction) -> tuple[PredictedState, ...]:
        return (
            track.estimated_state,
            *track.predicted_states,
        )
