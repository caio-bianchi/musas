from __future__ import annotations

from itertools import combinations
from math import dist

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins._track_comparison import (
    IndexedTrack,
    bounding_box_gap,
)
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

    Plans are compared only over their common time window, so a plan that
    starts right after another one ends is never compared with it. A terminal
    rule covers this gap: a conflict is also reported when one UAV departs
    less than ``terminal_radius`` from the destination of another, at most
    ``terminal_buffer`` seconds after that arrival. The arriving UAV may still
    be there, because it completes its flight slightly after the planned
    arrival time. Setting either parameter to zero disables the rule.
    """

    def __init__(
        self,
        minimum_separation_distance: float = 30.0,
        sampling_interval: float = 0.5,
        terminal_buffer: float = 1.0,
        terminal_radius: float = 5.0,
    ) -> None:
        if minimum_separation_distance <= 0:
            raise ValueError(
                "minimum_separation_distance must be greater than zero"
            )
        if sampling_interval <= 0:
            raise ValueError("sampling_interval must be greater than zero")
        if terminal_buffer < 0:
            raise ValueError("terminal_buffer cannot be negative")
        if terminal_radius < 0:
            raise ValueError("terminal_radius cannot be negative")
        self.minimum_separation_distance = float(
            minimum_separation_distance
        )
        self.sampling_interval = float(sampling_interval)
        self.terminal_buffer = float(terminal_buffer)
        self.terminal_radius = float(terminal_radius)

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
            # The plans do not overlap in time; only the terminal rule applies.
            return self._terminal_conflict(
                proposed_plan,
                approved_plan,
                approved_uav,
            )

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
        return self._terminal_conflict(
            proposed_plan,
            approved_plan,
            approved_uav,
        )

    def _terminal_conflict(
        self,
        proposed_plan: FlightPlan,
        approved_plan: FlightPlan,
        approved_uav: str,
    ) -> StrategicTrafficConflict | None:
        """Report a departure from the point where another UAV just arrived."""
        for departing, arriving in (
            (proposed_plan, approved_plan),
            (approved_plan, proposed_plan),
        ):
            origin = departing.waypoint_list[0]
            destination = arriving.waypoint_list[-1]
            time_after_arrival = origin.t - destination.t
            separation = dist(origin.position(), destination.position())
            if (
                0.0 <= time_after_arrival <= self.terminal_buffer
                and separation < self.terminal_radius
            ):
                proposed_waypoint, approved_waypoint = (
                    (origin, destination)
                    if departing is proposed_plan
                    else (destination, origin)
                )
                return StrategicTrafficConflict(
                    timestamp=origin.t,
                    conflicting_uav=approved_uav,
                    proposed_position=tuple(proposed_waypoint.position()),
                    conflicting_position=tuple(approved_waypoint.position()),
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
    """Predict spherical separation and airspace conflicts from tracks.

    For each pair of tracks, every state of the first track is compared with
    the closest-in-time state of the second one. Pairs whose bounding boxes
    are at least ``minimum_separation_distance`` apart cannot conflict and are
    skipped before this state-by-state comparison.
    """

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
        indexed_tracks = {
            name: IndexedTrack.from_track(track)
            for name, track in tracks.items()
        }

        for uav_name in sorted(tracks):
            conflicts.extend(
                self._airspace_conflicts(
                    uav_name,
                    indexed_tracks[uav_name].states,
                    airspace,
                )
            )

        for uav_name, other_name in combinations(sorted(tracks), 2):
            conflict = self._traffic_conflict(
                uav_name,
                indexed_tracks[uav_name],
                other_name,
                indexed_tracks[other_name],
            )
            if conflict is not None:
                conflicts.append(conflict)
        return conflicts

    def _traffic_conflict(
        self,
        uav_name: str,
        track: IndexedTrack,
        other_name: str,
        other_track: IndexedTrack,
    ) -> TacticalTrafficConflict | None:
        # No separation can be smaller than the gap between the bounding
        # boxes, so distant pairs are discarded without further work.
        if (
            bounding_box_gap(track, other_track)
            >= self.minimum_separation_distance
        ):
            return None
        for state in track.states:
            other = other_track.nearest_state(state.timestamp)
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
