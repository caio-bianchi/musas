from __future__ import annotations

from musas.airspace.airspace import Airspace
from musas.algorithms.builtins.conflicts import (
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
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
from musas.domain.tracking import TrackPrediction


class VerticalManeuverDeconfliction(TacticalDeconflictionAlgorithm):
    """Resolve tactical conflicts with a temporary vertical maneuver.

    For traffic conflicts, the lexicographically last UAV is assigned the
    maneuver so both aircraft do not react simultaneously. For airspace
    conflicts, the affected UAV moves above or below the restriction. The
    resulting command is expressed only as a flight-plan update.
    """

    def __init__(
        self,
        altitude_offset: float = 30.0,
        airspace_clearance: float = 10.0,
        minimum_maneuver_duration: float = 4.0,
        cooldown: float = 10.0,
    ) -> None:
        if altitude_offset <= 0:
            raise ValueError("altitude_offset must be greater than zero")
        if airspace_clearance < 0:
            raise ValueError("airspace_clearance cannot be negative")
        if minimum_maneuver_duration <= 0:
            raise ValueError(
                "minimum_maneuver_duration must be greater than zero"
            )
        if cooldown < 0:
            raise ValueError("cooldown cannot be negative")
        self.altitude_offset = float(altitude_offset)
        self.airspace_clearance = float(airspace_clearance)
        self.minimum_maneuver_duration = float(
            minimum_maneuver_duration
        )
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

        for conflict in sorted(
            conflicts,
            key=lambda item: getattr(item, "timestamp", float("inf")),
        ):
            uav_name = self._maneuvering_uav(conflict)
            if (
                uav_name is None
                or uav_name in updated_uavs
                or uav_name not in tracks
                or uav_name not in flight_plans
            ):
                continue

            track = tracks[uav_name]
            now = float(track.generated_at)
            if now < self._cooldown_until.get(uav_name, float("-inf")):
                continue

            updated_plan = self._updated_plan(
                uav_name,
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

    @staticmethod
    def _maneuvering_uav(conflict: Conflict) -> str | None:
        if isinstance(conflict, TacticalAirspaceConflict):
            return conflict.uav_name
        if isinstance(conflict, TacticalTrafficConflict):
            return max(conflict.uav_name, conflict.other_uav)
        return None

    def _updated_plan(
        self,
        uav_name: str,
        conflict: Conflict,
        track: TrackPrediction,
        flight_plan: FlightPlan,
    ) -> FlightPlan | None:
        if not flight_plan.waypoint_list:
            return None
        current = track.estimated_state
        now = max(float(track.generated_at), float(current.timestamp))
        conflict_time = max(
            float(getattr(conflict, "timestamp", now)),
            now + self.minimum_maneuver_duration,
        )

        if isinstance(conflict, TacticalAirspaceConflict):
            maneuver_position = list(conflict.position)
            maneuver_position[2] = self._airspace_altitude(
                current.position[2],
                conflict,
            )
        elif isinstance(conflict, TacticalTrafficConflict):
            maneuver_position = list(
                conflict.position
                if uav_name == conflict.uav_name
                else conflict.other_position
            )
            maneuver_position[2] = (
                current.position[2] + self.altitude_offset
            )
        else:
            return None

        destination = flight_plan.waypoint_list[-1]
        destination_time = max(
            destination.t,
            conflict_time + self.minimum_maneuver_duration,
        )
        return FlightPlan([
            Waypoint(*current.position, now),
            Waypoint(*maneuver_position, conflict_time),
            Waypoint(
                destination.x,
                destination.y,
                destination.z,
                destination_time,
            ),
        ])

    def _airspace_altitude(
        self,
        current_altitude: float,
        conflict: TacticalAirspaceConflict,
    ) -> float:
        geometry = conflict.restriction.geometry
        minimum_z = getattr(geometry, "minimum_z", None)
        maximum_z = getattr(geometry, "maximum_z", None)
        if minimum_z is None or maximum_z is None:
            return current_altitude + self.altitude_offset

        above = float(maximum_z) + self.airspace_clearance
        below = float(minimum_z) - self.airspace_clearance
        if below >= 0 and abs(current_altitude - below) < abs(
            above - current_altitude
        ):
            return below
        return above
