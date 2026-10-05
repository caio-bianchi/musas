"""Iterative geometric conflict resolution from Acevedo et al. (2020)."""

from __future__ import annotations

from collections import deque
import math

from src.airspace.airspace import Airspace
from src.algorithms.defaults.grid_4d import (
    Grid4DConflict,
    GridWaypoint,
    Sparse4DGrid,
    conflict_signature,
    track_waypoints,
)
from src.domain.flight import Conflict, FlightPlan, Waypoint
from src.domain.tracking import TrackPrediction


class IterativeGeometricDeconfliction:
    """Separate conflicting future waypoints iteratively in 3-D space.

    This implements Equations 10 and 11 of the paper. Times are unchanged.
    As discussed by the authors, the method itself does not guarantee dynamic
    feasibility or compliance with a UAV's maximum speed.
    """

    def __init__(
        self,
        minimum_separation: float,
        time_step: float,
        maximum_iterations: int = 10_000,
        clearance_epsilon: float = 1e-6,
        fallback_speed: float = 15.0,
    ) -> None:
        self.minimum_separation = float(minimum_separation)
        self.time_step = float(time_step)
        self.maximum_iterations = int(maximum_iterations)
        self.clearance_epsilon = float(clearance_epsilon)
        self.fallback_speed = float(fallback_speed)
        if self.minimum_separation <= 0:
            raise ValueError("minimum_separation must be greater than zero")
        if self.time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if self.maximum_iterations <= 0:
            raise ValueError("maximum_iterations must be greater than zero")
        if self.clearance_epsilon < 0:
            raise ValueError("clearance_epsilon cannot be negative")
        if self.fallback_speed <= 0:
            raise ValueError("fallback_speed must be greater than zero")

    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> dict[str, FlightPlan]:
        waypoints = track_waypoints(tracks)
        grid = Sparse4DGrid(
            separation=self.minimum_separation,
            time_step=self.time_step,
        )
        for waypoint in waypoints.values():
            grid.insert(waypoint)

        queue = deque(
            conflict
            for conflict in conflicts
            if isinstance(conflict, Grid4DConflict)
        )
        queued = {conflict_signature(conflict) for conflict in queue}
        changed_uavs: set[str] = set()
        iterations = 0

        while queue:
            iterations += 1
            if iterations > self.maximum_iterations:
                raise RuntimeError(
                    "iterative geometric deconfliction did not converge"
                )

            stored_conflict = queue.popleft()
            signature = conflict_signature(stored_conflict)
            queued.discard(signature)
            first = waypoints.get(signature[0])
            second = waypoints.get(signature[1])
            if first is None or second is None:
                continue

            current_conflict = grid.make_conflict(first, second)
            if current_conflict is None:
                continue

            grid.remove(first)
            grid.remove(second)
            self._separate(first, second)
            grid.insert(first)
            grid.insert(second)
            changed_uavs.update((first.uav_name, second.uav_name))

            new_conflicts: dict[tuple, Grid4DConflict] = {}
            for modified in (first, second):
                for conflict in grid.conflicts_for(modified):
                    new_conflicts[conflict_signature(conflict)] = conflict
            for new_signature, conflict in new_conflicts.items():
                if new_signature not in queued:
                    queue.append(conflict)
                    queued.add(new_signature)

        return {
            uav_name: self._flight_plan_from_solution(
                uav_name,
                tracks[uav_name],
                flight_plans[uav_name],
                waypoints,
            )
            for uav_name in changed_uavs
            if uav_name in tracks and uav_name in flight_plans
        }

    def _separate(
        self,
        first: GridWaypoint,
        second: GridWaypoint,
    ) -> None:
        difference = [
            first_value - second_value
            for first_value, second_value in zip(
                first.position,
                second.position,
            )
        ]
        distance = math.sqrt(sum(value * value for value in difference))
        if distance <= 1e-12:
            direction = self._coincident_direction(first, second)
        else:
            direction = [value / distance for value in difference]

        displacement = max(
            0.0,
            (
                self.minimum_separation
                + self.clearance_epsilon
                - distance
            )
            / 2.0,
        )
        first.position = [
            value + displacement * component
            for value, component in zip(first.position, direction)
        ]
        second.position = [
            value - displacement * component
            for value, component in zip(second.position, direction)
        ]

    @staticmethod
    def _coincident_direction(
        first: GridWaypoint,
        second: GridWaypoint,
    ) -> list[float]:
        # Deterministic fallback for the undefined unit vector in Equations
        # 10/11 when both waypoints occupy exactly the same position.
        combined = (
            f"{first.uav_name}:{first.waypoint_index}:"
            f"{second.uav_name}:{second.waypoint_index}"
        )
        axis = sum(combined.encode("utf-8")) % 3
        direction = [0.0, 0.0, 0.0]
        direction[axis] = 1.0
        return direction

    def _flight_plan_from_solution(
        self,
        uav_name: str,
        track: TrackPrediction,
        original_plan: FlightPlan,
        waypoints: dict[tuple[str, int], GridWaypoint],
    ) -> FlightPlan:
        current = track.estimated_state
        result = [
            Waypoint(
                *current.position,
                max(float(track.generated_at), float(current.timestamp)),
            )
        ]
        for index, state in enumerate(track.predicted_states):
            resolved = waypoints[(uav_name, index)]
            result.append(Waypoint(*resolved.position, state.timestamp))

        goal = original_plan.waypoint_list[-1]
        if math.dist(result[-1].position(), goal.position()) > 1e-6:
            last_time = result[-1].t
            goal_time = max(
                goal.t,
                last_time
                + math.dist(result[-1].position(), goal.position())
                / self._continuation_speed(result),
            )
            result.append(Waypoint(*goal.position(), goal_time))
        return FlightPlan(result)

    def _continuation_speed(self, waypoints: list[Waypoint]) -> float:
        if len(waypoints) < 2:
            return self.fallback_speed
        duration = waypoints[-1].t - waypoints[-2].t
        if duration <= 0:
            return self.fallback_speed
        speed = math.dist(
            waypoints[-2].position(),
            waypoints[-1].position(),
        ) / duration
        return speed if speed > 1e-9 else self.fallback_speed
