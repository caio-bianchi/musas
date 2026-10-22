from __future__ import annotations

import math

from musas.domain.flight import FlightPlan, Waypoint


def position_at(flight_plan: FlightPlan, timestamp: float) -> list[float]:
    """Linearly interpolate a flight plan at ``timestamp``."""
    waypoints = flight_plan.waypoint_list
    if not waypoints:
        raise ValueError("flight plan must contain at least one waypoint")
    if timestamp <= waypoints[0].t:
        return waypoints[0].position()
    if timestamp >= waypoints[-1].t:
        return waypoints[-1].position()

    for start, end in zip(waypoints, waypoints[1:]):
        if start.t <= timestamp <= end.t:
            duration = end.t - start.t
            if duration <= 0:
                return end.position()
            fraction = (timestamp - start.t) / duration
            return [
                initial + fraction * (final - initial)
                for initial, final in zip(
                    start.position(),
                    end.position(),
                )
            ]

    return waypoints[-1].position()


def velocity_at(flight_plan: FlightPlan, timestamp: float) -> list[float]:
    """Return the nominal velocity of the segment containing ``timestamp``."""
    waypoints = flight_plan.waypoint_list
    if len(waypoints) < 2:
        return [0.0, 0.0, 0.0]
    if timestamp < waypoints[0].t or timestamp > waypoints[-1].t:
        return [0.0, 0.0, 0.0]

    for start, end in zip(waypoints, waypoints[1:]):
        if timestamp <= end.t:
            duration = end.t - start.t
            if duration <= 0:
                return [0.0, 0.0, 0.0]
            return [
                (final - initial) / duration
                for initial, final in zip(
                    start.position(),
                    end.position(),
                )
            ]

    start, end = waypoints[-2:]
    duration = end.t - start.t
    if duration <= 0:
        return [0.0, 0.0, 0.0]
    return [
        (final - initial) / duration
        for initial, final in zip(start.position(), end.position())
    ]


def sample_times(start: float, end: float, step: float):
    """Yield evenly spaced instants, always including both bounds."""
    if step <= 0:
        raise ValueError("sampling interval must be greater than zero")
    if end < start:
        return

    duration = end - start
    intervals = max(1, math.ceil(duration / step))
    for index in range(intervals + 1):
        yield start + duration * index / intervals


def shifted_flight_plan(
    flight_plan: FlightPlan,
    time_offset: float,
) -> FlightPlan:
    """Return a copy of a flight plan shifted in time."""
    return FlightPlan(
        waypoint_list=[
            Waypoint(
                x=waypoint.x,
                y=waypoint.y,
                z=waypoint.z,
                t=waypoint.t + time_offset,
            )
            for waypoint in flight_plan.waypoint_list
        ],
        priority=flight_plan.priority,
    )
