from __future__ import annotations

from collections.abc import Sequence
from math import dist

from musas.simulation.dataset import TrajectoryRecord


def path_length(positions: Sequence[Sequence[float]]) -> float:
    """Return the length of the polyline formed by the supplied positions."""
    return sum(
        dist(first, second)
        for first, second in zip(positions, positions[1:])
    )


def interpolated_position(
    trajectory: Sequence[TrajectoryRecord],
    timestamp: float,
) -> tuple[float, float, float] | None:
    """Linearly interpolate a UAV position at a recorded time."""
    if not trajectory:
        return None
    if timestamp < trajectory[0].timestamp or timestamp > trajectory[-1].timestamp:
        return None

    for previous, following in zip(trajectory, trajectory[1:]):
        if timestamp == previous.timestamp:
            return previous.position
        if previous.timestamp <= timestamp <= following.timestamp:
            interval = following.timestamp - previous.timestamp
            if interval == 0:
                return following.position
            fraction = (timestamp - previous.timestamp) / interval
            return tuple(
                start + fraction * (end - start)
                for start, end in zip(previous.position, following.position)
            )

    return trajectory[-1].position
