from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Sequence

from musas.airspace.geometry import Box3D, Cylinder3D, Geometry


_EPSILON = 1e-12


@dataclass(frozen=True)
class ClosestApproach:
    """Closest simultaneous positions along two linear motion segments."""

    fraction: float
    distance: float
    first_position: tuple[float, ...]
    second_position: tuple[float, ...]


@dataclass(frozen=True)
class SegmentIntersection:
    """First intersection of a motion segment with a geometry."""

    fraction: float
    position: tuple[float, ...]


def closest_approach(
    first_start: Sequence[float],
    first_end: Sequence[float],
    second_start: Sequence[float],
    second_end: Sequence[float],
) -> ClosestApproach:
    """Return the minimum simultaneous separation over one time interval."""
    starts = _matching_vectors(
        first_start,
        first_end,
        second_start,
        second_end,
    )
    first_initial, first_final, second_initial, second_final = starts
    relative_start = _subtract(first_initial, second_initial)
    relative_change = _subtract(
        _subtract(first_final, first_initial),
        _subtract(second_final, second_initial),
    )
    squared_change = _dot(relative_change, relative_change)
    if squared_change <= _EPSILON:
        fraction = 0.0
    else:
        fraction = _clamp(
            -_dot(relative_start, relative_change) / squared_change
        )

    first_position = _interpolate(first_initial, first_final, fraction)
    second_position = _interpolate(second_initial, second_final, fraction)
    separation = sqrt(
        _dot(
            _subtract(first_position, second_position),
            _subtract(first_position, second_position),
        )
    )
    return ClosestApproach(
        fraction=fraction,
        distance=separation,
        first_position=first_position,
        second_position=second_position,
    )


def first_sphere_collision(
    first_start: Sequence[float],
    first_end: Sequence[float],
    second_start: Sequence[float],
    second_end: Sequence[float],
    collision_distance: float,
) -> ClosestApproach | None:
    """Return the first time two linearly moving points reach a separation."""
    if collision_distance < 0.0:
        raise ValueError("collision_distance cannot be negative")

    first_initial, first_final, second_initial, second_final = (
        _matching_vectors(
            first_start,
            first_end,
            second_start,
            second_end,
        )
    )
    relative_start = _subtract(first_initial, second_initial)
    relative_change = _subtract(
        _subtract(first_final, first_initial),
        _subtract(second_final, second_initial),
    )
    c = _dot(relative_start, relative_start) - collision_distance**2
    if c <= 0.0:
        fraction = 0.0
    else:
        a = _dot(relative_change, relative_change)
        if a <= _EPSILON:
            return None
        b = 2.0 * _dot(relative_start, relative_change)
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None
        fraction = (-b - sqrt(max(0.0, discriminant))) / (2.0 * a)
        if fraction < -_EPSILON or fraction > 1.0 + _EPSILON:
            return None
        fraction = _clamp(fraction)

    first_position = _interpolate(first_initial, first_final, fraction)
    second_position = _interpolate(second_initial, second_final, fraction)
    separation_vector = _subtract(first_position, second_position)
    return ClosestApproach(
        fraction=fraction,
        distance=sqrt(_dot(separation_vector, separation_vector)),
        first_position=first_position,
        second_position=second_position,
    )


def first_geometry_intersection(
    start: Sequence[float],
    end: Sequence[float],
    geometry: Geometry,
) -> SegmentIntersection | None:
    """Return the first segment intersection for built-in geometries.

    Custom geometries retain a conservative endpoint fallback. They can still
    participate in discrete collision detection without extending the public
    ``Geometry`` contract required from framework users.
    """
    initial, final = _matching_vectors(start, end)
    if geometry.contains(initial):
        return SegmentIntersection(0.0, initial)

    if isinstance(geometry, Box3D):
        fraction = _box_entry_fraction(initial, final, geometry)
    elif isinstance(geometry, Cylinder3D):
        fraction = _cylinder_entry_fraction(initial, final, geometry)
    else:
        fraction = 1.0 if geometry.contains(final) else None

    if fraction is None:
        return None
    return SegmentIntersection(
        fraction=fraction,
        position=_interpolate(initial, final, fraction),
    )


def _box_entry_fraction(
    start: tuple[float, ...],
    end: tuple[float, ...],
    box: Box3D,
) -> float | None:
    lower = (box.minimum_x, box.minimum_y, box.minimum_z)
    upper = (box.maximum_x, box.maximum_y, box.maximum_z)
    entry, exit_ = 0.0, 1.0
    for initial, final, minimum, maximum in zip(start, end, lower, upper):
        change = final - initial
        if abs(change) <= _EPSILON:
            if initial < minimum or initial > maximum:
                return None
            continue
        first = (minimum - initial) / change
        second = (maximum - initial) / change
        axis_entry, axis_exit = sorted((first, second))
        entry = max(entry, axis_entry)
        exit_ = min(exit_, axis_exit)
        if entry > exit_ + _EPSILON:
            return None
    return _clamp(entry) if entry <= 1.0 + _EPSILON else None


def _cylinder_entry_fraction(
    start: tuple[float, ...],
    end: tuple[float, ...],
    cylinder: Cylinder3D,
) -> float | None:
    x0 = start[0] - cylinder.center_x
    y0 = start[1] - cylinder.center_y
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    radial_a = dx * dx + dy * dy
    radial_b = 2.0 * (x0 * dx + y0 * dy)
    radial_c = x0 * x0 + y0 * y0 - cylinder.radius**2

    if radial_a <= _EPSILON:
        if radial_c > 0.0:
            return None
        radial_interval = (0.0, 1.0)
    else:
        discriminant = radial_b * radial_b - 4.0 * radial_a * radial_c
        if discriminant < 0.0:
            return None
        root = sqrt(max(0.0, discriminant))
        radial_interval = tuple(sorted((
            (-radial_b - root) / (2.0 * radial_a),
            (-radial_b + root) / (2.0 * radial_a),
        )))

    z_interval = _axis_interval(
        start[2],
        end[2],
        cylinder.minimum_z,
        cylinder.maximum_z,
    )
    if z_interval is None:
        return None

    entry = max(0.0, radial_interval[0], z_interval[0])
    exit_ = min(1.0, radial_interval[1], z_interval[1])
    if entry > exit_ + _EPSILON:
        return None
    return _clamp(entry)


def _axis_interval(
    start: float,
    end: float,
    minimum: float,
    maximum: float,
) -> tuple[float, float] | None:
    change = end - start
    if abs(change) <= _EPSILON:
        return (0.0, 1.0) if minimum <= start <= maximum else None
    return tuple(sorted(((minimum - start) / change, (maximum - start) / change)))


def _matching_vectors(
    *vectors: Sequence[float],
) -> tuple[tuple[float, ...], ...]:
    converted = tuple(tuple(float(value) for value in vector) for vector in vectors)
    dimensions = {len(vector) for vector in converted}
    if len(dimensions) != 1 or not converted or not converted[0]:
        raise ValueError("vectors must be non-empty and share a dimension")
    return converted


def _subtract(
    first: Sequence[float],
    second: Sequence[float],
) -> tuple[float, ...]:
    return tuple(a - b for a, b in zip(first, second))


def _dot(first: Sequence[float], second: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(first, second))


def _interpolate(
    start: Sequence[float],
    end: Sequence[float],
    fraction: float,
) -> tuple[float, ...]:
    return tuple(
        initial + fraction * (final - initial)
        for initial, final in zip(start, end)
    )


def _clamp(value: float) -> float:
    return min(1.0, max(0.0, value))
