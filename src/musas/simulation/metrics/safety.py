from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable, Iterable, Sequence
from itertools import combinations
from math import dist, hypot, inf

from musas.simulation.collision_detection import (
    closest_approach,
    first_sphere_collision,
)
from musas.simulation.dataset import (
    AirspaceObjectRecord,
    AirspaceSnapshotRecord,
    SimulationDataset,
    TrajectoryRecord,
)


ACTIVE_STATUS = "ACTIVE"


def number_of_loss_of_separation_events(
    dataset: SimulationDataset,
    minimum_separation_distance: float,
) -> int:
    """Count entries into a sampled spherical separation volume."""
    _validate_nonnegative(
        minimum_separation_distance=minimum_separation_distance,
    )

    return _count_pairwise_continuous_episodes(
        dataset.get_trajectory(),
        minimum_separation_distance,
    )


def number_of_collision_events(
    dataset: SimulationDataset,
    collision_distance: float,
) -> int:
    """Count entries into a sampled 3-D collision-distance condition."""
    _validate_nonnegative(collision_distance=collision_distance)
    recorded_collisions = dataset.get_events("UAV_COLLISION")
    if recorded_collisions:
        return len(recorded_collisions)

    return _count_pairwise_continuous_episodes(
        dataset.get_trajectory(),
        collision_distance,
    )


def minimum_pairwise_separation(
    dataset: SimulationDataset,
) -> float | None:
    """Return the smallest recorded 3-D distance between UAVs.

    Active trajectory samples provide the regular observations. Collision
    event positions are also considered because the first trajectory sample
    after a collision already has status ``COLLIDED`` and would otherwise be
    excluded from the metric.
    """
    minimum = inf
    groups = list(_trajectory_by_timestamp(dataset.get_trajectory()).values())
    previous: dict[str, TrajectoryRecord] = {}
    for samples in groups:
        current = {
            sample.uav_name: sample
            for sample in samples
            if sample.status == ACTIVE_STATUS
        }
        for first, second in combinations(current.values(), 2):
            minimum = min(minimum, dist(first.position, second.position))

        common_names = sorted(set(previous) & set(current))
        for first_name, second_name in combinations(common_names, 2):
            approach = closest_approach(
                previous[first_name].position,
                current[first_name].position,
                previous[second_name].position,
                current[second_name].position,
            )
            minimum = min(minimum, approach.distance)
        previous = current

    for event in dataset.get_events("UAV_COLLISION"):
        position_a = event.details.get("position_a")
        position_b = event.details.get("position_b")
        if position_a is not None and position_b is not None:
            minimum = min(minimum, dist(position_a, position_b))

    return None if minimum == inf else minimum


def number_of_geofence_violations(dataset: SimulationDataset) -> int:
    """Count UAV entries into active KEEP_OUT geofences."""
    geofences = {
        item.object_id: item
        for item in dataset.get_airspace_objects("geofence")
    }
    snapshots = dataset.get_airspace_snapshots()

    def active_objects(timestamp: float) -> Iterable[AirspaceObjectRecord]:
        active_ids = _active_geofence_ids(timestamp, snapshots)
        return (
            geofences[object_id]
            for object_id in active_ids
            if object_id in geofences
        )

    return _count_region_episodes(
        dataset.get_trajectory(),
        active_objects,
    )


def number_of_obstacle_violations(dataset: SimulationDataset) -> int:
    """Count UAV entries into obstacle geometries."""
    recorded_collisions = dataset.get_events("OBSTACLE_COLLISION")
    if recorded_collisions:
        return len(recorded_collisions)
    obstacles = tuple(dataset.get_airspace_objects("obstacle"))
    return _count_region_episodes(
        dataset.get_trajectory(),
        lambda timestamp: obstacles,
    )


def _count_pairwise_continuous_episodes(
    trajectory: Sequence[TrajectoryRecord],
    separation_distance: float,
) -> int:
    """Count entries into a separation sphere between recorded instants."""
    groups = list(_trajectory_by_timestamp(trajectory).values())
    if not groups:
        return 0

    previous = {
        sample.uav_name: sample
        for sample in groups[0]
        if sample.status == ACTIVE_STATUS
    }
    previous_pairs = {
        tuple(sorted((first.uav_name, second.uav_name)))
        for first, second in combinations(previous.values(), 2)
        if dist(first.position, second.position) <= separation_distance
    }
    episode_count = len(previous_pairs)

    for samples in groups[1:]:
        current = {
            sample.uav_name: sample
            for sample in samples
            if sample.status == ACTIVE_STATUS
        }
        current_pairs = {
            tuple(sorted((first.uav_name, second.uav_name)))
            for first, second in combinations(current.values(), 2)
            if dist(first.position, second.position) <= separation_distance
        }
        interval_pairs: set[tuple[str, str]] = set()
        common_names = sorted(set(previous) & set(current))
        for first_name, second_name in combinations(common_names, 2):
            collision = first_sphere_collision(
                previous[first_name].position,
                current[first_name].position,
                previous[second_name].position,
                current[second_name].position,
                separation_distance,
            )
            if collision is not None:
                interval_pairs.add((first_name, second_name))

        episode_count += len(interval_pairs - previous_pairs)
        episode_count += len(current_pairs - interval_pairs - previous_pairs)
        previous = current
        previous_pairs = current_pairs

    return episode_count


def _count_region_episodes(
    trajectory: Sequence[TrajectoryRecord],
    objects_at: Callable[[float], Iterable[AirspaceObjectRecord]],
) -> int:
    previous_violations: set[tuple[str, str]] = set()
    episode_count = 0

    for timestamp, samples in _trajectory_by_timestamp(trajectory).items():
        objects = tuple(objects_at(timestamp))
        current_violations = {
            (sample.uav_name, item.object_id)
            for sample in samples
            if sample.status == ACTIVE_STATUS
            for item in objects
            if _geometry_contains(item, sample.position)
        }
        episode_count += len(current_violations - previous_violations)
        previous_violations = current_violations

    return episode_count


def _trajectory_by_timestamp(
    trajectory: Sequence[TrajectoryRecord],
) -> dict[float, list[TrajectoryRecord]]:
    grouped: dict[float, list[TrajectoryRecord]] = defaultdict(list)
    for sample in trajectory:
        grouped[sample.timestamp].append(sample)
    return dict(sorted(grouped.items()))


def _active_geofence_ids(
    timestamp: float,
    snapshots: Sequence[AirspaceSnapshotRecord],
) -> tuple[str, ...]:
    active_ids: tuple[str, ...] = ()
    for snapshot in snapshots:
        if snapshot.timestamp > timestamp:
            break
        active_ids = snapshot.geofence_ids
    return active_ids


def _geometry_contains(
    item: AirspaceObjectRecord,
    position: Sequence[float],
) -> bool:
    geometry_name = item.geometry_type.rsplit(".", 1)[-1]
    values = item.geometry
    x, y, z = position

    if geometry_name == "Box3D":
        return (
            values["minimum_x"] <= x <= values["maximum_x"]
            and values["minimum_y"] <= y <= values["maximum_y"]
            and values["minimum_z"] <= z <= values["maximum_z"]
        )
    if geometry_name == "Cylinder3D":
        return (
            hypot(x - values["center_x"], y - values["center_y"])
            <= values["radius"]
            and values["minimum_z"] <= z <= values["maximum_z"]
        )

    raise ValueError(
        "cannot calculate airspace violations for unsupported geometry "
        f"{item.geometry_type!r}"
    )


def _validate_nonnegative(**values: float) -> None:
    for name, value in values.items():
        if value < 0:
            raise ValueError(f"{name} cannot be negative")
