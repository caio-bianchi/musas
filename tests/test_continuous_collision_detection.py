from math import isclose

from musas.airspace import Box3D, Cylinder3D
from musas.simulation.collision_detection import (
    closest_approach,
    first_geometry_intersection,
    first_sphere_collision,
)
from musas.simulation.dataset.records import TrajectoryRecord
from musas.simulation.metrics import (
    minimum_pairwise_separation,
    number_of_collision_events,
    number_of_loss_of_separation_events,
)


class _TrajectoryDataset:
    def __init__(self, trajectory: list[TrajectoryRecord]) -> None:
        self.trajectory = trajectory

    def get_trajectory(self) -> list[TrajectoryRecord]:
        return self.trajectory

    def get_events(self, event_type: str):
        del event_type
        return []


def test_crossing_uavs_are_detected_between_samples() -> None:
    approach = closest_approach(
        (-5.0, 0.0, 100.0),
        (5.0, 0.0, 100.0),
        (0.0, -5.0, 100.0),
        (0.0, 5.0, 100.0),
    )
    collision = first_sphere_collision(
        (-5.0, 0.0, 100.0),
        (5.0, 0.0, 100.0),
        (0.0, -5.0, 100.0),
        (0.0, 5.0, 100.0),
        collision_distance=1.0,
    )

    assert isclose(approach.fraction, 0.5)
    assert isclose(approach.distance, 0.0, abs_tol=1e-12)
    assert collision is not None
    assert 0.0 < collision.fraction < 0.5
    assert isclose(collision.distance, 1.0, abs_tol=1e-9)


def test_segment_crosses_box_without_endpoint_inside() -> None:
    box = Box3D(-1.0, 1.0, -1.0, 1.0, 90.0, 110.0)

    intersection = first_geometry_intersection(
        (-2.0, 0.0, 100.0),
        (2.0, 0.0, 100.0),
        box,
    )

    assert intersection is not None
    assert isclose(intersection.fraction, 0.25)
    assert intersection.position == (-1.0, 0.0, 100.0)


def test_segment_crosses_cylinder_without_endpoint_inside() -> None:
    cylinder = Cylinder3D(0.0, 0.0, 1.0, 90.0, 110.0)

    intersection = first_geometry_intersection(
        (-2.0, 0.0, 100.0),
        (2.0, 0.0, 100.0),
        cylinder,
    )

    assert intersection is not None
    assert isclose(intersection.fraction, 0.25)
    assert intersection.position == (-1.0, 0.0, 100.0)


def test_safety_metrics_inspect_motion_between_samples() -> None:
    dataset = _TrajectoryDataset([
        TrajectoryRecord(0.0, "A", (-5.0, 0.0, 100.0), (10.0, 0.0, 0.0), "ACTIVE"),
        TrajectoryRecord(0.0, "B", (0.0, -5.0, 100.0), (0.0, 10.0, 0.0), "ACTIVE"),
        TrajectoryRecord(1.0, "A", (5.0, 0.0, 100.0), (10.0, 0.0, 0.0), "ACTIVE"),
        TrajectoryRecord(1.0, "B", (0.0, 5.0, 100.0), (0.0, 10.0, 0.0), "ACTIVE"),
    ])

    assert number_of_loss_of_separation_events(dataset, 2.0) == 1
    assert number_of_collision_events(dataset, 1.0) == 1
    assert isclose(minimum_pairwise_separation(dataset), 0.0, abs_tol=1e-12)
