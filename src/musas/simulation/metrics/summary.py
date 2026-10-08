from __future__ import annotations

from typing import Any

from musas.simulation.dataset import SimulationDataset
from musas.simulation.metrics.efficiency import (
    actual_distance,
    actual_flight_time,
    average_actual_distance,
    average_actual_flight_time,
    average_planned_distance,
    average_planned_flight_time,
    number_of_modified_flights,
    number_of_plan_updates,
    planned_distance,
    planned_flight_time,
)
from musas.simulation.metrics.operations import (
    number_of_detected_strategic_conflicts,
    number_of_detected_tactical_conflicts,
    number_of_flights,
    number_of_rejected_flights,
    number_of_tactical_interventions,
)
from musas.simulation.metrics.safety import (
    minimum_pairwise_separation,
    number_of_collision_events,
    number_of_geofence_violations,
    number_of_loss_of_separation_events,
    number_of_obstacle_violations,
)


def calculate_metrics(
    dataset: SimulationDataset,
    minimum_separation_distance: float,
    collision_distance: float,
) -> dict[str, Any]:
    """Calculate the standard safety and operational metrics."""
    per_uav = {
        name: {
            "planned_distance": planned_distance(dataset, name),
            "actual_distance": actual_distance(dataset, name),
            "planned_time": planned_flight_time(dataset, name),
            "actual_time": actual_flight_time(dataset, name),
            "updates": number_of_plan_updates(dataset, name),
        }
        for name in dataset.uav_names
    }

    return {
        "detected_strategic_conflicts": (
            number_of_detected_strategic_conflicts(dataset)
        ),
        "detected_tactical_conflicts": (
            number_of_detected_tactical_conflicts(dataset)
        ),
        "flights": number_of_flights(dataset),
        "modified_flights": number_of_modified_flights(dataset),
        "rejected_flights": number_of_rejected_flights(dataset),
        "tactical_interventions": number_of_tactical_interventions(dataset),
        "collision_events": number_of_collision_events(
            dataset,
            collision_distance,
        ),
        "geofence_violations": number_of_geofence_violations(dataset),
        "loss_of_separation_events": number_of_loss_of_separation_events(
            dataset,
            minimum_separation_distance,
        ),
        "minimum_pairwise_separation": minimum_pairwise_separation(dataset),
        "obstacle_violations": number_of_obstacle_violations(dataset),
        "average_planned_distance": average_planned_distance(dataset),
        "average_actual_distance": average_actual_distance(dataset),
        "average_planned_time": average_planned_flight_time(dataset),
        "average_actual_time": average_actual_flight_time(dataset),
        "per_uav": per_uav,
    }
