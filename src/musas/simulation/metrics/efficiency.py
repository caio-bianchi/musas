from __future__ import annotations

from collections.abc import Callable

from musas.domain.flight import FlightPlan
from musas.simulation.dataset import SimulationDataset
from musas.simulation.metrics._common import path_length


def planned_distance(
    dataset: SimulationDataset,
    uav_name: str,
    plan_kind: str = "proposed",
    plan_version: int = 0,
) -> float | None:
    """Return the 3-D polyline length of one stored flight plan."""
    plan = dataset.get_flight_plan(uav_name, plan_kind, plan_version)
    if plan is None:
        return None
    return _flight_plan_distance(plan)


def actual_distance(
    dataset: SimulationDataset,
    uav_name: str,
) -> float | None:
    """Return the traveled distance, or ``None`` if the flight never started."""
    samples = [
        sample
        for sample in dataset.get_trajectory(uav_name)
        if sample.status in {"ACTIVE", "COMPLETED"}
    ]
    if not samples:
        return None
    return path_length([sample.position for sample in samples])


def planned_flight_time(
    dataset: SimulationDataset,
    uav_name: str,
) -> float | None:
    """Return the time between the first and last proposed waypoints."""
    plan = dataset.get_flight_plan(uav_name, "proposed", 0)
    if plan is None or not plan.waypoint_list:
        return None
    return plan.waypoint_list[-1].t - plan.waypoint_list[0].t


def actual_flight_time(
    dataset: SimulationDataset,
    uav_name: str,
) -> float | None:
    """Return elapsed time from FLIGHT_STARTED to FLIGHT_COMPLETED."""
    starts = dataset.get_events("FLIGHT_STARTED", component=uav_name)
    completions = dataset.get_events("FLIGHT_COMPLETED", component=uav_name)
    if not starts or not completions:
        return None
    return completions[-1].timestamp - starts[0].timestamp


def number_of_plan_updates(dataset: SimulationDataset, uav_name: str) -> int:
    """Count changes from the proposal through all authorized plan versions."""
    plans: list[FlightPlan] = []
    proposed = dataset.get_flight_plan(uav_name, "proposed", 0)
    if proposed is not None:
        plans.append(proposed)
    plans.extend(
        plan
        for version in dataset.get_flight_plan_versions(uav_name)
        if (
            plan := dataset.get_flight_plan(
                uav_name,
                "authorized",
                version,
            )
        ) is not None
    )
    return sum(
        _plan_signature(previous) != _plan_signature(current)
        for previous, current in zip(plans, plans[1:])
    )


def number_of_modified_flights(dataset: SimulationDataset) -> int:
    """Count UAVs whose flight plan was modified at least once."""
    return sum(
        number_of_plan_updates(dataset, uav_name) > 0
        for uav_name in dataset.uav_names
    )


def average_planned_distance(dataset: SimulationDataset) -> float | None:
    """Return the average proposed-plan distance across UAVs."""
    return _average_per_uav(dataset, planned_distance)


def average_actual_distance(dataset: SimulationDataset) -> float | None:
    """Return the average executed distance across flights that started."""
    return _average_per_uav(dataset, actual_distance)


def average_planned_flight_time(dataset: SimulationDataset) -> float | None:
    """Return the average proposed duration across UAVs."""
    return _average_per_uav(dataset, planned_flight_time)


def average_actual_flight_time(dataset: SimulationDataset) -> float | None:
    """Return the average duration across completed UAV flights."""
    return _average_per_uav(dataset, actual_flight_time)


def _flight_plan_distance(plan: FlightPlan) -> float:
    return path_length(
        [waypoint.position() for waypoint in plan.waypoint_list]
    )


def _plan_signature(plan: FlightPlan) -> tuple[tuple[float, ...], ...]:
    return tuple(
        (waypoint.x, waypoint.y, waypoint.z, waypoint.t)
        for waypoint in plan.waypoint_list
    )


def _average_per_uav(
    dataset: SimulationDataset,
    metric: Callable[[SimulationDataset, str], float | None],
) -> float | None:
    values = [
        value
        for uav_name in dataset.uav_names
        if (value := metric(dataset, uav_name)) is not None
    ]
    if not values:
        return None
    return sum(values) / len(values)
