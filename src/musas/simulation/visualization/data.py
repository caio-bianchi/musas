from bisect import bisect_right
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from musas.domain.flight import FlightPlan
from musas.simulation.dataset import (
    AirspaceObjectRecord,
    AirspaceSnapshotRecord,
    SimulationDataset,
    StoredEvent,
    TrackingPredictionRecord,
    TrackingReportRecord,
    TrajectoryRecord,
)
from musas.simulation.visualization.config import VisualizationConfig


@dataclass
class VisualizationData:
    simulation_time: float
    uav_names: tuple[str, ...]
    trajectories: dict[str, list[TrajectoryRecord]]
    reports: dict[str, list[TrackingReportRecord]]
    predictions: dict[
        str,
        dict[float, list[TrackingPredictionRecord]],
    ]
    proposed_plans: dict[str, FlightPlan | None]
    authorized_plans: dict[str, dict[int, FlightPlan]]
    plan_events: dict[str, list[StoredEvent]]
    airspace_objects: list[AirspaceObjectRecord]
    airspace_snapshots: list[AirspaceSnapshotRecord]


def load_data(
    dataset_path: str | Path,
    config: VisualizationConfig,
) -> VisualizationData:
    with SimulationDataset(dataset_path) as dataset:
        available = dataset.uav_names
        selected = available if config.uav_names is None else config.uav_names
        unknown = sorted(set(selected) - set(available))
        if unknown:
            raise ValueError(f"unknown UAV names: {unknown}")
        if not selected:
            raise ValueError("select at least one UAV")

        trajectories = {
            name: dataset.get_trajectory(name)
            for name in selected
        }
        reports = {
            name: dataset.get_tracking_reports(name)
            if config.show_sensor_reports else []
            for name in selected
        }
        predictions = {
            name: _load_predictions(dataset, name)
            if config.show_tracks else {}
            for name in selected
        }
        proposed_plans = {
            name: dataset.get_flight_plan(name, "proposed", 0)
            for name in selected
        }
        authorized_plans = {
            name: {
                version: dataset.get_flight_plan(
                    name,
                    "authorized",
                    version,
                )
                for version in dataset.get_flight_plan_versions(name)
            }
            for name in selected
        }

        selected_set = set(selected)
        plan_events = defaultdict(list)
        for event in dataset.get_events("FLIGHT_PLAN_LOADED"):
            uav_name = event.details.get("uav")
            if uav_name in selected_set:
                plan_events[uav_name].append(event)

        return VisualizationData(
            simulation_time=dataset.simulation_time,
            uav_names=tuple(selected),
            trajectories=trajectories,
            reports=reports,
            predictions=predictions,
            proposed_plans=proposed_plans,
            authorized_plans=authorized_plans,
            plan_events=dict(plan_events),
            airspace_objects=dataset.get_airspace_objects(),
            airspace_snapshots=dataset.get_airspace_snapshots(),
        )


def active_plan(
    data: VisualizationData,
    uav_name: str,
    timestamp: float,
) -> FlightPlan | None:
    load_events = data.plan_events.get(uav_name, ())
    events = [event for event in load_events if event.timestamp <= timestamp]
    if events:
        version = int(events[-1].details["plan_version"])
        return data.authorized_plans[uav_name].get(version)
    if load_events:
        return None
    return data.proposed_plans[uav_name]


def latest_track(
    predictions: dict[float, list[TrackingPredictionRecord]],
    timestamp: float,
) -> list[TrackingPredictionRecord]:
    generations = sorted(predictions)
    index = bisect_right(generations, timestamp) - 1
    return [] if index < 0 else predictions[generations[index]]


def active_object_ids(
    data: VisualizationData,
    object_type: str,
    timestamp: float,
) -> set[str]:
    if data.airspace_snapshots:
        times = [snapshot.timestamp for snapshot in data.airspace_snapshots]
        index = bisect_right(times, timestamp) - 1
        if index < 0:
            return set()
        snapshot = data.airspace_snapshots[index]
        identifiers = (
            snapshot.geofence_ids
            if object_type == "geofence"
            else snapshot.wind_zone_ids
        )
        return set(identifiers)

    return {
        item.object_id
        for item in data.airspace_objects
        if item.object_type == object_type
        and object_is_active(item, timestamp)
    }


def object_is_active(
    item: AirspaceObjectRecord,
    timestamp: float,
) -> bool:
    return not (
        item.active_from is not None and timestamp < item.active_from
        or item.active_until is not None and timestamp > item.active_until
    )


def _load_predictions(
    dataset: SimulationDataset,
    uav_name: str,
) -> dict[float, list[TrackingPredictionRecord]]:
    grouped = defaultdict(list)
    for state in dataset.get_tracking_predictions(uav_name):
        grouped[state.generated_at].append(state)
    for states in grouped.values():
        states.sort(
            key=lambda state: (
                state.state_kind != "estimated",
                state.horizon_index,
            )
        )
    return dict(grouped)
