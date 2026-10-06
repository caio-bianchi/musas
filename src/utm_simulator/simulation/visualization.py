"""Configurable plots and animations generated from simulation datasets."""

from __future__ import annotations

from bisect import bisect_right
from collections import defaultdict
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Patch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from utm_simulator.domain.flight import FlightPlan
from utm_simulator.simulation.dataset import (
    AirspaceObjectRecord,
    AirspaceSnapshotRecord,
    SimulationDataset,
    StoredEvent,
    TrackingPredictionRecord,
    TrackingReportRecord,
    TrajectoryRecord,
)


View = Literal["3d", "xy", "xz", "yz"]


@dataclass(frozen=True)
class VisualizationConfig:
    """Select the entities, projection and layers to be rendered."""

    uav_names: tuple[str, ...] | None = None
    view: View = "3d"
    show_real_trajectories: bool = True
    show_current_positions: bool = True
    show_tracks: bool = False
    show_flight_plans: bool = False
    show_sensor_reports: bool = False
    show_geofences: bool = True
    show_wind_zones: bool = False
    show_obstacles: bool = True
    show_labels: bool = True
    show_legend: bool = True
    show_grid: bool = True
    start_time: float | None = None
    end_time: float | None = None
    frame_step: float = 1.0
    figure_size: tuple[float, float] = (10.0, 7.0)
    title: str | None = None

    def __post_init__(self) -> None:
        if self.view not in ("3d", "xy", "xz", "yz"):
            raise ValueError("view must be '3d', 'xy', 'xz' or 'yz'")
        if self.frame_step <= 0:
            raise ValueError("frame_step must be greater than zero")
        if (
            self.start_time is not None
            and self.end_time is not None
            and self.end_time < self.start_time
        ):
            raise ValueError("end_time cannot be earlier than start_time")


@dataclass
class _VisualizationData:
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


def plot_dataset(
    dataset_path: str | Path,
    config: VisualizationConfig | None = None,
    timestamp: float | None = None,
    output_path: str | Path | None = None,
    dpi: int = 150,
) -> Figure:
    """Create a static dataset view and optionally save it as an image."""
    config = config or VisualizationConfig()
    data = _load_data(dataset_path, config)
    current_time = _bounded_time(
        data,
        config,
        data.simulation_time if timestamp is None else float(timestamp),
    )
    figure, axis = _new_figure(config)
    _draw_frame(axis, data, config, current_time)
    figure.tight_layout()
    if output_path is not None:
        destination = Path(output_path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(destination, dpi=dpi, bbox_inches="tight")
    return figure


def animate_dataset(
    dataset_path: str | Path,
    config: VisualizationConfig | None = None,
    output_path: str | Path | None = None,
    fps: float = 10.0,
    dpi: int = 100,
    repeat: bool = True,
) -> FuncAnimation:
    """Create an animation and optionally save it as GIF or MP4."""
    if fps <= 0:
        raise ValueError("fps must be greater than zero")
    config = config or VisualizationConfig()
    data = _load_data(dataset_path, config)
    frame_times = _frame_times(data, config)
    figure, axis = _new_figure(config)

    def update(timestamp: float):
        axis.clear()
        _draw_frame(axis, data, config, timestamp)
        return ()

    animation = FuncAnimation(
        figure,
        update,
        frames=frame_times,
        interval=1000.0 / fps,
        repeat=repeat,
        blit=False,
    )
    if output_path is not None:
        destination = Path(output_path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        suffix = destination.suffix.lower()
        if suffix == ".gif":
            writer = PillowWriter(fps=fps)
        elif suffix == ".mp4":
            writer = FFMpegWriter(fps=fps)
        else:
            raise ValueError("animation output must end in .gif or .mp4")
        animation.save(destination, writer=writer, dpi=dpi)
    return animation


def _load_data(
    dataset_path: str | Path,
    config: VisualizationConfig,
) -> _VisualizationData:
    with SimulationDataset.open(dataset_path) as dataset:
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
        predictions = {}
        for name in selected:
            grouped = defaultdict(list)
            if config.show_tracks:
                for state in dataset.get_tracking_predictions(name):
                    grouped[state.generated_at].append(state)
                for states in grouped.values():
                    states.sort(
                        key=lambda state: (
                            state.state_kind != "estimated",
                            state.horizon_index,
                        )
                    )
            predictions[name] = dict(grouped)

        proposed_plans = {
            name: dataset.get_flight_plan(name, "proposed", 0)
            for name in selected
        }
        authorized_plans = {}
        for name in selected:
            authorized_plans[name] = {
                version: dataset.get_flight_plan(
                    name,
                    "authorized",
                    version,
                )
                for version in dataset.get_flight_plan_versions(name)
            }

        selected_set = set(selected)
        plan_events = defaultdict(list)
        for event in dataset.get_events("FLIGHT_PLAN_LOADED"):
            uav_name = event.details.get("uav")
            if uav_name in selected_set:
                plan_events[uav_name].append(event)

        return _VisualizationData(
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


def _new_figure(config: VisualizationConfig):
    figure = plt.figure(figsize=config.figure_size)
    axis = (
        figure.add_subplot(111, projection="3d")
        if config.view == "3d"
        else figure.add_subplot(111)
    )
    return figure, axis


def _draw_frame(
    axis,
    data: _VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    color_map = plt.get_cmap(
        "tab10" if len(data.uav_names) <= 10 else "turbo"
    )
    colors = {
        name: color_map(
            index % 10
            if len(data.uav_names) <= 10
            else index / max(len(data.uav_names) - 1, 1)
        )
        for index, name in enumerate(data.uav_names)
    }
    _draw_airspace(axis, data, config, timestamp)

    for name in data.uav_names:
        color = colors[name]
        if config.show_flight_plans:
            plan = _active_plan(data, name, timestamp)
            if plan is not None:
                _plot_positions(
                    axis,
                    config.view,
                    [waypoint.position() for waypoint in plan.waypoint_list],
                    color=color,
                    linestyle="--",
                    linewidth=1.3,
                    alpha=0.65,
                    label=f"{name} — plano",
                )

        trajectory = [
            state
            for state in data.trajectories[name]
            if _start_time(config) <= state.timestamp <= timestamp
        ]
        if trajectory and config.show_real_trajectories:
            _plot_positions(
                axis,
                config.view,
                [state.position for state in trajectory],
                color=color,
                linewidth=2.0,
                label=f"{name} — real",
            )
        if trajectory and config.show_current_positions:
            position = trajectory[-1].position
            _scatter_positions(
                axis,
                config.view,
                [position],
                color=[color],
                edgecolor="black",
                s=48,
                zorder=6,
            )
            if config.show_labels:
                _label_position(axis, config.view, position, name)

        if config.show_sensor_reports:
            reports = [
                report
                for report in data.reports[name]
                if _start_time(config) <= report.received_at <= timestamp
            ]
            if reports:
                _scatter_positions(
                    axis,
                    config.view,
                    [report.position for report in reports],
                    marker="x",
                    color=[color],
                    alpha=0.45,
                    s=22,
                    label=f"{name} — sensor",
                )

        if config.show_tracks:
            track = _latest_track(data.predictions[name], timestamp)
            if track:
                future = [
                    state.position
                    for state in track
                    if state.state_kind == "estimated"
                    or state.predicted_timestamp >= timestamp
                ]
                if future:
                    _plot_positions(
                        axis,
                        config.view,
                        future,
                        color=color,
                        linestyle=":",
                        linewidth=2.0,
                        marker=".",
                        label=f"{name} — track",
                    )

    _configure_axis(axis, data, config, timestamp)


def _active_plan(
    data: _VisualizationData,
    uav_name: str,
    timestamp: float,
) -> FlightPlan | None:
    load_events = data.plan_events.get(uav_name, ())
    events = [
        event
        for event in load_events
        if event.timestamp <= timestamp
    ]
    if events:
        version = int(events[-1].details["plan_version"])
        return data.authorized_plans[uav_name].get(version)
    if load_events:
        return None
    return data.proposed_plans[uav_name]


def _latest_track(
    predictions: dict[float, list[TrackingPredictionRecord]],
    timestamp: float,
) -> list[TrackingPredictionRecord]:
    generations = sorted(predictions)
    index = bisect_right(generations, timestamp) - 1
    return [] if index < 0 else predictions[generations[index]]


def _draw_airspace(axis, data, config, timestamp: float) -> None:
    active_geofences = _active_geofence_ids(
        data.airspace_snapshots,
        timestamp,
    )
    if not data.airspace_snapshots:
        active_geofences = {
            item.object_id
            for item in data.airspace_objects
            if item.object_type == "geofence"
            and _object_is_active(item, timestamp)
        }
    for item in data.airspace_objects:
        if item.object_type == "geofence":
            if not config.show_geofences or item.object_id not in active_geofences:
                continue
            _draw_region(axis, config.view, item, "crimson", 0.20)
        elif item.object_type == "obstacle":
            if config.show_obstacles:
                _draw_region(axis, config.view, item, "dimgray", 0.28)
        elif item.object_type == "wind_zone":
            if config.show_wind_zones and _object_is_active(item, timestamp):
                _draw_region(axis, config.view, item, "deepskyblue", 0.16)
                _draw_wind_vector(axis, config.view, item)


def _active_geofence_ids(
    snapshots: list[AirspaceSnapshotRecord],
    timestamp: float,
) -> set[str]:
    if not snapshots:
        return set()
    times = [snapshot.timestamp for snapshot in snapshots]
    index = bisect_right(times, timestamp) - 1
    return set() if index < 0 else set(snapshots[index].geofence_ids)


def _object_is_active(item: AirspaceObjectRecord, timestamp: float) -> bool:
    return not (
        item.active_from is not None and timestamp < item.active_from
        or item.active_until is not None and timestamp > item.active_until
    )


def _draw_region(axis, view, item, color, alpha) -> None:
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        _draw_cylinder(axis, view, geometry, color, alpha)
    elif geometry_type == "Box3D":
        _draw_box(axis, view, geometry, color, alpha)


def _draw_cylinder(axis, view, geometry, color, alpha) -> None:
    cx = geometry["center_x"]
    cy = geometry["center_y"]
    radius = geometry["radius"]
    minimum_z = geometry["minimum_z"]
    maximum_z = geometry["maximum_z"]
    if view == "3d":
        angles = np.linspace(0.0, 2.0 * np.pi, 30)
        heights = np.array([minimum_z, maximum_z])
        angles, heights = np.meshgrid(angles, heights)
        x = cx + radius * np.cos(angles)
        y = cy + radius * np.sin(angles)
        axis.plot_surface(x, y, heights, color=color, alpha=alpha, linewidth=0)
        for height in (minimum_z, maximum_z):
            axis.plot(
                cx + radius * np.cos(angles[0]),
                cy + radius * np.sin(angles[0]),
                height,
                color=color,
                alpha=min(1.0, alpha + 0.25),
            )
    elif view == "xy":
        axis.add_patch(Circle((cx, cy), radius, color=color, alpha=alpha))
    else:
        horizontal_center = cx if view == "xz" else cy
        axis.add_patch(Rectangle(
            (horizontal_center - radius, minimum_z),
            2.0 * radius,
            maximum_z - minimum_z,
            color=color,
            alpha=alpha,
        ))


def _draw_box(axis, view, geometry, color, alpha) -> None:
    x0, x1 = geometry["minimum_x"], geometry["maximum_x"]
    y0, y1 = geometry["minimum_y"], geometry["maximum_y"]
    z0, z1 = geometry["minimum_z"], geometry["maximum_z"]
    if view == "3d":
        vertices = [
            (x0, y0, z0), (x1, y0, z0),
            (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1),
            (x1, y1, z1), (x0, y1, z1),
        ]
        faces = [
            [vertices[index] for index in face]
            for face in (
                (0, 1, 2, 3), (4, 5, 6, 7),
                (0, 1, 5, 4), (2, 3, 7, 6),
                (1, 2, 6, 5), (0, 3, 7, 4),
            )
        ]
        axis.add_collection3d(Poly3DCollection(
            faces,
            facecolors=color,
            edgecolors=color,
            alpha=alpha,
        ))
    else:
        bounds = {
            "xy": (x0, x1, y0, y1),
            "xz": (x0, x1, z0, z1),
            "yz": (y0, y1, z0, z1),
        }[view]
        axis.add_patch(Rectangle(
            (bounds[0], bounds[2]),
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            color=color,
            alpha=alpha,
        ))


def _draw_wind_vector(axis, view, item) -> None:
    velocity = item.properties.get("velocity")
    if not velocity or math.isclose(np.linalg.norm(velocity), 0.0):
        return
    center = _geometry_center(item)
    if center is None:
        return
    if view == "3d":
        axis.quiver(*center, *velocity, color="dodgerblue", length=3.0)
    else:
        first, second = _projection_indices(view)
        axis.quiver(
            center[first],
            center[second],
            velocity[first],
            velocity[second],
            color="dodgerblue",
            angles="xy",
            scale_units="xy",
            scale=0.25,
        )


def _geometry_center(item: AirspaceObjectRecord):
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        return (
            geometry["center_x"],
            geometry["center_y"],
            (geometry["minimum_z"] + geometry["maximum_z"]) / 2.0,
        )
    if geometry_type == "Box3D":
        return (
            (geometry["minimum_x"] + geometry["maximum_x"]) / 2.0,
            (geometry["minimum_y"] + geometry["maximum_y"]) / 2.0,
            (geometry["minimum_z"] + geometry["maximum_z"]) / 2.0,
        )
    return None


def _plot_positions(axis, view, positions, **kwargs) -> None:
    if not positions:
        return
    values = np.asarray(positions, dtype=float)
    if view == "3d":
        axis.plot(values[:, 0], values[:, 1], values[:, 2], **kwargs)
    else:
        first, second = _projection_indices(view)
        axis.plot(values[:, first], values[:, second], **kwargs)


def _scatter_positions(axis, view, positions, **kwargs) -> None:
    values = np.asarray(positions, dtype=float)
    if view == "3d":
        axis.scatter(values[:, 0], values[:, 1], values[:, 2], **kwargs)
    else:
        first, second = _projection_indices(view)
        axis.scatter(values[:, first], values[:, second], **kwargs)


def _label_position(axis, view, position, label) -> None:
    if view == "3d":
        axis.text(*position, label, fontsize=8)
    else:
        first, second = _projection_indices(view)
        axis.annotate(
            label,
            (position[first], position[second]),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
        )


def _projection_indices(view: View) -> tuple[int, int]:
    return {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[view]


def _configure_axis(axis, data, config, timestamp) -> None:
    limits = _axis_limits(data, config)
    title = config.title or "Simulation dataset"
    axis.set_title(f"{title} — t={timestamp:.1f} s")
    if config.view == "3d":
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_zlabel("z")
        axis.set_xlim(*limits[0])
        axis.set_ylim(*limits[1])
        axis.set_zlim(*limits[2])
        spans = [maximum - minimum for minimum, maximum in limits]
        axis.set_box_aspect([max(span, 1e-6) for span in spans])
        axis.view_init(elev=25, azim=-55)
    else:
        first, second = _projection_indices(config.view)
        labels = ("x", "y", "z")
        axis.set_xlabel(labels[first])
        axis.set_ylabel(labels[second])
        axis.set_xlim(*limits[first])
        axis.set_ylim(*limits[second])
        axis.set_aspect("equal", adjustable="box")
    axis.grid(config.show_grid, alpha=0.25)
    if config.show_legend:
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        active_geofences = _active_geofence_ids(
            data.airspace_snapshots,
            timestamp,
        )
        if not data.airspace_snapshots:
            active_geofences = {
                item.object_id
                for item in data.airspace_objects
                if item.object_type == "geofence"
                and _object_is_active(item, timestamp)
            }
        if config.show_geofences and active_geofences:
            unique["Geofence"] = Patch(color="crimson", alpha=0.35)
        if config.show_obstacles and any(
            item.object_type == "obstacle"
            for item in data.airspace_objects
        ):
            unique["Obstáculo"] = Patch(color="dimgray", alpha=0.45)
        if config.show_wind_zones and any(
            item.object_type == "wind_zone"
            and _object_is_active(item, timestamp)
            for item in data.airspace_objects
        ):
            unique["Wind zone"] = Patch(
                color="deepskyblue",
                alpha=0.30,
            )
        if unique:
            axis.legend(
                unique.values(),
                unique.keys(),
                loc="upper left",
                fontsize=7,
            )


def _axis_limits(data, config):
    positions = []
    for name in data.uav_names:
        positions.extend(state.position for state in data.trajectories[name])
        plan = data.proposed_plans[name]
        if plan is not None:
            positions.extend(wp.position() for wp in plan.waypoint_list)
    for item in data.airspace_objects:
        visible = (
            item.object_type == "geofence" and config.show_geofences
            or item.object_type == "obstacle" and config.show_obstacles
            or item.object_type == "wind_zone" and config.show_wind_zones
        )
        if visible:
            positions.extend(_geometry_extremes(item))
    if not positions:
        positions = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
    values = np.asarray(positions, dtype=float)
    minimums = values.min(axis=0)
    maximums = values.max(axis=0)
    spans = maximums - minimums
    padding = np.maximum(spans * 0.06, 1.0)
    return tuple(
        (float(minimum - pad), float(maximum + pad))
        for minimum, maximum, pad in zip(minimums, maximums, padding)
    )


def _geometry_extremes(item):
    geometry = item.geometry
    geometry_type = item.geometry_type.rsplit(".", 1)[-1]
    if geometry_type == "Cylinder3D":
        radius = geometry["radius"]
        return [
            (
                geometry["center_x"] - radius,
                geometry["center_y"] - radius,
                geometry["minimum_z"],
            ),
            (
                geometry["center_x"] + radius,
                geometry["center_y"] + radius,
                geometry["maximum_z"],
            ),
        ]
    if geometry_type == "Box3D":
        return [
            (
                geometry["minimum_x"],
                geometry["minimum_y"],
                geometry["minimum_z"],
            ),
            (
                geometry["maximum_x"],
                geometry["maximum_y"],
                geometry["maximum_z"],
            ),
        ]
    return []


def _frame_times(data, config) -> list[float]:
    start = max(0.0, _start_time(config))
    end = min(
        data.simulation_time,
        data.simulation_time if config.end_time is None else config.end_time,
    )
    if end < start:
        raise ValueError("selected time range is outside the simulation")
    if math.isclose(start, end):
        return [start]
    count = math.floor((end - start) / config.frame_step)
    times = [start + index * config.frame_step for index in range(count + 1)]
    if not math.isclose(times[-1], end):
        times.append(end)
    return times


def _start_time(config) -> float:
    return 0.0 if config.start_time is None else config.start_time


def _bounded_time(data, config, timestamp):
    start = _start_time(config)
    end = min(
        data.simulation_time,
        data.simulation_time if config.end_time is None else config.end_time,
    )
    if not start <= timestamp <= end:
        raise ValueError(f"timestamp must be between {start} and {end}")
    return timestamp
