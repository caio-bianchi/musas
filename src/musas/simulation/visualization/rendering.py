import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
import numpy as np

from musas.simulation.visualization.airspace import (
    draw_airspace,
    geometry_extremes,
)
from musas.simulation.visualization.config import (
    View,
    VisualizationConfig,
    projection_indices,
)
from musas.simulation.visualization.data import (
    VisualizationData,
    active_object_ids,
    active_plan,
    latest_track,
    object_is_active,
)


def new_figure(config: VisualizationConfig):
    figure = plt.figure(
        figsize=config.figure_size,
        facecolor="white",
        layout="constrained",
    )
    axis = (
        figure.add_subplot(111, projection="3d")
        if config.view == "3d"
        else figure.add_subplot(111)
    )
    axis.set_facecolor("#f8fafc")
    return figure, axis


def draw_frame(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    colors = _uav_colors(data.uav_names)
    draw_airspace(axis, data, config, timestamp)
    start_time = 0.0 if config.start_time is None else config.start_time

    for name in data.uav_names:
        color = colors[name]
        if config.show_flight_plans:
            plan = active_plan(data, name, timestamp)
            if plan is not None:
                _plot_positions(
                    axis,
                    config.view,
                    [waypoint.position() for waypoint in plan.waypoint_list],
                    color=color,
                    linestyle="--",
                    linewidth=1.5,
                    alpha=0.72,
                    label=f"{name} — Flight plan",
                    zorder=2,
                )

        trajectory = [
            state
            for state in data.trajectories[name]
            if start_time <= state.timestamp <= timestamp
        ]
        if trajectory and config.show_real_trajectories:
            _plot_positions(
                axis,
                config.view,
                [state.position for state in trajectory],
                color=color,
                linewidth=2.4,
                solid_capstyle="round",
                label=f"{name} — Actual trajectory",
                zorder=4,
            )
        if trajectory and config.show_current_positions:
            current_state = trajectory[-1]
            position = current_state.position
            if current_state.status == "COLLIDED":
                _scatter_positions(
                    axis,
                    config.view,
                    [position],
                    color=[color],
                    marker="x",
                    linewidth=2.4,
                    s=90,
                    zorder=6,
                )
            else:
                _scatter_positions(
                    axis,
                    config.view,
                    [position],
                    color=[color],
                    edgecolor="white",
                    linewidth=1.2,
                    s=64,
                    zorder=6,
                )
            if config.show_labels:
                _label_position(axis, config.view, position, name)

        if config.show_sensor_reports:
            reports = [
                report
                for report in data.reports[name]
                if start_time <= report.received_at <= timestamp
            ]
            if reports:
                _scatter_positions(
                    axis,
                    config.view,
                    [report.position for report in reports],
                    marker="x",
                    color=[color],
                    alpha=0.60,
                    linewidth=1.1,
                    s=26,
                    label=f"{name} — Sensor measurements",
                    zorder=3,
                )

        if config.show_tracks:
            track = latest_track(data.predictions[name], timestamp)
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
                    linewidth=2.2,
                    marker=".",
                    markersize=5,
                    label=f"{name} — Predicted track",
                    zorder=5,
                )

    _configure_axis(axis, data, config, timestamp)


def _uav_colors(uav_names: tuple[str, ...]) -> dict[str, object]:
    count = len(uav_names)
    if count <= 10:
        color_map = plt.get_cmap("tab10")
        return {
            name: color_map(index)
            for index, name in enumerate(uav_names)
        }
    if count <= 20:
        color_map = plt.get_cmap("tab20")
        return {
            name: color_map(index)
            for index, name in enumerate(uav_names)
        }

    color_map = plt.get_cmap("turbo")
    return {
        name: color_map(index / max(count - 1, 1))
        for index, name in enumerate(uav_names)
    }


def _plot_positions(axis, view: View, positions, **kwargs) -> None:
    if not positions:
        return
    values = np.asarray(positions, dtype=float)
    if view == "3d":
        axis.plot(values[:, 0], values[:, 1], values[:, 2], **kwargs)
    else:
        first, second = projection_indices(view)
        axis.plot(values[:, first], values[:, second], **kwargs)


def _scatter_positions(axis, view: View, positions, **kwargs) -> None:
    values = np.asarray(positions, dtype=float)
    if view == "3d":
        axis.scatter(values[:, 0], values[:, 1], values[:, 2], **kwargs)
    else:
        first, second = projection_indices(view)
        axis.scatter(values[:, first], values[:, second], **kwargs)


def _label_position(axis, view: View, position, label: str) -> None:
    if view == "3d":
        axis.text(
            *position,
            label,
            fontsize=8,
            color="#1e293b",
        )
    else:
        first, second = projection_indices(view)
        axis.annotate(
            label,
            (position[first], position[second]),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
            color="#1e293b",
        )


def _configure_axis(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    limits = _axis_limits(data, config)
    title = config.title or "Simulation dataset"
    axis.set_title(
        f"{title}  |  Time: {timestamp:.1f} s",
        loc="left",
        pad=14,
        color="#0f172a",
        fontweight="bold",
        fontsize=13,
    )
    if config.view == "3d":
        axis.set_xlabel("X", labelpad=8)
        axis.set_ylabel("Y", labelpad=8)
        axis.set_zlabel("Z", labelpad=8)
        axis.set_xlim(*limits[0])
        axis.set_ylim(*limits[1])
        axis.set_zlim(*limits[2])
        for coordinate_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
            coordinate_axis.set_major_locator(MaxNLocator(nbins=6))
            coordinate_axis.pane.set_facecolor((0.97, 0.98, 1.0, 1.0))
            coordinate_axis.pane.set_edgecolor("#cbd5e1")
        spans = [maximum - minimum for minimum, maximum in limits]
        axis.set_box_aspect([max(span, 1e-6) for span in spans])
        axis.view_init(elev=25, azim=-55)
    else:
        first, second = projection_indices(config.view)
        labels = ("X", "Y", "Z")
        axis.set_xlabel(labels[first])
        axis.set_ylabel(labels[second])
        axis.set_xlim(*limits[first])
        axis.set_ylim(*limits[second])
        axis.set_aspect("equal", adjustable="box")
        axis.xaxis.set_major_locator(MaxNLocator(nbins=8))
        axis.yaxis.set_major_locator(MaxNLocator(nbins=8))
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            axis.spines[side].set_color("#94a3b8")
    axis.tick_params(colors="#475569", labelsize=8)
    axis.grid(
        config.show_grid,
        color="#cbd5e1",
        linestyle="--",
        linewidth=0.7,
        alpha=0.65,
    )
    if config.show_legend:
        _draw_legend(axis, data, config, timestamp)


def _draw_legend(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    active_geofences = active_object_ids(data, "geofence", timestamp)
    active_wind_zones = active_object_ids(data, "wind_zone", timestamp)

    if config.show_geofences and active_geofences:
        unique["Geofence"] = Patch(color="crimson", alpha=0.35)
    if config.show_obstacles and any(
        item.object_type == "obstacle"
        for item in data.airspace_objects
    ):
        unique["Obstacle"] = Patch(color="dimgray", alpha=0.45)
    if config.show_wind_zones and any(
        item.object_type == "wind_zone"
        and item.object_id in active_wind_zones
        and object_is_active(item, timestamp)
        for item in data.airspace_objects
    ):
        unique["Wind zone"] = Patch(color="deepskyblue", alpha=0.30)
    if unique:
        entry_count = len(unique)
        axis.legend(
            unique.values(),
            unique.keys(),
            loc="upper left",
            bbox_to_anchor=(1.02, 1.0),
            borderaxespad=0.0,
            fontsize=7 if entry_count <= 20 else 6,
            frameon=True,
            fancybox=True,
            framealpha=0.96,
            facecolor="white",
            edgecolor="#cbd5e1",
            labelcolor="#334155",
            handlelength=2.4,
            borderpad=0.8,
            labelspacing=0.55,
        )


def _axis_limits(
    data: VisualizationData,
    config: VisualizationConfig,
):
    positions = []
    for name in data.uav_names:
        positions.extend(state.position for state in data.trajectories[name])
        plan = data.proposed_plans[name]
        if plan is not None:
            positions.extend(
                waypoint.position()
                for waypoint in plan.waypoint_list
            )
    for item in data.airspace_objects:
        visible = (
            item.object_type == "geofence" and config.show_geofences
            or item.object_type == "obstacle" and config.show_obstacles
            or item.object_type == "wind_zone" and config.show_wind_zones
        )
        if visible:
            positions.extend(geometry_extremes(item))
    if not positions:
        positions = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]

    values = np.asarray(positions, dtype=float)
    minimums = values.min(axis=0)
    maximums = values.max(axis=0)
    spans = maximums - minimums

    # A planar or linear trajectory may have no variation along one of the
    # axes. In a 3-D view, using that near-zero span directly collapses the
    # corresponding axis and makes its tick labels overlap. Expand small axes
    # around their data center while preserving the scale of the largest axis.
    if config.view == "3d":
        reference_span = max(float(spans.max()), 1.0)
        minimum_span = 0.2 * reference_span
        centers = (minimums + maximums) / 2.0
        displayed_spans = np.maximum(spans, minimum_span)
        minimums = centers - displayed_spans / 2.0
        maximums = centers + displayed_spans / 2.0
        spans = displayed_spans

    padding = np.maximum(spans * 0.06, 1.0)
    return tuple(
        (float(minimum - pad), float(maximum + pad))
        for minimum, maximum, pad in zip(minimums, maximums, padding)
    )
