import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.lines import Line2D
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
    active_plan,
    latest_track,
)
from musas.simulation.visualization.style import (
    AIRSPACE_STYLES,
    GRID,
    INK,
    MUTED_INK,
    SPINE,
    UAV_PALETTE,
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
    axis.set_facecolor("white")
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
                    linestyle=(0, (5, 2.5)),
                    linewidth=1.25,
                    marker="o",
                    markersize=3.8,
                    markerfacecolor="white",
                    markeredgecolor=color,
                    markeredgewidth=0.8,
                    alpha=0.82,
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
                linewidth=2.0,
                solid_capstyle="round",
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
                    color=["#A61B1B"],
                    marker="X",
                    edgecolor="white",
                    linewidth=0.7,
                    s=82,
                    zorder=6,
                )
            else:
                _scatter_positions(
                    axis,
                    config.view,
                    [position],
                    color=[color],
                    edgecolor="white",
                    linewidth=0.9,
                    s=52,
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
                    alpha=0.62,
                    linewidth=0.9,
                    s=22,
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
                    linestyle=(0, (1.2, 1.8)),
                    linewidth=1.6,
                    marker=".",
                    markersize=4,
                    zorder=5,
                )

    _configure_axis(axis, data, config, timestamp)


def _uav_colors(uav_names: tuple[str, ...]) -> dict[str, object]:
    count = len(uav_names)
    if count <= len(UAV_PALETTE):
        return {
            name: UAV_PALETTE[index]
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
    text_style = {
        "fontsize": 7.5,
        "color": INK,
        "bbox": {
            "boxstyle": "round,pad=0.16",
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.78,
        },
    }
    if view == "3d":
        axis.text(
            *position,
            label,
            **text_style,
        )
    else:
        first, second = projection_indices(view)
        axis.annotate(
            label,
            (position[first], position[second]),
            xytext=(4, 4),
            textcoords="offset points",
            **text_style,
        )


def _configure_axis(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> None:
    limits = _axis_limits(data, config)
    if config.title:
        axis.set_title(
            config.title,
            loc="left",
            pad=11,
            color=INK,
            fontweight="bold",
            fontsize=11,
        )
    if config.show_timestamp:
        time_label = rf"$t = {timestamp:.1f}\,\mathrm{{{config.time_unit}}}$"
        timestamp_text = axis.text2D if config.view == "3d" else axis.text
        timestamp_text(
            0.985,
            0.98,
            time_label,
            transform=axis.transAxes,
            ha="right",
            va="top",
            fontsize=9,
            color=INK,
            bbox={
                "boxstyle": "round,pad=0.22",
                "facecolor": "white",
                "edgecolor": GRID,
                "linewidth": 0.6,
                "alpha": 0.92,
            },
            zorder=20,
        )

    labels = ("x", "y", "z")
    coordinate_labels = tuple(
        _axis_label(label, config.spatial_unit)
        for label in labels
    )
    if config.view == "3d":
        axis.set_xlabel(coordinate_labels[0], labelpad=7)
        axis.set_ylabel(coordinate_labels[1], labelpad=7)
        axis.set_zlabel(coordinate_labels[2], labelpad=7)
        axis.set_xlim(*limits[0])
        axis.set_ylim(*limits[1])
        axis.set_zlim(*limits[2])
        for coordinate_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
            coordinate_axis.set_major_locator(MaxNLocator(nbins=6))
            coordinate_axis.pane.set_facecolor((1.0, 1.0, 1.0, 1.0))
            coordinate_axis.pane.set_edgecolor(GRID)
            coordinate_axis._axinfo["grid"].update({
                "color": GRID,
                "linestyle": ":",
                "linewidth": 0.55,
            })
        spans = [maximum - minimum for minimum, maximum in limits]
        axis.set_box_aspect([max(span, 1e-6) for span in spans])
        axis.set_proj_type("ortho")
        axis.view_init(elev=24, azim=-58)
    else:
        first, second = projection_indices(config.view)
        axis.set_xlabel(coordinate_labels[first])
        axis.set_ylabel(coordinate_labels[second])
        axis.set_xlim(*limits[first])
        axis.set_ylim(*limits[second])
        axis.set_aspect("equal", adjustable="box")
        axis.xaxis.set_major_locator(MaxNLocator(nbins=8))
        axis.yaxis.set_major_locator(MaxNLocator(nbins=8))
        axis.tick_params(top=True, right=True)
        for spine in axis.spines.values():
            spine.set_color(SPINE)
            spine.set_linewidth(0.75)
    axis.tick_params(colors=MUTED_INK, labelsize=8.5, direction="in")
    axis.grid(
        config.show_grid,
        color=GRID,
        linestyle=":",
        linewidth=0.6,
        alpha=0.78,
    )
    if config.show_legend:
        _draw_legend(axis, data, config)


def _draw_legend(
    axis,
    data: VisualizationData,
    config: VisualizationConfig,
) -> None:
    colors = _uav_colors(data.uav_names)

    uav_handles = [
        Line2D(
            [0],
            [0],
            color=colors[name],
            linewidth=2.0,
            marker="o",
            markersize=4.5,
            markeredgecolor="white",
            label=name,
        )
        for name in data.uav_names
    ]
    layer_handles = []
    if config.show_flight_plans:
        layer_handles.append(Line2D(
            [0], [0], color=INK, linewidth=1.25,
            linestyle=(0, (5, 2.5)), marker="o", markersize=3.5,
            markerfacecolor="white", label="Flight plan",
        ))
    if config.show_real_trajectories:
        layer_handles.append(Line2D(
            [0], [0], color=INK, linewidth=2.0,
            label="Actual trajectory",
        ))
    if config.show_tracks:
        layer_handles.append(Line2D(
            [0], [0], color=INK, linewidth=1.6,
            linestyle=(0, (1.2, 1.8)), marker=".", markersize=4,
            label="Predicted track",
        ))
    if config.show_sensor_reports:
        layer_handles.append(Line2D(
            [0], [0], color=INK, linewidth=0, marker="x",
            markersize=5, label="Sensor measurement",
        ))
    if config.show_geofences and any(
        item.object_type == "geofence"
        for item in data.airspace_objects
    ):
        layer_handles.append(_region_legend_patch("geofence"))
    if config.show_obstacles and any(
        item.object_type == "obstacle"
        for item in data.airspace_objects
    ):
        layer_handles.append(_region_legend_patch("obstacle"))
    if config.show_wind_zones and any(
        item.object_type == "wind_zone"
        for item in data.airspace_objects
    ):
        layer_handles.append(_region_legend_patch("wind_zone"))

    legend_style = {
        "loc": "upper left",
        "bbox_to_anchor": (1.015, 1.0),
        "borderaxespad": 0.0,
        "fontsize": 7.5,
        "frameon": True,
        "fancybox": False,
        "framealpha": 0.96,
        "facecolor": "white",
        "edgecolor": "#9CA3AF",
        "labelcolor": INK,
        "handlelength": 2.4,
        "borderpad": 0.65,
        "labelspacing": 0.45,
    }
    if uav_handles:
        uav_legend = axis.legend(
            handles=uav_handles,
            title="UAVs",
            ncol=max(1, (len(uav_handles) + 11) // 12),
            **legend_style,
        )
        uav_legend.get_title().set_fontweight("bold")
        axis.add_artist(uav_legend)
    if layer_handles:
        layer_legend = axis.legend(
            handles=layer_handles,
            title="Layers",
            loc="lower left",
            bbox_to_anchor=(1.015, 0.0),
            **{
                key: value
                for key, value in legend_style.items()
                if key not in {"loc", "bbox_to_anchor"}
            },
        )
        layer_legend.get_title().set_fontweight("bold")


def _axis_label(coordinate: str, unit: str | None) -> str:
    label = f"${coordinate}$"
    return label if unit is None else f"{label} [{unit}]"


def _region_legend_patch(object_type: str) -> Patch:
    style = AIRSPACE_STYLES[object_type]
    return Patch(
        facecolor=to_rgba(style["color"], style["alpha"]),
        edgecolor=style["color"],
        hatch=style["hatch"],
        linewidth=0.9,
        label=style["label"],
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
