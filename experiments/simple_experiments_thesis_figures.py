"""Thesis figures of the simple validation experiments (Experiments 1--7).

The figures are drawn from the SQLite datasets written by the experiment
notebooks, so the simulations are not run again. Each figure combines several
views or instants of the same scenario, carries no title (the caption is
written in LaTeX), and uses one shared legend.

Run from the ``experiments`` directory:

    python simple_experiments_thesis_figures.py [output_directory]
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from musas.simulation import VisualizationConfig  # noqa: E402
from musas.simulation.visualization.data import load_data  # noqa: E402
from musas.simulation.visualization.rendering import draw_frame  # noqa: E402
from musas.simulation.visualization.style import (  # noqa: E402
    AIRSPACE_STYLES,
    UAV_PALETTE,
    academic_style_context,
)

OUTPUTS = Path("outputs")
FIGURES = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("figs") / "thesis"
TEXT_WIDTH_IN = 6.3
INK = "#202124"

STYLE = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8.5,
    "axes.titlesize": 8.5,
    "axes.labelsize": 8.5,
    "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5,
    "legend.fontsize": 8,
    "pdf.fonttype": 42,
}


def config(view="xy", *, tracks=False, obstacles=False, geofences=False):
    return VisualizationConfig(
        view=view,
        show_real_trajectories=True,
        show_current_positions=True,
        show_tracks=tracks,
        show_flight_plans=True,
        show_sensor_reports=False,
        show_geofences=geofences,
        show_wind_zones=False,
        show_obstacles=obstacles,
        show_legend=False,
        show_timestamp=False,
        spatial_unit="m",
    )


def panel(axis, dataset, cfg, timestamp, title, *, xlim=None, ylim=None,
          before_update=False):
    data = load_data(OUTPUTS / dataset, cfg)
    if before_update:
        # Events logged at the same instant are processed in order: draw the
        # state seen by the tactical assessment, i.e. the airspace at
        # `timestamp` with the plans and tracks that existed just before it.
        for name in data.uav_names:
            data.plan_events[name] = [
                event for event in data.plan_events.get(name, ())
                if event.timestamp < timestamp
            ]
            data.predictions[name] = {
                generated_at: states
                for generated_at, states in data.predictions[name].items()
                if generated_at < timestamp
            }
    draw_frame(axis, data, cfg, timestamp)
    if xlim is not None:
        axis.set_xlim(*xlim)
    if ylim is not None:
        axis.set_ylim(*ylim)
    if cfg.view == "3d":
        axis.set_title(title, loc="left", pad=0, y=0.98)
    else:
        axis.set_title(title, loc="left", pad=3)


def share_limits(axes):
    """Apply the union of the x and y limits to every 2D axis."""
    axes = list(axes)
    xs = [axis.get_xlim() for axis in axes]
    ys = [axis.get_ylim() for axis in axes]
    xlim = (min(x[0] for x in xs), max(x[1] for x in xs))
    ylim = (min(y[0] for y in ys), max(y[1] for y in ys))
    for axis in axes:
        axis.set_xlim(*xlim)
        axis.set_ylim(*ylim)


def legend(figure, *, uavs=(), tracks=False, region=None, collision=False):
    neutral = "#3A3A3A"
    handles = [
        Line2D([], [], color=color, linewidth=2.0, label=name)
        for name, color in zip(uavs, UAV_PALETTE)
    ]
    handles += [
        Line2D([], [], color=neutral, linestyle=(0, (5, 2.5)), marker="o",
               markersize=3.8, markerfacecolor="white", linewidth=1.25,
               label="Current flight plan"),
        Line2D([], [], color=neutral, linewidth=2.0,
               label="Executed trajectory"),
        Line2D([], [], color=neutral, linestyle="none", marker="o",
               markersize=6, markeredgecolor="white", label="Current position"),
    ]
    if tracks:
        handles.append(Line2D([], [], color=neutral, linestyle=(0, (1.2, 1.8)),
                              marker=".", markersize=4, linewidth=1.6,
                              label="Predicted track"))
    if collision:
        handles.append(Line2D([], [], color="#A61B1B", linestyle="none",
                              marker="X", markersize=7, markeredgecolor="white",
                              label="Collision"))
    if region is not None:
        style = AIRSPACE_STYLES[region]
        handles.append(Patch(facecolor=style["color"], alpha=0.35,
                             edgecolor=style["color"], hatch=style["hatch"],
                             label=style["label"]))
    figure.legend(handles=handles, loc="outside lower center",
                  ncol=len(handles) if len(handles) <= 5
                  else -(-len(handles) // 2), frameon=False,
                  handlelength=2.4, columnspacing=1.4)


def save(figure, name):
    FIGURES.mkdir(parents=True, exist_ok=True)
    figure.savefig(FIGURES / f"{name}.pdf", bbox_inches="tight")
    figure.savefig(FIGURES / f"{name}.png", dpi=250, bbox_inches="tight")
    plt.close(figure)


def experiment_1():
    figure, axes = plt.subplots(2, 2, figsize=(TEXT_WIDTH_IN, 4.6),
                                layout="constrained")
    for index, (axis, tau) in enumerate(zip(axes.flat, ("0_0", "0_5", "1_5", "4_0"))):
        label = tau.replace("_", ".")
        panel(axis, f"output_1/tau_{tau}.sqlite", config(), 80.0,
              rf"({chr(97 + index)}) $\tau={label}$ s",
              xlim=(-75, 135), ylim=(-30, 110))
    legend(figure)
    save(figure, "exp1_dynamics")


def experiment_2():
    figure, axes = plt.subplots(2, 1, figsize=(TEXT_WIDTH_IN, 3.6),
                                layout="constrained")
    cfg = config(obstacles=True)
    panel(axes[0], "output_2/without_sd.sqlite", cfg, 45.0,
          "(a) Without SD (collision at $t=11.5$ s)")
    panel(axes[1], "output_2/with_sd.sqlite", cfg, 45.0,
          "(b) With SD (strategic detour)")
    share_limits(axes)
    legend(figure, region="obstacle", collision=True)
    save(figure, "exp2_strategic_obstacle")


def experiment_3():
    cfg = config(tracks=True, obstacles=True)
    instants = (
        (3.0, "(a) $t=3$ s: no conflict predicted"),
        (5.7, "(b) $t=5.7$ s: track reaches the obstacle"),
        (8.0, "(c) $t=8$ s: tactical detour"),
        (45.0, "(d) Final trajectory"),
    )
    figure, axes = plt.subplots(2, 2, figsize=(TEXT_WIDTH_IN, 3.0),
                                layout="constrained")
    for axis, (timestamp, title) in zip(axes.flat, instants):
        panel(axis, "output_3/with_td.sqlite", cfg, timestamp, title)
    share_limits(axes.flat)
    legend(figure, tracks=True, region="obstacle")
    save(figure, "exp3_tactical_obstacle")


def experiment_4():
    figure, axes = plt.subplots(2, 2, figsize=(TEXT_WIDTH_IN, 5.9),
                                layout="constrained")
    cfg = config()
    limits = {"xlim": (-110, 110), "ylim": (-110, 110)}
    panel(axes[0, 0], "output_4/without_sd.sqlite", cfg, 12.0,
          "(a) Without SD, $t=12$ s", **limits)
    panel(axes[0, 1], "output_4/with_sd.sqlite", cfg, 7.0,
          "(b) With SD, $t=7$ s: departure of B", **limits)
    panel(axes[1, 0], "output_4/with_sd.sqlite", cfg, 12.0,
          "(c) With SD, $t=12$ s", **limits)
    panel(axes[1, 1], "output_4/with_sd.sqlite", cfg, 17.0,
          "(d) With SD, $t=17$ s: B crosses", **limits)
    legend(figure, uavs=("UAV A", "UAV B"), collision=True)
    save(figure, "exp4_strategic_uavs")


def experiment_5():
    instants = (
        (3.0, "(a) $t=3$ s: UASs approaching", True),
        (4.2, "(b) $t=4.2$ s: predicted tracks in conflict", True),
        (12.0, "(c) $t=12$ s: B passes below A", False),
        (45.0, "(d) Final trajectories", False),
    )
    figure = plt.figure(figsize=(TEXT_WIDTH_IN, 5.6), layout="constrained")
    for index, (timestamp, title, tracks) in enumerate(instants):
        axis = figure.add_subplot(2, 2, index + 1, projection="3d")
        panel(axis, "output_5/with_td.sqlite", config("3d", tracks=tracks),
              timestamp, title)
        axis.set_xlim(-110, 110)
        axis.set_ylim(-110, 110)
        axis.set_zlim(30, 110)
        axis.set_box_aspect((1.0, 1.0, 0.55), zoom=0.9)
        axis.view_init(elev=38, azim=-60)
        if index == 1:
            _mark_track_ends(axis, "output_5/with_td.sqlite", generated_at=4.0)
    legend(figure, uavs=("UAV A", "UAV B"), tracks=True)
    save(figure, "exp5_tactical_uavs")


def _mark_track_ends(axis, dataset, *, generated_at):
    """Join the last predicted states of A and B and label their distance."""
    import math
    import sqlite3

    with sqlite3.connect(OUTPUTS / dataset) as connection:
        ends = [
            connection.execute(
                "SELECT predicted_timestamp, x, y, z FROM tracking_predictions "
                "WHERE uav_name = ? AND generated_at = ? "
                "ORDER BY predicted_timestamp DESC LIMIT 1",
                (name, generated_at),
            ).fetchone()
            for name in ("UAV A", "UAV B")
        ]
    (time, *a), (_, *b) = ends
    distance = math.dist(a, b)
    axis.plot(*zip(a, b), color="#A61B1B", linewidth=1.6, marker="o",
              markersize=3, zorder=10)
    middle = [(p + q) / 2 for p, q in zip(a, b)]
    axis.text(middle[0] + 45, middle[1] - 15, middle[2],
              f"{distance:.0f} m apart\nat $t={time:.0f}$ s", color="#A61B1B",
              fontsize=7.5, zorder=11,
              bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.85,
                    "pad": 0.8})


def _geofence_sequence(dataset, instants, name, height):
    figure, axes = plt.subplots(len(instants), 1,
                                figsize=(TEXT_WIDTH_IN, height),
                                layout="constrained")
    for axis, (timestamp, title, *options) in zip(axes, instants):
        tracks = not options or options[0]
        before_update = len(options) > 1 and options[1]
        cfg = config(tracks=tracks, geofences=True)
        panel(axis, dataset, cfg, timestamp, title, ylim=(-48, 30),
              before_update=before_update)
    legend(figure, tracks=True, region="geofence")
    save(figure, name)


def experiment_6():
    _geofence_sequence(
        "output_6/with_td.sqlite",
        (
            (8.0, "(a) $t=8$ s: before the geofence is activated"),
            (12.0, "(b) $t=12$ s: geofence activated, before the assessment",
             True, True),
            (12.0, "(c) $t=12$ s: escape plan issued"),
            (19.0, "(d) $t=19$ s: contour of the restriction", False),
            (45.0, "(e) Final trajectory"),
        ),
        "exp6_geofence_around",
        7.9,
    )


def experiment_7():
    _geofence_sequence(
        "output_7/with_td.sqlite",
        (
            (7.0, "(a) $t=7$ s: before the geofence is activated"),
            (10.0, "(b) $t=10$ s: geofence activated, before the assessment",
             True, True),
            (11.0, "(c) $t=11$ s: the track follows the detour plan"),
            (13.0, "(d) $t=13$ s: detour in progress"),
            (45.0, "(e) Final trajectory"),
        ),
        "exp7_geofence_ahead",
        7.9,
    )


if __name__ == "__main__":
    with academic_style_context(), plt.rc_context(STYLE):
        for function in (experiment_1, experiment_2, experiment_3,
                         experiment_4, experiment_5, experiment_6,
                         experiment_7):
            function()
            print("Saved", function.__name__)
