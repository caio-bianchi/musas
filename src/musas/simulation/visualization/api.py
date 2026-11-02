import math
from pathlib import Path

from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.figure import Figure

from musas.simulation.visualization.config import VisualizationConfig
from musas.simulation.visualization.data import VisualizationData, load_data
from musas.simulation.visualization.rendering import draw_frame, new_figure
from musas.simulation.visualization.style import academic_style_context


def plot_dataset(
    dataset_path: str | Path,
    config: VisualizationConfig | None = None,
    timestamp: float | None = None,
    output_path: str | Path | None = None,
    dpi: int = 300,
) -> Figure:
    """Create a publication-ready view and optionally save it.

    Raster formats use ``dpi``. Use a ``.pdf`` or ``.svg`` output path for a
    vector figure suitable for direct inclusion in academic documents.
    """
    config = config or VisualizationConfig()
    data = load_data(dataset_path, config)
    current_time = _bounded_time(
        data,
        config,
        data.simulation_time if timestamp is None else float(timestamp),
    )
    with academic_style_context():
        figure, axis = new_figure(config)
        draw_frame(axis, data, config, current_time)
    if output_path is not None:
        destination = Path(output_path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(
            destination,
            dpi=dpi,
            bbox_inches="tight",
            facecolor="white",
        )
    return figure


def animate_dataset(
    dataset_path: str | Path,
    config: VisualizationConfig | None = None,
    output_path: str | Path | None = None,
    fps: float = 10.0,
    dpi: int = 120,
    repeat: bool = True,
) -> FuncAnimation:
    """Create an animation and optionally save it as GIF or MP4."""
    if fps <= 0:
        raise ValueError("fps must be greater than zero")
    config = config or VisualizationConfig()
    data = load_data(dataset_path, config)
    frame_times = _frame_times(data, config)
    with academic_style_context():
        figure, axis = new_figure(config)

    def update(timestamp: float):
        with academic_style_context():
            axis.clear()
            draw_frame(axis, data, config, timestamp)
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


def _frame_times(
    data: VisualizationData,
    config: VisualizationConfig,
) -> list[float]:
    start = max(
        0.0,
        0.0 if config.start_time is None else config.start_time,
    )
    end = min(
        data.simulation_time,
        data.simulation_time
        if config.end_time is None
        else config.end_time,
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


def _bounded_time(
    data: VisualizationData,
    config: VisualizationConfig,
    timestamp: float,
) -> float:
    start = 0.0 if config.start_time is None else config.start_time
    end = min(
        data.simulation_time,
        data.simulation_time
        if config.end_time is None
        else config.end_time,
    )
    if not start <= timestamp <= end:
        raise ValueError(f"timestamp must be between {start} and {end}")
    return timestamp
