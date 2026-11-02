from dataclasses import dataclass
from typing import Literal


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
    show_labels: bool = False
    show_legend: bool = True
    show_grid: bool = True
    start_time: float | None = None
    end_time: float | None = None
    frame_step: float = 1.0
    figure_size: tuple[float, float] = (8.0, 5.5)
    title: str | None = None
    show_timestamp: bool = True
    spatial_unit: str | None = None
    time_unit: str = "s"

    def __post_init__(self) -> None:
        if self.view not in ("3d", "xy", "xz", "yz"):
            raise ValueError("view must be '3d', 'xy', 'xz' or 'yz'")
        if self.frame_step <= 0:
            raise ValueError("frame_step must be greater than zero")
        if not self.time_unit:
            raise ValueError("time_unit cannot be empty")
        if (
            self.start_time is not None
            and self.end_time is not None
            and self.end_time < self.start_time
        ):
            raise ValueError("end_time cannot be earlier than start_time")


def projection_indices(view: View) -> tuple[int, int]:
    return {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[view]
