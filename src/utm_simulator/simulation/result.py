from dataclasses import dataclass
from pathlib import Path
from typing import Any

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.airspace.restrictions import Geofence
from utm_simulator.airspace.wind import WindZone
from utm_simulator.core.simulation_event import SimulationEvent
from utm_simulator.domain.flight import FlightPlan
from utm_simulator.domain.tracking import TrackPrediction, TrackingReport
from utm_simulator.uav.uav import UAVStatus


@dataclass(frozen=True)
class UAVSnapshot:
    """Serializable snapshot of a UAV at the end of a run."""

    name: str
    status: UAVStatus
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    dynamics_type: str
    guidance_type: str
    sensor_type: str


@dataclass(frozen=True)
class UAVStateSample:
    """Position, velocity, and status recorded at one instant."""

    timestamp: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    status: UAVStatus


@dataclass(frozen=True)
class AirspaceSnapshot:
    """Geofences and wind zones present at one simulation instant."""

    timestamp: float
    geofences: tuple[Geofence, ...]
    wind_zones: tuple[WindZone, ...]


@dataclass
class SimulationResult:
    """Data collected up to the current simulation time."""

    time: float
    airspace: Airspace
    uavs: dict[str, UAVSnapshot]
    proposed_flight_plans: dict[str, FlightPlan]
    flight_plan_history: dict[str, tuple[FlightPlan, ...]]
    state_history: dict[str, list[UAVStateSample]]
    airspace_history: list[AirspaceSnapshot]
    tracking_reports: list[TrackingReport]
    tracking_predictions: list[TrackPrediction]
    event_history: list[SimulationEvent]
    configuration: dict[str, Any]
    dataset_path: Path | None = None
    history_complete: bool = True

    @property
    def latest_flight_plans(self) -> dict[str, FlightPlan]:
        """Return the latest strategic or tactical plan for each UAV."""
        return {
            uav_name: history[-1]
            for uav_name, history in self.flight_plan_history.items()
            if history
        }

    def save(
        self,
        path: str | Path,
        metadata: dict[str, Any] | None = None,
        overwrite: bool = False,
    ) -> Path:
        """Persist this result as a self-contained SQLite dataset."""
        if not self.history_complete:
            raise RuntimeError(
                "this result contains only the latest in-memory buffer; "
                "use its dataset_path for the complete history"
            )

        from utm_simulator.simulation.dataset import SimulationDatasetRecorder

        return SimulationDatasetRecorder.write(
            result=self,
            path=path,
            metadata=metadata,
            overwrite=overwrite,
        )
