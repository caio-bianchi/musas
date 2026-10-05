from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.airspace.airspace import Airspace
from src.airspace.restrictions import Geofence
from src.core.simulation_event import SimulationEvent
from src.domain.flight import FlightPlan
from src.domain.tracking import TrackPrediction, TrackingReport
from src.uav.uav import UAV, UAVStatus
from src.utm_modules.flight_plan_manager import FlightPlanManager
from src.utm_modules.tactical_deconfliction_module import (
    TacticalDeconflictionModule,
)
from src.utm_modules.tracking_module import TrackingModule


@dataclass
class SimulationResult:
    """State and histories collected up to the current simulation time."""
    time: float
    airspace: Airspace
    uavs: dict[str, UAV]
    proposed_flight_plans: dict[str, FlightPlan]
    flight_plan_history: dict[str, tuple[FlightPlan, ...]]
    trajectory_history: dict[
        str,
        list[tuple[float, list[float], UAVStatus]],
    ]
    velocity_history: dict[
        str,
        list[tuple[float, list[float]]],
    ]
    airspace_history: list[tuple[float, tuple[Geofence, ...]]]
    tracking_prediction_history: list[
        tuple[float, dict[str, TrackPrediction]]
    ]
    tracking_report_history: list[
        tuple[float, dict[str, tuple[TrackingReport, ...]]]
    ]
    flight_plan_manager: FlightPlanManager
    tracking_module: TrackingModule | None
    tactical_module: TacticalDeconflictionModule | None
    event_history: list[SimulationEvent]
    configuration: dict[str, Any]

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
        from src.simulation.dataset import SimulationDatasetWriter

        return SimulationDatasetWriter.write(
            result=self,
            path=path,
            metadata=metadata,
            overwrite=overwrite,
        )
