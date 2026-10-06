from dataclasses import dataclass
from pathlib import Path
from typing import Any

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.airspace.restrictions import Geofence
from utm_simulator.core.simulation_event import SimulationEvent
from utm_simulator.domain.flight import FlightPlan
from utm_simulator.domain.tracking import TrackPrediction, TrackingReport
from utm_simulator.uav.uav import UAV, UAVStatus
from utm_simulator.utm.flight_plan_manager import FlightPlanManager
from utm_simulator.utm.tactical_deconfliction_module import (
    TacticalDeconflictionModule,
)
from utm_simulator.utm.tracking_module import TrackingModule


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
        from utm_simulator.simulation.dataset import SimulationDatasetWriter

        return SimulationDatasetWriter.write(
            result=self,
            path=path,
            metadata=metadata,
            overwrite=overwrite,
        )
