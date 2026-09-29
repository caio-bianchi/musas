from pathlib import Path

import simpy

from src.airspace.airspace import Airspace
from src.airspace.restrictions import Geofence
from src.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from src.algorithms.interfaces.dynamics import DynamicsAlgorithm
from src.algorithms.interfaces.guidance import GuidanceAlgorithm
from src.algorithms.interfaces.sensor_model import SensorModel
from src.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from src.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from src.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from src.algorithms.interfaces.tracking import TrackingAlgorithm
from src.core.network import CommunicationNetwork
from src.domain.flight import FlightPlan
from src.domain.tracking import TrackPrediction, TrackingReport
from src.uav.uav import UAV, UAVStatus
from src.utm_modules.flight_plan_manager import FlightPlanManager
from src.utm_modules.strategic_deconflicition_module import (
    StrategicDeconflictionModule,
)
from src.utm_modules.tactical_deconfliction_module import (
    TacticalDeconflictionModule,
)
from src.utm_modules.tracking_module import TrackingModule

from src.simulation.result import SimulationResult


def _type_name(value) -> str | None:
    if value is None:
        return None
    value_type = type(value)
    return f"{value_type.__module__}.{value_type.__qualname__}"


class Simulation:
    """Assemble and coordinate one UTM simulation."""

    def __init__(
        self,
        airspace: Airspace,
        strategic_conflict_prediction_algorithm: ConflictPredictionAlgorithm,
        strategic_deconfliction_algorithm: StrategicDeconflictionAlgorithm,
        tracking_algorithm: TrackingAlgorithm | None = None,
        tactical_conflict_prediction_algorithm: (
            TacticalConflictPredictionAlgorithm | None
        ) = None,
        tactical_deconfliction_algorithm: (
            TacticalDeconflictionAlgorithm | None
        ) = None,
        network_delay: float = 0.0,
        flight_update_interval: float = 0.1,
        trajectory_record_interval: float = 0.2,
        report_interval: float = 0.5,
        history_window: int = 6,
        prediction_horizon: int = 10,
        prediction_step: float = 0.5,
        tracking_processing_time: float = 0.0,
        tactical_processing_time: float = 0.0,
        log_file: str | Path | None = None,
    ) -> None:
        if network_delay < 0:
            raise ValueError("network_delay cannot be negative")
        if flight_update_interval <= 0:
            raise ValueError("flight_update_interval must be greater than zero")
        if trajectory_record_interval <= 0:
            raise ValueError(
                "trajectory_record_interval must be greater than zero"
            )
        if report_interval <= 0:
            raise ValueError("report_interval must be greater than zero")

        tactical_algorithms = (
            tactical_conflict_prediction_algorithm,
            tactical_deconfliction_algorithm,
        )
        if any(algorithm is not None for algorithm in tactical_algorithms):
            if not all(algorithm is not None for algorithm in tactical_algorithms):
                raise ValueError(
                    "both tactical algorithms must be provided together"
                )
            if tracking_algorithm is None:
                raise ValueError(
                    "tracking_algorithm is required for tactical deconfliction"
                )

        self.airspace = airspace
        self.env = simpy.Environment()
        self.network = CommunicationNetwork(
            env=self.env,
            delay=network_delay,
            log_file=log_file,
        )
        self.flight_update_interval = float(flight_update_interval)
        self.trajectory_record_interval = float(
            trajectory_record_interval
        )
        self.report_interval = float(report_interval)
        self.configuration = {
            "network_delay": float(network_delay),
            "flight_update_interval": self.flight_update_interval,
            "trajectory_record_interval": self.trajectory_record_interval,
            "report_interval": self.report_interval,
            "history_window": int(history_window),
            "prediction_horizon": int(prediction_horizon),
            "prediction_step": float(prediction_step),
            "tracking_processing_time": float(tracking_processing_time),
            "tactical_processing_time": float(tactical_processing_time),
            "strategic_conflict_prediction_algorithm": _type_name(
                strategic_conflict_prediction_algorithm
            ),
            "strategic_deconfliction_algorithm": _type_name(
                strategic_deconfliction_algorithm
            ),
            "tracking_algorithm": _type_name(tracking_algorithm),
            "tactical_conflict_prediction_algorithm": _type_name(
                tactical_conflict_prediction_algorithm
            ),
            "tactical_deconfliction_algorithm": _type_name(
                tactical_deconfliction_algorithm
            ),
        }

        self.flight_plan_manager = FlightPlanManager(
            name="Flight Plan Manager",
            env=self.env,
            network=self.network,
            airspace=airspace,
        )
        self.strategic_module = StrategicDeconflictionModule(
            name="Strategic Deconfliction Module",
            env=self.env,
            network=self.network,
            conflict_prediction_algorithm=(
                strategic_conflict_prediction_algorithm
            ),
            deconfliction_algorithm=strategic_deconfliction_algorithm,
            flight_plan_manager=self.flight_plan_manager,
        )

        self.tactical_module = None
        if all(algorithm is not None for algorithm in tactical_algorithms):
            self.tactical_module = TacticalDeconflictionModule(
                name="Tactical Deconfliction Module",
                env=self.env,
                network=self.network,
                conflict_prediction_algorithm=(
                    tactical_conflict_prediction_algorithm
                ),
                deconfliction_algorithm=tactical_deconfliction_algorithm,
                flight_plan_manager=self.flight_plan_manager,
                processing_time=tactical_processing_time,
            )

        self.tracking_module = None
        if tracking_algorithm is not None:
            self.tracking_module = TrackingModule(
                name="Tracking Module",
                env=self.env,
                network=self.network,
                flight_plan_manager=self.flight_plan_manager,
                algorithm=tracking_algorithm,
                history_window=history_window,
                prediction_horizon=prediction_horizon,
                prediction_step=prediction_step,
                tactical_deconfliction_module=self.tactical_module,
                processing_time=tracking_processing_time,
            )

        self.uavs: dict[str, UAV] = {}
        self.proposed_flight_plans: dict[str, FlightPlan] = {}
        self.submission_times: dict[str, float] = {}
        self.trajectory_history: dict[
            str,
            list[tuple[float, list[float], UAVStatus]],
        ] = {}
        self.velocity_history: dict[
            str,
            list[tuple[float, list[float]]],
        ] = {}
        self.airspace_history: list[
            tuple[float, tuple[Geofence, ...]]
        ] = []
        self.tracking_prediction_history: list[
            tuple[float, dict[str, TrackPrediction]]
        ] = []
        self.tracking_report_history: list[
            tuple[float, dict[str, tuple[TrackingReport, ...]]]
        ] = []
        self._scheduled_geofences: list[
            tuple[Geofence, float, float | None]
        ] = []
        self._started = False

    def add_uav(
        self,
        name: str,
        flight_plan: FlightPlan,
        dynamics_algorithm: DynamicsAlgorithm,
        guidance_algorithm: GuidanceAlgorithm,
        sensor_model: SensorModel | None = None,
        submission_time: float = 0.0,
        waypoint_acceptance_radius: float = 1.0,
    ) -> UAV:
        """Create and register a UAV and its proposed flight plan."""
        if self._started:
            raise RuntimeError("UAVs cannot be added after the simulation starts")
        if name in self.uavs:
            raise ValueError(f"UAV {name!r} is already registered")
        if not flight_plan.waypoint_list:
            raise ValueError("flight_plan must contain at least one waypoint")
        if submission_time < 0:
            raise ValueError("submission_time cannot be negative")
        if waypoint_acceptance_radius < 0:
            raise ValueError(
                "waypoint_acceptance_radius cannot be negative"
            )

        uav = UAV(
            name=name,
            env=self.env,
            network=self.network,
            position=flight_plan.waypoint_list[0].position(),
            dynamics_algorithm=dynamics_algorithm,
            guidance_algorithm=guidance_algorithm,
            strategic_deconfliction_module=self.strategic_module,
            waypoint_acceptance_radius=waypoint_acceptance_radius,
            environmental_velocity_provider=self.airspace.wind_at,
            sensor_model=sensor_model,
        )
        self.uavs[name] = uav
        self.proposed_flight_plans[name] = flight_plan
        self.submission_times[name] = float(submission_time)
        self.trajectory_history[name] = []
        self.velocity_history[name] = []
        return uav

    def schedule_geofence(
        self,
        geofence: Geofence,
        activation_time: float,
        removal_time: float | None = None,
    ) -> None:
        """Add and optionally remove a geofence at simulation times."""
        if self._started:
            raise RuntimeError(
                "geofences must be scheduled before the simulation starts"
            )
        if activation_time < 0:
            raise ValueError("activation_time cannot be negative")
        if removal_time is not None and removal_time < activation_time:
            raise ValueError(
                "removal_time cannot be earlier than activation_time"
            )
        existing_ids = {
            scheduled.geofence_id
            for scheduled, _, _ in self._scheduled_geofences
        }
        existing_ids.update(
            existing.geofence_id
            for existing in self.airspace.geofences
        )
        if geofence.geofence_id in existing_ids:
            raise ValueError(
                f"Geofence {geofence.geofence_id!r} is already configured"
            )
        self._scheduled_geofences.append(
            (
                geofence,
                float(activation_time),
                (
                    None
                    if removal_time is None
                    else float(removal_time)
                ),
            )
        )

    def run(self, until: float) -> SimulationResult:
        """Run or continue the simulation up to an absolute time."""
        if not self.uavs:
            raise RuntimeError("add at least one UAV before running")
        if until <= self.env.now:
            raise ValueError("until must be greater than the current time")
        if not self._started:
            self._start_processes()
            self._started = True

        self.env.run(until=until)
        return self.result()

    def result(self) -> SimulationResult:
        """Return a snapshot of the state and histories collected so far."""
        flight_plan_history = {
            name: tuple(self.flight_plan_manager.get_history(name))
            for name in self.uavs
        }
        return SimulationResult(
            time=float(self.env.now),
            airspace=self.airspace,
            uavs=self.uavs.copy(),
            proposed_flight_plans=self.proposed_flight_plans.copy(),
            flight_plan_history=flight_plan_history,
            trajectory_history={
                name: list(history)
                for name, history in self.trajectory_history.items()
            },
            velocity_history={
                name: list(history)
                for name, history in self.velocity_history.items()
            },
            airspace_history=list(self.airspace_history),
            tracking_prediction_history=list(
                self.tracking_prediction_history
            ),
            tracking_report_history=list(self.tracking_report_history),
            flight_plan_manager=self.flight_plan_manager,
            tracking_module=self.tracking_module,
            tactical_module=self.tactical_module,
            event_history=self.network.get_event_history(),
            configuration=dict(self.configuration),
        )

    def _start_processes(self) -> None:
        self.env.process(self.strategic_module.run())
        if self.tracking_module is not None:
            self.env.process(self.tracking_module.run())
        if self.tactical_module is not None:
            self.env.process(self.tactical_module.run())

        for name, uav in self.uavs.items():
            self.env.process(uav.receive_messages())
            self.env.process(
                uav.fly(update_interval=self.flight_update_interval)
            )
            if self.tracking_module is not None:
                self.env.process(
                    uav.send_periodic_reports(
                        tracking_module=self.tracking_module,
                        update_interval=self.report_interval,
                    )
                )
            self.env.process(
                self._submit_flight_plan(
                    uav=uav,
                    flight_plan=self.proposed_flight_plans[name],
                    submission_time=self.submission_times[name],
                )
            )

        self.env.process(self._record_trajectories())
        self.env.process(self._record_airspace())
        if self.tracking_module is not None:
            self.env.process(self._record_tracking())

        for geofence, activation_time, removal_time in (
            self._scheduled_geofences
        ):
            self.env.process(
                self._manage_geofence(
                    geofence,
                    activation_time,
                    removal_time,
                )
            )

    def _submit_flight_plan(
        self,
        uav: UAV,
        flight_plan: FlightPlan,
        submission_time: float,
    ):
        if submission_time > self.env.now:
            yield self.env.timeout(submission_time - self.env.now)
        uav.send_flight_plan(flight_plan)

    def _manage_geofence(
        self,
        geofence: Geofence,
        activation_time: float,
        removal_time: float | None,
    ):
        if activation_time > self.env.now:
            yield self.env.timeout(activation_time - self.env.now)
        self.airspace.add_geofence(geofence)
        self.network.log_event(
            component="Airspace",
            event="GEOFENCE_ACTIVATED",
            geofence=geofence.geofence_id,
        )

        if removal_time is not None:
            if removal_time > self.env.now:
                yield self.env.timeout(removal_time - self.env.now)
            self.airspace.remove_geofence(geofence.geofence_id)
            self.network.log_event(
                component="Airspace",
                event="GEOFENCE_REMOVED",
                geofence=geofence.geofence_id,
            )

    def _record_trajectories(self):
        while True:
            for name, uav in self.uavs.items():
                self.trajectory_history[name].append(
                    (
                        float(self.env.now),
                        uav.dynamics.position.copy(),
                        uav.status,
                    )
                )
                self.velocity_history[name].append(
                    (
                        float(self.env.now),
                        uav.dynamics.velocity.copy(),
                    )
                )
            yield self.env.timeout(self.trajectory_record_interval)

    def _record_airspace(self):
        while True:
            self.airspace_history.append(
                (
                    float(self.env.now),
                    tuple(self.airspace.geofences),
                )
            )
            yield self.env.timeout(self.trajectory_record_interval)

    def _record_tracking(self):
        while True:
            self.tracking_prediction_history.append(
                (
                    float(self.env.now),
                    self.tracking_module.get_all_predictions(),
                )
            )
            self.tracking_report_history.append(
                (
                    float(self.env.now),
                    self.tracking_module.get_all_report_histories(),
                )
            )
            yield self.env.timeout(self.trajectory_record_interval)
