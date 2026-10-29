from collections.abc import Collection
from itertools import combinations
from math import dist
from pathlib import Path
from typing import Any

import simpy

from musas.airspace.airspace import Airspace
from musas.airspace.restrictions import Geofence
from musas.airspace.wind import WindZone
from musas.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.dynamics import DynamicsAlgorithm
from musas.algorithms.interfaces.guidance import GuidanceAlgorithm
from musas.algorithms.interfaces.processing_time import (
    ProcessingTimeModel,
)
from musas.algorithms.interfaces.sensor_model import SensorModel
from musas.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from musas.algorithms.interfaces.tracking import TrackingAlgorithm
from musas.algorithms.builtins.constant_time_processing import (
    ConstantTimeProcessing,
)
from musas.algorithms.builtins.dummy_strategic_deconfliction import (
    DummyStrategicDeconfliction,
)
from musas.core.network import CommunicationNetwork
from musas.domain.flight import FlightPlan
from musas.domain.message_types import MessageType
from musas.domain.tracking import TrackPrediction, TrackingReport
from musas.uav.uav import UAV, UAVStatus
from musas.utm.flight_plan_manager import FlightPlanManager
from musas.utm.strategic_deconfliction import (
    StrategicDeconflictionModule,
)
from musas.utm.tactical_deconfliction_module import (
    TacticalDeconflictionModule,
)
from musas.utm.tracking_module import TrackingModule

from musas.simulation.result import (
    AirspaceSnapshot,
    SimulationResult,
    UAVSnapshot,
    UAVStateSample,
)
from musas.simulation.dataset import SimulationDatasetRecorder


DEFAULT_MESSAGE_TYPES_TO_LOG = frozenset(
    message_type
    for message_type in MessageType
    if message_type
    not in {
        MessageType.PERIODIC_REPORT,
        MessageType.TRACK_UPDATE,
    }
)


def _type_name(value) -> str | None:
    if value is None:
        return None
    value_type = type(value)
    return f"{value_type.__module__}.{value_type.__qualname__}"


def _require_algorithm(
    value: object | None,
    expected_type: type,
    parameter_name: str,
) -> None:
    """Require injected algorithms to explicitly inherit their interface."""
    if value is not None and not isinstance(value, expected_type):
        raise TypeError(
            f"{parameter_name} must inherit from "
            f"{expected_type.__name__}"
        )


class Simulation:
    """Assemble and coordinate one UTM simulation."""

    def __init__(
        self,
        airspace: Airspace,
        strategic_conflict_prediction_algorithm: (
            ConflictPredictionAlgorithm | None
        ) = None,
        strategic_deconfliction_algorithm: (
            StrategicDeconflictionAlgorithm | None
        ) = None,
        tracking_algorithm: TrackingAlgorithm | None = None,
        tactical_conflict_prediction_algorithm: (
            TacticalConflictPredictionAlgorithm | None
        ) = None,
        tactical_deconfliction_algorithm: (
            TacticalDeconflictionAlgorithm | None
        ) = None,
        network_delay: float = 0.0,
        message_types_to_log: Collection[MessageType] | None = (
            DEFAULT_MESSAGE_TYPES_TO_LOG
        ),
        flight_update_interval: float = 0.1,
        trajectory_record_interval: float = 0.2,
        report_interval: float = 0.5,
        history_window: int = 6,
        prediction_horizon: int = 10,
        prediction_step: float = 0.5,
        tactical_assessment_interval: float = 1.0,
        collision_distance: float = 1.0,
        strategic_processing_time: ProcessingTimeModel = (
            ConstantTimeProcessing(0)
        ),
        tracking_processing_time: ProcessingTimeModel = (
            ConstantTimeProcessing(0)
        ),
        tactical_processing_time: ProcessingTimeModel = (
            ConstantTimeProcessing(0)
        ),
        dataset_path: str | Path | None = None,
        dataset_flush_interval: float = 10.0,
        dataset_metadata: dict[str, Any] | None = None,
        overwrite_dataset: bool = False,
        keep_history_in_memory: bool = True,
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
        if tactical_assessment_interval <= 0:
            raise ValueError(
                "tactical_assessment_interval must be greater than zero"
            )
        if collision_distance <= 0:
            raise ValueError("collision_distance must be greater than zero")
        if dataset_flush_interval <= 0:
            raise ValueError(
                "dataset_flush_interval must be greater than zero"
            )
        if dataset_metadata is not None and dataset_path is None:
            raise ValueError(
                "dataset_path is required when dataset_metadata is provided"
            )
        if not keep_history_in_memory and dataset_path is None:
            raise ValueError(
                "dataset_path is required when keep_history_in_memory is False"
            )

        strategic_algorithms = (
            strategic_conflict_prediction_algorithm,
            strategic_deconfliction_algorithm,
        )
        if not any(
            algorithm is not None for algorithm in strategic_algorithms
        ):
            dummy_strategic_deconfliction = (
                DummyStrategicDeconfliction()
            )
            strategic_conflict_prediction_algorithm = (
                dummy_strategic_deconfliction
            )
            strategic_deconfliction_algorithm = (
                dummy_strategic_deconfliction
            )
            strategic_deconfliction_enabled = False
        elif not all(
            algorithm is not None for algorithm in strategic_algorithms
        ):
            raise ValueError(
                "both strategic algorithms must be provided together, "
                "or both must be None"
            )
        else:
            strategic_deconfliction_enabled = True

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

        _require_algorithm(
            strategic_conflict_prediction_algorithm,
            ConflictPredictionAlgorithm,
            "strategic_conflict_prediction_algorithm",
        )
        _require_algorithm(
            strategic_deconfliction_algorithm,
            StrategicDeconflictionAlgorithm,
            "strategic_deconfliction_algorithm",
        )
        _require_algorithm(
            tracking_algorithm,
            TrackingAlgorithm,
            "tracking_algorithm",
        )
        _require_algorithm(
            tactical_conflict_prediction_algorithm,
            TacticalConflictPredictionAlgorithm,
            "tactical_conflict_prediction_algorithm",
        )
        _require_algorithm(
            tactical_deconfliction_algorithm,
            TacticalDeconflictionAlgorithm,
            "tactical_deconfliction_algorithm",
        )

        self.airspace = airspace
        self.env = simpy.Environment()
        self.network = CommunicationNetwork(
            env=self.env,
            delay=network_delay,
            message_types_to_log=message_types_to_log,
        )
        self.flight_update_interval = float(flight_update_interval)
        self.trajectory_record_interval = float(
            trajectory_record_interval
        )
        self.report_interval = float(report_interval)
        self.tactical_assessment_interval = float(
            tactical_assessment_interval
        )
        self.collision_distance = float(collision_distance)
        self.dataset_path = (
            None
            if dataset_path is None
            else Path(dataset_path).expanduser().resolve()
        )
        self.dataset_flush_interval = float(dataset_flush_interval)
        self.dataset_metadata = dict(dataset_metadata or {})
        self.overwrite_dataset = overwrite_dataset
        self.keep_history_in_memory = bool(keep_history_in_memory)
        self.configuration = {
            "network_delay": float(network_delay),
            "message_types_to_log": (
                None
                if self.network.message_types_to_log is None
                else sorted(
                    message_type.name
                    for message_type in self.network.message_types_to_log
                )
            ),
            "flight_update_interval": self.flight_update_interval,
            "trajectory_record_interval": self.trajectory_record_interval,
            "report_interval": self.report_interval,
            "history_window": int(history_window),
            "prediction_horizon": int(prediction_horizon),
            "prediction_step": float(prediction_step),
            "tactical_assessment_interval": (
                self.tactical_assessment_interval
            ),
            "collision_distance": self.collision_distance,
            "strategic_deconfliction_enabled": (
                strategic_deconfliction_enabled
            ),
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
            "incremental_dataset_enabled": self.dataset_path is not None,
            "dataset_flush_interval": self.dataset_flush_interval,
            "keep_history_in_memory": self.keep_history_in_memory,
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
            processing_time=strategic_processing_time,
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
                assessment_interval=self.tactical_assessment_interval,
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
                report_observer=self._record_tracking_report,
                prediction_observer=self._record_tracking_prediction,
            )

        self.uavs: dict[str, UAV] = {}
        self.proposed_flight_plans: dict[str, FlightPlan] = {}
        self.submission_times: dict[str, float] = {}
        self.state_history: dict[str, list[UAVStateSample]] = {}
        self.airspace_history: list[AirspaceSnapshot] = []
        self.tracking_predictions: list[TrackPrediction] = []
        self.tracking_reports: list[TrackingReport] = []
        self._scheduled_geofences: list[
            tuple[Geofence, float, float | None]
        ] = []
        self._scheduled_wind_zones: list[
            tuple[WindZone, float, float | None]
        ] = []
        self._started = False
        self._closed = False
        self._dataset_writer: (
            SimulationDatasetRecorder | None
        ) = None

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
        _require_algorithm(
            dynamics_algorithm,
            DynamicsAlgorithm,
            "dynamics_algorithm",
        )
        _require_algorithm(
            guidance_algorithm,
            GuidanceAlgorithm,
            "guidance_algorithm",
        )
        _require_algorithm(
            sensor_model,
            SensorModel,
            "sensor_model",
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
        self.state_history[name] = []
        return uav

    def add_geofence(self, geofence: Geofence) -> None:
        """Add a geofence and record the dynamic airspace change."""
        self.airspace.add_geofence(geofence)
        self._record_airspace_snapshot()
        self.network.record_event(
            component="Airspace",
            event="GEOFENCE_ACTIVATED",
            geofence=geofence.geofence_id,
        )

    def remove_geofence(self, geofence_id: str) -> Geofence:
        """Remove a geofence and record the dynamic airspace change."""
        removed = self.airspace.remove_geofence(geofence_id)
        self._record_airspace_snapshot()
        self.network.record_event(
            component="Airspace",
            event="GEOFENCE_REMOVED",
            geofence=geofence_id,
        )
        return removed

    def add_wind_zone(self, wind_zone: WindZone) -> None:
        """Add a wind zone and record the dynamic airspace change."""
        self.airspace.add_wind_zone(wind_zone)
        self._record_airspace_snapshot()
        self.network.record_event(
            component="Airspace",
            event="WIND_ZONE_ACTIVATED",
            wind_zone=wind_zone.wind_id,
        )

    def remove_wind_zone(self, wind_id: str) -> WindZone:
        """Remove a wind zone and record the dynamic airspace change."""
        removed = self.airspace.remove_wind_zone(wind_id)
        self._record_airspace_snapshot()
        self.network.record_event(
            component="Airspace",
            event="WIND_ZONE_REMOVED",
            wind_zone=wind_id,
        )
        return removed

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
        existing_ids.update(self.airspace.geofences)
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

    def schedule_wind_zone(
        self,
        wind_zone: WindZone,
        activation_time: float,
        removal_time: float | None = None,
    ) -> None:
        """Add and optionally remove a wind zone at simulation times."""
        if self._started:
            raise RuntimeError(
                "wind zones must be scheduled before the simulation starts"
            )
        if activation_time < 0:
            raise ValueError("activation_time cannot be negative")
        if removal_time is not None and removal_time < activation_time:
            raise ValueError(
                "removal_time cannot be earlier than activation_time"
            )
        existing_ids = {
            scheduled.wind_id
            for scheduled, _, _ in self._scheduled_wind_zones
        }
        existing_ids.update(self.airspace.wind_zones)
        if wind_zone.wind_id in existing_ids:
            raise ValueError(
                f"Wind zone {wind_zone.wind_id!r} is already configured"
            )
        self._scheduled_wind_zones.append(
            (
                wind_zone,
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
        if self._closed:
            raise RuntimeError("simulation is closed")
        if not self.uavs:
            raise RuntimeError("add at least one UAV before running")
        if until <= self.env.now:
            raise ValueError("until must be greater than the current time")
        if not self._started:
            self._start_dataset_recording()
            self._start_processes()
            self._started = True

        try:
            self.env.run(until=until)
        except Exception:
            if self._dataset_writer is not None:
                try:
                    self._flush_dataset("FAILED")
                except Exception:
                    # Preserve the simulation error that triggered this path.
                    pass
                finally:
                    self._dataset_writer.close()
            self._closed = True
            raise

        result = self.result()
        self._flush_dataset(self._dataset_run_status())
        return result

    def close(self) -> None:
        """Flush and close an incremental dataset, if configured."""
        if self._closed:
            return
        try:
            if self._dataset_writer is not None:
                try:
                    self._flush_dataset(
                        "COMPLETED"
                        if self._all_flights_completed()
                        else "STOPPED"
                    )
                finally:
                    self._dataset_writer.close()
        finally:
            self._closed = True

    def result(self) -> SimulationResult:
        """Return a snapshot of the state and histories collected so far."""
        return self._build_result(copy_histories=True)

    def _build_result(self, copy_histories: bool) -> SimulationResult:
        """Build the single data view used by callers and the recorder."""
        flight_plan_history = {
            name: tuple(self.flight_plan_manager.get_history(name))
            for name in self.uavs
        }
        state_history = (
            {
                name: list(history)
                for name, history in self.state_history.items()
            }
            if copy_histories
            else self.state_history
        )
        uav_snapshots = {
            name: UAVSnapshot(
                name=name,
                status=uav.status,
                position=tuple(uav.dynamics.position),
                velocity=tuple(uav.dynamics.velocity),
                dynamics_type=_type_name(uav.dynamics.algorithm) or "",
                guidance_type=_type_name(uav.guidance.algorithm) or "",
                sensor_type=_type_name(uav.sensor_model) or "",
            )
            for name, uav in self.uavs.items()
        }
        return SimulationResult(
            time=float(self.env.now),
            airspace=Airspace(
                obstacles=dict(self.airspace.obstacles),
                geofences=dict(self.airspace.geofences),
                wind_zones=dict(self.airspace.wind_zones),
            ),
            uavs=uav_snapshots,
            proposed_flight_plans=self.proposed_flight_plans.copy(),
            flight_plan_history=flight_plan_history,
            state_history=state_history,
            airspace_history=(
                list(self.airspace_history)
                if copy_histories
                else self.airspace_history
            ),
            tracking_reports=(
                list(self.tracking_reports)
                if copy_histories
                else self.tracking_reports
            ),
            tracking_predictions=(
                list(self.tracking_predictions)
                if copy_histories
                else self.tracking_predictions
            ),
            event_history=(
                self.network.get_event_history()
                if copy_histories
                else self.network.event_history
            ),
            configuration=dict(self.configuration),
            dataset_path=self.dataset_path,
            history_complete=self.keep_history_in_memory,
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
                        report_interval=self.report_interval,
                    )
                )
            self.env.process(
                self._submit_flight_plan(
                    uav=uav,
                    flight_plan=self.proposed_flight_plans[name],
                    submission_time=self.submission_times[name],
                )
            )

        self._record_airspace_snapshot()
        self.env.process(self._monitor_collisions())
        self.env.process(self._record_states())
        if self._dataset_writer is not None:
            self.env.process(self._record_dataset())

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
        for wind_zone, activation_time, removal_time in (
            self._scheduled_wind_zones
        ):
            self.env.process(
                self._manage_wind_zone(
                    wind_zone,
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
        self.add_geofence(geofence)

        if removal_time is not None:
            if removal_time > self.env.now:
                yield self.env.timeout(removal_time - self.env.now)
            self.remove_geofence(geofence.geofence_id)

    def _manage_wind_zone(
        self,
        wind_zone: WindZone,
        activation_time: float,
        removal_time: float | None,
    ):
        if activation_time > self.env.now:
            yield self.env.timeout(activation_time - self.env.now)
        self.add_wind_zone(wind_zone)

        if removal_time is not None:
            if removal_time > self.env.now:
                yield self.env.timeout(removal_time - self.env.now)
            self.remove_wind_zone(wind_zone.wind_id)

    def _record_states(self):
        while True:
            for name, uav in self.uavs.items():
                self.state_history[name].append(
                    UAVStateSample(
                        timestamp=float(self.env.now),
                        position=tuple(uav.dynamics.position),
                        velocity=tuple(uav.dynamics.velocity),
                        status=uav.status,
                    )
                )
            yield self.env.timeout(self.trajectory_record_interval)

    def _monitor_collisions(self):
        """Stop UAVs that hit fixed obstacles or another active UAV."""
        while True:
            active_uavs = {
                name: uav
                for name, uav in self.uavs.items()
                if uav.status == UAVStatus.ACTIVE
            }

            obstacle_hits = {}
            for name, uav in active_uavs.items():
                obstacle_hits[name] = next(
                    (
                        obstacle
                        for obstacle in self.airspace.obstacles.values()
                        if obstacle.contains(uav.dynamics.position)
                    ),
                    None,
                )

            uav_hits: list[tuple[str, str]] = []
            for (first_name, first), (second_name, second) in combinations(
                active_uavs.items(),
                2,
            ):
                if (
                    dist(first.dynamics.position, second.dynamics.position)
                    <= self.collision_distance
                ):
                    uav_hits.append((first_name, second_name))

            colliding_with: dict[str, list[str]] = {}
            for first_name, second_name in uav_hits:
                colliding_with.setdefault(first_name, []).append(second_name)
                colliding_with.setdefault(second_name, []).append(first_name)
                self.network.record_event(
                    component="Simulation",
                    event="UAV_COLLISION",
                    uavs=[first_name, second_name],
                    collision_distance=self.collision_distance,
                    position_a=list(
                        active_uavs[first_name].dynamics.position
                    ),
                    position_b=list(
                        active_uavs[second_name].dynamics.position
                    ),
                )

            affected_names = set(colliding_with) | {
                name
                for name, obstacle in obstacle_hits.items()
                if obstacle is not None
            }
            for name in sorted(affected_names):
                uav = active_uavs[name]
                obstacle = obstacle_hits.get(name)
                other_uavs = sorted(colliding_with.get(name, ()))
                if obstacle is not None:
                    reason = "OBSTACLE_COLLISION"
                    obstacle_id = obstacle.obstacle_id
                    other_uav = other_uavs[0] if other_uavs else None
                    self.network.record_event(
                        component="Simulation",
                        event="OBSTACLE_COLLISION",
                        uav=name,
                        obstacle_id=obstacle_id,
                        position=list(uav.dynamics.position),
                    )
                else:
                    reason = "UAV_COLLISION"
                    obstacle_id = None
                    other_uav = other_uavs[0]

                if uav.crash(
                    reason=reason,
                    other_uav=other_uav,
                    obstacle_id=obstacle_id,
                ):
                    self.flight_plan_manager.terminate(name, reason)

            yield self.env.timeout(self.flight_update_interval)

    def _record_airspace_snapshot(self) -> None:
        snapshot = AirspaceSnapshot(
            timestamp=float(self.env.now),
            geofences=tuple(self.airspace.geofences.values()),
            wind_zones=tuple(self.airspace.wind_zones.values()),
        )
        if (
            self.airspace_history
            and self.airspace_history[-1].timestamp == snapshot.timestamp
        ):
            self.airspace_history[-1] = snapshot
        else:
            self.airspace_history.append(snapshot)

    def _record_tracking_report(self, report: TrackingReport) -> None:
        self.tracking_reports.append(report)

    def _record_tracking_prediction(self, prediction: TrackPrediction) -> None:
        self.tracking_predictions.append(prediction)

    def _start_dataset_recording(self) -> None:
        if self.dataset_path is None:
            return
        self._dataset_writer = SimulationDatasetRecorder(
            path=self.dataset_path,
            configuration=dict(self.configuration),
            metadata=self.dataset_metadata,
            overwrite=self.overwrite_dataset,
        )
        self._flush_dataset()

    def _record_dataset(self):
        while True:
            yield self.env.timeout(self.dataset_flush_interval)
            self._flush_dataset(self._dataset_run_status())

    def _flush_dataset(self, run_status: str = "RUNNING") -> None:
        if self._dataset_writer is None:
            return
        self._dataset_writer.flush(
            self._build_result(copy_histories=False),
            run_status=run_status,
        )
        if not self.keep_history_in_memory:
            self._clear_persisted_history()

    def _clear_persisted_history(self) -> None:
        for history in self.state_history.values():
            history.clear()
        self.airspace_history.clear()
        self.tracking_reports.clear()
        self.tracking_predictions.clear()
        self.network.event_history.clear()
        if self._dataset_writer is not None:
            self._dataset_writer.reset_transient_offsets()

    def _all_flights_completed(self) -> bool:
        return bool(self.uavs) and all(
            uav.status in {UAVStatus.COMPLETED, UAVStatus.COLLIDED}
            for uav in self.uavs.values()
        )

    def _dataset_run_status(self) -> str:
        return "COMPLETED" if self._all_flights_completed() else "RUNNING"
