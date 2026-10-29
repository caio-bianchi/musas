import simpy

from musas.algorithms.builtins.constant_time_processing import (
    ConstantTimeProcessing,
)
from musas.algorithms.builtins.conflicts import (
    TacticalAirspaceConflict,
    TacticalTrafficConflict,
)
from musas.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from musas.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from musas.algorithms.interfaces.processing_time import (
    ProcessingTimeModel,
)
from musas.core.message import Message
from musas.core.network import CommunicationNetwork
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.message_types import MessageType
from musas.domain.tracking import TrackPrediction
from musas.utm.flight_plan_manager import FlightPlanManager


class TacticalDeconflictionModule:
    """Receive tracks and coordinate tactical flight-plan updates."""

    def __init__(
        self,
        name: str,
        env: simpy.Environment,
        network: CommunicationNetwork,
        conflict_prediction_algorithm: TacticalConflictPredictionAlgorithm,
        deconfliction_algorithm: TacticalDeconflictionAlgorithm,
        flight_plan_manager: FlightPlanManager,
        assessment_interval: float = 1.0,
        processing_time: ProcessingTimeModel = ConstantTimeProcessing(0),
    ) -> None:
        if assessment_interval <= 0:
            raise ValueError("assessment_interval must be greater than zero")
        self.name = name
        self.env = env
        self.network = network
        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.assessment_interval = float(assessment_interval)
        self.processing_time = processing_time

        self.inbox = simpy.Store(env)
        self._tracks: dict[str, TrackPrediction] = {}
        self._completed_uavs: set[str] = set()
        self._latest_conflicts: list[Conflict] = []
        self._issued_updates: dict[str, FlightPlan] = {}
        self._issued_update_history: dict[str, list[FlightPlan]] = {}
        self._assessment_count = 0

        network.register(self)

    def run(self):
        """Receive tracks continuously and assess their snapshots periodically."""
        self.env.process(self.receive_track_updates())
        while True:
            yield self.env.timeout(self.assessment_interval)
            yield self.env.process(self.assess_conflicts())

    def receive_track_updates(self):
        """Keep the latest track for each active UAV without assessing it."""
        while True:
            message = yield self.inbox.get()
            if message.message_type == MessageType.TRACK_UPDATE:
                self.process_track_update(message)

    def _processing_context(
        self,
        operation: str,
        tracks: dict[str, TrackPrediction],
        conflict_count: int = 0,
    ) -> dict[str, object]:
        return {
            "module": self.name,
            "operation": operation,
            "queue_length": len(self.inbox.items),
            "assessment_interval": self.assessment_interval,
            "track_count": len(tracks),
            "flight_plan_count": len(self.flight_plan_manager.get_all()),
            "predicted_state_count": sum(
                len(track.predicted_states)
                for track in tracks.values()
            ),
            "conflict_count": conflict_count,
        }

    def process_track_update(self, message: Message) -> None:
        """Store a track if it is not stale or from a completed UAV."""
        updated_track = message.payload["track_prediction"]
        if updated_track.uav_name in self._completed_uavs:
            return
        current_track = self._tracks.get(updated_track.uav_name)

        if (
            current_track is not None
            and updated_track.generated_at < current_track.generated_at
        ):
            return

        self._tracks[updated_track.uav_name] = updated_track

    def assess_conflicts(self):
        """Assess one coherent snapshot and apply one batch of updates."""
        tracks = self.get_all_tracks()
        if not tracks:
            return None
        self._assessment_count += 1
        assessment_id = self._assessment_count

        airspace = self.flight_plan_manager.get_airspace_snapshot()
        flight_plans = self.flight_plan_manager.get_all()
        prediction_delay = self.processing_time.calculate(
            **self._processing_context(
                "conflict_prediction",
                tracks,
            )
        )
        if prediction_delay < 0:
            raise ValueError("processing time cannot be negative")
        if prediction_delay:
            yield self.env.timeout(prediction_delay)

        conflicts = self.conflict_prediction_algorithm.predict(
            tracks=tracks,
            airspace=airspace,
        )
        self._latest_conflicts = list(conflicts)

        if not conflicts:
            return None

        conflict_uav_groups = [
            list(uav_names)
            for conflict in conflicts
            if (uav_names := self._conflict_uav_names(conflict))
        ]
        self.network.record_event(
            component=self.name,
            event="TACTICAL_CONFLICTS_DETECTED",
            summary=(
                f"Assessment {assessment_id} observed {len(conflicts)} "
                "predicted tactical conflict(s)"
            ),
            assessment_id=assessment_id,
            observation_scope="single_assessment_snapshot",
            conflict_count=len(conflicts),
            track_count=len(tracks),
            track_generated_at={
                uav_name: track.generated_at
                for uav_name, track in sorted(tracks.items())
            },
            conflicting_uavs=sorted({
                uav_name
                for group in conflict_uav_groups
                for uav_name in group
            }),
            conflict_uav_groups=conflict_uav_groups,
            conflicts=[
                self._conflict_details(conflict)
                for conflict in conflicts
            ],
        )

        deconfliction_delay = self.processing_time.calculate(
            **self._processing_context(
                "deconfliction",
                tracks,
                conflict_count=len(conflicts),
            )
        )
        if deconfliction_delay < 0:
            raise ValueError("processing time cannot be negative")
        if deconfliction_delay:
            yield self.env.timeout(deconfliction_delay)

        updates = self.deconfliction_algorithm.deconflict(
            conflicts=conflicts,
            tracks=tracks,
            flight_plans=flight_plans,
            airspace=airspace,
        )
        if updates:
            self.network.record_event(
                component=self.name,
                event="TACTICAL_UPDATES_ISSUED",
                summary=(
                    f"Assessment {assessment_id} issued "
                    f"{len(updates)} tactical flight-plan update(s)"
                ),
                assessment_id=assessment_id,
                update_count=len(updates),
                uavs=[update.uav_name for update in updates],
                updates=[
                    {
                        "uav": update.uav_name,
                        "waypoint_count": len(
                            update.flight_plan.waypoint_list
                        ),
                        "valid_from": (
                            update.flight_plan.waypoint_list[0].t
                        ),
                        "planned_arrival": (
                            update.flight_plan.waypoint_list[-1].t
                        ),
                    }
                    for update in updates
                ],
            )
            for update in updates:
                self._issued_updates[update.uav_name] = update.flight_plan
                self._issued_update_history.setdefault(
                    update.uav_name,
                    [],
                ).append(
                    update.flight_plan
                )
            self.flight_plan_manager.launch_updates(updates)
        return updates

    @staticmethod
    def _conflict_uav_names(conflict: Conflict) -> tuple[str, ...]:
        """Extract UAV names from built-in and compatible conflict types."""
        grouped_names = getattr(conflict, "uav_names", None)
        if grouped_names is not None:
            if isinstance(grouped_names, str):
                return (grouped_names,)
            return tuple(sorted({
                name
                for name in grouped_names
                if isinstance(name, str)
            }))

        names = {
            name
            for attribute in ("uav_name", "other_uav")
            if isinstance(name := getattr(conflict, attribute, None), str)
        }
        return tuple(sorted(names))

    @staticmethod
    def _conflict_details(conflict: Conflict) -> dict[str, object]:
        """Return a readable, JSON-compatible tactical conflict summary."""
        if isinstance(conflict, TacticalTrafficConflict):
            return {
                "conflict_type": "UAV_TO_UAV",
                "uavs": [conflict.uav_name, conflict.other_uav],
                "predicted_at": conflict.timestamp,
                "predicted_separation": conflict.separation_distance,
                "positions": {
                    conflict.uav_name: list(conflict.position),
                    conflict.other_uav: list(conflict.other_position),
                },
            }

        if isinstance(conflict, TacticalAirspaceConflict):
            restriction = conflict.restriction
            restriction_id = getattr(
                restriction,
                "obstacle_id",
                getattr(restriction, "geofence_id", None),
            )
            return {
                "conflict_type": (
                    "UAV_TO_OBSTACLE"
                    if hasattr(restriction, "obstacle_id")
                    else "UAV_TO_GEOFENCE"
                ),
                "uavs": [conflict.uav_name],
                "predicted_at": conflict.timestamp,
                "predicted_position": list(conflict.position),
                "restriction_id": restriction_id,
                "restriction_name": getattr(restriction, "name", None),
            }

        return {
            "conflict_type": type(conflict).__name__,
            "uavs": list(
                TacticalDeconflictionModule._conflict_uav_names(conflict)
            ),
            "predicted_at": getattr(conflict, "timestamp", None),
        }

    def get_track(self, uav_name: str) -> TrackPrediction | None:
        return self._tracks.get(uav_name)

    def get_all_tracks(self) -> dict[str, TrackPrediction]:
        return self._tracks.copy()

    def remove_uav(self, uav_name: str) -> TrackPrediction | None:
        """Forget the current live track of a UAV."""
        removed = self._tracks.pop(uav_name, None)
        if removed is not None:
            self._latest_conflicts = []
        return removed

    def complete_uav(self, uav_name: str) -> TrackPrediction | None:
        """Permanently reject tracks for a completed UAV."""
        self._completed_uavs.add(uav_name)
        return self.remove_uav(uav_name)

    def get_latest_conflicts(self) -> list[Conflict]:
        return list(self._latest_conflicts)

    def get_issued_update(self, uav_name: str) -> FlightPlan | None:
        return self._issued_updates.get(uav_name)

    def get_issued_update_history(self, uav_name: str) -> list[FlightPlan]:
        return list(self._issued_update_history.get(uav_name, ()))
