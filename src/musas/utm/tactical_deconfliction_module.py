import simpy

from musas.algorithms.builtins.constant_time_processing import (
    ConstantTimeProcessing,
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
        processing_time: ProcessingTimeModel = ConstantTimeProcessing(0),
    ) -> None:

        self.name = name
        self.env = env
        self.network = network
        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.processing_time = processing_time

        self.inbox = simpy.Store(env)
        self._tracks: dict[str, TrackPrediction] = {}
        self._completed_uavs: set[str] = set()
        self._latest_conflicts: list[Conflict] = []
        self._issued_updates: dict[str, FlightPlan] = {}
        self._issued_update_history: dict[str, list[FlightPlan]] = {}

        network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if message.message_type == MessageType.TRACK_UPDATE:
                yield self.env.process(
                    self.process_track_update(message)
                )

    def _processing_context(
        self,
        message: Message,
        operation: str,
        conflict_count: int = 0,
    ) -> dict[str, object]:
        updated_track = message.payload["track_prediction"]
        incoming_uav_is_new = updated_track.uav_name not in self._tracks
        return {
            "module": self.name,
            "operation": operation,
            "message": message,
            "uav_name": updated_track.uav_name,
            "queue_length": len(self.inbox.items),
            "track_count": len(self._tracks) + int(incoming_uav_is_new),
            "flight_plan_count": len(self.flight_plan_manager.get_all()),
            "predicted_state_count": len(updated_track.predicted_states),
            "conflict_count": conflict_count,
        }

    def process_track_update(
        self,
        message: Message,
    ):
        updated_track = message.payload["track_prediction"]
        if updated_track.uav_name in self._completed_uavs:
            return None
        current_track = self._tracks.get(updated_track.uav_name)

        if (
            current_track is not None
            and updated_track.generated_at < current_track.generated_at
        ):
            return None

        self._tracks[updated_track.uav_name] = updated_track

        airspace = self.flight_plan_manager.get_airspace_snapshot()
        prediction_delay = self.processing_time.calculate(
            **self._processing_context(
                message,
                "conflict_prediction",
            )
        )
        if prediction_delay < 0:
            raise ValueError("processing time cannot be negative")
        if prediction_delay:
            yield self.env.timeout(prediction_delay)

        conflicts = self.conflict_prediction_algorithm.predict(
            updated_track=updated_track,
            tracks=self.get_all_tracks(),
            airspace=airspace,
        )
        self._latest_conflicts = list(conflicts)

        if not conflicts:
            return None

        self.network.record_event(
            component=self.name,
            event="TACTICAL_CONFLICTS_DETECTED",
            uav=updated_track.uav_name,
            conflict_count=len(conflicts),
        )

        deconfliction_delay = self.processing_time.calculate(
            **self._processing_context(
                message,
                "deconfliction",
                conflict_count=len(conflicts),
            )
        )
        if deconfliction_delay < 0:
            raise ValueError("processing time cannot be negative")
        if deconfliction_delay:
            yield self.env.timeout(deconfliction_delay)

        updates = self.deconfliction_algorithm.deconflict(
            conflicts=conflicts,
            tracks=self.get_all_tracks(),
            flight_plans=self.flight_plan_manager.get_all(),
            airspace=airspace,
        )
        if updates:
            self.network.record_event(
                component=self.name,
                event="TACTICAL_UPDATES_ISSUED",
                uavs=[update.uav_name for update in updates],
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
