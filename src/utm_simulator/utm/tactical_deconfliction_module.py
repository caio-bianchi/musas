import simpy

from utm_simulator.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from utm_simulator.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from utm_simulator.core.message import Message
from utm_simulator.core.network import CommunicationNetwork
from utm_simulator.core.processing_time import ProcessingTime, processing_time_config
from utm_simulator.domain.flight import Conflict, FlightPlan
from utm_simulator.domain.message_types import MessageType
from utm_simulator.domain.tracking import TrackPrediction
from utm_simulator.utm.flight_plan_manager import FlightPlanManager


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
        processing_time: float | ProcessingTime = 0.0,
    ) -> None:

        self.name = name
        self.env = env
        self.network = network
        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.processing_time = processing_time_config(processing_time)

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
                delay = self.processing_time.calculate(
                    self._processing_context(message)
                )
                if delay:
                    yield self.env.timeout(delay)
                self.process_track_update(message)

    def _processing_context(self, message: Message) -> dict[str, object]:
        updated_track = message.payload["track_prediction"]
        incoming_uav_is_new = updated_track.uav_name not in self._tracks
        return {
            "module": self.name,
            "message": message,
            "uav_name": updated_track.uav_name,
            "queue_length": len(self.inbox.items),
            "track_count": len(self._tracks) + int(incoming_uav_is_new),
            "flight_plan_count": len(self.flight_plan_manager.get_all()),
            "predicted_state_count": len(updated_track.predicted_states),
        }

    def process_track_update(self, message: Message) -> dict[str, FlightPlan] | None:
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
                uavs=list(updates),
            )
            self._issued_updates.update(updates)
            for uav_name, flight_plan in updates.items():
                self._issued_update_history.setdefault(uav_name, []).append(
                    flight_plan
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
