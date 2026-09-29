import simpy

from src.algorithms.interfaces.tactical_conflict_prediction import (
    TacticalConflictPredictionAlgorithm,
)
from src.algorithms.interfaces.tactical_deconfliction import (
    TacticalDeconflictionAlgorithm,
)
from src.core.message import Message
from src.core.network import CommunicationNetwork
from src.domain.flight import Conflict, FlightPlan
from src.domain.message_features import MessageType
from src.domain.tracking import TrackPrediction
from src.utm_modules.flight_plan_manager import FlightPlanManager


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
        processing_time: float = 0.0
    ) -> None:

        self.name = name
        self.env = env
        self.network = network
        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.processing_time = float(processing_time)
        if self.processing_time < 0:
            raise ValueError("processing_time must be non-negative")

        self.inbox = simpy.Store(env)
        self._tracks: dict[str, TrackPrediction] = {}
        self._latest_conflicts: list[Conflict] = []
        self._issued_updates: dict[str, FlightPlan] = {}
        self._issued_update_history: dict[str, list[FlightPlan]] = {}

        network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if self.processing_time:
                yield self.env.timeout(self.processing_time)

            if message.message_type == MessageType.TRACK_UPDATE:
                self.process_track_update(message)

    def process_track_update(self, message: Message) -> dict[str, FlightPlan] | None:
        updated_track = message.payload["track_prediction"]
        current_track = self._tracks.get(updated_track.uav_name)

        if (current_track is not None and updated_track.generated_at < current_track.generated_at):
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

        self.network.log_event(
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
            self.network.log_event(
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

    def get_latest_conflicts(self) -> list[Conflict]:
        return list(self._latest_conflicts)

    def get_issued_update(self, uav_name: str) -> FlightPlan | None:
        return self._issued_updates.get(uav_name)

    def get_issued_update_history(self, uav_name: str) -> list[FlightPlan]:
        return list(self._issued_update_history.get(uav_name, ()))
