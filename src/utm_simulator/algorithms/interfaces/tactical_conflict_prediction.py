from typing import Protocol

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.domain.flight import Conflict
from utm_simulator.domain.tracking import TrackPrediction


class TacticalConflictPredictionAlgorithm(Protocol):
    """
    Contract for detecting conflicts in short-term track predictions.
    """

    def predict(
        self,
        updated_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> list[Conflict]:
        """
        Predict tactical conflicts involving the updated track.

        Returns all detected conflicts as Conflict instances.
        """
        ...