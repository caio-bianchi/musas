from typing import Protocol

from src.airspace.airspace import Airspace
from src.domain.tracking import TrackPrediction


class TacticalConflictPredictionAlgorithm(Protocol):
    """Contract for detecting conflicts in short-term track predictions."""

    def predict(
        self,
        updated_track: TrackPrediction,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> list:
        ...
