from abc import ABC, abstractmethod

from musas.airspace.airspace import Airspace
from musas.domain.flight import Conflict
from musas.domain.tracking import TrackPrediction


class TacticalConflictPredictionAlgorithm(ABC):
    """
    Contract for detecting conflicts in short-term track predictions.
    """

    @abstractmethod
    def predict(
        self,
        tracks: dict[str, TrackPrediction],
        airspace: Airspace,
    ) -> list[Conflict]:
        """
        Predict tactical conflicts in a snapshot of the current tracks.

        Each traffic pair should be evaluated only once. Returns all detected
        conflicts as Conflict instances.
        """
        ...
