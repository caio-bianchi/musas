from abc import ABC, abstractmethod

from musas.airspace.airspace import Airspace
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.tracking import TrackPrediction


class TacticalDeconflictionAlgorithm(ABC):
    """
    Contract for producing flight-plan updates for tactical conflicts.
    """

    @abstractmethod
    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[FlightPlanUpdate]:
        """
        Resolve tactical conflicts by producing flight-plan updates.

        Returns the requested flight-plan updates.
        """
        ...
