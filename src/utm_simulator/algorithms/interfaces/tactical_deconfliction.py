from abc import ABC, abstractmethod

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from utm_simulator.domain.tracking import TrackPrediction


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
