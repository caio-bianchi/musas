from typing import Protocol

from src.airspace.airspace import Airspace
from src.domain.flight import Conflict, FlightPlan
from src.domain.tracking import TrackPrediction


class TacticalDeconflictionAlgorithm(Protocol):
    """
    Contract for producing flight-plan updates for tactical conflicts.
    """

    def deconflict(
        self,
        conflicts: list[Conflict],
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> dict[str, FlightPlan]:
        """
        Resolve tactical conflicts by producing flight-plan updates.

        Returns a mapping from UAV names to their updated flight plans.
        """
        ...