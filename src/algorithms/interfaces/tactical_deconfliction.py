from typing import Protocol

from src.airspace.airspace import Airspace
from src.domain.flight import FlightPlan
from src.domain.tracking import TrackPrediction


class TacticalDeconflictionAlgorithm(Protocol):
    """Contract for producing flight-plan updates for tactical conflicts."""

    def deconflict(
        self,
        conflicts: list,
        tracks: dict[str, TrackPrediction],
        flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> dict[str, FlightPlan]:
        ...
