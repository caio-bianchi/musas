from abc import ABC, abstractmethod

from utm_simulator.domain.flight import FlightPlan, Conflict
from utm_simulator.airspace.airspace import Airspace


class ConflictPredictionAlgorithm(ABC):
    """
    Contract for conflict prediction algorithms.

    Any compatible algorithm must implement the predict() method.
    """

    @abstractmethod
    def predict(
        self,
        proposed_flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[Conflict]:
        """
        Predict conflicts between a proposed flight plan and
        the currently approved flight plans.
        """
        ...
