from abc import ABC, abstractmethod

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.domain.flight import Conflict, FlightPlan
from utm_simulator.domain.message_types import StrategicDecision


class StrategicDeconflictionAlgorithm(ABC):
    """
    Contract for strategic deconfliction algorithms.

    Any compatible algorithm must implement the deconflict() method.
    """

    @abstractmethod
    def deconflict(
        self,
        proposed_flight_plan: FlightPlan,
        conflicts: list[Conflict],
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> tuple[
        StrategicDecision,
        FlightPlan | None,
        dict[str, FlightPlan],
    ]:
        """
        Attempt to resolve conflicts affecting a proposed flight plan.

        Returns the strategic decision, an optional counterproposal,
        and updates to other flight plans.
        """
        ...
