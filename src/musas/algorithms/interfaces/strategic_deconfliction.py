from abc import ABC, abstractmethod

from musas.airspace.airspace import Airspace
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.message_types import StrategicDecision


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
        inactive_flight_plans: dict[str, FlightPlan],
        active_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> tuple[
        StrategicDecision,
        FlightPlan | None,
        list[FlightPlanUpdate],
    ]:
        """
        Attempt to resolve conflicts affecting a proposed flight plan.

        Returns the strategic decision, an optional counterproposal,
        and updates to other flight plans.

        Inactive flight plans may be updated. Active flight plans are
        immutable context that must still be considered by the algorithm.
        """
        ...
