from utm_simulator.airspace.airspace import Airspace
from utm_simulator.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from utm_simulator.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from utm_simulator.domain.flight import Conflict, FlightPlan
from utm_simulator.domain.message_types import StrategicDecision


class DummyStrategicDeconfliction(
    ConflictPredictionAlgorithm,
    StrategicDeconflictionAlgorithm,
):
    """Approve every proposed flight plan without checking conflicts."""

    def predict(
        self,
        proposed_flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> list[Conflict]:
        return []

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
        return StrategicDecision.APPROVE, None, {}
