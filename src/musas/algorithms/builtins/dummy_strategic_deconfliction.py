from musas.airspace.airspace import Airspace
from musas.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm
from musas.algorithms.interfaces.strategic_deconfliction import StrategicDeconflictionAlgorithm
from musas.domain.flight import Conflict, FlightPlan, FlightPlanUpdate
from musas.domain.message_types import StrategicDecision


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
        inactive_flight_plans: dict[str, FlightPlan],
        active_flight_plans: dict[str, FlightPlan],
        airspace: Airspace,
    ) -> tuple[
        StrategicDecision,
        FlightPlan | None,
        list[FlightPlanUpdate],
    ]:
        return StrategicDecision.APPROVE, None, []
