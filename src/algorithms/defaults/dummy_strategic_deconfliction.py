from src.airspace.airspace import Airspace
from src.domain.flight import Conflict, FlightPlan
from src.domain.message_features import StrategicDecision


class DummyStrategicDeconfliction:
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
