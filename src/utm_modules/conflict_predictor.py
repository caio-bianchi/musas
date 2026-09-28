from src.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm
from src.domain.flight import Conflict, FlightPlan
from src.utm_modules.flight_plan_manager import FlightPlanManager


class ConflictPredictor:
    def __init__(
        self,
        name: str,
        algorithm: ConflictPredictionAlgorithm,
        flight_plan_manager: FlightPlanManager,
    ) -> None:
        self.name = name
        self.algorithm = algorithm
        self.flight_plan_manager = flight_plan_manager

    def predict(
        self,
        proposed_flight_plan: FlightPlan,
        approved_flight_plans: dict[str, FlightPlan],
    ) -> list[Conflict]:
        airspace = self.flight_plan_manager.get_airspace_snapshot()
        return self.algorithm.predict(
            proposed_flight_plan=proposed_flight_plan,
            approved_flight_plans=approved_flight_plans,
            airspace=airspace,
        )
