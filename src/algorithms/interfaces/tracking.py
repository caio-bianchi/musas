from typing import Protocol, Sequence

from src.domain.flight import FlightPlan
from src.domain.tracking import TrackPrediction, TrackingReport

class TrackingAlgorithm(Protocol):
    """
    Contract for tracking algorithm.

    Any compatible algorithm must implement the predict() method.
    """

    def predict(
        self,
        uav_name: str,
        report_history: Sequence[TrackingReport],
        flight_plan: FlightPlan,
        prediction_start: float,
        prediction_horizon: int,
        prediction_step: float,
    ) -> TrackPrediction:
        ...
