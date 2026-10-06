from typing import Protocol, Sequence

from utm_simulator.domain.flight import FlightPlan
from utm_simulator.domain.tracking import TrackPrediction, TrackingReport


class TrackingAlgorithm(Protocol):
    """
    Contract for tracking algorithms.

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
        """
        Estimate the current UAV state and predict its future states.

        prediction_horizon is the number of future time steps.
        prediction_step is the duration of each time step.
        """
        ...