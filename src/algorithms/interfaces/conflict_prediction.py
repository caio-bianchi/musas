from typing import Protocol

class ConflictPredictionAlgorithm(Protocol):
    """
    Contract for conflict prediction algorithms.

    Any compatible algorithm must implement the predict() method.
    """

    def predict(self, proposed_flight_plan, approved_flight_plans, airspace):
        """
        Predict conflicts between a proposed flight plan and
        the currently approved flight plans.
        """
        ...