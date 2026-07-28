from typing import Protocol

class StrategicDeconflictionAlgorithm(Protocol):
    """
    Contract for strategic deconfliction algorithms.

    Any compatible algorithm must implement the deconflict() method.
    """

    def deconflict(self, proposed_flight_plan, conflicts, approved_flight_plans):
        """
        Predict conflicts between a proposed flight plan and
        the currently approved flight plans.
        """
        ...