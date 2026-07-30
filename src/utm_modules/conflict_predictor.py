import simpy
from src.core.message import Message
from src.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm
from src.domain.message_features import MessageType

class ConflictPredictor:
    def __init__(self, name, algorithm: ConflictPredictionAlgorithm):
        self.name = name
        self.algorithm = algorithm

    def predict(self, proposed_flight_plans, approved_flight_plans):
        return self.algorithm.predict(proposed_flight_plan=proposed_flight_plans, approved_flight_plans=approved_flight_plans)
