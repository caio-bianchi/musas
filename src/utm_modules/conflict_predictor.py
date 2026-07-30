from src.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm


class ConflictPredictor:
    def __init__(self, name, algorithm: ConflictPredictionAlgorithm):
        self.name = name
        self.algorithm = algorithm

    def predict(self, proposed_flight_plans, approved_flight_plans):
        return self.algorithm.predict(proposed_flight_plan=proposed_flight_plans, approved_flight_plans=approved_flight_plans)
