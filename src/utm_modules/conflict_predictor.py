import simpy
from src.core.message import Message
from src.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm

class ConflictPredictor:
    def __init__(self, name, env, network, algorithm: ConflictPredictionAlgorithm):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)

        self.algorithm = algorithm

        network.register(self)

    def run(self):
        while True:

            # Wait for a request in the inbox
            request = yield self.inbox.get()

            proposed_flight_plans = request.payload["proposed_flight_plan"]
            approved_flight_plans = request.payload["approved_flight_plans"]

            predicted_conflicts = self.algorithm.predict(
                proposed_flight_plan=proposed_flight_plans,
                approved_flight_plans=approved_flight_plans
            )

            response = Message(
                source=self.name,
                destination=request.source,
                message_type="CONFLICT_PREDICTIONS",
                payload={
                    "predicted_conflicts": predicted_conflicts
                },
                issue_timestamp=self.env.now,
                correlation_id=request.correlation_id
            )

            self.network.send(response)
