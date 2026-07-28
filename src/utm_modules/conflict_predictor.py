import simpy
from utils import Message
from src.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm

class ConflictPredictor:
    def __init__(self, name, env, network, algorithm: ConflictPredictionAlgorithm):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)

        self.algorithm = algorithm

        network.register(self)

    def handle_request(self):
        while True:
            request = yield self.inbox.get()

            # trajectories = request.payload.trajectories
            # if request.payload["approved_flight_plans"] is None: request.payload["approved_flight_plans"] = {}
            # trajectories = request.payload["proposed_flight_plan"] + request.payload["approved_flight_plans"]
            trajectories = 0
            predicted_conflicts = self.algorithm.predict(trajectories)

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
