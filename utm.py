import simpy
from utils import Message

APPROVE = 1
REJECT = 0

class ConflictPredictor:
    def __init__(self, name, env, network):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        network.register(self)

    def handle_request(self):
        while True:
            request = yield self.inbox.get()

            # trajectories = request.payload.trajectories
            trajectories = 0
            source = request.source

            predicted_conflicts = self.predict_conflicts(trajectories)

            response = Message(
                source=self.name,
                destination=request.source,
                message_type="CONFLICT_PREDICTIONS",
                payload={
                    "request": request.payload["proposal_msg"],
                    "predicted_conflicts": predicted_conflicts
                },
                issue_timestamp=self.env.now
            )

            self.network.send(response)

    def predict_conflicts(self, trajectories):
        return "TBD: Predicted Conflicts"
        pass


class StrategicDeconflictionModule:
    def __init__(self, name, env, network, conflict_predictor):
            self.name = name
            self.env = env
            self.network = network
            self.inbox = simpy.Store(env)
            self.conflict_predictor = conflict_predictor
            network.register(self)

    def get_approved_flight_plans(self):
        return

    def handle_request(self):
            while True:
                msg = yield self.inbox.get()
                print(msg)

                if msg.message_type == "PROPOSAL":
                    print("a")
                    self.env.process(self._handle_proposal(msg))

                elif msg.message_type == "CONFLICT_PREDICTIONS":
                    self.env.process(self._handle_conflict_predictions(msg))

                    
                # How to handle response?

    def _handle_proposal(self,proposal_msg):
        print("b")
        proposed_flight_plan = proposal_msg.payload
        approved_flight_plans = self.get_approved_flight_plans()

        look_for_conflicts_message = Message(
            source = self.name,
            destination = self.conflict_predictor,
            message_type = "REQUEST_CONFLICT_PREDICTION",
            payload={
                "proposal_msg": proposal_msg,
                "approved_flight_plans": approved_flight_plans
            },
            issue_timestamp = self.env.now
        )

        yield self.network.send(look_for_conflicts_message)

    def _handle_conflict_predictions(self, conflict_prediction_msg):
        initial_request = conflict_prediction_msg.payload["request"]
        proposed_flight_plan = initial_request.payload

        predicted_conflicts = conflict_prediction_msg.payload["predicted_conflicts"]

        if predicted_conflicts is None:
            decision = APPROVE
            counter_proposal = None
            updates = None

        else:
            # decision, counter_proposal, updates = self._strategic_deconfliction(predicted_conflicts, proposed_flight_plan)
            decision, counter_proposal, updates = "X", "Y", "Z"
        response = Message(
            source = self.name,
            destination = initial_request.source,
            message_type = "SD_RESPONSE",
            payload = {
                "decision": decision,
                "counter_proposal": counter_proposal
            },
            issue_timestamp = self.env.now
        )

        yield self.network.send(response)

    def _strategic_deconfliction(self, predicted_conflicts, proposed_flight_plan):
        pass



################################
class TrackingModule:
    def __init__(self, name, env, network, processing_time):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        self.processing_time = processing_time
        network.register(self)

    
    def run(self):
        while True:
            msg = yield self.inbox.get()

            print(
                f"t{self.env.now}: {self.name} processing "
                f"{msg.message_type} from {msg.source}"
            )

            yield self.env.timeout(self.processing_time)

            if msg.message_type == "PERIODIC_REPORT":
                self.process_report(msg)

    def process_report(self, message):
        print(
            f"t{self.env.now}: position of "
            f"{message.source} updated"
        )

class TacticalDeconflictionModule:
    def __init__(self, name, env, network):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        network.register(self)

    

    