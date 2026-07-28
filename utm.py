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
            # if request.payload["approved_flight_plans"] is None: request.payload["approved_flight_plans"] = {}
            # trajectories = request.payload["proposed_flight_plan"] + request.payload["approved_flight_plans"]
            trajectories = 0
            predicted_conflicts = self.predict_conflicts(trajectories)

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

            # Dictionary for pending requests addresses
            self.pending_requests = {}

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
        proposal_id = proposal_msg.message_id

        # Add proposal address to the pending_requests dictionary
        self.pending_requests[proposal_id] = {
            "uav": proposal_msg.source,
            "flight_plan": proposal_msg.payload,
            "received_at": self.env.now,
        }

        conflict_request = Message(
            source = self.name,
            destination = self.conflict_predictor,
            message_type = "REQUEST_CONFLICT_PREDICTION",
            payload={
                "proposed_flight_plan": proposal_msg.payload,
                "approved_flight_plans": self.get_approved_flight_plans()
            },
            issue_timestamp = self.env.now,
            correlation_id=proposal_id,
        )

        yield self.network.send(conflict_request)

    def _handle_conflict_predictions(self, response):
        proposal_id = response.correlation_id
        pending = self.pending_requests.get(proposal_id)

        requesting_uav = pending["uav"]
        proposed_flight_plan = pending["flight_plan"]
        predicted_conflicts = response.payload["predicted_conflicts"]

        if predicted_conflicts is None:
            decision = APPROVE
            counter_proposal = proposed_flight_plan
            updates = None

        else:
            # decision, counter_proposal, updates = self._strategic_deconfliction(predicted_conflicts, proposed_flight_plan)
            decision, counter_proposal, updates = "X", "Y", "Z" # just a placeholder for testing
        final_response = Message(
            source = self.name,
            destination = requesting_uav,
            message_type = "SD_RESPONSE",
            payload = {
                "decision": decision,
                "counter_proposal": counter_proposal
            },
            issue_timestamp = self.env.now,
            correlation_id = proposal_id
        )

        yield self.network.send(final_response)
        del self.pending_requests[proposal_id]

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

    

    