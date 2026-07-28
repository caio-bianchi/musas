import simpy
from core.message import Message
from src.algorithms.interfaces.strategic_deconfliction import StrategicDeconflictionAlgorithm

class StrategicDeconflictionModule:
    def __init__(self, name, env, network, conflict_predictor, algorithm: StrategicDeconflictionAlgorithm):
            self.name = name
            self.env = env
            self.network = network
            self.inbox = simpy.Store(env)
            self.conflict_predictor = conflict_predictor
            self.algorithm = algorithm

            # Dictionary for pending requests addresses
            self.pending_requests = {}

            network.register(self)

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
            "flight_plan": proposal_msg.payload["flight_plan"],
            "received_at": self.env.now,
        }

        conflict_request = Message(
            source = self.name,
            destination = self.conflict_predictor,
            message_type = "REQUEST_CONFLICT_PREDICTION",
            payload={
                "proposed_flight_plan": proposal_msg.payload["flight_plan"],
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
            decision = "APPROVE"
            counter_proposal = proposed_flight_plan
            updates = None

        else:
            decision, counter_proposal, updates = self.algorithm.deconflict(proposed_flight_plan, conflicts, approved_flight_plans)
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
