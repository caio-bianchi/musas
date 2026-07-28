import simpy
from src.core.message import Message
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
            self.proposal_queue = simpy.Store(env)
            self.pending_cp_responses = {}
            self.pending_requests = {}

            # Dictionary for approved flight plans
            self.approved_flight_plans = {}
            # TODO: Implement FlightPlanManager as a data auxiliary
            #  structure to provide source of truth

            network.register(self)

    def run(self):
        self.env.process(self._proposal_worker())
        while True:
            message = yield self.inbox.get()

            if message.message_type == "PROPOSAL":
                yield self.proposal_queue.put(message)

            elif message.message_type == "CONFLICT_PREDICTIONS":
                self._deliver_cp_response(message)

    def _proposal_worker(self):
        while True:
            proposal = yield self.proposal_queue.get()

            yield self.env.process(
                self._process_proposal(proposal)
            )

    def _process_proposal(self, proposal):
        proposal_id = proposal.message_id
        requesting_uav = proposal.source
        proposed_flight_plan = proposal.payload["flight_plan"]

        approved_snapshot = self.approved_flight_plans.copy()

        self.pending_requests[proposal_id] = {
            "uav": requesting_uav,
            "flight_plan": proposed_flight_plan,
            "approved_flight_plans": approved_snapshot,
            "received_at": self.env.now,
        }

        # Event that will me done when the CP answers.
        response_event = self.env.event()
        self.pending_cp_responses[proposal_id] = response_event

        conflict_request = Message(
            source=self.name,
            destination=self.conflict_predictor,
            message_type="REQUEST_CONFLICT_PREDICTION",
            payload={
                "proposed_flight_plan": proposed_flight_plan,
                "approved_flight_plans": approved_snapshot,
            },
            issue_timestamp=self.env.now,
            correlation_id=proposal_id,
        )

        self.network.send(conflict_request)

        # The worker stops here, but run() keeps receiving messages.
        cp_response = yield response_event

        predicted_conflicts = cp_response.payload[
            "predicted_conflicts"
        ]

        if not predicted_conflicts:
            decision = "APPROVE"
            counter_proposal = None
            updates = None

            self.approved_flight_plans[
                requesting_uav
            ] = proposed_flight_plan

        else:
            decision, counter_proposal, updates = (
                self.algorithm.deconflict(
                    proposed_flight_plan=proposed_flight_plan,
                    conflicts=predicted_conflicts,
                    approved_flight_plans=approved_snapshot,
                )
            )

            self._apply_decision(
                requesting_uav=requesting_uav,
                proposed_flight_plan=proposed_flight_plan,
                decision=decision,
                counter_proposal=counter_proposal,
                updates=updates,
            )

        final_response = Message(
            source=self.name,
            destination=requesting_uav,
            message_type="SD_RESPONSE",
            payload={
                "decision": decision,
                "counter_proposal": counter_proposal,
            },
            issue_timestamp=self.env.now,
            correlation_id=proposal_id,
        )

        yield self.network.send(final_response)

        self.pending_requests.pop(proposal_id, None)
        self.pending_cp_responses.pop(proposal_id, None)

    def _deliver_cp_response(self, response):
        response_event = self.pending_cp_responses.get(
            response.correlation_id
        )

        if response_event is None:
            return

        if not response_event.triggered:
            # Here, the CP response is triggered.
            response_event.succeed(response)

    def _apply_decision(
        self,
        requesting_uav,
        proposed_flight_plan,
        decision,
        counter_proposal,
        updates,
    ):
        if decision == "COUNTER_PROPOSAL":
            self.approved_flight_plans[
                requesting_uav
            ] = counter_proposal

        elif decision == "APPROVE":
            self.approved_flight_plans[requesting_uav] = proposed_flight_plan

        elif decision == "REJECTED":
            pass

        if updates:
            for uav_name, updated_plan in updates.items():
                self.approved_flight_plans[
                    uav_name
                ] = updated_plan