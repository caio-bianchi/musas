import simpy
from src.core.message import Message
from src.algorithms.interfaces.strategic_deconfliction import StrategicDeconflictionAlgorithm
from src.domain.message_features import MessageType, StrategicDecision
from src.utm_modules.conflict_predictor import ConflictPredictor

class StrategicDeconflictionModule:
    def __init__(self, name, env, network, conflict_predictor: ConflictPredictor, algorithm: StrategicDeconflictionAlgorithm):
            self.name = name
            self.env = env
            self.network = network
            self.inbox = simpy.Store(env)
            self.conflict_predictor = conflict_predictor
            self.algorithm = algorithm

            # Dictionary for approved flight plans
            self.approved_flight_plans = {}
            # TODO: Implement FlightPlanManager as a data auxiliary
            #  structure to provide source of truth

            network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if message.message_type == MessageType.PROPOSAL:
                # Here, I wait for the proposal is processed
                yield self.env.process(
                    self._process_proposal(message)
                )

    def _process_proposal(self, proposal):
        proposal_id = proposal.message_id
        requesting_uav = proposal.source
        proposed_flight_plan = proposal.payload["flight_plan"]

        approved_snapshot = self.approved_flight_plans.copy()

        predicted_conflicts = self.conflict_predictor.predict(proposed_flight_plan, approved_snapshot)

        if not predicted_conflicts:
            decision = StrategicDecision.APPROVE
            counter_proposal = None
            updates = []

            self.approved_flight_plans[requesting_uav] = (
                proposed_flight_plan
            )

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
            message_type=MessageType.SD_RESPONSE,
            payload={
                "decision": decision,
                "counter_proposal": counter_proposal,
            },
            issue_timestamp=self.env.now,
            in_reply_to=proposal_id,
        )

        yield self.network.send(final_response)

    def _apply_decision(
        self,
        requesting_uav,
        proposed_flight_plan,
        decision,
        counter_proposal,
        updates,
    ):
        if decision == StrategicDecision.COUNTER_PROPOSAL:
            self.approved_flight_plans[
                requesting_uav
            ] = counter_proposal

        elif decision == StrategicDecision.APPROVE:
            self.approved_flight_plans[requesting_uav] = proposed_flight_plan

        elif decision == StrategicDecision.REJECT:
            pass

        if updates:
            for uav_name, updated_plan in updates.items():
                self.approved_flight_plans[
                    uav_name
                ] = updated_plan