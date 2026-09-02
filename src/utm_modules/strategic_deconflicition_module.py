import simpy
from src.core.message import Message
from src.algorithms.interfaces.strategic_deconfliction import StrategicDeconflictionAlgorithm
from src.domain.message_features import MessageType, StrategicDecision
from src.utm_modules.conflict_predictor import ConflictPredictor
from src.utm_modules.flight_plan_manager import FlightPlanManager

class StrategicDeconflictionModule:
    def __init__(self, name, env, network, conflict_predictor: ConflictPredictor, algorithm: StrategicDeconflictionAlgorithm, flight_plan_manager: FlightPlanManager):
            self.name = name
            self.env = env
            self.network = network
            self.inbox = simpy.Store(env)

            self.conflict_predictor = conflict_predictor
            self.flight_plan_manager = flight_plan_manager

            self.algorithm = algorithm

            network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if message.message_type == MessageType.PROPOSAL:
                # Here, I wait for the proposal is processed
                yield self.env.process(
                    self._process_proposal(message)
                )

            elif message.message_type == MessageType.FLIGHT_STARTED:
                self.flight_plan_manager.activate(
                    message.source
                )

            elif message.message_type == MessageType.FLIGHT_COMPLETED:
                self.flight_plan_manager.complete(
                    message.source
                )

    def _process_proposal(self, proposal):
        proposal_id = proposal.message_id
        requesting_uav = proposal.source
        proposed_flight_plan = proposal.payload["flight_plan"]

        approved_snapshot = {
            uav_name: flight_plan
            for uav_name, flight_plan
            in self.flight_plan_manager.get_all().items()
            if uav_name != requesting_uav
        } # All, except the requesting

        predicted_conflicts = self.conflict_predictor.predict(proposed_flight_plan, approved_snapshot)

        if not predicted_conflicts:
            decision = StrategicDecision.APPROVE
            counter_proposal = None
            updates = []

            self.flight_plan_manager.approve(requesting_uav, proposed_flight_plan)

        else:
            airspace_snapshot = (
                self.flight_plan_manager.get_airspace_snapshot()
            )

            decision, counter_proposal, updates = (
                self.algorithm.deconflict(
                    proposed_flight_plan=proposed_flight_plan,
                    conflicts=predicted_conflicts,
                    approved_flight_plans=approved_snapshot,
                    airspace=airspace_snapshot,
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
            self.flight_plan_manager.approve(requesting_uav, counter_proposal)

        elif decision == StrategicDecision.APPROVE:
            self.flight_plan_manager.approve(requesting_uav, proposed_flight_plan)

        elif decision == StrategicDecision.REJECT:
            pass

        if updates:
            self.flight_plan_manager.launch_updates(updates)