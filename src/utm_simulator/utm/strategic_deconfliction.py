import simpy

from utm_simulator.algorithms.interfaces.conflict_prediction import (
    ConflictPredictionAlgorithm,
)
from utm_simulator.algorithms.interfaces.strategic_deconfliction import (
    StrategicDeconflictionAlgorithm,
)
from utm_simulator.core.message import Message
from utm_simulator.core.network import CommunicationNetwork
from utm_simulator.core.processing_time import ProcessingTime, processing_time_config
from utm_simulator.domain.flight import FlightPlan, FlightPlanUpdate
from utm_simulator.domain.message_types import MessageType, StrategicDecision
from utm_simulator.utm.flight_plan_manager import FlightPlanManager


class StrategicDeconflictionModule:
    def __init__(
        self,
        name: str,
        env: simpy.Environment,
        network: CommunicationNetwork,
        conflict_prediction_algorithm: ConflictPredictionAlgorithm,
        deconfliction_algorithm: StrategicDeconflictionAlgorithm,
        flight_plan_manager: FlightPlanManager,
        processing_time: float | ProcessingTime = 0.0,
    ) -> None:
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)

        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.processing_time = processing_time_config(processing_time)

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

    def _process_proposal(self, proposal: Message):
        proposal_id = proposal.message_id
        requesting_uav = proposal.source
        proposed_flight_plan = proposal.payload["flight_plan"]
        self.network.record_event(
            component=self.name,
            event="PROPOSAL_RECEIVED",
            uav=requesting_uav,
            proposal_id=proposal_id,
        )

        delay = self.processing_time.calculate(
            self._processing_context(proposal)
        )
        if delay:
            yield self.env.timeout(delay)

        approved_snapshot = {
            uav_name: flight_plan
            for uav_name, flight_plan
            in self.flight_plan_manager.get_all().items()
            if uav_name != requesting_uav
        }  # All, except the requesting

        airspace_snapshot = (
            self.flight_plan_manager.get_airspace_snapshot()
        )
        predicted_conflicts = self.conflict_prediction_algorithm.predict(
            proposed_flight_plan=proposed_flight_plan,
            approved_flight_plans=approved_snapshot,
            airspace=airspace_snapshot,
        )
        self.network.record_event(
            component=self.name,
            event="CONFLICT_PREDICTION_COMPLETED",
            uav=requesting_uav,
            conflict_count=len(predicted_conflicts),
        )

        if not predicted_conflicts:
            decision = StrategicDecision.APPROVE
            counter_proposal = None
            updates: list[FlightPlanUpdate] = []

            plan_version = self.flight_plan_manager.approve(
                requesting_uav,
                proposed_flight_plan,
            )

        else:
            decision, counter_proposal, updates = (
                self.deconfliction_algorithm.deconflict(
                    proposed_flight_plan=proposed_flight_plan,
                    conflicts=predicted_conflicts,
                    approved_flight_plans=approved_snapshot,
                    airspace=airspace_snapshot,
                )
            )

            plan_version = self._apply_decision(
                requesting_uav=requesting_uav,
                proposed_flight_plan=proposed_flight_plan,
                decision=decision,
                counter_proposal=counter_proposal,
                updates=updates,
            )

        self.network.record_event(
            component=self.name,
            event="STRATEGIC_DECISION_MADE",
            uav=requesting_uav,
            decision=decision.value,
            update_count=len(updates),
        )

        final_response = Message(
            source=self.name,
            destination=requesting_uav,
            message_type=MessageType.SD_RESPONSE,
            payload={
                "decision": decision,
                "counter_proposal": counter_proposal,
                "plan_version": plan_version,
            },
            issue_timestamp=self.env.now,
            in_reply_to=proposal_id,
        )

        yield self.network.send(final_response)

    def _processing_context(self, proposal: Message) -> dict[str, object]:
        flight_plan = proposal.payload["flight_plan"]
        airspace = self.flight_plan_manager.get_airspace_snapshot()
        return {
            "module": self.name,
            "message": proposal,
            "uav_name": proposal.source,
            "queue_length": len(self.inbox.items),
            "approved_flight_plan_count": len(
                self.flight_plan_manager.get_all()
            ),
            "waypoint_count": len(flight_plan.waypoint_list),
            "obstacle_count": len(airspace.obstacles),
            "geofence_count": len(airspace.geofences),
        }

    def _apply_decision(
        self,
        requesting_uav: str,
        proposed_flight_plan: FlightPlan,
        decision: StrategicDecision,
        counter_proposal: FlightPlan | None,
        updates: list[FlightPlanUpdate],
    ) -> int | None:
        plan_version = None
        if decision == StrategicDecision.COUNTER_PROPOSAL:
            if counter_proposal is not None:
                plan_version = self.flight_plan_manager.approve(
                    requesting_uav,
                    counter_proposal,
                )

        elif decision == StrategicDecision.APPROVE:
            plan_version = self.flight_plan_manager.approve(
                requesting_uav,
                proposed_flight_plan,
            )

        elif decision == StrategicDecision.REJECT:
            pass

        if updates:
            self.flight_plan_manager.launch_updates(updates)

        return plan_version
