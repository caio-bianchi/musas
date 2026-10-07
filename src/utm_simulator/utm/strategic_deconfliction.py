import simpy

from utm_simulator.algorithms.builtins.constant_time_processing import ConstantTimeProcessing
from utm_simulator.algorithms.interfaces.conflict_prediction import ConflictPredictionAlgorithm
from utm_simulator.algorithms.interfaces.strategic_deconfliction import StrategicDeconflictionAlgorithm
from utm_simulator.algorithms.interfaces.processing_time import ProcessingTimeModel
from utm_simulator.core.message import Message
from utm_simulator.core.network import CommunicationNetwork
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
        processing_time: ProcessingTimeModel = ConstantTimeProcessing(0),
    ) -> None:
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)

        self.conflict_prediction_algorithm = conflict_prediction_algorithm
        self.deconfliction_algorithm = deconfliction_algorithm
        self.flight_plan_manager = flight_plan_manager
        self.processing_time = processing_time

        network.register(self)

    def run(self):
        while True:
            message = yield self.inbox.get()

            if message.message_type == MessageType.PROPOSAL:
                yield self.env.process(
                    self._process_proposal(message)
                )

            elif message.message_type == MessageType.FLIGHT_STARTED:
                self.flight_plan_manager.activate(message.source)

            elif message.message_type == MessageType.FLIGHT_COMPLETED:
                self.flight_plan_manager.complete(message.source)

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

        if self.flight_plan_manager.is_active(requesting_uav):
            yield self._reject_active_proposal(proposal)
            return

        approved_snapshot = {
            uav_name: flight_plan
            for uav_name, flight_plan
            in self.flight_plan_manager.get_all().items()
            if uav_name != requesting_uav
        }  # All, except the requesting
        active_snapshot = {
            uav_name: flight_plan
            for uav_name, flight_plan
            in self.flight_plan_manager.get_active().items()
            if uav_name != requesting_uav
        }
        inactive_snapshot = {
            uav_name: flight_plan
            for uav_name, flight_plan
            in self.flight_plan_manager.get_inactive_approved().items()
            if uav_name != requesting_uav
        }

        airspace_snapshot = (
            self.flight_plan_manager.get_airspace_snapshot()
        )

        prediction_delay = self.processing_time.calculate(
            **self._processing_context(
                proposal,
                "conflict_prediction",
            )
        )
        if prediction_delay < 0:
            raise ValueError("processing time cannot be negative")
        if prediction_delay:
            yield self.env.timeout(prediction_delay)

        if self.flight_plan_manager.is_active(requesting_uav):
            yield self._reject_active_proposal(proposal)
            return

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
            deconfliction_delay = self.processing_time.calculate(
                **self._processing_context(
                    proposal,
                    "deconfliction",
                )
            )
            if deconfliction_delay < 0:
                raise ValueError("processing time cannot be negative")
            if deconfliction_delay:
                yield self.env.timeout(deconfliction_delay)

            decision, counter_proposal, updates = (
                self.deconfliction_algorithm.deconflict(
                    proposed_flight_plan=proposed_flight_plan,
                    conflicts=predicted_conflicts,
                    inactive_flight_plans=inactive_snapshot,
                    active_flight_plans=active_snapshot,
                    airspace=airspace_snapshot,
                )
            )

            self._validate_strategic_updates(
                updates,
                inactive_snapshot,
                active_snapshot,
            )

            plan_version = self._apply_decision(
                requesting_uav=requesting_uav,
                proposed_flight_plan=proposed_flight_plan,
                decision=decision,
                counter_proposal=counter_proposal,
                updates=updates,
            )

        yield self._send_response(
            proposal=proposal,
            decision=decision,
            counter_proposal=counter_proposal,
            plan_version=plan_version,
            update_count=len(updates),
        )

    def _validate_strategic_updates(
        self,
        updates: list[FlightPlanUpdate],
        inactive_flight_plans: dict[str, FlightPlan],
        active_flight_plans: dict[str, FlightPlan],
    ) -> None:
        if any(
            not isinstance(update, FlightPlanUpdate)
            for update in updates
        ):
            raise TypeError(
                "strategic updates must be FlightPlanUpdate instances"
            )
        active_targets = sorted({
            update.uav_name
            for update in updates
            if (
                update.uav_name in active_flight_plans
                or self.flight_plan_manager.is_active(update.uav_name)
            )
        })
        if active_targets:
            self.network.record_event(
                component=self.name,
                event="INVALID_ACTIVE_FLIGHT_STRATEGIC_UPDATE",
                target_uavs=active_targets,
            )
            raise ValueError(
                "strategic deconfliction cannot update active flight plans: "
                f"{active_targets}"
            )
        ineligible_targets = sorted({
            update.uav_name
            for update in updates
            if update.uav_name not in inactive_flight_plans
        })
        if ineligible_targets:
            raise ValueError(
                "strategic deconfliction can update only inactive approved "
                f"flight plans: {ineligible_targets}"
            )

    def _reject_active_proposal(self, proposal: Message):
        self.network.record_event(
            component=self.name,
            event="ACTIVE_FLIGHT_STRATEGIC_PROPOSAL_REJECTED",
            uav=proposal.source,
            proposal_id=proposal.message_id,
        )
        return self._send_response(
            proposal=proposal,
            decision=StrategicDecision.REJECT,
            counter_proposal=None,
            plan_version=None,
            update_count=0,
        )

    def _send_response(
        self,
        proposal: Message,
        decision: StrategicDecision,
        counter_proposal: FlightPlan | None,
        plan_version: int | None,
        update_count: int,
    ):
        self.network.record_event(
            component=self.name,
            event="STRATEGIC_DECISION_MADE",
            uav=proposal.source,
            decision=decision.value,
            update_count=update_count,
        )
        response = Message(
            source=self.name,
            destination=proposal.source,
            message_type=MessageType.SD_RESPONSE,
            payload={
                "decision": decision,
                "counter_proposal": counter_proposal,
                "plan_version": plan_version,
            },
            issue_timestamp=self.env.now,
            in_reply_to=proposal.message_id,
        )
        return self.network.send(response)

    def _processing_context(
        self,
        proposal: Message,
        operation: str,
    ) -> dict[str, object]:
        flight_plan = proposal.payload["flight_plan"]
        airspace = self.flight_plan_manager.get_airspace_snapshot()
        return {
            "module": self.name,
            "operation": operation,
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
