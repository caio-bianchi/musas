from typing import Callable

import simpy
from enum import Enum

from utm_simulator.algorithms.builtins.perfect_sensor import PerfectSensor
from utm_simulator.algorithms.interfaces.dynamics import DynamicsAlgorithm
from utm_simulator.algorithms.interfaces.guidance import GuidanceAlgorithm
from utm_simulator.algorithms.interfaces.sensor_model import SensorModel
from utm_simulator.core.message import Message
from utm_simulator.core.network import CommunicationNetwork
from utm_simulator.domain.flight import FlightPlan
from utm_simulator.domain.message_types import MessageType, StrategicDecision
from utm_simulator.uav.uav_dynamics import UAVDynamics
from utm_simulator.uav.uav_guidance import UAVGuidance
from utm_simulator.utm.strategic_deconfliction import (
    StrategicDeconflictionModule,
)
from utm_simulator.utm.tracking_module import TrackingModule


class UAVStatus(Enum):
    INACTIVE = 1
    WAITING_FLIGHT_PLAN = 2
    FLIGHT_PLAN_REJECTED = 3
    ACTIVE = 4
    COMPLETED = 5
    WAITING_DEPARTURE = 6


class UAV:
    """
    Represents a UAV node in the simulation.
    """

    def __init__(
        self,
        name: str,
        env: simpy.Environment,
        network: CommunicationNetwork,
        position: list[float],
        dynamics_algorithm: DynamicsAlgorithm,
        guidance_algorithm: GuidanceAlgorithm,
        strategic_deconfliction_module: StrategicDeconflictionModule,
        waypoint_acceptance_radius: float = 1.0,
        environmental_velocity_provider: (
            Callable | None
        ) = None,
        sensor_model: SensorModel | None = None,
    ) -> None:
        self.name = name
        self.env = env
        self.network = network

        self.dynamics = UAVDynamics(
            initial_position=position,
            algorithm=dynamics_algorithm,
            environmental_velocity_provider=environmental_velocity_provider,
        )

        self.guidance = UAVGuidance(
            algorithm=guidance_algorithm,
            waypoint_acceptance_radius=waypoint_acceptance_radius,
        )

        self.strategic_deconfliction_module = (
            strategic_deconfliction_module
        )

        self.sensor_model = (
            PerfectSensor()
            if sensor_model is None
            else sensor_model
        )

        self.inbox = simpy.Store(env)

        self.status: UAVStatus = UAVStatus.INACTIVE
        self.pending_proposals: dict[str, dict] = {}

        self.network.register(self)

    def send_periodic_reports(
        self,
        tracking_module: TrackingModule,
        update_interval: float,
    ):
        while True:
            measurement = self.sensor_model.measure(
                position=self.dynamics.position,
                velocity=self.dynamics.velocity,
            )

            message = Message(
                source=self.name,
                destination=tracking_module.name,
                message_type=MessageType.PERIODIC_REPORT,
                payload=measurement,
                issue_timestamp=self.env.now,
            )

            self.network.send(message)

            yield self.env.timeout(update_interval)

    def send_flight_plan(
        self,
        flight_plan: FlightPlan,
    ) -> None:
        proposal = Message(
            source=self.name,
            destination=self.strategic_deconfliction_module.name,
            message_type=MessageType.PROPOSAL,
            payload={
                "flight_plan": flight_plan,
            },
            issue_timestamp=self.env.now,
        )

        self.pending_proposals[proposal.message_id] = {
            "flight_plan": flight_plan,
            "sent_at": self.env.now,
        }

        self.status = UAVStatus.WAITING_FLIGHT_PLAN

        self.network.record_event(
            component=self.name,
            event="FLIGHT_PLAN_PROPOSED",
            proposal_id=proposal.message_id,
        )

        self.network.send(proposal)

    def receive_messages(self):
        while True:
            message = yield self.inbox.get()

            self._handle_message(message)

    def _handle_message(self, message: Message) -> None:
        if message.message_type == MessageType.SD_RESPONSE:
            self._handle_SD_response(message)

        if message.message_type == MessageType.UPDATE_FLIGHT_PLAN:
            self._handle_update_flight_plan(message)

    def _handle_SD_response(
        self,
        message: Message,
    ) -> None:
        decision = message.payload["decision"]

        if decision == StrategicDecision.APPROVE:
            pending = self.pending_proposals.get(
                message.in_reply_to
            )
            flight_plan = pending["flight_plan"]

            self.load_flight_plan(flight_plan)

        elif decision == StrategicDecision.COUNTER_PROPOSAL:
            self.load_flight_plan(
                message.payload["counter_proposal"]
            )

        elif decision == StrategicDecision.REJECT:
            self.status = UAVStatus.FLIGHT_PLAN_REJECTED

        self.network.record_event(
            component=self.name,
            event="STRATEGIC_RESPONSE_RECEIVED",
            decision=decision.value,
        )

        self.pending_proposals.pop(
            message.in_reply_to,
            None,
        )

    def _handle_update_flight_plan(
        self,
        message: Message,
    ) -> None:
        new_flight_plan = message.payload["new_flight_plan"]

        if self.status == UAVStatus.ACTIVE:
            # A tactical update changes the remaining route; it must not
            # reset the physical state of an aircraft already flying.
            self.guidance.load_flight_plan(new_flight_plan)
            self.network.record_event(
                component=self.name,
                event="ACTIVE_FLIGHT_PLAN_UPDATED",
            )
        else:
            self.load_flight_plan(new_flight_plan)

    def load_flight_plan(self, flight_plan: FlightPlan) -> None:
        first_waypoint = flight_plan.waypoint_list[0]

        self.dynamics.reset(
            position=first_waypoint.position()
        )

        self.guidance.load_flight_plan(
            flight_plan=flight_plan
        )

        if self.env.now >= first_waypoint.t:
            self.status = UAVStatus.ACTIVE
            self._notify_activation()
        else:
            self.status = UAVStatus.WAITING_DEPARTURE
            self.network.record_event(
                component=self.name,
                event="WAITING_FOR_DEPARTURE",
                departure_time=first_waypoint.t,
            )

    def fly(self, update_interval: float = 0.1):
        while True:
            if self.status == UAVStatus.WAITING_DEPARTURE:
                departure_time = (
                    self.guidance
                    .flight_plan
                    .waypoint_list[0]
                    .t
                )

                if self.env.now >= departure_time:
                    self.status = UAVStatus.ACTIVE
                    self._notify_activation()

            if self.status == UAVStatus.ACTIVE:
                commanded_velocity = (
                    self.guidance.update(
                        current_position=self.dynamics.position,
                        current_time=self.env.now,
                    )
                )

                self.dynamics.update(
                    commanded_velocity=commanded_velocity,
                    dt=update_interval,
                    current_time=self.env.now,
                )

                if self.guidance.completed:
                    self._complete_flight_plan()

            yield self.env.timeout(update_interval)

    def _complete_flight_plan(self) -> None:
        self.status = UAVStatus.COMPLETED
        self.network.record_event(
            component=self.name,
            event="FLIGHT_COMPLETED",
            position=self.dynamics.position,
        )

        message = Message(
            source=self.name,
            destination=self.strategic_deconfliction_module.name,
            message_type=MessageType.FLIGHT_COMPLETED,
            payload={},
            issue_timestamp=self.env.now,
        )

        self.network.send(message)

    def _notify_activation(self) -> None:
        self.network.record_event(
            component=self.name,
            event="FLIGHT_STARTED",
            position=self.dynamics.position,
        )

        message = Message(
            source=self.name,
            destination=self.strategic_deconfliction_module.name,
            message_type=MessageType.FLIGHT_STARTED,
            payload={},
            issue_timestamp=self.env.now,
        )

        self.network.send(message)
