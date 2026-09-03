import simpy
from enum import Enum

from src.core.message import Message
from src.uav.uav_dynamics import UAVDynamics
from src.uav.uav_guidance import UAVGuidance
from src.domain.message_features import MessageType, StrategicDecision
from src.utm_modules.strategic_deconflicition_module import StrategicDeconflictionModule

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
        name,
        env,
        network,
        position,
        dynamics_algorithm,
        guidance_algorithm,
        strategic_deconfliction_module: StrategicDeconflictionModule,
        waypoint_acceptance_radius=1.0,
        environmental_velocity_provider=None,
    ):
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

        self.strategic_deconfliction_module = strategic_deconfliction_module

        self.inbox = simpy.Store(env)

        self.status = UAVStatus.INACTIVE
        self.pending_proposals = {}

        self.network.register(self)

    def send_periodic_reports(self, tracking_module, update_interval,):
        while True:
            message = Message(
                source=self.name,
                destination=tracking_module.name,
                message_type=MessageType.PERIODIC_REPORT,
                payload={
                    "position": self.dynamics.position.copy(),
                    "velocity": self.dynamics.velocity.copy(),
                },
                issue_timestamp=self.env.now,
            )

            self.network.send(message)
            yield self.env.timeout(update_interval)

    def send_flight_plan(self, flight_plan):
        proposal = Message(
            source = self.name,
            destination = self.strategic_deconfliction_module.name,
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

        self.network.send(proposal)

    def receive_messages(self):
        while True:
            message = yield self.inbox.get()

            self._handle_message(message)

    def _handle_message(self, message):
        if message.message_type == MessageType.SD_RESPONSE:
            self._handle_SD_response(message)

        if message.message_type == MessageType.UPDATE_FLIGHT_PLAN:
            self._handle_update_flight_plan(message)

    def _handle_SD_response(self, message):
        decision = message.payload["decision"]

        if decision == StrategicDecision.APPROVE:
            pending = self.pending_proposals.get(message.in_reply_to)
            flight_plan = pending["flight_plan"]

            self.load_flight_plan(flight_plan)

        elif decision == StrategicDecision.COUNTER_PROPOSAL:
            self.load_flight_plan(message.payload["counter_proposal"])

        elif decision == StrategicDecision.REJECT:
            self.status = UAVStatus.FLIGHT_PLAN_REJECTED

        self.pending_proposals.pop(message.in_reply_to, None,)

    def _handle_update_flight_plan(self, message):
        new_flight_plan = message.payload["new_flight_plan"]
        self.load_flight_plan(new_flight_plan)

    def load_flight_plan(self, flight_plan,):
        first_waypoint = flight_plan.waypoint_list[0]
        
        self.dynamics.reset(position=first_waypoint.position())

        self.guidance.load_flight_plan(flight_plan=flight_plan)

        if self.env.now >= first_waypoint.t:
            self.status = UAVStatus.ACTIVE
            self._notify_activation()
        else:
            self.status = UAVStatus.WAITING_DEPARTURE

    def fly(self, update_interval=0.1):
        while True:
            if self.status == UAVStatus.WAITING_DEPARTURE:
                departure_time = (
                    self.guidance.flight_plan
                    .waypoint_list[0]
                    .t
                )

                if self.env.now >= departure_time:
                    self.status = UAVStatus.ACTIVE
                    self._notify_activation()

            if self.status == UAVStatus.ACTIVE:
                commanded_velocity = self.guidance.calculate_commanded_velocity(
                        current_position=self.dynamics.position,
                        current_time=self.env.now,
                    )
                
                self.dynamics.update(
                    commanded_velocity=commanded_velocity,
                    dt=update_interval,
                    current_time=self.env.now,
                )

                if self.guidance.completed:
                    self._complete_flight_plan()

            yield self.env.timeout(update_interval)

    def _complete_flight_plan(self):
        self.status = UAVStatus.COMPLETED

        message = Message(
            source=self.name,
            destination=self.strategic_deconfliction_module.name,
            message_type=MessageType.FLIGHT_COMPLETED,
            payload={},
            issue_timestamp=self.env.now,
        )

        self.network.send(message)

    def _notify_activation(self):
        message = Message(
            source=self.name,
            destination=self.strategic_deconfliction_module.name,
            message_type=MessageType.FLIGHT_STARTED,
            payload={},
            issue_timestamp=self.env.now,
        )

        self.network.send(message)
