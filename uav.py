import simpy
from network import CommunicationNetwork
from utils import Message, clip, norm, substract

class UAV:
    '''
        Represents a UAV.
    '''
    def __init__(self, name, env, network, position, min_speed = 0.0, max_speed = 20.0, max_acceleration = 3.0, velocity_time_constant = 1.0, waypoint_acceptance_radius = 1.0):
        self.name = name
        self.env = env
        self.network = network

        self.position = position
        self.velocity = [0.0, 0.0, 0.0]
        self.commanded_velocity = [0.0, 0.0, 0.0]

        self.min_speed = min_speed
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.velocity_time_constant = velocity_time_constant
        self.waypoint_acceptance_radius = waypoint_acceptance_radius

        self.flight_plan = None
        self.target_waypoint_index = None
        self.flight_plan_start_time = None

        self.inbox = simpy.Store(env)
        self.status = "INACTIVE"
        self.pending_proposals = {}

        network.register(self)

    
    def send_periodic_reports(self,tracking_module, update_interval):
        ''''
            Periodically send reports to the Tracking Module.
        '''
        while True:
            msg = Message(
                source=self.name,
                destination=tracking_module,
                message_type="PERIODIC_REPORT",
                payload={
                    "position": self.position.copy()
                },
                issue_timestamp=self.env.now
            )

            yield self.network.send(msg)
            yield self.env.timeout(update_interval)

    def send_flight_plan(self,strategic_deconfliction_module, flight_plan):
        ''''
            Sends flight plan for approval to the Strategic Deconfliction Module.
        '''
        proposal = Message(
                source=self.name,
                destination=strategic_deconfliction_module,
                message_type="PROPOSAL",
                payload=flight_plan,
                issue_timestamp=self.env.now,
            )

        self.pending_proposals[proposal.message_id] = {
            "flight_plan": flight_plan,
            "sent_at": self.env.now,
        }

        self.status = "WAITING_FLIGHT_PLAN"

        yield self.network.send(proposal)

    def receive_messages(self):
        while True:
            message = yield self.inbox.get()

            print(
                f"t{self.env.now}: {self.name} received "
                f"{message.message_type} from {message.source}"
            )

            self._handle_message(message)

    def _handle_message(self, message):
        return
        if message.message_type == "APPROVAL":
            self.status = ""

    def fly(self, update_interval=0.1):
        if update_interval <= 0:
            raise ValueError("update_interval must be positive")

        while True:
            if self.status == "ACTIVE":
                self.commanded_velocity = (self._calculate_commanded_velocity(update_interval))

                self._update_velocity(self.commanded_velocity,update_interval)
                
                self.position = [
                    position + velocity * update_interval
                    for position, velocity
                    in zip(self.position, self.velocity)
                ]

            yield self.env.timeout(update_interval)

    def activate_flight_plan(self, flight_plan):
        self.flight_plan = flight_plan
        self.flight_plan_start_time = self.env.now
        self.target_waypoint_index = 1

        first_waypoint = flight_plan.waypoint_list[0]
        self.position = first_waypoint.position()
        self.velocity = [0.0, 0.0, 0.0]
        self.status = "ACTIVE"

    def _calculate_commanded_velocity(self, dt):
        if self.flight_plan is None:
            return [0.0, 0.0, 0.0]

        waypoints = self.flight_plan.waypoint_list

        if self.target_waypoint_index >= len(waypoints):
            return [0.0, 0.0, 0.0] 

        target = waypoints[self.target_waypoint_index]
        displacement = substract(
            target.position(),
            self.position,
        )
        remaining_distance = norm(displacement)

        if remaining_distance <= self.waypoint_acceptance_radius:
            self._advance_waypoint()

            if self.status == "COMPLETED":
                return [0.0, 0.0, 0.0]

            return self._calculate_commanded_velocity(dt)

        direction = [
            component / remaining_distance
            for component in displacement
        ]

        target_simulation_time = (
            self.flight_plan_start_time + target.t
        )
        remaining_time = target_simulation_time - self.env.now

        if remaining_time <= 0:
            required_speed = self.max_speed
        else:
            required_speed = remaining_distance / remaining_time

        commanded_speed = clip(
            required_speed,
            self.min_speed,
            self.max_speed,
        )

        return [
            commanded_speed * component
            for component in direction
        ]

    def _advance_waypoint(self):
        self.target_waypoint_index += 1

        if self.target_waypoint_index >= len(
            self.flight_plan.waypoint_list
        ):
            self.status = "COMPLETED"
            self.commanded_velocity = [0.0, 0.0, 0.0]

            print(
                f"t{self.env.now}: {self.name} completed "
                f"the flight plan"
            )

    def _update_velocity(self, commanded_velocity, dt):
        tau = self.velocity_time_constant

        if tau <= 0:
            desired_velocity_change = [
                commanded - current
                for commanded, current
                in zip(commanded_velocity, self.velocity)
            ]
        else:
            # check for this later
            # isn't better to "alpha = 1.0 - math.exp(-dt / tau)"
            alpha = min(dt / tau, 1.0)

            desired_velocity_change = [
                alpha * (commanded - current)
                for commanded, current
                in zip(commanded_velocity, self.velocity)
            ]

        change_norm = norm(desired_velocity_change)
        maximum_change = self.max_acceleration * dt

        if change_norm > maximum_change and change_norm > 0:
            scale = maximum_change / change_norm
            desired_velocity_change = [
                component * scale
                for component in desired_velocity_change
            ]

        self.velocity = [
            current + change
            for current, change
            in zip(self.velocity, desired_velocity_change)
        ]
