import simpy
from src.core.message import Message
from src.domain.message_features import MessageType


class FlightPlanManager:
    def __init__(self, name, env, network):
        self.name = name
        self.env = env
        self.network = network
        self._approved_flight_plans = {}
        self._active_flight_plans = {}
        self._history = {}

        self.inbox = simpy.Store(env)

        network.register(self)

    def approve(self, uav_name, flight_plan):
        self._approved_flight_plans[uav_name] = flight_plan

        if uav_name not in self._history:
            self._history[uav_name] = []

        self._history[uav_name].append(flight_plan)

    def activate(self, uav_name):
        flight_plan = self._approved_flight_plans.get(uav_name)

        if flight_plan is None:
            return None

        self._active_flight_plans[uav_name] = flight_plan
        return flight_plan

    def complete(self, uav_name):
        flight_plan = self._active_flight_plans.pop(uav_name, None)

        if flight_plan is None:
            flight_plan = self._approved_flight_plans.get(uav_name)

        self._approved_flight_plans.pop(uav_name, None)
        return flight_plan

    def get(self, uav_name):
        return self._approved_flight_plans.get(uav_name)

    def get_all(self):
        return self._approved_flight_plans.copy()

    def remove(self, uav_name):
        return self._approved_flight_plans.pop(uav_name, None)
    
    def get_history(self, uav_name):
        return list(self._history.get(uav_name, []))

    def launch_updates(self, updates):
        for uav_name, flight_plan in updates.items():
            self.approve(uav_name, flight_plan)

            if uav_name in self._active_flight_plans:
                self._active_flight_plans[uav_name] = flight_plan

            message = Message(
                source=self.name,
                destination=uav_name,
                message_type=MessageType.UPDATE_FLIGHT_PLAN,
                payload={
                    "new_flight_plan": flight_plan,
                },
                issue_timestamp=self.env.now,
            )

            self.network.send(message)
