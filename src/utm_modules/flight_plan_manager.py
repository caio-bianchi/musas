import simpy

from src.airspace.airspace import Airspace
from src.core.message import Message
from src.core.network import CommunicationNetwork
from src.domain.flight import FlightPlan
from src.domain.message_features import MessageType


class FlightPlanManager:
    def __init__(
        self,
        name: str,
        env: simpy.Environment,
        network: CommunicationNetwork,
        airspace: Airspace,
    ) -> None:
        self.name = name
        self.env = env
        self.network = network
        self.airspace = airspace
        self._approved_flight_plans: dict[str, FlightPlan] = {}
        self._active_flight_plans: dict[str, FlightPlan] = {}
        self._history: dict[str, list[FlightPlan]] = {}

        self.inbox = simpy.Store(env)

        network.register(self)

    def get_airspace(self) -> Airspace:
        return self.airspace

    def get_airspace_snapshot(self) -> Airspace:
        return Airspace(
            obstacles=list(self.airspace.obstacles),
            geofences=list(
                self.airspace.active_geofences_at(self.env.now)
            ),
            # Environmental disturbances are intentionally not exposed to
            # strategic algorithms through the operational snapshot.
            wind_zones=[],
        )

    def approve(self, uav_name: str, flight_plan: FlightPlan) -> None:
        self._approved_flight_plans[uav_name] = flight_plan

        if uav_name not in self._history:
            self._history[uav_name] = []

        self._history[uav_name].append(flight_plan)
        self.network.log_event(
            component=self.name,
            event="FLIGHT_PLAN_APPROVED",
            uav=uav_name,
            waypoint_count=len(flight_plan.waypoint_list),
        )

    def activate(self, uav_name: str) -> FlightPlan | None:
        flight_plan = self._approved_flight_plans.get(uav_name)

        if flight_plan is None:
            self.network.log_event(
                component=self.name,
                event="FLIGHT_PLAN_ACTIVATION_FAILED",
                uav=uav_name,
            )
            return None

        self._active_flight_plans[uav_name] = flight_plan
        self.network.log_event(
            component=self.name,
            event="FLIGHT_PLAN_ACTIVATED",
            uav=uav_name,
        )
        return flight_plan

    def complete(self, uav_name: str) -> FlightPlan | None:
        flight_plan = self._active_flight_plans.pop(uav_name, None)

        if flight_plan is None:
            flight_plan = self._approved_flight_plans.get(uav_name)

        self._approved_flight_plans.pop(uav_name, None)
        self.network.log_event(
            component=self.name,
            event="FLIGHT_PLAN_COMPLETED",
            uav=uav_name,
            found=flight_plan is not None,
        )
        return flight_plan

    def get(self, uav_name: str) -> FlightPlan | None:
        return self._approved_flight_plans.get(uav_name)

    def get_all(self) -> dict[str, FlightPlan]:
        return self._approved_flight_plans.copy()

    def remove(self, uav_name: str) -> FlightPlan | None:
        return self._approved_flight_plans.pop(uav_name, None)

    def get_history(self, uav_name: str) -> list[FlightPlan]:
        return list(self._history.get(uav_name, []))

    def launch_updates(self, updates: dict[str, FlightPlan]) -> None:
        self.network.log_event(
            component=self.name,
            event="FLIGHT_PLAN_UPDATES_LAUNCHED",
            uavs=list(updates),
        )
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
