from collections.abc import Callable, Sequence

import simpy

from utm_simulator.airspace.airspace import Airspace
from utm_simulator.core.message import Message
from utm_simulator.core.network import CommunicationNetwork
from utm_simulator.domain.flight import FlightPlan, FlightPlanUpdate
from utm_simulator.domain.message_types import MessageType


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
        self._completion_observers: list[Callable[[str], None]] = []

        self.inbox = simpy.Store(env)

        network.register(self)

    def add_completion_observer(
        self,
        observer: Callable[[str], None],
    ) -> None:
        if observer not in self._completion_observers:
            self._completion_observers.append(observer)

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

    def approve(self, uav_name: str, flight_plan: FlightPlan) -> int:
        self._approved_flight_plans[uav_name] = flight_plan

        if uav_name not in self._history:
            self._history[uav_name] = []

        self._history[uav_name].append(flight_plan)
        plan_version = len(self._history[uav_name]) - 1
        self.network.record_event(
            component=self.name,
            event="FLIGHT_PLAN_APPROVED",
            uav=uav_name,
            plan_version=plan_version,
            waypoint_count=len(flight_plan.waypoint_list),
        )
        return plan_version

    def activate(self, uav_name: str) -> FlightPlan | None:
        flight_plan = self._approved_flight_plans.get(uav_name)

        if flight_plan is None:
            self.network.record_event(
                component=self.name,
                event="FLIGHT_PLAN_ACTIVATION_FAILED",
                uav=uav_name,
            )
            return None

        self._active_flight_plans[uav_name] = flight_plan
        self.network.record_event(
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
        self.network.record_event(
            component=self.name,
            event="FLIGHT_PLAN_COMPLETED",
            uav=uav_name,
            found=flight_plan is not None,
        )
        for observer in tuple(self._completion_observers):
            observer(uav_name)
        return flight_plan

    def get(self, uav_name: str) -> FlightPlan | None:
        return self._approved_flight_plans.get(uav_name)

    def get_all(self) -> dict[str, FlightPlan]:
        return self._approved_flight_plans.copy()

    def remove(self, uav_name: str) -> FlightPlan | None:
        return self._approved_flight_plans.pop(uav_name, None)

    def get_history(self, uav_name: str) -> list[FlightPlan]:
        return list(self._history.get(uav_name, []))

    def launch_updates(
        self,
        updates: Sequence[FlightPlanUpdate],
    ) -> None:
        if any(
            not isinstance(update, FlightPlanUpdate)
            for update in updates
        ):
            raise TypeError(
                "updates must contain only FlightPlanUpdate instances"
            )
        uav_names = [update.uav_name for update in updates]
        if len(set(uav_names)) != len(uav_names):
            raise ValueError(
                "updates cannot contain more than one plan for the same UAV"
            )
        self.network.record_event(
            component=self.name,
            event="FLIGHT_PLAN_UPDATES_LAUNCHED",
            uavs=uav_names,
        )
        for update in updates:
            uav_name = update.uav_name
            flight_plan = update.flight_plan
            plan_version = self.approve(uav_name, flight_plan)

            if uav_name in self._active_flight_plans:
                self._active_flight_plans[uav_name] = flight_plan

            message = Message(
                source=self.name,
                destination=uav_name,
                message_type=MessageType.UPDATE_FLIGHT_PLAN,
                payload={
                    "new_flight_plan": flight_plan,
                    "plan_version": plan_version,
                },
                issue_timestamp=self.env.now,
            )

            self.network.send(message)
