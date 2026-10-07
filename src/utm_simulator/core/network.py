from collections.abc import Collection
from typing import Protocol

import simpy

from utm_simulator.core.message import Message
from utm_simulator.core.simulation_event import SimulationEvent
from utm_simulator.domain.message_types import MessageType


class CommunicationNode(Protocol):
    """Contract for nodes connected to the communication network."""

    name: str
    inbox: simpy.Store


class CommunicationNetwork:
    def __init__(self, env: simpy.Environment, delay: float = 0.0, 
                 message_types_to_log: Collection[MessageType] | None = None) -> None:
        self.env = env
        self.delay = float(delay)
        self.nodes: dict[str, CommunicationNode] = {}
        self.event_history: list[SimulationEvent] = []
        self.message_types_to_log = (
            None # if is None, every message will appear in log
            if message_types_to_log is None 
            else frozenset(message_types_to_log)
        )

        self.record_event(
            component="Network",
            event="SIMULATION_STARTED",
        )

    # ** indicates method can receive arbitrary quantity of named arguments
    def record_event(self, component: str, event: str, **details: object,) -> None:
        self.event_history.append(
            SimulationEvent(
                timestamp=float(self.env.now),
                component=component,
                event=event,
                details=dict(details),
            )
        )

    def get_event_history(self) -> list[SimulationEvent]:
        """Return a copy of the structured simulation event history."""
        return list(self.event_history)

    def register(self, node: CommunicationNode) -> None:
        """
        Enrolls a communication node into the network.

        A communication node may represent a UAV or a UTM module,
        such as a deconfliction module or tracking module.
        """
        self.nodes[node.name] = node
        self.record_event(component="Network", event="NODE_REGISTERED", node=node.name)

    def send(self, message: Message) -> simpy.Process:
        """
        Sends a message through the network.

        The message source and destination are defined in the
        message object.
        """
        return self.env.process(self._transmit(message))

    def _transmit(self, message: Message):

        # is the message type in the list of messages i want to appear in log?
        should_log_this_message = self.message_types_to_log is None or message.message_type in self.message_types_to_log

        if should_log_this_message:
            self.record_event(
                component="Network",
                event="MESSAGE_SENT",
                source=message.source,
                destination=message.destination,
                message_type=message.message_type.name,
                message_id=message.message_id,
            )

        yield self.env.timeout(self.delay)

        destination = self.nodes[message.destination]

        # Every node has an inbox attribute for message passing.
        yield destination.inbox.put(message)

        if should_log_this_message:
            self.record_event(
                component="Network",
                event="MESSAGE_DELIVERED",
                source=message.source,
                destination=message.destination,
                message_type=message.message_type.name,
                message_id=message.message_id,
            )
