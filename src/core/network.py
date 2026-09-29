from typing import Protocol
from pathlib import Path

import simpy

from src.core.message import Message
from src.core.simulation_logger import SimulationEvent, SimulationLogger
from src.domain.message_features import MessageType


class CommunicationNode(Protocol):
    """Contract for nodes connected to the communication network."""

    name: str
    inbox: simpy.Store


class CommunicationNetwork:
    def __init__(self,env: simpy.Environment,delay: float = 0.0,log_file: str | Path | None = None) -> None:
        self.env = env
        self.delay = float(delay)
        self.nodes: dict[str, CommunicationNode] = {}
        self.event_history: list[SimulationEvent] = []
        self.simulation_logger = (
            SimulationLogger(env, log_file)
            if log_file is not None
            else None
        )

        self.log_event(
            component="Network",
            event="SIMULATION_LOG_STARTED",
            file=log_file,
        )

    def log_event(self,component: str,event: str,**details) -> None:
        self.event_history.append(
            SimulationEvent(
                timestamp=float(self.env.now),
                component=component,
                event=event,
                details=dict(details),
            )
        )
        if self.simulation_logger is not None:
            self.simulation_logger.log(
                component=component,
                event=event,
                **details,
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
        self.log_event(
            component="Network",
            event="NODE_REGISTERED",
            node=node.name,
        )

    def send(self, message: Message) -> simpy.events.Process:
        """
        Sends a message through the network.

        The message source and destination are defined in the
        message object.
        """
        return self.env.process(self._transmit(message))

    def _transmit(self, message: Message):
        should_log_message = message.message_type not in {
            MessageType.PERIODIC_REPORT,
            MessageType.TRACK_UPDATE,
        }
        if should_log_message:
            self.log_event(
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

        if should_log_message:
            self.log_event(
                component="Network",
                event="MESSAGE_DELIVERED",
                source=message.source,
                destination=message.destination,
                message_type=message.message_type.name,
                message_id=message.message_id,
            )
