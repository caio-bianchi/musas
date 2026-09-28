from typing import Protocol

import simpy

from src.core.message import Message


class CommunicationNode(Protocol):
    """Contract for nodes connected to the communication network."""

    name: str
    inbox: simpy.Store


class CommunicationNetwork:
    def __init__(
        self,
        env: simpy.Environment,
        delay: float = 0.0,
    ) -> None:
        self.env = env
        self.delay = float(delay)
        self.nodes: dict[str, CommunicationNode] = {}

    def register(self, node: CommunicationNode) -> None:
        """
        Enrolls a communication node into the network.

        A communication node may represent a UAV or a UTM module,
        such as a deconfliction module or tracking module.
        """
        self.nodes[node.name] = node

    def send(self, message: Message) -> simpy.events.Process:
        """
        Sends a message through the network.

        The message source and destination are defined in the
        message object.
        """
        return self.env.process(self._transmit(message))

    def _transmit(self, message: Message):
        

        yield self.env.timeout(self.delay)

        destination = self.nodes[message.destination]

        # Every node has an inbox attribute for message passing.
        yield destination.inbox.put(message)