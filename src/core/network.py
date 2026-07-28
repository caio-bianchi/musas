import simpy

class CommunicationNetwork:
    def __init__(self, env, delay=0):
        self.env = env
        self.delay = delay
        self.nodes = {}

    def register(self, node):
        """
        Enrolls a communication node into the network.

        A communication node may represent a UAV or a UTM module,
        such as a deconfliction module or tracking module.
        """
        self.nodes[node.name] = node

    def send(self, message):
        """
        Sends a message through the network.

        The message source and destination are defined in the
        message object.
        """
        return self.env.process(self._transmit(message))

    def _transmit(self, message):
        print(
            f"t{self.env.now}: {message.source} sent "
            f"{message.message_type} to {message.destination} "
            f"Payload: {message.payload}"
        )

        yield self.env.timeout(self.delay)

        destination = self.nodes[message.destination]

        # Every node has an inbox attribute for message passing.
        yield destination.inbox.put(message)

        print(
            f"t{self.env.now}: {message.message_type} from "
            f"{message.source} delivered to {message.destination} "
            f"Payload: {message.payload}"
        )