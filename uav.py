import simpy
from network import CommunicationNetwork
from utils import Message

class UAV:
    '''
        Represents a UAV.
    '''
    def __init__(self, name, env, network, position):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        self.position = position
        
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