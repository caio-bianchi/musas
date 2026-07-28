import simpy

class TrackingModule:
    def __init__(self, name, env, network, processing_time):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        self.processing_time = processing_time
        network.register(self)

    
    def run(self):
        while True:
            msg = yield self.inbox.get()

            print(
                f"t{self.env.now}: {self.name} processing "
                f"{msg.message_type} from {msg.source}"
            )

            yield self.env.timeout(self.processing_time)

            if msg.message_type == "PERIODIC_REPORT":
                self.process_report(msg)

    def process_report(self, message):
        print(
            f"t{self.env.now}: position of "
            f"{message.source} updated"
        )
