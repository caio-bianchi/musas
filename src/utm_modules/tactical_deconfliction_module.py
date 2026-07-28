import simpy

class TacticalDeconflictionModule:
    def __init__(self, name, env, network):
        self.name = name
        self.env = env
        self.network = network
        self.inbox = simpy.Store(env)
        network.register(self)