import simpy
from src.core.network import CommunicationNetwork
from src.uav.uav import UAV
from src.core.message import Message
from src.utm_modules.conflict_predictor import ConflictPredictor

env = simpy.Environment()

network = CommunicationNetwork(env=env, delay = 2)

trackingModule = TrackingModule(
    env=env,
    name="TrackingModule",
    network=network,
    processing_time=0.5
)

uav_1 = UAV(
    env=env,
    name="UAV 1",
    network=network,
    position = [0,0,100],
)

uav_2 = UAV(
    env=env,
    name="UAV 2",
    network=network,
    position = [100,50,100]
    )



cp = ConflictPredictor(
    env=env,
    name="Conflict Predictor",
    network=network
)

sdm = StrategicDeconflictionModule(
    env=env,
    name="Strategic Deconfliction Module",
    network=network,
    conflict_predictor="Conflict Predictor"
)



# env.process(trackingModule.run())

env.process(uav_1.receive_messages())
env.process(uav_2.receive_messages())
env.process(cp.handle_request())
env.process(sdm.handle_request())
env.process(uav_1.send_flight_plan(strategic_deconfliction_module="Strategic Deconfliction Module",flight_plan="FP1"))
env.process(uav_2.send_flight_plan(strategic_deconfliction_module="Strategic Deconfliction Module",flight_plan="FP2"))

env.run(until=200)