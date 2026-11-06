from musas.algorithms import StrategicDeconflictionAlgorithm
from musas.domain import StrategicDecision


class RejectOnConflict(StrategicDeconflictionAlgorithm):
    """Approve conflict-free proposals and reject all the others."""

    def deconflict(self, proposed_flight_plan, conflicts,
                   inactive_flight_plans, active_flight_plans, airspace):
        if conflicts:
            return StrategicDecision.REJECT, None, []
        return StrategicDecision.APPROVE, None, []


custom_simulation = Simulation(
    airspace=Airspace(),
    strategic_conflict_prediction_algorithm=strategic_predictor,
    strategic_deconfliction_algorithm=RejectOnConflict(),
)
for name, flight_plan in flight_plans.items():
    custom_simulation.add_uav(
        name=name,
        flight_plan=flight_plan,
        dynamics_algorithm=FirstOrderDynamics(),
        guidance_algorithm=WaypointGuidance(max_speed=15.0),
    )
custom_result = custom_simulation.run(until=40.0)
print({name: uav.status.name for name, uav in custom_result.uavs.items()})
