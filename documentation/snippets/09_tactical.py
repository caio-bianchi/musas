from musas.airspace import Geofence
from musas.algorithms import (
    FlightPlanAwareTracking,
    HybridGeometricManeuver,
    MinimumSeparationTacticalConflictPrediction,
)

TACTICAL_DATASET = OUTPUTS / "tactical.sqlite"

tactical_predictor = MinimumSeparationTacticalConflictPrediction(
    minimum_separation_distance=30.0, maximum_time_difference=0.25,
)
tactical_simulation = Simulation(
    airspace=Airspace(),
    # no strategic layer: both strategic algorithms are omitted
    tracking_algorithm=FlightPlanAwareTracking(),
    tactical_conflict_prediction_algorithm=tactical_predictor,
    tactical_deconfliction_algorithm=HybridGeometricManeuver(
        conflict_prediction_algorithm=tactical_predictor,
        minimum_separation_distance=30.0,
        airspace_clearance=15.0,
        maneuver_duration=4.0,
        maximum_speed=15.0,
        cooldown=2.0,
    ),
    report_interval=0.25,
    prediction_horizon=24,          # 24 states x 0.25 s = 6 s ahead
    prediction_step=0.25,
    tactical_assessment_interval=0.25,
    dataset_path=TACTICAL_DATASET,
    overwrite_dataset=True,
)

# A no-fly zone appears 40 m ahead of the UAV while it is flying.
tactical_simulation.schedule_geofence(
    Geofence(
        geofence_id="temporary-area",
        geometry=Cylinder3D(20.0, 0.0, 20.0, 0.0, 200.0),
    ),
    activation_time=10.0,
)
tactical_simulation.add_uav(
    name="UAV C",
    flight_plan=FlightPlan([
        Waypoint(-120.0, 0.0, 100.0, 2.0),
        Waypoint(120.0, 0.0, 100.0, 26.0),
    ]),
    dynamics_algorithm=FirstOrderDynamics(
        max_acceleration=100.0, velocity_time_constant=0.0,
    ),
    guidance_algorithm=WaypointGuidance(max_speed=15.0),
    waypoint_acceptance_radius=2.0,
)
tactical_result = tactical_simulation.run(until=45.0)
tactical_simulation.close()
