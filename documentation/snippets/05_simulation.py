QUICKSTART_DATASET = OUTPUTS / "quickstart.sqlite"

simulation = Simulation(
    airspace=airspace,
    strategic_conflict_prediction_algorithm=strategic_predictor,
    strategic_deconfliction_algorithm=strategic_deconfliction,
    dataset_path=QUICKSTART_DATASET,
    overwrite_dataset=True,
)

for name, flight_plan in flight_plans.items():
    simulation.add_uav(
        name=name,
        flight_plan=flight_plan,
        dynamics_algorithm=FirstOrderDynamics(
            max_acceleration=10.0, velocity_time_constant=0.3,
        ),
        guidance_algorithm=WaypointGuidance(max_speed=15.0),
        sensor_model=PerfectSensor(),
        waypoint_acceptance_radius=2.0,
    )
