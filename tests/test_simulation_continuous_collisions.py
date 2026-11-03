from musas.airspace import Airspace, Box3D, Obstacle
from musas.algorithms import FirstOrderDynamics, WaypointGuidance
from musas.domain import FlightPlan, Waypoint
from musas.simulation import Simulation


def _add_crossing_uav(
    simulation: Simulation,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> None:
    simulation.add_uav(
        name=name,
        flight_plan=FlightPlan([
            Waypoint(*start, 0.0),
            Waypoint(*end, 1.0),
        ]),
        dynamics_algorithm=FirstOrderDynamics(
            max_acceleration=1000.0,
            velocity_time_constant=0.0,
        ),
        guidance_algorithm=WaypointGuidance(
            min_speed=0.0,
            max_speed=20.0,
        ),
        waypoint_acceptance_radius=0.1,
    )


def test_simulation_crashes_uavs_that_cross_between_updates() -> None:
    simulation = Simulation(
        airspace=Airspace(),
        flight_update_interval=1.0,
        trajectory_record_interval=1.0,
        collision_distance=0.5,
    )
    _add_crossing_uav(
        simulation,
        "A",
        (-5.0, 0.0, 100.0),
        (5.0, 0.0, 100.0),
    )
    _add_crossing_uav(
        simulation,
        "B",
        (0.0, -5.0, 100.0),
        (0.0, 5.0, 100.0),
    )

    result = simulation.run(until=4.0)
    simulation.close()

    assert all(uav.status.name == "COLLIDED" for uav in result.uavs.values())
    collisions = [
        event for event in result.event_history
        if event.event == "UAV_COLLISION"
    ]
    assert len(collisions) == 1
    assert collisions[0].details["separation"] <= 0.5 + 1e-9


def test_simulation_crashes_uav_that_crosses_obstacle_between_updates() -> None:
    airspace = Airspace()
    airspace.add_obstacle(Obstacle(
        obstacle_id="box",
        geometry=Box3D(-1.0, 1.0, -1.0, 1.0, 90.0, 110.0),
    ))
    simulation = Simulation(
        airspace=airspace,
        flight_update_interval=1.0,
        trajectory_record_interval=1.0,
        collision_distance=0.5,
    )
    _add_crossing_uav(
        simulation,
        "A",
        (-5.0, 0.0, 100.0),
        (5.0, 0.0, 100.0),
    )

    result = simulation.run(until=4.0)
    simulation.close()

    assert result.uavs["A"].status.name == "COLLIDED"
    collisions = [
        event for event in result.event_history
        if event.event == "OBSTACLE_COLLISION"
    ]
    assert len(collisions) == 1
    assert collisions[0].details["position"] == [-1.0, 0.0, 100.0]
