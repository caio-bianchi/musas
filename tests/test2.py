"""Compare a planned route with the trajectory produced by UAV dynamics.

This example intentionally does not create a SimPy environment, communication
network, or UAV node.  It drives ``UAVGuidance`` and ``UAVDynamics`` directly,
which keeps the test focused on the simulation update that was decoupled from
the node.
"""

import argparse

import matplotlib.pyplot as plt

from musas.algorithms.builtins.first_order_dynamics import (
    FirstOrderDynamics,
)
from musas.algorithms.builtins.waypoint_guidance import (
    WaypointGuidance,
)
from musas.domain.flight import FlightPlan, Waypoint
from musas.uav.uav_dynamics import UAVDynamics
from musas.uav.uav_guidance import UAVGuidance


UPDATE_INTERVAL = 0.1
MAX_SIMULATION_TIME = 70.0


def build_flight_plan():
    return FlightPlan(
        [
            Waypoint(x=0, y=0, z=100, t=0),
            Waypoint(x=50, y=0, z=100, t=5),
            Waypoint(x=100, y=0, z=100, t=10),
            Waypoint(x=150, y=50, z=110, t=17),
            Waypoint(x=200, y=100, z=120, t=25),
            Waypoint(x=250, y=150, z=120, t=32),
            Waypoint(x=300, y=200, z=110, t=40),
            Waypoint(x=350, y=200, z=100, t=48),
        ]
    )


def simulate(flight_plan, dt=UPDATE_INTERVAL):
    """Run guidance and dynamics directly and return the executed trajectory."""
    first_position = flight_plan.waypoint_list[0].position()
    dynamics = UAVDynamics(
        initial_position=first_position,
        algorithm=FirstOrderDynamics(
            max_acceleration=3.0,
            velocity_time_constant=2.0,
        ),
    )
    guidance = UAVGuidance(
        algorithm=WaypointGuidance(
            min_speed=0.0,
            max_speed=20.0,
        ),
        waypoint_acceptance_radius=2.0,
    )
    guidance.load_flight_plan(flight_plan)

    trajectory = [dynamics.position.copy()]
    simulation_time = 0.0

    while not guidance.completed and simulation_time < MAX_SIMULATION_TIME:
        commanded_velocity = guidance.update(
            current_position=dynamics.position,
            current_time=simulation_time,
        )
        if guidance.completed:
            break

        dynamics.update(commanded_velocity, dt)
        trajectory.append(dynamics.position.copy())
        simulation_time += dt

    if not guidance.completed:
        raise RuntimeError(
            f"flight plan was not completed in {MAX_SIMULATION_TIME:.1f} seconds"
        )

    return trajectory, simulation_time


def plot_comparison(flight_plan, simulated_trajectory, output_path):
    planned_trajectory = [
        waypoint.position() for waypoint in flight_plan.waypoint_list
    ]
    planned_x, planned_y, planned_z = zip(*planned_trajectory)
    simulated_x, simulated_y, simulated_z = zip(*simulated_trajectory)

    figure = plt.figure(figsize=(10, 7))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(
        planned_x,
        planned_y,
        planned_z,
        "o--",
        linewidth=2,
        markersize=6,
        label="Flight plan",
    )
    axis.plot(
        simulated_x,
        simulated_y,
        simulated_z,
        linewidth=2,
        label="Simulated UAV trajectory",
    )
    axis.set_title("Flight plan vs. UAV dynamics")
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.set_zlabel("z")
    axis.legend()
    axis.grid(True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    return figure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the graph without opening a window",
    )
    parser.add_argument(
        "--output",
        default="trajectory_comparison.png",
        help="path of the generated graph",
    )
    arguments = parser.parse_args()

    flight_plan = build_flight_plan()
    trajectory, completion_time = simulate(flight_plan)
    figure = plot_comparison(flight_plan, trajectory, arguments.output)

    final_waypoint = flight_plan.waypoint_list[-1].position()
    final_position = trajectory[-1]
    final_error = sum( 
        (planned - simulated) ** 2
        for planned, simulated in zip(final_waypoint, final_position)
    ) ** 0.5

    print(f"Flight completed at t={completion_time:.1f} s")
    print(f"Final waypoint error: {final_error:.2f}")
    print(f"Graph saved to {arguments.output}")

    if not arguments.no_show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
