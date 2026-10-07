from utm_simulator.utils import norm, subtract


class UAVGuidance:
    """
    Stores the active flight plan and manages waypoint progress.
    """

    def __init__(self, algorithm, waypoint_acceptance_radius: float = 1.0,):
        self.algorithm = algorithm
        self.waypoint_acceptance_radius = waypoint_acceptance_radius

        self.flight_plan = None
        self.target_waypoint_index = None
        self.completed = False

    def load_flight_plan(self, flight_plan,):

        self.flight_plan = flight_plan

        # The UAV starts at waypoint zero and flies toward
        # waypoint one.
        self.target_waypoint_index = 1

        self.completed = (
            len(flight_plan.waypoint_list) <= 1
        )

    def update(self, current_position, current_time):
        """Advance flight-plan progress and return the velocity command."""
        if self.flight_plan is None or self.completed:
            return [0.0, 0.0, 0.0]

        self._advance_reached_waypoints(current_position)

        if self.completed:
            return [0.0, 0.0, 0.0]

        target_waypoint = (
            self.flight_plan.waypoint_list[self.target_waypoint_index]
        )

        return (
            self.algorithm
            .calculate_commanded_velocity(
                current_position=current_position,
                current_time=current_time,
                target_waypoint=target_waypoint,
            )
        )

    def _advance_reached_waypoints(self, current_position,):
        while not self.completed:
            target_waypoint = (
                self.flight_plan.waypoint_list[
                    self.target_waypoint_index
                ]
            )

            displacement = subtract(target_waypoint.position(),current_position)

            remaining_distance = norm(displacement)

            if remaining_distance > self.waypoint_acceptance_radius:
                return

            self.target_waypoint_index += 1

            if (
                self.target_waypoint_index
                >= len(
                    self.flight_plan.waypoint_list
                )
            ):
                self.completed = True

    @property
    def target_waypoint(self):
        if self.flight_plan is None or self.completed:
            return None

        return self.flight_plan.waypoint_list[self.target_waypoint_index]
