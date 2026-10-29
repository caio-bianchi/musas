from musas.utils import norm, subtract


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
        self._previous_position = None

    def load_flight_plan(self, flight_plan,):

        self.flight_plan = flight_plan

        # The UAV starts at waypoint zero and flies toward
        # waypoint one.
        self.target_waypoint_index = 1
        self._previous_position = None

        self.completed = (
            len(flight_plan.waypoint_list) <= 1
        )

    def update(self, current_position, current_time):
        """Advance flight-plan progress and return the velocity command."""
        if self.flight_plan is None or self.completed:
            return [0.0, 0.0, 0.0]

        self._advance_reached_waypoints(current_position)

        if self.completed:
            self._previous_position = list(current_position)
            return [0.0, 0.0, 0.0]

        target_waypoint = (
            self.flight_plan.waypoint_list[self.target_waypoint_index]
        )

        commanded_velocity = (
            self.algorithm
            .calculate_commanded_velocity(
                current_position=current_position,
                current_time=current_time,
                target_waypoint=target_waypoint,
            )
        )
        self._previous_position = list(current_position)
        return commanded_velocity

    def _advance_reached_waypoints(self, current_position,):
        while not self.completed:
            target_waypoint = (
                self.flight_plan.waypoint_list[
                    self.target_waypoint_index
                ]
            )

            displacement = subtract(target_waypoint.position(),current_position)

            remaining_distance = norm(displacement)

            crossed_acceptance_region = (
                self._previous_position is not None
                and self._distance_to_segment(
                    target_waypoint.position(),
                    self._previous_position,
                    current_position,
                )
                <= self.waypoint_acceptance_radius
            )

            if (
                remaining_distance > self.waypoint_acceptance_radius
                and not crossed_acceptance_region
            ):
                return

            self.target_waypoint_index += 1

            if (
                self.target_waypoint_index
                >= len(
                    self.flight_plan.waypoint_list
                )
            ):
                self.completed = True

    @staticmethod
    def _distance_to_segment(point, start, end):
        """Return the shortest distance from a point to a line segment."""
        segment = subtract(end, start)
        squared_length = sum(component ** 2 for component in segment)
        if squared_length == 0:
            return norm(subtract(point, start))

        from_start = subtract(point, start)
        fraction = sum(
            component * direction
            for component, direction in zip(from_start, segment)
        ) / squared_length
        fraction = max(0.0, min(1.0, fraction))
        closest = [
            initial + fraction * direction
            for initial, direction in zip(start, segment)
        ]
        return norm(subtract(point, closest))

    @property
    def target_waypoint(self):
        if self.flight_plan is None or self.completed:
            return None

        return self.flight_plan.waypoint_list[self.target_waypoint_index]
