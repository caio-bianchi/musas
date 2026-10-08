from musas.algorithms.interfaces.guidance import GuidanceAlgorithm
from musas.utils import clip, norm, subtract


class WaypointGuidance(GuidanceAlgorithm):
    """
    Calculates a velocity command toward a target waypoint.

    This class does not store the flight plan or the target
    waypoint index.
    """

    def __init__(
        self,
        min_speed: float = 0.0,
        max_speed: float = 20.0,
    ):

        self.min_speed = min_speed
        self.max_speed = max_speed

    def calculate_commanded_velocity(
        self,
        current_position,
        current_time,
        target_waypoint,
    ):
        """
        Calculates the velocity required to reach the target
        waypoint at its scheduled time.
        """
        displacement = subtract(
            target_waypoint.position(),
            current_position,
        )

        remaining_distance = norm(displacement)

        if remaining_distance == 0:
            return [
                0.0
            ] * len(current_position)

        direction = [
            component / remaining_distance
            for component in displacement
        ]

        remaining_time = (
            target_waypoint.t - current_time
        )

        if remaining_time <= 0:
            required_speed = self.max_speed
        else:
            required_speed = (
                remaining_distance / remaining_time
            )

        commanded_speed = clip(
            required_speed,
            self.min_speed,
            self.max_speed,
        )

        return [
            commanded_speed * component
            for component in direction
        ]
