from typing import Protocol

from src.domain.flight import Waypoint


class GuidanceAlgorithm(Protocol):
    """
    Contract for UAV guidance algorithms.
    """

    def calculate_commanded_velocity(
        self,
        current_position: list[float],
        current_time: float,
        target_waypoint: Waypoint,
    ) -> list[float]:
        """
        Calculate the commanded velocity toward the target waypoint.
        """
        ...