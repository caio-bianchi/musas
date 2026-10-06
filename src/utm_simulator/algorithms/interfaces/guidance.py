from abc import ABC, abstractmethod

from utm_simulator.domain.flight import Waypoint


class GuidanceAlgorithm(ABC):
    """
    Contract for UAV guidance algorithms.
    """

    @abstractmethod
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
