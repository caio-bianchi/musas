from typing import Protocol


class DynamicsAlgorithm(Protocol):
    """
    Contract for UAV dynamics algorithms.
    """

    def step(
        self,
        position: list[float],
        velocity: list[float],
        commanded_velocity: list[float],
        dt: float,
    ) -> tuple[list[float], list[float]]:
        """
        Advance the UAV state by one simulation step.

        Returns the new position and velocity.
        """
        ...