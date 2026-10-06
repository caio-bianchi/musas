from typing import Protocol


class SensorModel(Protocol):
    """
    Contract for models that generate UAV sensor measurements.
    """

    def measure(
        self,
        position: list[float],
        velocity: list[float],
    ) -> dict[str, list[float]]:
        """
        Generate measured position and velocity values.
        """
        ...