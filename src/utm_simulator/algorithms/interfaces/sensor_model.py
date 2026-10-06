from abc import ABC, abstractmethod


class SensorModel(ABC):
    """
    Contract for models that generate UAV sensor measurements.
    """

    @abstractmethod
    def measure(
        self,
        position: list[float],
        velocity: list[float],
    ) -> dict[str, list[float]]:
        """
        Generate measured position and velocity values.
        """
        ...
