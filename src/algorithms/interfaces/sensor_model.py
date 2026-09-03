from typing import Protocol


class SensorModel(Protocol):
    """Contract for models that generate UAV sensor measurements."""

    def measure(self, position, velocity) -> dict:
        ...
