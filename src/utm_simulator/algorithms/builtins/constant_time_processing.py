from dataclasses import dataclass

from utm_simulator.algorithms.interfaces.processing_time import (
    ProcessingTimeModel,
)


@dataclass(frozen=True)
class ConstantTimeProcessing(ProcessingTimeModel):
    """Return the same simulated processing delay for every operation."""

    delay: float = 0.0

    def __post_init__(self) -> None:
        if self.delay < 0:
            raise ValueError("processing time cannot be negative")

    def calculate(self, **context: object) -> float:
        return float(self.delay)
