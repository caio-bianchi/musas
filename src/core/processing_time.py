from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Mapping


class ProcessingMode(Enum):
    """Available ways of determining a module's processing delay."""

    FIXED = "FIXED"
    MODEL = "MODEL"


ProcessingContext = Mapping[str, object]
ProcessingTimeModel = Callable[[ProcessingContext], float]


@dataclass(frozen=True)
class ProcessingTime:
    """Configuration for simulated processing delays.

    In ``FIXED`` mode, ``fixed_time`` is returned for every operation. In
    ``MODEL`` mode, the supplied function receives a module-specific context
    and returns the delay in simulation-time units.
    """

    mode: ProcessingMode = ProcessingMode.FIXED
    fixed_time: float = 0.0
    model: ProcessingTimeModel | None = None

    def __post_init__(self) -> None:
        if self.fixed_time < 0:
            raise ValueError("fixed_time cannot be negative")
        if self.mode == ProcessingMode.MODEL and self.model is None:
            raise ValueError("MODEL processing mode requires a model function")
        if self.mode == ProcessingMode.FIXED and self.model is not None:
            raise ValueError("FIXED processing mode does not accept a model")

    def calculate(self, context: ProcessingContext) -> float:
        if self.mode == ProcessingMode.FIXED:
            return float(self.fixed_time)

        delay = float(self.model(context))
        if delay < 0:
            raise ValueError("processing-time model returned a negative delay")
        return delay


def processing_time_config(
    value: float | ProcessingTime,
) -> ProcessingTime:
    """Normalize legacy numeric delays to a FIXED configuration."""
    if isinstance(value, ProcessingTime):
        return value
    return ProcessingTime(
        mode=ProcessingMode.FIXED,
        fixed_time=float(value),
    )
