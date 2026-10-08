from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SimulationEvent:
    """Structured event emitted while a simulation is running."""

    timestamp: float
    component: str
    event: str
    details: dict[str, Any]
