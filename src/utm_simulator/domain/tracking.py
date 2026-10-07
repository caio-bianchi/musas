from dataclasses import dataclass


@dataclass
class TrackingReport:
    """State reported by a UAV and its communication timestamps."""

    uav_name: str
    position: list[float]
    velocity: list[float]
    timestamp: float
    received_at: float


@dataclass
class PredictedState:
    """Estimated UAV state at a particular future instant."""

    position: list[float]
    velocity: list[float]
    timestamp: float


@dataclass
class TrackPrediction:
    """Current state estimate and the following predicted states."""

    uav_name: str
    estimated_state: PredictedState
    predicted_states: tuple[PredictedState, ...]
    generated_at: float