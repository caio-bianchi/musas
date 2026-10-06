class TrackingReport:
    """
    State reported by a UAV and its communication timestamps.
    """

    def __init__(
        self,
        uav_name: str,
        position: list[float],
        velocity: list[float],
        timestamp: float,
        received_at: float,
    ) -> None:
        self.uav_name: str = uav_name
        self.position: list[float] = position
        self.velocity: list[float] = velocity
        self.timestamp: float = timestamp
        self.received_at: float = received_at


class PredictedState:
    """
    Estimated UAV state at a particular future instant.
    """

    def __init__(
        self,
        position: list[float],
        velocity: list[float],
        timestamp: float,
    ) -> None:
        self.position: list[float] = position
        self.velocity: list[float] = velocity
        self.timestamp: float = timestamp


class TrackPrediction:
    """
    Current state estimate and the following predicted states.
    """

    def __init__(
        self,
        uav_name: str,
        estimated_state: PredictedState,
        predicted_states: tuple[PredictedState, ...],
        generated_at: float,
    ) -> None:
        self.uav_name: str = uav_name
        self.estimated_state: PredictedState = estimated_state
        self.predicted_states: tuple[PredictedState, ...] = predicted_states
        self.generated_at: float = generated_at