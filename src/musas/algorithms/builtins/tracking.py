from __future__ import annotations

from collections.abc import Sequence

from musas.algorithms.builtins._trajectory import position_at, velocity_at
from musas.algorithms.interfaces.tracking import TrackingAlgorithm
from musas.domain.flight import FlightPlan
from musas.domain.tracking import PredictedState, TrackingReport, TrackPrediction


class ConstantVelocityTracking(TrackingAlgorithm):
    """Predict future states by maintaining the observed velocity constant."""

    def predict(
        self,
        uav_name: str,
        report_history: Sequence[TrackingReport],
        flight_plan: FlightPlan,
        prediction_start: float,
        prediction_horizon: int,
        prediction_step: float,
    ) -> TrackPrediction:
        del flight_plan
        reports = _validated_reports(
            report_history,
            prediction_horizon,
            prediction_step,
        )
        latest = reports[-1]
        velocity = _observed_velocity(reports)
        estimated_position = _propagate(
            latest.position,
            velocity,
            max(0.0, prediction_start - latest.timestamp),
        )

        return _constant_velocity_prediction(
            uav_name=uav_name,
            position=estimated_position,
            velocity=velocity,
            prediction_start=prediction_start,
            prediction_horizon=prediction_horizon,
            prediction_step=prediction_step,
        )


class AlphaBetaTracking(TrackingAlgorithm):
    """Estimate position and velocity with a classical alpha-beta filter.

    ``alpha`` controls how strongly the estimated position follows each new
    position measurement. ``beta`` controls the corresponding velocity
    correction. The filtered state is propagated with constant velocity over
    the prediction horizon.
    """

    def __init__(self, alpha: float = 0.85, beta: float = 0.10) -> None:
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be between zero and one")
        if not 0.0 <= beta <= 1.0:
            raise ValueError("beta must be between zero and one")
        self.alpha = float(alpha)
        self.beta = float(beta)

    def predict(
        self,
        uav_name: str,
        report_history: Sequence[TrackingReport],
        flight_plan: FlightPlan,
        prediction_start: float,
        prediction_horizon: int,
        prediction_step: float,
    ) -> TrackPrediction:
        del flight_plan
        reports = _validated_reports(
            report_history,
            prediction_horizon,
            prediction_step,
        )

        position = _vector(reports[0].position)
        velocity = _vector(reports[0].velocity)
        state_timestamp = float(reports[0].timestamp)

        for report in reports[1:]:
            elapsed = float(report.timestamp) - state_timestamp
            if elapsed < 0.0:
                continue

            predicted_position = _propagate(position, velocity, elapsed)
            residual = [
                measured - predicted
                for measured, predicted in zip(
                    _vector(report.position),
                    predicted_position,
                )
            ]
            position = [
                predicted + self.alpha * error
                for predicted, error in zip(predicted_position, residual)
            ]
            if elapsed > 0.0:
                velocity = [
                    component + self.beta * error / elapsed
                    for component, error in zip(velocity, residual)
                ]
            state_timestamp = float(report.timestamp)

        estimated_position = _propagate(
            position,
            velocity,
            max(0.0, prediction_start - state_timestamp),
        )
        return _constant_velocity_prediction(
            uav_name=uav_name,
            position=estimated_position,
            velocity=velocity,
            prediction_start=prediction_start,
            prediction_horizon=prediction_horizon,
            prediction_step=prediction_step,
        )


class FlightPlanAwareTracking(TrackingAlgorithm):
    """Combine the planned trajectory with observed position and velocity.

    The flight plan supplies upcoming turns and altitude changes. The current
    position offset and the observed velocity error are carried into the
    prediction, allowing disturbances such as wind to influence the track.
    """

    def predict(
        self,
        uav_name: str,
        report_history: Sequence[TrackingReport],
        flight_plan: FlightPlan,
        prediction_start: float,
        prediction_horizon: int,
        prediction_step: float,
    ) -> TrackPrediction:
        reports = _validated_reports(
            report_history,
            prediction_horizon,
            prediction_step,
        )
        if not flight_plan.waypoint_list:
            raise ValueError("flight plan must contain at least one waypoint")

        latest = reports[-1]
        observed_velocity = _observed_velocity(reports)
        estimated_position = _propagate(
            latest.position,
            observed_velocity,
            max(0.0, prediction_start - latest.timestamp),
        )
        nominal_position = position_at(flight_plan, prediction_start)
        nominal_velocity = velocity_at(flight_plan, prediction_start)
        position_error = [
            estimated - nominal
            for estimated, nominal in zip(estimated_position, nominal_position)
        ]
        velocity_error = [
            observed - nominal
            for observed, nominal in zip(
                observed_velocity,
                nominal_velocity,
            )
        ]

        predicted_states: list[PredictedState] = []
        for step in range(1, prediction_horizon + 1):
            elapsed = prediction_step * step
            timestamp = prediction_start + elapsed
            planned_position = position_at(flight_plan, timestamp)
            planned_velocity = velocity_at(flight_plan, timestamp)
            predicted_states.append(
                PredictedState(
                    position=[
                        planned + offset + drift * elapsed
                        for planned, offset, drift in zip(
                            planned_position,
                            position_error,
                            velocity_error,
                        )
                    ],
                    velocity=[
                        planned + drift
                        for planned, drift in zip(
                            planned_velocity,
                            velocity_error,
                        )
                    ],
                    timestamp=timestamp,
                )
            )

        return TrackPrediction(
            uav_name=uav_name,
            estimated_state=PredictedState(
                position=estimated_position,
                velocity=observed_velocity,
                timestamp=prediction_start,
            ),
            predicted_states=tuple(predicted_states),
            generated_at=prediction_start,
        )


def _constant_velocity_prediction(
    uav_name: str,
    position: Sequence[float],
    velocity: Sequence[float],
    prediction_start: float,
    prediction_horizon: int,
    prediction_step: float,
) -> TrackPrediction:
    estimated_position = _vector(position)
    estimated_velocity = _vector(velocity)
    predicted_states = tuple(
        PredictedState(
            position=_propagate(
                estimated_position,
                estimated_velocity,
                prediction_step * step,
            ),
            velocity=estimated_velocity.copy(),
            timestamp=prediction_start + prediction_step * step,
        )
        for step in range(1, prediction_horizon + 1)
    )
    return TrackPrediction(
        uav_name=uav_name,
        estimated_state=PredictedState(
            position=estimated_position,
            velocity=estimated_velocity,
            timestamp=prediction_start,
        ),
        predicted_states=predicted_states,
        generated_at=prediction_start,
    )


def _observed_velocity(
    report_history: Sequence[TrackingReport],
) -> list[float]:
    latest = report_history[-1]
    if len(report_history) < 2:
        return _vector(latest.velocity)

    previous = report_history[-2]
    elapsed = float(latest.timestamp) - float(previous.timestamp)
    if elapsed <= 0.0:
        return _vector(latest.velocity)
    return [
        (current - past) / elapsed
        for current, past in zip(
            _vector(latest.position),
            _vector(previous.position),
        )
    ]


def _propagate(
    position: Sequence[float],
    velocity: Sequence[float],
    elapsed: float,
) -> list[float]:
    return [
        coordinate + component * elapsed
        for coordinate, component in zip(
            _vector(position),
            _vector(velocity),
        )
    ]


def _vector(values: Sequence[float]) -> list[float]:
    return [float(value) for value in values]


def _validated_reports(
    report_history: Sequence[TrackingReport],
    prediction_horizon: int,
    prediction_step: float,
) -> tuple[TrackingReport, ...]:
    if not report_history:
        raise ValueError("report_history cannot be empty")
    if prediction_horizon <= 0:
        raise ValueError("prediction_horizon must be greater than zero")
    if prediction_step <= 0.0:
        raise ValueError("prediction_step must be greater than zero")

    reports = tuple(sorted(report_history, key=lambda report: report.timestamp))
    dimensions = {
        len(values)
        for report in reports
        for values in (report.position, report.velocity)
    }
    if len(dimensions) != 1 or next(iter(dimensions)) == 0:
        raise ValueError("report positions and velocities must share a dimension")
    return reports
