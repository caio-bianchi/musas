from __future__ import annotations

from typing import Sequence

from musas.algorithms.builtins._trajectory import position_at, velocity_at
from musas.algorithms.interfaces.tracking import TrackingAlgorithm
from musas.domain.flight import FlightPlan
from musas.domain.tracking import (
    PredictedState,
    TrackingReport,
    TrackPrediction,
)


def _estimated_velocity(
    report_history: Sequence[TrackingReport],
) -> list[float]:
    latest = report_history[-1]
    if len(report_history) < 2:
        return list(latest.velocity)

    previous = report_history[-2]
    elapsed = latest.timestamp - previous.timestamp
    if elapsed <= 0:
        return list(latest.velocity)
    return [
        (current - past) / elapsed
        for current, past in zip(latest.position, previous.position)
    ]


class ConstantVelocityTracking(TrackingAlgorithm):
    """Propagate the latest measured state with constant velocity."""

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
        _validate_prediction_inputs(
            report_history,
            prediction_horizon,
            prediction_step,
        )
        latest = report_history[-1]
        velocity = _estimated_velocity(report_history)
        elapsed = max(0.0, prediction_start - latest.timestamp)
        estimated_position = [
            position + component * elapsed
            for position, component in zip(latest.position, velocity)
        ]
        predicted_states = tuple(
            PredictedState(
                position=[
                    position + component * prediction_step * step
                    for position, component in zip(
                        estimated_position,
                        velocity,
                    )
                ],
                velocity=velocity.copy(),
                timestamp=prediction_start + prediction_step * step,
            )
            for step in range(1, prediction_horizon + 1)
        )
        return TrackPrediction(
            uav_name=uav_name,
            estimated_state=PredictedState(
                position=estimated_position,
                velocity=velocity,
                timestamp=prediction_start,
            ),
            predicted_states=predicted_states,
            generated_at=prediction_start,
        )


class FlightPlanAwareTracking(TrackingAlgorithm):
    """Follow the nominal plan while preserving observed motion errors.

    The future flight-plan shape supplies turns and altitude changes. Current
    position and velocity errors are added to that nominal trajectory, so
    disturbances such as wind are not immediately discarded.
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
        _validate_prediction_inputs(
            report_history,
            prediction_horizon,
            prediction_step,
        )
        if not flight_plan.waypoint_list:
            raise ValueError("flight plan must contain at least one waypoint")

        latest = report_history[-1]
        observed_velocity = _estimated_velocity(report_history)
        report_age = max(0.0, prediction_start - latest.timestamp)
        estimated_position = [
            position + component * report_age
            for position, component in zip(
                latest.position,
                observed_velocity,
            )
        ]
        nominal_position = position_at(flight_plan, prediction_start)
        nominal_velocity = velocity_at(flight_plan, prediction_start)
        position_error = [
            estimated - nominal
            for estimated, nominal in zip(
                estimated_position,
                nominal_position,
            )
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
            predicted_states.append(PredictedState(
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
            ))

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


def _validate_prediction_inputs(
    report_history: Sequence[TrackingReport],
    prediction_horizon: int,
    prediction_step: float,
) -> None:
    if not report_history:
        raise ValueError("report_history cannot be empty")
    if prediction_horizon <= 0:
        raise ValueError("prediction_horizon must be greater than zero")
    if prediction_step <= 0:
        raise ValueError("prediction_step must be greater than zero")
