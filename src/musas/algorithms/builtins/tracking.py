from __future__ import annotations

from collections.abc import Sequence
from math import sqrt

from musas.algorithms.builtins._trajectory import position_at, velocity_at
from musas.algorithms.interfaces.tracking import TrackingAlgorithm
from musas.domain.flight import FlightPlan
from musas.domain.tracking import PredictedState, TrackingReport, TrackPrediction


class FlightPlanAwareTracking(TrackingAlgorithm):
    """Combine the planned trajectory with observed position and velocity.

    The flight plan supplies upcoming turns and altitude changes. The current
    position offset and the observed velocity error are carried into the
    prediction, allowing disturbances such as wind to influence the track.
    If an active flight exceeds its planned arrival time, the prediction
    continues toward the final waypoint using its observed speed.

    The prediction ends at the predicted arrival. A UAV that reaches its
    destination completes its flight and leaves the airspace, so it is not
    represented as hovering there for the rest of the horizon. This matches
    the strategic layer, which treats a flight plan as ending at its final
    waypoint, and the safety metrics, which only consider active UAVs. The
    returned track may therefore contain fewer than ``prediction_horizon``
    predicted states, or none when the UAV is already at its destination.
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
        planned_end = float(flight_plan.waypoint_list[-1].t)
        destination = _vector(flight_plan.waypoint_list[-1].position())
        final_segment_start = float(
            flight_plan.waypoint_list[-2].t
            if len(flight_plan.waypoint_list) > 1
            else flight_plan.waypoint_list[-1].t
        )
        if prediction_start < planned_end:
            elapsed_to_planned_end = planned_end - prediction_start
            position_at_planned_end = [
                planned + offset + drift * elapsed_to_planned_end
                for planned, offset, drift in zip(
                    position_at(flight_plan, planned_end),
                    position_error,
                    velocity_error,
                )
            ]
        else:
            position_at_planned_end = estimated_position

        observed_speed = sqrt(sum(value * value for value in observed_velocity))
        previous_position = _vector(estimated_position)
        destination_reached = _has_reached_destination(
            previous_position,
            previous_position,
            destination,
        )
        for step in range(1, prediction_horizon + 1):
            if destination_reached:
                break

            elapsed = prediction_step * step
            timestamp = prediction_start + elapsed

            if timestamp <= planned_end:
                planned_position = position_at(flight_plan, timestamp)
                planned_velocity = velocity_at(flight_plan, timestamp)
                predicted_position = [
                    planned + offset + drift * elapsed
                    for planned, offset, drift in zip(
                        planned_position,
                        position_error,
                        velocity_error,
                    )
                ]
                predicted_velocity = [
                    planned + drift
                    for planned, drift in zip(
                        planned_velocity,
                        velocity_error,
                    )
                ]
                if (
                    timestamp >= final_segment_start
                    and _has_reached_destination(
                        previous_position,
                        predicted_position,
                        destination,
                    )
                ):
                    predicted_position = destination.copy()
                    predicted_velocity = [0.0 for _ in destination]
                    destination_reached = True
            else:
                post_plan_elapsed = timestamp - max(
                    prediction_start,
                    planned_end,
                )
                predicted_position, predicted_velocity = _advance_toward(
                    position_at_planned_end,
                    destination,
                    observed_speed * post_plan_elapsed,
                    observed_speed,
                )
                destination_reached = predicted_position == destination

            predicted_states.append(
                PredictedState(
                    position=predicted_position,
                    velocity=predicted_velocity,
                    timestamp=timestamp,
                )
            )
            previous_position = _vector(predicted_position)

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


def _observed_velocity(
    report_history: Sequence[TrackingReport],
) -> list[float]:
    """Return a recency-weighted mean of reported ground velocities.

    The UAV reports its resultant velocity, including environmental effects.
    Using these measurements avoids the sampling artefact produced by finite
    differences when report and dynamics intervals are not synchronized.
    Linear weights retain the smoothing benefit of the full history window
    while giving the freshest reports the greatest influence.
    """
    velocities = [_vector(report.velocity) for report in report_history]
    weights = tuple(range(1, len(velocities) + 1))
    total_weight = float(sum(weights))
    return [
        sum(
            weight * velocity[axis]
            for weight, velocity in zip(weights, velocities)
        )
        / total_weight
        for axis in range(len(velocities[-1]))
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


def _advance_toward(
    position: Sequence[float],
    destination: Sequence[float],
    distance_to_travel: float,
    speed: float,
) -> tuple[list[float], list[float]]:
    current = _vector(position)
    target = _vector(destination)
    displacement = [final - initial for initial, final in zip(current, target)]
    remaining = sqrt(sum(value * value for value in displacement))
    if remaining <= 1e-9 or distance_to_travel >= remaining:
        return target, [0.0 for _ in target]
    if speed <= 1e-12:
        return current, [0.0 for _ in current]

    direction = [value / remaining for value in displacement]
    return (
        [
            coordinate + direction_component * distance_to_travel
            for coordinate, direction_component in zip(current, direction)
        ],
        [direction_component * speed for direction_component in direction],
    )


def _has_reached_destination(
    start: Sequence[float],
    end: Sequence[float],
    destination: Sequence[float],
) -> bool:
    start_to_destination = [
        target - coordinate
        for coordinate, target in zip(start, destination)
    ]
    end_to_destination = [
        target - coordinate
        for coordinate, target in zip(end, destination)
    ]
    if sum(value * value for value in end_to_destination) <= 1e-18:
        return True
    movement = [final - initial for initial, final in zip(start, end)]
    return (
        sum(value * value for value in movement) > 1e-18
        and sum(
            before * after
            for before, after in zip(
                start_to_destination,
                end_to_destination,
            )
        )
        <= 0.0
    )


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
    return reports
