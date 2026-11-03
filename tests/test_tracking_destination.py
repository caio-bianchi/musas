from musas.algorithms import FlightPlanAwareTracking
from musas.domain import FlightPlan, Waypoint
from musas.domain.tracking import TrackingReport


def test_track_stops_at_destination_after_planned_arrival() -> None:
    algorithm = FlightPlanAwareTracking()
    flight_plan = FlightPlan([
        Waypoint(0.0, 0.0, 100.0, 0.0),
        Waypoint(100.0, 0.0, 100.0, 10.0),
    ])
    reports = [
        TrackingReport(
            uav_name="UAV A",
            position=(70.0, 0.0, 100.0),
            velocity=(10.0, 0.0, 0.0),
            timestamp=8.0,
            received_at=8.0,
        )
    ]

    prediction = algorithm.predict(
        uav_name="UAV A",
        report_history=reports,
        flight_plan=flight_plan,
        prediction_start=8.0,
        prediction_horizon=8,
        prediction_step=1.0,
    )

    positions = [tuple(state.position) for state in prediction.predicted_states]
    assert positions[-1] == (100.0, 0.0, 100.0)
    assert all(position[0] <= 100.0 for position in positions)
    assert tuple(prediction.predicted_states[-1].velocity) == (0.0, 0.0, 0.0)


def test_late_track_continues_to_destination_then_stops() -> None:
    algorithm = FlightPlanAwareTracking()
    flight_plan = FlightPlan([
        Waypoint(0.0, 0.0, 100.0, 0.0),
        Waypoint(100.0, 0.0, 100.0, 10.0),
    ])
    reports = [
        TrackingReport(
            uav_name="UAV A",
            position=(80.0, 0.0, 100.0),
            velocity=(5.0, 0.0, 0.0),
            timestamp=11.0,
            received_at=11.0,
        )
    ]

    prediction = algorithm.predict(
        uav_name="UAV A",
        report_history=reports,
        flight_plan=flight_plan,
        prediction_start=11.0,
        prediction_horizon=6,
        prediction_step=1.0,
    )

    positions = [tuple(state.position) for state in prediction.predicted_states]
    assert positions[:4] == [
        (85.0, 0.0, 100.0),
        (90.0, 0.0, 100.0),
        (95.0, 0.0, 100.0),
        (100.0, 0.0, 100.0),
    ]
    assert positions[4:] == [
        (100.0, 0.0, 100.0),
        (100.0, 0.0, 100.0),
    ]
