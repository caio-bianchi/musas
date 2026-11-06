# Waypoint(x, y, z, t): position in metres, time in seconds.
flight_plans = {
    "UAV A": FlightPlan([
        Waypoint(-100.0, 0.0, 100.0, 2.0),
        Waypoint(100.0, 0.0, 100.0, 22.0),
    ]),
    "UAV B": FlightPlan([
        Waypoint(0.0, -100.0, 100.0, 2.0),
        Waypoint(0.0, 100.0, 100.0, 22.0),
    ]),
}
