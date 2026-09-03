class PerfectSensor:
    """Return the true UAV state without measurement errors."""

    def measure(self, position, velocity) -> dict:
        return {
            "position": list(position),
            "velocity": list(velocity),
        }
