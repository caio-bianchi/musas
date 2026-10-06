from utm_simulator.algorithms.interfaces.sensor_model import SensorModel


class PerfectSensor(SensorModel):
    """Return the true UAV state without measurement errors."""

    def measure(self, position, velocity) -> dict:
        return {
            "position": list(position),
            "velocity": list(velocity),
        }
