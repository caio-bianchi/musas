import random

from musas.algorithms.interfaces.sensor_model import SensorModel


class GaussianSensor(SensorModel):
    """Add independent zero-mean Gaussian noise to position and velocity."""

    def __init__(
        self,
        position_standard_deviation=1.0,
        velocity_standard_deviation=0.1,
        seed=None,
    ):

        self.position_standard_deviation = float(
            position_standard_deviation
        )
        self.velocity_standard_deviation = float(
            velocity_standard_deviation
        )
        self.random = random.Random(seed)

    def measure(self, position, velocity) -> dict:
        return {
            "position": [
                value
                + self.random.gauss(
                    0.0,
                    self.position_standard_deviation,
                )
                for value in position
            ],
            "velocity": [
                value
                + self.random.gauss(
                    0.0,
                    self.velocity_standard_deviation,
                )
                for value in velocity
            ],
        }
