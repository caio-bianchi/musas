import math

from musas.algorithms.interfaces.dynamics import DynamicsAlgorithm
from musas.utils import norm


class FirstOrderDynamics(DynamicsAlgorithm):
    """
    First-order velocity response with an acceleration limit.

    This class does not store the UAV state. It only calculates
    the next position and velocity from the current state.
    """

    def __init__(self, max_acceleration: float = 3.0, velocity_time_constant: float = 1.0):
        self.max_acceleration = max_acceleration
        self.velocity_time_constant = velocity_time_constant

    def step(self, position, velocity, commanded_velocity, dt):
        """
        Calculates the next position and velocity.

        Returns:
            tuple:
                new_position, new_velocity
        """
        
        new_velocity = self._calculate_velocity(
            current_velocity=velocity,
            commanded_velocity=commanded_velocity,
            dt=dt,
        )

        new_position = [
            current_position + current_velocity * dt
            for current_position, current_velocity
            in zip(position, new_velocity)
        ]

        return new_position, new_velocity

    def _calculate_velocity(self, current_velocity, commanded_velocity, dt):
        tau = self.velocity_time_constant

        if tau <= 0:
            alpha = 1.0
        else:
            # Exact discrete response of a first-order system.
            alpha = 1.0 - math.exp(-dt / tau)

        desired_velocity_change = [
            alpha * (commanded - current)
            for commanded, current
            in zip(commanded_velocity, current_velocity)
        ]

        desired_change_norm = norm(
            desired_velocity_change
        )

        maximum_velocity_change = (
            self.max_acceleration * dt
        )

        if (
            desired_change_norm > maximum_velocity_change
            and desired_change_norm > 0
        ):
            scale = (
                maximum_velocity_change
                / desired_change_norm
            )

            desired_velocity_change = [
                component * scale
                for component in desired_velocity_change
            ]

        return [
            current + change
            for current, change
            in zip(
                current_velocity,
                desired_velocity_change,
            )
        ]
