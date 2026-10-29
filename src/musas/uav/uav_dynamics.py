class UAVDynamics:
    """
    Stores and updates the physical state of one UAV.
    """

    def __init__(
        self,
        initial_position,
        algorithm,
        environmental_velocity_provider=None,
    ):
        self.algorithm = algorithm
        self.environmental_velocity_provider = environmental_velocity_provider

        self.position = initial_position
        self.air_velocity = [0.0, 0.0, 0.0]
        self.environmental_velocity = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.commanded_velocity = [0.0, 0.0, 0.0]

    def update(self, commanded_velocity, dt: float, current_time=0.0):
        """
        Advances the physical state by dt seconds.
        """
        self.commanded_velocity = commanded_velocity

        new_position, new_velocity = (
            self.algorithm.step(
                position=self.position,
                velocity=self.air_velocity,
                commanded_velocity=(
                    self.commanded_velocity
                ),
                dt=dt,
            )
        )

        if self.environmental_velocity_provider is None:
            environmental_velocity = [0.0, 0.0, 0.0]
        else:
            environmental_velocity = list(
                self.environmental_velocity_provider(
                    self.position,
                    current_time
                )
            )

        self.position = [
            position + disturbance * dt
            for position, disturbance in zip(
                new_position,
                environmental_velocity,
            )
        ]
        self.air_velocity = new_velocity
        self.environmental_velocity = environmental_velocity
        self.velocity = [
            air + disturbance
            for air, disturbance in zip(
                self.air_velocity,
                self.environmental_velocity,
            )
        ]

    def reset(self, position, velocity=None):
        """
        Resets the UAV physical state.
        """
        self.position = position

        if velocity is None:
            velocity = [0.0, 0.0, 0.0]

        self.air_velocity = list(velocity)
        self.environmental_velocity = [0.0, 0.0, 0.0]
        self.velocity = list(velocity)

        self.commanded_velocity = [0.0, 0.0, 0.0]

    def stop(self):
        """Stop all motion without changing the current position."""
        self.air_velocity = [0.0, 0.0, 0.0]
        self.environmental_velocity = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.commanded_velocity = [0.0, 0.0, 0.0]
