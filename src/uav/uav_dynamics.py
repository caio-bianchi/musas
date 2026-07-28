class UAVDynamics:
    """
    Stores and updates the physical state of one UAV.
    """

    def __init__(self, initial_position, algorithm):
        self.algorithm = algorithm

        self.position = initial_position
        self.velocity = [0.0, 0.0, 0.0]
        self.commanded_velocity = [0.0, 0.0, 0.0]

    def update(self, commanded_velocity, dt: float):
        """
        Advances the physical state by dt seconds.
        """
        self.commanded_velocity = commanded_velocity

        new_position, new_velocity = (
            self.algorithm.step(
                position=self.position,
                velocity=self.velocity,
                commanded_velocity=(
                    self.commanded_velocity
                ),
                dt=dt,
            )
        )

        self.position = new_position
        self.velocity = new_velocity

    def reset(self, position, velocity=None):
        """
        Resets the UAV physical state.
        """
        self.position = position

        if velocity is None:
            self.velocity = [0.0, 0.0, 0.0]
        else:
            self.velocity = velocity

        self.commanded_velocity = [0.0, 0.0, 0.0]