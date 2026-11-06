airspace = Airspace()
airspace.add_obstacle(
    Obstacle(
        obstacle_id="tower",
        name="Communication tower",
        geometry=Cylinder3D(
            center_x=0.0, center_y=50.0, radius=15.0,
            minimum_z=0.0, maximum_z=150.0,
        ),
    )
)
