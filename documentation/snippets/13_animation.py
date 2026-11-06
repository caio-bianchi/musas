from musas.simulation import animate_dataset

animation = animate_dataset(
    QUICKSTART_DATASET,
    config=VisualizationConfig(view="xy", show_flight_plans=True,
                               frame_step=0.5, spatial_unit="m"),
    output_path=OUTPUTS / "quickstart.gif",
    fps=10,
)
