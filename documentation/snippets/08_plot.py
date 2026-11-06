figure = plot_dataset(
    QUICKSTART_DATASET,
    config=VisualizationConfig(view="xy", show_flight_plans=True,
                               spatial_unit="m"),
    timestamp=14.0,
    output_path=FIGURES / "quickstart_xy.pdf",
)
