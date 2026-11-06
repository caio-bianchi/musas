with SimulationDataset(TACTICAL_DATASET) as dataset:
    for event in dataset.get_events():
        if event.event in ("GEOFENCE_ACTIVATED", "TACTICAL_UPDATES_ISSUED"):
            print(f"t = {event.timestamp:5.2f} s  {event.event}")
    tactical_metrics = calculate_metrics(dataset, 30.0, 1.0)
print("geofence violations:", tactical_metrics["geofence_violations"])

figure = plot_dataset(
    TACTICAL_DATASET,
    config=VisualizationConfig(view="xy", show_flight_plans=True,
                               show_tracks=True, spatial_unit="m"),
    timestamp=12.0,
    output_path=FIGURES / "tactical_xy.pdf",
)
