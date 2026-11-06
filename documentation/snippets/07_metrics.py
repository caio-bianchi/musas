with SimulationDataset(QUICKSTART_DATASET) as dataset:
    metrics = calculate_metrics(
        dataset,
        minimum_separation_distance=30.0,
        collision_distance=1.0,
    )

for key in ("flights", "modified_flights", "detected_strategic_conflicts",
            "loss_of_separation_events", "collision_events",
            "minimum_pairwise_separation", "average_actual_distance"):
    value = metrics[key]
    print(f"{key:30s} {value:.1f}" if isinstance(value, float)
          else f"{key:30s} {value}")
