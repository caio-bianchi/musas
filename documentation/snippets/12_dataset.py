with SimulationDataset(QUICKSTART_DATASET) as dataset:
    print("UAVs:", dataset.uav_names)
    print("Run status:", dataset.run_status)

    # Strategic decisions taken for each proposal
    for event in dataset.get_events("STRATEGIC_DECISION_MADE"):
        print(f"t = {event.timestamp:.1f} s", event.details["summary"])

    # Recorded true states of one UAV
    samples = dataset.get_trajectory("UAV B", start_time=10.0, end_time=11.0)
    for sample in samples:
        print(f"t = {sample.timestamp:.1f} s",
              [round(value, 1) for value in sample.position])
