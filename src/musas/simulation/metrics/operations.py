from __future__ import annotations

from musas.simulation.dataset import SimulationDataset


def number_of_flights(dataset: SimulationDataset) -> int:
    """Return the number of UAV operations stored in the dataset."""
    return len(dataset.uav_names)


def number_of_rejected_flights(dataset: SimulationDataset) -> int:
    """Count UAVs whose final state is FLIGHT_PLAN_REJECTED."""
    return sum(
        uav.final_status == "FLIGHT_PLAN_REJECTED"
        for uav in dataset.get_uavs()
    )


def number_of_detected_strategic_conflicts(dataset: SimulationDataset) -> int:
    """Sum conflicts reported by strategic prediction calls."""
    return sum(
        int(event.details.get("conflict_count", 0))
        for event in dataset.get_events("CONFLICT_PREDICTION_COMPLETED")
    )


def number_of_detected_tactical_conflicts(dataset: SimulationDataset) -> int:
    """Sum conflicts reported by tactical prediction calls.

    The same physical encounter may be reported by more than one prediction
    cycle because conflicts do not yet have persistent identifiers.
    """
    return sum(
        int(event.details.get("conflict_count", 0))
        for event in dataset.get_events("TACTICAL_CONFLICTS_DETECTED")
    )


def number_of_tactical_interventions(dataset: SimulationDataset) -> int:
    """Count tactical cycles that issued at least one flight-plan update."""
    return len(dataset.get_events("TACTICAL_UPDATES_ISSUED"))
