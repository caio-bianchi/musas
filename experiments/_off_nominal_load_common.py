from __future__ import annotations

import csv
import math
import os
import random
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from pathlib import Path
from statistics import fmean
from time import perf_counter

from musas.airspace import Airspace, Box3D, Cylinder3D, Geofence, WindZone
from musas.algorithms import (
    DetourPathPlanner,
    ExtendedHybridGeometricManeuver,
    FirstOrderDynamics,
    FlightPlanAwareTracking,
    MinimumSeparationConflictPrediction,
    MinimumSeparationTacticalConflictPrediction,
    RecursiveTimeShiftDeconfliction,
    WaypointGuidance,
)
from musas.domain import FlightPlan, Waypoint
from musas.simulation import Simulation, SimulationDataset
from musas.simulation.metrics import calculate_metrics


#N_VALUES = (5, 10, 20, 40, 60)
#K = 10

N_VALUES = (5, 10, 15, 20, 25, 30, 35, 40, 45, 50)
# K is the TOTAL number of replications per strategy and fleet size.
# Replications already saved in replication_metrics.csv are skipped, so
# raising K only runs (and appends) the missing ones. Rerunning after an
# interruption continues where the CSV stopped.
K = 800
# Completed replications are appended to the CSV in batches of SAVE_EVERY.
SAVE_EVERY = 50

# Datasets are only needed to compute each run's metrics and to draw the
# snapshot. With KEEP_DATASETS = False, only the first DATASETS_KEPT_PER_CASE
# replications of each strategy and fleet size are kept on disk.
KEEP_DATASETS = False
DATASETS_KEPT_PER_CASE = 1
SUPPORTED_STRATEGIES = ("sd_only", "td_only", "sd_td")
OFF_NOMINAL_STRATEGIES = ("sd_only", "sd_td")
STRATEGY_LABELS = {
    "sd_only": "SD only",
    "td_only": "TD only",
    "sd_td": "SD + TD",
}
BASE_SEED = 20261102

AREA_HALF_SIZE = 200.0
ALTITUDE = 100.0
EARLIEST_DEPARTURE = 2.0
LATEST_DEPARTURE = 80.0
NOMINAL_SPEED = 10.0
FLIGHT_UPDATE_INTERVAL = 0.2
SIMULATION_END = 300.0
MINIMUM_ROUTE_LENGTH = 150.0
REQUIRED_MINIMUM_SEPARATION_DISTANCE = 30.0
STRATEGIC_MINIMUM_SEPARATION_DISTANCE = 40.0
STRATEGIC_SAMPLING_INTERVAL = 0.5
TRACK_PREDICTION_HORIZON = 60
TRACK_PREDICTION_STEP = 0.5
COLLISION_DISTANCE = 1.0
TIME_SHIFT_STEP = 3.0

WIND_ACTIVATION_TIME = 25.0
GEOFENCE_ACTIVATION_TIMES = (25.0, 40.0, 55.0)

METRIC_KEYS = (
    "detected_strategic_conflicts",
    "detected_tactical_conflicts",
    "detected_conflicts",
    "flights",
    "modified_flights",
    "rejected_flights",
    "tactical_interventions",
    "loss_of_separation_events",
    "collision_events",
    "geofence_violations",
    "minimum_pairwise_separation",
    "average_planned_distance",
    "average_actual_distance",
    "average_planned_time",
    "average_actual_time",
    "average_updates_per_uav",
    "completion_rate",
    "wall_time_seconds",
)

def point_on_border(
    rng: random.Random,
    border: str,
) -> tuple[float, float]:
    coordinate = rng.uniform(-AREA_HALF_SIZE, AREA_HALF_SIZE)
    if border == "left":
        return -AREA_HALF_SIZE, coordinate
    if border == "right":
        return AREA_HALF_SIZE, coordinate
    if border == "bottom":
        return coordinate, -AREA_HALF_SIZE
    return coordinate, AREA_HALF_SIZE


def random_flight_plan(rng: random.Random) -> FlightPlan:
    borders = ("left", "right", "bottom", "top")
    while True:
        start_border = rng.choice(borders)
        end_border = rng.choice(
            [border for border in borders if border != start_border]
        )
        start = point_on_border(rng, start_border)
        end = point_on_border(rng, end_border)
        route_length = math.dist(start, end)
        if route_length < MINIMUM_ROUTE_LENGTH:
            continue

        departure_time = (
            math.ceil(
                rng.uniform(EARLIEST_DEPARTURE, LATEST_DEPARTURE)
                / FLIGHT_UPDATE_INTERVAL
            )
            * FLIGHT_UPDATE_INTERVAL
        )
        arrival_time = departure_time + route_length / NOMINAL_SPEED
        return FlightPlan([
            Waypoint(*start, ALTITUDE, departure_time),
            Waypoint(*end, ALTITUDE, arrival_time),
        ])


def create_flight_plans(
    number_of_uavs: int,
    seed: int,
) -> dict[str, FlightPlan]:
    rng = random.Random(seed)
    return {
        f"UAV {index + 1:03d}": random_flight_plan(rng)
        for index in range(number_of_uavs)
    }


def localized_wind_zones() -> tuple[WindZone, ...]:
    """Return four separated wind regions within the operating area."""
    return (
        WindZone(
            wind_id="northwest-wind",
            name="Northwest crosswind",
            geometry=Box3D(-165.0, -85.0, 55.0, 145.0, 60.0, 140.0),
            velocity=(3.5, -1.5, 0.0),
        ),
        WindZone(
            wind_id="southwest-wind",
            name="Southwest crosswind",
            geometry=Box3D(-150.0, -65.0, -155.0, -70.0, 60.0, 140.0),
            velocity=(0.5, 4.0, 0.0),
        ),
        WindZone(
            wind_id="north-central-wind",
            name="North-central crosswind",
            geometry=Box3D(-30.0, 45.0, 75.0, 160.0, 60.0, 140.0),
            velocity=(-3.5, 1.0, 0.0),
        ),
        WindZone(
            wind_id="southeast-wind",
            name="Southeast crosswind",
            geometry=Box3D(75.0, 160.0, -145.0, -55.0, 60.0, 140.0),
            velocity=(-1.0, -4.0, 0.0),
        ),
    )


def broad_wind_zones() -> tuple[WindZone, ...]:
    """Return the two broad crosswind regions used in Experiment 10."""
    return (
        WindZone(
            wind_id="west-crosswind",
            name="West crosswind",
            geometry=Box3D(
                -120.0,
                0.0,
                -AREA_HALF_SIZE,
                AREA_HALF_SIZE,
                0.0,
                200.0,
            ),
            velocity=(0.0, 4.0, 0.0),
        ),
        WindZone(
            wind_id="east-crosswind",
            name="East crosswind",
            geometry=Box3D(
                0.0,
                120.0,
                -AREA_HALF_SIZE,
                AREA_HALF_SIZE,
                0.0,
                200.0,
            ),
            velocity=(0.0, -4.0, 0.0),
        ),
    )


def dynamic_geofences() -> tuple[Geofence, ...]:
    """Return spatially separated KEEP_OUT regions activated during flight."""
    return (
        Geofence(
            geofence_id="dynamic-west",
            name="Dynamic west geofence",
            geometry=Cylinder3D(-85.0, -35.0, 32.0, 60.0, 140.0),
        ),
        Geofence(
            geofence_id="dynamic-northeast",
            name="Dynamic northeast geofence",
            geometry=Cylinder3D(70.0, 80.0, 35.0, 60.0, 140.0),
        ),
        Geofence(
            geofence_id="dynamic-south",
            name="Dynamic south geofence",
            geometry=Cylinder3D(20.0, -105.0, 30.0, 60.0, 140.0),
        ),
    )


def create_airspace(scenario: str) -> Airspace:
    """Create the initial airspace without leaking future restrictions."""
    if scenario not in {
        "nominal",
        "broad_wind",
        "geofences",
        "combined",
    }:
        raise ValueError(f"Unknown scenario: {scenario}")

    airspace = Airspace()
    if scenario == "broad_wind":
        for wind_zone in broad_wind_zones():
            airspace.add_wind_zone(wind_zone)
    return airspace


def configure_scenario(simulation: Simulation, scenario: str) -> None:
    """Schedule conditions that only become available during execution."""
    if scenario not in {
        "nominal",
        "broad_wind",
        "geofences",
        "combined",
    }:
        raise ValueError(f"Unknown scenario: {scenario}")

    if scenario == "combined":
        for wind_zone in localized_wind_zones():
            simulation.schedule_wind_zone(
                wind_zone,
                activation_time=WIND_ACTIVATION_TIME,
            )

    if scenario in {"geofences", "combined"}:
        for geofence, activation_time in zip(
            dynamic_geofences(),
            GEOFENCE_ACTIVATION_TIMES,
        ):
            simulation.schedule_geofence(
                geofence,
                activation_time=activation_time,
            )


def delete_dataset(dataset_path: Path) -> None:
    """Remove a SQLite dataset together with its WAL and shared-memory files."""
    for suffix in ("", "-wal", "-shm"):
        Path(f"{dataset_path}{suffix}").unlink(missing_ok=True)


def run_simulation(
    scenario: str,
    experiment_label: str,
    output_directory: Path,
    strategy: str,
    number_of_uavs: int,
    replication: int,
    keep_datasets: bool = KEEP_DATASETS,
) -> dict[str, float | int | str | None]:
    if strategy not in SUPPORTED_STRATEGIES:
        raise ValueError(f"Unknown strategy: {strategy}")

    seed = BASE_SEED + 1000 * number_of_uavs + replication
    dataset_directory = output_directory / "datasets"
    dataset_directory.mkdir(parents=True, exist_ok=True)
    dataset_path = (
        dataset_directory
        / f"{strategy}_n_{number_of_uavs:03d}_run_{replication + 1:02d}.sqlite"
    )

    algorithm_arguments: dict[str, object] = {}
    if strategy in {"sd_only", "sd_td"}:
        strategic_predictor = MinimumSeparationConflictPrediction(
            minimum_separation_distance=(
                STRATEGIC_MINIMUM_SEPARATION_DISTANCE
            ),
            sampling_interval=STRATEGIC_SAMPLING_INTERVAL,
        )
        algorithm_arguments.update(
            strategic_conflict_prediction_algorithm=strategic_predictor,
            strategic_deconfliction_algorithm=RecursiveTimeShiftDeconfliction(
                conflict_prediction_algorithm=strategic_predictor,
                path_planner=DetourPathPlanner(
                    conflict_prediction_algorithm=strategic_predictor,
                ),
                time_shift_step=TIME_SHIFT_STEP,
                maximum_time_shifts=30,
            ),
        )

    if strategy in {"td_only", "sd_td"}:
        tactical_predictor = MinimumSeparationTacticalConflictPrediction(
            minimum_separation_distance=(
                REQUIRED_MINIMUM_SEPARATION_DISTANCE
            ),
            maximum_time_difference=0.5,
        )
        algorithm_arguments.update(
            tracking_algorithm=FlightPlanAwareTracking(),
            tactical_conflict_prediction_algorithm=tactical_predictor,
            tactical_deconfliction_algorithm=ExtendedHybridGeometricManeuver(
                conflict_prediction_algorithm=tactical_predictor,
                minimum_separation_distance=(
                    REQUIRED_MINIMUM_SEPARATION_DISTANCE
                ),
                maximum_speed=15.0,
                cooldown=5.0,
            ),
        )

    simulation = Simulation(
        airspace=create_airspace(scenario),
        report_interval=0.5,
        history_window=6,
        prediction_horizon=TRACK_PREDICTION_HORIZON,
        prediction_step=TRACK_PREDICTION_STEP,
        tactical_assessment_interval=0.5,
        flight_update_interval=FLIGHT_UPDATE_INTERVAL,
        trajectory_record_interval=0.5,
        collision_distance=COLLISION_DISTANCE,
        message_types_to_log=(),
        dataset_path=dataset_path,
        dataset_metadata={
            "experiment": experiment_label,
            "scenario": scenario,
            "strategy": strategy,
            "number_of_uavs": number_of_uavs,
            "replication": replication + 1,
            "seed": seed,
        },
        overwrite_dataset=True,
        keep_history_in_memory=False,
        **algorithm_arguments,
    )
    configure_scenario(simulation, scenario)

    for index, (uav_name, flight_plan) in enumerate(
        create_flight_plans(number_of_uavs, seed).items()
    ):
        simulation.add_uav(
            name=uav_name,
            flight_plan=flight_plan,
            dynamics_algorithm=FirstOrderDynamics(
                max_acceleration=100.0,
                velocity_time_constant=0.0,
            ),
            guidance_algorithm=WaypointGuidance(
                min_speed=0.0,
                max_speed=15.0,
            ),
            submission_time=index * 0.01,
            waypoint_acceptance_radius=1.0,
        )

    wall_time_start = perf_counter()
    simulation.run(until=SIMULATION_END)
    simulation.close()
    wall_time_seconds = perf_counter() - wall_time_start

    metrics_read = False
    try:
        with SimulationDataset(dataset_path) as dataset:
            metrics = calculate_metrics(
                dataset,
                minimum_separation_distance=(
                    REQUIRED_MINIMUM_SEPARATION_DISTANCE
                ),
                collision_distance=COLLISION_DISTANCE,
            )
            completed_flights = sum(
                uav.final_status == "COMPLETED"
                for uav in dataset.get_uavs()
            )
        metrics_read = True
    finally:
        if not keep_datasets and (
            replication >= DATASETS_KEPT_PER_CASE or not metrics_read
        ):
            delete_dataset(dataset_path)

    metrics["average_updates_per_uav"] = fmean(
        values["updates"] for values in metrics["per_uav"].values()
    )
    metrics["detected_conflicts"] = (
        metrics["detected_strategic_conflicts"]
        + metrics["detected_tactical_conflicts"]
    )
    metrics["completion_rate"] = completed_flights / number_of_uavs
    metrics["wall_time_seconds"] = wall_time_seconds
    return {
        "strategy": strategy,
        "number_of_uavs": number_of_uavs,
        "replication": replication + 1,
        "seed": seed,
        **{key: metrics[key] for key in METRIC_KEYS},
    }


def execute_experiment(
    scenario: str,
    experiment_label: str,
    output_directory: Path,
    keep_datasets: bool = KEEP_DATASETS,
    strategies: tuple[str, ...] = OFF_NOMINAL_STRATEGIES,
    n_values: tuple[int, ...] = N_VALUES,
    replications: int = K,
) -> list[dict[str, float | int | str | None]]:
    """Run the missing replications and return replications 1..K.

    New rows are only ever appended to ``replication_metrics.csv``. They are
    written every ``SAVE_EVERY`` replications and once more when the run ends
    or is interrupted, so at most ``SAVE_EVERY - 1`` replications are lost
    if the process is killed abruptly; those are rerun on the next call.
    """
    output_directory.mkdir(parents=True, exist_ok=True)
    results_path = output_directory / "replication_metrics.csv"
    drop_incomplete_last_line(results_path)
    completed = {
        (row["strategy"], row["number_of_uavs"], row["replication"])
        for row in read_replication_metrics(results_path)
    }

    pending_rows = []
    try:
        unsupported = set(strategies) - set(SUPPORTED_STRATEGIES)
        if unsupported:
            raise ValueError(f"Unknown strategies: {sorted(unsupported)}")

        for strategy in strategies:
            for number_of_uavs in n_values:
                missing = [
                    replication
                    for replication in range(replications)
                    if (strategy, number_of_uavs, replication + 1)
                    not in completed
                ]
                if not missing:
                    print(
                        f"{STRATEGY_LABELS[strategy]:7s} | "
                        f"N={number_of_uavs:3d} | "
                        f"already has {replications} replications"
                    )
                    continue

                # Time per replication: simulation plus metric calculation.
                started_at = perf_counter()
                for replication in missing:
                    print(
                        f"{STRATEGY_LABELS[strategy]:7s} | "
                        f"N={number_of_uavs:3d} | "
                        f"replication={replication + 1:3d}/{replications}",
                        end="\r",
                    )
                    pending_rows.append(run_simulation(
                        scenario,
                        experiment_label,
                        output_directory,
                        strategy,
                        number_of_uavs,
                        replication,
                        keep_datasets,
                    ))
                    if len(pending_rows) >= SAVE_EVERY:
                        append_rows(results_path, pending_rows)
                        pending_rows.clear()
                mean_time = (perf_counter() - started_at) / len(missing)
                print(
                    f"{STRATEGY_LABELS[strategy]:7s} | "
                    f"N={number_of_uavs:3d} | "
                    f"completed {len(missing)} new replications "
                    f"(total {replications}) | "
                    f"mean {mean_time:.2f} s/replication"
                )
    finally:
        # Also runs on errors and on KeyboardInterrupt.
        append_rows(results_path, pending_rows)

    print("Saved:", results_path)
    return [
        row
        for row in read_replication_metrics(results_path)
        if row["strategy"] in strategies
        and row["number_of_uavs"] in n_values
        and row["replication"] <= replications
    ]


def execute_experiment_parallel(
    scenario: str,
    experiment_label: str,
    output_directory: Path,
    keep_datasets: bool = KEEP_DATASETS,
    strategies: tuple[str, ...] = OFF_NOMINAL_STRATEGIES,
    n_values: tuple[int, ...] = N_VALUES,
    replications: int = K,
    max_workers: int | None = None,
) -> list[dict[str, float | int | str | None]]:
    """Run missing replications in separate processes.

    Each worker writes only its replication-specific SQLite dataset. The main
    process is solely responsible for appending metrics to the shared CSV,
    avoiding concurrent writes to ``replication_metrics.csv``.
    """
    if max_workers is not None and max_workers < 1:
        raise ValueError("max_workers must be at least 1")

    unsupported = set(strategies) - set(SUPPORTED_STRATEGIES)
    if unsupported:
        raise ValueError(f"Unknown strategies: {sorted(unsupported)}")

    output_directory.mkdir(parents=True, exist_ok=True)
    results_path = output_directory / "replication_metrics.csv"
    drop_incomplete_last_line(results_path)
    completed = {
        (row["strategy"], row["number_of_uavs"], row["replication"])
        for row in read_replication_metrics(results_path)
    }
    tasks = [
        (strategy, number_of_uavs, replication)
        for strategy in strategies
        for number_of_uavs in n_values
        for replication in range(replications)
        if (strategy, number_of_uavs, replication + 1) not in completed
    ]

    if not tasks:
        print(
            f"All requested cases already have {replications} replications."
        )
    else:
        workers = max_workers or max(1, (os.cpu_count() or 2) - 1)
        print(
            f"Running {len(tasks)} missing replications with "
            f"{workers} worker processes."
        )
        pending_rows = []
        started_at = perf_counter()
        try:
            with ProcessPoolExecutor(max_workers=workers) as executor:
                task_iterator = iter(tasks)
                initial_tasks = []
                for _ in range(workers):
                    task = next(task_iterator, None)
                    if task is None:
                        break
                    initial_tasks.append(task)
                futures = {
                    executor.submit(
                        run_simulation,
                        scenario,
                        experiment_label,
                        output_directory,
                        strategy,
                        number_of_uavs,
                        replication,
                        keep_datasets,
                    )
                    for strategy, number_of_uavs, replication in initial_tasks
                }
                completed_count = 0
                while futures:
                    done, futures = wait(
                        futures,
                        return_when=FIRST_COMPLETED,
                    )
                    for future in done:
                        pending_rows.append(future.result())
                        completed_count += 1
                        print(
                            f"Completed {completed_count}/{len(tasks)} "
                            "replications",
                            end="\r",
                        )
                        if len(pending_rows) >= SAVE_EVERY:
                            append_rows(results_path, pending_rows)
                            pending_rows.clear()

                        task = next(task_iterator, None)
                        if task is not None:
                            strategy, number_of_uavs, replication = task
                            futures.add(executor.submit(
                                run_simulation,
                                scenario,
                                experiment_label,
                                output_directory,
                                strategy,
                                number_of_uavs,
                                replication,
                                keep_datasets,
                            ))
        finally:
            append_rows(results_path, pending_rows)

        elapsed = perf_counter() - started_at
        print(
            f"Completed {len(tasks)} new replications in {elapsed:.1f} s."
        )

    print("Saved:", results_path)
    return [
        row
        for row in read_replication_metrics(results_path)
        if row["strategy"] in strategies
        and row["number_of_uavs"] in n_values
        and row["replication"] <= replications
    ]


def read_replication_metrics(
    path: Path,
) -> list[dict[str, float | int | str | None]]:
    """Read saved replications, converting the CSV text back into numbers."""
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        required = {"strategy", "number_of_uavs", "replication", "seed"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"{path} has columns {reader.fieldnames}, expected at least "
                f"{sorted(required)}"
            )
        rows: list[dict[str, float | int | str | None]] = list(reader)
    for row in rows:
        for key in ("number_of_uavs", "replication", "seed"):
            row[key] = int(row[key])
        for key in METRIC_KEYS:
            value = row.get(key)
            if value in (None, ""):
                # Older nominal CSVs predate this airspace metric.
                row[key] = 0.0 if key == "geofence_violations" else None
            else:
                row[key] = float(value)
    return rows


def append_rows(
    path: Path,
    rows: list[dict[str, float | int | str | None]],
) -> None:
    """Append rows to a CSV and force them onto the disk."""
    if not rows:
        return
    write_header = not path.exists() or path.stat().st_size == 0
    fieldnames = list(rows[0])
    if not write_header:
        with path.open(newline="", encoding="utf-8") as file:
            existing_header = next(csv.reader(file), None)
        if existing_header:
            fieldnames = existing_header
    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        if write_header:
            writer.writeheader()
        writer.writerows(rows)
        file.flush()
        os.fsync(file.fileno())


def drop_incomplete_last_line(path: Path) -> None:
    """Remove a partially written last line left by an interrupted write."""
    if not path.exists():
        return
    content = path.read_bytes()
    if content and not content.endswith(b"\n"):
        with path.open("r+b") as file:
            file.truncate(content.rfind(b"\n") + 1)


