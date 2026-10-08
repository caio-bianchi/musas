import sqlite3
import tempfile
import unittest
from contextlib import closing
from pathlib import Path

from musas.simulation.dataset import SimulationDataset
from musas.simulation.dataset.schema import SCHEMA_SQL
from musas.simulation.dataset.serialization import json_dumps
from musas.simulation.metrics import (
    actual_distance,
    calculate_metrics,
    number_of_collision_events,
    number_of_loss_of_separation_events,
    number_of_plan_updates,
    planned_distance,
)


class MetricsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.dataset_path = Path(self.temporary_directory.name) / "metrics.sqlite"
        self._create_dataset()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def _create_dataset(self) -> None:
        with closing(sqlite3.connect(self.dataset_path)) as connection:
            connection.executescript(SCHEMA_SQL)
            connection.executemany(
                "INSERT INTO metadata(key, value_json) VALUES (?, ?)",
                [
                    ("schema_version", json_dumps(1)),
                    ("simulation_time", json_dumps(3.0)),
                    ("last_committed_time", json_dumps(3.0)),
                    ("run_status", json_dumps("COMPLETED")),
                ],
            )
            connection.executemany(
                "INSERT INTO uavs VALUES (?, ?, ?, ?, ?)",
                [
                    ("A", "COMPLETED", "Dynamics", "Guidance", "Sensor"),
                    ("B", "COMPLETED", "Dynamics", "Guidance", "Sensor"),
                ],
            )
            connection.executemany(
                """
                INSERT INTO flight_plan_waypoints
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                [
                    ("A", "proposed", 0, 0, 0, 0, 0, 0),
                    ("A", "proposed", 0, 1, 10, 0, 0, 1),
                    ("A", "authorized", 0, 0, 0, 0, 0, 0),
                    ("A", "authorized", 0, 1, 10, 0, 0, 1),
                    ("A", "authorized", 1, 0, 0, 0, 0, 0),
                    ("A", "authorized", 1, 1, 10, 10, 0, 1),
                    ("B", "proposed", 0, 0, 10, 0, 0, 0),
                    ("B", "proposed", 0, 1, 0, 0, 0, 1),
                    ("B", "authorized", 0, 0, 10, 0, 0, 0),
                    ("B", "authorized", 0, 1, 0, 0, 0, 1),
                ],
            )
            connection.executemany(
                "INSERT INTO trajectory VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                [
                    ("A", 0, 0, 0, 0, 0, 0, 0, "ACTIVE"),
                    ("B", 0, 10, 0, 0, 0, 0, 0, "ACTIVE"),
                    ("A", 1, 5, 0, 0, 0, 0, 0, "ACTIVE"),
                    ("B", 1, 5.5, 0, 0, 0, 0, 0, "ACTIVE"),
                    ("A", 2, 10, 0, 0, 0, 0, 0, "COMPLETED"),
                    ("B", 2, 0, 0, 0, 0, 0, 0, "COMPLETED"),
                ],
            )
            connection.executemany(
                "INSERT INTO events(timestamp, component, event, details_json) "
                "VALUES (?, ?, ?, ?)",
                [
                    (0, "A", "FLIGHT_STARTED", "{}"),
                    (2, "A", "FLIGHT_COMPLETED", "{}"),
                    (0, "B", "FLIGHT_STARTED", "{}"),
                    (2, "B", "FLIGHT_COMPLETED", "{}"),
                ],
            )
            connection.commit()

    def test_safety_metrics_count_one_encounter(self) -> None:
        with SimulationDataset(self.dataset_path) as dataset:
            self.assertEqual(
                number_of_loss_of_separation_events(dataset, 2),
                1,
            )
            self.assertEqual(number_of_collision_events(dataset, 1), 1)

    def test_efficiency_metrics(self) -> None:
        with SimulationDataset(self.dataset_path) as dataset:
            self.assertAlmostEqual(planned_distance(dataset, "A"), 10)
            self.assertAlmostEqual(actual_distance(dataset, "A"), 10)
            self.assertEqual(number_of_plan_updates(dataset, "A"), 1)

    def test_metric_summary(self) -> None:
        with SimulationDataset(self.dataset_path) as dataset:
            metrics = calculate_metrics(
                dataset,
                minimum_separation_distance=2,
                collision_distance=1,
            )
        self.assertEqual(metrics["flights"], 2)
        self.assertEqual(metrics["modified_flights"], 1)
        self.assertEqual(metrics["loss_of_separation_events"], 1)
        self.assertEqual(metrics["collision_events"], 1)
        self.assertAlmostEqual(metrics["average_planned_distance"], 10)
        self.assertAlmostEqual(metrics["average_actual_distance"], 10)
        self.assertAlmostEqual(metrics["average_planned_time"], 1)
        self.assertAlmostEqual(metrics["average_actual_time"], 2)


if __name__ == "__main__":
    unittest.main()
