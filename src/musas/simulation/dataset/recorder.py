from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import sqlite3
import tempfile
from typing import Any, TYPE_CHECKING

from musas.domain.flight import FlightPlan
from musas.simulation.dataset.schema import SCHEMA_SQL, SCHEMA_VERSION
from musas.simulation.dataset.serialization import (
    geometry_data,
    json_dumps,
)

if TYPE_CHECKING:
    from musas.simulation.result import SimulationResult


class SimulationDatasetRecorder:
    """Write complete or incremental simulation data to SQLite."""

    @classmethod
    def write(
        cls,
        result: SimulationResult,
        path: str | Path,
        metadata: dict[str, Any] | None = None,
        overwrite: bool = False,
    ) -> Path:
        """Write a complete simulation result."""
        recorder = cls(
            path=path,
            configuration=result.configuration,
            metadata=metadata,
            overwrite=overwrite,
        )
        try:
            recorder.flush(result, run_status="SNAPSHOT")
        finally:
            recorder.close()
        return recorder.path

    def __init__(
        self,
        path: str | Path,
        configuration: dict[str, Any],
        metadata: dict[str, Any] | None = None,
        overwrite: bool = False,
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        if self.path.exists() and not overwrite:
            raise FileExistsError(
                f"simulation dataset already exists: {self.path}"
            )
        self.path.parent.mkdir(parents=True, exist_ok=True)

        temporary_file = tempfile.NamedTemporaryFile(
            prefix=f".{self.path.name}.",
            suffix=".tmp",
            dir=self.path.parent,
            delete=False,
        )
        temporary_path = Path(temporary_file.name)
        temporary_file.close()
        try:
            with sqlite3.connect(temporary_path) as connection:
                connection.executescript(SCHEMA_SQL)
                initial_metadata = {
                    **(metadata or {}),
                    "schema_version": SCHEMA_VERSION,
                    "created_at": datetime.now(timezone.utc).isoformat(),
                    "simulation_time": 0.0,
                    "last_committed_time": 0.0,
                    "run_status": "RUNNING",
                    "configuration": configuration,
                }
                connection.executemany(
                    "INSERT INTO metadata(key, value_json) VALUES (?, ?)",
                    [
                        (key, json_dumps(value))
                        for key, value in initial_metadata.items()
                    ],
                )
            os.replace(temporary_path, self.path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise

        self._connection = sqlite3.connect(self.path)
        self._connection.execute("PRAGMA journal_mode = WAL")
        self._connection.execute("PRAGMA synchronous = NORMAL")
        self._state_lengths: dict[str, int] = {}
        self._written_proposed_plans: set[str] = set()
        self._flight_plan_versions: dict[str, int] = {}
        self._airspace_history_length = 0
        self._tracking_prediction_length = 0
        self._tracking_report_length = 0
        self._event_history_length = 0
        self._closed = False

    def flush(
        self,
        result: SimulationResult,
        run_status: str = "RUNNING",
    ) -> None:
        """Persist records added since the previous successful commit."""
        if self._closed:
            raise RuntimeError("simulation dataset recorder is closed")

        with self._connection:
            self._write_metadata(result, run_status)
            self._write_uavs(result)
            self._write_flight_plans(result)
            self._write_states(result)
            self._write_airspace(result)
            self._write_tracking(result)
            self._write_events(result)

        self._state_lengths = {
            name: len(history)
            for name, history in result.state_history.items()
        }
        self._written_proposed_plans.update(result.proposed_flight_plans)
        self._flight_plan_versions = {
            name: len(history)
            for name, history in result.flight_plan_history.items()
        }
        self._airspace_history_length = len(result.airspace_history)
        self._tracking_prediction_length = len(result.tracking_predictions)
        self._tracking_report_length = len(result.tracking_reports)
        self._event_history_length = len(result.event_history)

    def _write_metadata(
        self,
        result: SimulationResult,
        run_status: str,
    ) -> None:
        values = {
            "simulation_time": result.time,
            "last_committed_time": result.time,
            "run_status": run_status,
        }
        self._connection.executemany(
            """
            INSERT INTO metadata(key, value_json) VALUES (?, ?)
            ON CONFLICT(key) DO UPDATE SET value_json = excluded.value_json
            """,
            [(key, json_dumps(value)) for key, value in values.items()],
        )

    def _write_uavs(self, result: SimulationResult) -> None:
        self._connection.executemany(
            """
            INSERT INTO uavs(
                name, final_status, dynamics_type, guidance_type, sensor_type
            ) VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(name) DO UPDATE SET
                final_status = excluded.final_status,
                dynamics_type = excluded.dynamics_type,
                guidance_type = excluded.guidance_type,
                sensor_type = excluded.sensor_type
            """,
            [
                (
                    name,
                    uav.status.name,
                    uav.dynamics_type,
                    uav.guidance_type,
                    uav.sensor_type,
                )
                for name, uav in result.uavs.items()
            ],
        )

    def _write_flight_plans(self, result: SimulationResult) -> None:
        rows = []

        def append_plan(
            uav_name: str,
            plan_kind: str,
            version: int,
            flight_plan: FlightPlan,
        ) -> None:
            rows.extend(
                (
                    uav_name,
                    plan_kind,
                    version,
                    index,
                    waypoint.x,
                    waypoint.y,
                    waypoint.z,
                    waypoint.t,
                )
                for index, waypoint in enumerate(flight_plan.waypoint_list)
            )

        for name, plan in result.proposed_flight_plans.items():
            if name not in self._written_proposed_plans:
                append_plan(name, "proposed", 0, plan)
        for name, history in result.flight_plan_history.items():
            first_version = self._flight_plan_versions.get(name, 0)
            for version in range(first_version, len(history)):
                append_plan(name, "authorized", version, history[version])

        self._connection.executemany(
            """
            INSERT INTO flight_plan_waypoints(
                uav_name, plan_kind, plan_version, waypoint_index,
                x, y, z, timestamp
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            rows,
        )

    def _write_states(self, result: SimulationResult) -> None:
        rows = []
        for name, history in result.state_history.items():
            first_index = self._state_lengths.get(name, 0)
            rows.extend(
                (
                    name,
                    sample.timestamp,
                    *sample.position,
                    *sample.velocity,
                    sample.status.name,
                )
                for sample in history[first_index:]
            )
        self._connection.executemany(
            """
            INSERT INTO trajectory(
                uav_name, timestamp, x, y, z, vx, vy, vz, status
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            rows,
        )

    def _write_airspace(self, result: SimulationResult) -> None:
        geofences = {
            item.geofence_id: item
            for snapshot in result.airspace_history
            for item in snapshot.geofences
        }
        geofences.update(result.airspace.geofences)

        wind_zones = {
            item.wind_id: item
            for snapshot in result.airspace_history
            for item in snapshot.wind_zones
        }
        wind_zones.update(result.airspace.wind_zones)

        objects = [
            (
                "geofence",
                item.geofence_id,
                item.name,
                item.geometry,
                item.active_from,
                item.active_until,
                {},
            )
            for item in geofences.values()
        ]
        objects.extend(
            (
                "obstacle",
                item.obstacle_id,
                item.name,
                item.geometry,
                None,
                None,
                {},
            )
            for item in result.airspace.obstacles.values()
        )
        objects.extend(
            (
                "wind_zone",
                item.wind_id,
                item.name,
                item.geometry,
                item.active_from,
                item.active_until,
                {"velocity": list(item.velocity)},
            )
            for item in wind_zones.values()
        )

        object_rows = []
        for (
            object_type,
            object_id,
            name,
            geometry,
            active_from,
            active_until,
            properties,
        ) in objects:
            geometry_type, geometry_values = geometry_data(geometry)
            object_rows.append(
                (
                    object_type,
                    object_id,
                    name,
                    geometry_type,
                    json_dumps(geometry_values),
                    active_from,
                    active_until,
                    json_dumps(properties),
                )
            )

        self._connection.executemany(
            """
            INSERT INTO airspace_objects(
                object_type, object_id, name, geometry_type, geometry_json,
                active_from, active_until, properties_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(object_type, object_id) DO UPDATE SET
                name = excluded.name,
                geometry_type = excluded.geometry_type,
                geometry_json = excluded.geometry_json,
                active_from = excluded.active_from,
                active_until = excluded.active_until,
                properties_json = excluded.properties_json
            """,
            object_rows,
        )
        self._connection.executemany(
            """
            INSERT INTO airspace_snapshots(
                timestamp, geofence_ids_json, wind_zone_ids_json
            ) VALUES (?, ?, ?)
            ON CONFLICT(timestamp) DO UPDATE SET
                geofence_ids_json = excluded.geofence_ids_json,
                wind_zone_ids_json = excluded.wind_zone_ids_json
            """,
            [
                (
                    snapshot.timestamp,
                    json_dumps([
                        item.geofence_id for item in snapshot.geofences
                    ]),
                    json_dumps([
                        item.wind_id for item in snapshot.wind_zones
                    ]),
                )
                for snapshot in result.airspace_history[
                    self._airspace_history_length:
                ]
            ],
        )

    def _write_tracking(self, result: SimulationResult) -> None:
        reports = result.tracking_reports[self._tracking_report_length:]
        self._connection.executemany(
            """
            INSERT OR IGNORE INTO tracking_reports(
                uav_name, timestamp, received_at, x, y, z, vx, vy, vz
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            [
                (
                    report.uav_name,
                    report.timestamp,
                    report.received_at,
                    *report.position,
                    *report.velocity,
                )
                for report in reports
            ],
        )

        rows = []
        for prediction in result.tracking_predictions[
            self._tracking_prediction_length:
        ]:
            estimated = prediction.estimated_state
            rows.append(
                (
                    prediction.uav_name,
                    prediction.generated_at,
                    "estimated",
                    -1,
                    estimated.timestamp,
                    *estimated.position,
                    *estimated.velocity,
                )
            )
            rows.extend(
                (
                    prediction.uav_name,
                    prediction.generated_at,
                    "predicted",
                    index,
                    state.timestamp,
                    *state.position,
                    *state.velocity,
                )
                for index, state in enumerate(prediction.predicted_states)
            )
        self._connection.executemany(
            """
            INSERT OR IGNORE INTO tracking_predictions(
                uav_name, generated_at, state_kind, horizon_index,
                predicted_timestamp, x, y, z, vx, vy, vz
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            rows,
        )

    def _write_events(self, result: SimulationResult) -> None:
        self._connection.executemany(
            """
            INSERT INTO events(timestamp, component, event, details_json)
            VALUES (?, ?, ?, ?)
            """,
            [
                (
                    event.timestamp,
                    event.component,
                    event.event,
                    json_dumps(event.details),
                )
                for event in result.event_history[
                    self._event_history_length:
                ]
            ],
        )

    def reset_transient_offsets(self) -> None:
        """Reset cursors after persisted in-memory buffers are cleared."""
        self._state_lengths = {}
        self._airspace_history_length = 0
        self._tracking_prediction_length = 0
        self._tracking_report_length = 0
        self._event_history_length = 0

    def close(self) -> None:
        if not self._closed:
            self._connection.close()
            self._closed = True
