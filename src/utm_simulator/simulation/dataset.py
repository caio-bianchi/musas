from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import os
from pathlib import Path
import sqlite3
import tempfile
from typing import Any, TYPE_CHECKING

from utm_simulator.domain.flight import FlightPlan, Waypoint

if TYPE_CHECKING:
    from utm_simulator.simulation.result import SimulationResult


SCHEMA_VERSION = 2


@dataclass(frozen=True)
class TrajectoryRecord:
    timestamp: float
    uav_name: str
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    status: str


@dataclass(frozen=True)
class StoredEvent:
    timestamp: float
    component: str
    event: str
    details: dict[str, Any]


@dataclass(frozen=True)
class AirspaceObjectRecord:
    object_type: str
    object_id: str
    name: str | None
    geometry_type: str
    geometry: dict[str, Any]
    active_from: float | None
    active_until: float | None
    properties: dict[str, Any]


@dataclass(frozen=True)
class AirspaceSnapshotRecord:
    timestamp: float
    geofence_ids: tuple[str, ...]
    wind_zone_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class UAVRecord:
    name: str
    final_status: str
    dynamics_type: str
    guidance_type: str
    sensor_type: str


@dataclass(frozen=True)
class TrackingReportRecord:
    uav_name: str
    timestamp: float
    received_at: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]


@dataclass(frozen=True)
class TrackingPredictionRecord:
    uav_name: str
    generated_at: float
    state_kind: str
    horizon_index: int
    predicted_timestamp: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]


def _json_default(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return asdict(value)
    if hasattr(value, "__dict__"):
        return {
            key: item
            for key, item in vars(value).items()
            if not key.startswith("_")
        }
    return repr(value)


def _json_dumps(value: Any) -> str:
    return json.dumps(
        value,
        default=_json_default,
        ensure_ascii=False,
        sort_keys=True,
    )


def _qualified_type_name(value: Any) -> str:
    value_type = type(value)
    return f"{value_type.__module__}.{value_type.__qualname__}"


def _geometry_data(geometry: Any) -> tuple[str, dict[str, Any]]:
    if is_dataclass(geometry):
        values = asdict(geometry)
    else:
        try:
            values = {
                key: value
                for key, value in vars(geometry).items()
                if not key.startswith("_")
            }
        except TypeError:
            values = {"representation": repr(geometry)}
    return _qualified_type_name(geometry), values


class _DatasetSchema:
    """Create the SQLite schema shared by every recording mode."""

    @staticmethod
    def _create_schema(connection: sqlite3.Connection) -> None:
        connection.executescript(
            """
            CREATE TABLE metadata (
                key TEXT PRIMARY KEY,
                value_json TEXT NOT NULL
            );

            CREATE TABLE uavs (
                name TEXT PRIMARY KEY,
                final_status TEXT NOT NULL,
                dynamics_type TEXT NOT NULL,
                guidance_type TEXT NOT NULL,
                sensor_type TEXT NOT NULL
            );

            CREATE TABLE flight_plan_waypoints (
                uav_name TEXT NOT NULL,
                plan_kind TEXT NOT NULL,
                plan_version INTEGER NOT NULL,
                waypoint_index INTEGER NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                timestamp REAL NOT NULL,
                PRIMARY KEY (
                    uav_name, plan_kind, plan_version, waypoint_index
                )
            );

            CREATE TABLE trajectory (
                uav_name TEXT NOT NULL,
                timestamp REAL NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                vx REAL NOT NULL,
                vy REAL NOT NULL,
                vz REAL NOT NULL,
                status TEXT NOT NULL,
                PRIMARY KEY (uav_name, timestamp)
            );

            CREATE INDEX trajectory_time_index
                ON trajectory(timestamp);

            CREATE TABLE airspace_objects (
                object_type TEXT NOT NULL,
                object_id TEXT NOT NULL,
                name TEXT,
                geometry_type TEXT NOT NULL,
                geometry_json TEXT NOT NULL,
                active_from REAL,
                active_until REAL,
                properties_json TEXT NOT NULL,
                PRIMARY KEY (object_type, object_id)
            );

            CREATE TABLE airspace_snapshots (
                timestamp REAL PRIMARY KEY,
                geofence_ids_json TEXT NOT NULL,
                wind_zone_ids_json TEXT NOT NULL
            );

            CREATE TABLE tracking_reports (
                uav_name TEXT NOT NULL,
                timestamp REAL NOT NULL,
                received_at REAL NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                vx REAL NOT NULL,
                vy REAL NOT NULL,
                vz REAL NOT NULL,
                PRIMARY KEY (uav_name, timestamp, received_at)
            );

            CREATE TABLE tracking_predictions (
                uav_name TEXT NOT NULL,
                generated_at REAL NOT NULL,
                state_kind TEXT NOT NULL,
                horizon_index INTEGER NOT NULL,
                predicted_timestamp REAL NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                vx REAL NOT NULL,
                vy REAL NOT NULL,
                vz REAL NOT NULL,
                PRIMARY KEY (
                    uav_name, generated_at, state_kind, horizon_index
                )
            );

            CREATE TABLE events (
                event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                component TEXT NOT NULL,
                event TEXT NOT NULL,
                details_json TEXT NOT NULL
            );

            CREATE INDEX events_type_index ON events(event);
            CREATE INDEX events_time_index ON events(timestamp);
            """
        )


class SimulationDatasetRecorder:
    """Write complete or incremental simulation data to SQLite."""

    @staticmethod
    def write(
        result: SimulationResult,
        path: str | Path,
        metadata: dict[str, Any] | None = None,
        overwrite: bool = False,
    ) -> Path:
        """Write a complete result using the incremental recorder."""
        destination = Path(path).expanduser().resolve()
        if destination.exists() and not overwrite:
            raise FileExistsError(
                f"simulation dataset already exists: {destination}"
            )
        destination.parent.mkdir(parents=True, exist_ok=True)

        temporary_file = tempfile.NamedTemporaryFile(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
            delete=False,
        )
        temporary_path = Path(temporary_file.name)
        temporary_file.close()
        temporary_path.unlink()

        try:
            recorder = SimulationDatasetRecorder(
                path=temporary_path,
                configuration=result.configuration,
                metadata=metadata,
            )
            try:
                recorder.flush(result, run_status="SNAPSHOT")
            finally:
                recorder.close()
            os.replace(temporary_path, destination)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise

        return destination

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
                connection.execute("PRAGMA foreign_keys = ON")
                _DatasetSchema._create_schema(connection)
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
                        (key, _json_dumps(value))
                        for key, value in initial_metadata.items()
                    ],
                )
            os.replace(temporary_path, self.path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise

        self._connection = sqlite3.connect(self.path)
        self._connection.execute("PRAGMA foreign_keys = ON")
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

    @property
    def closed(self) -> bool:
        return self._closed

    def flush(
        self,
        result: SimulationResult,
        run_status: str = "RUNNING",
    ) -> None:
        """Persist records added since the previous successful commit."""
        if self._closed:
            raise RuntimeError("simulation dataset recorder is closed")

        state_lengths = {
            name: len(history)
            for name, history in result.state_history.items()
        }
        flight_plan_versions = {
            name: len(history)
            for name, history in result.flight_plan_history.items()
        }
        airspace_history_length = len(result.airspace_history)
        prediction_history_length = len(
            result.tracking_predictions
        )
        report_history_length = len(result.tracking_reports)
        event_history_length = len(result.event_history)

        with self._connection:
            self._write_metadata(result, run_status)
            self._write_uavs(result)
            self._write_flight_plans(result)
            self._write_states(result)
            self._write_airspace(result)
            self._write_tracking(result)
            self._write_events(result)

        self._state_lengths = state_lengths
        self._written_proposed_plans.update(
            result.proposed_flight_plans
        )
        self._flight_plan_versions = flight_plan_versions
        self._airspace_history_length = airspace_history_length
        self._tracking_prediction_length = prediction_history_length
        self._tracking_report_length = report_history_length
        self._event_history_length = event_history_length

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
            [(key, _json_dumps(value)) for key, value in values.items()],
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
            for index, waypoint in enumerate(flight_plan.waypoint_list):
                rows.append(
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
            for sample in history[first_index:]:
                rows.append(
                    (
                        name,
                        sample.timestamp,
                        *sample.position,
                        *sample.velocity,
                        sample.status.name,
                    )
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
            geofence.geofence_id: geofence
            for snapshot in result.airspace_history
            for geofence in snapshot.geofences
        }
        geofences.update(
            {
                geofence.geofence_id: geofence
                for geofence in result.airspace.geofences.values()
            }
        )
        wind_zones = {
            wind_zone.wind_id: wind_zone
            for snapshot in result.airspace_history
            for wind_zone in snapshot.wind_zones
        }
        wind_zones.update(
            {
                wind_zone.wind_id: wind_zone
                for wind_zone in result.airspace.wind_zones.values()
            }
        )
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
            geometry_type, geometry_values = _geometry_data(geometry)
            object_rows.append(
                (
                    object_type,
                    object_id,
                    name,
                    geometry_type,
                    _json_dumps(geometry_values),
                    active_from,
                    active_until,
                    _json_dumps(properties),
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
            )
            VALUES (?, ?, ?)
            ON CONFLICT(timestamp) DO UPDATE SET
                geofence_ids_json = excluded.geofence_ids_json,
                wind_zone_ids_json = excluded.wind_zone_ids_json
            """,
            [
                (
                    snapshot.timestamp,
                    _json_dumps(
                        [item.geofence_id for item in snapshot.geofences]
                    ),
                    _json_dumps(
                        [item.wind_id for item in snapshot.wind_zones]
                    ),
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
                    _json_dumps(event.details),
                )
                for event in result.event_history[
                    self._event_history_length:
                ]
            ],
        )

    def reset_transient_offsets(self) -> None:
        """Reset cursors after the caller releases persisted buffers."""
        self._state_lengths = {}
        self._airspace_history_length = 0
        self._tracking_prediction_length = 0
        self._tracking_report_length = 0
        self._event_history_length = 0

    def close(self) -> None:
        if not self._closed:
            self._connection.close()
            self._closed = True


class SimulationDataset:
    """Read-only access to a persisted simulation execution."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser().resolve()
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self._connection = sqlite3.connect(
            f"file:{self.path}?mode=ro",
            uri=True,
        )
        self._connection.row_factory = sqlite3.Row
        self.schema_version = self.metadata.get("schema_version")
        if self.schema_version not in (1, SCHEMA_VERSION):
            self.close()
            raise ValueError(
                "unsupported simulation dataset schema version"
            )

    @classmethod
    def open(cls, path: str | Path) -> SimulationDataset:
        return cls(path)

    @property
    def metadata(self) -> dict[str, Any]:
        rows = self._connection.execute(
            "SELECT key, value_json FROM metadata"
        )
        return {
            row["key"]: json.loads(row["value_json"])
            for row in rows
        }

    @property
    def simulation_time(self) -> float:
        return float(self.metadata["simulation_time"])

    @property
    def last_committed_time(self) -> float:
        return float(
            self.metadata.get("last_committed_time", self.simulation_time)
        )

    @property
    def run_status(self) -> str:
        return str(self.metadata.get("run_status", "SNAPSHOT"))

    @property
    def uav_names(self) -> tuple[str, ...]:
        rows = self._connection.execute(
            "SELECT name FROM uavs ORDER BY name"
        )
        return tuple(row["name"] for row in rows)

    def get_uavs(self) -> list[UAVRecord]:
        rows = self._connection.execute(
            "SELECT * FROM uavs ORDER BY name"
        )
        return [
            UAVRecord(
                name=row["name"],
                final_status=row["final_status"],
                dynamics_type=row["dynamics_type"],
                guidance_type=row["guidance_type"],
                sensor_type=row["sensor_type"],
            )
            for row in rows
        ]

    def get_trajectory(
        self,
        uav_name: str | None = None,
        start_time: float | None = None,
        end_time: float | None = None,
    ) -> list[TrajectoryRecord]:
        clauses = []
        parameters: list[Any] = []
        if uav_name is not None:
            clauses.append("uav_name = ?")
            parameters.append(uav_name)
        if start_time is not None:
            clauses.append("timestamp >= ?")
            parameters.append(float(start_time))
        if end_time is not None:
            clauses.append("timestamp <= ?")
            parameters.append(float(end_time))

        where = f"WHERE {' AND '.join(clauses)}" if clauses else ""
        rows = self._connection.execute(
            f"""
            SELECT uav_name, timestamp, x, y, z, vx, vy, vz, status
            FROM trajectory
            {where}
            ORDER BY timestamp, uav_name
            """,
            parameters,
        )
        return [
            TrajectoryRecord(
                timestamp=row["timestamp"],
                uav_name=row["uav_name"],
                position=(row["x"], row["y"], row["z"]),
                velocity=(row["vx"], row["vy"], row["vz"]),
                status=row["status"],
            )
            for row in rows
        ]

    def get_flight_plan(
        self,
        uav_name: str,
        plan_kind: str = "proposed",
        plan_version: int = 0,
    ) -> FlightPlan | None:
        rows = list(
            self._connection.execute(
                """
                SELECT x, y, z, timestamp
                FROM flight_plan_waypoints
                WHERE uav_name = ?
                  AND plan_kind = ?
                  AND plan_version = ?
                ORDER BY waypoint_index
                """,
                (uav_name, plan_kind, plan_version),
            )
        )
        if not rows:
            return None
        return FlightPlan(
            [
                Waypoint(
                    row["x"],
                    row["y"],
                    row["z"],
                    row["timestamp"],
                )
                for row in rows
            ]
        )

    def get_flight_plan_versions(
        self,
        uav_name: str,
        plan_kind: str = "authorized",
    ) -> tuple[int, ...]:
        rows = self._connection.execute(
            """
            SELECT DISTINCT plan_version
            FROM flight_plan_waypoints
            WHERE uav_name = ? AND plan_kind = ?
            ORDER BY plan_version
            """,
            (uav_name, plan_kind),
        )
        return tuple(row["plan_version"] for row in rows)

    def get_events(
        self,
        event_type: str | None = None,
        component: str | None = None,
        start_time: float | None = None,
        end_time: float | None = None,
    ) -> list[StoredEvent]:
        clauses = []
        parameters: list[Any] = []
        for column, value in (
            ("event", event_type),
            ("component", component),
        ):
            if value is not None:
                clauses.append(f"{column} = ?")
                parameters.append(value)
        if start_time is not None:
            clauses.append("timestamp >= ?")
            parameters.append(float(start_time))
        if end_time is not None:
            clauses.append("timestamp <= ?")
            parameters.append(float(end_time))

        where = f"WHERE {' AND '.join(clauses)}" if clauses else ""
        rows = self._connection.execute(
            f"""
            SELECT timestamp, component, event, details_json
            FROM events
            {where}
            ORDER BY timestamp, event_id
            """,
            parameters,
        )
        return [
            StoredEvent(
                timestamp=row["timestamp"],
                component=row["component"],
                event=row["event"],
                details=json.loads(row["details_json"]),
            )
            for row in rows
        ]

    def get_airspace_objects(
        self,
        object_type: str | None = None,
    ) -> list[AirspaceObjectRecord]:
        if object_type is None:
            rows = self._connection.execute(
                "SELECT * FROM airspace_objects ORDER BY object_type, object_id"
            )
        else:
            rows = self._connection.execute(
                """
                SELECT * FROM airspace_objects
                WHERE object_type = ?
                ORDER BY object_id
                """,
                (object_type,),
            )
        return [
            AirspaceObjectRecord(
                object_type=row["object_type"],
                object_id=row["object_id"],
                name=row["name"],
                geometry_type=row["geometry_type"],
                geometry=json.loads(row["geometry_json"]),
                active_from=row["active_from"],
                active_until=row["active_until"],
                properties=json.loads(row["properties_json"]),
            )
            for row in rows
        ]

    def get_tracking_reports(
        self,
        uav_name: str | None = None,
    ) -> list[TrackingReportRecord]:
        if uav_name is None:
            rows = self._connection.execute(
                """
                SELECT * FROM tracking_reports
                ORDER BY timestamp, uav_name
                """
            )
        else:
            rows = self._connection.execute(
                """
                SELECT * FROM tracking_reports
                WHERE uav_name = ?
                ORDER BY timestamp
                """,
                (uav_name,),
            )
        return [
            TrackingReportRecord(
                uav_name=row["uav_name"],
                timestamp=row["timestamp"],
                received_at=row["received_at"],
                position=(row["x"], row["y"], row["z"]),
                velocity=(row["vx"], row["vy"], row["vz"]),
            )
            for row in rows
        ]

    def get_tracking_predictions(
        self,
        uav_name: str | None = None,
    ) -> list[TrackingPredictionRecord]:
        if uav_name is None:
            rows = self._connection.execute(
                """
                SELECT * FROM tracking_predictions
                ORDER BY generated_at, uav_name, state_kind, horizon_index
                """
            )
        else:
            rows = self._connection.execute(
                """
                SELECT * FROM tracking_predictions
                WHERE uav_name = ?
                ORDER BY generated_at, state_kind, horizon_index
                """,
                (uav_name,),
            )
        return [
            TrackingPredictionRecord(
                uav_name=row["uav_name"],
                generated_at=row["generated_at"],
                state_kind=row["state_kind"],
                horizon_index=row["horizon_index"],
                predicted_timestamp=row["predicted_timestamp"],
                position=(row["x"], row["y"], row["z"]),
                velocity=(row["vx"], row["vy"], row["vz"]),
            )
            for row in rows
        ]

    def get_active_geofence_ids(self, timestamp: float) -> tuple[str, ...]:
        row = self._connection.execute(
            """
            SELECT geofence_ids_json
            FROM airspace_snapshots
            WHERE timestamp <= ?
            ORDER BY timestamp DESC
            LIMIT 1
            """,
            (float(timestamp),),
        ).fetchone()
        if row is None:
            return ()
        return tuple(json.loads(row["geofence_ids_json"]))

    def get_active_wind_zone_ids(self, timestamp: float) -> tuple[str, ...]:
        if self.schema_version == 1:
            return tuple(
                item.object_id
                for item in self.get_airspace_objects("wind_zone")
                if (
                    item.active_from is None
                    or item.active_from <= timestamp
                )
                and (
                    item.active_until is None
                    or timestamp <= item.active_until
                )
            )
        row = self._connection.execute(
            """
            SELECT wind_zone_ids_json
            FROM airspace_snapshots
            WHERE timestamp <= ?
            ORDER BY timestamp DESC
            LIMIT 1
            """,
            (float(timestamp),),
        ).fetchone()
        if row is None:
            return ()
        return tuple(json.loads(row["wind_zone_ids_json"]))

    def get_airspace_snapshots(self) -> list[AirspaceSnapshotRecord]:
        if self.schema_version == 1:
            rows = list(
                self._connection.execute(
                    """
                    SELECT timestamp, geofence_ids_json
                    FROM airspace_snapshots
                    ORDER BY timestamp
                    """
                )
            )
            return [
                AirspaceSnapshotRecord(
                    timestamp=row["timestamp"],
                    geofence_ids=tuple(
                        json.loads(row["geofence_ids_json"])
                    ),
                    wind_zone_ids=self.get_active_wind_zone_ids(
                        row["timestamp"]
                    ),
                )
                for row in rows
            ]
        rows = self._connection.execute(
            """
            SELECT timestamp, geofence_ids_json, wind_zone_ids_json
            FROM airspace_snapshots
            ORDER BY timestamp
            """
        )
        return [
            AirspaceSnapshotRecord(
                timestamp=row["timestamp"],
                geofence_ids=tuple(
                    json.loads(row["geofence_ids_json"])
                ),
                wind_zone_ids=tuple(
                    json.loads(row["wind_zone_ids_json"])
                ),
            )
            for row in rows
        ]

    def table_count(self, table: str) -> int:
        allowed_tables = {
            "uavs",
            "flight_plan_waypoints",
            "trajectory",
            "airspace_objects",
            "airspace_snapshots",
            "tracking_reports",
            "tracking_predictions",
            "events",
        }
        if table not in allowed_tables:
            raise ValueError(f"unknown dataset table: {table}")
        row = self._connection.execute(
            f"SELECT COUNT(*) AS count FROM {table}"
        ).fetchone()
        return int(row["count"])

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> SimulationDataset:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
