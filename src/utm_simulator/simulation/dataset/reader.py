from __future__ import annotations

import json
from pathlib import Path
import sqlite3
from typing import Any

from utm_simulator.domain.flight import FlightPlan, Waypoint
from utm_simulator.simulation.dataset.records import (
    AirspaceObjectRecord,
    AirspaceSnapshotRecord,
    StoredEvent,
    TrackingPredictionRecord,
    TrackingReportRecord,
    TrajectoryRecord,
    UAVRecord,
)
from utm_simulator.simulation.dataset.schema import SCHEMA_VERSION


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
        self.schema_version = self.metadata["schema_version"]
        if self.schema_version != SCHEMA_VERSION:
            self.close()
            raise ValueError(
                "unsupported simulation dataset schema version: "
                f"{self.schema_version}"
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
        return float(self.metadata["last_committed_time"])

    @property
    def run_status(self) -> str:
        return str(self.metadata["run_status"])

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
        return FlightPlan([
            Waypoint(
                row["x"],
                row["y"],
                row["z"],
                row["timestamp"],
            )
            for row in rows
        ])

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
        if event_type is not None:
            clauses.append("event = ?")
            parameters.append(event_type)
        if component is not None:
            clauses.append("component = ?")
            parameters.append(component)
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
                """
                SELECT * FROM airspace_objects
                ORDER BY object_type, object_id
                """
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
                geofence_ids=tuple(json.loads(row["geofence_ids_json"])),
                wind_zone_ids=tuple(json.loads(row["wind_zone_ids_json"])),
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
