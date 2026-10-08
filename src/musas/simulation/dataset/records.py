from dataclasses import dataclass
from typing import Any


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
    wind_zone_ids: tuple[str, ...]


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
