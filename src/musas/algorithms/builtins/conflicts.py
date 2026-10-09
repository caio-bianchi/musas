from __future__ import annotations

from dataclasses import dataclass

from musas.airspace.restrictions import Geofence, Obstacle
from musas.domain.flight import Conflict


AirspaceRestriction = Obstacle | Geofence


@dataclass(frozen=True)
class StrategicTrafficConflict(Conflict):
    """Loss of separation between a proposal and an approved flight plan."""

    timestamp: float
    conflicting_uav: str
    proposed_position: tuple[float, ...]
    conflicting_position: tuple[float, ...]
    separation_distance: float


@dataclass(frozen=True)
class StrategicAirspaceConflict(Conflict):
    """Intersection between a proposed flight plan and a restriction."""

    timestamp: float
    position: tuple[float, ...]
    restriction: AirspaceRestriction


@dataclass(frozen=True)
class TacticalTrafficConflict(Conflict):
    """Loss of separation predicted from two short-term tracks."""

    timestamp: float
    uav_name: str
    other_uav: str
    position: tuple[float, ...]
    other_position: tuple[float, ...]
    separation_distance: float


@dataclass(frozen=True)
class TacticalAirspaceConflict(Conflict):
    """Intersection between a short-term track and a restriction."""

    timestamp: float
    uav_name: str
    position: tuple[float, ...]
    restriction: AirspaceRestriction
