from __future__ import annotations

from dataclasses import dataclass, field

from src.airspace.restrictions import Geofence, Obstacle
from src.airspace.wind import WindZone


@dataclass
class Airspace:
    obstacles: list[Obstacle]
    geofences: list[Geofence]
    wind_zones: list[WindZone] = field(default_factory=list)

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)

    def add_geofence(self, geofence: Geofence) -> None:
        if any(
            existing.geofence_id == geofence.geofence_id
            for existing in self.geofences
        ):
            raise ValueError(
                f"Geofence {geofence.geofence_id!r} already exists"
            )

        self.geofences.append(geofence)

    def remove_geofence(self, geofence_id: str) -> Geofence:
        for index, geofence in enumerate(self.geofences):
            if geofence.geofence_id == geofence_id:
                return self.geofences.pop(index)

        raise KeyError(f"Geofence {geofence_id!r} does not exist")

    def active_geofences_at(self, timestamp: float) -> list[Geofence]:
        return [
            geofence
            for geofence in self.geofences
            if geofence.is_active_at(timestamp)
        ]

    def add_wind_zone(self, wind_zone: WindZone) -> None:
        if any(
            existing.wind_id == wind_zone.wind_id
            for existing in self.wind_zones
        ):
            raise ValueError(
                f"Wind zone {wind_zone.wind_id!r} already exists"
            )
        self.wind_zones.append(wind_zone)

    def remove_wind_zone(self, wind_id: str) -> WindZone:
        for index, wind_zone in enumerate(self.wind_zones):
            if wind_zone.wind_id == wind_id:
                return self.wind_zones.pop(index)
        raise KeyError(f"Wind zone {wind_id!r} does not exist")

    def active_wind_zones_at(self, timestamp: float) -> list[WindZone]:
        return [
            wind_zone
            for wind_zone in self.wind_zones
            if wind_zone.is_active_at(timestamp)
        ]

    def wind_zones_at(self, position, timestamp: float) -> list[WindZone]:
        return [
            wind_zone
            for wind_zone in self.active_wind_zones_at(timestamp)
            if wind_zone.geometry.contains(position)
        ]

    def wind_at(self, position, timestamp: float) -> list[float]:
        wind_zones = self.wind_zones_at(position, timestamp)
        return [
            sum(wind_zone.velocity[axis] for wind_zone in wind_zones)
            for axis in range(3)
        ]

    def prohibited_regions_at(
        self,
        position,
        timestamp,
    ) -> list[Obstacle | Geofence]:
        containing_obstacles = [
            obstacle
            for obstacle in self.obstacles
            if obstacle.contains(position)
        ]
        containing_geofences = [
            geofence
            for geofence in self.active_geofences_at(timestamp)
            if geofence.geometry.contains(position)
        ]

        return [*containing_obstacles, *containing_geofences]

    def is_prohibited(
        self,
        position,
        timestamp,
    ) -> bool:
        return bool(self.prohibited_regions_at(position, timestamp))
