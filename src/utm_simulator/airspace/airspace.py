from dataclasses import dataclass, field

from utm_simulator.airspace.restrictions import Geofence, Obstacle
from utm_simulator.airspace.wind import WindZone


@dataclass
class Airspace:
    obstacles: dict[str, Obstacle] = field(default_factory=dict)
    geofences: dict[str, Geofence] = field(default_factory=dict)
    wind_zones: dict[str, WindZone] = field(default_factory=dict)

    def add_obstacle(self, obstacle: Obstacle) -> None:
        if obstacle.obstacle_id in self.obstacles:
            raise ValueError(
                f"Obstacle {obstacle.obstacle_id!r} already exists"
            )
        self.obstacles[obstacle.obstacle_id] = obstacle

    def remove_obstacle(self, obstacle_id: str) -> Obstacle:
        try:
            return self.obstacles.pop(obstacle_id)
        except KeyError:
            raise KeyError(
                f"Obstacle {obstacle_id!r} does not exist"
            ) from None

    def add_geofence(self, geofence: Geofence) -> None:
        if geofence.geofence_id in self.geofences:
            raise ValueError(
                f"Geofence {geofence.geofence_id!r} already exists"
            )

        self.geofences[geofence.geofence_id] = geofence

    def remove_geofence(self, geofence_id: str) -> Geofence:
        try:
            return self.geofences.pop(geofence_id)
        except KeyError:
            raise KeyError(
                f"Geofence {geofence_id!r} does not exist"
            ) from None

    def active_geofences_at(self, timestamp: float) -> list[Geofence]:
        return [
            geofence
            for geofence in self.geofences.values()
            if geofence.is_active_at(timestamp)
        ]

    def add_wind_zone(self, wind_zone: WindZone) -> None:
        if wind_zone.wind_id in self.wind_zones:
            raise ValueError(
                f"Wind zone {wind_zone.wind_id!r} already exists"
            )
        self.wind_zones[wind_zone.wind_id] = wind_zone

    def remove_wind_zone(self, wind_id: str) -> WindZone:
        try:
            return self.wind_zones.pop(wind_id)
        except KeyError:
            raise KeyError(
                f"Wind zone {wind_id!r} does not exist"
            ) from None

    def active_wind_zones_at(self, timestamp: float) -> list[WindZone]:
        return [
            wind_zone
            for wind_zone in self.wind_zones.values()
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
            for obstacle in self.obstacles.values()
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
