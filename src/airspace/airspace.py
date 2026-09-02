from __future__ import annotations

from dataclasses import dataclass

from src.airspace.restrictions import Geofence, Obstacle


@dataclass
class Airspace:
    obstacles: list[Obstacle]
    geofences: list[Geofence]

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)

    def add_geofence(self, geofence: Geofence):
        self.geofences.append(geofence)

    def active_geofences_at(self, timestamp: float) -> list[Geofence]:
        return [
            geofence
            for geofence in self.geofences
            if geofence.is_active_at(timestamp)
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
