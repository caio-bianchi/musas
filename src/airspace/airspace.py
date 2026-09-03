from __future__ import annotations

from dataclasses import dataclass

from src.airspace.restrictions import Geofence, Obstacle


@dataclass
class Airspace:
    obstacles: list[Obstacle]
    geofences: list[Geofence]

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
