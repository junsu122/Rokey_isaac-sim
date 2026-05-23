from __future__ import annotations

import math
from dataclasses import dataclass

from smart_factory.models import FactoryMap, Pose2D


@dataclass(frozen=True)
class GridTransform:
    origin_x: float = 0.0
    origin_y: float = 0.0
    resolution: float = 1.0

    def grid_to_world(self, point: tuple[int, int]) -> tuple[float, float]:
        return (
            self.origin_x + point[0] * self.resolution,
            self.origin_y + point[1] * self.resolution,
        )

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return (
            round((x - self.origin_x) / self.resolution),
            round((y - self.origin_y) / self.resolution),
        )


@dataclass(frozen=True)
class CurrentLocation:
    pose: Pose2D
    grid_cell: tuple[int, int]
    nearest_waypoint: str
    nearest_distance: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def estimate_current_location(
    pose: Pose2D,
    factory_map: FactoryMap,
    transform: GridTransform,
) -> CurrentLocation:
    grid_cell = transform.world_to_grid(pose.x, pose.y)
    nearest_name = ""
    nearest_distance = math.inf

    for name, waypoint in factory_map.waypoints.items():
        wx, wy = transform.grid_to_world(waypoint.point)
        distance = math.hypot(pose.x - wx, pose.y - wy)
        if distance < nearest_distance or (
            math.isclose(distance, nearest_distance) and _is_alias_name(name, nearest_name)
        ):
            nearest_name = name
            nearest_distance = distance

    return CurrentLocation(
        pose=pose,
        grid_cell=grid_cell,
        nearest_waypoint=nearest_name,
        nearest_distance=nearest_distance,
    )


def _is_alias_name(candidate: str, current: str) -> bool:
    return not _is_grid_name(candidate) and _is_grid_name(current)


def _is_grid_name(name: str) -> bool:
    if not name.startswith("N"):
        return False
    parts = name[1:].split("_")
    return len(parts) == 2 and all(part.isdigit() for part in parts)
