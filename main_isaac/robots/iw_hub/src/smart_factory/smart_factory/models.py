from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Set, Tuple


GridPoint = Tuple[int, int]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def relative_to(self, reference: "Pose2D") -> "Pose2D":
        return Pose2D(
            x=self.x - reference.x,
            y=self.y - reference.y,
            yaw=self.yaw - reference.yaw,
        )


class TaskStatus(str, Enum):
    WAITING = "waiting"
    ASSIGNED = "assigned"
    DONE = "done"


class RobotStatus(str, Enum):
    IDLE = "idle"
    MOVING_TO_PICKUP = "moving_to_pickup"
    MOVING_TO_DROPOFF = "moving_to_dropoff"
    CHARGING = "charging"
    MOVING_TO_SHELF = "moving_to_shelf"
    CARRYING_SHELF = "carrying_shelf"


class MarkerType(str, Enum):
    SHELF_BOTTOM = "shelf_bottom"
    WALL_SLOT = "wall_slot"


class AlignmentPhase(str, Enum):
    PICKUP = "pickup"
    DROPOFF = "dropoff"


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    marker_type: MarkerType
    pose: Pose2D
    target_id: str


@dataclass(frozen=True)
class AlignmentTolerance:
    max_x_error: float = 0.03
    max_y_error: float = 0.03
    max_yaw_error: float = 0.035


@dataclass(frozen=True)
class AlignmentCommand:
    linear_x: float
    linear_y: float
    angular_z: float
    aligned: bool
    reason: str


@dataclass(frozen=True)
class MarkerOffset:
    target_id: str
    marker_id: int
    marker_type: MarkerType
    offset: Pose2D


@dataclass(frozen=True)
class Waypoint:
    name: str
    point: GridPoint


@dataclass
class CargoTask:
    task_id: str
    cargo_type: str
    pickup: str
    dropoff: str
    priority: int = 0
    status: TaskStatus = TaskStatus.WAITING
    assigned_robot: str | None = None


@dataclass
class RobotState:
    robot_id: str
    waypoint: str
    status: RobotStatus = RobotStatus.IDLE
    available_at: int = 0
    route: List[str] = field(default_factory=list)
    carrying_shelf_id: str | None = None


@dataclass(frozen=True)
class PlannedRoute:
    robot_id: str
    task_id: str
    waypoints: List[str]
    start_time: int
    pickup_time: int
    finish_time: int


@dataclass(frozen=True)
class ShelfGeometry:
    width_cells: int = 3
    depth_cells: int = 3
    safety_margin_cells: int = 0

    def footprint_offsets(self) -> Set[GridPoint]:
        half_width = self.width_cells // 2
        half_depth = self.depth_cells // 2
        margin = self.safety_margin_cells
        return {
            (dx, dy)
            for dx in range(-half_width - margin, half_width + margin + 1)
            for dy in range(-half_depth - margin, half_depth + margin + 1)
        }

    def leg_offsets(self) -> Set[GridPoint]:
        half_width = self.width_cells // 2
        half_depth = self.depth_cells // 2
        return {
            (-half_width, -half_depth),
            (-half_width, half_depth),
            (half_width, -half_depth),
            (half_width, half_depth),
        }


@dataclass(frozen=True)
class Shelf:
    shelf_id: str
    center: GridPoint
    zone: str
    slot: str
    marker_id: int | None = None
    geometry: ShelfGeometry = field(default_factory=ShelfGeometry)


@dataclass(frozen=True)
class ShelfSlot:
    name: str
    center: GridPoint
    zone: str
    occupied_by: str | None = None

    @property
    def is_empty(self) -> bool:
        return self.occupied_by is None


@dataclass(frozen=True)
class ShelfTransportTask:
    task_id: str
    robot_id: str
    target_slot: str
    source_zone: str = "WAIT"


@dataclass(frozen=True)
class ShelfTransportPlan:
    task_id: str
    robot_id: str
    shelf_id: str
    target_slot: str
    approach_path: List[GridPoint]
    carry_path: List[GridPoint]
    start_time: int
    pickup_time: int
    finish_time: int


@dataclass
class FactoryMap:
    waypoints: Dict[str, Waypoint]
    edges: Dict[str, List[str]]

    def neighbors(self, waypoint: str) -> List[str]:
        return self.edges.get(waypoint, [])

    def distance(self, left: str, right: str) -> int:
        ax, ay = self.waypoints[left].point
        bx, by = self.waypoints[right].point
        return abs(ax - bx) + abs(ay - by)
