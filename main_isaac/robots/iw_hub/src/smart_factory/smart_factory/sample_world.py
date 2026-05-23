from __future__ import annotations

from smart_factory.graph import build_grid_map
from smart_factory.models import (
    CargoTask,
    FactoryMap,
    MarkerOffset,
    MarkerType,
    Pose2D,
    RobotState,
    Shelf,
    ShelfSlot,
    Waypoint,
)
from smart_factory.robot_defaults import default_robot_id


def make_sample_factory_map() -> FactoryMap:
    factory_map = build_grid_map(width=7, height=5, blocked={(3, 1), (3, 2), (3, 3)})
    aliases = {
        "IN_A": (0, 0),
        "IN_B": (0, 4),
        "SORT_RED": (6, 0),
        "SORT_BLUE": (6, 4),
        "STACK": (5, 2),
        "CHARGE": (1, 2),
    }

    for name, point in aliases.items():
        node_name = f"N{point[0]}_{point[1]}"
        factory_map.waypoints[name] = Waypoint(name=name, point=point)
        factory_map.edges[name] = list(factory_map.edges[node_name])
        for neighbor in factory_map.edges[node_name]:
            factory_map.edges[neighbor].append(name)

    return factory_map


def make_sample_robots() -> list[RobotState]:
    return [
        RobotState(robot_id=default_robot_id(1), waypoint="CHARGE"),
        RobotState(robot_id=default_robot_id(2), waypoint="N1_4"),
    ]


def make_sample_tasks() -> list[CargoTask]:
    return [
        CargoTask(task_id="box_001", cargo_type="red", pickup="IN_A", dropoff="SORT_RED", priority=2),
        CargoTask(task_id="box_002", cargo_type="blue", pickup="IN_B", dropoff="SORT_BLUE", priority=1),
        CargoTask(task_id="box_003", cargo_type="mixed", pickup="IN_A", dropoff="STACK", priority=0),
    ]


def make_shelf_transport_world() -> tuple[int, int, list[Shelf], dict[str, ShelfSlot]]:
    width = 16
    height = 13
    shelves = [
        Shelf(shelf_id="shelf_a_1", center=(3, 2), zone="A", slot="A-1", marker_id=201),
        Shelf(shelf_id="shelf_b_1", center=(3, 6), zone="B", slot="B-1", marker_id=301),
        Shelf(shelf_id="shelf_c_1", center=(3, 10), zone="C", slot="C-1", marker_id=401),
    ]
    slots = {
        "A-1": ShelfSlot(name="A-1", center=(3, 2), zone="A", occupied_by="shelf_a_1"),
        "A-2": ShelfSlot(name="A-2", center=(3, 4), zone="A"),
        "A-3": ShelfSlot(name="A-3", center=(3, 6), zone="A"),
        "B-1": ShelfSlot(name="B-1", center=(3, 6), zone="B", occupied_by="shelf_b_1"),
        "B-2": ShelfSlot(name="B-2", center=(3, 8), zone="B"),
        "B-3": ShelfSlot(name="B-3", center=(3, 10), zone="B"),
        "C-1": ShelfSlot(name="C-1", center=(3, 10), zone="C", occupied_by="shelf_c_1"),
        "C-2": ShelfSlot(name="C-2", center=(3, 8), zone="C"),
        "C-3": ShelfSlot(name="C-3", center=(3, 6), zone="C"),
        "D-1": ShelfSlot(name="D-1", center=(12, 2), zone="D"),
        "D-2": ShelfSlot(name="D-2", center=(12, 4), zone="D"),
        "D-3": ShelfSlot(name="D-3", center=(12, 6), zone="D"),
        "E-1": ShelfSlot(name="E-1", center=(12, 6), zone="E"),
        "E-2": ShelfSlot(name="E-2", center=(12, 8), zone="E"),
        "E-3": ShelfSlot(name="E-3", center=(12, 10), zone="E"),
        "F-1": ShelfSlot(name="F-1", center=(9, 2), zone="F"),
        "F-2": ShelfSlot(name="F-2", center=(9, 6), zone="F"),
        "F-3": ShelfSlot(name="F-3", center=(9, 10), zone="F"),
    }
    return width, height, shelves, slots


def make_marker_offsets() -> dict[str, MarkerOffset]:
    return {
        "D-1": MarkerOffset(
            target_id="D-1",
            marker_id=801,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
        "D-2": MarkerOffset(
            target_id="D-2",
            marker_id=802,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
        "E-2": MarkerOffset(
            target_id="E-2",
            marker_id=902,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
        "F-2": MarkerOffset(
            target_id="F-2",
            marker_id=1002,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
    }
