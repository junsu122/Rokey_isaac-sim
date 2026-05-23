from __future__ import annotations

from smart_factory.footprint_reservation import FootprintReservationTable
from smart_factory.grid_planner import astar_grid_path
from smart_factory.models import (
    GridPoint,
    RobotState,
    Shelf,
    ShelfSlot,
    ShelfTransportPlan,
    ShelfTransportTask,
)
from smart_factory.occupancy_grid import build_shelf_occupancy_grid


class ShelfTransportPlanner:
    def __init__(
        self,
        width: int,
        height: int,
        shelves: list[Shelf],
        slots: dict[str, ShelfSlot],
        reservations: FootprintReservationTable | None = None,
    ) -> None:
        self.width = width
        self.height = height
        self.shelves = {shelf.shelf_id: shelf for shelf in shelves}
        self.slots = slots
        self.reservations = reservations or FootprintReservationTable()

    def plan(self, robot: RobotState, robot_cell: GridPoint, task: ShelfTransportTask) -> ShelfTransportPlan:
        target_slot = self.slots[task.target_slot]
        if not target_slot.is_empty:
            raise ValueError(f"Target slot {target_slot.name} is occupied by {target_slot.occupied_by}")

        shelf = self._select_waiting_shelf(task.source_zone)

        approach_grid = build_shelf_occupancy_grid(
            self.width,
            self.height,
            list(self.shelves.values()),
            target_shelf_id=shelf.shelf_id,
        )
        approach_path = astar_grid_path(approach_grid, robot_cell, shelf.center)
        approach_path = self.reservations.plan_with_waits(approach_path, robot.available_at)
        self.reservations.reserve_path(approach_path, robot.available_at)

        pickup_time = robot.available_at + len(approach_path) - 1
        carry_grid = build_shelf_occupancy_grid(
            self.width,
            self.height,
            list(self.shelves.values()),
            carrying_shelf_id=shelf.shelf_id,
        )
        carry_path = astar_grid_path(
            carry_grid,
            shelf.center,
            target_slot.center,
            carried_geometry=shelf.geometry,
        )
        carry_path = self.reservations.plan_with_waits(
            carry_path,
            pickup_time,
            geometry=shelf.geometry,
        )
        self.reservations.reserve_path(carry_path, pickup_time, geometry=shelf.geometry)

        return ShelfTransportPlan(
            task_id=task.task_id,
            robot_id=task.robot_id,
            shelf_id=shelf.shelf_id,
            target_slot=task.target_slot,
            approach_path=approach_path,
            carry_path=carry_path,
            start_time=robot.available_at,
            pickup_time=pickup_time,
            finish_time=pickup_time + len(carry_path) - 1,
        )

    def _select_waiting_shelf(self, source_zone: str) -> Shelf:
        waiting_shelves = [
            shelf for shelf in self.shelves.values() if shelf.zone == source_zone
        ]
        if not waiting_shelves:
            raise ValueError(f"No shelf found in source zone {source_zone}")
        return sorted(waiting_shelves, key=lambda shelf: shelf.shelf_id)[0]
