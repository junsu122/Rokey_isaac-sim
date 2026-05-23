from __future__ import annotations

from heapq import heappop, heappush

from smart_factory.models import GridPoint, ShelfGeometry
from smart_factory.occupancy_grid import OccupancyGrid
from smart_factory.shelf_geometry import carried_footprint_cells


def manhattan(left: GridPoint, right: GridPoint) -> int:
    return abs(left[0] - right[0]) + abs(left[1] - right[1])


def astar_grid_path(
    grid: OccupancyGrid,
    start: GridPoint,
    goal: GridPoint,
    *,
    carried_geometry: ShelfGeometry | None = None,
) -> list[GridPoint]:
    if start == goal:
        return [start]

    if not _candidate_is_free(grid, start, carried_geometry):
        raise ValueError(f"Start cell {start} is blocked")
    if not _candidate_is_free(grid, goal, carried_geometry):
        raise ValueError(f"Goal cell {goal} is blocked")

    frontier: list[tuple[int, GridPoint]] = []
    heappush(frontier, (0, start))
    came_from: dict[GridPoint, GridPoint | None] = {start: None}
    cost_so_far: dict[GridPoint, int] = {start: 0}

    while frontier:
        _, current = heappop(frontier)
        if current == goal:
            break

        for next_cell in grid.neighbors(current):
            if not _candidate_is_free(grid, next_cell, carried_geometry):
                continue
            new_cost = cost_so_far[current] + 1
            if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                cost_so_far[next_cell] = new_cost
                priority = new_cost + manhattan(next_cell, goal)
                heappush(frontier, (priority, next_cell))
                came_from[next_cell] = current

    if goal not in came_from:
        raise ValueError(f"No grid path from {start} to {goal}")

    path = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _candidate_is_free(
    grid: OccupancyGrid,
    center: GridPoint,
    carried_geometry: ShelfGeometry | None,
) -> bool:
    if carried_geometry is None:
        return grid.is_free(center)
    return all(grid.is_free(cell) for cell in carried_footprint_cells(center, carried_geometry))
