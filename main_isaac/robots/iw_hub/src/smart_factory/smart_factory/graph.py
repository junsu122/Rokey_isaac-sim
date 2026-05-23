from __future__ import annotations

from heapq import heappop, heappush
from typing import Dict, List

from smart_factory.models import FactoryMap, Waypoint


def build_grid_map(width: int, height: int, blocked: set[tuple[int, int]] | None = None) -> FactoryMap:
    blocked = blocked or set()
    waypoints: Dict[str, Waypoint] = {}
    edges: Dict[str, List[str]] = {}

    for y in range(height):
        for x in range(width):
            if (x, y) in blocked:
                continue
            name = f"N{x}_{y}"
            waypoints[name] = Waypoint(name=name, point=(x, y))

    for name, waypoint in waypoints.items():
        x, y = waypoint.point
        linked: List[str] = []
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            candidate = f"N{nx}_{ny}"
            if candidate in waypoints:
                linked.append(candidate)
        edges[name] = linked

    return FactoryMap(waypoints=waypoints, edges=edges)


def shortest_path(factory_map: FactoryMap, start: str, goal: str) -> List[str]:
    if start == goal:
        return [start]

    frontier: list[tuple[int, str]] = []
    heappush(frontier, (0, start))
    came_from: dict[str, str | None] = {start: None}
    cost_so_far: dict[str, int] = {start: 0}

    while frontier:
        _, current = heappop(frontier)
        if current == goal:
            break

        for next_node in factory_map.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + factory_map.distance(next_node, goal)
                heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    if goal not in came_from:
        raise ValueError(f"No path from {start} to {goal}")

    path = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
