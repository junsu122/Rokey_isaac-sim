from __future__ import annotations

import heapq
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


WorldPoint = tuple[float, float]


@dataclass(frozen=True)
class NoGoZone:
    name: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    clearance: float = 0.45

    def expanded(self) -> "NoGoZone":
        return NoGoZone(
            name=self.name,
            min_x=self.min_x - self.clearance,
            max_x=self.max_x + self.clearance,
            min_y=self.min_y - self.clearance,
            max_y=self.max_y + self.clearance,
            clearance=0.0,
        )


def load_no_go_zones() -> list[NoGoZone]:
    try:
        from robot_config import IW_HUB_NO_GO_ZONES
    except Exception:
        robot_config_dir = _find_robot_config_dir()
        if robot_config_dir is None:
            return []
        sys.path.insert(0, str(robot_config_dir))
        try:
            from robot_config import IW_HUB_NO_GO_ZONES
        except Exception:
            return []
    return [_zone_from_config(index, item) for index, item in enumerate(IW_HUB_NO_GO_ZONES, start=1)]


def _find_robot_config_dir() -> Path | None:
    roots = [Path(__file__).resolve(), Path.cwd().resolve()]
    for root in roots:
        for parent in (root, *root.parents):
            candidates = [parent / "robot_config.py", parent / "main_isaac" / "robot_config.py"]
            for candidate in candidates:
                if candidate.is_file():
                    return candidate.parent
    return None


def plan_axis_route_around_zones(
    start: WorldPoint,
    target: WorldPoint,
    *,
    axis_order: str,
    zones: Sequence[NoGoZone] | None = None,
    detour_margin: float = 0.6,
) -> tuple[list[WorldPoint], list[str]] | None:
    zones = list(zones if zones is not None else load_no_go_zones())
    if not zones:
        return None

    if _point_blocked(start, zones) or _point_blocked(target, zones):
        raise ValueError(f"Start or target is inside a no-go zone: start={start}, target={target}")

    direct_points, direct_axes = _direct_axis_steps(start, target, axis_order)
    if _route_is_clear(start, direct_points, zones):
        return None

    xs, ys = _candidate_lines(start, target, zones, detour_margin)
    graph_start = start
    graph_target = target
    nodes = [(x, y) for x in xs for y in ys if not _point_blocked((x, y), zones)]
    node_set = set(nodes)
    if graph_start not in node_set:
        nodes.append(graph_start)
        node_set.add(graph_start)
    if graph_target not in node_set:
        nodes.append(graph_target)
        node_set.add(graph_target)

    x_to_ys: dict[float, list[float]] = {}
    y_to_xs: dict[float, list[float]] = {}
    for x, y in nodes:
        x_to_ys.setdefault(x, []).append(y)
        y_to_xs.setdefault(y, []).append(x)
    for values in x_to_ys.values():
        values.sort()
    for values in y_to_xs.values():
        values.sort()

    path = _astar_rectilinear(
        graph_start,
        graph_target,
        x_to_ys=x_to_ys,
        y_to_xs=y_to_xs,
        zones=zones,
        axis_order=axis_order,
    )
    return _path_to_route(start, path)


def route_crosses_no_go(
    start: WorldPoint,
    waypoints: Sequence[WorldPoint],
    zones: Sequence[NoGoZone] | None = None,
) -> bool:
    zones = list(zones if zones is not None else load_no_go_zones())
    return not _route_is_clear(start, waypoints, zones)


def _zone_from_config(index: int, item) -> NoGoZone:
    if isinstance(item, NoGoZone):
        return item
    if isinstance(item, dict):
        if "center" in item and "half_extent" in item:
            cx, cy = item["center"][:2]
            hx, hy = item["half_extent"][:2]
            return NoGoZone(
                name=str(item.get("name", f"NO_GO_{index}")),
                min_x=float(cx) - float(hx),
                max_x=float(cx) + float(hx),
                min_y=float(cy) - float(hy),
                max_y=float(cy) + float(hy),
                clearance=float(item.get("clearance", 0.45)),
            )
        if "center" in item and "size" in item:
            cx, cy = item["center"][:2]
            sx, sy = item["size"][:2]
            return NoGoZone(
                name=str(item.get("name", f"NO_GO_{index}")),
                min_x=float(cx) - float(sx) / 2.0,
                max_x=float(cx) + float(sx) / 2.0,
                min_y=float(cy) - float(sy) / 2.0,
                max_y=float(cy) + float(sy) / 2.0,
                clearance=float(item.get("clearance", 0.45)),
            )
        return NoGoZone(
            name=str(item.get("name", f"NO_GO_{index}")),
            min_x=float(item["min_x"]),
            max_x=float(item["max_x"]),
            min_y=float(item["min_y"]),
            max_y=float(item["max_y"]),
            clearance=float(item.get("clearance", 0.45)),
        )
    name, min_x, max_x, min_y, max_y, *rest = item
    clearance = rest[0] if rest else 0.45
    return NoGoZone(str(name), float(min_x), float(max_x), float(min_y), float(max_y), float(clearance))


def _direct_axis_steps(start: WorldPoint, target: WorldPoint, axis_order: str) -> tuple[list[WorldPoint], list[str]]:
    if axis_order == "xy":
        steps = [((target[0], start[1]), "x"), (target, "y")]
    elif axis_order == "yx":
        steps = [((start[0], target[1]), "y"), (target, "x")]
    else:
        raise ValueError("axis_order must be 'xy' or 'yx'")
    return _deduplicate_route_steps(start, steps)


def _deduplicate_route_steps(
    start: WorldPoint,
    steps: Iterable[tuple[WorldPoint, str]],
) -> tuple[list[WorldPoint], list[str]]:
    waypoints: list[WorldPoint] = []
    axes: list[str] = []
    previous = start
    for point, axis in steps:
        if not _same_point(previous, point):
            waypoints.append(point)
            axes.append(axis)
            previous = point
    return waypoints, axes


def _candidate_lines(
    start: WorldPoint,
    target: WorldPoint,
    zones: Sequence[NoGoZone],
    detour_margin: float,
) -> tuple[list[float], list[float]]:
    xs = {start[0], target[0]}
    ys = {start[1], target[1]}
    for zone in zones:
        expanded = zone.expanded()
        xs.update((expanded.min_x - detour_margin, expanded.max_x + detour_margin))
        ys.update((expanded.min_y - detour_margin, expanded.max_y + detour_margin))
    return sorted(xs), sorted(ys)


def _astar_rectilinear(
    start: WorldPoint,
    target: WorldPoint,
    *,
    x_to_ys: dict[float, list[float]],
    y_to_xs: dict[float, list[float]],
    zones: Sequence[NoGoZone],
    axis_order: str,
) -> list[WorldPoint]:
    start_state = (start, "")
    frontier: list[tuple[tuple[int, int, float], int, tuple[WorldPoint, str]]] = []
    counter = 0
    heapq.heappush(frontier, ((0, 0, 0.0), counter, start_state))
    came_from: dict[tuple[WorldPoint, str], tuple[WorldPoint, str] | None] = {start_state: None}
    cost_so_far: dict[tuple[WorldPoint, str], tuple[int, int, float]] = {start_state: (0, 0, 0.0)}
    target_state: tuple[WorldPoint, str] | None = None

    while frontier:
        _, _, current_state = heapq.heappop(frontier)
        current, previous_axis = current_state
        if _same_point(current, target):
            target_state = current_state
            break

        for neighbor, axis in _neighbors(current, x_to_ys=x_to_ys, y_to_xs=y_to_xs, axis_order=axis_order):
            if not _segment_is_clear(current, neighbor, zones):
                continue
            turns, segments, distance = cost_so_far[current_state]
            next_state = (neighbor, axis)
            new_cost = (
                turns + (0 if previous_axis in {"", axis} else 1),
                segments + 1,
                distance + _distance(current, neighbor),
            )
            if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                cost_so_far[next_state] = new_cost
                counter += 1
                heapq.heappush(frontier, (new_cost, counter, next_state))
                came_from[next_state] = current_state

    if target_state is None:
        raise ValueError(f"No no-go-free route from {start} to {target}")

    path = [target_state[0]]
    current_state = target_state
    while came_from[current_state] is not None:
        current_state = came_from[current_state]
        path.append(current_state[0])
    path.reverse()
    return path


def _neighbors(
    current: WorldPoint,
    *,
    x_to_ys: dict[float, list[float]],
    y_to_xs: dict[float, list[float]],
    axis_order: str,
) -> list[tuple[WorldPoint, str]]:
    x, y = current
    horizontal = [((nx, y), "x") for nx in y_to_xs[y] if not math.isclose(nx, x, abs_tol=1e-6)]
    vertical = [((x, ny), "y") for ny in x_to_ys[x] if not math.isclose(ny, y, abs_tol=1e-6)]
    return horizontal + vertical if axis_order == "xy" else vertical + horizontal


def _path_to_route(start: WorldPoint, path: Sequence[WorldPoint]) -> tuple[list[WorldPoint], list[str]]:
    if len(path) <= 1:
        return [], []

    simplified = [path[0]]
    previous_axis = _axis_between(path[0], path[1])
    for index in range(1, len(path) - 1):
        axis = _axis_between(path[index], path[index + 1])
        if axis != previous_axis:
            simplified.append(path[index])
            previous_axis = axis
    simplified.append(path[-1])

    steps = [(point, _axis_between(previous, point)) for previous, point in zip(simplified, simplified[1:])]
    return _deduplicate_route_steps(start, steps)


def _axis_between(left: WorldPoint, right: WorldPoint) -> str:
    if math.isclose(left[0], right[0], abs_tol=1e-6):
        return "y"
    if math.isclose(left[1], right[1], abs_tol=1e-6):
        return "x"
    raise ValueError(f"Route segment is not axis-aligned: {left} -> {right}")


def _route_is_clear(start: WorldPoint, waypoints: Sequence[WorldPoint], zones: Sequence[NoGoZone]) -> bool:
    previous = start
    for point in waypoints:
        if not _segment_is_clear(previous, point, zones):
            return False
        previous = point
    return True


def _segment_is_clear(start: WorldPoint, end: WorldPoint, zones: Sequence[NoGoZone]) -> bool:
    if _point_blocked(start, zones) or _point_blocked(end, zones):
        return False
    if _same_point(start, end):
        return True
    if not (math.isclose(start[0], end[0], abs_tol=1e-6) or math.isclose(start[1], end[1], abs_tol=1e-6)):
        raise ValueError(f"Route segment is not axis-aligned: {start} -> {end}")
    return all(not _axis_segment_intersects_zone(start, end, zone.expanded()) for zone in zones)


def _axis_segment_intersects_zone(start: WorldPoint, end: WorldPoint, zone: NoGoZone) -> bool:
    x1, y1 = start
    x2, y2 = end
    if math.isclose(y1, y2, abs_tol=1e-6):
        low_x, high_x = sorted((x1, x2))
        return zone.min_y <= y1 <= zone.max_y and _ranges_overlap(low_x, high_x, zone.min_x, zone.max_x)
    low_y, high_y = sorted((y1, y2))
    return zone.min_x <= x1 <= zone.max_x and _ranges_overlap(low_y, high_y, zone.min_y, zone.max_y)


def _point_blocked(point: WorldPoint, zones: Sequence[NoGoZone]) -> bool:
    x, y = point
    return any(zone.expanded().min_x <= x <= zone.expanded().max_x and zone.expanded().min_y <= y <= zone.expanded().max_y for zone in zones)


def _ranges_overlap(a_min: float, a_max: float, b_min: float, b_max: float) -> bool:
    return max(a_min, b_min) <= min(a_max, b_max)


def _same_point(left: WorldPoint, right: WorldPoint) -> bool:
    return math.isclose(left[0], right[0], abs_tol=1e-3) and math.isclose(left[1], right[1], abs_tol=1e-3)


def _distance(left: WorldPoint, right: WorldPoint) -> float:
    return abs(left[0] - right[0]) + abs(left[1] - right[1])
