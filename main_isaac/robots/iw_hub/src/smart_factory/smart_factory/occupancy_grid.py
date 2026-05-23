from __future__ import annotations

from dataclasses import dataclass, field

from smart_factory.models import GridPoint, Shelf
from smart_factory.shelf_geometry import shelf_footprint_cells, shelf_leg_cells


ORTHOGONAL_MOVES: tuple[GridPoint, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))


@dataclass
class OccupancyGrid:
    width: int
    height: int
    blocked_cells: set[GridPoint] = field(default_factory=set)

    def in_bounds(self, cell: GridPoint) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: GridPoint) -> bool:
        return self.in_bounds(cell) and cell not in self.blocked_cells

    def neighbors(self, cell: GridPoint) -> list[GridPoint]:
        x, y = cell
        candidates = [(x + dx, y + dy) for dx, dy in ORTHOGONAL_MOVES]
        return [candidate for candidate in candidates if self.is_free(candidate)]


def build_shelf_occupancy_grid(
    width: int,
    height: int,
    shelves: list[Shelf],
    *,
    target_shelf_id: str | None = None,
    carrying_shelf_id: str | None = None,
) -> OccupancyGrid:
    blocked: set[GridPoint] = set()

    for shelf in shelves:
        if carrying_shelf_id is not None:
            if shelf.shelf_id != carrying_shelf_id:
                blocked.update(shelf_footprint_cells(shelf))
            continue

        if shelf.shelf_id == target_shelf_id:
            # The robot must enter under this shelf, but its four legs remain obstacles.
            blocked.update(shelf_leg_cells(shelf))
        else:
            blocked.update(shelf_footprint_cells(shelf))

    blocked = {(x, y) for x, y in blocked if 0 <= x < width and 0 <= y < height}
    return OccupancyGrid(width=width, height=height, blocked_cells=blocked)
