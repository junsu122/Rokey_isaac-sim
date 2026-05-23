from __future__ import annotations

from dataclasses import dataclass, field

from smart_factory.models import GridPoint, ShelfGeometry
from smart_factory.shelf_geometry import carried_footprint_cells


@dataclass
class FootprintReservationTable:
    cell_slots: dict[int, set[GridPoint]] = field(default_factory=dict)

    def is_free(self, cells: set[GridPoint], time_step: int) -> bool:
        occupied = self.cell_slots.get(time_step, set())
        return cells.isdisjoint(occupied)

    def reserve_path(
        self,
        path: list[GridPoint],
        start_time: int,
        *,
        geometry: ShelfGeometry | None = None,
    ) -> None:
        for offset, center in enumerate(path):
            cells = _occupied_cells(center, geometry)
            self.cell_slots.setdefault(start_time + offset, set()).update(cells)

    def plan_with_waits(
        self,
        path: list[GridPoint],
        start_time: int,
        *,
        geometry: ShelfGeometry | None = None,
    ) -> list[GridPoint]:
        if not path:
            return []

        planned = [path[0]]
        index = 0
        time_step = start_time

        while index < len(path) - 1:
            target = path[index + 1]
            cells = _occupied_cells(target, geometry)
            if self.is_free(cells, time_step + 1):
                planned.append(target)
                index += 1
            else:
                planned.append(path[index])
            time_step += 1

        return planned


def _occupied_cells(center: GridPoint, geometry: ShelfGeometry | None) -> set[GridPoint]:
    if geometry is None:
        return {center}
    return carried_footprint_cells(center, geometry)
