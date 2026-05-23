from __future__ import annotations

from smart_factory.models import GridPoint, Shelf, ShelfGeometry


def translate_offsets(center: GridPoint, offsets: set[GridPoint]) -> set[GridPoint]:
    cx, cy = center
    return {(cx + dx, cy + dy) for dx, dy in offsets}


def shelf_footprint_cells(shelf: Shelf, geometry: ShelfGeometry | None = None) -> set[GridPoint]:
    active_geometry = geometry or shelf.geometry
    return translate_offsets(shelf.center, active_geometry.footprint_offsets())


def shelf_leg_cells(shelf: Shelf) -> set[GridPoint]:
    return translate_offsets(shelf.center, shelf.geometry.leg_offsets())


def carried_footprint_cells(center: GridPoint, geometry: ShelfGeometry) -> set[GridPoint]:
    return translate_offsets(center, geometry.footprint_offsets())
