from __future__ import annotations

from pathlib import Path
import sys


MAP_RESOLUTION = 0.1
MAP_MIN_X = -17.0
MAP_MIN_Y = -17.0
MAP_MAX_X = 17.0
MAP_MAX_Y = 17.0


def _repo_main_isaac_dir() -> Path:
    return Path(__file__).resolve().parents[5]


def _zone_bounds(zone: dict) -> tuple[float, float, float, float]:
    clearance = float(zone.get("clearance", 0.0))
    if "center" in zone and "half_extent" in zone:
        cx, cy, *_ = zone["center"]
        hx, hy, *_ = zone["half_extent"]
        return (
            float(cx) - float(hx) - clearance,
            float(cx) + float(hx) + clearance,
            float(cy) - float(hy) - clearance,
            float(cy) + float(hy) + clearance,
        )
    return (
        float(zone["min_x"]) - clearance,
        float(zone["max_x"]) + clearance,
        float(zone["min_y"]) - clearance,
        float(zone["max_y"]) + clearance,
    )


def _world_to_cell(x: float, y: float) -> tuple[int, int]:
    col = int((x - MAP_MIN_X) / MAP_RESOLUTION)
    row_from_bottom = int((y - MAP_MIN_Y) / MAP_RESOLUTION)
    return col, row_from_bottom


def _mark_rect(grid: list[list[int]], bounds: tuple[float, float, float, float]) -> None:
    min_x, max_x, min_y, max_y = bounds
    width = len(grid[0])
    height = len(grid)
    min_col, min_row = _world_to_cell(min_x, min_y)
    max_col, max_row = _world_to_cell(max_x, max_y)
    min_col = max(0, min(width - 1, min_col))
    max_col = max(0, min(width - 1, max_col))
    min_row = max(0, min(height - 1, min_row))
    max_row = max(0, min(height - 1, max_row))
    for row_from_bottom in range(min_row, max_row + 1):
        image_row = height - 1 - row_from_bottom
        for col in range(min_col, max_col + 1):
            grid[image_row][col] = 0


def main() -> None:
    main_isaac_dir = _repo_main_isaac_dir()
    if str(main_isaac_dir) not in sys.path:
        sys.path.insert(0, str(main_isaac_dir))

    from robot_config import IW_HUB_NO_GO_ZONES

    config_dir = Path(__file__).resolve().parents[1] / "config"
    pgm_path = config_dir / "iw_hub_warehouse_map.pgm"
    yaml_path = config_dir / "iw_hub_warehouse_map.yaml"

    width = int(round((MAP_MAX_X - MAP_MIN_X) / MAP_RESOLUTION))
    height = int(round((MAP_MAX_Y - MAP_MIN_Y) / MAP_RESOLUTION))
    grid = [[254 for _ in range(width)] for _ in range(height)]

    for zone in IW_HUB_NO_GO_ZONES:
        _mark_rect(grid, _zone_bounds(zone))

    with pgm_path.open("w", encoding="ascii") as f:
        f.write("P2\n")
        f.write(f"{width} {height}\n")
        f.write("255\n")
        for row in grid:
            f.write(" ".join(str(value) for value in row))
            f.write("\n")

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                f"resolution: {MAP_RESOLUTION}",
                f"origin: [{MAP_MIN_X}, {MAP_MIN_Y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(f"wrote {yaml_path}")
    print(f"wrote {pgm_path}")


if __name__ == "__main__":
    main()
