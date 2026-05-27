"""
draw_warehouse.py
=================
창고 레이아웃을 matplotlib 으로 시각화.
Isaac Sim 없이 standalone 실행 가능.

    python main_isaac/draw_warehouse.py
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import numpy as np

# ── 좌표 계산 (robot_config.py 동일 로직) ─────────────────────────────
def _make_grid(cx, cy, cols=3, rows=3, dx=3.5, dy=3.0, z=0.0):
    xs = [cx + (c - (cols - 1) / 2.0) * dx for c in range(cols)]
    ys = [cy + (r - (rows - 1) / 2.0) * dy for r in range(rows)]
    return [(round(x, 3), round(y, 3)) for y in ys for x in xs]

SECTION_PODS = {
    "A": _make_grid(0.0,  10.0),
    "B": _make_grid(0.0,   0.0),
    "C": _make_grid(0.0, -10.0),
}

SECTION_COLOR = {"A": "#FF9900", "B": "#067D62", "C": "#8B5CF6"}

# 외벽
OUTER_WALL = (-16.65, 16.65, -16.65, 16.65)  # x0,x1,y0,y1

# 조작실 (서쪽, M0609 영역)
MANIP_ROOMS = [
    (-16.5, -8.0,  3.5, 10.0),
    (-16.5, -8.0, -3.5,  3.5),
    (-16.5, -8.0,-10.0, -3.5),
]

# 소팅룸 (동쪽)
SORTING_ROOMS = [
    (16.5, 23.0,  5.1, 15.9),
    (16.5, 23.0, -5.3,  5.3),
    (16.5, 23.0,-15.9, -5.1),
]

# 컨베이어 위치 (minimap.py _CONV_CENTERS)
CONV_CENTERS = [
    (13.5, -10.5), (15.5, -10.5), (17.5, -10.5), (19.5, -10.5),
    (13.5,   0.0), (15.5,   0.0), (17.5,   0.0), (19.5,   0.0),
    (13.5,  10.5), (15.5,  10.5), (17.5,  10.5), (19.5,  10.5),
    (-15.0, 0.0), (-17.0, 0.0), (-19.0, 0.0),
    (-12.8, -2.35), (-12.8,  2.35),
    (-11.3, -7.8),
    (-9.5,  0.0),
]

POD_STACKS = [
    {"name": "PodStack_01", "xy": (-12.8,  9.0)},
    {"name": "PodStack_02", "xy": ( -8.2,  1.5)},
    {"name": "PodStack_03", "xy": ( -9.7, -8.9)},
    {"name": "PodStack_04", "xy": ( 12.0, 14.0)},
]

ROBOTS = [
    {"name": "Drone_01",   "xy": (-16.0, -16.0), "marker": "^", "color": "#00BFFF", "size": 120},
    {"name": "Spot_01",    "xy": (  0.2,   4.8), "marker": "s", "color": "#FFD700", "size": 100},
    {"name": "Spot_02",    "xy": (  0.0,  -5.5), "marker": "s", "color": "#FFD700", "size": 100},
    {"name": "iw_hub_01",  "xy": (-12.8,  14.0), "marker": "D", "color": "#00DCDC", "size": 100},
    {"name": "iw_hub_02",  "xy": ( -6.45,  1.5), "marker": "D", "color": "#00DCDC", "size": 100},
    {"name": "iw_hub_03",  "xy": ( -9.7, -13.0), "marker": "D", "color": "#00DCDC", "size": 100},
    {"name": "M0609_A",    "xy": (-12.07,  7.92), "marker": "*", "color": "#FF6B35", "size": 200},
    {"name": "M0609_B",    "xy": ( -9.45,  0.79), "marker": "*", "color": "#FF6B35", "size": 200},
    {"name": "M0609_C",    "xy": (-10.45, -7.80), "marker": "*", "color": "#FF6B35", "size": 200},
    {"name": "M0609_3way", "xy": (-14.8,   0.5),  "marker": "*", "color": "#FF3333", "size": 200},
]

ARUCO_BOXES = [
    {"xy": (-16.0, -0.4), "color": "#22CC22", "label": "green_id0"},
    {"xy": (-16.0,  0.0), "color": "#CC2222", "label": "red_id1"},
    {"xy": (-16.0,  0.4), "color": "#2244CC", "label": "blue_id2"},
]

FULL_SLOTS = [7, 8, 9]  # 슬롯 7~9: 박스 적재 (마지막 행)

# ── 그리기 ────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 13))
ax.set_facecolor("#1A1A2E")
fig.patch.set_facecolor("#1A1A2E")

# 월드 범위
ax.set_xlim(-20, 25)
ax.set_ylim(-20, 20)
ax.set_aspect("equal")
ax.set_title("Warehouse Layout — Isaac Sim", color="white", fontsize=14, pad=10)
ax.tick_params(colors="gray")
for spine in ax.spines.values():
    spine.set_edgecolor("#333")

# 그리드
for x in range(-20, 26, 2):
    ax.axvline(x, color="#222244", linewidth=0.4, zorder=0)
for y in range(-20, 21, 2):
    ax.axhline(y, color="#222244", linewidth=0.4, zorder=0)
ax.axhline(0, color="#333366", linewidth=0.8, zorder=0)
ax.axvline(0, color="#333366", linewidth=0.8, zorder=0)

# 외벽
x0, x1, y0, y1 = OUTER_WALL
rect = mpatches.Rectangle((x0, y0), x1-x0, y1-y0,
    linewidth=1.5, edgecolor="#888888", facecolor="none", zorder=1)
ax.add_patch(rect)

# 조작실 (서쪽)
for (rx0, rx1, ry0, ry1) in MANIP_ROOMS:
    rect = mpatches.Rectangle((rx0, ry0), rx1-rx0, ry1-ry0,
        linewidth=1, edgecolor="#556655", facecolor="#22332218", zorder=1)
    ax.add_patch(rect)
ax.text(-12.5, 10.5, "Manip\nRoom A", color="#778877", fontsize=5, ha="center", zorder=3)
ax.text(-12.5,  0.5, "Manip\nRoom B", color="#778877", fontsize=5, ha="center", zorder=3)
ax.text(-12.5, -9.5, "Manip\nRoom C", color="#778877", fontsize=5, ha="center", zorder=3)

# 소팅룸 (동쪽)
for (rx0, rx1, ry0, ry1) in SORTING_ROOMS:
    rect = mpatches.Rectangle((rx0, ry0), rx1-rx0, ry1-ry0,
        linewidth=1, edgecolor="#555566", facecolor="#22223318", zorder=1)
    ax.add_patch(rect)
ax.text(19.5, 10.5, "Sort\nRoom A", color="#7777AA", fontsize=5, ha="center", zorder=3)
ax.text(19.5,  0.0, "Sort\nRoom B", color="#7777AA", fontsize=5, ha="center", zorder=3)
ax.text(19.5,-10.5, "Sort\nRoom C", color="#7777AA", fontsize=5, ha="center", zorder=3)

# 컨베이어 노드 (원)
for (cx, cy) in CONV_CENTERS:
    circle = plt.Circle((cx, cy), 0.4, color="#666644", alpha=0.7, zorder=2)
    ax.add_patch(circle)

# 계류장 (USD staging_platform 실제 좌표)
STAGING_PLATFORMS = [
    {"xy": (-14.25,  0.0),  "label": "Stage\n(3way)"},    # comp_in staging
    {"xy": (-11.32,  8.66), "label": "Stage N\n(Sec A)"},  # comp_out_north
    {"xy": ( -8.7,   0.04), "label": "Stage W\n(Sec B)"},  # comp_out_west
    {"xy": (-11.2,  -8.55), "label": "Stage S\n(Sec C)"},  # comp_out_south
]
for st in STAGING_PLATFORMS:
    sx, sy = st["xy"]
    rect = mpatches.FancyBboxPatch(
        (sx - 1.0, sy - 0.75), 2.0, 1.5,
        boxstyle="round,pad=0.1",
        linewidth=1.5, edgecolor="#FFEE88", facecolor="#FFEE8820",
        linestyle="--", zorder=3)
    ax.add_patch(rect)
    ax.text(sx, sy + 1.0, st["label"], color="#FFEE88",
            fontsize=5.5, ha="center", va="bottom", fontweight="bold",
            zorder=4, linespacing=1.3)

# 섹션 배경 박스
SECTION_BOUNDS = {
    "A": (-4.7,  6.5,  9.4, 7.0),  # x, y, w, h  (3×3, dx=3.5, dy=3.0)
    "B": (-4.7, -3.5,  9.4, 7.0),
    "C": (-4.7,-13.5,  9.4, 7.0),
}
for sec, (x, y, w, h) in SECTION_BOUNDS.items():
    col = SECTION_COLOR[sec]
    rect = mpatches.FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.1",
        linewidth=1.5, edgecolor=col,
        facecolor=col + "18", zorder=1,
    )
    ax.add_patch(rect)
    ax.text(x + w + 0.3, y + h / 2, f"Sec {sec}",
            color=col, fontsize=11, fontweight="bold",
            va="center", zorder=5)

# Section Pods
for sec, slots in SECTION_PODS.items():
    col = SECTION_COLOR[sec]
    for i, (px, py) in enumerate(slots, start=1):
        if i == 1:
            circle = plt.Circle((px, py), 0.55, color=col,
                                 fill=False, linestyle="--", linewidth=1, zorder=2)
            ax.add_patch(circle)
            ax.text(px, py, "01", color=col, fontsize=5,
                    ha="center", va="center", zorder=3)
        elif i in FULL_SLOTS:
            rect = mpatches.Rectangle(
                (px - 0.5, py - 0.5), 1.0, 1.0,
                linewidth=1, edgecolor=col, facecolor=col + "99", zorder=2,
            )
            ax.add_patch(rect)
            ax.text(px, py, f"{i:02d}", color="white", fontsize=5,
                    ha="center", va="center", fontweight="bold", zorder=3)
        else:
            circle = plt.Circle((px, py), 0.55, color=col,
                                 fill=True, alpha=0.25, linewidth=1, zorder=2)
            ax.add_patch(circle)
            ax.text(px, py, f"{i:02d}", color=col, fontsize=5,
                    ha="center", va="center", zorder=3)

# 고정 PodStack
for ps in POD_STACKS:
    px, py = ps["xy"]
    rect = mpatches.Rectangle(
        (px - 0.6, py - 0.6), 1.2, 1.2,
        linewidth=1.5, edgecolor="#AAAAAA", facecolor="#44444488", zorder=2,
    )
    ax.add_patch(rect)
    ax.text(px, py - 0.9, ps["name"], color="#AAAAAA", fontsize=5,
            ha="center", va="top", zorder=3)

# ArUco 박스 (컨베이어 위)
for box in ARUCO_BOXES:
    bx, by = box["xy"]
    ax.scatter(bx, by, s=60, color=box["color"], marker="s", zorder=4, linewidths=0)

# 로봇
for r in ROBOTS:
    rx, ry = r["xy"]
    ax.scatter(rx, ry, s=r["size"], color=r["color"],
               marker=r["marker"], zorder=5, edgecolors="white", linewidths=0.5)
    ax.text(rx + 0.3, ry + 0.3, r["name"], color=r["color"],
            fontsize=6, zorder=6)

# Spot 순찰 경로
spot1_wp = [(4.5,5.0),(4.5,14.5),(0.0,14.5),(-4.5,14.5),(-4.5,5.0),(-4.5,-4.7),(0.0,-4.7),(4.5,-4.7)]
spot2_wp = [(4.5,-4.7),(4.5,-14.5),(0.0,-14.5),(-4.5,-14.5),(-4.5,-4.7),(-4.5,4.3),(0.0,4.3),(4.5,4.3)]
for wp, col, name in [(spot1_wp, "#FFD70066", "Spot_01"), (spot2_wp, "#FFD70044", "Spot_02")]:
    xs = [p[0] for p in wp] + [wp[0][0]]
    ys = [p[1] for p in wp] + [wp[0][1]]
    ax.plot(xs, ys, color=col, linewidth=0.8, linestyle="--", zorder=1)

# 범례
legend_items = [
    mpatches.Patch(color="#FF9900", label="Section A"),
    mpatches.Patch(color="#067D62", label="Section B"),
    mpatches.Patch(color="#8B5CF6", label="Section C"),
    plt.Line2D([0],[0], marker="*", color="w", markerfacecolor="#FF6B35", markersize=10, label="M0609"),
    plt.Line2D([0],[0], marker="D", color="w", markerfacecolor="#00DCDC", markersize=8,  label="iw_hub"),
    plt.Line2D([0],[0], marker="s", color="w", markerfacecolor="#FFD700", markersize=8,  label="Spot"),
    plt.Line2D([0],[0], marker="^", color="w", markerfacecolor="#00BFFF", markersize=8,  label="Drone"),
    mpatches.Patch(color="#FF990099", label="Pod (full, slot 10-12)"),
    mpatches.Patch(color="#FF990030", label="Pod (empty)"),
    mpatches.Patch(facecolor="#FFEE8820", edgecolor="#FFEE88", linestyle="--", label="Staging Platform"),
]
ax.legend(handles=legend_items, loc="lower right",
          facecolor="#111122", edgecolor="#444", labelcolor="white", fontsize=7)

ax.set_xlabel("X (m)", color="gray", fontsize=9)
ax.set_ylabel("Y (m)", color="gray", fontsize=9)

plt.tight_layout()
plt.savefig("warehouse_layout.png", dpi=150, bbox_inches="tight",
            facecolor=fig.get_facecolor())
plt.show()
print("[draw_warehouse] 저장 완료: warehouse_layout.png")
