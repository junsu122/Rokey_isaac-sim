"""
warehouse_layout.py
===================
창고 레이아웃 시각화 (matplotlib).
실행: python3 warehouse_layout.py
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np

# ── 좌표 데이터 ──────────────────────────────────────────────────────────

SECTIONS = {
    "A": {"x": (-4.9, 4.9), "y": ( 6.1, 13.9), "cy": 10.0},
    "B": {"x": (-4.9, 4.9), "y": ( -3.9,  3.9), "cy": -0.2},
    "C": {"x": (-4.9, 4.9), "y": (-13.9, -6.1), "cy": -10.0},
}

def make_grid(cx, cy, cols=3, rows=3, dx=3.5, dy=3.0):
    xs = [cx + (c - (cols - 1) / 2.0) * dx for c in range(cols)]
    ys = [cy + (r - (rows - 1) / 2.0) * dy for r in range(rows)]
    return [(round(x, 3), round(y, 3)) for y in ys for x in xs]

# cols=3 (x 방향), rows=3 (y 방향),  3×3 = 9 슬롯
# dx=3.5, dy=2.8  → pod 사이 간격 넓힘
# Sec A pods: x=[-3.5, 0.0, 3.5]  y=[7.2, 10.0, 12.8]
# Sec B pods: x=[-3.5, 0.0, 3.5]  y=[-3.0, -0.2,  2.6]
# Sec C pods: x=[-3.5, 0.0, 3.5]  y=[-12.8,-10.0, -7.2]
SECTION_PODS = {
    "A": make_grid(0.0,  10.0),
    "B": make_grid(0.0,  -0.2),
    "C": make_grid(0.0, -10.0),
}

POD_STACKS = [
    {"name": "PS01", "xy": (-12.8,  9.0)},
    {"name": "PS02", "xy": ( -8.2,  1.5)},
    {"name": "PS03", "xy": ( -9.7, -8.9)},
    {"name": "PS04", "xy": ( 12.0, 14.0)},
]

IW_HUBS = [
    {"name": "Hub-01", "xy": (-12.8,  9.2), "yaw":  90, "section": "A", "home_y":  9.0},
    {"name": "Hub-02", "xy": ( -6.45, 1.5), "yaw":   0, "section": "B", "home_y":  1.5},
    {"name": "Hub-03", "xy": ( -9.7, -8.6), "yaw":  90, "section": "C", "home_y": -8.9},
]

M0609 = [
    {"name": "M0609_A",    "xy": (-12.07,  7.92)},
    {"name": "M0609_B",    "xy": ( -9.45,  0.79)},
    {"name": "M0609_C",    "xy": (-10.45, -7.80)},
    {"name": "M0609_3way", "xy": (-14.8,   0.5 )},
]

SPOTS = [
    {"name": "Spot-01", "xy": ( 0.2,  4.8)},
    {"name": "Spot-02", "xy": ( 0.0, -5.5)},
]

DRONE = {"name": "Drone-01", "xy": (-16.0, -16.0)}

DROP_LINE_X  = -7.5
SECTION_WEST = -4.9   # 섹션 서쪽 경계 (IW Hub 진입/진출 경계)
AISLE_X      = -3.5   # 섹션 내부 세로 통로 (서쪽 pod 열 왼쪽)

# 섹션별 진입/진출 y 좌표 (Hub home y 기준)
SECTION_EAST = 4.9
# entry/exit y = h_corr[0] (행1-2 사이 수평 통로)
SECTION_NAV = {
    "A": {"entry_xy": (SECTION_WEST,  8.5), "exit_xy": (SECTION_EAST,  8.5),
          "pod_rows_y": [7.0, 10.0, 13.0]},
    "B": {"entry_xy": (SECTION_WEST, -1.7), "exit_xy": (SECTION_EAST, -1.7),
          "pod_rows_y": [-3.2, -0.2,  2.8]},
    "C": {"entry_xy": (SECTION_WEST, -8.5), "exit_xy": (SECTION_EAST, -8.5),
          "pod_rows_y": [-13.0, -10.0, -7.0]},
}

# ── 그리기 ───────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(22, 20))
ax.set_aspect("equal")
ax.set_facecolor("#1a1a2e")
fig.patch.set_facecolor("#0f0f1a")

# ── 1) 배경 격자 (1m 간격) ───────────────────────────────────────────────
for x in np.arange(-18, 17, 1.0):
    ax.axvline(x, color="#3a3a6a", linewidth=0.6, zorder=0)
for y in np.arange(-18, 18, 1.0):
    ax.axhline(y, color="#3a3a6a", linewidth=0.6, zorder=0)
# 굵은 격자 (5m 간격)
for x in np.arange(-15, 16, 5.0):
    ax.axvline(x, color="#6666aa", linewidth=1.2, zorder=0)
for y in np.arange(-15, 16, 5.0):
    ax.axhline(y, color="#6666aa", linewidth=1.2, zorder=0)

ax.set_xticks(range(-18, 17, 1))
ax.set_yticks(range(-18, 18, 1))
ax.tick_params(colors="#aaaacc", labelsize=7)
for spine in ax.spines.values():
    spine.set_edgecolor("#555588")

# ── 2) 창고 외벽 ─────────────────────────────────────────────────────────
warehouse_rect = mpatches.FancyBboxPatch(
    (-18, -18), 34, 36,
    boxstyle="square,pad=0", linewidth=2,
    edgecolor="#aaaacc", facecolor="#16213e", zorder=1
)
ax.add_patch(warehouse_rect)

# ── 3) 섹션 경계 (Red Line) ──────────────────────────────────────────────
for sec, s in SECTIONS.items():
    x0, x1 = s["x"]
    y0, y1 = s["y"]
    rect = mpatches.Rectangle(
        (x0, y0), x1 - x0, y1 - y0,
        linewidth=2.5, edgecolor="#ff3333", facecolor="#ff333315", zorder=2
    )
    ax.add_patch(rect)
    ax.text((x0 + x1) / 2, (y0 + y1) / 2, f"Section {sec}",
            ha="center", va="center", fontsize=18, fontweight="bold",
            color="#ff5555", alpha=0.25, zorder=2)

COL_XS = [-3.5, 0.0, 3.5]

# ── 4) 섹션 내부 격자 (pod 간격: 2.8m × 2.0m) ───────────────────────────
for sec, s in SECTIONS.items():
    x0, x1 = s["x"]
    y0, y1 = s["y"]
    nav = SECTION_NAV[sec]
    rows = nav["pod_rows_y"]
    # 수직 통로 음영 (pod 열 사이)
    v_corridors = [
        (x0, COL_XS[0] - 0.4),           # 서쪽 벽 ~ col1 왼쪽
        (COL_XS[0]+0.4, COL_XS[1]-0.4),  # col1 오른쪽 ~ col2 왼쪽
        (COL_XS[1]+0.4, COL_XS[2]-0.4),  # col2 오른쪽 ~ col3 왼쪽
        (COL_XS[2]+0.4, x1),             # col3 오른쪽 ~ 동쪽 벽
    ]
    for vx0, vx1 in v_corridors:
        ax.fill_betweenx([y0, y1], vx0, vx1, color="#ffffff07", zorder=1)
    # 수평 통로 음영 (pod 행 사이)
    h_corridors = [
        (y0, rows[0]-0.4),               # 남쪽 벽 ~ row1 아래
        (rows[0]+0.4, rows[1]-0.4),      # row1 위 ~ row2 아래
        (rows[1]+0.4, rows[2]-0.4),      # row2 위 ~ row3 아래
        (rows[2]+0.4, y1),               # row3 위 ~ 북쪽 벽
    ]
    for hy0, hy1 in h_corridors:
        ax.fill_between([x0, x1], hy0, hy1, color="#ffffff07", zorder=1)
    # pod 열 점선
    COL_XS_draw = [-3.5, 0.0, 3.5]
    for col_x in COL_XS_draw:
        ax.plot([col_x, col_x], [y0, y1],
                color="#5599ff", linewidth=1.2, linestyle=":", alpha=0.6, zorder=2)
    # pod 행 점선
    for row_y in rows:
        ax.plot([x0, x1], [row_y, row_y],
                color="#5599ff", linewidth=1.2, linestyle=":", alpha=0.5, zorder=2)
    # h_corr 통로 중심선
    h_corr = [(rows[i]+rows[i+1])/2 for i in range(len(rows)-1)]
    for hc in h_corr:
        ax.plot([x0, x1], [hc, hc],
                color="#ffffff22", linewidth=1.0, linestyle="-", zorder=2)

# ── 5) 섹션 내부 Aisle (세로 통로: x=AISLE_X) ───────────────────────────
for sec, s in SECTIONS.items():
    y0, y1 = s["y"]
    ax.plot([AISLE_X, AISLE_X], [y0 + 0.1, y1 - 0.1],
            color="#00ff8855", linewidth=2.0, linestyle="--", zorder=3)

ax.text(AISLE_X - 0.15, 0, "Aisle\nx=-3.5", ha="right", va="center",
        fontsize=7, color="#00ff8888", rotation=90, zorder=4)

# ── 6) Drop Line ─────────────────────────────────────────────────────────
ax.axvline(x=DROP_LINE_X, color="#ffaa00", linewidth=2.0,
           linestyle="--", zorder=4)
ax.text(DROP_LINE_X - 0.2, 16.8, "DROP LINE\nx=-7.5",
        ha="center", va="top", fontsize=7.5, color="#ffaa00",
        fontweight="bold", zorder=5)

# ── 7) 컨베이어 ──────────────────────────────────────────────────────────
conveyor = mpatches.Rectangle(
    (-17.5, -2.5), 1.5, 5.0,
    linewidth=1.5, edgecolor="#00ccff", facecolor="#00ccff22", zorder=2
)
ax.add_patch(conveyor)
ax.text(-16.75, 0, "CONV\n(BoxSpawn)", ha="center", va="center",
        fontsize=7, color="#00ccff", zorder=5)

# ── 8) Pod 슬롯 ──────────────────────────────────────────────────────────
drone_pickup_slots = {7, 8, 9}   # 3×3 마지막 행

for sec, pods in SECTION_PODS.items():
    for i, (px, py) in enumerate(pods, start=1):
        if i == 1:
            rect = mpatches.Rectangle(
                (px - 0.5, py - 0.5), 1.0, 1.0,
                linewidth=1.5, edgecolor="#777777",
                facecolor="none", linestyle="--", zorder=3
            )
            ax.add_patch(rect)
            ax.text(px, py, f"S{sec}\n#{i:02d}\n[IW]", ha="center", va="center",
                    fontsize=5.5, color="#777777", zorder=4)
        elif i in drone_pickup_slots:
            circle = plt.Circle((px, py), 0.45, color="#aa44ff",
                                 alpha=0.85, zorder=3)
            ax.add_patch(circle)
            ax.text(px, py, f"S{sec}\n#{i:02d}\n[D]", ha="center", va="center",
                    fontsize=5.5, color="white", fontweight="bold", zorder=4)
        else:
            rect = mpatches.FancyBboxPatch(
                (px - 0.45, py - 0.45), 0.9, 0.9,
                boxstyle="round,pad=0.05", linewidth=1.2,
                edgecolor="#4488ff", facecolor="#4488ff33", zorder=3
            )
            ax.add_patch(rect)
            ax.text(px, py, f"S{sec}\n#{i:02d}", ha="center", va="center",
                    fontsize=5.5, color="#88aaff", zorder=4)

# ── 9) IW Hub 진입/진출 지점 (Entry / Exit markers) ─────────────────────
for sec, nav in SECTION_NAV.items():
    ex, ey   = nav["entry_xy"]
    ox, oy   = nav["exit_xy"]

    # Entry 삼각형 — 서쪽 경계, 동쪽(+x)을 향함
    entry_tri = plt.Polygon(
        [[ex, ey + 0.5], [ex + 0.8, ey], [ex, ey - 0.5]],
        closed=True, edgecolor="#00ffcc", facecolor="#00ffcc99",
        linewidth=1.5, zorder=6
    )
    ax.add_patch(entry_tri)
    ax.text(ex - 0.15, ey, f"IN\n{sec}", ha="right", va="center",
            fontsize=7, color="#00ffcc", fontweight="bold", zorder=7)

    # Exit 삼각형 — 동쪽 경계, 동쪽(+x)을 향함 (빠져나가는 방향)
    exit_tri = plt.Polygon(
        [[ox - 0.8, oy + 0.5], [ox, oy], [ox - 0.8, oy - 0.5]],
        closed=True, edgecolor="#ff6644", facecolor="#ff664499",
        linewidth=1.5, zorder=6
    )
    ax.add_patch(exit_tri)
    ax.text(ox + 0.15, oy, f"OUT\n{sec}", ha="left", va="center",
            fontsize=7, color="#ff6644", fontweight="bold", zorder=7)

    # 서쪽 경계 강조
    y0, y1 = SECTIONS[sec]["y"]
    ax.plot([SECTION_WEST, SECTION_WEST], [y0, y1],
            color="#00ffcc44", linewidth=3.0, zorder=3)
    # 동쪽 경계 강조
    ax.plot([SECTION_EAST, SECTION_EAST], [y0, y1],
            color="#ff664444", linewidth=3.0, zorder=3)

# ── 10) IW Hub 홈 → Entry → 각 열 도킹 경로 ——————————————————————————————————
# pod 열 x 좌표
COL_XS     = [-3.5,  0.0,  3.5]
# 수직 통로 (pod 열 사이 중간)
V_CORR     = [(COL_XS[i] + COL_XS[i+1]) / 2 for i in range(len(COL_XS)-1)]  # [-1.75, 1.75]

COL_COLORS = ["#00ffcc", "#ffee44", "#ff44cc"]

for hub in IW_HUBS:
    hx, hy = hub["xy"]
    sec    = hub["section"]
    nav    = SECTION_NAV[sec]
    rows   = nav["pod_rows_y"]
    ox, oy = nav["exit_xy"]

    # 행 사이 수평 통로 y (행 중간)
    h_corr = [(rows[i] + rows[i+1]) / 2 for i in range(len(rows)-1)]
    travel_y = h_corr[0]   # 주 수평 통로

    # 홈 → Drop Line → 진입 통로
    ax.plot([hx, DROP_LINE_X, DROP_LINE_X, SECTION_WEST],
            [hy, hy,          travel_y,    travel_y],
            color="#ffffff33", lw=1.4, linestyle="-.", zorder=3)

    # Col1 (x=-3.5): 진입 → 서쪽 벽과 col1 사이 통로(vc0)에서 y 조정 → 도킹
    ty1 = rows[1]
    vc0 = (SECTION_WEST + COL_XS[0]) / 2   # -4.2: 경계~col1 사이 통로
    # 수평 진입: SECTION_WEST → vc0
    ax.plot([SECTION_WEST, vc0], [travel_y, travel_y],
            color=COL_COLORS[0], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    # 수직 이동: travel_y → ty1 (섹션 내부에서 명확히 보임)
    ax.plot([vc0, vc0], [travel_y, ty1],
            color=COL_COLORS[0], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    ax.annotate("", xy=(vc0, ty1), xytext=(vc0, travel_y),
                arrowprops=dict(arrowstyle="->", color=COL_COLORS[0],
                                lw=1.5, mutation_scale=12), zorder=4)
    # 도킹: vc0 → col1
    ax.annotate("", xy=(COL_XS[0], ty1), xytext=(vc0, ty1),
                arrowprops=dict(arrowstyle="->", color=COL_COLORS[0],
                                lw=1.5, mutation_scale=12), zorder=4)
    ax.text(vc0 - 0.1, (travel_y + ty1) / 2, "C1",
            ha="right", va="center", fontsize=7.5,
            color=COL_COLORS[0], fontweight="bold", zorder=5)

    # Col2 (x=0): h_corr[0] 동진 → V_CORR[0]=-1.75 → y 조정 → 도킹
    ty2 = rows[2]
    ax.plot([SECTION_WEST, V_CORR[0]], [travel_y, travel_y],
            color=COL_COLORS[1], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    ax.plot([V_CORR[0], V_CORR[0]], [travel_y, ty2],
            color=COL_COLORS[1], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    ax.annotate("", xy=(COL_XS[1], ty2), xytext=(V_CORR[0], ty2),
                arrowprops=dict(arrowstyle="->", color=COL_COLORS[1],
                                lw=1.5, mutation_scale=12), zorder=4)
    ax.text(V_CORR[0], (travel_y + ty2) / 2 + 0.2, "C2",
            ha="center", va="center", fontsize=7.5,
            color=COL_COLORS[1], fontweight="bold", zorder=5)

    # Col3 (x=3.5): h_corr[0] 동진 → V_CORR[1]=1.75 → y 조정 → 도킹
    ty3 = rows[0]
    ax.plot([SECTION_WEST, V_CORR[1]], [travel_y, travel_y],
            color=COL_COLORS[2], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    ax.plot([V_CORR[1], V_CORR[1]], [travel_y, ty3],
            color=COL_COLORS[2], lw=1.5, linestyle="--", alpha=0.9, zorder=3)
    ax.annotate("", xy=(COL_XS[2], ty3), xytext=(V_CORR[1], ty3),
                arrowprops=dict(arrowstyle="->", color=COL_COLORS[2],
                                lw=1.5, mutation_scale=12), zorder=4)
    ax.text(V_CORR[1], (travel_y + ty3) / 2 - 0.2, "C3",
            ha="center", va="center", fontsize=7.5,
            color=COL_COLORS[2], fontweight="bold", zorder=5)

    # 탈쳙: V_CORR[1] → h_corr[0] 북귀 → SECTION_EAST(OUT)
    ax.plot([V_CORR[1], SECTION_EAST], [travel_y, travel_y],
            color="#ff664466", lw=1.5, linestyle="-.", zorder=3)
    ax.annotate("", xy=(SECTION_EAST, oy), xytext=(SECTION_EAST, travel_y),
                arrowprops=dict(arrowstyle="->", color="#ff664499",
                                lw=1.5, mutation_scale=11), zorder=3)

# ── 11) IW Hub 본체 ──────────────────────────────────────────────────────
for hub in IW_HUBS:
    hx, hy = hub["xy"]
    yaw_rad = np.radians(hub["yaw"])
    dx_arr = np.cos(yaw_rad) * 1.0
    dy_arr = np.sin(yaw_rad) * 1.0

    hub_rect = mpatches.FancyBboxPatch(
        (hx - 0.4, hy - 0.3), 0.8, 0.6,
        boxstyle="round,pad=0.05", linewidth=2,
        edgecolor="#00ff88", facecolor="#00ff8844", zorder=7
    )
    ax.add_patch(hub_rect)
    ax.annotate("", xy=(hx + dx_arr, hy + dy_arr), xytext=(hx, hy),
                arrowprops=dict(arrowstyle="->", color="#00ff88",
                                lw=1.8, mutation_scale=14), zorder=8)
    ax.text(hx, hy - 0.75, hub["name"], ha="center", va="top",
            fontsize=7, color="#00ff88", fontweight="bold", zorder=8)

# ── 12) 외부 Pod Stacks ──────────────────────────────────────────────────
for ps in POD_STACKS:
    px, py = ps["xy"]
    circle = plt.Circle((px, py), 0.6, color="#ffcc00", alpha=0.9, zorder=6)
    ax.add_patch(circle)
    ax.text(px, py + 0.85, ps["name"], ha="center", va="bottom",
            fontsize=7.5, color="#ffcc00", fontweight="bold", zorder=7)

# ── 13) M0609 팔 ─────────────────────────────────────────────────────────
for arm in M0609:
    ax_pt, ay = arm["xy"]
    triangle = plt.Polygon(
        [[ax_pt, ay + 0.7], [ax_pt - 0.5, ay - 0.4], [ax_pt + 0.5, ay - 0.4]],
        closed=True, edgecolor="#ff8800", facecolor="#ff880055",
        linewidth=2, zorder=6
    )
    ax.add_patch(triangle)
    ax.text(ax_pt, ay - 0.75, arm["name"], ha="center", va="top",
            fontsize=6.5, color="#ff8800", fontweight="bold", zorder=7)

# ── 14) Spot ─────────────────────────────────────────────────────────────
for spot in SPOTS:
    sx, sy = spot["xy"]
    circle = plt.Circle((sx, sy), 0.5, color="#ffdd44", alpha=0.85, zorder=6)
    ax.add_patch(circle)
    ax.text(sx, sy + 0.65, spot["name"], ha="center", va="bottom",
            fontsize=7, color="#ffdd44", zorder=7)

spot1_path = [(4.5, 5.0),(4.5, 14.5),(0.0, 14.5),(-4.5, 14.5),
              (-4.5, 5.0),(-4.5, -4.7),(0.0, -4.7),(4.5, -4.7)]
xs_p, ys_p = zip(*spot1_path + [spot1_path[0]])
ax.plot(xs_p, ys_p, color="#ffdd4433", linewidth=0.8, linestyle=":", zorder=2)

spot2_path = [(4.5,-4.7),(4.5,-14.5),(0.0,-14.5),(-4.5,-14.5),
              (-4.5,-4.7),(-4.5,4.3),(0.0,4.3),(4.5,4.3)]
xs_p2, ys_p2 = zip(*spot2_path + [spot2_path[0]])
ax.plot(xs_p2, ys_p2, color="#ffdd4422", linewidth=0.8, linestyle=":", zorder=2)

# ── 15) 드론 ─────────────────────────────────────────────────────────────
dx_d, dy_d = DRONE["xy"]
ax.plot(dx_d, dy_d, "^", markersize=14, color="#cc88ff",
        markeredgecolor="#aa66dd", markeredgewidth=1.5, zorder=7)
ax.text(dx_d, dy_d - 0.9, "Drone-01\nspawn", ha="center", va="top",
        fontsize=6.5, color="#cc88ff", zorder=7)

# ── 16) Goal Zone ─────────────────────────────────────────────────────────
goal_zones = [("GZ_A\nid0", -3.0, 15.15),
              ("GZ_B\nid1",  0.0, 15.15),
              ("GZ_C\nid2",  3.0, 15.15)]
for label, gx, gy in goal_zones:
    rect = mpatches.Rectangle(
        (gx - 1.5, gy - 1.5), 3.0, 3.0,
        linewidth=2, edgecolor="#00ff44", facecolor="#00ff4422", zorder=3
    )
    ax.add_patch(rect)
    ax.text(gx, gy, label, ha="center", va="center",
            fontsize=7, color="#00ff44", fontweight="bold", zorder=4)

# ── 17) 원점 ─────────────────────────────────────────────────────────────
ax.plot(0, 0, "+", color="white", markersize=12, markeredgewidth=2, zorder=9)
ax.text(0.2, 0.3, "Origin", fontsize=7, color="white", zorder=9)

# ── 18) 범례 ─────────────────────────────────────────────────────────────
legend_elements = [
    mpatches.Patch(facecolor="#ff333344", edgecolor="#ff3333",
                   label="Section boundary (Red Line)"),
    plt.Line2D([0],[0], color="#4488ff44", lw=1, linestyle=":",
               label="Pod grid lines (2.8m x / 2.0m y)"),
    plt.Line2D([0],[0], color="#00ff8855", lw=2, linestyle="--",
               label="IW Hub aisle  x=-3.5"),
    mpatches.Patch(facecolor="#4488ff33", edgecolor="#4488ff",
                   label="Pod slot (normal)"),
    mpatches.Patch(facecolor="#aa44ff", edgecolor="#aa44ff",
                   label="Pod slot #10~12  (drone pickup)"),
    mpatches.Patch(facecolor="none", edgecolor="#777777", linestyle="--",
                   label="Slot #01  (IW Hub delivery reserved)"),
    mpatches.Patch(facecolor="#ffcc0099", edgecolor="#ffcc00",
                   label="Pod Stack (external)"),
    mpatches.Patch(facecolor="#00ff8844", edgecolor="#00ff88",
                   label="IW Hub (arrow = heading)"),
    plt.Line2D([0],[0], color="#00ffcc44", lw=1.5, linestyle="-.",
               label="IW Hub route  home -> entry"),
    plt.Line2D([0],[0], color="#00ffcc", lw=1.4, linestyle="--",
               label="Col 1 dock path  (direct from west aisle)"),
    plt.Line2D([0],[0], color="#ffee44", lw=1.4, linestyle="--",
               label="Col 2 dock path  (via slot-01 corridor)"),
    plt.Line2D([0],[0], color="#ff44cc", lw=1.4, linestyle="--",
               label="Col 3 dock path  (via slot-01 corridor, deeper)"),
    mpatches.Patch(facecolor="#00ffcc99", edgecolor="#00ffcc",
                   label="Entry point (IN)"),
    mpatches.Patch(facecolor="#ff664499", edgecolor="#ff6644",
                   label="Exit point  (OUT)"),
    mpatches.Patch(facecolor="#ff880055", edgecolor="#ff8800",
                   label="M0609 arm"),
    mpatches.Patch(facecolor="#ffdd4499", edgecolor="#ffdd44",
                   label="Spot"),
    mpatches.Patch(facecolor="#00ff4422", edgecolor="#00ff44",
                   label="Goal Zone (drone delivery)"),
    mpatches.Patch(facecolor="#00ccff22", edgecolor="#00ccff",
                   label="Conveyor"),
    plt.Line2D([0],[0], color="#ffaa00", lw=2, linestyle="--",
               label="Drop Line  x=-7.5"),
]
ax.legend(handles=legend_elements, loc="lower right", fontsize=7,
          facecolor="#1a1a2e", edgecolor="#555588", labelcolor="white",
          framealpha=0.85)

# ── 19) 축 ───────────────────────────────────────────────────────────────
ax.set_xlim(-19, 17)
ax.set_ylim(-19, 18)
ax.set_xlabel("X  [m]", color="white", fontsize=11)
ax.set_ylabel("Y  [m]", color="white", fontsize=11)

ax.set_title(
    "Warehouse Layout  —  Robot & Pod Positions\n"
    "Section A/B/C  |  IW Hub paths  |  Entry/Exit points  |  Pod grid",
    color="white", fontsize=13, pad=14
)

plt.tight_layout()
plt.savefig("/home/rokey/Rokey_isaac-sim/grid/warehouse_layout.png",
            dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
print("Saved: warehouse_layout.png")
plt.show()
