"""
Amazon Hercules 방식 물류센터 시뮬레이션 환경
Goods-to-Person 방식: 로봇이 파드를 픽스테이션으로 운반

레이아웃 (좌 → 우):
  [입고 도크] → [스토우 스테이션] → [파드 스토리지(QR그리드)] → [픽스테이션] → [팩스테이션] → [출고 도크]

Hercules 로봇:
  - 높이 20cm, 풋프린트 75×60cm
  - 파드 하부를 들어올려 이동
  - 오렌지 색상

Isaac Sim USD 내보내기 호환
"""

import bpy
import math
import random

random.seed(42)


# ═══════════════════════════════════════════════
#  유틸리티
# ═══════════════════════════════════════════════

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    for m in list(bpy.data.materials):
        bpy.data.materials.remove(m)


def new_col(name):
    c = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(c)
    return c


def link_to(obj, col):
    col.objects.link(obj)
    if obj.name in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.unlink(obj)


_mat_cache = {}
def mat(name, rgb, roughness=0.8, metallic=0.0, alpha=1.0, emission=None):
    key = name
    if key in _mat_cache:
        return _mat_cache[key]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nodes = m.node_tree.nodes
    links = m.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value  = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value   = roughness
        bsdf.inputs["Metallic"].default_value    = metallic
        if alpha < 1.0:
            bsdf.inputs["Alpha"].default_value   = alpha
            m.blend_method = 'BLEND'
        if emission:
            bsdf.inputs["Emission Color"].default_value = (*emission, 1.0)
            bsdf.inputs["Emission Strength"].default_value = 2.0
    _mat_cache[key] = m
    return m


def apply_mat(obj, material):
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)


def box(name, loc, size, col, material=None):
    bpy.ops.mesh.primitive_cube_add(location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (size[0], size[1], size[2])
    bpy.ops.object.transform_apply(scale=True)
    link_to(obj, col)
    if material:
        apply_mat(obj, material)
    return obj


def cyl(name, loc, r, h, col, material=None, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h, location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.rotation_euler = rot
    bpy.ops.object.transform_apply(rotation=True)
    link_to(obj, col)
    if material:
        apply_mat(obj, material)
    return obj


# ═══════════════════════════════════════════════
#  1. 건물 구조
# ═══════════════════════════════════════════════
# 전체 크기: X 60m(-30~+30), Y 40m(-20~+20), 높이 10m
WX, WY, WH = 30.0, 20.0, 10.0   # 반치수

def create_structure(col):
    m_floor  = mat("Floor",    (0.50, 0.50, 0.50), roughness=0.95)
    m_wall   = mat("Wall",     (0.80, 0.80, 0.78), roughness=1.00)
    m_ceil   = mat("Ceiling",  (0.70, 0.70, 0.70), roughness=1.00)
    m_pillar = mat("Pillar",   (0.65, 0.65, 0.65), metallic=0.2)
    m_qr_bg  = mat("QR_Floor", (0.42, 0.42, 0.42), roughness=0.95)

    # 바닥
    box("Floor", (0, 0, -0.15), (WX, WY, 0.15), col, m_floor)

    # 천장
    box("Ceiling", (0, 0, WH + 0.15), (WX, WY, 0.15), col, m_ceil)

    # 벽 4면
    for name, loc, sz in [
        ("Wall_N", ( 0,  WY,  WH/2), (WX,   0.25, WH/2)),
        ("Wall_S", ( 0, -WY,  WH/2), (WX,   0.25, WH/2)),
        ("Wall_E", ( WX,  0,  WH/2), (0.25, WY,   WH/2)),
        ("Wall_W", (-WX,  0,  WH/2), (0.25, WY,   WH/2)),
    ]:
        box(name, loc, sz, col, m_wall)

    # 기둥 격자 (10m 간격)
    for xi in range(-2, 3):
        for yi in range(-1, 2):
            box(f"Pillar_{xi}_{yi}",
                (xi * 10, yi * 10, WH/2),
                (0.3, 0.3, WH/2), col, m_pillar)

    # QR 네비게이션 마커 (파드 스토리지 구역, 1.5m 간격)
    qr_x_start, qr_x_end = -8.0, 14.0
    qr_y_start, qr_y_end = -13.0, 13.0
    spacing = 1.5
    xi = qr_x_start
    idx = 0
    while xi <= qr_x_end:
        yi = qr_y_start
        while yi <= qr_y_end:
            box(f"QR_{idx}",
                (xi, yi, -0.14),
                (0.1, 0.1, 0.01), col, m_qr_bg)
            yi += spacing
            idx += 1
        xi += spacing

    # 구역 구분선
    m_zone = mat("ZoneLine", (1.0, 0.65, 0.0), roughness=1.0)
    zone_lines = [
        # 입고 | 스토우 경계
        ((-20, 0, 0.01), (0.06, WY, 0.01)),
        # 스토우 | 파드스토리지 경계
        ((-8,  0, 0.01), (0.06, WY, 0.01)),
        # 파드스토리지 | 픽스테이션 경계
        ((14,  0, 0.01), (0.06, WY, 0.01)),
        # 픽스테이션 | 팩스테이션 경계
        ((20,  0, 0.01), (0.06, WY, 0.01)),
        # 팩스테이션 | 출고 경계
        ((24,  0, 0.01), (0.06, WY, 0.01)),
    ]
    for i, (loc, sz) in enumerate(zone_lines):
        box(f"ZoneLine_{i}", loc, sz, col, m_zone)

    # 구역 바닥 색상
    zone_floors = [
        ("ZoneFloor_Inbound",  (-25,  0, -0.12), (5.0,  WY, 0.01), (0.45, 0.52, 0.60)),
        ("ZoneFloor_Stow",     (-14,  0, -0.12), (6.0,  WY, 0.01), (0.42, 0.55, 0.42)),
        ("ZoneFloor_Pick",     ( 17,  0, -0.12), (3.0,  WY, 0.01), (0.55, 0.42, 0.42)),
        ("ZoneFloor_Pack",     ( 22,  0, -0.12), (2.0,  WY, 0.01), (0.42, 0.42, 0.60)),
        ("ZoneFloor_Outbound", ( 27,  0, -0.12), (3.0,  WY, 0.01), (0.52, 0.45, 0.35)),
    ]
    for name, loc, sz, rgb in zone_floors:
        box(name, loc, sz, col, mat(f"M_{name}", rgb, roughness=0.95))


# ═══════════════════════════════════════════════
#  2. 입고 도크 (Inbound Dock)
# ═══════════════════════════════════════════════

def create_inbound_dock(col):
    m_dock    = mat("DockFloor",  (0.38, 0.38, 0.38), roughness=0.9)
    m_door    = mat("DockDoor",   (0.55, 0.58, 0.60), metallic=0.4)
    m_stripe  = mat("DockStripe", (0.95, 0.85, 0.05), roughness=1.0)
    m_tote    = mat("Tote",       (0.95, 0.75, 0.05), roughness=0.8)
    m_pallet  = mat("Pallet",     (0.50, 0.32, 0.14), roughness=0.9)

    # 도크 플랫폼 3개 (남쪽 벽 측)
    for i, dy in enumerate([-12, -4, 4]):
        # 플랫폼
        box(f"DockPlatform_{i}", (-27, dy, 0.4),
            (2.5, 2.8, 0.4), col, m_dock)
        # 도크 도어 (벽에 붙음)
        box(f"DockDoor_{i}", (-29.6, dy, 2.5),
            (0.15, 2.5, 2.5), col, m_door)
        # 안전 줄무늬
        for j in range(4):
            box(f"DockStripe_{i}_{j}", (-24.5 + j*0.5, dy, 0.02),
                (0.1, 2.6, 0.01), col, m_stripe)

    # 노란 토트(tote) 스택
    for i, (tx, ty) in enumerate([
        (-22, -15), (-22, -12), (-22, -9),
        (-22,  9),  (-22, 12),  (-22, 15),
    ]):
        for lv in range(3):
            box(f"Tote_{i}_L{lv}",
                (tx, ty, 0.3 + lv * 0.35),
                (0.28, 0.38, 0.17), col, m_tote)

    # 입고 팔레트
    for i, (px, py) in enumerate([(-21, -6), (-21, 0), (-21, 6)]):
        box(f"InPallet_{i}",     (px, py, 0.08), (0.60, 0.40, 0.08), col, m_pallet)
        box(f"InPallet_{i}_top", (px, py, 0.50), (0.55, 0.38, 0.38), col,
            mat(f"InCargo_{i}", (0.6+i*0.05, 0.5, 0.35), roughness=0.85))

    # 포크리프트 (단순 형태)
    m_fl = mat("Forklift", (0.95, 0.72, 0.0), metallic=0.3, roughness=0.4)
    box("Forklift_Body", (-23, 14, 0.75),  (0.8, 1.2, 0.75), col, m_fl)
    box("Forklift_Mast", (-23, 12.5, 2.0), (0.1, 0.1, 2.0),  col, m_fl)
    box("Forklift_Fork_L", (-23, 12.0, 0.15),
        (0.08, 0.8, 0.05), col, mat("Fork", (0.3,0.3,0.3), metallic=0.8))
    box("Forklift_Fork_R", (-23, 12.0, 0.15),
        (0.08, 0.8, 0.05), col, mat("Fork", (0.3,0.3,0.3), metallic=0.8))


# ═══════════════════════════════════════════════
#  3. 스토우 스테이션 (Stow Station)
# ═══════════════════════════════════════════════

def create_stow_stations(col):
    m_desk   = mat("StowDesk",   (0.85, 0.82, 0.78), roughness=0.7)
    m_screen = mat("Screen",     (0.05, 0.08, 0.15), roughness=0.2,
                   emission=(0.1, 0.5, 1.0))
    m_tote   = mat("Tote",       (0.95, 0.75, 0.05), roughness=0.8)
    m_conv   = mat("StowConv",   (0.35, 0.35, 0.35), roughness=0.85)

    for i, dy in enumerate([-12, -4, 4, 12]):
        sx = -14.0
        # 작업 데스크
        box(f"StowDesk_{i}",       (sx, dy, 1.0),       (1.2, 0.4, 0.05), col, m_desk)
        box(f"StowDesk_{i}_leg1",  (sx-1.1, dy-0.35, 0.5), (0.05, 0.05, 0.5), col, m_desk)
        box(f"StowDesk_{i}_leg2",  (sx-1.1, dy+0.35, 0.5), (0.05, 0.05, 0.5), col, m_desk)
        box(f"StowDesk_{i}_leg3",  (sx+1.1, dy-0.35, 0.5), (0.05, 0.05, 0.5), col, m_desk)
        box(f"StowDesk_{i}_leg4",  (sx+1.1, dy+0.35, 0.5), (0.05, 0.05, 0.5), col, m_desk)
        # 모니터
        box(f"StowMonitor_{i}",    (sx, dy-0.5, 1.65),  (0.4, 0.03, 0.3),  col, m_screen)
        box(f"StowMonBase_{i}",    (sx, dy-0.5, 1.1),   (0.05, 0.03, 0.15), col, m_desk)
        # 토트 컨베이어 (입고 → 스토우)
        box(f"StowConv_{i}",       (sx+2.5, dy, 0.95),  (1.5, 0.3, 0.05),  col, m_conv)
        # 스캐너 (형태 단순화)
        box(f"StowScanner_{i}",    (sx-0.5, dy-0.5, 1.1), (0.08, 0.04, 0.08), col,
            mat(f"Scanner_{i}", (0.1, 0.1, 0.1), metallic=0.6))
        # 대기 토트
        for j in range(2):
            box(f"StowTote_{i}_{j}", (sx+1.0, dy+0.3*j, 1.05),
                (0.25, 0.34, 0.15), col, m_tote)


# ═══════════════════════════════════════════════
#  4. 이동식 파드 (Inventory Pod)
#     4면 선반, 30+ 큐비, 높이 2.4m
# ═══════════════════════════════════════════════

def create_pod(name, cx, cy, col, has_robot=False):
    m_frame  = mat("PodFrame",  (0.18, 0.18, 0.18), metallic=0.5, roughness=0.4)
    m_shelf  = mat("PodShelf",  (0.88, 0.88, 0.88), roughness=0.7)
    m_tote   = mat("Tote",      (0.95, 0.75, 0.05), roughness=0.8)

    PW, PD, PH = 1.0, 1.0, 2.4   # 파드 크기
    levels = 5
    lh = PH / levels

    # 파드가 로봇 위에 올라가 있으면 높이 조정
    z_offset = 0.22 if has_robot else 0.0

    # 외부 프레임 기둥 4개
    for dx, dy in [(-PW/2+0.03, -PD/2+0.03), (PW/2-0.03, -PD/2+0.03),
                   (-PW/2+0.03,  PD/2-0.03), (PW/2-0.03,  PD/2-0.03)]:
        box(f"{name}_Post_{dx:.2f}_{dy:.2f}",
            (cx+dx, cy+dy, z_offset + PH/2),
            (0.025, 0.025, PH/2), col, m_frame)

    # 수평 빔 & 선반
    for lv in range(levels + 1):
        z = z_offset + lv * lh
        # 수평 빔 (앞뒤)
        box(f"{name}_BeamF_L{lv}", (cx, cy - PD/2 + 0.03, z),
            (PW/2 - 0.03, 0.02, 0.02), col, m_frame)
        box(f"{name}_BeamB_L{lv}", (cx, cy + PD/2 - 0.03, z),
            (PW/2 - 0.03, 0.02, 0.02), col, m_frame)
        # 선반판
        if lv < levels:
            box(f"{name}_Shelf_L{lv}",
                (cx, cy, z + lh*0.5),
                (PW/2 - 0.04, PD/2 - 0.04, 0.015), col, m_shelf)
            # 큐비 내용물 (토트 2개/면 × 4면 → 8개/층 간략화)
            for side, (tdx, tdy) in enumerate([
                (-0.25, 0), (0.25, 0),
                (0, -0.25), (0, 0.25),
            ]):
                if random.random() > 0.25:  # 75% 확률로 채워진 큐비
                    tc = random.choice([
                        (0.95,0.75,0.05), (0.8,0.2,0.2),
                        (0.2,0.6,0.8),    (0.5,0.8,0.3),
                    ])
                    box(f"{name}_Tote_L{lv}_S{side}",
                        (cx+tdx, cy+tdy, z + lh*0.5 + 0.03),
                        (0.18, 0.18, 0.14), col,
                        mat(f"ToteC_{tc[0]:.2f}", tc, roughness=0.85))


def create_pod_storage(col):
    """파드 스토리지 그리드: 10열 × 8행"""
    COLS, ROWS = 10, 8
    SPACING    = 1.5    # 파드 중심 간격 (0.5m 로봇 통로)
    START_X    = -6.75
    START_Y    = -5.25

    total = COLS * ROWS
    robot_slots = random.sample(range(total), 6)  # 로봇이 파드 든 슬롯 6개

    idx = 0
    for row in range(ROWS):
        for col_i in range(COLS):
            cx = START_X + col_i * SPACING
            cy = START_Y + row  * SPACING
            has_robot = idx in robot_slots
            create_pod(f"Pod_{row}_{col_i}", cx, cy, col, has_robot)
            idx += 1


# ═══════════════════════════════════════════════
#  5. Hercules 로봇
# ═══════════════════════════════════════════════

def create_hercules_robot(name, cx, cy, angle, carrying, col):
    """
    Hercules: 높이 20cm, 75×60cm, 오렌지 색상
    carrying=True 이면 파드를 들고 있는 상태
    """
    m_body   = mat("HerculesBody",  (0.95, 0.40, 0.05), metallic=0.2, roughness=0.4)
    m_top    = mat("HerculesTop",   (0.15, 0.15, 0.15), metallic=0.5, roughness=0.3)
    m_wheel  = mat("HerculesWheel", (0.10, 0.10, 0.10), roughness=0.9)
    m_sensor = mat("HerculesSensor",(0.05, 0.05, 0.05), metallic=0.8)
    m_light  = mat("HerculesLight", (0.3, 0.8, 0.3),
                   roughness=0.1, emission=(0.3, 1.0, 0.3))

    z_base = 0.10  # 로봇 중심 높이

    # 메인 바디 (오렌지)
    body = box(f"{name}_Body", (cx, cy, z_base),
               (0.375, 0.30, 0.09), col, m_body)
    body.rotation_euler = (0, 0, angle)
    bpy.ops.object.transform_apply(rotation=True)

    # 탑 패널 (검정)
    top = box(f"{name}_Top", (cx, cy, z_base + 0.09),
              (0.32, 0.26, 0.02), col, m_top)

    # 카메라/센서 (전방)
    sa = math.cos(angle) * 0.38
    sb = math.sin(angle) * 0.38
    box(f"{name}_Sensor", (cx + sa, cy + sb, z_base + 0.05),
        (0.04, 0.04, 0.04), col, m_sensor)

    # 상태 표시등
    box(f"{name}_Light", (cx, cy, z_base + 0.12),
        (0.03, 0.03, 0.01), col, m_light)

    # 바퀴 4개
    for wx, wy in [(-0.28, -0.22), (-0.28,  0.22),
                   ( 0.28, -0.22), ( 0.28,  0.22)]:
        wlx = cx + wx * math.cos(angle) - wy * math.sin(angle)
        wly = cy + wx * math.sin(angle) + wy * math.cos(angle)
        cyl(f"{name}_Wheel_{wx:.2f}_{wy:.2f}",
            (wlx, wly, 0.065), 0.065, 0.06, col, m_wheel,
            rot=(math.pi/2, 0, angle))

    # 파드를 들고 이동 중이면 리프팅 암 표시
    if carrying:
        m_lift = mat("LiftArm", (0.55, 0.55, 0.55), metallic=0.7)
        box(f"{name}_LiftPlate", (cx, cy, 0.20),
            (0.38, 0.38, 0.02), col, m_lift)


def create_all_robots(col):
    # 파드 스토리지 내 이동 중인 로봇들
    robots_in_storage = [
        ("Hercules_S1",  -3.0,  -3.0, math.radians( 45), False),
        ("Hercules_S2",   1.5,   1.5, math.radians(  0), False),
        ("Hercules_S3",  -0.75,  3.0, math.radians(180), False),
    ]
    # 파드 들고 픽스테이션으로 이동 중
    robots_carrying = [
        ("Hercules_C1",  11.0,  -8.0, math.radians(-90), True),
        ("Hercules_C2",  11.0,   4.0, math.radians(-90), True),
        ("Hercules_C3",  12.5, -15.0, math.radians( 90), True),
    ]
    for args in robots_in_storage + robots_carrying:
        create_hercules_robot(*args, col)


# ═══════════════════════════════════════════════
#  6. 픽 스테이션 (Pick Station)
# ═══════════════════════════════════════════════

def create_pick_stations(col):
    m_desk   = mat("PickDesk",   (0.22, 0.24, 0.28), roughness=0.6)
    m_screen = mat("PickScreen", (0.03, 0.06, 0.12), roughness=0.1,
                   emission=(0.1, 0.5, 1.0))
    m_frame  = mat("PickFrame",  (0.60, 0.60, 0.60), metallic=0.5)
    m_tote   = mat("Tote",       (0.95, 0.75, 0.05), roughness=0.8)
    m_belt   = mat("PickBelt",   (0.20, 0.20, 0.20), roughness=0.9)

    for i, dy in enumerate([-14, -7, 0, 7, 14]):
        sx = 17.0
        # 픽 포트 프레임 (파드가 들어오는 입구)
        box(f"PickFrame_{i}_L",  (sx-1.5, dy-1.0, 2.5),
            (0.08, 0.08, 2.5), col, m_frame)
        box(f"PickFrame_{i}_R",  (sx-1.5, dy+1.0, 2.5),
            (0.08, 0.08, 2.5), col, m_frame)
        box(f"PickFrame_{i}_Top",(sx-1.5, dy, 5.0),
            (0.08, 1.1, 0.08), col, m_frame)

        # 작업 데스크
        box(f"PickDesk_{i}",     (sx+1.0, dy, 1.05),
            (1.0, 0.4, 0.05), col, m_desk)
        # 모니터 2개 (pick-to-light)
        box(f"PickMon_{i}_1",    (sx+0.5, dy-0.25, 1.7),
            (0.35, 0.03, 0.25), col, m_screen)
        box(f"PickMon_{i}_2",    (sx+0.5, dy+0.25, 1.7),
            (0.35, 0.03, 0.25), col, m_screen)
        # 토트 출력 컨베이어
        box(f"PickConv_{i}",     (sx+2.2, dy, 0.9),
            (0.9, 0.28, 0.04), col, m_belt)
        # 대기 토트
        for j in range(3):
            box(f"PickTote_{i}_{j}", (sx+2.8, dy, 0.95 + j*0.32),
                (0.26, 0.35, 0.15), col, m_tote)


# ═══════════════════════════════════════════════
#  7. 팩 스테이션 (Pack Station)
# ═══════════════════════════════════════════════

def create_pack_stations(col):
    m_desk   = mat("PackDesk",   (0.85, 0.82, 0.78), roughness=0.7)
    m_screen = mat("PackScreen", (0.03, 0.06, 0.12), roughness=0.1,
                   emission=(0.1, 0.5, 1.0))
    m_box    = mat("PackBox",    (0.75, 0.62, 0.42), roughness=0.85)
    m_tape   = mat("TapeDsp",    (0.3, 0.3, 0.3), metallic=0.3)
    m_scale  = mat("Scale",      (0.8, 0.8, 0.8), metallic=0.4)

    for i, dy in enumerate([-16, -10, -4, 2, 8, 14]):
        sx = 22.0
        # 데스크
        box(f"PackDesk_{i}",     (sx, dy, 1.0),  (0.9, 0.5, 0.05), col, m_desk)
        # 모니터
        box(f"PackMon_{i}",      (sx-0.4, dy-0.45, 1.65), (0.35, 0.03, 0.28), col, m_screen)
        # 저울
        box(f"PackScale_{i}",    (sx+0.3, dy, 1.06), (0.18, 0.18, 0.02), col, m_scale)
        # 테이프 디스펜서
        box(f"PackTape_{i}",     (sx-0.6, dy+0.2, 1.1), (0.06, 0.06, 0.12), col, m_tape)
        # 포장 박스 (크기별)
        sizes = [(0.28, 0.22, 0.18), (0.35, 0.28, 0.24), (0.20, 0.18, 0.15)]
        for j, (bx, by, bz) in enumerate(sizes):
            box(f"PackBox_{i}_{j}",
                (sx+0.3, dy+0.3 - j*0.3, 1.07 + j*0.12),
                (bx/2, by/2, bz/2), col, m_box)
        # 데스크 다리
        for lx, ly in [(-0.85, -0.45), (-0.85, 0.45), (0.85, -0.45), (0.85, 0.45)]:
            box(f"PackLeg_{i}_{lx:.2f}_{ly:.2f}",
                (sx+lx, dy+ly, 0.5), (0.04, 0.04, 0.5), col, m_desk)


# ═══════════════════════════════════════════════
#  8. 출고 컨베이어 시스템 (Outbound Sorter)
# ═══════════════════════════════════════════════

def create_outbound(col):
    m_frame  = mat("OutFrame",  (0.35, 0.35, 0.35), metallic=0.7)
    m_belt   = mat("OutBelt",   (0.12, 0.12, 0.12), roughness=0.9)
    m_roller = mat("OutRoller", (0.45, 0.45, 0.45), metallic=0.8)
    m_sign   = mat("OutSign",   (0.1, 0.35, 0.1),   roughness=0.6)
    m_pkg    = mat("Package",   (0.68, 0.55, 0.38), roughness=0.85)

    # 메인 소터 컨베이어 (Y축 방향으로 길게)
    conv_x = 26.0
    box("OutConv_Main_Belt",  (conv_x, 0, 0.9),  (0.55, WY, 0.04), col, m_belt)
    box("OutConv_Main_FrameL",(conv_x, 0, 0.82), (0.60, WY, 0.06), col, m_frame)

    # 롤러
    for i, ry in enumerate(range(-18, 19, 1)):
        cyl(f"OutRoller_{i}", (conv_x, ry, 0.9),
            0.06, 1.1, col, m_roller, rot=(0, math.pi/2, 0))

    # 팩 → 출고 분기 컨베이어 (수평)
    for i, dy in enumerate([-16, -10, -4, 2, 8, 14]):
        lx = 24.5
        box(f"OutBranch_{i}_Belt",  (lx, dy, 0.9), (1.5, 0.28, 0.04), col, m_belt)
        box(f"OutBranch_{i}_Frame", (lx, dy, 0.82),(1.5, 0.32, 0.06), col, m_frame)

    # 배송 목적지 레이블 (출고 도크)
    dest_labels = ["서울", "경기", "인천", "부산"]
    for i, (label, dy) in enumerate(zip(dest_labels, [-12, -4, 4, 12])):
        # 표지판 기둥
        box(f"DestPost_{i}",  (28.5, dy, 2.5), (0.06, 0.06, 2.5), col, m_sign)
        box(f"DestBoard_{i}", (28.5, dy, 5.1), (0.8,  0.06, 0.45), col, m_sign)

    # 출고 대기 박스들
    for i in range(12):
        px = 27.5 + (i % 2) * 0.7
        py = -14 + i * 2.2
        box(f"OutPkg_{i}", (px, py, 0.2),
            (0.22, 0.18, 0.18), col, m_pkg)

    # 출고 도크 도어 3개
    m_door = mat("OutDoor", (0.55, 0.58, 0.60), metallic=0.4)
    for i, dy in enumerate([-10, 0, 10]):
        box(f"OutDock_{i}", (29.6, dy, 2.5), (0.15, 2.5, 2.5), col, m_door)


# ═══════════════════════════════════════════════
#  9. 조명
# ═══════════════════════════════════════════════

def create_lighting(col):
    light_positions = [
        # 입고
        (-25, -12, 9.5), (-25, 0, 9.5), (-25, 12, 9.5),
        # 스토우
        (-14, -10, 9.5), (-14, 0, 9.5), (-14, 10, 9.5),
        # 파드 스토리지
        (-5, -10, 9.5), (-5, 0, 9.5), (-5, 10, 9.5),
        ( 3, -10, 9.5), ( 3, 0, 9.5), ( 3, 10, 9.5),
        (10, -10, 9.5), (10, 0, 9.5), (10, 10, 9.5),
        # 픽 스테이션
        (17, -14, 9.5), (17, -7, 9.5), (17, 0, 9.5),
        (17,   7, 9.5), (17, 14, 9.5),
        # 팩 스테이션
        (22, -12, 9.5), (22, 0, 9.5), (22, 12, 9.5),
        # 출고
        (27, -10, 9.5), (27, 0, 9.5), (27, 10, 9.5),
    ]
    for i, (lx, ly, lz) in enumerate(light_positions):
        bpy.ops.object.light_add(type='AREA', location=(lx, ly, lz))
        light = bpy.context.active_object
        light.name = f"Light_{i}"
        light.data.energy = 500
        light.data.size   = 5.0
        link_to(light, col)

    # 월드 앰비언트
    bpy.context.scene.world.use_nodes = True
    bg = bpy.context.scene.world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Strength"].default_value = 0.3


# ═══════════════════════════════════════════════
#  10. 카메라
# ═══════════════════════════════════════════════

def setup_cameras():
    # 전체 조망 카메라
    bpy.ops.object.camera_add(location=(0, -45, 35))
    cam1 = bpy.context.active_object
    cam1.name = "Cam_Overview"
    cam1.rotation_euler = (math.radians(55), 0, 0)
    bpy.context.scene.camera = cam1

    # 탑뷰
    bpy.ops.object.camera_add(location=(0, 0, 45))
    cam2 = bpy.context.active_object
    cam2.name = "Cam_TopView"
    cam2.rotation_euler = (0, 0, 0)

    # 파드 스토리지 클로즈업
    bpy.ops.object.camera_add(location=(3, -20, 12))
    cam3 = bpy.context.active_object
    cam3.name = "Cam_PodZone"
    cam3.rotation_euler = (math.radians(65), 0, math.radians(15))


# ═══════════════════════════════════════════════
#  메인
# ═══════════════════════════════════════════════

def main():
    print("\n" + "="*55)
    print(" Amazon Hercules 물류센터 환경 생성")
    print("="*55)
    print(" 레이아웃: 입고도크 → 스토우 → 파드스토리지 → 픽 → 팩 → 출고")
    print("="*55)

    clear_scene()

    col_structure = new_col("01_Structure")
    col_inbound   = new_col("02_InboundDock")
    col_stow      = new_col("03_StowStation")
    col_pods      = new_col("04_PodStorage")
    col_robots    = new_col("05_HerculesRobots")
    col_pick      = new_col("06_PickStation")
    col_pack      = new_col("07_PackStation")
    col_outbound  = new_col("08_OutboundSorter")
    col_lighting  = new_col("09_Lighting")

    print("▶ 건물 구조 + QR 그리드...")
    create_structure(col_structure)

    print("▶ 입고 도크 (트럭 도크 3개)...")
    create_inbound_dock(col_inbound)

    print("▶ 스토우 스테이션 (4개)...")
    create_stow_stations(col_stow)

    print("▶ 파드 스토리지 그리드 (10×8 = 80 파드)...")
    create_pod_storage(col_pods)

    print("▶ Hercules 로봇 (6대)...")
    create_all_robots(col_robots)

    print("▶ 픽 스테이션 (5개)...")
    create_pick_stations(col_pick)

    print("▶ 팩 스테이션 (6개)...")
    create_pack_stations(col_pack)

    print("▶ 출고 소터 컨베이어...")
    create_outbound(col_outbound)

    print("▶ 조명 (27개)...")
    create_lighting(col_lighting)

    print("▶ 카메라 (3대) 배치...")
    setup_cameras()

    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.samples = 64

    print("\n" + "="*55)
    print(" ✓ 완료!")
    print()
    print(" 컬렉션 구조:")
    print("  01_Structure      건물/바닥/벽/QR마커")
    print("  02_InboundDock    입고도크/포크리프트/토트")
    print("  03_StowStation    스토우 작업대 4개")
    print("  04_PodStorage     이동식 파드 80개")
    print("  05_HerculesRobots 로봇 6대 (이동/운반)")
    print("  06_PickStation    픽스테이션 5개")
    print("  07_PackStation    팩스테이션 6개")
    print("  08_OutboundSorter 출고 소터 컨베이어")
    print("  09_Lighting       조명 27개")
    print()
    print(" USD 내보내기: File > Export > USD (.usd)")
    print("="*55)


main()
