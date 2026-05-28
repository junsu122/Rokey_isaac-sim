"""
main_isaac/robot_config.py
====================
★ 로봇 스폰 좌표는 ROBOT_REGISTRY 에서 한곳에서 수정합니다.

좌표계: warehouse_v3.usd 월드 원점(0,0,0) 기준, 단위 m
"""
from pathlib import Path

# ── 기준 경로 (이 파일 위치 기준으로 자동 계산) ──────────────────────
_BASE  = Path(__file__).parent           # main_isaac/
_SPOT  = _BASE / "robots" / "spot"      # main_isaac/robots/spot/
_M0609 = _BASE / "robots" / "m0609"     # main_isaac/robots/m0609/
_DRONE = _BASE / "robots" / "drone"     # main_isaac/robots/drone/
_IWHUB = _BASE / "robots" / "iw_hub"   # main_isaac/robots/iw_hub/

# ── 경로 설정 ─────────────────────────────────────────────────────────
WAREHOUSE_USD     = str(_BASE / "usd" / "warehouse_v7_1.usda")
SPOT_SRC_DIR      = str(_SPOT  / "spot_test")
M0609_SRC_DIR     = str(_M0609 / "m0609_aruco_detect")

# Drone 관련 경로
DRONE_DEPS_DIR    = str(_DRONE / "drone_deps")
PEGASUS_SIM_DIR   = str(_DRONE / "pegasus_simulator")

# Spot용 그리퍼 USD (사전 컴파일된 USD 파일)
SPOT_GRIPPER_USD  = str(_SPOT / "onrobot_rg2" / "urdf" / "onrobot_rg2" / "onrobot_rg2.usd")

# IW Hub USD
IW_HUB_USD        = str(_IWHUB / "iw_hub_v2.usda")

# M0609 관련 경로 (M0609_SRC_DIR 기준)
M0609_URDF        = M0609_SRC_DIR + "/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF          = M0609_SRC_DIR + "/onrobot_rg2/urdf/onrobot_rg2.urdf"
M0609_RMPFLOW_CFG = M0609_SRC_DIR + "/m0609_rmpflow_common.yaml"
M0609_DESC_YAML   = M0609_SRC_DIR + "/m0609_rg2_description.yaml"
ARUCO_TEXTURE_DIR = M0609_SRC_DIR + "/aruco_marker_6x6"

# ── 시뮬레이션 타이밍 ────────────────────────────────────────────────
PHYSICS_DT   = 1 / 500   # 500 Hz
RENDERING_DT = 1 / 50    # 50  Hz

# ── RealSense 카메라 활성화 여부 ─────────────────────────────────────
# True  : RealSense D455 를 로봇에 부착하고 이미지 스트림 활성화
# False : RealSense 를 로드하지 않음 (Isaac Sim 시작 속도 향상, 리소스 절약)
USE_REALSENSE = True

# ── 로봇 스폰 설정 ──────────────────────────────────────────────────
#
#  type        : "spot" | "m0609" | "drone" | "iw_hub"
#  name        : 고유 이름 (USD prim path 에 사용됨)
#  spawn_xyz   : (x, y, z)  단위 m  ← 이 값을 수정해 위치 조정
#  spawn_yaw   : 초기 방향각 deg   (Z축 기준 반시계 +)
#  cube_xyz    : 집을 큐브 초기 위치  (spot, m0609 전용)
#  goal_xyz    : 놓을 목표 위치       (m0609 전용)
#  scale  : 큐브/goal/ArUco 크기 배율 (m0609 전용, 기본값 1.0 = 5 cm 큐브)
#  environment : 로드할 환경 이름     (drone 전용, 기본값: "Black Gridroom")
#  takeoff_alt : 이륙 목표 고도 m     (drone 전용, 기본값: 1.5)
#
#  ──────────────────────────────────────────────────────────────────
#  창고 레이아웃 참고: /home/rokey/isaac_warehouse_v1/layout.png
#  ──────────────────────────────────────────────────────────────────




####################################################
# main.py에서 지정한 이름이 "type"이 됩니다.
# "name"은 아이작심에서의 stage의 이름이 됩니다.
# "spawn_xyz" 는 스폰할때 world 좌표 기준 스폰 위치가 됩니다.
# "spawn_yaw" 는 스폰할때 world 좌표 기준 yaw 방향(z축 회전 방향)이 됩니다.
#####################################################




# ── ArUco 마커 박스 설정 ─────────────────────────────────────────────
#
#  type    : "green_id0" | "red_id1" | "blue_id2"
#  xyz     : 스폰 위치 (x, y, z) [m]  — 컨베이어 벨트(-16, 0) 위, 순서대로 배치
#
_ARUCO_USD_DIR = str(_BASE / "aruco_marker_box" / "usd")

ARUCO_BOXES = [
    {
        "type": "green_id0",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_green_id0.usda",
        "xyz" : (-16.0, -0.4, 1.0),   # 컨베이어 위 #1 (id0)
    },
    {
        "type": "red_id1",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_red_id1.usda",
        "xyz" : (-16.0,  0.0, 1.0),   # 컨베이어 위 #2 (id1)
    },
    {
        "type": "blue_id2",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_blue_id2.usda",
        "xyz" : (-16.0,  0.4, 1.0),   # 컨베이어 위 #3 (id2)
    },
]

# ── Pod Stack 설정 ────────────────────────────────────────────────────
#
#  usd  : pod_stack_4_v2.usda 파일 경로 (고정)
#  xyz  : 스폰 위치 (x, y, z) [m]  ← 이 값을 수정해 위치 조정
#  yaw  : 회전각 deg (Z축 기준, 기본 0)
#
_POD_USD = str(_BASE / "usd" / "pot_v1.usda")
POD_USD  = _POD_USD   # public alias used by minimap.py

POD_STACKS = [
    {"name": "PodStack_01", "usd": _POD_USD, "xyz": (-12.8,  9.0, 0.0), "yaw": 0.0},  # IW Hub A 홈
    {"name": "PodStack_02", "usd": _POD_USD, "xyz": ( -8.2,  1.55, 0.0), "yaw": 0.0},  # IW Hub B 홈
    {"name": "PodStack_03", "usd": _POD_USD, "xyz": ( -9.65, -8.9, 0.0), "yaw": 0.0},  # IW Hub C 홈
    {"name": "PodStack_04", "usd": _POD_USD, "xyz": ( 12.0, 12.0, 0.0), "yaw": 0.0},  # 드론 배달 목적지
]

# ── Section Pod Stacks (A / B / C  3×3 슬롯) ─────────────────────────
SECTION_POD_USD = _POD_USD
SECTION_GRID_COLS = 3
SECTION_GRID_ROWS = 3
SECTION_GRID_DX = 3.5
SECTION_GRID_DY = 3.0
SECTION_GRID_CENTERS = {
    "A": (0.0,  10.0),
    "B": (0.0,   0.0),
    "C": (0.0, -10.0),
}


def _make_grid(cx: float, cy: float,
               cols: int = SECTION_GRID_COLS, rows: int = SECTION_GRID_ROWS,
               dx: float = SECTION_GRID_DX, dy: float = SECTION_GRID_DY,
               z: float = 0.0) -> list:
    """Return explicit pod-center positions for a section grid.

    Slot 01 is the left/conveyor-side slot of the upper row. Every section
    uses the same fixed dx/dy step. The average of all returned slot centers
    is exactly the section bounding-box center (cx, cy).
    """
    xs = [cx + (c - (cols - 1) / 2.0) * dx for c in range(cols)]
    ys = [cy + (r - (rows - 1) / 2.0) * dy for r in range(rows)]
    return [(round(x, 3), round(y, 3), z) for y in ys for x in xs]


SECTION_PODS = {
    sec: _make_grid(cx, cy)
    for sec, (cx, cy) in SECTION_GRID_CENTERS.items()
}

ROBOT_REGISTRY = [

    # # ── Drone #1 ────────────────────────────────────────────────────
    {
        "type"        : "drone",
        "name"        : "Drone_01",
        "spawn_xyz"   : (-6.8, -11.5, 0.15),
        "spawn_yaw"   : 0.0,
        "takeoff_alt" : 2.5,
        # 드론 자율 미션: 각 섹션의 지정 슬롯 포드를 집어서 배달지(12,12)로 운반
        # 슬롯 번호는 세계 좌표 기준 1-indexed (슬롯 01은 비어 있음)
        "auto_mission"     : False,
        "section_targets" : {"A": 3, "B": 2, "C": 3},
        "delivery_xyz"    : (12.0, 12.0, 0.0),
    },

    # # ── Spot #1 — Section A+B 순찰 (A와 B 사이 시작) ─────────────────────
    # # 경로: x=±5.5, y도 section 밖으로 더 넓게 돌아 IW Hub/Pod 영역을 피함
    {
        "type"      : "spot",
        "name"      : "Spot_01",
        "spawn_xyz" : ( 0.2,  4.8, 0.7),
        "spawn_yaw" : 0.0,
        "waypoints": [
            ( 5.8,  14.0),
            ( 5.8, -14.0),
        ],
        "aruco_goals": {
            0: (-2.8,  14.4),
            1: ( 0.0,  14.4),
            2: ( 2.8,  14.4),
        },
    },

    # # ── Spot #2 — Section B+C 순찰 (B와 C 사이 시작) ─────────────────────
    # # 경로: x=±5.5, y도 section 밖으로 더 넓게 돌아 IW Hub/Pod 영역을 피함
    {
        "type"      : "spot",
        "name"      : "Spot_02",
        "spawn_xyz" : ( 0.0, -5.5, 0.7),
        "spawn_yaw" : 0.0,
        "waypoints": [
            ( 6.0, -14.0),
            ( 6.0,  14.0),
        ],
        "aruco_goals": {
            0: (-2.8,  14.4),
            1: ( 0.0,  14.4),
            2: ( 2.8,  14.4),
        },
    },

    # ── IW Hub #1 ─ Section A 스크립트 루트 ────────────────────────────
    # 경로: spawn(-12.8,13.0) yaw=0 → south y=9.0(pod) → lift
    #        → north y=13.0 → east x=-3.5 → south y=7.0(slot A-01) → lower
    #        → north y=13.0 → west x=-12.7 → wait
    {
        "type"            : "iw_hub",
        "name"            : "iw_hub_01",
        "spawn_xyz"       : (-12.8, 13.0, -0.14),
        "spawn_yaw"       : 0.0,
        "mode"            : "section_a",
        "section"         : "A",
        "complete_topic"  : "/m0609_A/work",
        "complete_signal" : "A_complete",
        "complete_threshold": 1,
    },

    # ── IW Hub #2 ─ 픽업 모드: 포드스택 집어 Section B 슬롯으로 배달 ────────
    # 흐름: spawn(-6.45,1.5) → 트리거 → X이동→pickup(-7.9,1.5) → 리프트업
    #        → 통로(-6.0,1.5) → Y이동→X이동 → 슬롯01 → 리프트다운
    #        → 후진 이탈 → 다음 섹션 pod 픽업 → conveyor 옆 pickup 위치로 복귀
    {
        "type"            : "iw_hub",
        "name"            : "iw_hub_02",
        "spawn_xyz"       : (-6.45, 1.55, -0.14),
        "spawn_yaw"       : 0.0,
        "mode"            : "section_b",
        "section"         : "B",
        "complete_topic"  : "/m0609_B/work",
        "complete_signal" : "B_complete",
        "complete_threshold": 1,
    },

    # ── IW Hub #3 ─ Section C 스크립트 루트 ────────────────────────────
    # 경로: spawn(-9.7,-11.0) yaw=0 → north y=-8.9(pod) → lift
    #        → east x=-3.5 → south y=-13.0(slot C-01) → lower
    #        → north y=-11.0 → west x=-9.7 → wait
    {
        "type"            : "iw_hub",
        "name"            : "iw_hub_03",
        "spawn_xyz"       : (-9.65, -11.0, -0.14),
        "spawn_yaw"       : 0.0,
        "mode"            : "section_c",
        "section"         : "C",
        "complete_topic"  : "/m0609_C/work",
        "complete_signal" : "C_complete",
        "complete_threshold": 1,
    },

    # ── M0609 #1 ────────────────────────────────────────────────────
    # 위치: 창고 서측 작업 스테이션 A
    # 역할: ArUco 마커 시각 서보 → 큐브 픽 앤 플레이스
    {
        "type"       : "m0609",
        "name"       : "M0609_A",
        "spawn_xyz"  : (-12.07, 7.92, 0.93),  # conv_3way/comp_out_north/staging_platform_north/post_nw 위
        "spawn_yaw"  : -90.0,
        "goal_xyz"   : (-12.7, 9.00, 1.3),
        "scale"      : 2.0,
        "box_type"   : "blue_id2",
        "aruco_box_wh": (0.30, 0.30),           # zone2 박스 bw=0.3 bd=0.3
        "waypoint_xyz"      : (-11.3, 8.7, 1.5),
        "pick_z_offset"     : -0.1,   # ★ 픽 Z 추가 오프셋 (음수=더 아래, m)
        "work_complete_count": 1,     # ★ 1회 픽앤플레이스마다 IW Hub 트리거 신호 발행
        "wait_after_complete": True,  # ★ 신호 발행 후 X_start 토픽 수신 대기
        # "pad_reach"        : 0.144,  # ★ EE→흡착패드끝 거리(m)
        # "movel_steps"      : 30,     # ★ MOVEL 속도 (↓값=빠름, 기본 60)
        # "home_return_steps": 150,    # ★ 홈복귀 속도 (↓값=빠름, 기본 250=0.5초)
    },

    # ── M0609 #2 ────────────────────────────────────────────────────
    {
        "type"       : "m0609",
        "name"       : "M0609_B",
        "spawn_xyz"  : (-9.45, 0.79, 0.93),    # staging_platform_west / post_nw 위
        "spawn_yaw"  : 180.0,
        "goal_xyz"   : (-8.2, 1.4, 1.3),
        "scale"      : 2.0,
        "box_type"   : "red_id1",
        "aruco_box_wh": (0.25, 0.25),           # zone1 박스 bw=0.25 bd=0.25
        "waypoint_xyz"      : (-8.7, 0.0, 1.5),
        "pick_z_offset"     : -0.1,   # ★ 픽 Z 추가 오프셋 (음수=더 아래, m)
        "work_complete_count": 1,     # ★ 1회 픽앤플레이스마다 IW Hub 트리거 신호 발행
        "wait_after_complete": True,  # ★ 신호 발행 후 X_start 토픽 수신 대기
        # "movel_steps"      : 30,     # ★ MOVEL 속도 (↓값=빠름, 기본 60)
    },

    # ── M0609 #3 ────────────────────────────────────────────────────
    {
        "type"       : "m0609",
        "name"       : "M0609_C",
        "spawn_xyz"  : (-10.45, -7.80, 0.91),  # staging_platform_south / post_nw 위
        "spawn_yaw"  : 90.0,
        "goal_xyz"   : (-9.7, -8.9, 1.3),
        "scale"      : 2.0,
        "box_type"   : "green_id0",
        "aruco_box_wh": (0.20, 0.20),           # zone0 박스 bw=0.2 bd=0.15
        "waypoint_xyz"      : (-11.3, -8.7, 1.5),
        "pad_reach"         : 0.2,    # ★ EE→흡착패드끝 거리(m). 미지정 시 (stem+pad)×scale 자동계산
        "pick_z_offset"     : -0.1,   # ★ 픽 Z 추가 오프셋 (음수=더 아래, m)
        "work_complete_count": 1,     # ★ 3회 픽앤플레이스마다 IW Hub 트리거 신호 발행
        "wait_after_complete": True,  # ★ 신호 발행 후 X_start 토픽 수신 대기
        # "movel_steps"      : 30,     # ★ MOVEL 속도 (↓값=빠름, 기본 60)
    },

    # ── M0609 삼거리용 (3종 ArUco 인식) ──────────────────────────────────────
    {
        "type"        : "m0609",
        "name"        : "M0609_3way",
        "spawn_xyz"   : (-14.8, 0.5, 2.295),   # post_nw(1.995) + 받침대 0.3m
        "spawn_yaw"   : 180.0,
        "scale"       : 2.5,
        "pad_reach"   : 0.2,
        "pedestal"    : (0.45, 0.3, 0.2),         # post_nw 위에 놓는 받침대 (w,d,h) m
        "pick_xyz_offset": (0.3, 0.0, 0.0),    # 픽 위치 보정 (x,y,z) m
        "pick_z_offset"  : -0.05,             # ★ 픽 Z 추가 오프셋 (음수=더 아래, m)
        "home_deg"      : [90.0, 0.0, 90.0, 0.0, 90.0, 0.0],  # 3way 전용 홈 각도 (joint_1=60°)
        "waypoint_xyz"  : (-14.3, 0.0, 2.5),  # LIFT 후 GOAL 전 경유 웨이포인트
        # ── 속도 튜닝 ─────────────────────────────────────────────────
        "movel_steps"        : 30,    # MOVEL 보간 수 (↓ 빠름, 기본 60)
        "home_return_steps"  : 120,   # 홈 복귀 스텝 수 (↓ 빠름, 기본 250)
        "approach_tol"       : 0.20,  # APPROACH 도달 판정 거리 m (↑ 빠름, 기본 0.15)
        "servo_dz"           : 0.008, # SERVO_DESCEND Z 하강속도/틱 m (↑ 빠름, 기본 0.005)
        # "joint1_limits_deg": (-210.0, 70.0),     # joint_1 물리 한계 (-210 ~ +70°)
        "multi_targets": [
            {"box_type": "green_id0", "goal_xyz": (-14.3, -1.00, 2.2), "aruco_box_wh": (0.20, 0.20)},
            {"box_type": "red_id1",   "goal_xyz": (-13.1, 0.0,  2.2), "aruco_box_wh": (0.25, 0.25)},
            {"box_type": "blue_id2",  "goal_xyz": (-14.3,  1.00, 2.2), "aruco_box_wh": (0.30, 0.30)},
        ],
    },
]
