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
#  xyz     : 스폰 위치 (x, y, z) [m]  ← 이 값을 수정해 위치 조정
#
_ARUCO_USD_DIR = str(_BASE / "aruco_marker_box" / "usd")

ARUCO_BOXES = [
    {
        "type": "green_id0",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_green_id0.usda",
        "xyz" : (-13.0, 9.5, 1.0),   # ★ M0609_A 앞 큐브 위치
    },
    {
        "type": "red_id1",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_red_id1.usda",
        "xyz" : (-7.88, 1.64, 1.0),   # ★ M0609_B 앞 큐브 위치
    },
    {
        "type": "blue_id2",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_blue_id2.usda",
        "xyz" : (-9.63, -9.38, 1.0),  # ★ M0609_C 앞 큐브 위치
    },
]

# ── Pod Stack 설정 ────────────────────────────────────────────────────
#
#  usd  : pod_stack_4_v2.usda 파일 경로 (고정)
#  xyz  : 스폰 위치 (x, y, z) [m]  ← 이 값을 수정해 위치 조정
#  yaw  : 회전각 deg (Z축 기준, 기본 0)
#
_POD_USD = str(_BASE / "usd" / "pot_v1.usda")

POD_STACKS = [
    {"name": "PodStack_01", "usd": _POD_USD, "xyz": (-12.8, 9.0, 0.0), "yaw": 0.0},  # ★
    {"name": "PodStack_02", "usd": _POD_USD, "xyz": (-8.2, 1.5, 0.0), "yaw": 0.0},  # ★
    {"name": "PodStack_03", "usd": _POD_USD, "xyz": (-9.7, -8.9, 0.0), "yaw": 0.0},  # ★
]

# ── IW Hub 이동 금지 구역 ─────────────────────────────────────────────
#
# 사각형 영역을 월드 XY 좌표로 등록하면 smart_factory 이동 경로가 이 공간을
# clearance 만큼 더 넓게 잡고 우회합니다.
#
# 형식 1: min/max 직접 지정
#   {"name": "pillar_01", "min_x": -2.0, "max_x": -1.2, "min_y": 3.0, "max_y": 4.0, "clearance": 0.5}
#
# 형식 2: 중심점 + 크기 지정
#   {"name": "rail_01", "center": (0.0, 2.0), "size": (8.0, 0.4), "clearance": 0.5}
#
IW_HUB_NO_GO_ZONES = [
    {
        "name": "wall_n",
        "center": (0.0, 16.65, 2.5),
        "half_extent": (16.8, 0.15, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_s",
        "center": (0.0, -16.65, 2.5),
        "half_extent": (16.8, 0.15, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_w_n",
        "center": (-16.65, 9.15, 2.5),
        "half_extent": (0.15, 7.65, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_w_s",
        "center": (-16.65, -9.15, 2.5),
        "half_extent": (0.15, 7.65, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_e_seg0",
        "center": (16.65, 14.35, 2.5),
        "half_extent": (0.15, 2.45, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_e_seg1",
        "center": (16.65, 5.2, 2.5),
        "half_extent": (0.15, 3.7, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_e_seg2",
        "center": (16.65, -5.2, 2.5),
        "half_extent": (0.15, 3.7, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "wall_e_seg3",
        "center": (16.65, -14.35, 2.5),
        "half_extent": (0.15, 2.45, 2.5),
        "clearance": 0.5,
    },
    {
        "name": "left_workcell_forbidden_zone",
        "center": (-12.245, 0.08, 0.0),
        "half_extent": (2.755, 7.79, 0.1),
        "clearance": 0.5,
    },
    {
        "name": "robot_cell_bottom",
        "center": (-11.2, -8.5, 0.0),
        "half_extent": (0.79, 0.79, 0.1),
        "clearance": 0.2,
    },
    {
        "name": "robot_cell_top",
        "center": (-11.3, 8.66, 0.0),
        "half_extent": (0.79, 0.79, 0.1),
        "clearance": 0.2,
    },
    {
        "name": "robot_cell_center",
        "center": (-8.7, 0.0, 0.0),
        "half_extent": (0.79, 0.79, 0.1),
        "clearance": 0.2,
    },
]

ROBOT_REGISTRY = [

    # ── Drone #1 ────────────────────────────────────────────────────
    # 위치: 창고 내 원하는 위치
    # 역할: 키보드/조이스틱 자유 비행 + 깊이 카메라 HUD
    # {
    #     "type"        : "drone",
    #     "name"        : "Drone_01",
    #     "spawn_xyz"   : (0.0, 0.0, 2.2),
    #     "spawn_yaw"   : 0.0,
    #     "takeoff_alt" : 1.5,               # 선택 (기본값)
    # },

    # ── Spot #1 ─────────────────────────────────────────────────────
    # 위치: 창고 중앙 통로
    # 역할: 웨이포인트 순찰 → ArUco ID 감지 → 박스 픽업 → ID별 목표로 이동
    {
        "type"      : "spot",
        "name"      : "Spot_01",
        "spawn_xyz" : ( 4.15,  7.0, 0.7),   # ★ 스폰 (x, y, z)
        "spawn_yaw" : 0.0,                   # ★ 초기 방향 (deg)
        #
        # ── 순찰 웨이포인트 [(x, y), ...] [m] ───────────────────────
        # 이 순서대로 순환 주행합니다. 미지정 시 스폰 주변 기본 경로 사용.
        # ★ 좌표를 자유롭게 추가/수정/삭제 가능
        "waypoints": [
            ( 4.15, 7.0),   # wp0 ← ★
            ( 1.15, 7.0),   # wp1 ← ★
            ( 1.15, 13.0),   # wp2 ← ★
            ( 4.15, 13.0),   # wp3 ← ★
        ],
        #
        # ── ArUco ID별 목표 XY 위치 ─────────────────────────────────
        # ArUco ID 에 해당하는 박스를 집은 뒤 이 좌표로 이동해 내려놓습니다.
        # 키: ArUco ID (int), 값: (x, y) [m]  ← ★ 이 값을 수정해 목표 위치 조정
        "aruco_goals": {
            0: ( -3.0,  15.15),   # green_id0 → 목표 A  ← ★
            1: ( 0.0,  15.15),   # red_id1   → 목표 B  ← ★
            2: ( 3.0, 15.15),   # blue_id2  → 목표 C  ← ★
        },
    },

    # ── Spot #2 ─────────────────────────────────────────────────────
    {
        "type"      : "spot",
        "name"      : "Spot_02",
        "spawn_xyz" : ( -4.1, 13.0, 0.7),
        "spawn_yaw" : 0.0,
        #
        # ── 순찰 웨이포인트 [(x, y), ...] [m] ───────────────────────
        # 이 순서대로 순환 주행합니다. 미지정 시 스폰 주변 기본 경로 사용.
        # ★ 좌표를 자유롭게 추가/수정/삭제 가능
        "waypoints": [
            ( -4.1, 13.0),   # wp2 ← ★
            ( -1.3, 13.0),   # wp3 ← ★
            ( -1.3,  7.0),   # wp0 ← ★
            ( -4.1,  7.0),   # wp1 ← ★
        ],
        #
        # ── ArUco ID별 목표 XY 위치 ─────────────────────────────────
        # ArUco ID 에 해당하는 박스를 집은 뒤 이 좌표로 이동해 내려놓습니다.
        # 키: ArUco ID (int), 값: (x, y) [m]  ← ★ 이 값을 수정해 목표 위치 조정
        "aruco_goals": {
            0: ( -3.0,  15.15),   # green_id0 → 목표 A  ← ★
            1: ( 0.0,  15.15),   # red_id1   → 목표 B  ← ★
            2: ( 3.0, 15.15),   # blue_id2  → 목표 C  ← ★
        },
    },

    # ── IW Hub #1 ───────────────────────────────────────────────────
    # 역할: 물품 이송 (이동은 ROS2 /cmd_vel 토픽으로 제어)
    # 이름은 ROS2 토픽과 일치해야 한다: /iw_hub_01/cmd_vel, /iw_hub_01/odom
    {
        "type"      : "iw_hub",
        "name"      : "iw_hub_01",
        "spawn_xyz" : (-8.0, -14.0, 0.0),  # ★ 스폰 = WAIT_1 위치
        "spawn_yaw" : 90.0,                 # 긴 쪽이 y축 평행
    },

    # ── IW Hub #2 ───────────────────────────────────────────────────
    # 이름은 ROS2 토픽과 일치해야 한다: /iw_hub_02/cmd_vel, /iw_hub_02/odom
    {
        "type"      : "iw_hub",
        "name"      : "iw_hub_02",
        "spawn_xyz" : (-10.0, -14.0, 0.0), # ★ 스폰 = WAIT_3 위치
        "spawn_yaw" : 90.0,                 # 긴 쪽이 y축 평행
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
        "pad_reach" : 0.2,  # ★ EE→흡착패드끝 거리(m). 미지정 시 (stem+pad)×scale 자동계산
        # "movel_steps"      : 30,     # ★ MOVEL 속도 (↓값=빠름, 기본 60)

    },
]
