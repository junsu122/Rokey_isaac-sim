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

# ── 경로 설정 ─────────────────────────────────────────────────────────
WAREHOUSE_USD     = "/home/rokey/Rokey_isaac-sim/main_isaac/usd/warehouse_v7_test_ver5.usda"
SPOT_SRC_DIR      = str(_SPOT  / "spot_test")
M0609_SRC_DIR     = str(_M0609 / "m0609_aruco_detect")

# Drone 관련 경로
DRONE_DEPS_DIR    = str(_DRONE / "drone_deps")
PEGASUS_SIM_DIR   = str(_DRONE / "pegasus_simulator")

# Spot용 그리퍼 USD (사전 컴파일된 USD 파일)
SPOT_GRIPPER_USD  = str(_SPOT / "onrobot_rg2" / "urdf" / "onrobot_rg2" / "onrobot_rg2.usd")

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
#  type        : "spot" | "m0609" | "drone"
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
_ARUCO_USD_DIR = "/home/rokey/Rokey_isaac-sim/aruco_marker_box/usd"

ARUCO_BOXES = [
    {
        "type": "green_id0",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_green_id0.usda",
        "xyz" : (12.3,  9.9, 0.025),   # ★ M0609_A 앞 큐브 위치
    },
    {
        "type": "red_id1",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_red_id1.usda",
        "xyz" : (12.3, -0.1, 0.025),   # ★ M0609_B 앞 큐브 위치
    },
    {
        "type": "blue_id2",
        "usd" : _ARUCO_USD_DIR + "/aruco_box_blue_id2.usda",
        "xyz" : (12.3, -10.1, 0.025),  # ★ M0609_C 앞 큐브 위치
    },
]

# ── Pod Stack 설정 ────────────────────────────────────────────────────
#
#  usd  : pod_stack_4_v2.usda 파일 경로 (고정)
#  xyz  : 스폰 위치 (x, y, z) [m]  ← 이 값을 수정해 위치 조정
#  yaw  : 회전각 deg (Z축 기준, 기본 0)
#
_POD_USD = "/home/rokey/Rokey_isaac-sim/main_isaac/usd/pod_stack_4_v2.usda"

POD_STACKS = [
    {"name": "PodStack_01", "usd": _POD_USD, "xyz": (-12.0, 7.35, 0.0), "yaw": 0.0},  # ★
    {"name": "PodStack_02", "usd": _POD_USD, "xyz": (-10.3, 0.0, 0.0), "yaw": 0.0},  # ★
    {"name": "PodStack_03", "usd": _POD_USD, "xyz": (-12.0, -7.5, 0.0), "yaw": 0.0},  # ★
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
    # 위치: 창고 중앙 통로 남측
    # 역할: 웨이포인트 순찰 + 블루 큐브 탐지 & 픽업 → 홈 복귀
    # {
    #     "type"      : "spot",
    #     "name"      : "Spot_01",
    #     "spawn_xyz" : ( 4.45,  0.0, 0.7),   # ★ 스폰 (x, y, z)
    #     "spawn_yaw" : 0.0,                   # ★ 초기 방향 (deg)
    #     "cube_xyz"  : ( 5.0,  0.0, 0.025),  # ★ 블루 큐브 초기 위치
    # },

    # ── Spot #2 ─────────────────────────────────────────────────────
    # 위치: 창고 중앙 통로 북측
    # 역할: 웨이포인트 순찰 + 블루 큐브 탐지 & 픽업 → 홈 복귀
    # {
    #     "type"      : "spot",
    #     "name"      : "Spot_02",
    #     "spawn_xyz" : ( 4.25,  5.7, 0.7),   # ★ 스폰
    #     "spawn_yaw" : 0.0,
    #     "cube_xyz"  : ( 5.0,  4.0, 0.025),  # ★ 블루 큐브 초기 위치
    # },

    # ── M0609 #1 ────────────────────────────────────────────────────
    # 위치: 창고 서측 작업 스테이션 A
    # 역할: ArUco 마커 시각 서보 → 큐브 픽 앤 플레이스
    {
        "type"       : "m0609",
        "name"       : "M0609_A",
        "spawn_xyz"  : (-13.2, 6.8, 0.0),
        "spawn_yaw"  : -90.0,
        "goal_xyz"   : (11.8, 9.6,  0.0),
        "scale"      : 2.0,
        "box_type"   : "green_id0",     # ★ ARUCO_BOXES 에서 연결할 박스
    },

    # ── M0609 #2 ────────────────────────────────────────────────────
    {
        "type"       : "m0609",
        "name"       : "M0609_B",
        "spawn_xyz"  : (-10.8, 1.2, 0.0),
        "spawn_yaw"  : 180.0,
        "goal_xyz"   : (11.8, -0.4,  0.0),
        "scale"      : 2.0,
        "box_type"   : "red_id1",
    },

    # ── M0609 #3 ────────────────────────────────────────────────────
    {
        "type"       : "m0609",
        "name"       : "M0609_C",
        "spawn_xyz"  : (-10.8, -7.0, 0.0),
        "spawn_yaw"  : 90.0,
        "goal_xyz"   : (11.8, -10.4,  0.0),
        "scale"      : 2.0,
        "box_type"   : "blue_id2",
    },
]
