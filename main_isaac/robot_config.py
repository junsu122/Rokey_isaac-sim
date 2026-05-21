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

# ── 경로 설정 ─────────────────────────────────────────────────────────
WAREHOUSE_USD     = str(_BASE / "usd" / "warehouse_v3.usd")
SPOT_SRC_DIR      = str(_SPOT  / "spot_test")
M0609_SRC_DIR     = str(_M0609 / "m0609_aruco_detect")

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

# ── 로봇 스폰 설정 ──────────────────────────────────────────────────
#
#  type      : "spot"  또는  "m0609"
#  name      : 고유 이름 (USD prim path 에 사용됨)
#  spawn_xyz : (x, y, z)  단위 m  ← 이 값을 수정해 위치 조정
#  spawn_yaw : 초기 방향각 deg   (Z축 기준 반시계 +)
#  cube_xyz  : 집을 큐브 초기 위치  (두 타입 공통)
#  goal_xyz  : 놓을 목표 위치       (m0609 전용, spot 은 미사용)
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




ROBOT_REGISTRY = [

    # ── Spot #1 ─────────────────────────────────────────────────────
    # 위치: 창고 중앙 통로 남측
    # 역할: 웨이포인트 순찰 + 블루 큐브 탐지 & 픽업 → 홈 복귀
    {
        "type"      : "spot",
        "name"      : "Spot_01",
        "spawn_xyz" : ( 4.45,  0.0, 0.7),   # ★ 스폰 (x, y, z)
        "spawn_yaw" : 0.0,                   # ★ 초기 방향 (deg)
        "cube_xyz"  : ( 5.0,  0.0, 0.025),  # ★ 블루 큐브 초기 위치
    },

    # ── Spot #2 ─────────────────────────────────────────────────────
    # 위치: 창고 중앙 통로 북측
    # 역할: 웨이포인트 순찰 + 블루 큐브 탐지 & 픽업 → 홈 복귀
    {
        "type"      : "spot",
        "name"      : "Spot_02",
        "spawn_xyz" : ( 4.25,  5.7, 0.7),   # ★ 스폰
        "spawn_yaw" : 0.0,
        "cube_xyz"  : ( 5.0,  4.0, 0.025),  # ★ 블루 큐브 초기 위치
    },

    # ── M0609 #1 ────────────────────────────────────────────────────
    # 위치: 창고 서측 작업 스테이션 A
    # 역할: ArUco 마커 시각 서보 → 큐브 픽 앤 플레이스
    {
        "type"      : "m0609",
        "name"      : "M0609_01",
        "spawn_xyz" : (0.0,  5.8, 0.0),    # ★ 스폰
        "spawn_yaw" : 0.0,
        "cube_xyz"  : (-1.6,  0.2, 0.025),  # ★ 큐브 초기 위치
        "goal_xyz"  : (-1.45,-0.35, 0.0),   # ★ 목표(place) 위치
    },

    # ── M0609 #2 ────────────────────────────────────────────────────
    # 위치: 창고 서측 작업 스테이션 B
    # 역할: ArUco 마커 시각 서보 → 큐브 픽 앤 플레이스
    {
        "type"      : "m0609",
        "name"      : "M0609_02",
        "spawn_xyz" : (0.0,  8.2, 0.0),    # ★ 스폰
        "spawn_yaw" : 0.0,
        "cube_xyz"  : (-1.6,  4.2, 0.025),  # ★ 큐브 초기 위치
        "goal_xyz"  : (-1.45, 3.65, 0.0),   # ★ 목표(place) 위치
    },
]
