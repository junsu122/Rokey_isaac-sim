from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── 창고 웨이포인트 (warehouse_v7 기준) ──────────────────────────────
# ★ 실제 창고 좌표에 맞게 수정 필요
WAYPOINTS: dict[str, tuple[float, float]] = {
    "WAIT_1":   ( -8.0, -14.0),  # iw_hub_01 대기 위치 (= 스폰 위치)
    "WAIT_2":   ( -9.0, -14.0),  # 예비 대기 위치
    "WAIT_3":   (-10.0, -14.0),  # iw_hub_02 대기 위치 (= 스폰 위치)
    "STACK_1":  (-12.8,  9.0),   # PodStack_01 픽업 위치 (robot_config.py 기준)
    "STACK_2":  ( -8.2,  1.5),   # PodStack_02 픽업 위치 (robot_config.py 기준)
    "STACK_3":  ( -9.7, -8.9),   # PodStack_03 픽업 위치 (robot_config.py 기준)
    "UNLOAD_1": (  4.0, -13.0),  # 언로드 위치 1
    "UNLOAD_2": (  4.0,  -3.0),  # 언로드 위치 2
    "UNLOAD_3": (  4.0,   7.0),  # 언로드 위치 3
}
