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
    "WAIT_1":   (-10.0,  7.0),   # ★ 대기 위치 1
    "WAIT_2":   (-10.0,  0.0),   # ★ 대기 위치 2
    "WAIT_3":   (-10.0, -7.0),   # ★ 대기 위치 3
    "STACK_1":  (-12.0,  7.35),  # PodStack_01 픽업 위치
    "STACK_2":  (-10.3,  0.0),   # PodStack_02 픽업 위치
    "STACK_3":  (-12.0, -7.5),   # PodStack_03 픽업 위치
    "UNLOAD_1": ( 11.8,  9.6),   # M0609_A 언로드 위치
    "UNLOAD_2": ( 11.8, -0.4),   # M0609_B 언로드 위치
    "UNLOAD_3": ( 11.8, -10.4),  # M0609_C 언로드 위치
}
