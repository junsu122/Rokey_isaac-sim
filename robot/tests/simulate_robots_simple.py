"""
ROS2 없이 Firebase를 직접 업데이트해서 대시보드 이동을 테스트합니다.

실행:
  cd /path/to/robot
  python3 tests/simulate_robots_simple.py
"""

import sys
import time
import math
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import sys as _sys
from pathlib import Path as _Path
_root = _Path(__file__).resolve().parent
while not (_root / "DB").exists() and _root.parent != _root:
    _root = _root.parent
if str(_root) not in _sys.path:
    _sys.path.insert(0, str(_root))
del _root

from DB.firebase_manager import init_firebase
from DB.robot_status import RobotFleet

# 웨이포인트: (x, y, yaw)
# 시작 → A-1 → 분류장소 → 강남 → 복귀
AMR_WAYPOINTS = [
    (0.0,   0.0,   0.0),
    (-0.4,  0.3,  135.0),
    (0.2,   0.1,   0.0),
    (0.8,   0.0,  -20.0),
    (1.5,   0.5,   45.0),
    (0.8,   0.0,  180.0),
    (0.0,   0.0,  180.0),
]

DRONE_WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (0.2, 0.1, 1.5),
    (0.8, 0.2, 2.0),
    (1.2, 0.3, 2.0),
    (0.8, 0.2, 1.0),
    (0.0, 0.0, 0.0),
]

def lerp(a, b, t):
    return a + (b - a) * t

def interpolate(waypoints, steps=25):
    path = []
    for i in range(len(waypoints) - 1):
        for s in range(steps):
            t = s / steps
            p = tuple(lerp(waypoints[i][j], waypoints[i+1][j], t)
                      for j in range(len(waypoints[i])))
            path.append(p)
    path.append(waypoints[-1])
    return path


def main():
    print("Firebase 연결 중...")
    db = init_firebase()
    fleet = RobotFleet(db)
    print("연결 완료\n")

    amr_path   = interpolate(AMR_WAYPOINTS,   steps=25)
    drone_path = interpolate(DRONE_WAYPOINTS, steps=20)
    total = max(len(amr_path), len(drone_path))
    battery = 95.0

    arm_events = {
        int(total * 0.15): ("picking", "sim_task_001"),
        int(total * 0.35): ("placing", None),
        int(total * 0.50): ("idle",    None),
    }

    print(f"시뮬레이션 시작 — {total}스텝 × 0.3s = 약 {total*0.3:.0f}초")
    print("브라우저에서 http://localhost:5173 을 열고 로봇 아이콘 이동을 확인하세요.\n")

    for i in range(total):
        # AMR 위치
        if i < len(amr_path):
            x, y, yaw = amr_path[i]
            fleet.amr.update_pose(x, y, yaw, speed=0.3)
            battery = max(20.0, battery - 0.04)
            fleet.amr.update_battery(battery)

        # 드론 위치
        if i < len(drone_path):
            dx, dy, dz = drone_path[i]
            fleet.drone.update_pose(dx, dy, dz)
            fleet.drone.update_battery(max(20.0, 87.5 - i * 0.05))

        # 암 상태 전환
        if i in arm_events:
            state, task_id = arm_events[i]
            if state == "picking":
                fleet.arm.set_picking(task_id)
                fleet.arm.set_detected_item(0, "Apple Watch", "item",
                                            (0.05, 0.0, 0.42))
                print(f"  [암] → picking  (Apple Watch 감지)")
            elif state == "placing":
                fleet.arm.set_placing()
                print(f"  [암] → placing")
            elif state == "idle":
                fleet.arm.set_idle()
                print(f"  [암] → idle")

        # 진행률 출력
        if i % 25 == 0:
            pct = (i / total) * 100
            ix = min(i, len(amr_path) - 1)
            x, y, _ = amr_path[ix]
            print(f"  [{pct:5.1f}%] AMR ({x:+.2f}, {y:+.2f})  배터리 {battery:.1f}%")

        time.sleep(0.3)

    print("\n시뮬레이션 완료 — 대시보드에서 최종 상태를 확인하세요.")


if __name__ == "__main__":
    main()
