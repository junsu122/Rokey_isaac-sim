"""
ROS2 없이 가짜 토픽을 발행해서 Firebase 업데이트를 테스트합니다.

실행:
  source /opt/ros/humble/setup.bash
  python3 tests/simulate_robots.py

시나리오:
  - AMR: A-1 구획 → 강남 배송지로 이동 (맵에서 아이콘 이동 확인)
  - 드론: 이륙 → 호버링 → 착륙
  - M0609: idle → picking → placing → idle
"""

import sys
import time
import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

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


# ── AMR 이동 경로 웨이포인트 ──────────────────────────────────
# 시작(0,0) → A-1(-0.4, 0.3) → 분류장소(0.8, 0.0) → 강남(1.5, 0.5) → 복귀(0,0)
AMR_WAYPOINTS = [
    (0.0,   0.0,  0.0),
    (-0.4,  0.3,  135.0),
    (0.0,   0.15, 0.0),
    (0.8,   0.0,  -30.0),
    (1.5,   0.5,  45.0),
    (0.8,   0.0,  180.0),
    (0.0,   0.0,  180.0),
]

DRONE_PATH = [
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 2.0),
    (0.5, 0.2, 2.0),
    (1.0, 0.3, 1.5),
    (0.5, 0.2, 1.0),
    (0.0, 0.0, 0.0),
]


def lerp(a, b, t):
    return a + (b - a) * t


def interpolate_path(waypoints, steps_per_segment=20):
    """웨이포인트 목록을 부드러운 연속 좌표로 보간."""
    path = []
    for i in range(len(waypoints) - 1):
        for s in range(steps_per_segment):
            t = s / steps_per_segment
            p = tuple(lerp(waypoints[i][j], waypoints[i+1][j], t)
                      for j in range(len(waypoints[i])))
            path.append(p)
    path.append(waypoints[-1])
    return path


class FakePublisher(Node):
    """가짜 ROS2 토픽 발행 노드."""

    def __init__(self):
        super().__init__("fake_robot_publisher")
        self._amr_odom  = self.create_publisher(Odometry,     "/amr_001/odom",          10)
        self._amr_bat   = self.create_publisher(BatteryState, "/amr_001/battery_state",  10)
        self._drone_odom= self.create_publisher(Odometry,     "/drone_001/odom",         10)

    def pub_amr_odom(self, x, y, yaw_deg, speed=0.3):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        half = math.radians(yaw_deg) / 2
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.twist.twist.linear.x = speed
        self._amr_odom.publish(msg)

    def pub_amr_battery(self, pct):
        msg = BatteryState()
        msg.percentage = pct / 100.0
        self._amr_bat.publish(msg)

    def pub_drone_odom(self, x, y, z):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        self._drone_odom.publish(msg)


def run_simulation(db):
    """Firebase 직접 업데이트 시뮬레이션 (ROS2 브릿지 없이)."""
    fleet = RobotFleet(db)

    amr_path   = interpolate_path(AMR_WAYPOINTS,   steps_per_segment=30)
    drone_path = interpolate_path(DRONE_PATH,       steps_per_segment=25)

    total = max(len(amr_path), len(drone_path))
    battery = 100.0

    arm_states = ["idle", "picking", "placing", "idle"]
    arm_times  = [0, int(total * 0.2), int(total * 0.4), int(total * 0.6)]
    arm_idx = 0

    print(f"[시뮬레이션 시작] 총 {total} 스텝 (간격 0.2s = 약 {total * 0.2:.0f}초)")
    print("대시보드 http://localhost:5173 에서 로봇 아이콘 이동을 확인하세요.\n")

    for i in range(total):
        # ── AMR 위치 업데이트 ──────────────────────────────────
        if i < len(amr_path):
            x, y, yaw = amr_path[i]
            fleet.amr.update_pose(x, y, yaw, speed=0.3)
            battery = max(20.0, battery - 0.05)
            fleet.amr.update_battery(battery)

        # ── 드론 위치 업데이트 ─────────────────────────────────
        if i < len(drone_path):
            dx, dy, dz = drone_path[i]
            fleet.drone.update_pose(dx, dy, dz)

        # ── 암 상태 전환 ───────────────────────────────────────
        if arm_idx < len(arm_times) - 1 and i >= arm_times[arm_idx + 1]:
            arm_idx += 1
            state = arm_states[arm_idx]
            if state == "picking":
                fleet.arm.set_picking("sim_task_001")
                fleet.arm.set_detected_item(0, "Apple Watch", "item",
                                            (0.05, 0.0, 0.42))
            elif state == "placing":
                fleet.arm.set_placing()
            elif state == "idle":
                fleet.arm.set_idle()
            print(f"  [암] 상태 → {state}")

        # ── 진행률 출력 ────────────────────────────────────────
        if i % 30 == 0:
            pct = (i / total) * 100
            ix = min(i, len(amr_path) - 1)
            x, y, _ = amr_path[ix]
            print(f"  [{pct:5.1f}%] AMR ({x:.2f}, {y:.2f})  배터리 {battery:.1f}%")

        time.sleep(0.2)

    print("\n[시뮬레이션 완료]")


def main():
    rclpy.init()
    node = FakePublisher()

    db = init_firebase()

    # ROS2 스핀을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_simulation(db)
    except KeyboardInterrupt:
        print("\n[중단됨]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
