"""
드론 (drone_001) ↔ Firebase 양방향 ROS2 브릿지.

Isaac Sim → Firebase:
  /drone_001/odom          → robots/drone_001.position, altitude, speed
  /drone_001/battery_state → robots/drone_001.battery

Firebase → Isaac Sim:
  robots/drone_001 status 변경 시
    → /drone_001/pose_command (PoseStamped) 발행 — 이동 목표
    → /drone_001/firebase_status (String) 발행
"""

import math
import queue
import threading
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

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

from DB.robot_status import DroneManager


class DroneBridge(Node):
    """
    드론 상태를 Firebase ↔ ROS2 간 양방향으로 동기화합니다.

    Isaac Sim ROS2 Bridge 설정 예:
      - Publish: /drone_001/odom  (OmniGraph: ROS2 Publish Odometry)
      - Subscribe: /drone_001/pose_command (OmniGraph: ROS2 Subscribe PoseStamped)
    """

    def __init__(self, db, robot_id: str = "drone_001",
                 update_interval: float = 0.5):
        super().__init__("drone_firebase_bridge")
        self._drone = DroneManager(db, robot_id)
        self._robot_id = robot_id
        self._update_interval = update_interval
        self._last_pose_update = 0.0
        self._lock = threading.Lock()
        self._pub_queue: queue.Queue = queue.Queue()

        # ── 구독 (Isaac Sim → Firebase) ────────────────────────
        self.create_subscription(
            Odometry, f"/{robot_id}/odom", self._on_odom, 10)
        self.create_subscription(
            BatteryState, f"/{robot_id}/battery_state", self._on_battery, 10)

        # ── 발행 (Firebase → Isaac Sim) ────────────────────────
        self._pose_cmd_pub = self.create_publisher(
            PoseStamped, f"/{robot_id}/pose_command", 10)
        self._status_pub = self.create_publisher(
            String, f"/{robot_id}/firebase_status", 10)

        # Firebase → ROS2 큐 처리 타이머
        self.create_timer(0.1, self._flush_pub_queue)

        # ── Firestore 리스너 (로봇 상태 변경 감지) ─────────────
        self._robot_watch = (db.collection("robots")
                             .document(robot_id)
                             .on_snapshot(self._on_robot_change))

        self.get_logger().info(f"[DroneBridge] 시작")

    # ── 내부 헬퍼 ──────────────────────────────────────────────

    def _make_pose_stamped(self, x: float, y: float, z: float,
                           yaw_deg: float = 0.0) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        half = math.radians(yaw_deg) / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        return msg

    def _flush_pub_queue(self):
        while not self._pub_queue.empty():
            try:
                topic, msg = self._pub_queue.get_nowait()
                if topic == "pose_cmd":
                    self._pose_cmd_pub.publish(msg)
                elif topic == "status":
                    self._status_pub.publish(msg)
            except queue.Empty:
                break

    # ── ROS2 구독 콜백 (Isaac Sim → Firebase) ──────────────────

    def _on_odom(self, msg: Odometry):
        now = time.time()
        if now - self._last_pose_update < self._update_interval:
            return
        self._last_pose_update = now

        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        z  = msg.pose.pose.position.z
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        heading = math.degrees(2.0 * math.atan2(qz, qw)) % 360.0
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        speed = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        with self._lock:
            self._drone.update_pose(x, y, z, heading, speed)

    def _on_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        with self._lock:
            self._drone.update_battery(pct)

    # ── Firestore 리스너 콜백 (Firebase → Isaac Sim) ────────────

    def _on_robot_change(self, doc_snapshot, changes, read_time):
        """
        robots/drone_001 문서 변경 시 호출.
        cargo_status가 'transporting'이면 목표 위치를 발행합니다.
        """
        for doc in doc_snapshot:
            data = doc.to_dict()
            if not data:
                continue

            cargo  = data.get("cargo_status", "empty")
            status = data.get("charge_status", "operating")

            # 상태 문자열 발행
            status_msg = String()
            status_msg.data = (
                f'{{"cargo_status":"{cargo}",'
                f'"charge_status":"{status}"}}'
            )
            self._pub_queue.put(("status", status_msg))

    def destroy_node(self):
        self._robot_watch.unsubscribe()
        super().destroy_node()
