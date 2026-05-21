"""
AMR (amr_001) ↔ Firebase 양방향 ROS2 브릿지.

Isaac Sim → Firebase:
  /amr_001/odom          → robots/amr_001.position, speed
  /amr_001/battery_state → robots/amr_001.battery
  /amr_001/cmd_vel       → (속도 로깅)

Firebase → Isaac Sim:
  navigation/amr_001 변경 시
    → /amr_001/goal (PoseStamped) 발행 — AMR 이동 목표
    → /amr_001/firebase_status (String) 발행 — 상태 동기화
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
from geometry_msgs.msg import PoseStamped, Twist
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

from DB.firebase_manager import now_ts
from DB.robot_status import AMRManager


class AMRBridge(Node):
    """
    AMR 상태를 Firebase ↔ ROS2 간 양방향으로 동기화합니다.

    Isaac Sim ROS2 Bridge 설정 예:
      - Publish: /amr_001/odom  (OmniGraph: ROS2 Publish Odometry)
      - Publish: /amr_001/cmd_vel (OmniGraph: ROS2 Publish Twist)
      - Subscribe: /amr_001/goal (OmniGraph: ROS2 Subscribe PoseStamped)
    """

    def __init__(self, db, robot_id: str = "amr_001",
                 update_interval: float = 0.5):
        super().__init__("amr_firebase_bridge")
        self._amr = AMRManager(db, robot_id)
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
        self.create_subscription(
            Twist, f"/{robot_id}/cmd_vel", self._on_cmd_vel, 10)

        # ── 발행 (Firebase → Isaac Sim) ────────────────────────
        self._goal_pub = self.create_publisher(
            PoseStamped, f"/{robot_id}/goal", 10)
        self._status_pub = self.create_publisher(
            String, f"/{robot_id}/firebase_status", 10)

        # Firebase → ROS2 큐를 처리하는 타이머 (100ms 주기)
        self.create_timer(0.1, self._flush_pub_queue)

        # ── Firestore 리스너 (navigation 변경 → ROS2 goal 발행) ─
        self._section_map = self._load_section_map(db)
        self._nav_watch = (db.collection("navigation")
                           .document(robot_id)
                           .on_snapshot(self._on_nav_change))

        self.get_logger().info(
            f"[AMRBridge] 시작 — 섹션 맵: {list(self._section_map.keys())}")

    # ── 내부 헬퍼 ──────────────────────────────────────────────

    def _load_section_map(self, db) -> dict:
        """sections/ 컬렉션 → {section_id: {x, y, z}} 맵."""
        return {
            d.to_dict()["section_id"]: d.to_dict()["position"]
            for d in db.collection("sections").stream()
        }

    def _make_pose_stamped(self, x: float, y: float, yaw_deg: float = 0.0
                           ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        half = math.radians(yaw_deg) / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        return msg

    def _flush_pub_queue(self):
        """타이머 콜백 — Firestore 스레드에서 쌓인 발행 요청을 처리."""
        while not self._pub_queue.empty():
            try:
                topic, msg = self._pub_queue.get_nowait()
                if topic == "goal":
                    self._goal_pub.publish(msg)
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

        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = math.degrees(2.0 * math.atan2(qz, qw))
        vx  = msg.twist.twist.linear.x
        vy  = msg.twist.twist.linear.y
        speed = math.sqrt(vx ** 2 + vy ** 2)

        with self._lock:
            self._amr.update_pose(x, y, yaw, speed)

    def _on_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        with self._lock:
            self._amr.update_battery(pct)

    def _on_cmd_vel(self, msg: Twist):
        # cmd_vel은 빈번하므로 Firebase 쓰기 없이 로그만
        speed = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        self.get_logger().debug(f"[AMR] cmd_vel speed={speed:.3f} m/s")

    # ── Firestore 리스너 콜백 (Firebase → Isaac Sim) ────────────

    def _on_nav_change(self, doc_snapshot, changes, read_time):
        """
        navigation/amr_001 문서가 바뀔 때 호출됩니다.
        Firestore 스레드에서 실행되므로 _pub_queue에 넣고 반환합니다.
        """
        for doc in doc_snapshot:
            data = doc.to_dict()
            if not data:
                continue

            target = data.get("current_target")
            status = data.get("status", "idle")

            # navigating 상태 → 목표 좌표를 /amr_001/goal 으로 발행
            if status == "navigating" and target:
                pos = self._section_map.get(target)
                if pos:
                    goal_msg = self._make_pose_stamped(pos["x"], pos["y"])
                    self._pub_queue.put(("goal", goal_msg))
                    self.get_logger().info(
                        f"[AMR] Firebase→ROS goal: {target} "
                        f"({pos['x']:.2f}, {pos['y']:.2f})")

            # 상태 문자열 발행
            status_msg = String()
            status_msg.data = (
                f'{{"status":"{status}",'
                f'"target":"{target or ""}"}}'
            )
            self._pub_queue.put(("status", status_msg))

    def destroy_node(self):
        self._nav_watch.unsubscribe()
        super().destroy_node()
