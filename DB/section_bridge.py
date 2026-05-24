"""
DB/section_bridge.py
=====================
ROS2 토픽 → Firestore 실시간 동기화 노드.

구독 토픽:
    /{robot_name}/state   (std_msgs/String)   → m0609 / iw_hub 상태
    /{robot_name}/odom    (nav_msgs/Odometry)  → iw_hub 위치
    /section_{id}/pod_update (std_msgs/String, JSON) → Pod 상태

Firestore 쓰기:
    sections/{A|B|C}/robots/m0609/state
    sections/{A|B|C}/robots/iw_hub/state, location
    sections/{A|B|C}/pods/{pod_id}/state

실행:
    source /opt/ros/humble/setup.bash
    python3 DB/section_bridge.py
"""
import os
import sys
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import firebase_admin
from firebase_admin import credentials, firestore

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

# ── 섹션별 로봇 이름 매핑 ─────────────────────────────────────────────
SECTION_ROBOTS = {
    "A": {"m0609": "M0609_A",  "iw_hub": "iw_hub_01"},
    "B": {"m0609": "M0609_B",  "iw_hub": "iw_hub_02"},
    "C": {"m0609": "M0609_C",  "iw_hub": "iw_hub_03"},
}

# iw_hub odom 업데이트 최소 간격 (초)
ODOM_UPDATE_INTERVAL = 0.5

# m0609 / iw_hub 허용 state 값
VALID_ROBOT_STATES = {"working", "stop"}
# Pod 허용 state 값
VALID_POD_STATES   = {"full", "empty", "filling", "moving"}


class SectionBridge(Node):

    def __init__(self, db):
        super().__init__("section_bridge")
        self._db   = db
        self._lock = threading.Lock()
        self._last_odom: dict[str, float] = {}   # robot_name → last update time

        for section_id, robots in SECTION_ROBOTS.items():
            m0609_name  = robots["m0609"]
            iw_hub_name = robots["iw_hub"]

            # ── m0609 상태 구독 ────────────────────────────────────
            self.create_subscription(
                String,
                f"/{m0609_name}/state",
                lambda msg, sid=section_id: self._on_m0609_state(msg, sid),
                10,
            )

            # ── iw_hub 상태 구독 ───────────────────────────────────
            self.create_subscription(
                String,
                f"/{iw_hub_name}/state",
                lambda msg, sid=section_id: self._on_iw_hub_state(msg, sid),
                10,
            )

            # ── iw_hub 위치(odom) 구독 ─────────────────────────────
            self.create_subscription(
                Odometry,
                f"/{iw_hub_name}/odom",
                lambda msg, sid=section_id, rn=iw_hub_name: self._on_iw_hub_odom(msg, sid, rn),
                10,
            )

            # ── Pod 상태 업데이트 구독 ─────────────────────────────
            self.create_subscription(
                String,
                f"/section_{section_id}/pod_update",
                lambda msg, sid=section_id: self._on_pod_update(msg, sid),
                10,
            )

        self.get_logger().info("SectionBridge 시작 — 섹션 A / B / C 모니터링 중")

    # ── m0609 상태 콜백 ───────────────────────────────────────────────

    def _on_m0609_state(self, msg: String, section_id: str):
        state = msg.data.strip()
        if state not in VALID_ROBOT_STATES:
            self.get_logger().warn(f"[Section {section_id}] m0609 알 수 없는 state: {state}")
            return

        with self._lock:
            self._db.collection("sections").document(section_id).update({
                "robots.m0609.state": state,
                "last_updated": firestore.SERVER_TIMESTAMP,
            })
        self.get_logger().info(f"[Section {section_id}] m0609 state → {state}")

    # ── iw_hub 상태 콜백 ──────────────────────────────────────────────

    def _on_iw_hub_state(self, msg: String, section_id: str):
        state = msg.data.strip()
        if state not in VALID_ROBOT_STATES:
            self.get_logger().warn(f"[Section {section_id}] iw_hub 알 수 없는 state: {state}")
            return

        with self._lock:
            self._db.collection("sections").document(section_id).update({
                "robots.iw_hub.state": state,
                "last_updated": firestore.SERVER_TIMESTAMP,
            })
        self.get_logger().info(f"[Section {section_id}] iw_hub state → {state}")

    # ── iw_hub odom 콜백 ──────────────────────────────────────────────

    def _on_iw_hub_odom(self, msg: Odometry, section_id: str, robot_name: str):
        now = time.time()
        if now - self._last_odom.get(robot_name, 0) < ODOM_UPDATE_INTERVAL:
            return
        self._last_odom[robot_name] = now

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        with self._lock:
            self._db.collection("sections").document(section_id).update({
                "robots.iw_hub.location": {"x": round(x, 3), "y": round(y, 3)},
                "last_updated": firestore.SERVER_TIMESTAMP,
            })
        self.get_logger().debug(f"[Section {section_id}] iw_hub location → ({x:.2f}, {y:.2f})")

    # ── Pod 상태 콜백 ─────────────────────────────────────────────────

    def _on_pod_update(self, msg: String, section_id: str):
        """
        수신 JSON 형식:
            {"pod_id": "pod_01", "state": "full", "location": {"x": 1.0, "y": 2.0}}
        location 은 선택 항목.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[Section {section_id}] pod_update JSON 파싱 실패: {e}")
            return

        pod_id = data.get("pod_id")
        state  = data.get("state")

        if not pod_id or not state:
            self.get_logger().error(f"[Section {section_id}] pod_update 필드 누락: {data}")
            return

        if state not in VALID_POD_STATES:
            self.get_logger().warn(f"[Section {section_id}] pod 알 수 없는 state: {state}")
            return

        update = {"state": state}
        if "location" in data:
            update["location"] = data["location"]

        with self._lock:
            (self._db
             .collection("sections").document(section_id)
             .collection("pods").document(pod_id)
             .update(update))
        self.get_logger().info(f"[Section {section_id}] {pod_id} state → {state}")


# ── 진입점 ────────────────────────────────────────────────────────────

def main(args=None):
    # Firebase 초기화
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    print("[bridge] Firebase 연결 완료")

    # ROS2 초기화
    rclpy.init(args=args)
    node = SectionBridge(db)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
