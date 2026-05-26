"""
DB/trigger_bridge.py
=====================
시뮬 → DB → 로봇 명령 통합 브릿지 노드.

ROS2 구독:
  /{m0609_name_lower}/work   String    "A_complete" 수신
  /{iw_hub_name}/odom        Odometry  iw_hub 위치 (단계별 pod state 전환)
  /{iw_hub_name}/work_done   String    swap 시퀀스 완료 신호

ROS2 발행:
  /{m0609_name}/work         String    "work_start"
  /{iw_hub_name}/command     String    JSON (swap)

swap 커맨드 형식:
  {
    "action":           "swap",
    "full_pod_id":      "pod_03",
    "pickup_pos":       {"x": ..., "y": ...},   ← CONVEYOR_WAIT_POS (고정)
    "return_pos":       {"x": ..., "y": ...},   ← 빈 격자 슬롯
    "empty_pod_id":     "pod_07",
    "empty_pickup_pos": {"x": ..., "y": ...},   ← empty pod 격자 위치
    "deliver_pos":      {"x": ..., "y": ...}    ← CONVEYOR_WAIT_POS (고정)
  }

iw_hub swap 단계별 pod state 전환 (odom 기반):
  phase 1 — belt 도착       : full pod  full    → moving
  phase 2 — 격자 슬롯 도착  : full pod  moving  → full  (location 업데이트)
  phase 3 — empty pod 도착  : empty pod empty   → moving
  phase 4 — belt 재도착     : empty pod moving  → filling (location = belt)

work_done 수신:
  m0609 state → working, "work_start" 발행

실행:
    source /opt/ros/humble/setup.bash
    python3 DB/trigger_bridge.py
"""
import os
import sys
import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import firebase_admin
from firebase_admin import credentials, firestore

sys.path.insert(0, os.path.dirname(__file__))
from init_db import SECTOR_POD_LAYOUT, POD_X_COLS

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

SECTION_ROBOTS = {
    "A": {"m0609": "M0609_A", "iw_hub": "iw_hub_01"},
    "B": {"m0609": "M0609_B", "iw_hub": "iw_hub_02"},
    "C": {"m0609": "M0609_C", "iw_hub": "iw_hub_03"},
}

def _m0609_pub_topic(name: str) -> str:
    return f"/{name[0].lower()}{name[1:]}/work"

# 섹터별 컨베이어 앞 대기 좌표 — TODO: 실제 좌표 확정 후 수정
CONVEYOR_WAIT_POS = {
    "A": {"x": -12.8, "y":  9.0},
    "B": {"x":  -8.2, "y":  1.5},
    "C": {"x":  -9.7, "y": -8.9},
}

ODOM_UPDATE_INTERVAL  = 0.5
WAIT_RADIUS           = 0.5   # belt 도착 판정 반경 (m) — TODO: 확정 후 수정
GRID_ARRIVAL_RADIUS   = 0.3   # 격자 위치 도착 판정 반경 (m) — TODO: 확정 후 수정


# ── 헬퍼 ─────────────────────────────────────────────────────────────

def _all_grid_positions(section_id: str) -> list[dict]:
    layout = SECTOR_POD_LAYOUT[section_id]
    return [
        {"x": float(x), "y": float(y)}
        for y in layout["y_rows"]
        for x in POD_X_COLS
    ]


def _find_empty_grid_slot(section_id: str, pods: list[dict]) -> dict | None:
    """현재 pod가 점유하지 않은 격자 슬롯 반환."""
    occupied = {
        (round(p["location"]["x"], 2), round(p["location"]["y"], 2))
        for p in pods
        if p.get("location") and p.get("state") != "moving"
    }
    for pos in _all_grid_positions(section_id):
        if (round(pos["x"], 2), round(pos["y"], 2)) not in occupied:
            return pos
    return None


def _farthest_empty(pods: list[dict]) -> dict | None:
    """격자 원점에서 가장 먼 empty pod 반환."""
    empties = [p for p in pods if p.get("state") == "empty"]
    if not empties:
        return None
    return max(empties, key=lambda p: math.hypot(p["location"]["x"], p["location"]["y"]))


def _arrived(pos: dict, x: float, y: float, radius: float) -> bool:
    return math.hypot(x - pos["x"], y - pos["y"]) <= radius


# ── 노드 ─────────────────────────────────────────────────────────────

class TriggerBridge(Node):

    def __init__(self, db):
        super().__init__("trigger_bridge")
        self._db   = db
        self._lock = threading.Lock()
        self._last_odom: dict[str, float] = {}

        # 섹션별 swap 진행 상태
        # phase 1: iw_hub → belt        (full pod: full → moving)
        # phase 2: iw_hub → return_pos  (full pod: moving → full)
        # phase 3: iw_hub → empty pos   (empty pod: empty → moving)
        # phase 4: iw_hub → belt        (empty pod: moving → filling)
        self._pending_swap: dict[str, dict] = {}

        self._m0609_pub : dict[str, any] = {}
        self._iw_hub_pub: dict[str, any] = {}

        for section_id, robots in SECTION_ROBOTS.items():
            m0609_name  = robots["m0609"]
            iw_hub_name = robots["iw_hub"]

            self._m0609_pub[section_id] = self.create_publisher(
                String, f"/{m0609_name}/work", 10
            )
            self._iw_hub_pub[section_id] = self.create_publisher(
                String, f"/{iw_hub_name}/command", 10
            )

            # m0609 work_complete 구독
            self.create_subscription(
                String,
                _m0609_pub_topic(m0609_name),
                lambda msg, sid=section_id: self._on_m0609_complete(msg, sid),
                10,
            )

            # iw_hub odom 구독
            self.create_subscription(
                Odometry,
                f"/{iw_hub_name}/odom",
                lambda msg, sid=section_id, rn=iw_hub_name: self._on_iw_hub_odom(msg, sid, rn),
                10,
            )

            # iw_hub work_done 구독
            self.create_subscription(
                String,
                f"/{iw_hub_name}/work_done",
                lambda msg, sid=section_id: self._on_iw_hub_work_done(msg, sid),
                10,
            )

        self.get_logger().info("TriggerBridge 시작 — 섹션 A / B / C 감시 중")

    # ── m0609 work_complete 콜백 ──────────────────────────────────────

    def _on_m0609_complete(self, msg: String, section_id: str):
        if not msg.data.endswith("_complete"):
            return

        self.get_logger().info(f"[Section {section_id}] m0609 work_complete 수신")

        with self._lock:
            pods_ref = self._db.collection("sections").document(section_id).collection("pods")
            all_pods = [s.to_dict() for s in pods_ref.stream()]

            filling = [p for p in all_pods if p.get("state") == "filling"]
            if not filling:
                self.get_logger().warn(f"[Section {section_id}] filling pod 없음 — 스킵")
                return
            full_pod = filling[0]

            empty_pod = _farthest_empty(all_pods)
            if not empty_pod:
                self.get_logger().warn(f"[Section {section_id}] empty pod 없음 — swap 불가")
                return

            return_pos = _find_empty_grid_slot(section_id, all_pods)
            if not return_pos:
                self.get_logger().error(f"[Section {section_id}] 빈 격자 슬롯 없음 — swap 불가")
                return

            # DB: filling → full, m0609 → wait
            pods_ref.document(full_pod["pod_id"]).update({"state": "full"})
            self._db.collection("sections").document(section_id).update({
                "robots.m0609.state": "wait",
                "last_updated": firestore.SERVER_TIMESTAMP,
            })

            # swap 진행 정보 저장
            self._pending_swap[section_id] = {
                "phase":          1,
                "full_pod_id":    full_pod["pod_id"],
                "full_return_pos": return_pos,
                "empty_pod_id":   empty_pod["pod_id"],
                "empty_pos":      empty_pod["location"],
            }

            # iw_hub swap 명령 발행
            self._send_iw_hub(section_id, {
                "action":           "swap",
                "full_pod_id":      full_pod["pod_id"],
                "pickup_pos":       CONVEYOR_WAIT_POS[section_id],
                "return_pos":       return_pos,
                "empty_pod_id":     empty_pod["pod_id"],
                "empty_pickup_pos": empty_pod["location"],
                "deliver_pos":      CONVEYOR_WAIT_POS[section_id],
            })
            self.get_logger().info(
                f"[Section {section_id}] swap 명령 발행 — "
                f"full:{full_pod['pod_id']} return:{return_pos}, "
                f"empty:{empty_pod['pod_id']}"
            )

    # ── iw_hub odom 콜백 ─────────────────────────────────────────────

    def _on_iw_hub_odom(self, msg: Odometry, section_id: str, robot_name: str):
        now = time.time()
        if now - self._last_odom.get(robot_name, 0) < ODOM_UPDATE_INTERVAL:
            return
        self._last_odom[robot_name] = now

        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)

        belt_pos = CONVEYOR_WAIT_POS[section_id]
        at_belt  = _arrived(belt_pos, x, y, WAIT_RADIUS)
        iw_state = "wait" if at_belt else "working"

        self._db.collection("sections").document(section_id).update({
            "robots.iw_hub.location": {"x": x, "y": y},
            "robots.iw_hub.state":    iw_state,
            "last_updated": firestore.SERVER_TIMESTAMP,
        })

        swap = self._pending_swap.get(section_id)
        if not swap:
            return

        pods_ref = self._db.collection("sections").document(section_id).collection("pods")
        phase    = swap["phase"]

        # phase 1: belt 도착 → full pod: full → moving
        if phase == 1 and at_belt:
            pods_ref.document(swap["full_pod_id"]).update({"state": "moving"})
            swap["phase"] = 2
            self.get_logger().info(
                f"[Section {section_id}] phase1 완료: {swap['full_pod_id']} full → moving"
            )

        # phase 2: 격자 슬롯 도착 → full pod: moving → full (위치 업데이트)
        elif phase == 2 and _arrived(swap["full_return_pos"], x, y, GRID_ARRIVAL_RADIUS):
            pods_ref.document(swap["full_pod_id"]).update({
                "state":    "full",
                "location": swap["full_return_pos"],
            })
            swap["phase"] = 3
            self.get_logger().info(
                f"[Section {section_id}] phase2 완료: {swap['full_pod_id']} moving → full @ {swap['full_return_pos']}"
            )

        # phase 3: empty pod 위치 도착 → empty pod: empty → moving
        elif phase == 3 and _arrived(swap["empty_pos"], x, y, GRID_ARRIVAL_RADIUS):
            pods_ref.document(swap["empty_pod_id"]).update({"state": "moving"})
            swap["phase"] = 4
            self.get_logger().info(
                f"[Section {section_id}] phase3 완료: {swap['empty_pod_id']} empty → moving"
            )

        # phase 4: belt 재도착 → empty pod: moving → filling (위치 업데이트)
        elif phase == 4 and at_belt:
            pods_ref.document(swap["empty_pod_id"]).update({
                "state":    "filling",
                "location": CONVEYOR_WAIT_POS[section_id],
            })
            swap["phase"] = 5
            self.get_logger().info(
                f"[Section {section_id}] phase4 완료: {swap['empty_pod_id']} moving → filling"
            )

    # ── iw_hub work_done 콜백 ─────────────────────────────────────────

    def _on_iw_hub_work_done(self, msg: String, section_id: str):
        with self._lock:
            if section_id in self._pending_swap:
                del self._pending_swap[section_id]

            # m0609 work_start
            start_msg = String()
            start_msg.data = "work_start"
            self._m0609_pub[section_id].publish(start_msg)

            self._db.collection("sections").document(section_id).update({
                "robots.m0609.state": "working",
                "last_updated": firestore.SERVER_TIMESTAMP,
            })
            self.get_logger().info(
                f"[Section {section_id}] work_done 수신 → m0609 work_start 발행"
            )

    # ── 발행 헬퍼 ────────────────────────────────────────────────────

    def _send_iw_hub(self, section_id: str, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self._iw_hub_pub[section_id].publish(msg)


# ── 진입점 ────────────────────────────────────────────────────────────

def main(args=None):
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    print("[trigger] Firebase 연결 완료")

    rclpy.init(args=args)
    node = TriggerBridge(db)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
