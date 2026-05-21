"""
두산 M0609 협동로봇 ↔ Firebase 양방향 ROS2 브릿지.

Isaac Sim → Firebase:
  /m0609/joint_states      → robots/m0609.joints
  /m0609/end_effector_pose → robots/m0609.position

Firebase → Isaac Sim:
  tasks/ 에서 pending 작업 생기면
    → /m0609/task_command (String JSON) 발행 — 작업 지시
    → /m0609/firebase_status (String) 발행 — 상태 동기화
"""

import json
import queue
import threading
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

from DB.robot_status import ArmManager
from DB.task_manager import TaskStatus


class ArmBridge(Node):
    """
    M0609 암 상태를 Firebase ↔ ROS2 간 양방향으로 동기화합니다.

    Isaac Sim ROS2 Bridge 설정 예:
      - Publish: /m0609/joint_states     (OmniGraph: ROS2 Publish Joint State)
      - Publish: /m0609/end_effector_pose (OmniGraph: ROS2 Publish Transform)
      - Subscribe: /m0609/task_command   (OmniGraph: ROS2 Subscribe String)
    """

    JOINT_NAMES = ["joint1", "joint2", "joint3",
                   "joint4", "joint5", "joint6"]

    def __init__(self, db, robot_id: str = "m0609",
                 update_interval: float = 0.5):
        super().__init__("arm_firebase_bridge")
        self._arm = ArmManager(db, robot_id)
        self._robot_id = robot_id
        self._db = db
        self._update_interval = update_interval
        self._last_joint_update = 0.0
        self._lock = threading.Lock()
        self._pub_queue: queue.Queue = queue.Queue()

        # 이미 발행한 task_id 추적 (중복 발행 방지)
        self._dispatched_tasks: set = set()

        # ── 구독 (Isaac Sim → Firebase) ────────────────────────
        self.create_subscription(
            JointState, f"/{robot_id}/joint_states", self._on_joint_states, 10)
        self.create_subscription(
            PoseStamped, f"/{robot_id}/end_effector_pose", self._on_ee_pose, 10)

        # 작업 완료 신호 수신 (Isaac Sim → Firebase task complete)
        self.create_subscription(
            String, f"/{robot_id}/task_done", self._on_task_done, 10)

        # ── 발행 (Firebase → Isaac Sim) ────────────────────────
        self._task_cmd_pub = self.create_publisher(
            String, f"/{robot_id}/task_command", 10)
        self._status_pub = self.create_publisher(
            String, f"/{robot_id}/firebase_status", 10)

        # Firebase → ROS2 큐 처리 타이머
        self.create_timer(0.1, self._flush_pub_queue)

        # ── Firestore 리스너 (tasks/ 변경 감지) ─────────────────
        self._tasks_watch = (db.collection("tasks")
                             .where("robot_id", "==", robot_id)
                             .where("status",   "==", TaskStatus.PENDING)
                             .on_snapshot(self._on_task_change))

        self.get_logger().info(f"[ArmBridge] 시작")

    # ── 내부 헬퍼 ──────────────────────────────────────────────

    def _flush_pub_queue(self):
        while not self._pub_queue.empty():
            try:
                topic, msg = self._pub_queue.get_nowait()
                if topic == "task_cmd":
                    self._task_cmd_pub.publish(msg)
                elif topic == "status":
                    self._status_pub.publish(msg)
            except queue.Empty:
                break

    # ── ROS2 구독 콜백 (Isaac Sim → Firebase) ──────────────────

    def _on_joint_states(self, msg: JointState):
        now = time.time()
        if now - self._last_joint_update < self._update_interval:
            return
        self._last_joint_update = now

        # 관절 이름 순서에 맞춰 정렬
        name_to_pos = dict(zip(msg.name, msg.position))
        joints = [
            round(float(name_to_pos.get(n, 0.0)), 3)
            for n in self.JOINT_NAMES
        ]

        with self._lock:
            self._arm.update_pose(
                x=0.0, y=0.0, z=0.0,   # ee_pose 콜백에서 별도 갱신
                joints=joints,
            )

    def _on_ee_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        with self._lock:
            self._arm.update_pose(x, y, z)

    def _on_task_done(self, msg: String):
        """
        Isaac Sim이 작업 완료를 알릴 때 호출.
        메시지 형식: {"task_id": "task_xxx", "result": "success"|"fail"}
        """
        try:
            data = json.loads(msg.data)
            task_id = data.get("task_id")
            result  = data.get("result", "success")
            if not task_id:
                return

            task_ref = self._db.collection("tasks").document(task_id)
            if result == "success":
                from DB.task_manager import TaskManager
                TaskManager(self._db).complete(task_id)
                self._arm.set_idle()
                self.get_logger().info(f"[Arm] 작업 완료: {task_id}")
            else:
                from DB.task_manager import TaskManager
                TaskManager(self._db).fail(task_id, reason=data.get("reason", ""))
                self.get_logger().warn(f"[Arm] 작업 실패: {task_id}")
        except json.JSONDecodeError:
            self.get_logger().error(f"[Arm] task_done 파싱 오류: {msg.data}")

    # ── Firestore 리스너 콜백 (Firebase → Isaac Sim) ────────────

    def _on_task_change(self, doc_snapshot, changes, read_time):
        """
        tasks/ 컬렉션에서 m0609의 pending 작업이 생기면 호출.
        /m0609/task_command 토픽으로 JSON 작업 지시를 발행합니다.

        발행 JSON 예:
        {
          "task_id":    "task_49051116",
          "item_id":    "ITEM-2836C439",
          "marker_id":  0,
          "destination": "A-1",
          "action":     "pick"
        }
        """
        for doc in doc_snapshot:
            data = doc.to_dict()
            if not data:
                continue

            task_id = data.get("task_id", doc.id)

            # 이미 발행한 작업은 건너뜀
            if task_id in self._dispatched_tasks:
                continue
            self._dispatched_tasks.add(task_id)

            cmd = {
                "task_id":     task_id,
                "item_id":     data.get("item_id", ""),
                "marker_id":   data.get("marker_id", -1),
                "destination": data.get("destination", ""),
                "action":      "pick",
            }

            msg = String()
            msg.data = json.dumps(cmd, ensure_ascii=False)
            self._pub_queue.put(("task_cmd", msg))
            self.get_logger().info(
                f"[Arm] 작업 지시 발행: {task_id}  "
                f"dest={cmd['destination']}")

    def destroy_node(self):
        self._tasks_watch.unsubscribe()
        super().destroy_node()
