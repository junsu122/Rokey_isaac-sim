"""
tasks/ 컬렉션 관리.
물품 검출 시 작업을 생성하고 로봇에게 할당합니다.
"""

from firebase_admin import firestore
from .firebase_manager import now_ts
import uuid

# 작업 상태값 상수
class TaskStatus:
    PENDING     = "pending"      # 로봇 할당 대기
    IN_PROGRESS = "in_progress"  # 로봇 수행 중
    COMPLETED   = "completed"    # 완료
    FAILED      = "failed"       # 실패


class TaskManager:
    def __init__(self, db: firestore.Client):
        self._col = db.collection("tasks")

    def create(self, item_id: str, marker_id: int,
               destination: str, robot_id: str = "m0609") -> str:
        """
        물품 검출 시 작업을 생성합니다.
        Returns: 생성된 task_id
        """
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        self._col.document(task_id).set({
            "task_id":     task_id,
            "item_id":     item_id,
            "marker_id":   marker_id,
            "destination": destination,
            "robot_id":    robot_id,
            "status":      TaskStatus.PENDING,
            "created_at":  now_ts(),
            "started_at":  None,
            "completed_at": None,
        })
        print(f"[Task/{task_id}] 생성: '{destination}'  물품={item_id}")
        return task_id

    def start(self, task_id: str):
        self._col.document(task_id).update({
            "status":     TaskStatus.IN_PROGRESS,
            "started_at": now_ts(),
        })
        print(f"[Task/{task_id}] 시작")

    def complete(self, task_id: str):
        self._col.document(task_id).update({
            "status":       TaskStatus.COMPLETED,
            "completed_at": now_ts(),
        })
        print(f"[Task/{task_id}] 완료")

    def fail(self, task_id: str, reason: str = ""):
        self._col.document(task_id).update({
            "status":       TaskStatus.FAILED,
            "completed_at": now_ts(),
            "fail_reason":  reason,
        })
        print(f"[Task/{task_id}] 실패: {reason}")

    def get_next_pending(self, robot_id: str = "m0609") -> dict | None:
        """해당 로봇의 다음 대기 작업을 반환합니다."""
        docs = (self._col
                .where("robot_id", "==", robot_id)
                .where("status",   "==", TaskStatus.PENDING)
                .order_by("created_at")
                .limit(1)
                .stream())
        results = [d.to_dict() for d in docs]
        return results[0] if results else None

    def get_stats(self) -> dict:
        """작업 현황 통계."""
        all_tasks = [d.to_dict() for d in self._col.stream()]
        stats = {s: 0 for s in [TaskStatus.PENDING, TaskStatus.IN_PROGRESS,
                                  TaskStatus.COMPLETED, TaskStatus.FAILED]}
        dest_count: dict[str, int] = {}
        for t in all_tasks:
            stats[t["status"]] = stats.get(t["status"], 0) + 1
            if t["status"] == TaskStatus.COMPLETED:
                dest = t["destination"]
                dest_count[dest] = dest_count.get(dest, 0) + 1
        return {"status_counts": stats, "destination_counts": dest_count}
