"""
items/ 컬렉션 관리.
ArUco 검출 → 물품 등록 → 상태 변화 이력을 Firestore에 저장합니다.
"""

from firebase_admin import firestore
from .firebase_manager import now_ts
import uuid

# 물품 상태값 상수
class ItemStatus:
    WAITING   = "waiting"    # 컨베이어 대기 중
    DETECTED  = "detected"   # 카메라가 마커 인식
    PICKED    = "picked"     # 로봇이 집음
    DELIVERED = "delivered"  # 목적지 빈에 배달 완료


class ItemTracker:
    def __init__(self, db: firestore.Client):
        self._col = db.collection("items")

    def register(self, marker_id: int, label: str, category: str,
                 position_xyz: tuple[float, float, float] | None = None) -> str:
        """
        ArUco 검출 시 물품을 Firestore에 등록합니다.
        Returns: 생성된 item_id
        """
        item_id = f"item_{uuid.uuid4().hex[:8]}"
        self._col.document(item_id).set({
            "item_id":       item_id,
            "marker_id":     marker_id,
            "label":         label,
            "category":      category,
            "status":        ItemStatus.DETECTED,
            "position_xyz":  list(position_xyz) if position_xyz else None,
            "assigned_robot": None,
            "detected_at":   now_ts(),
            "picked_at":     None,
            "delivered_at":  None,
        })
        print(f"[Item/{item_id}] 등록: ID={marker_id} '{label}'  위치={position_xyz}")
        return item_id

    def set_picked(self, item_id: str, robot_id: str):
        self._col.document(item_id).update({
            "status":         ItemStatus.PICKED,
            "assigned_robot": robot_id,
            "picked_at":      now_ts(),
        })
        print(f"[Item/{item_id}] 집기 완료 → 로봇: {robot_id}")

    def set_delivered(self, item_id: str):
        self._col.document(item_id).update({
            "status":       ItemStatus.DELIVERED,
            "delivered_at": now_ts(),
        })
        print(f"[Item/{item_id}] 배달 완료")

    def get_pending(self) -> list[dict]:
        """아직 처리되지 않은(detected 상태) 물품 목록 반환."""
        docs = (self._col
                .where("status", "==", ItemStatus.DETECTED)
                .order_by("detected_at")
                .stream())
        return [d.to_dict() for d in docs]

    def get_history(self, label: str | None = None, limit: int = 50) -> list[dict]:
        """배달 완료 이력 조회. label 지정 시 해당 목적지만 필터."""
        q = self._col.where("status", "==", ItemStatus.DELIVERED)
        if label:
            q = q.where("label", "==", label)
        docs = q.order_by("delivered_at", direction=firestore.Query.DESCENDING).limit(limit).stream()
        return [d.to_dict() for d in docs]

    def get(self, item_id: str) -> dict | None:
        doc = self._col.document(item_id).get()
        return doc.to_dict() if doc.exists else None
