"""
물품 데이터베이스 관리.

컬렉션 구조:
  sections/   ← 구획 마스터  (A-1, B-2 같은 창고 위치 정보)
  products/   ← 물품 마스터  (어떤 물건이 어느 구획에 있어야 하는지)
  items/      ← 배송 추적    (실시간 상태 — 어디 있고 어디로 가는지)
"""

from firebase_admin import firestore
from .firebase_manager import now_ts
import uuid


# ── 배송 상태 상수 ────────────────────────────────────────────
class DeliveryStatus:
    REGISTERED  = "registered"   # 시스템에 등록됨
    WAITING     = "waiting"      # 구획에서 픽업 대기
    DETECTED    = "detected"     # 카메라가 마커 인식
    IN_TRANSIT  = "in_transit"   # 로봇이 운반 중
    DELIVERED   = "delivered"    # 배송 완료
    RETURNED    = "returned"     # 반품 처리


# ── 구획 관리 ─────────────────────────────────────────────────
class SectionManager:
    """
    sections/ 컬렉션 관리.

    구획(Section)이란?
      창고·공장 내 물품이 보관되는 위치 단위.
      예: A-1, B-3, C-2 (열-번호 형식)

    Firestore 필드:
      section_id  : "A-1"
      description : "A열 1번 구획"
      position    : {x, y, z}   ← 실제 공간 좌표 (로봇 이동에 사용)
      capacity    : 10           ← 최대 수용 물품 수
      current_count: 3          ← 현재 보관 물품 수
      is_active   : true
    """
    def __init__(self, db: firestore.Client):
        self._col = db.collection("sections")

    def register(self, section_id: str, description: str,
                 position: dict, capacity: int = 10) -> str:
        """구획 등록. position: {"x": 1.0, "y": 2.0, "z": 0.0}"""
        self._col.document(section_id).set({
            "section_id":    section_id,
            "description":   description,
            "position":      position,
            "capacity":      capacity,
            "current_count": 0,
            "is_active":     True,
            "created_at":    now_ts(),
        })
        print(f"[Section] 등록: {section_id}  {description}  위치={position}")
        return section_id

    def increment(self, section_id: str):
        """물품 입고 시 수량 증가."""
        self._col.document(section_id).update({
            "current_count": firestore.Increment(1)
        })

    def decrement(self, section_id: str):
        """물품 출고 시 수량 감소."""
        self._col.document(section_id).update({
            "current_count": firestore.Increment(-1)
        })

    def get(self, section_id: str) -> dict | None:
        doc = self._col.document(section_id).get()
        return doc.to_dict() if doc.exists else None

    def get_all(self) -> list[dict]:
        return [d.to_dict() for d in self._col.stream()]


# ── 물품 마스터 관리 ──────────────────────────────────────────
class ProductManager:
    """
    products/ 컬렉션 관리.

    물품 마스터란?
      실제로 존재하는 물품의 기본 정보.
      ArUco 마커 ID와 연결되어 로봇이 물품을 식별하는 기준.

    Firestore 필드:
      product_id  : "PROD-001"
      name        : "상품명"
      marker_id   : 0            ← ArUco 마커 ID
      section     : "A-1"        ← 있어야 할 구획
      destination : "강남"       ← 배송지
      weight      : 1.5          ← 무게 (kg)
      size        : {w, d, h}    ← 크기 (m)
      description : "상품 설명"
      is_active   : true
      created_at  : timestamp
    """
    def __init__(self, db: firestore.Client):
        self._col = db.collection("products")

    def register(self, name: str, marker_id: int,
                 section: str, destination: str,
                 weight: float = 0.0,
                 size: dict | None = None,
                 description: str = "") -> str:
        """
        물품 마스터 등록.

        Args:
            name        : 물품 이름
            marker_id   : 부착된 ArUco 마커 ID
            section     : 보관 구획 (예: "A-1")
            destination : 배송지 (예: "강남")
        """
        product_id = f"PROD-{uuid.uuid4().hex[:6].upper()}"
        self._col.document(product_id).set({
            "product_id":  product_id,
            "name":        name,
            "marker_id":   marker_id,
            "section":     section,
            "destination": destination,
            "weight":      weight,
            "size":        size or {"w": 0.08, "d": 0.08, "h": 0.08},
            "description": description,
            "is_active":   True,
            "created_at":  now_ts(),
        })
        print(f"[Product] 등록: {product_id}  '{name}'  "
              f"마커={marker_id}  구획={section}  배송지={destination}")
        return product_id

    def get_by_marker(self, marker_id: int) -> dict | None:
        """마커 ID로 물품 조회."""
        docs = (self._col
                .where("marker_id", "==", marker_id)
                .where("is_active", "==", True)
                .limit(1)
                .stream())
        results = [d.to_dict() for d in docs]
        return results[0] if results else None

    def get(self, product_id: str) -> dict | None:
        doc = self._col.document(product_id).get()
        return doc.to_dict() if doc.exists else None

    def get_all(self) -> list[dict]:
        return [d.to_dict() for d in
                self._col.where("is_active", "==", True).stream()]

    def update_section(self, product_id: str, section: str):
        """물품 보관 구획 변경."""
        self._col.document(product_id).update({
            "section":    section,
            "updated_at": now_ts(),
        })
        print(f"[Product/{product_id}] 구획 변경: {section}")

    def update_destination(self, product_id: str, destination: str):
        """배송지 변경."""
        self._col.document(product_id).update({
            "destination": destination,
            "updated_at":  now_ts(),
        })
        print(f"[Product/{product_id}] 배송지 변경: {destination}")


# ── 배송 추적 ─────────────────────────────────────────────────
class ItemTracker:
    """
    items/ 컬렉션 관리.

    배송 인스턴스 — 물품이 구획에서 배송지까지 이동하는 과정을 추적합니다.

    Firestore 필드:
      item_id       : "ITEM-xxxxxxxx"
      product_id    : "PROD-001"     ← products/ 참조
      name          : "상품명"
      marker_id     : 0
      section       : "A-1"          ← 출발 구획
      destination   : "강남"         ← 배송지
      status        : "waiting" | "detected" | "in_transit" | "delivered" | "returned"
      position_xyz  : [x, y, z]      ← 마지막 감지 위치
      assigned_robot: "m0609"
      current_task  : "task_xxx"
      registered_at : timestamp
      detected_at   : timestamp
      delivered_at  : timestamp
    """
    def __init__(self, db: firestore.Client):
        self._col     = db.collection("items")
        self._products = ProductManager(db)
        self._sections = SectionManager(db)

    def register(self, marker_id: int, label: str, category: str,
                 position_xyz: tuple | None = None,
                 item_id: str | None = None) -> str:
        """
        ArUco 검출 시 배송 인스턴스 생성.
        products/ 에서 해당 마커의 물품 정보(구획·배송지)를 자동으로 조회합니다.
        """
        # 물품 마스터에서 구획·배송지 조회
        product = self._products.get_by_marker(marker_id)
        section     = product["section"]     if product else "미지정"
        destination = product["destination"] if product else label
        name        = product["name"]        if product else f"물품_{marker_id}"
        product_id  = product["product_id"]  if product else None

        iid = item_id or f"ITEM-{uuid.uuid4().hex[:8].upper()}"
        self._col.document(iid).set({
            "item_id":       iid,
            "product_id":    product_id,
            "name":          name,
            "marker_id":     marker_id,
            "section":       section,
            "destination":   destination,
            "status":        DeliveryStatus.DETECTED,
            "position_xyz":  list(position_xyz) if position_xyz else None,
            "assigned_robot": None,
            "current_task":  None,
            "registered_at": now_ts(),
            "detected_at":   now_ts(),
            "delivered_at":  None,
        })

        # 구획 수량 감소 (출고됨)
        if product:
            self._sections.decrement(section)

        print(f"[Item/{iid}] 등록: '{name}'  구획={section}  배송지={destination}")
        return iid

    def set_in_transit(self, item_id: str, robot_id: str, task_id: str):
        self._col.document(item_id).update({
            "status":         DeliveryStatus.IN_TRANSIT,
            "assigned_robot": robot_id,
            "current_task":   task_id,
        })
        print(f"[Item/{item_id}] 운반 중  로봇={robot_id}")

    def set_delivered(self, item_id: str):
        self._col.document(item_id).update({
            "status":       DeliveryStatus.DELIVERED,
            "delivered_at": now_ts(),
            "current_task": None,
        })
        print(f"[Item/{item_id}] 배송 완료")

    def set_returned(self, item_id: str):
        self._col.document(item_id).update({
            "status":       DeliveryStatus.RETURNED,
            "delivered_at": now_ts(),
        })
        print(f"[Item/{item_id}] 반품 처리")

    def get_pending(self) -> list[dict]:
        """픽업 대기 중인 물품 목록."""
        docs = (self._col
                .where("status", "==", DeliveryStatus.DETECTED)
                .order_by("detected_at")
                .stream())
        return [d.to_dict() for d in docs]

    def get_by_section(self, section: str) -> list[dict]:
        """특정 구획의 물품 목록."""
        docs = self._col.where("section", "==", section).stream()
        return [d.to_dict() for d in docs]

    def get_by_destination(self, destination: str) -> list[dict]:
        """특정 배송지로 가는 물품 목록."""
        docs = self._col.where("destination", "==", destination).stream()
        return [d.to_dict() for d in docs]

    def get_history(self, limit: int = 50) -> list[dict]:
        """배송 완료 이력 (최신순)."""
        docs = (self._col
                .where("status", "==", DeliveryStatus.DELIVERED)
                .order_by("delivered_at", direction=firestore.Query.DESCENDING)
                .limit(limit)
                .stream())
        return [d.to_dict() for d in docs]

    def get(self, item_id: str) -> dict | None:
        doc = self._col.document(item_id).get()
        return doc.to_dict() if doc.exists else None
