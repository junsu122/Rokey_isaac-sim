"""
AMR 섹션 네비게이션 매니저.

동작 흐름:
  1. 물품 마커 인식 → target_section 조회
  2. AMR에 목적 섹션 할당 (navigate_to_section)
  3. AMR이 이동하다가 섹션 마커 인식
  4. confirm_arrival() → 목적 섹션과 일치하면 도착 확인
  5. AMR set_unloading() → 물품 내려놓음
"""

from firebase_admin import firestore
from .firebase_manager import now_ts


class NavigationManager:
    """
    AMR의 섹션 이동 목표와 도착 확인을 관리합니다.

    Firestore 구조 (navigation/ 컬렉션):
      navigation/{amr_id}/
        current_target   : "A-1"          ← 현재 이동 목표 섹션
        target_position  : {x, y}         ← 목표 섹션의 실제 좌표
        assigned_item_id : "ITEM-xxxx"    ← 운반 중인 물품 ID
        status           : "idle" | "navigating" | "arrived" | "unloading"
        confirmed_section: "A-1"          ← 마커로 확인된 섹션
        updated_at       : timestamp
    """
    def __init__(self, db: firestore.Client, section_map: dict):
        """
        section_map: yaml markers에서 section 역할만 뽑은 dict
          {section_id: {position: {x,y,z}, label, ...}}
        """
        self._col = db.collection("navigation")
        self._section_map = section_map   # "A-1" → {x, y, z}

    def navigate_to_section(self, amr_id: str, target_section: str,
                            item_id: str) -> dict | None:
        """
        AMR에 목적 섹션을 할당하고 이동 목표 좌표를 반환합니다.
        Returns: {"x": ..., "y": ...} 또는 None (섹션 없음)
        """
        if target_section not in self._section_map:
            print(f"[Navigation] 섹션 '{target_section}' 이 등록되어 있지 않습니다.")
            return None

        pos = self._section_map[target_section]
        self._col.document(amr_id).set({
            "amr_id":          amr_id,
            "current_target":  target_section,
            "target_position": pos,
            "assigned_item_id": item_id,
            "status":          "navigating",
            "confirmed_section": None,
            "updated_at":      now_ts(),
        })
        print(f"[Navigation] AMR={amr_id}  목표={target_section}"
              f"  좌표=({pos['x']:.2f}, {pos['y']:.2f})")
        return pos

    def confirm_arrival(self, amr_id: str, detected_section_id: str) -> bool:
        """
        AMR이 섹션 마커를 읽었을 때 목표 섹션과 일치하는지 확인합니다.
        Returns: True = 목표 섹션 도착 확인 / False = 다른 섹션
        """
        doc = self._col.document(amr_id).get()
        if not doc.exists:
            return False

        data = doc.to_dict()
        target = data.get("current_target")

        if detected_section_id == target:
            self._col.document(amr_id).update({
                "status":           "arrived",
                "confirmed_section": detected_section_id,
                "updated_at":       now_ts(),
            })
            print(f"[Navigation] AMR={amr_id}  섹션 '{detected_section_id}' 도착 확인!")
            return True
        else:
            print(f"[Navigation] AMR={amr_id}  "
                  f"감지={detected_section_id}  목표={target}  → 계속 이동")
            return False

    def set_unloading_done(self, amr_id: str):
        self._col.document(amr_id).update({
            "status":          "idle",
            "current_target":  None,
            "assigned_item_id": None,
            "updated_at":      now_ts(),
        })
        print(f"[Navigation] AMR={amr_id}  물품 내려놓음 완료 → 대기")

    def get_status(self, amr_id: str) -> dict | None:
        doc = self._col.document(amr_id).get()
        return doc.to_dict() if doc.exists else None
