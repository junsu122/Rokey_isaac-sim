"""
DB/init_db.py
=============
Firestore 초기값 등록.

sections/{A|B|C} 문서 + pods 서브컬렉션을 생성하고
pod 좌표/상태를 POD_LAYOUT 기준으로 설정한다.

실행:
    python3 DB/init_db.py
"""
import os
import sys
import firebase_admin
from firebase_admin import credentials, firestore

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "main_isaac"))
from robot_config import ROBOT_REGISTRY

# ── 섹션 기본 설정 ────────────────────────────────────────────────────
SECTION_CONFIG = {
    "A": {"package_size": "Big",    "m0609": "M0609_A", "iw_hub": "iw_hub_01"},
    "B": {"package_size": "Medium", "m0609": "M0609_B", "iw_hub": "iw_hub_02"},
    "C": {"package_size": "Small",  "m0609": "M0609_C", "iw_hub": "iw_hub_03"},
}

# ── Pod 그리드 레이아웃 ───────────────────────────────────────────────
# 설계 문서: main_isaac/POD_LAYOUT.md
POD_X_COLS = [-2.25, -0.75, 0.75, 2.25]

SECTOR_POD_LAYOUT = {
    "A": {"y_rows": [7.3,  8.7,  10.1, 11.5, 12.9], "full_count": 9},
    "B": {"y_rows": [-2.8, -1.4,  0.0,  1.4,  2.8], "full_count": 9},
    "C": {"y_rows": [-7.3, -8.7, -10.1, -11.5, -12.9], "full_count": 9},
}


def _gen_pods(section_id: str) -> list[dict]:
    layout     = SECTOR_POD_LAYOUT[section_id]
    y_rows     = layout["y_rows"]
    full_count = layout["full_count"]
    total      = len(y_rows) * len(POD_X_COLS)
    empty_count = total - full_count

    pods = []
    idx  = 0
    for y in y_rows:
        for x in POD_X_COLS:
            pod_id = f"pod_{idx + 1:02d}"
            state  = "empty" if idx < empty_count else "full"
            pods.append({
                "pod_id":   pod_id,
                "state":    state,
                "location": {"x": float(x), "y": float(y)},
            })
            idx += 1
    return pods


def _get_iwhub_spawn(iw_hub_name: str) -> dict | None:
    for robot in ROBOT_REGISTRY:
        if robot.get("type") == "iw_hub" and robot.get("name") == iw_hub_name:
            x, y, _ = robot["spawn_xyz"]
            return {"x": float(x), "y": float(y)}
    return None


def init(db):
    for section_id, cfg in SECTION_CONFIG.items():
        section_ref = db.collection("sections").document(section_id)

        iwhub_loc = _get_iwhub_spawn(cfg["iw_hub"]) or {"x": 0.0, "y": 0.0}

        section_ref.set({
            "section_id":   section_id,
            "package_size": cfg["package_size"],
            "pod_amount":   20,
            "robots": {
                "m0609":  {"robot_name": cfg["m0609"],  "state": "stop"},
                "iw_hub": {"robot_name": cfg["iw_hub"], "state": "wait", "location": iwhub_loc},
            },
            "last_updated": firestore.SERVER_TIMESTAMP,
        })

        pods     = _gen_pods(section_id)
        pods_ref = section_ref.collection("pods")
        batch    = db.batch()
        for pod in pods:
            batch.set(pods_ref.document(pod["pod_id"]), pod)
        batch.commit()

        empty = sum(1 for p in pods if p["state"] == "empty")
        full  = sum(1 for p in pods if p["state"] == "full")
        print(f"[init] Section {section_id}: pods {len(pods)}개 (empty={empty}, full={full}), iw_hub={iwhub_loc}")


if __name__ == "__main__":
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    init(db)
    print("[init] 완료.")
