"""
DB/setup_schema.py
==================
Firestore에 새 schema 초기 데이터를 등록한다.
sections A / B / C 문서 + 각 섹션의 pods 서브컬렉션 생성.

실행:
    python3 DB/setup_schema.py
"""
import os
import firebase_admin
from firebase_admin import credentials, firestore

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

# ── 섹션 초기 설정 ────────────────────────────────────────────────────
SECTIONS = {
    "A": {"package_size": "Big",    "pod_amount": 20},
    "B": {"package_size": "Medium", "pod_amount": 20},
    "C": {"package_size": "Small",  "pod_amount": 20},
}

# 섹션별 로봇 이름 (robot_config.py 기준)
SECTION_ROBOTS = {
    "A": {"m0609": "M0609_A",  "iw_hub": "iw_hub_01"},
    "B": {"m0609": "M0609_B",  "iw_hub": "iw_hub_02"},
    "C": {"m0609": "M0609_C",  "iw_hub": "iw_hub_03"},
}


def setup():
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()

    for section_id, cfg in SECTIONS.items():
        robots = SECTION_ROBOTS[section_id]

        # ── sections/{section_id} 문서 ────────────────────────────
        section_ref = db.collection("sections").document(section_id)
        section_ref.set({
            "section_id":   section_id,
            "package_size": cfg["package_size"],
            "pod_amount":   cfg["pod_amount"],
            "robots": {
                "m0609": {
                    "robot_name": robots["m0609"],
                    "state": "stop",
                },
                "iw_hub": {
                    "robot_name": robots["iw_hub"],
                    "state": "stop",
                    "location": {"x": 0.0, "y": 0.0},
                },
            },
            "last_updated": firestore.SERVER_TIMESTAMP,
        })
        print(f"[setup] sections/{section_id} 등록 완료")

        # ── sections/{section_id}/pods/{pod_id} 서브컬렉션 ───────
        pods_ref = section_ref.collection("pods")
        batch = db.batch()
        for i in range(1, cfg["pod_amount"] + 1):
            pod_id = f"pod_{i:02d}"
            batch.set(pods_ref.document(pod_id), {
                "pod_id":   pod_id,
                "state":    "empty",
                "location": {"x": 0.0, "y": 0.0},
            })
        batch.commit()
        print(f"[setup] sections/{section_id}/pods  {cfg['pod_amount']}개 등록 완료")

    print("\n[setup] 완료.")


if __name__ == "__main__":
    setup()
