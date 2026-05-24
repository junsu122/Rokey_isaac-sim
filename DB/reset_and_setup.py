"""
DB/reset_and_setup.py
======================
기존 Firestore 컬렉션 전체 삭제 후 새 schema로 재등록.

삭제 대상 (구 schema):
    robots/, sections/, products/, items/, tasks/, navigation/

등록 대상 (신 schema):
    sections/{A,B,C} + 서브컬렉션 pods/

실행:
    python3 DB/reset_and_setup.py
"""
import os
import firebase_admin
from firebase_admin import credentials, firestore

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

# 삭제할 기존 컬렉션
OLD_COLLECTIONS = ["robots", "sections", "products", "items", "tasks", "navigation"]

# 새 섹션 설정
SECTIONS = {
    "A": {"package_size": "Big",    "pod_amount": 20},
    "B": {"package_size": "Medium", "pod_amount": 20},
    "C": {"package_size": "Small",  "pod_amount": 20},
}

SECTION_ROBOTS = {
    "A": {"m0609": "M0609_A",  "iw_hub": "iw_hub_01"},
    "B": {"m0609": "M0609_B",  "iw_hub": "iw_hub_02"},
    "C": {"m0609": "M0609_C",  "iw_hub": "iw_hub_03"},
}


def delete_collection(db, col_ref, batch_size=100):
    """컬렉션의 모든 문서를 재귀적으로 삭제."""
    deleted = 0
    while True:
        docs = list(col_ref.limit(batch_size).stream())
        if not docs:
            break
        for doc in docs:
            # 서브컬렉션 먼저 삭제
            for sub in doc.reference.collections():
                delete_collection(db, sub, batch_size)
            doc.reference.delete()
            deleted += 1
    return deleted


def reset(db):
    print("[reset] 기존 컬렉션 삭제 시작")
    for col_name in OLD_COLLECTIONS:
        col_ref = db.collection(col_name)
        count = delete_collection(db, col_ref)
        print(f"[reset]   {col_name}/  → {count}개 문서 삭제")
    print("[reset] 완료\n")


def setup(db):
    print("[setup] 새 schema 등록 시작")
    for section_id, cfg in SECTIONS.items():
        robots = SECTION_ROBOTS[section_id]

        # sections/{section_id} 문서
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
        print(f"[setup]   sections/{section_id} 등록")

        # pods 서브컬렉션 (배치 쓰기)
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
        print(f"[setup]   sections/{section_id}/pods  {cfg['pod_amount']}개 등록")

    print("[setup] 완료")


def main():
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    print("[Firebase] 연결 완료\n")

    reset(db)
    setup(db)

    print("\n[done] Firestore 업데이트 완료.")


if __name__ == "__main__":
    main()
