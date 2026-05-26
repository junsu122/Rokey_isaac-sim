"""
DB/reset_and_setup.py
======================
Firestore 전체 리셋 후 초기값 재등록.

기존 컬렉션을 전부 삭제하고 init_db.py 를 실행한다.

실행:
    python3 DB/reset_and_setup.py
"""
import os
import sys
import firebase_admin
from firebase_admin import credentials, firestore

sys.path.insert(0, os.path.dirname(__file__))
from init_db import init

KEY_PATH = os.path.join(os.path.dirname(__file__), "serviceAccountKey.json")

OLD_COLLECTIONS = ["robots", "sections", "products", "items", "tasks", "navigation"]


def _delete_collection(db, col_ref, batch_size=100):
    deleted = 0
    while True:
        docs = list(col_ref.limit(batch_size).stream())
        if not docs:
            break
        for doc in docs:
            for sub in doc.reference.collections():
                _delete_collection(db, sub, batch_size)
            doc.reference.delete()
            deleted += 1
    return deleted


def main():
    cred = credentials.Certificate(KEY_PATH)
    firebase_admin.initialize_app(cred)
    db = firestore.client()

    print("[reset] 기존 컬렉션 삭제")
    for col_name in OLD_COLLECTIONS:
        count = _delete_collection(db, db.collection(col_name))
        if count:
            print(f"  {col_name}/ → {count}개 삭제")

    print("\n[init] 초기값 등록")
    init(db)
    print("\n완료.")


if __name__ == "__main__":
    main()
