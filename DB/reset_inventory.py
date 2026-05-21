"""
Firestore의 sections / products / items / tasks / navigation 컬렉션을 초기화합니다.
실행: python3 firebase/reset_inventory.py
"""

import sys
from pathlib import Path

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

from DB.firebase_manager import init_firebase


def delete_collection(db, col_name: str):
    docs = list(db.collection(col_name).stream())
    for doc in docs:
        doc.reference.delete()
    print(f"  [{col_name}] {len(docs)}건 삭제")


def main():
    db = init_firebase()
    print("=== Firestore 초기화 ===")
    for col in ["sections", "products", "items", "tasks", "navigation"]:
        delete_collection(db, col)
    print("완료. setup_inventory.py 를 다시 실행하세요.")


if __name__ == "__main__":
    main()
