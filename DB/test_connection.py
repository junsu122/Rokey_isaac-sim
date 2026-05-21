import sys as _sys
from pathlib import Path as _Path
_root = _Path(__file__).resolve().parent
while not (_root / "DB").exists() and _root.parent != _root:
    _root = _root.parent
if str(_root) not in _sys.path:
    _sys.path.insert(0, str(_root))
del _root

"""
Firebase 연결 테스트 스크립트.
실행: python3 firebase/test_connection.py
"""

import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


def test():
    print("Firebase 연결 테스트 시작...\n")

    # 1. 초기화
    try:
        from DB.firebase_manager import init_firebase
        db = init_firebase()
        print("  [OK] Firebase 초기화 성공")
    except FileNotFoundError as e:
        print(f"  [FAIL] 서비스 계정 키 없음\n  → {e}")
        return
    except Exception as e:
        print(f"  [FAIL] 초기화 실패: {e}")
        return

    # 2. 쓰기 테스트
    try:
        db.collection("test").document("ping").set({"status": "ok"})
        print("  [OK] Firestore 쓰기 성공")
    except Exception as e:
        print(f"  [FAIL] 쓰기 실패: {e}")
        return

    # 3. 읽기 테스트
    try:
        doc = db.collection("test").document("ping").get()
        assert doc.to_dict()["status"] == "ok"
        print("  [OK] Firestore 읽기 성공")
    except Exception as e:
        print(f"  [FAIL] 읽기 실패: {e}")
        return

    # 4. 테스트 문서 삭제
    db.collection("test").document("ping").delete()

    # 5. 로봇 Fleet 초기화
    try:
        from firebase import RobotFleet, ItemTracker, TaskManager
        fleet       = RobotFleet(db)
        item_tracker = ItemTracker(db)
        task_mgr    = TaskManager(db)
        print("  [OK] RobotFleet / ItemTracker / TaskManager 초기화 성공")
    except Exception as e:
        print(f"  [FAIL] 매니저 초기화 실패: {e}")
        return

    print("\n연결 테스트 완료. Firebase 준비됩니다.")
    print("다음 명령으로 실행하세요:")
    print("  python3 isaac_aruco_main.py --webcam 0")


if __name__ == "__main__":
    test()
