"""
예시 데이터를 Firestore에 저장합니다.

시나리오:
  1. Apple Watch  — AMR이 A-1 구획으로 이동 중  (in_transit)
  2. AirPods      — 방금 카메라에 인식됨         (detected / pending)
  3. Galaxy Tab   — 배송 완료                    (delivered)

실행: python3 firebase/seed_example_data.py
"""

import sys
import uuid
from datetime import datetime, timezone, timedelta
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

from DB.firebase_manager import init_firebase, now_ts
from DB.robot_status import RobotFleet
from DB.inventory import ItemTracker, ProductManager
from DB.task_manager import TaskManager, TaskStatus
from DB.navigation import NavigationManager


def ts_ago(seconds: int):
    """현재 시각에서 seconds 초 전 timestamp 반환."""
    return datetime.now(timezone.utc) - timedelta(seconds=seconds)


def main():
    db = init_firebase()

    prod_mgr = ProductManager(db)
    item_col  = db.collection("items")
    task_col  = db.collection("tasks")
    nav_col   = db.collection("navigation")

    fleet = RobotFleet(db)

    print("=== 예시 데이터 생성 ===\n")

    # ──────────────────────────────────────────────────────────
    # 시나리오 1: Apple Watch — AMR이 A-1 구획으로 이동 중
    # ──────────────────────────────────────────────────────────
    print("[1] Apple Watch — AMR 이동 중 (in_transit)")

    prod1 = prod_mgr.get_by_marker(0)   # Apple Watch
    item1_id = f"ITEM-{uuid.uuid4().hex[:8].upper()}"

    item_col.document(item1_id).set({
        "item_id":        item1_id,
        "product_id":     prod1["product_id"],
        "name":           "Apple Watch",
        "marker_id":      0,
        "section":        "A-1",
        "destination":    "Gangnam",
        "status":         "in_transit",
        "position_xyz":   [0.05, 0.0, 0.42],
        "assigned_robot": "amr_001",
        "current_task":   None,
        "registered_at":  ts_ago(90),
        "detected_at":    ts_ago(90),
        "delivered_at":   None,
    })

    task1_pick = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task1_pick).set({
        "task_id":      task1_pick,
        "item_id":      item1_id,
        "marker_id":    0,
        "destination":  "A-1",
        "robot_id":     "m0609",
        "status":       TaskStatus.COMPLETED,
        "created_at":   ts_ago(90),
        "started_at":   ts_ago(80),
        "completed_at": ts_ago(60),
    })

    task1_delivery = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task1_delivery).set({
        "task_id":      task1_delivery,
        "item_id":      item1_id,
        "marker_id":    0,
        "destination":  "Gangnam",
        "robot_id":     "amr_001",
        "status":       TaskStatus.IN_PROGRESS,
        "created_at":   ts_ago(90),
        "started_at":   ts_ago(60),
        "completed_at": None,
    })

    # AMR 상태: 이동 중
    fleet.amr.set_loading(task_id=task1_delivery)
    fleet.amr.set_transporting()

    # Navigation: A-1으로 이동 중
    nav_col.document("amr_001").set({
        "amr_id":           "amr_001",
        "current_target":   "A-1",
        "target_position":  {"x": -0.4, "y": 0.3, "z": 0.0},
        "assigned_item_id": item1_id,
        "status":           "navigating",
        "confirmed_section": None,
        "updated_at":       now_ts(),
    })

    # ARM 상태: 픽업 완료 후 대기
    fleet.arm.set_detected_item(
        marker_id=0, label="Apple Watch", category="item",
        position_xyz=(0.05, 0.0, 0.42), item_id=item1_id,
    )
    fleet.arm.set_idle()

    print(f"  item_id={item1_id}  pick={task1_pick}  delivery={task1_delivery}")

    # ──────────────────────────────────────────────────────────
    # 시나리오 2: AirPods — 방금 카메라에 인식됨 (대기 중)
    # ──────────────────────────────────────────────────────────
    print("\n[2] AirPods — 방금 인식됨 (detected / pending)")

    prod3 = prod_mgr.get_by_marker(3)   # AirPods
    item2_id = f"ITEM-{uuid.uuid4().hex[:8].upper()}"

    item_col.document(item2_id).set({
        "item_id":        item2_id,
        "product_id":     prod3["product_id"],
        "name":           "AirPods",
        "marker_id":      3,
        "section":        "A-1",
        "destination":    "Gangnam",
        "status":         "detected",
        "position_xyz":   [0.12, 0.03, 0.38],
        "assigned_robot": None,
        "current_task":   None,
        "registered_at":  ts_ago(10),
        "detected_at":    ts_ago(10),
        "delivered_at":   None,
    })

    task2_pick = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task2_pick).set({
        "task_id":      task2_pick,
        "item_id":      item2_id,
        "marker_id":    3,
        "destination":  "A-1",
        "robot_id":     "m0609",
        "status":       TaskStatus.PENDING,
        "created_at":   ts_ago(10),
        "started_at":   None,
        "completed_at": None,
    })

    task2_delivery = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task2_delivery).set({
        "task_id":      task2_delivery,
        "item_id":      item2_id,
        "marker_id":    3,
        "destination":  "Gangnam",
        "robot_id":     "amr_001",
        "status":       TaskStatus.PENDING,
        "created_at":   ts_ago(10),
        "started_at":   None,
        "completed_at": None,
    })

    print(f"  item_id={item2_id}  pick={task2_pick}  delivery={task2_delivery}")

    # ──────────────────────────────────────────────────────────
    # 시나리오 3: Galaxy Tab — 배송 완료
    # ──────────────────────────────────────────────────────────
    print("\n[3] Galaxy Tab — 배송 완료 (delivered)")

    prod2 = prod_mgr.get_by_marker(1)   # Galaxy Tab
    item3_id = f"ITEM-{uuid.uuid4().hex[:8].upper()}"

    item_col.document(item3_id).set({
        "item_id":        item3_id,
        "product_id":     prod2["product_id"],
        "name":           "Galaxy Tab",
        "marker_id":      1,
        "section":        "A-2",
        "destination":    "Seocho",
        "status":         "delivered",
        "position_xyz":   [-0.02, 0.1, 0.45],
        "assigned_robot": "amr_001",
        "current_task":   None,
        "registered_at":  ts_ago(300),
        "detected_at":    ts_ago(300),
        "delivered_at":   ts_ago(60),
    })

    task3_pick = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task3_pick).set({
        "task_id":      task3_pick,
        "item_id":      item3_id,
        "marker_id":    1,
        "destination":  "A-2",
        "robot_id":     "m0609",
        "status":       TaskStatus.COMPLETED,
        "created_at":   ts_ago(300),
        "started_at":   ts_ago(290),
        "completed_at": ts_ago(250),
    })

    task3_delivery = f"task_{uuid.uuid4().hex[:8]}"
    task_col.document(task3_delivery).set({
        "task_id":      task3_delivery,
        "item_id":      item3_id,
        "marker_id":    1,
        "destination":  "Seocho",
        "robot_id":     "amr_001",
        "status":       TaskStatus.COMPLETED,
        "created_at":   ts_ago(300),
        "started_at":   ts_ago(250),
        "completed_at": ts_ago(60),
    })

    print(f"  item_id={item3_id}  pick={task3_pick}  delivery={task3_delivery}")

    # ──────────────────────────────────────────────────────────
    # 드론 상태 설정
    # ──────────────────────────────────────────────────────────
    fleet.drone.update_pose(x=0.0, y=0.0, z=0.0)
    fleet.drone.update_battery(87.5)

    # ──────────────────────────────────────────────────────────
    # 결과 출력
    # ──────────────────────────────────────────────────────────
    print("\n=== 저장 완료 ===")
    print("\n[items]")
    for doc in db.collection("items").stream():
        d = doc.to_dict()
        print(f"  {d['item_id']}  '{d['name']:15}'  status={d['status']:12}"
              f"  dest={d['destination']}")

    print("\n[tasks]")
    for doc in db.collection("tasks").order_by("created_at").stream():
        d = doc.to_dict()
        print(f"  {d['task_id']}  robot={d['robot_id']:8}"
              f"  dest={d['destination']:15}  status={d['status']}")

    print("\n[navigation]")
    for doc in db.collection("navigation").stream():
        d = doc.to_dict()
        print(f"  amr={d['amr_id']}  target={d['current_target']}  status={d['status']}")

    print("\n[robots]")
    for doc in db.collection("robots").stream():
        d = doc.to_dict()
        cargo = d.get("cargo_status", d.get("status", "-"))
        print(f"  {d['robot_id']:12}  battery={d['battery']}%  상태={cargo}")


if __name__ == "__main__":
    main()
