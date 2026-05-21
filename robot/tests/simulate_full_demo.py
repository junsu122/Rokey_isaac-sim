"""
풍부한 가짜 데이터로 UI 변화를 실시간으로 확인합니다.

1단계: 모든 구획에 초기 재고를 시딩 (A-1 ~ B-2)
2단계: 세 로봇이 동시에 움직이며 아이템 상태 변화

실행:
  cd /path/to/robot
  python3 tests/simulate_full_demo.py
"""

import sys
import time
import uuid
import math
from pathlib import Path
from datetime import datetime, timezone, timedelta

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
from DB.robot_status import RobotFleet, DroneState

# ── 섹션 좌표 (창고 맵 기준) ─────────────────────────────────────
SECTION_POS = {
    "A-1": (-0.4,  0.3),
    "A-2": ( 0.0,  0.3),
    "A-3": ( 0.4,  0.3),
    "B-1": (-0.4, -0.3),
    "B-2": ( 0.0, -0.3),
}
DEST_POS = {
    "Gangnam":      (1.5,  0.5),
    "Seocho":       (1.5,  0.0),
    "Guro Digital": (1.5, -0.5),
}

# ── 상품 정보 ────────────────────────────────────────────────────
PRODUCTS = {
    "Apple Watch": {"marker_id": 0, "dest": "Gangnam"},
    "Galaxy Tab":  {"marker_id": 1, "dest": "Seocho"},
    "MacBook Pro": {"marker_id": 2, "dest": "Guro Digital"},
    "AirPods":     {"marker_id": 3, "dest": "Gangnam"},
    "Kindle":      {"marker_id": 4, "dest": "Seocho"},
}


def ts_ago(seconds: int):
    return datetime.now(timezone.utc) - timedelta(seconds=seconds)


def make_item(db, name, section, status="waiting",
              dest=None, robot=None, detected_sec=None, delivered_sec=None):
    """아이템 하나를 Firestore items/ 에 추가합니다."""
    prod = PRODUCTS[name]
    iid = f"ITEM-{uuid.uuid4().hex[:8].upper()}"
    db.collection("items").document(iid).set({
        "item_id":        iid,
        "product_id":     f"prod_{prod['marker_id']:03d}",
        "name":           name,
        "marker_id":      prod["marker_id"],
        "section":        section,
        "destination":    dest or prod["dest"],
        "status":         status,
        "position_xyz":   [0.0, 0.0, 0.4],
        "assigned_robot": robot,
        "current_task":   None,
        "registered_at":  ts_ago(detected_sec or 0),
        "detected_at":    ts_ago(detected_sec or 0) if detected_sec else None,
        "delivered_at":   ts_ago(delivered_sec) if delivered_sec else None,
    })
    return iid


def clear_items(db):
    """기존 items/ 전부 삭제."""
    for doc in db.collection("items").stream():
        doc.reference.delete()


def lerp(a, b, t):
    return a + (b - a) * t


def path_between(p1, p2, steps=20):
    """두 점 사이를 steps로 보간한 (x, y) 리스트."""
    return [(lerp(p1[0], p2[0], s / steps),
             lerp(p1[1], p2[1], s / steps)) for s in range(steps + 1)]


def yaw_toward(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx))


def step(dt=0.25):
    time.sleep(dt)


# ── 메인 ─────────────────────────────────────────────────────────

def main():
    print("Firebase 연결 중...")
    db = init_firebase()
    fleet = RobotFleet(db)
    print("연결 완료\n")

    # ══════════════════════════════════════════════════════════════
    # 1단계: 초기 재고 시딩
    # ══════════════════════════════════════════════════════════════
    print("=" * 55)
    print(" 1단계: 초기 재고 데이터 시딩")
    print("=" * 55)
    clear_items(db)

    # ── 선반에 재고 (waiting/detected 혼합) ─────────────────────
    # A-1: Apple Watch 3개, AirPods 2개
    for _ in range(3):
        make_item(db, "Apple Watch", "A-1", "waiting")
    for _ in range(2):
        make_item(db, "AirPods", "A-1", "waiting")

    # A-2: Galaxy Tab 2개, Kindle 1개
    for _ in range(2):
        make_item(db, "Galaxy Tab", "A-2", "waiting")
    make_item(db, "Kindle", "A-2", "waiting")

    # A-3: MacBook Pro 2개
    for _ in range(2):
        make_item(db, "MacBook Pro", "A-3", "waiting")

    # B-1: AirPods 1개 (방금 인식됨)
    make_item(db, "AirPods", "B-1", "detected", detected_sec=15)

    # B-2: Kindle 2개, Galaxy Tab 1개
    for _ in range(2):
        make_item(db, "Kindle", "B-2", "waiting")
    make_item(db, "Galaxy Tab", "B-2", "waiting")

    # ── 이미 배송 완료된 이력 ───────────────────────────────────
    make_item(db, "Apple Watch", "A-1", "delivered",
              robot="amr_001", detected_sec=600, delivered_sec=300)
    make_item(db, "MacBook Pro", "A-3", "delivered",
              robot="amr_001", detected_sec=900, delivered_sec=500)
    make_item(db, "Galaxy Tab",  "A-2", "delivered",
              robot="drone_001", detected_sec=450, delivered_sec=180)
    make_item(db, "Kindle",      "B-2", "delivered",
              robot="amr_001", detected_sec=1200, delivered_sec=700)
    make_item(db, "AirPods",     "B-1", "delivered",
              robot="amr_001", detected_sec=1500, delivered_sec=900)

    # ── 로봇 초기 상태 ───────────────────────────────────────────
    fleet.amr.update_pose(0.0, 0.0, 0.0)
    fleet.amr.update_battery(92.0)
    fleet.amr.set_empty()

    fleet.drone.update_pose(0.0, 0.0, 0.0)
    fleet.drone.update_battery(85.0)

    fleet.arm.set_idle()
    fleet.arm.update_battery(99.0)

    print("\n초기 재고 시딩 완료!")
    print("브라우저에서 http://localhost:5173 을 열어 재고 현황을 확인하세요.")
    print("\n5초 후 2단계(실시간 시뮬레이션) 시작...\n")
    time.sleep(5)

    # ══════════════════════════════════════════════════════════════
    # 2단계: 실시간 시뮬레이션
    # ══════════════════════════════════════════════════════════════
    print("=" * 55)
    print(" 2단계: 실시간 로봇 동작 시뮬레이션")
    print("=" * 55)
    print("  AMR  : A-1 → 강남 배송 → B-2 → 서초 배송 → 복귀")
    print("  Drone: 이륙 → A-3 호버링 → 구로 배송 → 착륙")
    print("  Arm  : AirPods 픽업 → 배치 → Galaxy Tab 픽업 → 배치")
    print()

    amr_bat  = 92.0
    drone_bat = 85.0
    origin = (0.0, 0.0)

    # ── 씬 1: AMR이 A-1으로 이동 (Apple Watch 픽업 준비) ─────────
    print("[씬 1] AMR → A-1 이동 중 (Apple Watch 픽업 예정)")
    a1_pos = SECTION_POS["A-1"]
    for x, y in path_between(origin, a1_pos, steps=18):
        yaw = yaw_toward(origin, a1_pos)
        fleet.amr.update_pose(x, y, yaw)
        amr_bat -= 0.05
        fleet.amr.update_battery(round(amr_bat, 1))
        step()

    # A-1 도착: Apple Watch in_transit으로 변경
    aw_item = make_item(db, "Apple Watch", "A-1", "in_transit",
                        robot="amr_001", detected_sec=5)
    fleet.amr.set_loading(aw_item)
    print(f"  → Apple Watch 적재 시작 (item_id={aw_item})")
    step(1.5)

    fleet.amr.set_transporting()
    step(0.5)

    # ── 씬 2: 드론 이륙 + 암 픽업 시작 ──────────────────────────
    print("[씬 2] Drone 이륙 / Arm AirPods 픽업 시작")

    # 암: AirPods 픽업
    airpods_item = make_item(db, "AirPods", "B-1", "in_transit",
                             robot="m0609", detected_sec=10)
    fleet.arm.set_detected_item(3, "AirPods", "item", (0.08, 0.0, 0.40), airpods_item)
    fleet.arm.set_picking(airpods_item)

    # 드론: 이륙
    fleet.drone._ref.update({
        "charge_status": DroneState.TAKING_OFF,
        "cargo_status":  "empty",
        "last_updated":  now_ts(),
    })

    for alt in [0.0, 0.3, 0.7, 1.2, 1.8, 2.0]:
        fleet.drone.update_pose(0.0, 0.0, alt)
        drone_bat -= 0.08
        fleet.drone.update_battery(round(drone_bat, 1))
        step(0.4)

    fleet.drone._ref.update({
        "charge_status": DroneState.FLYING,
        "last_updated":  now_ts(),
    })

    # ── 씬 3: AMR A-1 → 강남 이동 / Drone → A-3 비행 ─────────────
    print("[씬 3] AMR → 강남 / Drone → A-3")
    gangnam = DEST_POS["Gangnam"]
    a3_pos  = SECTION_POS["A-3"]

    amr_path   = path_between(a1_pos, gangnam, steps=22)
    drone_path = path_between((0.0, 0.0), a3_pos, steps=22)

    for i, ((ax, ay), (dx, dy)) in enumerate(zip(amr_path, drone_path)):
        # AMR
        yaw = yaw_toward(a1_pos, gangnam)
        fleet.amr.update_pose(ax, ay, yaw)
        amr_bat -= 0.04
        fleet.amr.update_battery(round(amr_bat, 1))
        # Drone (고도 2.0 유지)
        fleet.drone.update_pose(dx, dy, 2.0)
        drone_bat -= 0.06
        fleet.drone.update_battery(round(drone_bat, 1))
        step()

    # ── 씬 4: 암 picking → placing ───────────────────────────────
    print("[씬 4] Arm → AirPods 배치 중")
    fleet.arm.set_placing()
    step(1.5)

    db.collection("items").document(airpods_item).update({
        "status": "delivered",
        "delivered_at": now_ts(),
    })
    fleet.arm.set_idle()
    print(f"  → AirPods 배송 완료 (item_id={airpods_item})")
    step(0.5)

    # ── 씬 5: AMR 강남 도착 + 드론 A-3 도착 ─────────────────────
    print("[씬 5] AMR → 강남 도착 / Drone → A-3 호버링")

    # AMR 배달 완료
    fleet.amr.set_unloading()
    step(1.5)
    db.collection("items").document(aw_item).update({
        "status": "delivered",
        "delivered_at": now_ts(),
    })
    fleet.amr.set_empty()
    print(f"  → Apple Watch 배송 완료 (item_id={aw_item})")

    # 드론 호버링 at A-3
    fleet.drone._ref.update({
        "charge_status": DroneState.HOVERING,
        "last_updated":  now_ts(),
    })
    mb_item = make_item(db, "MacBook Pro", "A-3", "in_transit",
                        robot="drone_001", detected_sec=5)
    fleet.drone.set_loading(mb_item)
    fleet.drone.set_transporting()
    print(f"  → Drone MacBook Pro 픽업 (item_id={mb_item})")
    step(1.5)

    # ── 씬 6: AMR 복귀 → B-2 / Drone → 구로 ─────────────────────
    print("[씬 6] AMR → B-2 / Drone → 구로 비행")

    b2_pos = SECTION_POS["B-2"]
    guro   = DEST_POS["Guro Digital"]

    amr_path2   = path_between(gangnam, b2_pos, steps=24)
    drone_path2 = path_between(a3_pos, guro, steps=24)

    for (ax, ay), (dx, dy) in zip(amr_path2, drone_path2):
        yaw = yaw_toward(gangnam, b2_pos)
        fleet.amr.update_pose(ax, ay, yaw)
        amr_bat -= 0.04
        fleet.amr.update_battery(round(amr_bat, 1))

        fleet.drone.update_pose(dx, dy, 2.0)
        drone_bat -= 0.06
        fleet.drone.update_battery(round(drone_bat, 1))
        step()

    # ── 씬 7: AMR B-2 도착 (Kindle 픽업) + 암 Galaxy Tab 픽업 ────
    print("[씬 7] AMR B-2 도착 / Arm → Galaxy Tab 픽업")

    kindle_item = make_item(db, "Kindle", "B-2", "in_transit",
                            robot="amr_001", detected_sec=5)
    fleet.amr.set_loading(kindle_item)
    step(1.2)
    fleet.amr.set_transporting()

    # 암 두 번째 픽업: Galaxy Tab
    gtab_item = make_item(db, "Galaxy Tab", "A-2", "in_transit",
                          robot="m0609", detected_sec=8)
    fleet.arm.set_detected_item(1, "Galaxy Tab", "item", (0.05, 0.1, 0.42), gtab_item)
    fleet.arm.set_picking(gtab_item)
    step(0.8)

    # ── 씬 8: 드론 구로 도착 + 배달 ─────────────────────────────
    print("[씬 8] Drone → 구로 도착 + MacBook Pro 배달")
    fleet.drone.set_unloading()
    step(1.5)
    db.collection("items").document(mb_item).update({
        "status": "delivered",
        "delivered_at": now_ts(),
    })
    fleet.drone.set_empty()
    print(f"  → MacBook Pro 배송 완료 (item_id={mb_item})")
    step(0.5)

    # ── 씬 9: 드론 착륙 / AMR B-2 → 서초 ────────────────────────
    print("[씬 9] Drone 착륙 / AMR → 서초")

    seocho = DEST_POS["Seocho"]
    amr_path3  = path_between(b2_pos, seocho, steps=22)
    drone_desc = [(lerp(guro[0], 0.0, s / 20),
                   lerp(guro[1], 0.0, s / 20),
                   lerp(2.0, 0.0, s / 20)) for s in range(21)]

    for i in range(max(len(amr_path3), len(drone_desc))):
        if i < len(amr_path3):
            ax, ay = amr_path3[i]
            yaw = yaw_toward(b2_pos, seocho)
            fleet.amr.update_pose(ax, ay, yaw)
            amr_bat -= 0.04
            fleet.amr.update_battery(round(amr_bat, 1))
        if i < len(drone_desc):
            dx, dy, dz = drone_desc[i]
            fleet.drone.update_pose(dx, dy, dz)
            drone_bat -= 0.04
            fleet.drone.update_battery(round(drone_bat, 1))
        step()

    # 드론 착륙 완료
    fleet.drone._ref.update({
        "charge_status": DroneState.LANDING,
        "last_updated":  now_ts(),
    })
    step(0.5)
    fleet.drone.update_pose(0.0, 0.0, 0.0)
    fleet.drone._ref.update({
        "charge_status": "operating",
        "last_updated":  now_ts(),
    })
    print("  → Drone 착륙 완료")

    # ── 씬 10: AMR 서초 도착 + 암 완료 ──────────────────────────
    print("[씬 10] AMR → 서초 배달 / Arm → Galaxy Tab 배치")

    fleet.amr.set_unloading()
    fleet.arm.set_placing()
    step(1.5)

    db.collection("items").document(kindle_item).update({
        "status": "delivered",
        "delivered_at": now_ts(),
    })
    fleet.amr.set_empty()
    print(f"  → Kindle 배송 완료 (item_id={kindle_item})")

    db.collection("items").document(gtab_item).update({
        "status": "delivered",
        "delivered_at": now_ts(),
    })
    fleet.arm.set_idle()
    print(f"  → Galaxy Tab 배송 완료 (item_id={gtab_item})")
    step(1.0)

    # ── 씬 11: AMR 원점 복귀 ─────────────────────────────────────
    print("[씬 11] AMR 원점 복귀")
    for x, y in path_between(seocho, origin, steps=20):
        yaw = yaw_toward(seocho, origin)
        fleet.amr.update_pose(x, y, yaw)
        amr_bat -= 0.03
        fleet.amr.update_battery(round(amr_bat, 1))
        step()

    fleet.amr.update_pose(0.0, 0.0, 0.0)
    fleet.amr.set_empty()

    # ── 최종 상태 출력 ───────────────────────────────────────────
    print("\n" + "=" * 55)
    print(" 시뮬레이션 완료")
    print("=" * 55)

    items = list(db.collection("items").stream())
    by_status: dict[str, list[str]] = {}
    for doc in items:
        d = doc.to_dict()
        by_status.setdefault(d["status"], []).append(d["name"])

    for status, names in sorted(by_status.items()):
        print(f"  {status:12}: {len(names)}개  {names[:5]}")

    print(f"\n  AMR   배터리: {amr_bat:.1f}%")
    print(f"  Drone 배터리: {drone_bat:.1f}%")
    print("\n대시보드 최종 상태를 http://localhost:5173 에서 확인하세요.")


if __name__ == "__main__":
    main()
