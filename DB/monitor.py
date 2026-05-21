"""
Firestore 데이터 조회 스크립트.

실행:
  # 전체 현황 한 번 출력
  python3 firebase/monitor.py

  # 실시간 모니터링 (변경될 때마다 자동 출력)
  python3 firebase/monitor.py --watch

  # 특정 항목만 조회
  python3 firebase/monitor.py --robots
  python3 firebase/monitor.py --items
  python3 firebase/monitor.py --tasks
  python3 firebase/monitor.py --history arm m0609
  python3 firebase/monitor.py --history amr amr_001
  python3 firebase/monitor.py --history drone drone_001
"""

import argparse
import sys
import time
from pathlib import Path
from datetime import timezone

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


# ── 출력 포맷 헬퍼 ────────────────────────────────────────────

def fmt_ts(ts) -> str:
    if ts is None:
        return "-"
    try:
        return ts.astimezone().strftime("%m/%d %H:%M:%S")
    except Exception:
        return str(ts)


def fmt_pos(pos: dict | None) -> str:
    if not pos:
        return "-"
    parts = [f"{k}={v:.2f}" for k, v in pos.items() if isinstance(v, (int, float))]
    return "  ".join(parts)


def separator(title: str = ""):
    line = "─" * 60
    if title:
        print(f"\n{'─'*20} {title} {'─'*20}")
    else:
        print(line)


# ── 로봇 현황 ─────────────────────────────────────────────────

def print_robots(db):
    separator("로봇 현황")
    docs = db.collection("robots").stream()
    found = False
    for doc in docs:
        found = True
        d = doc.to_dict()
        rtype = d.get("type", "?")
        rid   = d.get("robot_id", doc.id)

        print(f"\n  [{rtype.upper()}]  {rid}")
        print(f"    배터리      : {d.get('battery', '-')}%")
        print(f"    충전 상태   : {d.get('charge_status', '-')}")
        print(f"    적재 상태   : {d.get('cargo_status', d.get('status', '-'))}")
        print(f"    위치        : {fmt_pos(d.get('position'))}")
        print(f"    현재 작업   : {d.get('current_task', '-')}")
        print(f"    마지막 업데이트: {fmt_ts(d.get('last_updated'))}")

        # 로봇팔 전용: 인식 물품
        if rtype == "arm" and d.get("detected_item"):
            item = d["detected_item"]
            print(f"    인식 물품   : ID={item.get('marker_id')}  "
                  f"'{item.get('label')}'  {fmt_ts(item.get('detected_at'))}")

        # AMR/드론 전용: 위치 인식
        loc = d.get("localization")
        if loc:
            print(f"    위치 인식   : ID={loc.get('marker_id')}  "
                  f"'{loc.get('label')}'  거리={loc.get('distance')}m  "
                  f"{fmt_ts(loc.get('detected_at'))}")

    if not found:
        print("  (데이터 없음)")


# ── 물품 현황 ─────────────────────────────────────────────────

def print_items(db, status_filter: str | None = None):
    separator("물품 현황")
    q = db.collection("items")
    if status_filter:
        q = q.where("status", "==", status_filter)
    q = q.order_by("detected_at", direction="DESCENDING").limit(20)

    docs = list(q.stream())
    if not docs:
        print("  (데이터 없음)")
        return

    status_icon = {
        "detected":  "🔍",
        "picked":    "🤖",
        "delivered": "✅",
    }
    for doc in docs:
        d = doc.to_dict()
        icon = status_icon.get(d.get("status", ""), "•")
        print(f"  {icon} [{d.get('status','?'):12}]  "
              f"ID={d.get('marker_id')}  '{d.get('label','?')}'  "
              f"인식={fmt_ts(d.get('detected_at'))}  "
              f"배달={fmt_ts(d.get('delivered_at'))}")


# ── 작업 현황 ─────────────────────────────────────────────────

def print_tasks(db, status_filter: str | None = None):
    separator("작업 현황")
    q = db.collection("tasks")
    if status_filter:
        q = q.where("status", "==", status_filter)
    q = q.order_by("created_at", direction="DESCENDING").limit(20)

    docs = list(q.stream())
    if not docs:
        print("  (데이터 없음)")
        return

    for doc in docs:
        d = doc.to_dict()
        print(f"  [{d.get('status','?'):12}]  "
              f"작업={doc.id}  로봇={d.get('robot_id')}  "
              f"목적지='{d.get('destination','?')}'  "
              f"생성={fmt_ts(d.get('created_at'))}")


# ── 통계 ──────────────────────────────────────────────────────

def print_stats(db):
    separator("통계")
    items = [d.to_dict() for d in db.collection("items").stream()]
    tasks = [d.to_dict() for d in db.collection("tasks").stream()]

    dest_count: dict[str, int] = {}
    status_count: dict[str, int] = {}
    for item in items:
        label  = item.get("label", "?")
        status = item.get("status", "?")
        dest_count[label]   = dest_count.get(label, 0) + 1
        status_count[status] = status_count.get(status, 0) + 1

    print(f"\n  총 물품    : {len(items)}건  |  총 작업: {len(tasks)}건")
    print("\n  목적지별 물품 수:")
    for dest, cnt in sorted(dest_count.items()):
        bar = "█" * cnt
        print(f"    {dest:15} : {bar} ({cnt})")
    print("\n  상태별 물품 수:")
    for st, cnt in sorted(status_count.items()):
        print(f"    {st:12} : {cnt}건")


# ── 인식/위치 이력 ────────────────────────────────────────────

def print_history(db, robot_type: str, robot_id: str):
    separator(f"{robot_type.upper()} / {robot_id} 이력")
    doc = db.collection("robots").document(robot_id).get()
    if not doc.exists:
        print(f"  로봇 '{robot_id}' 없음")
        return

    d = doc.to_dict()

    if robot_type == "arm":
        history = d.get("detection_history", [])
        print(f"\n  [물품 인식 이력]  총 {len(history)}건")
        for h in reversed(history):
            print(f"    ID={h.get('marker_id')}  '{h.get('label')}'  "
                  f"위치={h.get('position_xyz')}  {fmt_ts(h.get('detected_at'))}")
    else:
        history = d.get("localization_history", [])
        print(f"\n  [위치 인식 이력]  총 {len(history)}건")
        for h in reversed(history):
            print(f"    ID={h.get('marker_id')}  '{h.get('label')}'  "
                  f"추정위치={fmt_pos(h.get('estimated_pos'))}  "
                  f"거리={h.get('distance')}m  {fmt_ts(h.get('detected_at'))}")


# ── 실시간 감시 ───────────────────────────────────────────────

def watch(db):
    print("실시간 모니터링 시작 (Ctrl+C 로 종료)\n")

    def on_robot_change(col_snapshot, changes, read_time):
        print(f"\n[{fmt_ts(read_time)}] 로봇 상태 변경")
        for change in changes:
            d = change.document.to_dict()
            rtype = d.get("type", "?").upper()
            rid   = d.get("robot_id", change.document.id)
            cargo = d.get("cargo_status", d.get("status", "-"))
            print(f"  {rtype} / {rid}  적재={cargo}  배터리={d.get('battery','-')}%")

    def on_item_change(col_snapshot, changes, read_time):
        print(f"\n[{fmt_ts(read_time)}] 물품 변경")
        for change in changes:
            d = change.document.to_dict()
            print(f"  [{d.get('status','?')}]  ID={d.get('marker_id')}  '{d.get('label')}'")

    robot_watch = db.collection("robots").on_snapshot(on_robot_change)
    item_watch  = db.collection("items").on_snapshot(on_item_change)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot_watch.unsubscribe()
        item_watch.unsubscribe()
        print("\n모니터링 종료")


# ── Entry point ───────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Firestore 데이터 조회")
    parser.add_argument("--watch",   action="store_true", help="실시간 모니터링")
    parser.add_argument("--robots",  action="store_true")
    parser.add_argument("--items",   action="store_true")
    parser.add_argument("--tasks",   action="store_true")
    parser.add_argument("--stats",   action="store_true")
    parser.add_argument("--history", nargs=2, metavar=("TYPE", "ID"),
                        help="예: --history arm m0609  또는  --history amr amr_001")
    args = parser.parse_args()

    db = init_firebase()

    if args.watch:
        watch(db)
        return

    # 특정 항목만 지정했으면 해당 항목만 출력
    specific = any([args.robots, args.items, args.tasks, args.stats, args.history])

    if not specific or args.robots:
        print_robots(db)
    if not specific or args.items:
        print_items(db)
    if not specific or args.tasks:
        print_tasks(db)
    if not specific or args.stats:
        print_stats(db)
    if args.history:
        print_history(db, args.history[0], args.history[1])

    print()


if __name__ == "__main__":
    main()
