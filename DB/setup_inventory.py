"""
구획(sections)과 물품 마스터(products)를 Firestore에 초기 등록합니다.
처음 한 번만 실행하면 됩니다.

실행: python3 firebase/setup_inventory.py
"""

import sys
import yaml
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
from DB.inventory import SectionManager, ProductManager

CONFIG_PATH = ROOT / "config" / "object_registry.yaml"


def setup_sections(db, cfg: dict):
    mgr = SectionManager(db)
    print("=== 구획 등록 ===")
    for marker_id, info in cfg["markers"].items():
        if info.get("role") != "section":
            continue
        pos = info.get("position", {})
        mgr.register(
            section_id=info["section_id"],
            description=info["label"],
            position={"x": pos.get("x", 0.0),
                      "y": pos.get("y", 0.0),
                      "z": pos.get("z", 0.0)},
            capacity=5,
        )


def setup_products(db, cfg: dict):
    mgr = ProductManager(db)
    print("\n=== 물품 마스터 등록 ===")
    for marker_id, info in cfg["markers"].items():
        if info.get("role") != "item":
            continue
        mgr.register(
            name=info["label"],
            marker_id=int(marker_id),
            section=info.get("target_section", ""),
            destination=info.get("destination", ""),
            description=f"{info['label']} — dest: {info.get('destination','')}",
        )


def main():
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    db = init_firebase()
    setup_sections(db, cfg)
    setup_products(db, cfg)

    print("\n=== 등록 결과 확인 ===")

    print("\n[구획 목록]")
    for s in SectionManager(db).get_all():
        print(f"  {s['section_id']:8}  {s['description']:15}"
              f"  위치=({s['position']['x']:.1f}, {s['position']['y']:.1f})")

    print("\n[물품 마스터]")
    for p in ProductManager(db).get_all():
        print(f"  {p['product_id']}  '{p['name']:15}'  "
              f"마커={p['marker_id']}  구획={p['section']}  배송지={p['destination']}")

    print("\n완료.")


if __name__ == "__main__":
    main()
