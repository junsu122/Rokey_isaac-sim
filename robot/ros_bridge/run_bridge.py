"""
ROS2-Firebase 브릿지 전체 실행 엔트리포인트.

실행:
  python3 ros_bridge/run_bridge.py

옵션:
  --amr-only    AMR 브릿지만 실행
  --drone-only  드론 브릿지만 실행
  --arm-only    암 브릿지만 실행
"""

import argparse
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import rclpy
from rclpy.executors import MultiThreadedExecutor

import sys as _sys
from pathlib import Path as _Path
_root = _Path(__file__).resolve().parent
while not (_root / "DB").exists() and _root.parent != _root:
    _root = _root.parent
if str(_root) not in _sys.path:
    _sys.path.insert(0, str(_root))
del _root

from DB.firebase_manager import init_firebase
from ros_bridge.amr_bridge   import AMRBridge
from ros_bridge.drone_bridge  import DroneBridge
from ros_bridge.arm_bridge    import ArmBridge


def main():
    parser = argparse.ArgumentParser(description="ROS2-Firebase 브릿지")
    parser.add_argument("--amr-only",   action="store_true")
    parser.add_argument("--drone-only", action="store_true")
    parser.add_argument("--arm-only",   action="store_true")
    parser.add_argument("--update-interval", type=float, default=0.5,
                        help="Firebase 업데이트 최소 간격 (초, 기본값: 0.5)")
    args = parser.parse_args()

    # Firebase 초기화
    print("[Bridge] Firebase 연결 중...")
    db = init_firebase()
    print("[Bridge] Firebase 연결 완료")

    # ROS2 초기화
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    run_all = not any([args.amr_only, args.drone_only, args.arm_only])
    iv = args.update_interval

    try:
        if run_all or args.amr_only:
            amr_node = AMRBridge(db, update_interval=iv)
            executor.add_node(amr_node)
            nodes.append(amr_node)
            print("[Bridge] AMR 브릿지 시작 (amr_001)")

        if run_all or args.drone_only:
            drone_node = DroneBridge(db, update_interval=iv)
            executor.add_node(drone_node)
            nodes.append(drone_node)
            print("[Bridge] 드론 브릿지 시작 (drone_001)")

        if run_all or args.arm_only:
            arm_node = ArmBridge(db, update_interval=iv)
            executor.add_node(arm_node)
            nodes.append(arm_node)
            print("[Bridge] 암 브릿지 시작 (m0609)")

        print("\n[Bridge] 모든 브릿지 실행 중. Ctrl+C 로 종료합니다.\n")
        executor.spin()

    except KeyboardInterrupt:
        print("\n[Bridge] 종료 중...")
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        print("[Bridge] 종료 완료")


if __name__ == "__main__":
    main()
