import sys as _sys
from pathlib import Path as _Path
_root = _Path(__file__).resolve().parent
while not (_root / "DB").exists() and _root.parent != _root:
    _root = _root.parent
if str(_root) not in _sys.path:
    _sys.path.insert(0, str(_root))
del _root

"""
Isaac Sim + ArUco marker 분류 인식 메인 스크립트.

실행 방법:
  1. Isaac Sim 환경에서:
       python isaac_aruco_main.py

  2. Isaac Sim 없이 Mock 테스트:
       python isaac_aruco_main.py --mock

  3. 웹캠으로 테스트 (Firebase 연동):
       python isaac_aruco_main.py --webcam 0

  4. Firebase 없이 테스트:
       python isaac_aruco_main.py --webcam 0 --no-firebase

  5. ROS2 토픽 발행 포함:
       python isaac_aruco_main.py --webcam 0 --ros
"""

import argparse
import sys
import time
import yaml
import numpy as np
import cv2
from pathlib import Path

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

from utils.aruco_detector import ArucoDetector, DetectedMarker


# ──────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────

def load_config(config_path: str | Path) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def build_camera_matrix(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    c = cfg["camera"]
    K = np.array([
        [c["fx"],     0, c["cx"]],
        [    0, c["fy"], c["cy"]],
        [    0,     0,      1  ],
    ], dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)
    return K, dist


def build_marker_registry(cfg: dict) -> dict:
    return {int(k): v for k, v in cfg["markers"].items()}


# ──────────────────────────────────────────────
# Firebase 연동
# ──────────────────────────────────────────────

def init_firebase_managers():
    from firebase import init_firebase, RobotFleet, ItemTracker, TaskManager
    db = init_firebase()
    return (
        RobotFleet(db),   # amr_001 / drone_001 / m0609
        ItemTracker(db),
        TaskManager(db),
    )


# ──────────────────────────────────────────────
# ROS2 연동 (선택 사항)
# ──────────────────────────────────────────────

def init_ros2_aruco_bridge():
    """ROS2가 설치된 환경에서만 ArucoBridge를 초기화합니다."""
    try:
        import rclpy
        from ros_bridge.aruco_bridge import ArucoBridge
        rclpy.init()
        node = ArucoBridge()
        print("[ROS2] ArucoBridge 초기화 완료 — /aruco/detections 발행 준비\n")
        return node
    except Exception as e:
        print(f"[ROS2] 초기화 실패 — ROS2 없이 실행합니다.\n  원인: {e}\n")
        return None


# ──────────────────────────────────────────────
# 분류 콜백
# ──────────────────────────────────────────────

# 같은 마커를 반복 등록하지 않도록 쿨다운 관리 {marker_id: last_registered_time}
_last_registered: dict[int, float] = {}
REGISTER_COOLDOWN = 3.0   # 초 (같은 마커 재등록 최소 간격)


def on_detected(marker: DetectedMarker,
                fleet=None, item_tracker=None, task_mgr=None, nav_mgr=None,
                aruco_bridge=None):
    """
    ArUco 마커 검출 콜백 — 역할(role)에 따라 다르게 처리합니다.

      item        → 물품명 저장 + M0609 픽업 + AMR 목적 섹션 할당
      section     → AMR 도착 확인 (목표 섹션이면 unloading)
      destination → AMR 최종 배송지 도착 확인
    """
    role = marker.role
    print(f"  [{role:11}] {marker.label}  ID={marker.marker_id}", end="")
    if marker.position_xyz:
        print(f"  dist={marker.position_xyz[2]:.3f}m", end="")
    print()

    if not fleet:
        return

    # 쿨다운: 동일 마커 중복 처리 방지
    now = time.time()
    if now - _last_registered.get(marker.marker_id, 0) < REGISTER_COOLDOWN:
        return
    _last_registered[marker.marker_id] = now

    # ── 물품 마커: 물품명으로 등록 + M0609 픽업 + AMR 섹션 이동 지시
    if marker.is_item:
        target_section = marker.info.get("target_section")
        destination    = marker.info.get("destination", "")

        item_id = item_tracker.register(
            marker_id=marker.marker_id,
            label=marker.label,        # 물품명 저장 (예: "Apple Watch")
            category="item",
            position_xyz=marker.position_xyz,
        ) if item_tracker else None

        # m0609: 중간 경유지(정렬 구획)로 물품 이동
        pick_task_id = task_mgr.create(
            item_id=item_id or "",
            marker_id=marker.marker_id,
            destination=target_section or "",
            robot_id="m0609",
        ) if task_mgr else None

        # amr_001: 최종 배송지(예: Gangnam)까지 배달
        delivery_task_id = task_mgr.create(
            item_id=item_id or "",
            marker_id=marker.marker_id,
            destination=destination,
            robot_id="amr_001",
        ) if task_mgr else None

        fleet.arm.set_detected_item(
            marker_id=marker.marker_id,
            label=marker.label,
            category="item",
            position_xyz=marker.position_xyz,
            item_id=item_id,
        )
        if pick_task_id:
            fleet.arm.set_picking(task_id=pick_task_id)
        if delivery_task_id:
            fleet.amr.set_loading(task_id=delivery_task_id)

        # AMR에 목적 섹션 할당 → 이동 시작
        if nav_mgr and item_id and target_section:
            nav_mgr.navigate_to_section("amr_001", target_section, item_id)
            fleet.amr.set_transporting()
            print(f"  → AMR target section: {target_section}  ({marker.label})")

    # ── 섹션 마커: AMR 목표 섹션 도착 확인
    elif marker.is_section:
        detected_section = marker.info.get("section_id", marker.label)
        pos = marker.position_xyz

        # 위치 보정 (AMR + 드론)
        if pos:
            fleet.amr.set_localization(
                marker_id=marker.marker_id,
                label=marker.label,
                estimated_pos={"x": pos[0], "y": pos[1], "yaw": 0.0},
                distance=pos[2],
            )
            fleet.drone.set_localization(
                marker_id=marker.marker_id,
                label=marker.label,
                estimated_pos={"x": pos[0], "y": pos[1], "z": pos[2]},
                distance=pos[2],
            )

        # 목표 섹션 도착 확인
        if nav_mgr:
            arrived = nav_mgr.confirm_arrival("amr_001", detected_section)
            if arrived:
                fleet.amr.set_unloading()
                print(f"  → AMR arrived at '{detected_section}' → unloading")

    # ── 배송지 마커: AMR 최종 배송 완료
    elif marker.is_destination:
        fleet.amr.set_unloading()
        if nav_mgr:
            nav_mgr.set_unloading_done("amr_001")
        print(f"  → AMR final delivery '{marker.label}' confirmed")

    # ── ROS2 발행: 역할과 관계없이 모든 검출 결과를 토픽으로 발행
    if aruco_bridge:
        aruco_bridge.publish_detection(
            marker_id=marker.marker_id,
            role=marker.role,
            label=marker.label,
            position_xyz=marker.position_xyz,
            extra={k: v for k, v in marker.info.items()
                   if k in ("target_section", "destination", "section_id")},
        )


# ──────────────────────────────────────────────
# Isaac Sim 씬 설정
# ──────────────────────────────────────────────

def setup_isaac_scene(cfg: dict):
    from omni.isaac.kit import SimulationApp  # type: ignore
    simulation_app = SimulationApp({"headless": False})
    from omni.isaac.core import World          # type: ignore
    from omni.isaac.core.objects import DynamicCuboid  # type: ignore

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    for idx, (marker_id, info) in enumerate(cfg["markers"].items()):
        world.scene.add(DynamicCuboid(
            prim_path=f"/World/item_{marker_id}",
            name=info["label"],
            position=np.array([float(idx) * 0.25, 0.0, 0.05]),
            scale=np.array([0.08, 0.08, 0.08]),
        ))
    return simulation_app, world


# ──────────────────────────────────────────────
# 검출 루프
# ──────────────────────────────────────────────

def run_detection_loop(detector: ArucoDetector, camera,
                        fleet=None, item_tracker=None, task_mgr=None,
                        nav_mgr=None, display: bool = True,
                        aruco_bridge=None):
    camera.initialize()
    frame_count = 0
    t_start = time.time()
    print("[INFO] ArUco 분류 인식 시작. 'q' 키로 종료합니다.\n")

    while True:
        frame = camera.get_rgb()
        if frame is None:
            time.sleep(0.01)
            continue

        items = detector.detect(frame)

        if items:
            print(f"[Frame {frame_count}] 검출: {len(items)}개")
            for item in items:
                on_detected(item, fleet, item_tracker, task_mgr, nav_mgr,
                            aruco_bridge)

        if display:
            vis = detector.draw_detections(frame, items)
            fps = frame_count / max(time.time() - t_start, 1e-6)
            firebase_tag = "Firebase:ON" if item_tracker else "Firebase:OFF"
            cv2.putText(vis, f"FPS:{fps:.1f}  {firebase_tag}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("ArUco Classifier", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        frame_count += 1

    cv2.destroyAllWindows()
    if fleet:
        fleet.arm.set_status("idle")
        fleet.amr.set_status("idle")
        fleet.drone.set_status("idle")

    # 작업 통계 출력
    if task_mgr:
        stats = task_mgr.get_stats()
        print("\n=== 작업 통계 ===")
        for k, v in stats["status_counts"].items():
            print(f"  {k}: {v}건")
        print("  목적지별 완료:")
        for dest, cnt in stats["destination_counts"].items():
            print(f"    {dest}: {cnt}건")

    print(f"\n[INFO] 종료. 총 {frame_count}프레임 처리.")


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Isaac Sim ArUco Classifier")
    parser.add_argument("--config",      default=str(ROOT / "config" / "object_registry.yaml"))
    parser.add_argument("--mock",        action="store_true")
    parser.add_argument("--webcam",      type=int,  default=None)
    parser.add_argument("--video",       type=str)
    parser.add_argument("--no-display",  action="store_true")
    parser.add_argument("--no-firebase", action="store_true", help="Firebase 연동 없이 실행")
    parser.add_argument("--ros",         action="store_true", help="ROS2 토픽 발행 활성화")
    args = parser.parse_args()

    cfg      = load_config(args.config)
    K, dist  = build_camera_matrix(cfg)
    registry = build_marker_registry(cfg)

    print("=== 등록된 분류 기준 ===")
    for mid, info in registry.items():
        print(f"  ID {mid}  →  {info['label']}  ({info.get('description', info.get('role', ''))})")
    print()

    # Firebase 초기화
    fleet, item_tracker, task_mgr, nav_mgr = None, None, None, None
    if not args.no_firebase:
        try:
            from firebase import init_firebase, RobotFleet, TaskManager
            from DB.inventory import ItemTracker
            from DB.navigation import NavigationManager
            db           = init_firebase()
            fleet        = RobotFleet(db)
            item_tracker = ItemTracker(db)
            task_mgr     = TaskManager(db)

            # 네비게이션 맵: 구획(A-1 등) + 최종 배송지(Gangnam 등) 모두 포함
            # key = 탐색 대상 이름,  value = 실제 좌표 {x, y, z}
            section_map = {}
            for v in registry.values():
                role = v.get("role")
                if role == "section" and "section_id" in v:
                    section_map[v["section_id"]] = v.get("position", {})
                elif role == "destination" and "destination" in v:
                    section_map[v["destination"]] = v.get("position", {})

            nav_mgr = NavigationManager(db, section_map)
            print(f"[Firebase] 연결 완료  네비게이션 맵: {list(section_map.keys())}\n")
        except Exception as e:
            print(f"[Firebase] 연결 실패 — Firebase 없이 실행합니다.\n  원인: {e}\n")

    detector = ArucoDetector(
        camera_matrix=K,
        dist_coeffs=dist,
        aruco_dict_name=cfg["aruco"]["dictionary"],
        marker_registry=registry,
    )

    # 카메라 소스 선택
    if args.video:
        from utils.video_camera import VideoCamera
        camera = VideoCamera(args.video)
    elif args.webcam is not None:
        from utils.video_camera import VideoCamera
        camera = VideoCamera(args.webcam)
    elif args.mock:
        from utils.isaac_camera import MockIsaacCamera
        camera = MockIsaacCamera(
            width=cfg["camera"]["width"], height=cfg["camera"]["height"],
            fx=cfg["camera"]["fx"],       fy=cfg["camera"]["fy"],
        )
    else:
        _, _ = setup_isaac_scene(cfg)
        from utils.isaac_camera import IsaacCamera
        camera = IsaacCamera(
            prim_path="/World/Camera",
            width=cfg["camera"]["width"],
            height=cfg["camera"]["height"],
        )

    # ROS2 ArUco 브릿지 초기화 (--ros 옵션 시)
    aruco_bridge = None
    if args.ros:
        aruco_bridge = init_ros2_aruco_bridge()

    run_detection_loop(
        detector, camera,
        fleet=fleet,
        item_tracker=item_tracker,
        task_mgr=task_mgr,
        nav_mgr=nav_mgr,
        display=not args.no_display,
        aruco_bridge=aruco_bridge,
    )


if __name__ == "__main__":
    main()
