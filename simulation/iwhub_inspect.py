"""
iw.hub USD 파일 내부 구조 확인 스크립트
Isaac Sim Script Editor에서 실행 → 실제 Joint 이름과 LiDAR 경로 출력

iwhub_action_graph.py 실행 전에 먼저 이 스크립트를 실행해서
LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT, lidar_prim_path 값을 확인하세요.
"""

from pxr import Usd, UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
import carb

ROBOT_PRIM_PATH = "/World/iw_hub"  # 씬의 실제 경로로 수정


def inspect_iwhub():
    stage = get_current_stage()
    root = stage.GetPrimAtPath(ROBOT_PRIM_PATH)

    if not root.IsValid():
        carb.log_error(f"[inspect] Prim not found: {ROBOT_PRIM_PATH}")
        print(f"[ERROR] {ROBOT_PRIM_PATH} 를 찾을 수 없습니다.")
        print("  → 씬에 iw_hub 로봇이 로드되었는지 확인하세요.")
        return

    print("\n" + "=" * 60)
    print(f"[iw.hub 구조 분석]  {ROBOT_PRIM_PATH}")
    print("=" * 60)

    joints = []
    lidar_prims = []
    camera_prims = []

    # 전체 Prim 트리 순회
    for prim in Usd.PrimRange(root):
        prim_path = str(prim.GetPath())
        prim_type = prim.GetTypeName()

        # RevoluteJoint (바퀴 등 회전 조인트)
        if prim_type == "PhysicsRevoluteJoint":
            joint_name = prim.GetName()
            joints.append((joint_name, prim_path))

        # LiDAR 센서
        if "lidar" in prim.GetName().lower() or "Lidar" in prim_type:
            lidar_prims.append(prim_path)

        # Camera 센서
        if prim_type == "Camera":
            camera_prims.append(prim_path)

    # 결과 출력
    print("\n[RevoluteJoint (바퀴/관절)]")
    if joints:
        for name, path in joints:
            print(f"  Joint Name : '{name}'")
            print(f"  Full Path  :  {path}")
            print()
    else:
        print("  (RevoluteJoint 없음 - Prim 경로 확인 필요)")

    print("[LiDAR Sensor]")
    if lidar_prims:
        for p in lidar_prims:
            print(f"  {p}")
    else:
        print("  (LiDAR 없음 - iw_hub_sensors.usd 사용 중인지 확인)")

    print("\n[Camera Sensor]")
    if camera_prims:
        for p in camera_prims:
            print(f"  {p}")
    else:
        print("  (Camera 없음)")

    # action_graph.py 에 넣을 값 자동 출력
    print("\n" + "=" * 60)
    print("[iwhub_action_graph.py 에 복사할 값]")
    print("=" * 60)
    if len(joints) >= 2:
        print(f"LEFT_WHEEL_JOINT  = \"{joints[0][0]}\"")
        print(f"RIGHT_WHEEL_JOINT = \"{joints[1][0]}\"")
    if lidar_prims:
        print(f"lidar_prim_path   = \"{lidar_prims[0]}\"")
    print("=" * 60)


inspect_iwhub()
