"""
main_isaac/world_setup.py
=========================
창고 씬 공통 설정 (맵 로드 + 조명 + ArUco 박스).
"""
from isaacsim.core.utils.prims import define_prim
from pxr import Sdf, Gf, UsdGeom

from robot_config import WAREHOUSE_USD, ARUCO_BOXES, POD_STACKS


def setup_warehouse(world) -> None:
    """창고 USD 로드 + 원거리 조명 + ArUco 마커 박스 + Pod Stack 스폰."""
    # 창고 맵
    warehouse = define_prim("/World/Warehouse", "Xform")
    warehouse.GetReferences().AddReference(WAREHOUSE_USD)

    # 전체 조명
    light = define_prim("/World/DistantLight", "DistantLight")
    light.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
    light.CreateAttribute("angle",     Sdf.ValueTypeNames.Float).Set(0.53)

    print(f"[WorldSetup] 창고 맵 로드 완료: {WAREHOUSE_USD}")

    # ArUco 마커 박스 로드
    for box in ARUCO_BOXES:
        prim_path = f"/World/ArUcoBoxes/{box['type']}"
        prim = define_prim(prim_path, "Xform")
        prim.GetReferences().AddReference(box["usd"])

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*box["xyz"]))

        print(f"[WorldSetup] ArUco 박스 로드: {box['type']}  pos={box['xyz']}")

    # Pod Stack 로드
    for pod in POD_STACKS:
        prim_path = f"/World/PodStacks/{pod['name']}"
        prim = define_prim(prim_path, "Xform")
        prim.GetReferences().AddReference(pod["usd"])

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pod["xyz"]))
        yaw = float(pod.get("yaw", 0.0))
        if abs(yaw) > 1e-6:
            xf.AddRotateZOp().Set(yaw)

        print(f"[WorldSetup] Pod Stack 로드: {pod['name']}  pos={pod['xyz']}")
