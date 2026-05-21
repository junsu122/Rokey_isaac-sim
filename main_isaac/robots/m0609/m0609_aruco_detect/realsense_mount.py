from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

REALSENSE_D455_USD_REL = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

# color 카메라로 인정할 이름 키워드 (나머지는 비활성화)
_COLOR_CAM_KEYWORDS = ("OV9782", "Color")


def _get_assets_root():
    try:
        from isaacsim.storage.native import get_assets_root_path
        return get_assets_root_path()
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        return get_assets_root_path()


def attach_realsense_d455(parent_prim_path: str,
                          child_name: str = "realsense_d455",
                          translation=(0.05, 0.0, 0.02),
                          rpy_deg=(180.0, 0.0, 0.0),
                          usd_path: str | None = None) -> str:
    """
    parent_prim_path 아래에 RealSense D455 mesh 를 reference 로 부착.
    반환: 생성된 RealSense Xform prim path (wrist_camera.py 에서 sensor 부모로 사용)
    """
    stage = omni.usd.get_context().get_stage()

    if usd_path is None:
        root = _get_assets_root()
        if root is None:
            raise RuntimeError(
                "Isaac Sim assets root 를 찾을 수 없습니다. "
                "Nucleus 연결을 확인하거나 usd_path 를 직접 지정하세요."
            )
        usd_path = root + REALSENSE_D455_USD_REL

    rs_prim_path = f"{parent_prim_path}/{child_name}"

    existing = stage.GetPrimAtPath(rs_prim_path)
    if existing and existing.IsValid():
        stage.RemovePrim(rs_prim_path)

    rs_prim = stage.DefinePrim(rs_prim_path, "Xform")
    rs_prim.GetReferences().AddReference(usd_path)

    xform = UsdGeom.Xformable(rs_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rpy_deg))

    for prim in Usd.PrimRange(stage.GetPrimAtPath(rs_prim_path)):
        # 물리 비활성화 (기존)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)

        # color 카메라 외 depth/IR 카메라 비활성화 → 렌더링 부하 제거
        if prim.IsA(UsdGeom.Camera):
            name = prim.GetName()
            if not any(kw in name for kw in _COLOR_CAM_KEYWORDS):
                prim.SetActive(False)
                print(f"[realsense_mount] deactivated non-color camera: {prim.GetPath()}")

    print(f"[realsense_mount] D455 attached at {rs_prim_path}")
    print(f"                  source USD = {usd_path}")
    return rs_prim_path
