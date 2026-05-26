"""
realsense_mount.py
==================
RealSense D455 를 부모 prim 에 부착하는 헬퍼.

형상 전략
─────────
1) Nucleus USD 참조가 성공하면 → 실제 D455 메시 + 내장 Camera prim 사용
2) 참조 실패(Nucleus 미연결)하면 → 아래 절차적 형상으로 대체

절차적 D455 형상 (★ 수정하고 싶으면 여기서만 고치세요 ★)
    d455_body       — 본체 직육면체  124 × 29 × 26 mm  (어두운 남색)
    lens_ir_left    — 좌측 IR 렌즈 원판
    lens_rgb        — 중앙 RGB 렌즈
    lens_ir_right   — 우측 IR 렌즈
    lens_projector  — 도트 프로젝터
"""
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
import omni.usd

REALSENSE_D455_USD_REL = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

# ══════════════════════════════════════════════════════════════════════
#  ★ D455 절차적 형상 파라미터 ★
# ══════════════════════════════════════════════════════════════════════

# 본체 크기 (x=depth/전방, y=width/가로, z=height/세로)  단위 m
_D455_BODY_X   = 0.026   # 전후 깊이   26 mm
_D455_BODY_Y   = 0.124   # 좌우 너비  124 mm
_D455_BODY_Z   = 0.029   # 상하 높이   29 mm

# 본체 색 (R, G, B  0~1)
_D455_COLOR_BODY = Gf.Vec3f(0.13, 0.16, 0.20)   # 어두운 남색

# 렌즈 공통 깊이 (x 방향, 본체 앞면 돌출)
_D455_LENS_DEPTH = 0.003   # 3 mm

# 렌즈 위치·크기  (name, y_offset, z_offset, radius)
# y: 가로 방향, z: 세로 방향.  x=_D455_BODY_X/2 + 렌즈절반깊이 가 전방 면
_D455_LENSES = [
    ("lens_ir_left",   +0.050,  0.0, 0.007),   # 좌측 IR
    ("lens_rgb",       +0.015,  0.0, 0.009),   # 중앙 RGB
    ("lens_ir_right",  -0.030,  0.0, 0.007),   # 우측 IR
    ("lens_projector", -0.050,  0.0, 0.005),   # 도트 프로젝터
]

_D455_COLOR_LENS      = Gf.Vec3f(0.06, 0.06, 0.09)   # 어두운 렌즈
_D455_COLOR_PROJECTOR = Gf.Vec3f(0.60, 0.10, 0.10)   # 프로젝터 (붉은 점)

# ══════════════════════════════════════════════════════════════════════


def _get_assets_root():
    try:
        from isaacsim.storage.native import get_assets_root_path
        return get_assets_root_path()
    except ImportError:
        try:
            from omni.isaac.core.utils.nucleus import get_assets_root_path
            return get_assets_root_path()
        except ImportError:
            return None


def _build_d455_body(stage, parent_path: str) -> None:
    """
    D455 절차적 외형을 parent_path 하위에 생성.
    Nucleus USD 와 충돌하지 않는 prim 이름(d455_* / lens_*)을 사용.
    """
    # ── 본체 ──
    body = UsdGeom.Cube.Define(stage, f"{parent_path}/d455_body")
    body.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(body.GetPrim())
    xf.ClearXformOpOrder()
    # Cube size=1 → scale 로 실제 치수 적용
    xf.AddScaleOp().Set(Gf.Vec3f(_D455_BODY_X, _D455_BODY_Y, _D455_BODY_Z))
    body.GetPrim().CreateAttribute(
        "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
    ).Set([_D455_COLOR_BODY])

    # ── 렌즈 / 프로젝터 ──
    front_x = _D455_BODY_X / 2.0 + _D455_LENS_DEPTH / 2.0
    for name, y_off, z_off, radius in _D455_LENSES:
        cyl = UsdGeom.Cylinder.Define(stage, f"{parent_path}/{name}")
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(_D455_LENS_DEPTH))
        cyl.CreateAxisAttr("X")          # X+ 방향이 카메라 전방
        lxf = UsdGeom.Xformable(cyl.GetPrim())
        lxf.ClearXformOpOrder()
        lxf.AddTranslateOp().Set(Gf.Vec3d(float(front_x), float(y_off), float(z_off)))
        color = (_D455_COLOR_PROJECTOR if "projector" in name else _D455_COLOR_LENS)
        cyl.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
        ).Set([color])


def attach_realsense_d455(parent_prim_path: str,
                          child_name: str = "realsense_d455",
                          translation=(0.05, 0.0, 0.02),
                          rpy_deg=(180.0, 0.0, 0.0),
                          usd_path: str | None = None) -> str:
    """
    parent_prim_path 아래에 RealSense D455 를 부착.

    동작 방식
    ─────────
    • Nucleus 연결 시: 실제 rsd455.usd 참조 + 절차적 외형 추가
    • Nucleus 미연결 : 절차적 외형만 생성 (Camera prim 은 별도로 생성됨)

    반환: 생성된 RealSense Xform prim 경로
    """
    stage = omni.usd.get_context().get_stage()

    rs_prim_path = f"{parent_prim_path}/{child_name}"

    # 이미 있으면 제거하고 재생성
    existing = stage.GetPrimAtPath(rs_prim_path)
    if existing and existing.IsValid():
        stage.RemovePrim(rs_prim_path)

    # ── Xform 생성 + 위치/방향 설정 ──────────────────────────────────
    rs_prim = stage.DefinePrim(rs_prim_path, "Xform")
    xform = UsdGeom.Xformable(rs_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rpy_deg))

    # ── Nucleus USD 참조 시도 ─────────────────────────────────────────
    nucleus_ok = False
    if usd_path is None:
        root = _get_assets_root()
        if root:
            usd_path = root + REALSENSE_D455_USD_REL

    if usd_path:
        try:
            rs_prim.GetReferences().AddReference(usd_path)
            nucleus_ok = True
            print(f"[realsense_mount] Nucleus USD 참조 성공: {usd_path}")
        except Exception as e:
            print(f"[realsense_mount] Nucleus 참조 실패({e}) — 절차적 형상 사용")
    else:
        print("[realsense_mount] Nucleus 미연결 — 절차적 형상 사용")

    # ── 절차적 D455 외형 (항상 생성) ─────────────────────────────────
    # Nucleus USD 가 로드돼도 prim 이름이 겹치지 않으므로 병존 가능
    _build_d455_body(stage, rs_prim_path)

    # ── 물리 API 비활성화 ─────────────────────────────────────────────
    for prim in Usd.PrimRange(stage.GetPrimAtPath(rs_prim_path)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)

    print(f"[realsense_mount] D455 부착 완료: {rs_prim_path}  "
          f"(nucleus={'OK' if nucleus_ok else 'N/A'})")
    return rs_prim_path
