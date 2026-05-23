"""
aruco_box_spawner.py — Isaac Sim 에서 ArUco 마커 큐브를 스폰합니다.

사용법 (Isaac Sim Python 내부):
    import sys
    sys.path.insert(0, "/home/rokey/Rokey_isaac-sim")
    from aruco_marker_box import spawn_aruco_box, spawn_all_aruco_boxes

    # 단일 박스
    prim_path = spawn_aruco_box("green_id0", position=(1.0, 0.0, 0.025))

    # 3개 동시 스폰
    paths = spawn_all_aruco_boxes(positions={
        "green_id0": (0.0, 0.0, 0.025),
        "red_id1":   (0.5, 0.0, 0.025),
        "blue_id2":  (1.0, 0.0, 0.025),
    })

박스 종류:
    green_id0  — 초록색 큐브, ArUco ID 0 (6면 전체)
    red_id1    — 빨간색 큐브, ArUco ID 1 (6면 전체)
    blue_id2   — 파란색 큐브, ArUco ID 2 (6면 전체)
"""

import omni.usd
from pxr import UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

_ARUCO_DIR = (
    "/home/rokey/Rokey_isaac-sim/main_isaac/robots/m0609"
    "/m0609_aruco_detect/aruco_marker_6x6"
)

# 박스 타입 설정: ArUco ID, 색상 (R, G, B)
# 흰색 픽셀 → color 값으로 변환, 검은색 → 검은색 유지
BOX_CONFIGS = {
    "green_id0": {"aruco_id": 0, "color": (0.0, 0.8, 0.0)},
    "red_id1":   {"aruco_id": 1, "color": (0.8, 0.0, 0.0)},
    "blue_id2":  {"aruco_id": 2, "color": (0.0, 0.0, 0.8)},
}


def _make_box_mesh(stage, path: str, half: float):
    """
    UV 분리된 24점 박스 메시 생성 (면마다 전체 텍스처 매핑).
    법선벡터와 UV 모두 faceVarying 방식으로 각 면에 독립 지정.
    """
    mesh_prim = stage.DefinePrim(path, "Mesh")
    mesh = UsdGeom.Mesh(mesh_prim)

    h = half
    # 24점: 면마다 4점 분리 (UV/법선 독립 지정을 위해)
    # 각 면의 정점 순서: 바깥에서 봤을 때 CCW (법선이 바깥을 향함)
    points = [
        # +X 면
        Gf.Vec3f( h, -h, -h), Gf.Vec3f( h,  h, -h),
        Gf.Vec3f( h,  h,  h), Gf.Vec3f( h, -h,  h),
        # -X 면
        Gf.Vec3f(-h,  h, -h), Gf.Vec3f(-h, -h, -h),
        Gf.Vec3f(-h, -h,  h), Gf.Vec3f(-h,  h,  h),
        # +Y 면
        Gf.Vec3f( h,  h, -h), Gf.Vec3f(-h,  h, -h),
        Gf.Vec3f(-h,  h,  h), Gf.Vec3f( h,  h,  h),
        # -Y 면
        Gf.Vec3f(-h, -h, -h), Gf.Vec3f( h, -h, -h),
        Gf.Vec3f( h, -h,  h), Gf.Vec3f(-h, -h,  h),
        # +Z 면 (윗면)
        Gf.Vec3f(-h, -h,  h), Gf.Vec3f( h, -h,  h),
        Gf.Vec3f( h,  h,  h), Gf.Vec3f(-h,  h,  h),
        # -Z 면 (아랫면)
        Gf.Vec3f( h, -h, -h), Gf.Vec3f(-h, -h, -h),
        Gf.Vec3f(-h,  h, -h), Gf.Vec3f( h,  h, -h),
    ]
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set([4] * 6)
    mesh.GetFaceVertexIndicesAttr().Set(list(range(24)))

    # 각 면에 전체 텍스처 [0,1]² 매핑
    uv_face = [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]
    pv = UsdGeom.PrimvarsAPI(mesh_prim).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    pv.Set(uv_face * 6)

    # 각 면의 바깥 방향 법선 (점마다 소속 면의 법선)
    face_normals = [
        Gf.Vec3f( 1,  0,  0),  # +X
        Gf.Vec3f(-1,  0,  0),  # -X
        Gf.Vec3f( 0,  1,  0),  # +Y
        Gf.Vec3f( 0, -1,  0),  # -Y
        Gf.Vec3f( 0,  0,  1),  # +Z
        Gf.Vec3f( 0,  0, -1),  # -Z
    ]
    normals = [n for n in face_normals for _ in range(4)]
    mesh.GetNormalsAttr().Set(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    return mesh_prim


def _make_aruco_material(stage, mat_path: str, tex_file: str, color: tuple):
    """
    UsdPreviewSurface 기반 머티리얼:
      흑백 ArUco 텍스처 × color → 흰 영역=색상, 검은 영역=검정
    """
    material = UsdShade.Material.Define(stage, mat_path)

    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(0.0)

    # UV 좌표 리더
    uv_rd = UsdShade.Shader.Define(stage, f"{mat_path}/UVReader")
    uv_rd.CreateIdAttr("UsdPrimvarReader_float2")
    uv_rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_out = uv_rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    # 텍스처 샘플러: scale 로 회색조 × 색상 곱셈
    tex = UsdShade.Shader.Define(stage, f"{mat_path}/DiffuseTexture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file",  Sdf.ValueTypeNames.Asset).Set(tex_file)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    r, g, b = color
    tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(r, g, b, 1.0))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv_out)
    rgb_out = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb_out)
    surf_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surf_out)

    return material


def spawn_aruco_box(
    box_type: str,
    position: tuple = (0.0, 0.0, 0.025),
    size: float = 0.05,
    prim_path: str | None = None,
    add_physics: bool = True,
) -> str:
    """
    Isaac Sim 스테이지에 ArUco 마커 박스를 스폰합니다.

    Args:
        box_type   : "green_id0" | "red_id1" | "blue_id2"
        position   : 월드 좌표 (x, y, z) [m]
        size       : 박스 한 변 길이 [m] (기본 0.05 = 5 cm)
        prim_path  : USD prim 경로. None 이면 자동 생성.
        add_physics: RigidBody + Collision 추가 여부

    Returns:
        스폰된 박스의 USD prim 경로
    """
    if box_type not in BOX_CONFIGS:
        raise ValueError(
            f"box_type='{box_type}' 이 잘못되었습니다. "
            f"선택 가능: {list(BOX_CONFIGS)}"
        )

    cfg   = BOX_CONFIGS[box_type]
    stage = omni.usd.get_context().get_stage()

    if prim_path is None:
        prim_path = f"/World/ArUcoBoxes/{box_type}"

    # 루트 Xform (월드 위치)
    xform_prim = stage.DefinePrim(prim_path, "Xform")
    xf = UsdGeom.Xformable(xform_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))

    # 박스 메시 (UV·법선 포함)
    half     = size / 2.0
    mesh_prim = _make_box_mesh(stage, f"{prim_path}/Mesh", half)

    # ArUco 머티리얼 적용
    tex_file = f"{_ARUCO_DIR}/aruco_id{cfg['aruco_id']}.png"
    material = _make_aruco_material(
        stage, f"{prim_path}/Materials/Mat", tex_file, cfg["color"]
    )
    UsdShade.MaterialBindingAPI(mesh_prim).Bind(material)

    # 물리 (RigidBody + 충돌)
    if add_physics:
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        col = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        col.GetApproximationAttr().Set("convexHull")

        UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
        mass_api.GetMassAttr().Set(0.1)  # 100 g

    print(f"[aruco_box_spawner] {box_type} 스폰 완료 → {prim_path}  pos={position}")
    return prim_path


def spawn_all_aruco_boxes(
    positions: dict | None = None,
    size: float = 0.05,
    prim_root: str = "/World/ArUcoBoxes",
    add_physics: bool = True,
) -> dict:
    """
    3종 ArUco 박스를 모두 스폰합니다.

    Args:
        positions : {box_type: (x, y, z)} 형태로 위치 지정.
                    미지정 박스는 기본 위치(나란히 배치)를 사용.
        size      : 박스 한 변 길이 [m]
        prim_root : 부모 USD 경로
        add_physics: 물리 추가 여부

    Returns:
        {box_type: prim_path} 딕셔너리
    """
    default_pos = {
        "green_id0": (0.0,  0.0, size / 2),
        "red_id1":   (size * 2, 0.0, size / 2),
        "blue_id2":  (size * 4, 0.0, size / 2),
    }
    if positions:
        default_pos.update(positions)

    return {
        bt: spawn_aruco_box(
            bt,
            position=default_pos[bt],
            size=size,
            prim_path=f"{prim_root}/{bt}",
            add_physics=add_physics,
        )
        for bt in BOX_CONFIGS
    }
