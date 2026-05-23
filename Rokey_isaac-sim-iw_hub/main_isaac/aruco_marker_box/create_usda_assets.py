"""
create_usda_assets.py — ArUco 마커 박스 USD 파일 생성기.

Isaac Sim 없이도 실행 가능 (pxr 라이브러리만 필요).
생성된 .usda 파일은 Isaac Sim GUI 에서 드래그&드롭으로 로드 가능.

실행 방법:
    # Isaac Sim Python 환경에서:
    python /home/rokey/Rokey_isaac-sim/aruco_marker_box/create_usda_assets.py

    # 또는 Isaac Sim 내부 Script Editor 에서:
    exec(open("/home/rokey/Rokey_isaac-sim/aruco_marker_box/create_usda_assets.py").read())

생성 파일 (aruco_marker_box/usd/):
    aruco_box_green_id0.usda  — 초록색, ArUco ID 0
    aruco_box_red_id1.usda    — 빨간색, ArUco ID 1
    aruco_box_blue_id2.usda   — 파란색, ArUco ID 2
"""

from pathlib import Path
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

_THIS_DIR  = Path(__file__).parent
_USD_DIR   = _THIS_DIR / "usd"
_ARUCO_DIR = (
    "/home/rokey/Rokey_isaac-sim/main_isaac/robots/m0609"
    "/m0609_aruco_detect/aruco_marker_6x6"
)

_CONFIGS = [
    ("aruco_box_green_id0.usda", "ArUcoBox_Green_ID0", 0, (0.0, 0.8, 0.0)),
    ("aruco_box_red_id1.usda",   "ArUcoBox_Red_ID1",   1, (0.8, 0.0, 0.0)),
    ("aruco_box_blue_id2.usda",  "ArUcoBox_Blue_ID2",  2, (0.0, 0.0, 0.8)),
]


def _add_box_mesh(stage, parent_path: str, half: float):
    mesh_prim = stage.DefinePrim(f"{parent_path}/Mesh", "Mesh")
    mesh = UsdGeom.Mesh(mesh_prim)

    h = half
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
        # +Z 면
        Gf.Vec3f(-h, -h,  h), Gf.Vec3f( h, -h,  h),
        Gf.Vec3f( h,  h,  h), Gf.Vec3f(-h,  h,  h),
        # -Z 면
        Gf.Vec3f( h, -h, -h), Gf.Vec3f(-h, -h, -h),
        Gf.Vec3f(-h,  h, -h), Gf.Vec3f( h,  h, -h),
    ]
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set([4] * 6)
    mesh.GetFaceVertexIndicesAttr().Set(list(range(24)))

    uv_face = [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]
    pv = UsdGeom.PrimvarsAPI(mesh_prim).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    pv.Set(uv_face * 6)

    face_normals = [
        Gf.Vec3f( 1,  0,  0),
        Gf.Vec3f(-1,  0,  0),
        Gf.Vec3f( 0,  1,  0),
        Gf.Vec3f( 0, -1,  0),
        Gf.Vec3f( 0,  0,  1),
        Gf.Vec3f( 0,  0, -1),
    ]
    normals = [n for n in face_normals for _ in range(4)]
    mesh.GetNormalsAttr().Set(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    return mesh_prim


def _add_material(stage, mat_path: str, tex_file: str, color: tuple):
    material = UsdShade.Material.Define(stage, mat_path)

    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(0.0)

    uv_rd = UsdShade.Shader.Define(stage, f"{mat_path}/UVReader")
    uv_rd.CreateIdAttr("UsdPrimvarReader_float2")
    uv_rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_out = uv_rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)

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


def create_aruco_box_usd(
    filename: str,
    root_name: str,
    aruco_id: int,
    color: tuple,
    size: float = 0.05,
) -> str:
    """단일 ArUco 박스 .usda 파일 생성."""
    out_path = str(_USD_DIR / filename)

    stage = Usd.Stage.CreateNew(out_path)
    stage.SetMetadata("upAxis", "Z")
    stage.SetMetadata("metersPerUnit", 1.0)

    root_prim_path = f"/{root_name}"
    root = stage.DefinePrim(root_prim_path, "Xform")
    stage.SetDefaultPrim(root)

    half      = size / 2.0
    mesh_prim = _add_box_mesh(stage, root_prim_path, half)

    tex_file = f"{_ARUCO_DIR}/aruco_id{aruco_id}.png"
    mat = _add_material(stage, f"{root_prim_path}/Materials/Mat", tex_file, color)
    UsdShade.MaterialBindingAPI(mesh_prim).Bind(mat)

    # 물리
    UsdPhysics.CollisionAPI.Apply(mesh_prim)
    col = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    col.GetApproximationAttr().Set("convexHull")
    UsdPhysics.RigidBodyAPI.Apply(root)
    mass_api = UsdPhysics.MassAPI.Apply(root)
    mass_api.GetMassAttr().Set(0.1)

    stage.Save()
    print(f"[create_usda] 저장 완료 → {out_path}")
    return out_path


def main():
    _USD_DIR.mkdir(parents=True, exist_ok=True)
    for filename, name, aruco_id, color in _CONFIGS:
        create_aruco_box_usd(filename, name, aruco_id, color)
    print(f"\n[create_usda] 3개 USD 파일 생성 완료: {_USD_DIR}")


if __name__ == "__main__":
    main()
