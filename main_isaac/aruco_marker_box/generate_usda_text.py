"""
generate_usda_text.py — pxr 없이 일반 Python 으로 USDA 파일 생성.

USDA 는 텍스트 형식이므로 Isaac Sim 없이도 생성 가능.
생성된 .usda 파일은 Isaac Sim Content Browser 에서 드래그&드롭으로 로드 가능.

실행:
    python3 /home/rokey/Rokey_isaac-sim-main_sub/main_isaac/aruco_marker_box/generate_usda_text.py
"""

from pathlib import Path

_THIS_DIR  = Path(__file__).parent
_USD_DIR   = _THIS_DIR / "usd"
_ARUCO_DIR = "../../robots/m0609/m0609_aruco_detect/aruco_marker_6x6"

_BOX_CONFIGS = [
    ("aruco_box_green_id0.usda", "ArUcoBox_Green_ID0", 0, (0.0, 0.8, 0.0)),
    ("aruco_box_red_id1.usda",   "ArUcoBox_Red_ID1",   1, (0.8, 0.0, 0.0)),
    ("aruco_box_blue_id2.usda",  "ArUcoBox_Blue_ID2",  2, (0.0, 0.0, 0.8)),
]


def _box_points_str(h: float) -> str:
    """24점 박스 정점 USDA 문자열 반환 (면마다 4점 분리)."""
    pts = [
        # +X 면
        ( h, -h, -h), ( h,  h, -h), ( h,  h,  h), ( h, -h,  h),
        # -X 면
        (-h,  h, -h), (-h, -h, -h), (-h, -h,  h), (-h,  h,  h),
        # +Y 면
        ( h,  h, -h), (-h,  h, -h), (-h,  h,  h), ( h,  h,  h),
        # -Y 면
        (-h, -h, -h), ( h, -h, -h), ( h, -h,  h), (-h, -h,  h),
        # +Z 면
        (-h, -h,  h), ( h, -h,  h), ( h,  h,  h), (-h,  h,  h),
        # -Z 면
        ( h, -h, -h), (-h, -h, -h), (-h,  h, -h), ( h,  h, -h),
    ]
    items = ", ".join(f"({x}, {y}, {z})" for x, y, z in pts)
    return f"point3f[] points = [{items}]"


def _box_normals_str() -> str:
    """24점 법선 USDA 문자열 반환 (각 면 4점 동일 법선)."""
    face_normals = [
        ( 1,  0,  0),
        (-1,  0,  0),
        ( 0,  1,  0),
        ( 0, -1,  0),
        ( 0,  0,  1),
        ( 0,  0, -1),
    ]
    normals = [n for n in face_normals for _ in range(4)]
    items = ", ".join(f"({nx}, {ny}, {nz})" for nx, ny, nz in normals)
    return (
        "normal3f[] normals = [\n"
        f"        {items}\n"
        "    ] (\n"
        '        interpolation = "vertex"\n'
        "    )"
    )


def _box_uvs_str() -> str:
    """24점 UV USDA 문자열 반환 (각 면 전체 텍스처 매핑)."""
    uv_face = [(0, 0), (1, 0), (1, 1), (0, 1)]
    uvs = uv_face * 6
    items = ", ".join(f"({u}, {v})" for u, v in uvs)
    return (
        "texCoord2f[] primvars:st = [\n"
        f"        {items}\n"
        "    ] (\n"
        '        interpolation = "faceVarying"\n'
        "    )"
    )


def build_usda(root_name: str, aruco_id: int, color: tuple, size: float = 0.05) -> str:
    h  = size / 2.0
    r, g, b = color
    tex_path = f"{_ARUCO_DIR}/aruco_id{aruco_id}.png"
    mat_root = f"</{root_name}/Materials/Mat>"

    points_str  = _box_points_str(h)
    normals_str = _box_normals_str()
    uvs_str     = _box_uvs_str()

    return f"""\
#usda 1.0
(
    defaultPrim = "{root_name}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root_name}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    bool physics:rigidBodyEnabled = 1
    float physics:mass = 0.1

    def Mesh "Mesh" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
        {points_str}
        {normals_str}
        {uvs_str}
        bool physics:collisionEnabled = 1
        uniform token physics:approximation = "convexHull"
        rel material:binding = {mat_root}
    }}

    def Scope "Materials"
    {{
        def Material "Mat"
        {{
            token outputs:surface.connect = </{root_name}/Materials/Mat/Shader.outputs:surface>

            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor.connect = </{root_name}/Materials/Mat/DiffuseTexture.outputs:rgb>
                float inputs:roughness = 0.8
                float inputs:metallic = 0
                token outputs:surface
            }}

            def Shader "UVReader"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                token inputs:varname = "st"
                float2 outputs:result
            }}

            def Shader "DiffuseTexture"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @{tex_path}@
                float2 inputs:st.connect = </{root_name}/Materials/Mat/UVReader.outputs:result>
                token inputs:wrapS = "clamp"
                token inputs:wrapT = "clamp"
                float4 inputs:scale = ({r}, {g}, {b}, 1)
                float3 outputs:rgb
            }}
        }}
    }}
}}
"""


def main():
    _USD_DIR.mkdir(parents=True, exist_ok=True)

    for filename, root_name, aruco_id, color in _BOX_CONFIGS:
        content  = build_usda(root_name, aruco_id, color)
        out_path = _USD_DIR / filename
        out_path.write_text(content, encoding="utf-8")
        print(f"[generate_usda] 저장 완료 → {out_path}")

    print(f"\n3개 USDA 파일 생성 완료: {_USD_DIR}")


if __name__ == "__main__":
    main()
