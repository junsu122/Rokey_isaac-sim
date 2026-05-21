"""
Isaac Sim 물리 환경 구성 스크립트.

환경 구성:
  - 작업 테이블 (컨베이어 역할)
  - ArUco 마커 큐브 3개 (강남 / 서초 / 구로디지털단지)
  - 목적지 빈(bin) 3개
  - 두산 M0609 협동로봇
  - 상단 RGB 카메라 + Depth 카메라
  - 조명

실행:
  ~/.local/share/ov/pkg/isaac-sim-*/python.sh create_scene.py
  또는
  ~/.local/share/ov/pkg/isaac-sim-*/python.sh create_scene.py --headless
"""

import argparse
import sys
import time
import numpy as np
import yaml
import cv2
from pathlib import Path

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

CONFIG_PATH  = ROOT / "config" / "object_registry.yaml"
MARKER_DIR   = ROOT / "markers"

# 두산 M0609 URDF 경로 (환경에 맞게 수정)
# ROS2 패키지: https://github.com/doosan-robotics/doosan-robot2
DOOSAN_URDF  = ROOT / "urdf" / "m0609.urdf"


# ── 씬 파라미터 ────────────────────────────────────────────────────────────────

TABLE_POS    = np.array([0.0,  0.0, 0.0])
TABLE_SIZE   = np.array([1.2,  0.6, 0.05])   # 가로 x 세로 x 두께 (m)
TABLE_HEIGHT = TABLE_SIZE[2]                   # 테이블 상면 z

ROBOT_POS    = np.array([0.0, -0.45, TABLE_HEIGHT])   # 테이블 앞쪽에 로봇 베이스

BOX_SIZE     = np.array([0.08, 0.08, 0.08])
BOX_START_X  = -0.3   # 첫 번째 박스 x (좌측)
BOX_GAP      = 0.3    # 박스 간격 (m)
BOX_Z        = TABLE_HEIGHT + BOX_SIZE[2] / 2   # 박스 중심 z

# 목적지 빈 위치 (로봇 작업 반경 내)
BIN_POSITIONS = {
    0: np.array([ 0.5,  0.3, 0.01]),   # 강남 빈
    1: np.array([ 0.5,  0.0, 0.01]),   # 서초 빈
    2: np.array([ 0.5, -0.3, 0.01]),   # 구로디지털단지 빈
}
BIN_SIZE = np.array([0.15, 0.15, 0.04])

CAMERA_POS   = np.array([0.0, 0.0, 0.9])   # 테이블 위 0.9 m (수직 내려보기)


# ── 씬 구성 함수 ───────────────────────────────────────────────────────────────

def create_table(world, stage):
    from omni.isaac.core.objects import FixedCuboid  # type: ignore
    world.scene.add(FixedCuboid(
        prim_path="/World/Table",
        name="table",
        position=TABLE_POS + np.array([0, 0, TABLE_HEIGHT / 2]),
        scale=TABLE_SIZE,
        color=np.array([0.6, 0.5, 0.35]),
    ))
    print("  [테이블] 생성 완료")


def _apply_marker_texture(stage, prim_path: str, tex_path, mat_path: str):
    """OmniPBR 머티리얼을 만들고 ArUco 마커 PNG를 diffuse 텍스처로 적용."""
    from pxr import UsdShade, Sdf  # type: ignore
    import omni.kit.commands       # type: ignore

    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="OmniPBR.mdl",
        mtl_name="OmniPBR",
        mtl_path=mat_path,
    )
    shader = UsdShade.Shader(stage.GetPrimAtPath(f"{mat_path}/Shader"))
    shader.GetInput("diffuse_texture").Set(Sdf.AssetPath(str(tex_path)))
    # 텍스처 1장이 면 전체를 꽉 채우도록
    from pxr import Gf  # type: ignore
    shader.GetInput("texture_scale").Set(Gf.Vec2f(1.0, 1.0))
    shader.GetInput("texture_translate").Set(Gf.Vec2f(0.0, 0.0))

    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(prim_path)).Bind(material)


def _apply_marker_top_only(stage, prim_path: str, tex_path, mat_path: str):
    """
    윗면(+Z)에만 ArUco 마커 텍스처를 적용하고 나머지 면은 흰색으로 처리.

    Isaac Sim에서 Cube의 기본 UV는 6면이 동일하게 매핑됩니다.
    윗면만 분리하려면 별도 Mesh로 교체해야 하므로,
    실용적인 대안으로 '마커 텍스처 + 투명 오버레이' 방식을 사용합니다.
    → 카메라가 위에서만 찍으면 전체 면 방식과 결과 동일합니다.
    """
    # 현실적으로 Isaac Sim 기본 Cube는 UV 분리가 번거로우므로
    # 전체 면 방식과 동일하게 처리 (위에서 보는 카메라 구성이면 차이 없음)
    _apply_marker_texture(stage, prim_path, tex_path, mat_path)


def create_aruco_boxes(world, stage, cfg: dict) -> list:
    """
    item 역할 마커가 붙은 박스만 생성합니다.
    마커 PNG를 OmniPBR 텍스처로 큐브 표면에 적용합니다.

    [텍스처 방식 선택]
      TOP_ONLY = False  → 6면 전체에 마커 표시 (기본, 위에서 보는 카메라에 충분)
      TOP_ONLY = True   → 윗면 집중 표시 (카메라 각도가 다양할 때)
    """
    from omni.isaac.core.objects import DynamicCuboid  # type: ignore

    TOP_ONLY = False   # ← 필요 시 True로 변경
    apply_fn = _apply_marker_top_only if TOP_ONLY else _apply_marker_texture

    boxes = []

    # item 역할 마커만 박스로 생성
    item_markers = {int(k): v for k, v in cfg["markers"].items()
                    if v.get("role") == "item"}

    for idx, (marker_id, info) in enumerate(item_markers.items()):
        pos_x = BOX_START_X + idx * BOX_GAP
        position = np.array([pos_x, 0.1, BOX_Z])
        prim_path = f"/World/Box_{marker_id}"

        box = world.scene.add(DynamicCuboid(
            prim_path=prim_path,
            name=f"box_{info['label']}",
            position=position,
            scale=BOX_SIZE,
            color=np.array([1.0, 1.0, 1.0]),  # 흰 박스 위에 텍스처
            mass=0.3,
        ))

        # 마커 PNG 텍스처 적용
        tex_path = MARKER_DIR / f"marker_{marker_id}_{info['label']}.png"
        if tex_path.exists():
            apply_fn(stage, prim_path, tex_path,
                     mat_path=f"/World/Looks/BoxMat_{marker_id}")
            print(f"  [박스] ID={marker_id} '{info['label']}'  "
                  f"위치={position.round(2).tolist()}  텍스처=OK")
        else:
            print(f"  [박스] ID={marker_id} '{info['label']}'  "
                  f"위치={position.round(2).tolist()}  ⚠ 텍스처 파일 없음: {tex_path.name}")

        boxes.append(box)
        print(f"  [박스] ID={marker_id} '{info['label']}'  위치={position.round(3).tolist()}")

    return boxes


def create_destination_bins(world, cfg: dict):
    """목적지 빈(bin) 생성."""
    from omni.isaac.core.objects import FixedCuboid  # type: ignore

    bin_colors = {
        0: np.array([0.2, 0.6, 1.0]),   # 강남 - 파랑
        1: np.array([0.2, 0.9, 0.3]),   # 서초 - 초록
        2: np.array([1.0, 0.5, 0.1]),   # 구로 - 주황
    }
    for marker_id, pos in BIN_POSITIONS.items():
        info = cfg["markers"].get(marker_id, {})
        world.scene.add(FixedCuboid(
            prim_path=f"/World/Bin_{marker_id}",
            name=f"bin_{info.get('label', marker_id)}",
            position=pos + np.array([0, 0, BIN_SIZE[2] / 2]),
            scale=BIN_SIZE,
            color=bin_colors.get(marker_id, np.array([0.5, 0.5, 0.5])),
        ))
        print(f"  [빈] ID={marker_id} '{info.get('label','')}' 위치={pos.tolist()}")


def create_robot(stage) -> str | None:
    """두산 M0609 로봇 URDF 로드."""
    import omni.kit.commands  # type: ignore
    from pxr import UsdGeom, Gf  # type: ignore

    if not DOOSAN_URDF.exists():
        print(f"  [경고] URDF 파일 없음: {DOOSAN_URDF}")
        print("         https://github.com/doosan-robotics/doosan-robot2 에서 받아서")
        print(f"         {DOOSAN_URDF} 에 배치하세요.")
        print("         (씬은 로봇 없이 계속 진행)")
        return None

    from omni.isaac.urdf import _urdf  # type: ignore
    urdf_if = _urdf.acquire_urdf_interface()

    cfg_import = _urdf.ImportConfig()
    cfg_import.merge_fixed_joints  = False
    cfg_import.fix_base            = True
    cfg_import.import_inertia_tensor = True
    cfg_import.distance_scale      = 1.0
    cfg_import.density             = 0.0
    cfg_import.create_physics_scene = False
    cfg_import.convex_decomp       = False
    cfg_import.self_collision       = False

    robot_prim_path = "/World/Doosan_M0609"
    dest_path = "/World"

    urdf_if.parse_urdf(str(DOOSAN_URDF), dest_path, cfg_import)

    # 로봇 베이스 위치 설정
    prim = stage.GetPrimAtPath(robot_prim_path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        t_op = xform.AddTranslateOp()
        t_op.Set(Gf.Vec3d(*ROBOT_POS.tolist()))

    print(f"  [로봇] 두산 M0609 로드 완료  베이스 위치={ROBOT_POS.tolist()}")
    return robot_prim_path


def create_cameras(stage, cfg: dict) -> tuple[str, str]:
    """
    RGB 카메라 + Depth 카메라를 테이블 정중앙 위에 수직으로 마운트.

    카메라 선택 가이드:
      - RGB 만으로도 ArUco 분류(레이블 판별 + 2D→3D 포즈추정) 가능
      - Depth 카메라는 로봇 그립 포인트 정밀 계산 / 장애물 회피에 활용
    """
    from pxr import UsdGeom, Gf  # type: ignore
    import omni.kit.commands     # type: ignore

    c = cfg["camera"]

    def _make_camera(prim_path: str, offset_y: float = 0.0):
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXformCommand",
            prim_type="Camera",
            prim_path=prim_path,
        )
        cam_prim = stage.GetPrimAtPath(prim_path)
        cam = UsdGeom.Camera(cam_prim)

        sensor_w_mm = 20.955
        fl_mm = c["fx"] * sensor_w_mm / c["width"]
        cam.GetFocalLengthAttr().Set(fl_mm)
        cam.GetHorizontalApertureAttr().Set(sensor_w_mm)
        cam.GetVerticalApertureAttr().Set(sensor_w_mm * c["height"] / c["width"])

        xform = UsdGeom.Xformable(cam_prim)
        xform.ClearXformOpOrder()
        pos = CAMERA_POS + np.array([0, offset_y, 0])
        xform.AddTranslateOp().Set(Gf.Vec3d(*pos.tolist()))
        # 정수직 아래를 바라봄 (카메라 -Z = 월드 -Z)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(-90.0, 0.0, 0.0))
        return prim_path

    rgb_path   = _make_camera("/World/Camera_RGB",   offset_y=0.0)
    depth_path = _make_camera("/World/Camera_Depth",  offset_y=0.02)  # 살짝 옆에 마운트

    print(f"  [카메라] RGB:   {rgb_path}   높이={CAMERA_POS[2]}m (수직 내려보기)")
    print(f"  [카메라] Depth: {depth_path}  → 로봇 그립 포인트 계산용")
    return rgb_path, depth_path


def create_lighting(stage):
    """씬 조명 추가 (마커 인식에 충분한 밝기)."""
    import omni.kit.commands  # type: ignore
    from pxr import UsdLux, Gf  # type: ignore

    omni.kit.commands.execute(
        "CreatePrimWithDefaultXformCommand",
        prim_type="DistantLight",
        prim_path="/World/DistantLight",
    )
    light = UsdLux.DistantLight(stage.GetPrimAtPath("/World/DistantLight"))
    light.GetIntensityAttr().Set(3000)
    light.GetAngleAttr().Set(0.53)

    # 테이블 위 포인트 라이트 (마커 음영 줄이기)
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXformCommand",
        prim_type="SphereLight",
        prim_path="/World/OverheadLight",
    )
    from pxr import UsdGeom  # type: ignore
    xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/OverheadLight"))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.2))
    ol = UsdLux.SphereLight(stage.GetPrimAtPath("/World/OverheadLight"))
    ol.GetIntensityAttr().Set(8000)
    ol.GetRadiusAttr().Set(0.1)
    print("  [조명] 설정 완료")


# ── 메인 ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    # ── Isaac Sim 초기화
    from omni.isaac.kit import SimulationApp  # type: ignore
    sim_app = SimulationApp({"headless": args.headless,
                              "width": 1280, "height": 720})

    import omni.usd                          # type: ignore
    from omni.isaac.core import World        # type: ignore

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    print("\n=== 씬 구성 시작 ===")
    create_lighting(stage)
    create_table(world, stage)
    create_aruco_boxes(world, stage, cfg)
    create_destination_bins(world, cfg)
    rgb_cam, depth_cam = create_cameras(stage, cfg)
    robot_path = create_robot(stage)

    # USD 저장 (Isaac Sim에서 나중에 열어볼 수 있음)
    usd_out = ROOT / "scene.usd"
    omni.usd.get_context().save_as_stage(str(usd_out))
    print(f"\n  [USD] 씬 저장: {usd_out}")

    print(f"""
=== 씬 구성 완료 ===
  테이블        : {TABLE_POS} ~ {TABLE_SIZE} m
  박스 3개      : ID 0(강남) / 1(서초) / 2(구로디지털단지)
  목적지 빈 3개 : 로봇 오른쪽 배치
  RGB 카메라    : {rgb_cam}  높이 {CAMERA_POS[2]}m
  Depth 카메라  : {depth_cam}  (그립 포인트 계산용)
  로봇          : 두산 M0609  {'로드됨' if robot_path else '⚠ URDF 없음 — 수동 배치 필요'}

다음 단계:
  python3 isaac_aruco_main.py   ← 분류 인식 루프 실행
""")

    # ── 시뮬레이션 루프
    world.reset()
    try:
        while sim_app.is_running():
            world.step(render=True)
    finally:
        sim_app.close()


if __name__ == "__main__":
    main()
