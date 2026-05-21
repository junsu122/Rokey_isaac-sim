"""
Isaac Sim 씬 설정 스크립트.
ArUco 마커 텍스처가 붙은 박스들을 씬에 배치합니다.

실행:
    # Isaac Sim Python 환경에서
    python scene_setup.py

    # 마커 PNG만 먼저 생성 (Isaac Sim 없이)
    python scene_setup.py --generate-only
"""

import argparse
import sys
import os
from pathlib import Path
import numpy as np
import cv2
import yaml

ROOT = Path(__file__).parent
MARKER_DIR = ROOT / "markers"
CONFIG_PATH = ROOT / "config" / "object_registry.yaml"


# ──────────────────────────────────────────────
# Step 1: ArUco 마커 PNG 생성
# ──────────────────────────────────────────────

def generate_all_markers(cfg: dict) -> dict[int, Path]:
    """
    ArUco 마커 PNG 생성.
    역할(role)별로 테두리 색상과 크기를 다르게 표시합니다.

      item        → 초록 테두리  (박스용)
      section     → 주황 테두리  (구획용, 크게)
      destination → 파랑 테두리  (배송지용, 더 크게)
    """
    MARKER_DIR.mkdir(exist_ok=True)
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, cfg["aruco"]["dictionary"])
    )

    # 역할별 설정 (테두리 색 BGR, 캔버스 크기)
    role_style = {
        "item":        {"border_color": (0, 180, 0),   "size": 400, "border": 40},
        "section":     {"border_color": (0, 120, 255),  "size": 500, "border": 50},
        "destination": {"border_color": (200, 50,  0),  "size": 600, "border": 60},
    }

    paths: dict[int, Path] = {}
    for marker_id, info in cfg["markers"].items():
        marker_id = int(marker_id)
        role  = info.get("role", "item")
        label = info["label"]
        style = role_style.get(role, role_style["item"])

        size_px = style["size"]
        border  = style["border"]
        color   = style["border_color"]

        # 마커 이미지 생성
        inner = size_px - 2 * border
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, inner)

        # 흰 배경 캔버스 (컬러)
        canvas = np.ones((size_px, size_px, 3), dtype=np.uint8) * 255
        canvas[border:border + inner, border:border + inner] = cv2.cvtColor(
            marker_img, cv2.COLOR_GRAY2BGR)

        # 역할별 테두리
        cv2.rectangle(canvas, (2, 2), (size_px - 3, size_px - 3), color, 6)

        # 상단: ID + role 태그
        top_text = f"ID:{marker_id}  [{role.upper()}]"
        cv2.putText(canvas, top_text, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 하단: label
        cv2.putText(canvas, label, (10, size_px - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2)

        fname = MARKER_DIR / f"marker_{marker_id}_{label}.png"
        cv2.imwrite(str(fname), canvas)
        paths[marker_id] = fname
        print(f"  [OK] {fname.name:<45}  role={role}")

    return paths


# ──────────────────────────────────────────────
# Step 2: Isaac Sim 씬 구성
# ──────────────────────────────────────────────

def create_aruco_box(stage, world, marker_id: int, obj_info: dict,
                     texture_path: Path, position: np.ndarray):
    """
    ArUco 마커 텍스처가 적용된 박스를 씬에 추가합니다.

    Parameters
    ----------
    stage       : USD stage (omni.usd.get_context().get_stage())
    world       : omni.isaac.core.World 인스턴스
    marker_id   : ArUco 마커 ID
    obj_info    : config의 물체 정보 dict
    texture_path: 마커 PNG 절대 경로
    position    : 월드 좌표 [x, y, z]
    """
    from pxr import UsdGeom, UsdShade, Sdf, Gf
    import omni.kit.commands
    from omni.isaac.core.objects import DynamicCuboid

    prim_path = f"/World/ArUcoBox_{marker_id}"
    size = obj_info.get("size", [0.1, 0.1, 0.1])

    # 박스 생성
    cuboid = world.scene.add(DynamicCuboid(
        prim_path=prim_path,
        name=obj_info["name"],
        position=position,
        scale=np.array(size, dtype=np.float32),
        color=np.array([1.0, 1.0, 1.0]),  # 흰색 → 텍스처로 덮임
    ))

    # OmniPBR 머티리얼 생성
    mat_path = f"/World/Looks/ArUcoMat_{marker_id}"
    omni.kit.commands.execute(
        "CreateMdlMaterialPrimCommand",
        mtl_url="OmniPBR.mdl",
        mtl_name="OmniPBR",
        mtl_path=mat_path,
    )

    # 텍스처(마커 PNG) 적용
    shader_path = f"{mat_path}/Shader"
    shader_prim = stage.GetPrimAtPath(shader_path)
    shader = UsdShade.Shader(shader_prim)

    tex_asset = Sdf.AssetPath(str(texture_path))
    shader.GetInput("diffuse_texture").Set(tex_asset)
    # 텍스처가 타일링되지 않게 (마커 1장이 면 전체를 덮도록)
    shader.GetInput("texture_scale").Set(Gf.Vec2f(1.0, 1.0))

    # 머티리얼 → 박스 바인딩
    box_prim = stage.GetPrimAtPath(prim_path)
    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI(box_prim).Bind(material)

    print(f"  [씬 추가] {obj_info['name']}  위치={position.tolist()}  마커ID={marker_id}")
    return cuboid


def setup_camera(stage, cfg: dict, camera_path: str = "/World/Camera"):
    """Isaac Sim 카메라 프림을 생성하고 내부 파라미터를 설정합니다."""
    from pxr import UsdGeom, Gf
    import omni.kit.commands

    omni.kit.commands.execute(
        "CreatePrimWithDefaultXformCommand",
        prim_type="Camera",
        prim_path=camera_path,
    )

    cam_prim = stage.GetPrimAtPath(camera_path)
    cam = UsdGeom.Camera(cam_prim)

    c = cfg["camera"]
    # focal length: fx / (width/sensor_width) — Isaac Sim 기본 센서 폭 20.955mm
    sensor_width_mm = 20.955
    fl_mm = c["fx"] * sensor_width_mm / c["width"]
    cam.GetFocalLengthAttr().Set(fl_mm)
    cam.GetHorizontalApertureAttr().Set(sensor_width_mm)

    # 카메라 위치: 박스들을 내려다보는 위치
    xform = UsdGeom.Xformable(cam_prim)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(0.0, -0.5, 0.8))   # x=0, y=-0.5m, z=0.8m
    rx_op = xform.AddRotateXYZOp()
    rx_op.Set(Gf.Vec3f(-45.0, 0.0, 0.0))  # 45° 내려보기

    print(f"  [카메라] {camera_path}  focal_length={fl_mm:.2f}mm")
    return cam_prim


def run_isaac_scene(cfg: dict, texture_paths: dict[int, Path]):
    """Isaac Sim 환경을 초기화하고 씬을 구성합니다."""
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

    import omni.usd
    from omni.isaac.core import World

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    # 카메라 추가
    setup_camera(stage, cfg)

    # 물체 배치 (x 방향으로 0.25m 간격)
    for idx, (marker_id, obj_info) in enumerate(cfg["objects"].items()):
        marker_id = int(marker_id)
        pos = np.array([(idx - len(cfg["objects"]) / 2) * 0.25, 0.0, 0.05])
        create_aruco_box(
            stage, world,
            marker_id=marker_id,
            obj_info=obj_info,
            texture_path=texture_paths[marker_id],
            position=pos,
        )

    # 시뮬레이션 루프
    world.reset()
    print("\n[INFO] 씬 구성 완료. 시뮬레이션 실행 중... Ctrl+C로 종료")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    finally:
        simulation_app.close()


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Isaac Sim ArUco 박스 씬 설정")
    parser.add_argument("--generate-only", action="store_true",
                        help="마커 PNG만 생성하고 Isaac Sim은 실행하지 않음")
    args = parser.parse_args()

    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    print("=== ArUco 마커 이미지 생성 ===")
    texture_paths = generate_all_markers(cfg)

    if args.generate_only:
        print(f"\n완료. 마커 이미지가 {MARKER_DIR}/ 에 저장되었습니다.")
        return

    print("\n=== Isaac Sim 씬 구성 ===")
    run_isaac_scene(cfg, texture_paths)


if __name__ == "__main__":
    main()
