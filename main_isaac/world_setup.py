"""
main_isaac/world_setup.py
=========================
창고 씬 공통 설정 (맵 로드 + 조명 + Pod Stack 스폰).

ArUco 박스는 BoxSpawner 가 20초 간격으로 컨베이어 위에 동적 스폰한다.
섹션 A/B/C 의 슬롯 #01 은 IW Hub 배달 예약 슬롯으로 비워둔다.
각 섹션 슬롯 7~9 (3개) 에 드론 픽업용 ArUco 박스가 사전 배치된다.
"""
import omni.usd
from pxr import Sdf, Gf, UsdGeom, UsdPhysics

from robot_config import (
    WAREHOUSE_USD, POD_STACKS, SECTION_PODS, SECTION_POD_USD,
    ARUCO_BOXES, RENDERING_DT,
)

# 드론 픽업용 박스 슬롯: 9-slot section 마지막 행 슬롯 7~9
_SECTION_BOX_SLOTS = [7, 8, 9]
# 적층 박스 Z 위치 (선반 top=0.265m, 박스 높이=0.30m):
#   하단: 0.265+0.15=0.415m  중단: 0.715m  상단: 1.015m (케이지 개구부 위로 10cm 돌출)
_SECTION_BOX_Z_STACK = [0.415, 0.715, 1.015]
# 박스 크기: 컨트롤 센터 "Spawn Box Here" 와 동일 (30cm 정육면체)
_SECTION_BOX_SIZE = (0.30, 0.30, 0.30)
# 섹션별 ArUco ID / 색상: M0609 각 암 담당 타입과 동일
_SECTION_BOX_INFO = {
    "A": {"aruco_id": 2, "color": (0.2, 0.4, 0.9)},   # blue_id2
    "B": {"aruco_id": 1, "color": (0.8, 0.2, 0.2)},   # red_id1
    "C": {"aruco_id": 0, "color": (0.2, 0.8, 0.2)},   # green_id0
}


def _define_prim_with_ref(stage, prim_path: str, usd_ref: str):
    """stage.DefinePrim + AddReference 래퍼."""
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(usd_ref)
    return prim


def setup_warehouse(world) -> None:
    """창고 USD 로드 + 조명 + Pod Stack 스폰 + 섹션 박스 사전 배치."""
    stage = omni.usd.get_context().get_stage()

    # 창고 맵
    wh = stage.DefinePrim("/World/Warehouse", "Xform")
    wh.GetReferences().AddReference(WAREHOUSE_USD)

    # 전체 조명
    light = stage.DefinePrim("/World/DistantLight", "DistantLight")
    light.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
    light.CreateAttribute("angle",     Sdf.ValueTypeNames.Float).Set(0.53)

    print(f"[WorldSetup] 창고 맵 로드 완료: {WAREHOUSE_USD}")

    # Pod Stack 루트 컨테이너
    if not stage.GetPrimAtPath("/World/PodStacks").IsValid():
        UsdGeom.Xform.Define(stage, "/World/PodStacks")

    # 컨베이어 옆 고정 Pod Stack (PodStack_01~04)
    for pod in POD_STACKS:
        prim_path = f"/World/PodStacks/{pod['name']}"
        prim = _define_prim_with_ref(stage, prim_path, pod["usd"])

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pod["xyz"]))
        yaw = float(pod.get("yaw", 0.0))
        if abs(yaw) > 1e-6:
            xf.AddRotateZOp().Set(yaw)

        print(f"[WorldSetup] Pod Stack 로드: {pod['name']}  pos={pod['xyz']}")

    # Section A / B / C  12-slot 격자 — 슬롯 #01 은 IW Hub 배달용으로 비워둠
    for sec_name, positions in SECTION_PODS.items():
        count = 0
        for i, xyz in enumerate(positions, start=1):
            if i == 1:
                continue   # 슬롯 01: IW Hub 배달 예약 위치 — 비워 둠
            prim_path = f"/World/PodStacks/Sec_{sec_name}_{i:02d}"
            prim = _define_prim_with_ref(stage, prim_path, SECTION_POD_USD)
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            count += 1

        print(f"[WorldSetup] Section {sec_name} Pod Stack {count}개 스폰 완료 "
              f"(12 슬롯 중 슬롯 01 비워둠)")

    # 섹션 박스 사전 배치 (드론 픽업용: 슬롯 10/15/20)
    _spawn_section_boxes(stage)


def _spawn_section_boxes(stage) -> None:
    """섹션 A/B/C 의 슬롯 10~12 각 pod 에 30cm ArUco 박스 3개 적층 배치.

    적층 구조 (하→상):
      _b1 : z=0.415m (선반 위)
      _b2 : z=0.715m (중단)
      ''  : z=1.015m (상단, 드론 픽업 대상 — 케이지 개구부 위로 10cm 돌출)
    모두 Kinematic=True: 드론이 그랩할 때까지 위치 고정.
    """
    from robot_config import SECTION_PODS
    from auto_spawn_panel import _create_box_with_aruco

    if not stage.GetPrimAtPath("/World/SectionBoxes").IsValid():
        UsdGeom.Xform.Define(stage, "/World/SectionBoxes")

    bw, bd, bh = _SECTION_BOX_SIZE
    # 이름 suffix: 하단(_b1), 중단(_b2), 상단(드론 픽업용, 접미사 없음)
    suffixes = ["_b1", "_b2", ""]
    spawned = 0

    for sec, positions in SECTION_PODS.items():
        info = _SECTION_BOX_INFO.get(sec)
        if info is None:
            continue
        aruco_id  = info["aruco_id"]
        color_rgb = info["color"]

        for slot in _SECTION_BOX_SLOTS:
            pos_idx = slot - 1      # 0-based: slot 10 → index 9
            if pos_idx >= len(positions):
                continue

            x_m = float(positions[pos_idx][0])
            y_m = float(positions[pos_idx][1])

            for z_m, suffix in zip(_SECTION_BOX_Z_STACK, suffixes):
                prim_path = f"/World/SectionBoxes/Sec_{sec}_{slot:02d}{suffix}"
                _create_box_with_aruco(
                    prim_path, x_m, y_m, z_m,
                    bw, bd, bh,
                    color_rgb=color_rgb,
                    mass=2.0,
                    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                    aruco_id=aruco_id,
                )
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid():
                    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
                spawned += 1

    print(f"[WorldSetup] 섹션 박스 {spawned}개 사전 배치 완료 "
          f"(슬롯 10~12, 3개 적층×섹션 A/B/C, 30cm ArUco)")


# ══════════════════════════════════════════════════════════════════════
#  BoxSpawner — 컨베이어 위에 20초마다 ArUco 박스 동적 스폰
# ══════════════════════════════════════════════════════════════════════

class BoxSpawner:
    """
    렌더 루프에서 update() 를 매 step 호출하면
    20초 간격으로 컨베이어(-16, 0) 위에 ArUco 박스를 순서대로 스폰한다.
    green(id0) → red(id1) → blue(id2) → green ... 무한 반복.
    """

    _SPAWN_XYZ = (-16.0, 0.0, 1.0)
    _BOX_TYPES = ["green_id0", "red_id1", "blue_id2"]
    _INTERVAL_SEC = 20.0

    def __init__(self, rendering_dt: float = RENDERING_DT) -> None:
        # 첫 스폰은 즉시(스텝 0), 이후 매 interval 마다
        self._interval = max(1, round(self._INTERVAL_SEC / rendering_dt))
        self._step     = self._interval - 1   # 첫 update() 에서 즉시 스폰
        self._type_idx = 0
        self._count    = 0
        print(f"[BoxSpawner] 초기화 — {self._INTERVAL_SEC}초마다 스폰 "
              f"({self._interval} 렌더 스텝)")

    def update(self) -> None:
        self._step += 1
        if self._step % self._interval != 0:
            return
        self._spawn_next()

    def _spawn_next(self) -> None:
        import traceback
        box_type = self._BOX_TYPES[self._type_idx % 3]
        self._type_idx += 1
        self._count    += 1

        usd_path = None
        for b in ARUCO_BOXES:
            if b["type"] == box_type:
                usd_path = b["usd"]
                break
        if usd_path is None:
            print(f"[BoxSpawner] USD 없음: {box_type}")
            return

        try:
            stage     = omni.usd.get_context().get_stage()
            prim_path = f"/World/DynamicBoxes/{box_type}_{self._count:04d}"

            if not stage.GetPrimAtPath("/World/DynamicBoxes").IsValid():
                UsdGeom.Xform.Define(stage, "/World/DynamicBoxes")

            prim = stage.DefinePrim(prim_path, "Xform")
            prim.GetReferences().AddReference(usd_path)

            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*self._SPAWN_XYZ))

            print(f"[BoxSpawner] 스폰: {prim_path}  type={box_type}")
        except Exception:
            print(f"[BoxSpawner] 스폰 오류 (type={box_type}):")
            traceback.print_exc()
