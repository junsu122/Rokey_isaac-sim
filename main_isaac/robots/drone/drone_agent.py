"""
main_isaac/robots/drone/drone_agent.py
=======================================
Iris 쿼드로터 에이전트.

깊이 카메라 / 히트맵 제거 → 성능 최대화.
출력: 전방 RGB 뷰포트 + 경량 상태 HUD (텍스트 라벨만).

제어:
  키보드: T=이륙  L=착지  W/S=전후  A/D=좌우  Q/E=요  ↑↓=고도
  조이스틱: 스틱 조작시 키보드보다 우선
  HUD Go/Land 버튼으로 목표 좌표 수동 입력 가능

자율 미션 (robot_config.py 에서 section_targets 지정 시 자동 실행):
  각 섹션의 지정 슬롯 포드를 집어 delivery_xyz 로 운반.
  예) "section_targets": {"A": 3, "B": 2, "C": 3}
      "delivery_xyz"   : (12.0, 14.0, 0.0)

robot_config.py 에서 등록:
    {
        "type"            : "drone",
        "name"            : "Drone_01",
        "spawn_xyz"       : (-16.0, -16.0, 0.07),
        "takeoff_alt"     : 2.5,
        "section_targets" : {"A": 3, "B": 2, "C": 3},
        "delivery_xyz"    : (12.0, 14.0, 0.0),
    }
"""
import sys
import os
import math
from pathlib import Path

import numpy as np
import carb
import omni.usd
from pxr import UsdGeom, Gf, Usd, UsdPhysics, Sdf
from scipy.spatial.transform import Rotation

import robot_config as C
from ..base_robot import BaseRobotAgent

# ── Pegasus Simulator ─────────────────────────────────────────────────────
if C.PEGASUS_SIM_DIR not in sys.path:
    sys.path.insert(0, C.PEGASUS_SIM_DIR)

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# ── drone_deps ───────────────────────────────────────────────────────────
if C.DRONE_DEPS_DIR not in sys.path:
    sys.path.insert(0, C.DRONE_DEPS_DIR)

from drone_config    import CAM_FOCAL_LENGTH, CAM_MOUNT_FWD, HUD_UPDATE_N
from controller      import HybridController
from hud             import MinimalDroneHUD
from fast_multirotor import FastMultirotor

# ── 미션 FSM 상수 ─────────────────────────────────────────────────────────
_MS_IDLE         = "IDLE"
_MS_TAKEOFF      = "TAKEOFF"
_MS_FLY_PICK     = "FLY_PICK"
_MS_DESCEND_PICK = "DESCEND_PICK"
_MS_GRAB         = "GRAB"
_MS_ASCEND_PICK  = "ASCEND_PICK"
_MS_FLY_DROP     = "FLY_DROP"
_MS_DESCEND_DROP = "DESCEND_DROP"
_MS_RELEASE      = "RELEASE"
_MS_REASCEND     = "REASCEND"   # 배달 후 다음 픽업 전 고도 회복
_MS_DONE         = "DONE"

_HOVER_ALT        = 2.5    # 순항 고도 [m] — ceiling 충돌 방지용 고정 미션 고도
_GRAB_ALT         = 1.5    # 그랩 시 드론 고도 [m] — 섹션 박스(케이지 내부 z≈0.91m) 위 호버링
_DROP_ALT         = 1.5    # 배달지 릴리즈 고도 [m] — 순항 고도에서 떨어뜨리지 않고 내려놓기
_NAV_TOL_XY       = 0.5    # 수평 도착 허용 오차 [m]
_NAV_TOL_Z        = 0.25   # 수직 도착 허용 오차 [m]
_ALT_GATE         = 0.12   # 수직→수평 전환 고도 허용 오차 [m] — 먼저 상승 후 이동
_ALT_STABLE_STEPS = 5      # 수평 이동 전 2.5m 유지 확인 steps
_SAFE_XY_MIN      = -14.0
_SAFE_XY_MAX      = 14.0
_GRAB_WAIT        = 40     # 그랩 위치 안정화 대기 (physics steps)
_DESCEND_TIMEOUT  = 3000   # DESCEND_PICK 최대 대기 steps (~6s@500Hz) — 타임아웃 시 스킵


class DroneAgent(BaseRobotAgent):
    """
    Iris 쿼드로터 에이전트.
    전방 RGB 뷰포트 + 경량 상태 HUD 만 사용 (깊이 카메라/히트맵 없음).
    선택적 자율 미션: 섹션 포드 픽업 → 배달지 운반.
    """

    WARMUP_STEPS = 200

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        self._step_count = 0
        self._hud_step   = 0
        self.controller  = None
        self._cam_path   = None
        self._front_cam  = None
        self.hud         = None

        # ── 미션 초기화 ──────────────────────────────────────────────
        self._mission_state   = _MS_IDLE
        self._mission_queue   = []          # list of target dicts
        self._mission_idx     = 0
        self._carried_prim    = None        # prim path being carried
        self._carry_offset    = np.zeros(3) # pod-world relative to drone-world
        self._grab_wait_cnt   = 0
        self._descend_cnt     = 0           # DESCEND_PICK 체류 카운터 (타임아웃용)
        self._descend_xy      = None        # DESCEND_PICK 진입 시 고정 XY (수직 하강 보장)
        self._ascend_xy       = None        # 상승 중 고정 XY (수평 이동 금지)
        self._alt_stable_cnt  = 0           # 2.5m 도달 후 안정화 카운터
        self._delivery_xyz    = np.array(self.cfg.get("delivery_xyz", [12.0, 14.0, 0.0]),
                                         dtype=np.float64)

        # section_targets → mission queue 빌드
        sec_targets = self.cfg.get("section_targets", {})
        if sec_targets:
            self._mission_queue = self._build_mission_queue(sec_targets)
            print(f"[{self.name}] 미션 큐: {len(self._mission_queue)}개 타겟 → "
                  f"배달지={self._delivery_xyz}")

        carb.log_warn(f"[{self.name}] setup 완료 — prim 은 post_reset 에서 생성")

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        spawn = self.spawn_xyz

        # PegasusInterface 에 공유 월드 연결
        pg        = PegasusInterface()
        pg._world = self.world

        iris_path = ROBOTS["Iris"]
        carb.log_warn(f"[{self.name}] ROBOTS['Iris'] = {iris_path}")
        carb.log_warn(f"[{self.name}] iris.usd exists = {os.path.isfile(iris_path)}")

        # 컨트롤러 생성
        self.controller             = HybridController()
        self.controller.takeoff_alt = float(self.cfg.get("takeoff_alt", _HOVER_ALT))

        # FastMultirotor: no sensors, no propeller animation
        drone_cfg          = MultirotorConfig()
        drone_cfg.backends = [self.controller]
        drone_cfg.sensors  = []

        try:
            FastMultirotor(
                f"/World/{self.name}",
                iris_path, 0,
                list(spawn),
                Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
                config=drone_cfg,
            )
            carb.log_warn(f"[{self.name}] Multirotor 생성 완료  spawn={spawn}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] Multirotor 생성 실패: {e}")
            return

        # 전방 RGB 카메라 prim 생성
        self._cam_path = f"/World/{self.name}/body/FrontCamera"
        stage          = omni.usd.get_context().get_stage()
        try:
            cam = UsdGeom.Camera.Define(stage, self._cam_path)
            cam.GetFocalLengthAttr().Set(CAM_FOCAL_LENGTH)
            xf = UsdGeom.Xformable(cam.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(CAM_MOUNT_FWD, 0.0, 0.0))
            xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))
        except Exception as e:
            carb.log_warn(f"[{self.name}] 카메라 prim 생성 실패: {e}")

        # 흡착 석커(sucker) 시각화 — 드론 동체 하단에 소형 진공 그리퍼 (비물리, 충돌 없음)
        # 구성: 칼라(body 마운트) → 짧은 파이프 → 흡착 컵 디스크
        sucker_path = f"/World/{self.name}/body/Sucker"
        try:
            from pxr import UsdShade, UsdPhysics

            def _make_mat(mat_path, r, g, b):
                mat  = UsdShade.Material.Define(stage, mat_path)
                shdr = UsdShade.Shader.Define(stage, mat_path + "/Shader")
                shdr.CreateIdAttr("UsdPreviewSurface")
                shdr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((r, g, b))
                shdr.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.5)
                mat.CreateSurfaceOutput().ConnectToSource(shdr.ConnectableAPI(), "surface")
                return mat

            gray_mat   = _make_mat(f"{sucker_path}/GrayMat",   0.22, 0.22, 0.22)
            orange_mat = _make_mat(f"{sucker_path}/OrangeMat", 0.90, 0.30, 0.00)

            # ── 칼라: body 마운트 플랜지 (z=-0.042 ~ -0.057) ────────
            col = UsdGeom.Cylinder.Define(stage, f"{sucker_path}/Collar")
            col.GetRadiusAttr().Set(0.020)
            col.GetHeightAttr().Set(0.015)
            col.GetAxisAttr().Set("Z")
            UsdGeom.Xformable(col.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(0.0, 0.0, -0.050))
            UsdShade.MaterialBindingAPI(col.GetPrim()).Bind(gray_mat)

            # ── 파이프: 칼라~컵 연결 (z=-0.058 ~ -0.128) ────────────
            arm = UsdGeom.Cylinder.Define(stage, f"{sucker_path}/Arm")
            arm.GetRadiusAttr().Set(0.008)
            arm.GetHeightAttr().Set(0.070)
            arm.GetAxisAttr().Set("Z")
            UsdGeom.Xformable(arm.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(0.0, 0.0, -0.093))
            UsdShade.MaterialBindingAPI(arm.GetPrim()).Bind(gray_mat)

            # ── 흡착 컵 디스크 (z=-0.128 ~ -0.140) ──────────────────
            cup = UsdGeom.Cylinder.Define(stage, f"{sucker_path}/Cup")
            cup.GetRadiusAttr().Set(0.042)
            cup.GetHeightAttr().Set(0.012)
            cup.GetAxisAttr().Set("Z")
            UsdGeom.Xformable(cup.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(0.0, 0.0, -0.134))
            UsdShade.MaterialBindingAPI(cup.GetPrim()).Bind(orange_mat)

            # 충돌 비활성화: 석커는 순수 시각 요소 — 바닥/벽 충돌 방지
            for part in [col.GetPrim(), arm.GetPrim(), cup.GetPrim()]:
                if part.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(part).GetCollisionEnabledAttr().Set(False)
                else:
                    UsdPhysics.CollisionAPI.Apply(part).GetCollisionEnabledAttr().Set(False)

            carb.log_warn(f"[{self.name}] 흡착 석커 prim 생성 완료 (충돌 비활성화): {sucker_path}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] 석커 prim 생성 실패: {e}")

        # 경량 HUD
        self.hud = MinimalDroneHUD(self.controller, build_window=False)

        # RGB 카메라 센서
        if self._cam_path:
            try:
                try:
                    from isaacsim.sensors.camera import Camera
                except ImportError:
                    from omni.isaac.sensor import Camera
                self._front_cam = Camera(
                    prim_path=self._cam_path,
                    name=f"{self.name}_front_camera",
                    resolution=(320, 240),
                    frequency=10,
                )
                self._front_cam.initialize()
            except Exception as e:
                carb.log_warn(f"[{self.name}] RGB 카메라 초기화 실패: {e}")

        self._step_count = 0
        self._hud_step   = 0
        carb.log_warn(f"[{self.name}] post_reset 완료  spawn={spawn}")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, dt: float) -> None:
        self._step_count += 1
        if self._step_count < self.WARMUP_STEPS:
            return
        if self.controller is None or not self.controller._received_first_state:
            return
        if self.hud is None:
            return

        # 운반 중인 포드 위치 갱신
        if self._carried_prim:
            self._update_carried_pod()

        # 자율 미션 FSM
        if self._mission_queue:
            self._run_mission_fsm()

        # HUD 갱신 (50Hz 충분)
        self._hud_step += 1
        if self._hud_step % HUD_UPDATE_N != 0:
            return
        self.hud.update_status(
            self.controller.active_input,
            self.controller.is_airborne,
            self.controller.p,
            self.controller.target_pos,
        )

    # ── on_render_step ───────────────────────────────────────────────
    def on_render_step(self) -> None:
        pass

    def get_camera_rgb(self):
        if self._front_cam is None:
            return None
        try:
            rgba = self._front_cam.get_rgba()
            if rgba is None or rgba.size == 0:
                return None
            return rgba[..., :3].copy()
        except Exception:
            return None

    # ── 미니맵용 위치 조회 ────────────────────────────────────────────
    def get_world_xy(self) -> tuple:
        """(x, y, heading_rad, altitude) 반환. 미니맵용."""
        if self.controller is None or not getattr(self.controller, '_received_first_state', False):
            return (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]), 0.0, 0.0)
        try:
            pos = self.controller.p
            hdg = 0.0
            if self.controller.R is not None:
                fwd = self.controller.R.apply([1.0, 0.0, 0.0])
                hdg = math.atan2(float(fwd[1]), float(fwd[0]))
            return (float(pos[0]), float(pos[1]), hdg, float(pos[2]))
        except Exception:
            return (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]), 0.0, 0.0)

    # ── 외부 미션 설정 ────────────────────────────────────────────────
    def set_mission(self, section_targets: dict, delivery_xyz=None) -> None:
        """런타임에 미션 재설정. section_targets = {"A": slot_id, "B": slot_id, ...}"""
        if delivery_xyz is not None:
            self._delivery_xyz = np.array(delivery_xyz, dtype=np.float64)
        self._mission_queue = self._build_mission_queue(section_targets)
        self._mission_idx   = 0
        self._mission_state = _MS_IDLE
        print(f"[{self.name}] 미션 설정 완료 — {len(self._mission_queue)}개 타겟")

    # ── 미션 헬퍼 ─────────────────────────────────────────────────────

    def _build_mission_queue(self, section_targets: dict) -> list:
        """섹션 박스 인덱스(1~3) → 섹션 슬롯(07~09) 매핑으로 미션 타겟 생성.

        section_targets 예: {"A": 2, "B": 3, "C": 1}
          인덱스 N (1~3) → 슬롯 (6+N):  1→07, 2→08, 3→09
          prim: /World/SectionBoxes/Sec_{sec}_{slot:02d}
        """
        from robot_config import SECTION_PODS
        queue = []
        for sec, box_idx in section_targets.items():
            box_idx = int(box_idx)
            slot = 6 + box_idx          # idx 1→slot07, 2→slot08, 3→slot09
            if not (7 <= slot <= 9):
                carb.log_warn(f"[{self.name}] 섹션 {sec}: 박스 인덱스 {box_idx} 무효 "
                              f"→ 슬롯 {slot} (유효 범위 1~3)")
                continue
            positions = SECTION_PODS.get(sec, [])
            pos_idx = slot - 1          # 0-based
            if pos_idx >= len(positions):
                carb.log_warn(f"[{self.name}] 섹션 {sec} 슬롯 {slot}: 위치 없음")
                continue
            pos = positions[pos_idx]
            prim_path = f"/World/SectionBoxes/Sec_{sec}_{slot:02d}"
            queue.append({
                "sec"      : sec,
                "slot"     : slot,
                "box_idx"  : box_idx,
                "prim_path": prim_path,
                "world_xy" : (float(pos[0]), float(pos[1])),
            })
            print(f"[{self.name}] 미션 타겟: Sec {sec} 박스#{box_idx} "
                  f"→ 슬롯{slot} @ ({pos[0]:.2f}, {pos[1]:.2f})  prim={prim_path}")
        return queue

    def _grab_pod(self, prim_path: str) -> bool:
        """포드 prim 을 kinematic 으로 전환하고 드론 기준 오프셋을 기록."""
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            carb.log_warn(f"[{self.name}] 그랩 대상 prim 없음: {prim_path}")
            return False

        # 물리 kinematic 전환 (중력 비활성화)
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Set(True)

        # XY: 항상 0 — 석커는 드론 body 정중앙 하단에 위치하므로 박스도 드론 정중앙 아래
        #   (그랩 순간 드론이 박스 중심에서 최대 NAV_TOL_XY=0.5m 벗어날 수 있으므로
        #    동적 XY 오프셋 유지 시 박스가 석커 옆에 매달리는 현상 발생 → 강제 0으로 해결)
        # Z: 석커 컵 face(body 하단 -0.134m, 컵 반높이 -0.006m)에 박스 상단(+0.15m)이 닿도록
        try:
            drone_pos = self.controller.p.copy()
            # 석커 컵 실제 월드 Z 읽기 (body prim 원점 vs controller.p 오프셋 제거)
            cache    = UsdGeom.XformCache()
            cup_prim = stage.GetPrimAtPath(f"/World/{self.name}/body/Sucker/Cup")
            if cup_prim.IsValid():
                cup_z = float(
                    cache.GetLocalToWorldTransform(cup_prim).ExtractTranslation()[2])
                # 박스 상단(center+0.15)이 컵 하면(center−0.006)에 닿도록
                offset_z = (cup_z - 0.006 - 0.15) - float(drone_pos[2])
            else:
                offset_z = -(0.134 + 0.006 + 0.15)  # 폴백 −0.290 m
            self._carry_offset = np.array([0.0, 0.0, offset_z])
            print(f"[{self.name}] 그랩 오프셋: cup_z={cup_z:.3f} "
                  f"offset_z={offset_z:.3f} drone_z={drone_pos[2]:.3f}")
        except Exception as e:
            self._carry_offset = np.array([0.0, 0.0, -0.290])
            carb.log_warn(f"[{self.name}] 그랩 오프셋 계산 실패: {e}")

        self._carried_prim = prim_path
        print(f"[{self.name}] 포드 그랩: {prim_path}  offset={self._carry_offset.round(3)}")
        return True

    def _update_carried_pod(self) -> None:
        """매 physics step: 운반 중인 포드를 드론 아래로 추종."""
        if self._carried_prim is None:
            return
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(self._carried_prim)
        if not prim.IsValid():
            self._carried_prim = None
            return
        target = self.controller.p + self._carry_offset
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(target[0]), float(target[1]), float(target[2])))
                return
        # TranslateOp 없으면 추가
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(float(target[0]), float(target[1]), float(target[2])))

    def _release_pod(self) -> None:
        """포드 해제: kinematic → dynamic 복원."""
        if self._carried_prim is None:
            return
        try:
            stage = omni.usd.get_context().get_stage()
            prim  = stage.GetPrimAtPath(self._carried_prim)
            if prim.IsValid():
                for p in Usd.PrimRange(prim):
                    if p.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Set(False)
        except Exception as e:
            carb.log_warn(f"[{self.name}] 포드 해제 오류: {e}")
        print(f"[{self.name}] 포드 해제: {self._carried_prim}")
        self._carried_prim = None

    def _at_xy(self, tx: float, ty: float) -> bool:
        pos = self.controller.p
        tx = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(tx)))
        ty = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(ty)))
        return math.hypot(pos[0] - tx, pos[1] - ty) < _NAV_TOL_XY

    def _at_xyz(self, tx: float, ty: float, tz: float) -> bool:
        pos = self.controller.p
        tx = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(tx)))
        ty = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(ty)))
        return (math.hypot(pos[0] - tx, pos[1] - ty) < _NAV_TOL_XY and
                abs(pos[2] - tz) < _NAV_TOL_Z)

    def _goto(self, x: float, y: float, z: float) -> None:
        """드론 목표 위치를 매 스텝 강제 설정 (키보드 입력 우선순위 압도)."""
        sx = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(x)))
        sy = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(y)))
        self.controller.target_pos = np.array([sx, sy, float(z)], dtype=np.float64)

    # ── 미션 FSM ─────────────────────────────────────────────────────
    def _run_mission_fsm(self) -> None:
        ctrl = self.controller

        if self._mission_state == _MS_IDLE:
            # 자동 이륙
            ctrl._cmd_takeoff()
            self._mission_state = _MS_TAKEOFF
            print(f"[{self.name}] IDLE → TAKEOFF")

        elif self._mission_state == _MS_TAKEOFF:
            # 수직 상승만: XY 고정, Z만 목표고도로
            self._goto(ctrl.p[0], ctrl.p[1], _HOVER_ALT)
            # 2.5m ±5cm 이내에서만 수평 이동 허가
            if ctrl.is_airborne and abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                self._mission_idx   = 0
                self._mission_state = _MS_FLY_PICK
                print(f"[{self.name}] TAKEOFF → FLY_PICK  (타겟 {self._mission_idx})")

        elif self._mission_state == _MS_FLY_PICK:
            if self._mission_idx >= len(self._mission_queue):
                self._mission_state = _MS_DONE
                print(f"[{self.name}] 모든 미션 완료 → DONE")
                return
            tgt = self._mission_queue[self._mission_idx]
            tx, ty = tgt["world_xy"]

            # Phase A: 2.5m ±5cm 이내가 될 때까지 XY 고정, Z만 상승
            if abs(ctrl.p[2] - _HOVER_ALT) > _ALT_GATE:
                self._goto(ctrl.p[0], ctrl.p[1], _HOVER_ALT)
                return

            # Phase B: 정확히 2.5m 도달 후에만 수평 이동 — Z 고정
            self._goto(tx, ty, _HOVER_ALT)
            if self._at_xy(tx, ty) and abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                self._grab_wait_cnt   = 0
                self._descend_cnt     = 0
                self._descend_xy      = (float(ctrl.p[0]), float(ctrl.p[1]))  # 수직 하강용 XY 고정
                ctrl.integral         = np.zeros(3)  # 누적 오차 초기화
                self._mission_state   = _MS_DESCEND_PICK
                print(f"[{self.name}] FLY_PICK → DESCEND_PICK  "
                      f"Sec {tgt['sec']} 슬롯 {tgt['slot']}")

        elif self._mission_state == _MS_DESCEND_PICK:
            tgt  = self._mission_queue[self._mission_idx]
            # 수직 하강 보장: FLY_PICK 도달 시 고정된 XY 사용
            dx, dy = self._descend_xy if self._descend_xy else tgt["world_xy"]
            self._goto(dx, dy, _GRAB_ALT)
            self._descend_cnt += 1
            if self._at_xyz(dx, dy, _GRAB_ALT):
                self._grab_wait_cnt += 1
                if self._grab_wait_cnt >= _GRAB_WAIT:
                    ok = self._grab_pod(tgt["prim_path"])
                    if ok:
                        self._ascend_xy = (float(ctrl.p[0]), float(ctrl.p[1]))
                        self._alt_stable_cnt = 0
                        self._mission_state = _MS_ASCEND_PICK
                        print(f"[{self.name}] DESCEND_PICK → ASCEND_PICK")
                    else:
                        # 포드 없음 → 다음 타겟으로 스킵
                        self._mission_idx  += 1
                        self._mission_state = _MS_FLY_PICK
                        print(f"[{self.name}] 포드 없음 → 다음 타겟으로 스킵")
            elif self._descend_cnt > _DESCEND_TIMEOUT:
                # 타임아웃: 위치 도달 실패 → 상승 후 다음 타겟으로
                carb.log_warn(f"[{self.name}] DESCEND_PICK 타임아웃 "
                              f"(pos={ctrl.p.round(2)}) → 다음 타겟 스킵")
                self._mission_idx  += 1
                self._mission_state = _MS_FLY_PICK

        elif self._mission_state == _MS_ASCEND_PICK:
            # 수직 상승만: 고정 XY 유지, 2.5m 안정화 전까지 수평 이동 금지
            dx, dy = self._ascend_xy if self._ascend_xy else (
                self._descend_xy if self._descend_xy else (float(ctrl.p[0]), float(ctrl.p[1])))
            self._goto(dx, dy, _HOVER_ALT)
            if abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                self._alt_stable_cnt += 1
                if self._alt_stable_cnt >= _ALT_STABLE_STEPS:
                    self._mission_state = _MS_FLY_DROP
                    self._ascend_xy = None
                    self._alt_stable_cnt = 0
                    print(f"[{self.name}] ASCEND_PICK → FLY_DROP  "
                          f"배달지={self._delivery_xyz[:2]}")
            else:
                self._alt_stable_cnt = 0

        elif self._mission_state == _MS_FLY_DROP:
            dx, dy = float(self._delivery_xyz[0]), float(self._delivery_xyz[1])

            # Phase A: hover altitude 도달까지 XY 고정, Z만 상승
            if abs(ctrl.p[2] - _HOVER_ALT) > _ALT_GATE or self._alt_stable_cnt < _ALT_STABLE_STEPS:
                if self._ascend_xy is None:
                    self._ascend_xy = (float(ctrl.p[0]), float(ctrl.p[1]))
                self._goto(self._ascend_xy[0], self._ascend_xy[1], _HOVER_ALT)
                if abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                    self._alt_stable_cnt += 1
                else:
                    self._alt_stable_cnt = 0
                self._fly_drop_log = getattr(self, "_fly_drop_log", 0) + 1
                if self._fly_drop_log % 200 == 1:
                    print(f"[{self.name}] FLY_DROP Phase-A 고도 안정화중  "
                          f"pos=({ctrl.p[0]:.1f},{ctrl.p[1]:.1f},{ctrl.p[2]:.1f})  "
                          f"target_alt={_HOVER_ALT:.1f}")
                return

            # Phase B: hover altitude 도달 후 수평 이동
            self._ascend_xy = None
            self._goto(dx, dy, _HOVER_ALT)
            self._fly_drop_log = getattr(self, "_fly_drop_log", 0) + 1
            if self._fly_drop_log % 200 == 1:
                dist = math.hypot(ctrl.p[0] - dx, ctrl.p[1] - dy)
                print(f"[{self.name}] FLY_DROP Phase-B  "
                      f"pos=({ctrl.p[0]:.1f},{ctrl.p[1]:.1f},{ctrl.p[2]:.1f})  "
                      f"→ delivery=({dx:.1f},{dy:.1f})  dist={dist:.1f}m")
            if self._at_xy(dx, dy) and abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                self._descend_xy    = (dx, dy)
                ctrl.integral       = np.zeros(3)
                self._mission_state = _MS_DESCEND_DROP
                self._fly_drop_log  = 0
                print(f"[{self.name}] FLY_DROP → DESCEND_DROP")

        elif self._mission_state == _MS_DESCEND_DROP:
            dx, dy = self._descend_xy if self._descend_xy else (
                float(self._delivery_xyz[0]), float(self._delivery_xyz[1]))
            self._goto(dx, dy, _DROP_ALT)
            if self._at_xyz(dx, dy, _DROP_ALT):
                self._mission_state = _MS_RELEASE
                print(f"[{self.name}] DESCEND_DROP → RELEASE")

        elif self._mission_state == _MS_RELEASE:
            self._release_pod()
            ctrl.integral      = np.zeros(3)  # 배달 후 누적 오차 초기화
            self._mission_idx += 1
            self._ascend_xy = (float(ctrl.p[0]), float(ctrl.p[1]))
            self._alt_stable_cnt = 0
            self._mission_state = _MS_REASCEND
            print(f"[{self.name}] RELEASE → REASCEND  "
                  f"(다음 타겟 idx={self._mission_idx})")

        elif self._mission_state == _MS_REASCEND:
            # 수직 상승만: 배달 지점 XY 고정, 2.5m ±5cm 이내가 될 때까지 대기
            if self._ascend_xy is None:
                self._ascend_xy = (float(ctrl.p[0]), float(ctrl.p[1]))
            self._goto(self._ascend_xy[0], self._ascend_xy[1], _HOVER_ALT)
            if abs(ctrl.p[2] - _HOVER_ALT) < _ALT_GATE:
                self._alt_stable_cnt += 1
                if self._alt_stable_cnt >= _ALT_STABLE_STEPS:
                    self._ascend_xy = None
                    self._alt_stable_cnt = 0
                    if self._mission_idx < len(self._mission_queue):
                        self._mission_state = _MS_FLY_PICK
                        print(f"[{self.name}] REASCEND → FLY_PICK  "
                              f"타겟 {self._mission_idx}")
                    else:
                        self._mission_state = _MS_DONE
                        print(f"[{self.name}] REASCEND → DONE  모든 미션 완료!")
            else:
                self._alt_stable_cnt = 0

        elif self._mission_state == _MS_DONE:
            # 이륙 고도 유지하며 대기
            self._goto(ctrl.target_pos[0], ctrl.target_pos[1], _HOVER_ALT)
