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
      "delivery_xyz"   : (12.0, 12.0, 0.0)

robot_config.py 에서 등록:
    {
        "type"            : "drone",
        "name"            : "Drone_01",
        "spawn_xyz"       : (-6.8, -11.5, 0.07),
        "takeoff_alt"     : 2.5,
        "section_targets" : {"A": 3, "B": 2, "C": 3},
        "delivery_xyz"    : (12.0, 12.0, 0.0),
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
from input_monitor   import DroneInputMonitor
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

_AS_IDLE         = "IDLE"
_AS_ASCEND       = "ASCEND"
_AS_XY_ALIGN     = "XY_ALIGN"
_AS_DESCEND      = "DESCEND"
_AS_GRAB         = "GRAB"
_AS_ASCEND_CARRY = "ASCEND_CARRY"

_DS_IDLE         = "IDLE"
_DS_FLY_DROP     = "FLY_DROP"
_DS_DESCEND_DROP = "DESCEND_DROP"
_DS_RELEASE      = "RELEASE"
_DS_ASCEND       = "ASCEND"
_DS_RETURN_HOME  = "RETURN_HOME"

_HOVER_ALT        = 2.5    # 순항 고도 [m] — ceiling 충돌 방지용 고정 미션 고도
_GRAB_ALT         = 1.5    # 그랩 시 드론 고도 [m] — 섹션 박스(케이지 내부 z≈0.91m) 위 호버링
_DROP_ALT         = 1.5    # 배달지 릴리즈 고도 [m] — 순항 고도에서 떨어뜨리지 않고 내려놓기
_NAV_TOL_XY       = 0.5    # 수평 도착 허용 오차 [m]
_NAV_TOL_Z        = 0.25   # 수직 도착 허용 오차 [m]
_PICK_TOL_XY      = 0.12   # 픽업 전 정밀 XY 정렬 허용 오차 [m]
_DROP_TOL_XY      = 0.20   # 드롭 전 정밀 XY 정렬 허용 오차 [m]
_ALT_GATE         = 0.05   # 수직→수평 전환 고도 허용 오차 [m] — 2.5m 도달 전 XY 이동 금지
_ALT_STABLE_STEPS = 20     # 수평 이동 전 2.5m 유지 확인 steps
_ALIGN_STABLE_STEPS = 30   # 픽/드롭 전 XY+Z 안정화 확인 steps
_SAFE_XY_MIN      = -13.2
_SAFE_XY_MAX      = 13.2
_SAFE_AIR_X       = -6.8
_CENTER_LANE_Y    = 0.0
_WALL_LANE_Y      = 12.0
_AXIS_ROUTE_TOL   = 0.35
_XY_TARGET_RATE   = 1.15   # auto target speed limit [m/s] to prevent wall overshoot
_Z_TARGET_RATE    = 0.75   # vertical target speed limit [m/s]
_XY_CORRECT_RATE  = 0.45   # slow precision XY correction before descent [m/s]
_ASSIST_XY_RATE   = 0.75   # deliberate center-over-box movement speed [m/s]
_AUTO_XY_LOOKAHEAD = 1.2   # staged autopilot target distance ahead of drone [m]
_WALL_GUARD       = 13.05  # emergency retreat threshold [m]
_WALL_RETREAT     = 12.0   # retreat target inside safe air corridor [m]
_ALT_MAX          = 2.75   # hard recovery ceiling [m]
_ALT_RECOVER_Z    = 2.25   # target while recovering from overshoot [m]
_GRAB_WAIT        = 40     # 그랩 위치 안정화 대기 (physics steps)
_DESCEND_TIMEOUT  = 3000   # DESCEND_PICK 최대 대기 steps (~6s@500Hz) — 타임아웃 시 스킵
_ASSIST_PICK_TOL_XY = 0.04
_ASSIST_STABLE_STEPS = 30
_ASSIST_GRAB_MAX_DIST = 2.0


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
        self.input_monitor = None

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
        self._align_stable_cnt = 0          # 픽/드롭 정렬 안정화 카운터
        self._assist_state    = _AS_IDLE
        self._assist_target_path = None
        self._assist_target_xy = None
        self._assist_target_dist = None
        self._assist_grab_alt = _GRAB_ALT
        self._assist_wait_cnt = 0
        self._assist_grab_range = float(self.cfg.get("assist_grab_range", _ASSIST_GRAB_MAX_DIST))
        self._assist_align_alt = _GRAB_ALT
        self._delivery_state   = _DS_IDLE
        self._delivery_wait_cnt = 0
        self._delivery_xyz    = np.array(self.cfg.get("delivery_xyz", [12.0, 12.0, 0.0]),
                                         dtype=np.float64)

        # Manual joystick mode by default. Set auto_mission=True to restore queue mode.
        sec_targets = self.cfg.get("section_targets", {}) if self.cfg.get("auto_mission", False) else {}
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

        # Keep drone collisions enabled. Disabling them during flight made LAND
        # zero the thrust while the ground plane could not support the drone,
        # which let the body continue into negative Z.
        self._enable_drone_collisions()
        self._collisions_disabled = False
        self._prev_airborne       = False

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

        # Input visualiser — separate window showing live axes + button flashes
        try:
            self.input_monitor = DroneInputMonitor(self.controller)
            carb.log_warn(f"[{self.name}] Input Monitor 창 생성 완료")
        except Exception as e:
            self.input_monitor = None
            carb.log_warn(f"[{self.name}] Input Monitor 생성 실패: {e}")

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

        # Sync target_pos to spawn so HUD shows sensible values from the start.
        # PID doesn't run when grounded, but _cmd_takeoff() also resets this on T-press.
        self.controller.target_pos = np.array(
            [float(spawn[0]), float(spawn[1]), float(spawn[2])], dtype=np.float64)
        self.controller.target_yaw = 0.0
        self.controller.integral   = np.zeros(3)

        self._step_count = 0
        self._hud_step   = 0
        carb.log_warn(f"[{self.name}] post_reset 완료  spawn={spawn}  "
                      f"target_pos={self.controller.target_pos}")

    def _disable_drone_collisions(self) -> None:
        """Keep the drone from wall-crashing while the mission controller runs."""
        try:
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath(f"/World/{self.name}")
            if not root.IsValid():
                return
            disabled = 0
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                    disabled += 1
            carb.log_warn(f"[{self.name}] drone collision disabled on {disabled} prims")
        except Exception as e:
            carb.log_warn(f"[{self.name}] drone collision disable failed: {e}")

    def _enable_drone_collisions(self) -> None:
        """Re-enable drone collisions after landing so ground plane can support it."""
        try:
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath(f"/World/{self.name}")
            if not root.IsValid():
                return
            enabled = 0
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(True)
                    enabled += 1
            # Sucker parts are visual-only — keep their collision off
            sucker_path = f"/World/{self.name}/body/Sucker"
            sucker_root = stage.GetPrimAtPath(sucker_path)
            if sucker_root.IsValid():
                for prim in Usd.PrimRange(sucker_root):
                    if prim.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                        enabled -= 1
            carb.log_warn(f"[{self.name}] drone collision re-enabled on {enabled} prims")
        except Exception as e:
            carb.log_warn(f"[{self.name}] drone collision enable failed: {e}")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, dt: float) -> None:
        self._step_count += 1
        self._last_dt = float(dt) if dt and dt > 0 else 1.0 / 500.0
        if self._step_count < self.WARMUP_STEPS:
            return
        if self.controller is None or not self.controller._received_first_state:
            return
        if self.hud is None:
            return

        # Collision management: keep the ground plane active so landing cannot
        # pass through z=0. Sucker collisions stay disabled inside
        # _enable_drone_collisions().
        curr_airborne = self.controller.is_airborne
        if self._collisions_disabled:
            self._enable_drone_collisions()
            self._collisions_disabled = False
        self._prev_airborne = curr_airborne

        # 운반 중인 포드 위치 갱신
        if self._carried_prim:
            self._update_carried_pod()

        if hasattr(self.controller, "consume_release_request") and self.controller.consume_release_request():
            self._start_assisted_delivery()
            self._assist_state = _AS_IDLE

        if hasattr(self.controller, "consume_grab_request") and self.controller.consume_grab_request():
            self._start_assisted_grab()

        if self._delivery_state != _DS_IDLE:
            self._run_assisted_delivery_fsm()
        elif self._assist_state != _AS_IDLE:
            self._run_assisted_grab_fsm()

        # Optional legacy autonomous mission FSM, disabled by default in robot_config.
        elif self._mission_queue:
            if self._recover_altitude_overshoot():
                return
            if self._recover_from_wall_risk():
                return
            self._run_mission_fsm()

        # HUD + Input Monitor 갱신 (50Hz 충분)
        self._hud_step += 1
        if self._hud_step % HUD_UPDATE_N != 0:
            return
        self.hud.update_status(
            self.controller.active_input,
            self.controller.is_airborne,
            self.controller.p,
            self.controller.target_pos,
        )
        if self.input_monitor is not None:
            try:
                self.input_monitor.update()
            except Exception:
                pass

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
        self._alt_stable_cnt = 0
        self._align_stable_cnt = 0
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

    def _iter_grabbable_box_prims(self):
        stage = omni.usd.get_context().get_stage()
        roots = (
            "/World/SectionBoxes",
            "/World/DynamicBoxes",
            "/World/MinimapClickBoxes",
        )
        for root_path in roots:
            root = stage.GetPrimAtPath(root_path)
            if root.IsValid():
                yield from root.GetChildren()
        world = stage.GetPrimAtPath("/World")
        if world.IsValid():
            for prim in world.GetChildren():
                if str(prim.GetPath()).startswith("/World/AutoBox_"):
                    yield prim

    def _find_nearest_grabbable_box(self):
        if self.controller is None or not getattr(self.controller, "_received_first_state", False):
            return None, None, float("inf")
        pos_xy = np.array(self.controller.p[:2], dtype=np.float64)
        cache = UsdGeom.XformCache()
        best_path, best_xy, best_dist = None, None, float("inf")
        for prim in self._iter_grabbable_box_prims():
            path = str(prim.GetPath())
            if path.endswith("_b1") or path.endswith("_b2"):
                continue
            if path == self._carried_prim:
                continue
            if prim.GetCustomDataByKey("drone_delivered"):
                continue
            self._set_box_mass_kg(path, 2.0)
            try:
                tr = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                xy = np.array([float(tr[0]), float(tr[1])], dtype=np.float64)
                d = float(np.linalg.norm(xy - pos_xy))
                if d < best_dist:
                    best_path, best_xy, best_dist = path, xy, d
            except Exception:
                continue
        return best_path, best_xy, best_dist

    def _start_assisted_grab(self) -> None:
        if self._carried_prim is not None:
            print(f"[{self.name}] already carrying; press C to release first")
            return
        path, xy, dist = self._find_nearest_grabbable_box()
        if path is None or xy is None:
            carb.log_warn(f"[{self.name}] no grabbable box found near drone")
            return
        if dist > self._assist_grab_range:
            carb.log_warn(
                f"[{self.name}] Y grab assist ignored: closest box is {dist:.2f}m away "
                f"(range {self._assist_grab_range:.2f}m)"
            )
            return
        if self.controller is not None and not self.controller.is_airborne:
            self.controller._cmd_takeoff()
        self._assist_target_path = path
        self._assist_target_xy = xy
        self._assist_target_dist = dist
        self._assist_grab_alt = self._grab_alt_for_box(path)
        self._assist_wait_cnt = 0
        self._alt_stable_cnt = 0
        self._align_stable_cnt = 0
        self._descend_xy = None
        self._assist_align_alt = max(_GRAB_ALT, min(_HOVER_ALT, float(self.controller.p[2])))
        self.controller.autopilot_active = True
        self.controller.integral = np.zeros(3)
        self._assist_state = _AS_XY_ALIGN
        print(f"[{self.name}] Y grab assist start: {path} center={xy.round(2)} "
              f"dist={dist:.2f}m range={self._assist_grab_range:.2f}m "
              f"align_alt={self._assist_align_alt:.2f}m "
              f"grab_alt={self._assist_grab_alt:.2f}m")

    def _run_assisted_grab_fsm(self) -> None:
        if self._assist_target_path is None or self._assist_target_xy is None:
            self._assist_state = _AS_IDLE
            self.controller.autopilot_active = False
            return
        tx, ty = float(self._assist_target_xy[0]), float(self._assist_target_xy[1])

        if self._assist_state == _AS_ASCEND:
            self._assist_state = _AS_XY_ALIGN

        elif self._assist_state == _AS_XY_ALIGN:
            self._goto_xy_direct_at_alt(tx, ty, self._assist_align_alt)
            xy_err = self._xy_error(tx, ty)
            z_err = abs(float(self.controller.p[2]) - self._assist_align_alt)
            self._assist_wait_cnt += 1
            if self._assist_wait_cnt % 100 == 1:
                px, py, pz = self.controller.p[:3]
                print(f"[{self.name}] grab assist aligning  "
                      f"pos=({px:.2f},{py:.2f},{pz:.2f}) "
                      f"target=({tx:.2f},{ty:.2f},{self._assist_align_alt:.2f}) "
                      f"xy_err={xy_err:.2f} z_err={z_err:.2f}")
            if xy_err < _ASSIST_PICK_TOL_XY and z_err < _NAV_TOL_Z:
                self._align_stable_cnt += 1
                if self._align_stable_cnt >= _ASSIST_STABLE_STEPS:
                    self._descend_xy = (tx, ty)
                    self._assist_wait_cnt = 0
                    self._align_stable_cnt = 0
                    self.controller.integral = np.zeros(3)
                    self._assist_state = _AS_DESCEND
                    print(f"[{self.name}] grab assist: centered over box -> DESCEND")
            else:
                self._align_stable_cnt = 0

        elif self._assist_state == _AS_DESCEND:
            dx, dy = self._descend_xy if self._descend_xy else (tx, ty)
            if self._xy_error(dx, dy) > _ASSIST_PICK_TOL_XY:
                self._goto_xy_at_current_alt(dx, dy)
                self._assist_wait_cnt = 0
                return
            self._freeze_manual_input()
            self._goto(dx, dy, self._assist_grab_alt)
            if self._at_xyz(dx, dy, self._assist_grab_alt, _ASSIST_PICK_TOL_XY, _NAV_TOL_Z):
                self._assist_wait_cnt += 1
                if self._assist_wait_cnt >= _GRAB_WAIT:
                    self._assist_state = _AS_GRAB
            else:
                self._assist_wait_cnt = 0

        elif self._assist_state == _AS_GRAB:
            if self._grab_pod(self._assist_target_path):
                self._assist_wait_cnt = 0
                self._alt_stable_cnt = 0
                self._assist_state = _AS_ASCEND_CARRY
                print(f"[{self.name}] grab assist: GRAB -> ASCEND_CARRY")
            else:
                self._assist_state = _AS_IDLE
                self.controller.autopilot_active = False

        elif self._assist_state == _AS_ASCEND_CARRY:
            self._goto_vertical_only(_HOVER_ALT)
            if self._hover_alt_ready():
                self._alt_stable_cnt += 1
                if self._alt_stable_cnt >= _ALT_STABLE_STEPS:
                    self._assist_state = _AS_IDLE
                    self._assist_target_path = None
                    self._assist_target_xy = None
                    self._assist_target_dist = None
                    self.controller.autopilot_active = False
                    self._alt_stable_cnt = 0
                    print(f"[{self.name}] grab assist done at 2.5m; joystick control resumed")
            else:
                self._alt_stable_cnt = 0

    def _start_assisted_delivery(self) -> None:
        if self._carried_prim is None:
            print(f"[{self.name}] X delivery ignored: no carried box")
            return
        self._assist_state = _AS_IDLE
        self._delivery_state = _DS_FLY_DROP
        self._delivery_wait_cnt = 0
        self._align_stable_cnt = 0
        self._alt_stable_cnt = 0
        self.controller.autopilot_active = True
        self.controller.integral = np.zeros(3)
        dx, dy = self._clamp_xy(self._delivery_xyz[0], self._delivery_xyz[1])
        print(f"[{self.name}] X delivery assist start: target=({dx:.2f},{dy:.2f}) "
              f"return=({self.spawn_xyz[0]:.2f},{self.spawn_xyz[1]:.2f},2.50)")

    def _run_assisted_delivery_fsm(self) -> None:
        dx, dy = self._clamp_xy(self._delivery_xyz[0], self._delivery_xyz[1])
        hx, hy = float(self.spawn_xyz[0]), float(self.spawn_xyz[1])

        if float(self.controller.p[2]) > _ALT_MAX:
            self._freeze_manual_input()
            self.controller.integral = np.zeros(3)
            self._goto(self.controller.p[0], self.controller.p[1], _ALT_RECOVER_Z,
                       clamp_xy=False)
            self._align_stable_cnt = 0
            self._delivery_wait_cnt += 1
            if self._delivery_wait_cnt % 100 == 1:
                print(f"[{self.name}] delivery altitude recovery  "
                      f"z={self.controller.p[2]:.2f} -> target={_ALT_RECOVER_Z:.2f}")
            return

        if self._delivery_state == _DS_FLY_DROP:
            cmd_x, cmd_y = self._goto_xy_lookahead_at_alt(dx, dy, _HOVER_ALT)
            xy_err = self._xy_error(dx, dy)
            z_err = abs(float(self.controller.p[2]) - _HOVER_ALT)
            self._delivery_wait_cnt += 1
            if self._delivery_wait_cnt % 100 == 1:
                px, py, pz = self.controller.p[:3]
                print(f"[{self.name}] delivery aligning  "
                      f"pos=({px:.2f},{py:.2f},{pz:.2f}) "
                      f"target=({dx:.2f},{dy:.2f},{_HOVER_ALT:.2f}) "
                      f"cmd=({cmd_x:.2f},{cmd_y:.2f},{_HOVER_ALT:.2f}) "
                      f"xy_err={xy_err:.2f} z_err={z_err:.2f}")
            if xy_err < _DROP_TOL_XY and z_err < _ALT_GATE:
                self._align_stable_cnt += 1
                if self._align_stable_cnt >= _ALIGN_STABLE_STEPS:
                    self._delivery_state = _DS_RELEASE
                    self._delivery_wait_cnt = 0
                    self._align_stable_cnt = 0
                    self.controller.integral = np.zeros(3)
                    print(f"[{self.name}] delivery: stable over drop -> RELEASE")
            else:
                self._align_stable_cnt = 0

        elif self._delivery_state == _DS_DESCEND_DROP:
            if self._xy_error(dx, dy) > _DROP_TOL_XY:
                self._goto_xy_direct_at_alt(dx, dy, float(self.controller.p[2]))
                self._delivery_wait_cnt = 0
                return
            self._freeze_manual_input()
            self._goto(dx, dy, _DROP_ALT)
            if self._at_xyz(dx, dy, _DROP_ALT, _DROP_TOL_XY, _NAV_TOL_Z):
                self._delivery_wait_cnt += 1
                if self._delivery_wait_cnt >= _GRAB_WAIT:
                    self._delivery_state = _DS_RELEASE
            else:
                self._delivery_wait_cnt = 0

        elif self._delivery_state == _DS_RELEASE:
            self._release_pod()
            self._delivery_state = _DS_ASCEND
            self._alt_stable_cnt = 0
            self.controller.integral = np.zeros(3)
            print(f"[{self.name}] delivery: RELEASE -> ASCEND")

        elif self._delivery_state == _DS_ASCEND:
            self._goto_vertical_only(_HOVER_ALT)
            if self._hover_alt_ready():
                self._alt_stable_cnt += 1
                if self._alt_stable_cnt >= _ALT_STABLE_STEPS:
                    self._delivery_state = _DS_RETURN_HOME
                    self._alt_stable_cnt = 0
                    self._align_stable_cnt = 0
                    print(f"[{self.name}] delivery: ASCEND -> RETURN_HOME")
            else:
                self._alt_stable_cnt = 0

        elif self._delivery_state == _DS_RETURN_HOME:
            self._goto_xy_lookahead_at_alt(hx, hy, _HOVER_ALT)
            xy_err = math.hypot(float(self.controller.p[0]) - hx,
                                float(self.controller.p[1]) - hy)
            z_err = abs(float(self.controller.p[2]) - _HOVER_ALT)
            if xy_err < _DROP_TOL_XY and z_err < _ALT_GATE:
                self._align_stable_cnt += 1
                if self._align_stable_cnt >= _ALIGN_STABLE_STEPS:
                    self._delivery_state = _DS_IDLE
                    self._delivery_wait_cnt = 0
                    self._align_stable_cnt = 0
                    self.controller.autopilot_active = False
                    self.controller.target_pos = np.array([hx, hy, _HOVER_ALT], dtype=np.float64)
                    print(f"[{self.name}] delivery done; waiting at spawn z=2.5, joystick control resumed")
            else:
                self._align_stable_cnt = 0

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

    def _set_box_mass_kg(self, prim_path: str, mass: float = 2.0) -> None:
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(float(mass))
        except Exception:
            pass

    def _box_top_z(self, prim_path: str) -> float | None:
        rng = self._prim_world_range(prim_path)
        if rng is None or rng.IsEmpty():
            return None
        return float(rng.GetMax()[2])

    def _sucker_cup_bottom_offset(self) -> float:
        """World cup bottom z minus controller body z."""
        try:
            stage = omni.usd.get_context().get_stage()
            cup_prim = stage.GetPrimAtPath(f"/World/{self.name}/body/Sucker/Cup")
            if cup_prim.IsValid():
                cache = UsdGeom.XformCache()
                cup_z = float(cache.GetLocalToWorldTransform(cup_prim).ExtractTranslation()[2])
                return (cup_z - 0.006) - float(self.controller.p[2])
        except Exception:
            pass
        return -(0.134 + 0.006)

    def _grab_alt_for_box(self, prim_path: str) -> float:
        """Drone body altitude where suction cup just reaches the selected box top."""
        top_z = self._box_top_z(prim_path)
        if top_z is None:
            return _GRAB_ALT
        alt = top_z - self._sucker_cup_bottom_offset() + 0.01
        return max(0.25, min(_HOVER_ALT, float(alt)))

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
        """Release the carried box at the current drone pose so physics drops it."""
        if self._carried_prim is None:
            return
        try:
            stage = omni.usd.get_context().get_stage()
            prim  = stage.GetPrimAtPath(self._carried_prim)
            if prim.IsValid():
                place_xyz = self.controller.p + self._carry_offset
                xf = UsdGeom.Xformable(prim)
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(*[float(v) for v in place_xyz]))
                        break
                else:
                    xf.ClearXformOpOrder()
                    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in place_xyz]))
                for p in Usd.PrimRange(prim):
                    if p.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Set(False)
                prim.SetCustomDataByKey("drone_delivered", True)
                print(f"[{self.name}] delivery release drop: pos={place_xyz.round(3)}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] 포드 해제 오류: {e}")
        print(f"[{self.name}] 포드 해제: {self._carried_prim}")
        self._carried_prim = None

    def _prim_world_range(self, prim_path: str):
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                useExtentsHint=True,
            )
            return cache.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:
            return None

    def _prim_half_height(self, prim_path: str, default: float = 0.15) -> float:
        rng = self._prim_world_range(prim_path)
        if rng is None or rng.IsEmpty():
            return float(default)
        size = rng.GetSize()
        return max(0.02, float(size[2]) * 0.5)

    def _delivery_surface_z(self) -> float:
        """Top surface of the delivery stack at (12, 12), fallback to config z."""
        rng = self._prim_world_range("/World/PodStacks/PodStack_04")
        if rng is not None and not rng.IsEmpty():
            return float(rng.GetMax()[2])
        return float(self._delivery_xyz[2])

    def _release_pod_here(self) -> None:
        """Release carried box at the current joystick-positioned drone XY."""
        if self._carried_prim is None:
            print(f"[{self.name}] C release ignored: no carried box")
            return
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self._carried_prim)
            if prim.IsValid():
                rx, ry = self._clamp_xy(self.controller.p[0], self.controller.p[1])
                place_xyz = np.array([rx, ry, 0.15], dtype=np.float64)
                xf = UsdGeom.Xformable(prim)
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(*[float(v) for v in place_xyz]))
                        break
                else:
                    xf.ClearXformOpOrder()
                    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in place_xyz]))
                for p in Usd.PrimRange(prim):
                    if p.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Set(True)
                prim.SetCustomDataByKey("drone_delivered", True)
                print(f"[{self.name}] C release: {self._carried_prim} @ {place_xyz.round(2)}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] C release failed: {e}")
        self._carried_prim = None

    def _at_xy(self, tx: float, ty: float) -> bool:
        return self._xy_error(tx, ty) < _NAV_TOL_XY

    def _xy_error(self, tx: float, ty: float) -> float:
        pos = self.controller.p
        tx = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(tx)))
        ty = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(ty)))
        return math.hypot(pos[0] - tx, pos[1] - ty)

    def _at_xyz(self, tx: float, ty: float, tz: float,
                xy_tol: float = _NAV_TOL_XY, z_tol: float = _NAV_TOL_Z) -> bool:
        return self._xy_error(tx, ty) < xy_tol and abs(self.controller.p[2] - tz) < z_tol

    def _goto_vertical_only(self, z: float) -> None:
        """Change altitude without commanding lateral translation."""
        self._freeze_manual_input()
        self._goto(self.controller.p[0], self.controller.p[1], float(z), clamp_xy=False)

    def _goto_xy_only(self, x: float, y: float) -> None:
        """Move laterally only after hover altitude is stable."""
        self._freeze_manual_input()
        sx, sy = self._staged_xy_target(x, y)
        nx, ny = self._step_xy_toward(sx, sy, _XY_TARGET_RATE)
        self._goto(nx, ny, _HOVER_ALT)

    def _goto_xy_at_current_alt(self, x: float, y: float) -> None:
        """Correct XY while holding the current altitude target."""
        self._freeze_manual_input()
        nx, ny = self._step_xy_toward(x, y, _XY_CORRECT_RATE)
        self._goto(nx, ny, self.controller.p[2])

    def _goto_xy_direct_at_alt(self, x: float, y: float, z: float) -> None:
        """Move directly to a nearby box center and hold the requested altitude."""
        self._freeze_manual_input()
        self._goto(x, y, float(z))

    def _goto_xy_staged_at_alt(self, x: float, y: float, z: float, rate: float) -> tuple[float, float]:
        """Move toward a far target using a nearby staged target to avoid overshoot."""
        self._freeze_manual_input()
        sx, sy = self._staged_xy_target(x, y)
        nx, ny = self._lookahead_xy_toward(sx, sy, _AUTO_XY_LOOKAHEAD)
        self._goto(nx, ny, float(z))
        return nx, ny

    def _goto_xy_lookahead_at_alt(self, x: float, y: float, z: float) -> tuple[float, float]:
        """Move directly toward a world XY target using a bounded look-ahead point."""
        self._freeze_manual_input()
        nx, ny = self._lookahead_xy_toward(x, y, _AUTO_XY_LOOKAHEAD)
        self._goto(nx, ny, float(z))
        return nx, ny

    def _lookahead_xy_toward(self, x: float, y: float, lookahead: float) -> tuple[float, float]:
        """Return a target far enough ahead for the PID to create visible motion."""
        sx, sy = self._clamp_xy(x, y)
        px, py = float(self.controller.p[0]), float(self.controller.p[1])
        dx, dy = sx - px, sy - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return sx, sy
        step = min(float(lookahead), dist)
        scale = step / dist
        return px + dx * scale, py + dy * scale

    def _hover_alt_ready(self) -> bool:
        return (
            self.controller.is_airborne and
            abs(float(self.controller.p[2]) - _HOVER_ALT) < _ALT_GATE
        )

    def _recover_altitude_overshoot(self) -> bool:
        """Force vertical-only recovery if the controller overshoots above hover."""
        if self.controller is None or not self.controller.is_airborne:
            return False
        pz = float(self.controller.p[2])
        if pz <= _ALT_MAX:
            return False
        self._freeze_manual_input()
        self.controller.integral = np.zeros(3)
        self._goto(self.controller.p[0], self.controller.p[1], _ALT_RECOVER_Z,
                   clamp_xy=False)
        self._alt_stable_cnt = 0
        self._align_stable_cnt = 0
        return True

    def _recover_from_wall_risk(self) -> bool:
        """Retreat before the drone reaches wall/corner contact."""
        if self.controller is None or not self.controller.is_airborne:
            return False
        if self._mission_state in (_MS_DESCEND_PICK, _MS_DESCEND_DROP, _MS_RELEASE):
            return False
        px, py, pz = map(float, self.controller.p[:3])
        if abs(px) <= _WALL_GUARD and abs(py) <= _WALL_GUARD and pz >= 1.0:
            return False
        self._freeze_manual_input()
        rx = max(-_WALL_RETREAT, min(_WALL_RETREAT, px))
        ry = max(-_WALL_RETREAT, min(_WALL_RETREAT, py))
        if pz < _HOVER_ALT - _ALT_GATE:
            self._goto_vertical_only(_HOVER_ALT)
        else:
            nx, ny = self._step_xy_toward(rx, ry, _XY_TARGET_RATE)
            self._goto(nx, ny, _HOVER_ALT)
        self._alt_stable_cnt = 0
        self._align_stable_cnt = 0
        return True

    def _clamp_xy(self, x: float, y: float) -> tuple[float, float]:
        sx = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(x)))
        sy = max(_SAFE_XY_MIN, min(_SAFE_XY_MAX, float(y)))
        return sx, sy

    def _freeze_manual_input(self) -> None:
        """Auto mission owns the target; ignore held keys/sticks during staged motion."""
        try:
            for key in self.controller._keys:
                self.controller._keys[key] = False
            with self.controller._axes_lock:
                for key in self.controller._axes:
                    self.controller._axes[key] = 0.0
            self.controller.active_input = "auto"
        except Exception:
            pass

    def _step_scalar(self, cur: float, target: float, rate: float) -> float:
        dt = max(float(getattr(self, "_last_dt", 1.0 / 500.0)), 1.0 / 1000.0)
        max_step = max(rate * dt, 0.001)
        err = float(target) - float(cur)
        if abs(err) <= max_step:
            return float(target)
        return float(cur) + math.copysign(max_step, err)

    def _step_xy_toward(self, x: float, y: float, rate: float) -> tuple[float, float]:
        sx, sy = self._clamp_xy(x, y)
        px, py = float(self.controller.p[0]), float(self.controller.p[1])
        dx, dy = sx - px, sy - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return sx, sy
        dt = max(float(getattr(self, "_last_dt", 1.0 / 500.0)), 1.0 / 1000.0)
        step = max(rate * dt, 0.001)
        if dist <= step:
            return sx, sy
        scale = step / dist
        return px + dx * scale, py + dy * scale

    def _staged_xy_target(self, x: float, y: float) -> tuple[float, float]:
        """Avoid wall/corner hits by moving one axis at a time via the center lane."""
        sx, sy = self._clamp_xy(x, y)
        px, py = float(self.controller.p[0]), float(self.controller.p[1])

        # First leave tight west/north/south wall lanes before crossing the map.
        if px < _SAFE_AIR_X and abs(px - sx) > _AXIS_ROUTE_TOL:
            return _SAFE_AIR_X, py
        if abs(py) > _WALL_LANE_Y and abs(sy - py) > 1.0:
            return px, _CENTER_LANE_Y

        dx = abs(sx - px)
        dy = abs(sy - py)
        if dx > _AXIS_ROUTE_TOL and dy > _AXIS_ROUTE_TOL:
            # Cross in the center lane before going north/south again.
            if abs(py - _CENTER_LANE_Y) > _AXIS_ROUTE_TOL:
                return px, _CENTER_LANE_Y
            return sx, py
        return sx, sy

    def _goto(self, x: float, y: float, z: float, clamp_xy: bool = True) -> None:
        """드론 목표 위치를 매 스텝 강제 설정 (키보드 입력 우선순위 압도)."""
        if clamp_xy:
            sx, sy = self._clamp_xy(x, y)
        else:
            sx, sy = float(x), float(y)
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
            self._goto_vertical_only(_HOVER_ALT)
            # 2.5m ±5cm 이내에서만 수평 이동 허가
            if self._hover_alt_ready():
                self._alt_stable_cnt += 1
                if self._alt_stable_cnt >= _ALT_STABLE_STEPS:
                    self._mission_idx   = 0
                    self._mission_state = _MS_FLY_PICK
                    self._alt_stable_cnt = 0
                    print(f"[{self.name}] TAKEOFF → FLY_PICK  (타겟 {self._mission_idx})")
            else:
                self._alt_stable_cnt = 0

        elif self._mission_state == _MS_FLY_PICK:
            if self._mission_idx >= len(self._mission_queue):
                self._mission_state = _MS_DONE
                print(f"[{self.name}] 모든 미션 완료 → DONE")
                return
            tgt = self._mission_queue[self._mission_idx]
            tx, ty = tgt["world_xy"]

            # Phase A: 2.5m ±5cm 이내가 될 때까지 XY 고정, Z만 상승
            if not self._hover_alt_ready():
                self._goto_vertical_only(_HOVER_ALT)
                self._align_stable_cnt = 0
                self._alt_stable_cnt = 0
                return
            self._alt_stable_cnt += 1
            if self._alt_stable_cnt < _ALT_STABLE_STEPS:
                self._goto_vertical_only(_HOVER_ALT)
                return
            self._alt_stable_cnt = _ALT_STABLE_STEPS

            # Phase B: 고도 안정 후 수평 이동만 — Z 명령은 현재 고도로 유지
            self._goto_xy_only(tx, ty)
            if self._at_xyz(tx, ty, _HOVER_ALT, _PICK_TOL_XY, _ALT_GATE):
                self._align_stable_cnt += 1
            else:
                self._align_stable_cnt = 0

            if self._align_stable_cnt >= _ALIGN_STABLE_STEPS:
                self._grab_wait_cnt   = 0
                self._descend_cnt     = 0
                self._descend_xy      = (float(tx), float(ty))  # 정렬 완료 지점에서 수직 하강
                self._align_stable_cnt = 0
                ctrl.integral         = np.zeros(3)  # 누적 오차 초기화
                self._mission_state   = _MS_DESCEND_PICK
                print(f"[{self.name}] FLY_PICK → DESCEND_PICK  "
                      f"Sec {tgt['sec']} 슬롯 {tgt['slot']}")

        elif self._mission_state == _MS_DESCEND_PICK:
            tgt  = self._mission_queue[self._mission_idx]
            # 수직 하강 보장: FLY_PICK 도달 시 고정된 XY 사용
            dx, dy = self._descend_xy if self._descend_xy else tgt["world_xy"]

            # 드리프트가 크면 현재 고도에서 XY만 보정한 뒤 다시 하강한다.
            if self._xy_error(dx, dy) > _PICK_TOL_XY:
                self._goto_xy_at_current_alt(dx, dy)
                self._grab_wait_cnt = 0
                return

            self._goto_vertical_only(_GRAB_ALT)
            self._descend_cnt += 1
            if self._at_xyz(dx, dy, _GRAB_ALT, _PICK_TOL_XY, _NAV_TOL_Z):
                self._grab_wait_cnt += 1
                if self._grab_wait_cnt >= _GRAB_WAIT:
                    ok = self._grab_pod(tgt["prim_path"])
                    if ok:
                        self._ascend_xy = None
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
            # 수직 상승만: 2.5m 안정화 전까지 수평 이동 금지
            self._goto_vertical_only(_HOVER_ALT)
            if self._hover_alt_ready():
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
            if not self._hover_alt_ready() or self._alt_stable_cnt < _ALT_STABLE_STEPS:
                self._goto_vertical_only(_HOVER_ALT)
                if self._hover_alt_ready():
                    self._alt_stable_cnt += 1
                else:
                    self._alt_stable_cnt = 0
                self._fly_drop_log = getattr(self, "_fly_drop_log", 0) + 1
                if self._fly_drop_log % 200 == 1:
                    print(f"[{self.name}] FLY_DROP Phase-A 고도 안정화중  "
                          f"pos=({ctrl.p[0]:.1f},{ctrl.p[1]:.1f},{ctrl.p[2]:.1f})  "
                          f"target_alt={_HOVER_ALT:.1f}")
                return

            # Phase B: hover altitude 도달 후 수평 이동만
            self._ascend_xy = None
            self._goto_xy_only(dx, dy)
            self._fly_drop_log = getattr(self, "_fly_drop_log", 0) + 1
            if self._fly_drop_log % 200 == 1:
                dist = math.hypot(ctrl.p[0] - dx, ctrl.p[1] - dy)
                print(f"[{self.name}] FLY_DROP Phase-B  "
                      f"pos=({ctrl.p[0]:.1f},{ctrl.p[1]:.1f},{ctrl.p[2]:.1f})  "
                      f"→ delivery=({dx:.1f},{dy:.1f})  dist={dist:.1f}m")
            if self._at_xyz(dx, dy, _HOVER_ALT, _DROP_TOL_XY, _ALT_GATE):
                self._align_stable_cnt += 1
            else:
                self._align_stable_cnt = 0

            if self._align_stable_cnt >= _ALIGN_STABLE_STEPS:
                self._descend_xy    = (dx, dy)
                self._grab_wait_cnt  = 0
                self._align_stable_cnt = 0
                ctrl.integral       = np.zeros(3)
                self._mission_state = _MS_DESCEND_DROP
                self._fly_drop_log  = 0
                print(f"[{self.name}] FLY_DROP → DESCEND_DROP")

        elif self._mission_state == _MS_DESCEND_DROP:
            dx, dy = self._descend_xy if self._descend_xy else (
                float(self._delivery_xyz[0]), float(self._delivery_xyz[1]))
            if self._xy_error(dx, dy) > _DROP_TOL_XY:
                self._goto_xy_at_current_alt(dx, dy)
                self._grab_wait_cnt = 0
                return

            self._goto_vertical_only(_DROP_ALT)
            if self._at_xyz(dx, dy, _DROP_ALT, _DROP_TOL_XY, _NAV_TOL_Z):
                self._grab_wait_cnt += 1
                if self._grab_wait_cnt >= _GRAB_WAIT:
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
            self._goto_vertical_only(_HOVER_ALT)
            if self._hover_alt_ready():
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
