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

robot_config.py 에서 등록:
    {
        "type"      : "drone",
        "name"      : "Drone_01",
        "spawn_xyz" : (0.0, 0.0, 0.07),
        "takeoff_alt": 1.5,
    }
"""
import sys
import os
import math
from pathlib import Path

import numpy as np
import carb
import omni.usd
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


class DroneAgent(BaseRobotAgent):
    """
    Iris 쿼드로터 에이전트.
    전방 RGB 뷰포트 + 경량 상태 HUD 만 사용 (깊이 카메라/히트맵 없음).
    """

    WARMUP_STEPS = 200

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        self._step_count = 0
        self._hud_step   = 0
        self.controller  = None
        self._cam_path   = None
        self._front_cam   = None
        self.hud         = None
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
        self.controller.takeoff_alt = float(self.cfg.get("takeoff_alt", 1.5))

        # FastMultirotor: no sensors, no propeller animation — eliminates ~7,500 DC calls/sec
        drone_cfg          = MultirotorConfig()
        drone_cfg.backends = [self.controller]
        drone_cfg.sensors  = []   # remove Barometer/IMU/Magnetometer/GPS (500 Hz overhead)

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
            from pxr import UsdGeom, Gf
            cam = UsdGeom.Camera.Define(stage, self._cam_path)
            cam.GetFocalLengthAttr().Set(CAM_FOCAL_LENGTH)
            xf = UsdGeom.Xformable(cam.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(CAM_MOUNT_FWD, 0.0, 0.0))
            # XYZ order: rotate X→Y→Z; maps camera -Z→world +X, camera +Y→world +Z
            xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))
        except Exception as e:
            carb.log_warn(f"[{self.name}] 카메라 prim 생성 실패: {e}")

        # 경량 HUD (ControlCenter 안에 임베드)
        self.hud = MinimalDroneHUD(self.controller, build_window=False)

        # RGB 카메라 센서 래핑 (ControlCenter 안에서 표시)
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

        # HUD 텍스트 라벨은 50Hz(HUD_UPDATE_N=10)면 충분
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
