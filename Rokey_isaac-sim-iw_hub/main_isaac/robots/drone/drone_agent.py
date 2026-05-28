"""
main_isaac/robots/drone/drone_agent.py
=======================================
Iris 쿼드로터 에이전트.
DroneApp 로직을 BaseRobotAgent 로 캡슐화.

상태머신:
    WARMUP → FLYING
    (HybridController 가 키보드 / 조이스틱 입력으로 비행 제어)

robot_config.py 에서 등록:
    {
        "type"      : "drone",
        "name"      : "Drone_01",
        "spawn_xyz" : (0.0, 0.0, 0.07),
        "environment": "Black Gridroom",   # 선택 (기본값 적용)
        "takeoff_alt": 1.5,               # 선택 (기본값 적용)
    }

중요: Multirotor 는 post_reset() 에서 생성합니다.
  Isaac Sim 5.x 의 World.reset() 이 clear_all_callbacks() 를 호출하므로
  setup() 에서 등록된 Vehicle 물리 콜백이 모두 제거됩니다.
  post_reset() 에서 생성하면 reset 이후 콜백이 등록되어 제거되지 않습니다.
"""
import sys
import os
from pathlib import Path

import numpy as np
import carb
import omni.usd
from scipy.spatial.transform import Rotation

import robot_config as C
from ..base_robot import BaseRobotAgent

# ── Pegasus Simulator 경로를 sys.path 에 추가 ──────────────────────────
if C.PEGASUS_SIM_DIR not in sys.path:
    sys.path.insert(0, C.PEGASUS_SIM_DIR)

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# ── drone_deps 경로를 sys.path 에 추가 ────────────────────────────────
if C.DRONE_DEPS_DIR not in sys.path:
    sys.path.insert(0, C.DRONE_DEPS_DIR)

from drone_config import (
    CAM_FOCAL_LENGTH, CAM_MOUNT_FWD,
    DEPTH_UPDATE_N,
)
from controller   import HybridController
from depth_camera import SoftwareDepthCamera, FrustumDrawer
from hud          import DroneHUD


class DroneAgent(BaseRobotAgent):
    """Iris 쿼드로터 에이전트 (키보드 + 조이스틱 비행 제어, 깊이 카메라, HUD)."""

    WARMUP_STEPS = 200   # physics step 수 (HUD·깊이 카메라 활성화 전 대기)

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        """
        world.reset() 이전 단계 — USD prim 은 생성하지 않습니다.
        Vehicle 의 physics 콜백은 post_reset() 에서 Multirotor 를 생성하여
        world.reset() 이후에 등록합니다. (reset 이 콜백을 지우므로)
        """
        self._step_count = 0
        self.controller  = None
        self._cam_path   = None
        carb.log_warn(f"[{self.name}] setup 완료 — 드론 prim 은 post_reset 에서 생성")

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        """
        world.reset() 이후 단계 — 여기서 Multirotor 를 생성합니다.
        이렇게 하면 Vehicle.__init__ 이 등록하는 physics 콜백이
        reset 후에 등록되어 삭제되지 않습니다.
        """
        spawn = self.spawn_xyz

        # ── PegasusInterface 에 공유 월드 연결 ────────────────────────
        pg        = PegasusInterface()
        pg._world = self.world

        # ── iris.usd 경로 진단 로그 ───────────────────────────────────
        iris_path = ROBOTS["Iris"]
        carb.log_warn(f"[{self.name}] ROBOTS['Iris'] = {iris_path}")
        carb.log_warn(f"[{self.name}] iris.usd exists = {os.path.isfile(iris_path)}")

        # ── 컨트롤러 생성 ─────────────────────────────────────────────
        self.controller             = HybridController()
        self.controller.takeoff_alt = float(self.cfg.get("takeoff_alt", 1.5))

        # ── 드론 생성 (physics 콜백이 여기서 등록됨 — reset 이후이므로 유지) ──
        drone_cfg          = MultirotorConfig()
        drone_cfg.backends = [self.controller]

        try:
            Multirotor(
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

        # ── 전방 RGB 카메라 prim 생성 ─────────────────────────────────
        self._cam_path = f"/World/{self.name}/body/FrontCamera"
        stage          = omni.usd.get_context().get_stage()
        try:
            from pxr import UsdGeom, Gf
            cam = UsdGeom.Camera.Define(stage, self._cam_path)
            cam.GetFocalLengthAttr().Set(CAM_FOCAL_LENGTH)
            xf = UsdGeom.Xformable(cam.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(CAM_MOUNT_FWD, 0.0, 0.0))
            xf.AddRotateYOp().Set(-90.0)
        except Exception as e:
            carb.log_warn(f"[{self.name}] 카메라 prim 생성 실패: {e}")

        # ── 깊이 카메라 + 프러스텀 오버레이 ──────────────────────────
        self.depth_cam = SoftwareDepthCamera()
        self.frustum   = FrustumDrawer()
        self.depth_cam.initialize()
        self.frustum.initialize()

        # ── HUD ───────────────────────────────────────────────────────
        self.hud = DroneHUD(self.controller)

        # ── RGB 뷰포트 창 열기 ────────────────────────────────────────
        if self._cam_path:
            try:
                from isaacsim.core.utils.viewports import create_viewport_for_camera
                create_viewport_for_camera(
                    viewport_name=f"{self.name} Front Camera",
                    camera_prim_path=self._cam_path,
                    width=480, height=360,
                )
            except Exception as e:
                carb.log_warn(f"[{self.name}] RGB 뷰포트 생성 실패: {e}")

        # controller.start() 는 Vehicle 의 timeline 콜백(sim_start_stop)이
        # 시뮬레이션 시작 시 자동으로 호출합니다.
        # 여기서 호출하지 않아도 됩니다 (중복 호출 방지).

        self._step_count = 0
        carb.log_warn(f"[{self.name}] post_reset 완료  spawn={spawn}")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, dt: float) -> None:
        self._step_count += 1
        if self._step_count < self.WARMUP_STEPS:
            return

        # post_reset 이 완료되지 않은 경우 (예외로 중단된 경우) 스킵
        if self.controller is None:
            return
        if not self.controller._received_first_state:
            return

        drone_pos = self.controller.p
        drone_R   = self.controller.R

        depth = self.depth_cam.capture(drone_pos, drone_R,
                                       update_every=DEPTH_UPDATE_N)
        if depth is None:
            return

        self.hud.update_depth(depth, float(drone_pos[2]))
        self.hud.update_status(
            self.controller.active_input,
            self.controller.is_airborne,
            drone_pos,
            self.controller.target_pos,
        )
        self.hud.update_map(
            drone_pos,
            drone_R,
            self.controller.target_pos if self.controller.is_airborne else None,
        )

        cam_origin = drone_pos + drone_R.apply(
            np.array([CAM_MOUNT_FWD, 0.0, 0.0]))
        self.frustum.update(
            cam_origin, self.depth_cam._rays_body, drone_R, self.depth_cam)

    # ── on_render_step ───────────────────────────────────────────────
    def on_render_step(self) -> None:
        pass
