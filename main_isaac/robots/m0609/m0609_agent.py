"""
main_isaac/robots/m0609/m0609_agent.py
=======================================
Doosan M0609 + 진공 흡착 그리퍼 에이전트.

그리퍼 형상:
    link_6
     └── suction_gripper/          ← _SUCTION_* 상수로 크기/색상 조정
          ├── stem   (원통 스템)
          ├── pad    (납작 흡착 패드)
          └── rim    (패드 테두리 링)

상태머신:
    MOVE_TO_HOME → Detecting (joint_5 회전)
    → SEARCH → SERVO → PICK_AND_PLACE → DONE
"""
import sys
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
import carb
import cv2
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.grippers import Gripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction

import robot_config as C
from ..base_robot import BaseRobotAgent

if C.M0609_SRC_DIR not in sys.path:
    sys.path.insert(0, C.M0609_SRC_DIR)

from m0609_rmpflow_controller import RMPFlowController
from m0609_pick_place_controller import PickPlaceController
from aruco_tracker import ArucoTracker
from visual_servo_controller import VisualServoController
from camera_viewer import CameraViewer
if C.USE_REALSENSE:
    from realsense_mount import attach_realsense_d455
    from wrist_camera import WristCamera


# ══════════════════════════════════════════════════════════════════════
#  ★ 흡착 그리퍼 형상 파라미터 — 여기만 수정하세요 ★
# ══════════════════════════════════════════════════════════════════════

# ── 스템 (link_6 플랜지와 패드를 잇는 원통 몸체) ─────────────────────
_SUCTION_STEM_RADIUS   = 0.022   # [m] 스템 반지름          ← 수정 가능
_SUCTION_STEM_HEIGHT   = 0.060   # [m] 스템 높이(길이)      ← 수정 가능

# ── 흡착 패드 (넓고 납작한 원판, 실제 흡착면) ───────────────────────
_SUCTION_PAD_RADIUS    = 0.045   # [m] 패드 반지름          ← 수정 가능
_SUCTION_PAD_HEIGHT    = 0.012   # [m] 패드 두께            ← 수정 가능

# ── 림 (패드 가장자리 고무 테두리) ───────────────────────────────────
_SUCTION_RIM_RADIUS    = 0.048   # [m] 림 반지름 (패드보다 크게)
_SUCTION_RIM_HEIGHT    = 0.004   # [m] 림 두께

# ── 마운트 오프셋 (link_6 기준, 단위 m) ──────────────────────────────
# Z 방향이 로봇 툴 축. 값을 키우면 그리퍼가 더 아래(앞)로 내려감.
_SUCTION_MOUNT_OFFSET  = (0.0, 0.0, 0.0)   # (x, y, z)      ← 수정 가능

# ── 카메라 브라켓 (스템 측면에 돌출되는 직육면체 마운트) ──────────────
# 브라켓 위에 RealSense D455 가 부착됩니다.
_CAM_BRACKET_SIZE      = (0.040, 0.030, 0.020)  # (x, y, z) 크기 [m]  ← 수정 가능
# 스템 중심 기준 브라켓 중심 위치.  x: 스템 측면 바깥으로, z: 스템 중간 높이
_CAM_BRACKET_OFFSET    = (0.0, 0.030, 0.020)  # (x, y, z) [m]       ← 수정 가능

# ── 색상 (R, G, B  0~1) ──────────────────────────────────────────────
_SUCTION_COLOR_BODY    = Gf.Vec3f(0.30, 0.30, 0.30)  # 스템 (금속 회색)
_SUCTION_COLOR_PAD     = Gf.Vec3f(0.10, 0.10, 0.10)  # 패드 (검정 고무)
_SUCTION_COLOR_RIM     = Gf.Vec3f(0.05, 0.05, 0.05)  # 림   (검정)
_SUCTION_COLOR_BRACKET = Gf.Vec3f(0.20, 0.20, 0.22)  # 브라켓 (어두운 회색)

# 참고: 스템 + 패드 = link_6 플랜지에서 흡착면까지 총 길이
# _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT ≈ 0.072 m

# ══════════════════════════════════════════════════════════════════════
#  더미 그리퍼 (조인트 없음 — 흡착은 FixedJoint 로 처리)
# ══════════════════════════════════════════════════════════════════════

class NoOpGripper(Gripper):
    def __init__(self, end_effector_prim_path: str) -> None:
        super().__init__(end_effector_prim_path=end_effector_prim_path)

    def initialize(self, physics_sim_view=None, **kwargs) -> None:
        super().initialize(physics_sim_view=physics_sim_view)

    def post_reset(self) -> None:
        pass

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def set_default_state(self, *args, **kwargs):
        pass

    def get_default_state(self, *args, **kwargs):
        return None

    def forward(self, *args, **kwargs) -> ArticulationAction:
        return ArticulationAction()


# ══════════════════════════════════════════════════════════════════════
#  로봇/컨트롤러 고정 파라미터
# ══════════════════════════════════════════════════════════════════════

_EE_LINK       = "link_6"

# 카메라 위치/방향 — 브라켓 중심 기준 (브라켓이 카메라 parent prim)
_CAM_T         = (0.020, 0.02, 0.0)   # 브라켓 X+ 면(바깥쪽)에 배치  ← 수정 가능
_CAM_RPY       = (0.0, -90.0, 90.0)  # 기존 방향 유지               ← 수정 가능
_CAM_RES       = (640, 480)
_CAM_EXTRA_RPY = (0.0, 0.0, 90.0)
_CAM_FX, _CAM_FY = 500.0, 500.0
_CAM_CX, _CAM_CY = _CAM_RES[0] / 2.0, _CAM_RES[1] / 2.0
_DIST_COEFFS   = [0.0] * 12

_HOME_JOINTS   = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
_HOME_DEG      = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])  # joint_4 -90° → EE 아래 향함
_HOME_TOL_DEG  = 1.0
_SERVO_PX2WLD  = np.array([[0.0, -1.0], [-1.0, 0.0]])

_EE_INIT_H     = 0.25
_EE_OFFSET     = np.array([0.0, 0.0, 0.11])
_EVENTS_DT     = [0.008, 0.005, 0.02, 0.02, 0.005, 0.01, 0.005, 0.05, 0.008, 0.08]

# ── 흡착/해제 근접 임계값 ─────────────────────────────────────────────
# EE(link_6)와 큐브 픽업 위치 사이 거리가 이 값 이하일 때 큐브를 흡착
# 그리퍼 총 길이(_SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT) + 여유 ← 수정 가능
_ATTACH_REACH   = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT + 0.045   # [m]
# EE XY와 목표(place) 위치 XY 거리가 이 값 이하일 때 큐브를 해제    ← 수정 가능
_DETACH_XY_TOL  = 0.08   # [m]

_CUBE_EDGE     = 0.05
_ARUCO_LEN     = 0.045 * (600 / 720)   # ≈ 0.0375 m  (USDA 박스 고정 크기 기준)
_ARUCO_Z_OFF   = _CUBE_EDGE / 2.0 + 0.001  # 0.026 m

# box_type → ArUco ID 매핑
_ARUCO_ID_FROM_BOX = {
    "green_id0": 0,
    "red_id1"  : 1,
    "blue_id2" : 2,
}

_T_GL2CV       = np.diag([1.0, -1.0, -1.0, 1.0])
_FSM_EVERY     = 10



# ══════════════════════════════════════════════════════════════════════
#  M0609Agent
# ══════════════════════════════════════════════════════════════════════

class M0609Agent(BaseRobotAgent):
    """M0609 + 진공 흡착 그리퍼 에이전트 (ArUco 시각 서보 + 픽 앤 플레이스)."""

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        spawn    = self.spawn_xyz
        yaw_deg  = float(self.cfg.get("spawn_yaw", 0.0))
        scale    = float(self.cfg.get("scale", 1.0))
        box_type = self.cfg.get("box_type", "red_id1")
        goal_xyz = np.array(self.cfg["goal_xyz"])

        self._aruco_id   = _ARUCO_ID_FROM_BOX.get(box_type, 1)
        self._aruco_len  = _ARUCO_LEN
        self._goal_pos   = goal_xyz

        # ScaleOp은 visual만 스케일. physics 파라미터는 URDF 원본 단위 그대로.
        self._ee_init_h   = _EE_INIT_H
        self._ee_offset   = _EE_OFFSET
        self._attach_reach = _ATTACH_REACH * scale

        stage = omni.usd.get_context().get_stage()

        # ── URDF import ─────────────────────────────────────────────
        # distance_scale=1.0 고정 (전역 캐시 공유 버그 방지)
        robot_root, artic_path = self._import_urdf(C.M0609_URDF, fix_base=True)

        # 임포트 직후 /World/{name} 으로 이동 → 각 로봇이 독립 경로 확보
        # (setup() 은 physics 시작 전이므로 MovePrim 안전)
        target_root = f"/World/{self.name}"
        if robot_root != target_root:
            omni.kit.commands.execute("MovePrim",
                                      path_from=robot_root,
                                      path_to=target_root)
            # artic_path 도 새 경로에 맞게 갱신
            artic_path = target_root + artic_path[len(robot_root):]
            robot_root = target_root
        self._robot_root = robot_root

        # 개별 Xform: translate → rotateZ(yaw) → scale
        root_prim = stage.GetPrimAtPath(robot_root)
        xf = UsdGeom.Xformable(root_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(float(spawn[0]), float(spawn[1]), float(spawn[2])))
        if abs(yaw_deg) > 1e-6:
            xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(yaw_deg)
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(scale, scale, scale))

        # EE 경로 검색
        robot_ee = (self._find_prim(robot_root, _EE_LINK)
                    or f"{artic_path}/{_EE_LINK}")

        for _ in range(10):
            simulation_app_update()

        # NoOpGripper + SingleManipulator
        gripper = NoOpGripper(end_effector_prim_path=robot_ee)
        self._robot = self.world.scene.add(
            SingleManipulator(
                prim_path=artic_path,
                name=self.name,
                end_effector_prim_path=robot_ee,
                gripper=gripper,
            )
        )

        # ── 흡착 그리퍼 형상 (ScaleOp 로 자동 확대됨) ──────────────
        self._suction_path, cam_mount_path = self._build_suction_gripper(stage, robot_ee)
        self._grip_body_path = self._suction_path
        print(f"[{self.name}] 흡착 그리퍼 생성 완료: {self._suction_path}")

        # ── world_setup 에서 로드된 ArUco USDA 박스 참조 ───────────
        box_prim_path = f"/World/ArUcoBoxes/{box_type}"
        self._cube = self.world.scene.add(
            SingleRigidPrim(
                prim_path=box_prim_path,
                name=f"{self.name}_cube",
            )
        )

        # 카메라 — USE_REALSENSE=True 일 때만 부착
        self._rs_path   = None
        self._wrist_cam = None
        if C.USE_REALSENSE:
            self._setup_camera(stage, cam_mount_path)

        self._init_state(spawn)
        print(f"[{self.name}] setup 완료  spawn={spawn}  yaw={yaw_deg}°  "
              f"box={box_prim_path}  goal={goal_xyz}")

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        # RealSense / 흡착 그리퍼 물리 비활성화
        self._disable_rs_physics()
        self._disable_suction_physics()

        self._robot.initialize()
        yaw_rad = np.deg2rad(float(self.cfg.get("spawn_yaw", 0.0)))
        c, s = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
        self._robot.set_world_pose(
            position=np.array(self.spawn_xyz, dtype=np.float64),
            orientation=np.array([c, 0.0, 0.0, s]),   # wxyz — Z축 회전
        )
        self._robot.gripper.initialize(
            physics_sim_view=self.world.physics_sim_view,
            articulation_apply_action_func=self._robot.apply_action,
        )

        if self._wrist_cam is not None:
            self._wrist_cam.initialize()
            self._wrist_cam.camera.set_opencv_pinhole_properties(
                cx=_CAM_CX, cy=_CAM_CY, fx=_CAM_FX, fy=_CAM_FY,
                pinhole=_DIST_COEFFS,
            )

        K = np.array([[_CAM_FX, 0, _CAM_CX],
                      [0, _CAM_FY, _CAM_CY],
                      [0, 0, 1.0]], dtype=np.float64)

        self._tracker = ArucoTracker(
            marker_length=self._aruco_len, target_id=self._aruco_id, K=K)
        self._servo = VisualServoController(
            image_size=_CAM_RES, pixel_to_world_xy=_SERVO_PX2WLD)
        self._viewer = CameraViewer(enabled=True)

        self._cspace = RMPFlowController(
            name=f"{self.name}_rmpflow",
            robot_articulation=self._robot,
            urdf_path=C.M0609_URDF,
            robot_description_path=C.M0609_DESC_YAML,
            rmpflow_config_path=C.M0609_RMPFLOW_CFG,
            end_effector_frame_name=_EE_LINK,
        )
        self._pp = PickPlaceController(
            name=f"{self.name}_pp",
            gripper=self._robot.gripper,
            robot_articulation=self._robot,
            end_effector_initial_height=self._ee_init_h,
            events_dt=_EVENTS_DT,
            urdf_path=C.M0609_URDF,
            robot_description_path=C.M0609_DESC_YAML,
            rmpflow_config_path=C.M0609_RMPFLOW_CFG,
            end_effector_frame_name=_EE_LINK,
        )

        self._home_idx = self._find_joint_indices(self._robot, _HOME_JOINTS)
        self._home_pos = np.deg2rad(_HOME_DEG)
        self._home_tol = np.deg2rad(_HOME_TOL_DEG)

        self._state = "MOVE_TO_HOME"
        print(f"[{self.name}] post_reset 완료")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, _dt: float) -> None:
        if not hasattr(self, "_robot") or self._robot is None:
            return
        self._phys_cnt += 1
        if self._gripped:
            self._update_gripped_cube()
        if self._phys_cnt % _FSM_EVERY != 0:
            return
        self._run_fsm()

    def on_render_step(self) -> None:
        if hasattr(self, "_viewer") and self._viewer is not None:
            rgb = self._wrist_cam.get_rgb() if self._wrist_cam else None
            det = None
            if rgb is not None and hasattr(self, "_tracker"):
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                det = self._tracker.detect(bgr)
            if hasattr(self, "_det"):
                det = self._det
            label = self._display_label()
            self._viewer.update(rgb, det, state_str=label)

    # ══════════════════════════════════════════════════════════════════
    #  흡착 그리퍼 생성 / 물리 비활성화
    # ══════════════════════════════════════════════════════════════════

    def _build_suction_gripper(self, stage, ee_path: str) -> tuple:
        """
        진공 흡착 그리퍼 시각 형상을 link_6 하위 prim으로 생성.
        크기는 원본 상수 그대로 — robot root의 ScaleOp가 USD 계층을 통해 자동 적용됨.
        반환값: (루트 prim 경로, cam_mount prim 경로)
        """
        root_path = f"{ee_path}/suction_gripper"

        root_xf = UsdGeom.Xform.Define(stage, root_path)
        xf = UsdGeom.Xformable(root_xf.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*_SUCTION_MOUNT_OFFSET))

        def _cylinder(name: str, radius: float, height: float,
                      z_center: float, color: Gf.Vec3f) -> None:
            p = f"{root_path}/{name}"
            cyl = UsdGeom.Cylinder.Define(stage, p)
            cyl.CreateRadiusAttr(float(radius))
            cyl.CreateHeightAttr(float(height))
            cyl.CreateAxisAttr("Z")
            c_xf = UsdGeom.Xformable(cyl.GetPrim())
            c_xf.ClearXformOpOrder()
            c_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(z_center)))
            cyl.GetPrim().CreateAttribute(
                "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
            ).Set([color])

        _cylinder("stem",
                  _SUCTION_STEM_RADIUS, _SUCTION_STEM_HEIGHT,
                  _SUCTION_STEM_HEIGHT / 2.0, _SUCTION_COLOR_BODY)

        pad_z = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT / 2.0
        _cylinder("pad",
                  _SUCTION_PAD_RADIUS, _SUCTION_PAD_HEIGHT,
                  pad_z, _SUCTION_COLOR_PAD)

        # ── 카메라 브라켓 ───────────────────────────────────────────
        bracket_path = f"{root_path}/cam_bracket"
        bx, by, bz = _CAM_BRACKET_SIZE
        box = UsdGeom.Cube.Define(stage, bracket_path)
        box.CreateSizeAttr(1.0)
        b_xf = UsdGeom.Xformable(box.GetPrim())
        b_xf.ClearXformOpOrder()
        b_xf.AddTranslateOp().Set(Gf.Vec3d(*_CAM_BRACKET_OFFSET))
        b_xf.AddScaleOp().Set(Gf.Vec3f(bx, by, bz))
        box.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
        ).Set([_SUCTION_COLOR_BRACKET])

        # ── 카메라 마운트 (스케일 없는 Xform) ───────────────────────
        cam_mount_path = f"{root_path}/cam_mount"
        cm = UsdGeom.Xform.Define(stage, cam_mount_path)
        cm_xf = UsdGeom.Xformable(cm.GetPrim())
        cm_xf.ClearXformOpOrder()
        cm_xf.AddTranslateOp().Set(Gf.Vec3d(*_CAM_BRACKET_OFFSET))

        rim_z = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT - _SUCTION_RIM_HEIGHT / 2.0
        _cylinder("rim",
                  _SUCTION_RIM_RADIUS, _SUCTION_RIM_HEIGHT,
                  rim_z, _SUCTION_COLOR_RIM)

        return root_path, cam_mount_path

    def _disable_suction_physics(self) -> None:
        """흡착 그리퍼 prim 에 물리 API가 붙어있으면 모두 제거."""
        if not hasattr(self, "_suction_path") or self._suction_path is None:
            return
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self._suction_path)
        if not root.IsValid():
            return
        for p in Usd.PrimRange(root):
            for api in (UsdPhysics.RigidBodyAPI,
                        UsdPhysics.CollisionAPI,
                        UsdPhysics.MassAPI):
                if p.HasAPI(api):
                    p.RemoveAPI(api)

    # ══════════════════════════════════════════════════════════════════
    #  내부 초기화 / 카메라
    # ══════════════════════════════════════════════════════════════════

    def _init_state(self, spawn):
        self._phys_cnt          = 0
        self._state             = "MOVE_TO_HOME"
        self._servo_hold_z      = None
        self._servo_hold_ori    = None
        self._pick_world_pos    = None
        self._prev_ev           = -1
        self._cur_ev            = -1
        self._gripped            = False
        self._grab_offset_local  = None   # EE 로컬 프레임 기준 cube 오프셋
        self._det               = None
        # EE 서보 허용 범위 — 스폰 위치 기준 ±2 m
        self._ws_x = (float(spawn[0]) - 2.0, float(spawn[0]) + 2.0)
        self._ws_y = (float(spawn[1]) - 2.0, float(spawn[1]) + 2.0)

    def _setup_camera(self, stage, cam_parent_path: str):
        """RealSense D455 를 cam_parent_path prim 에 부착.
        위치 오프셋은 원본 그대로 — robot root ScaleOp가 USD 계층으로 자동 적용됨."""
        if not stage.GetPrimAtPath(cam_parent_path).IsValid():
            carb.log_warn(f"[{self.name}] camera parent 없음: {cam_parent_path}")
            return
        rs_path = attach_realsense_d455(
            parent_prim_path=cam_parent_path,
            child_name="realsense_d455",
            translation=_CAM_T,
            rpy_deg=_CAM_RPY,
        )
        self._rs_path = rs_path

        for _ in range(5):
            simulation_app_update()

        for p in Usd.PrimRange(stage.GetPrimAtPath(rs_path)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

        ov_path = None
        for p in Usd.PrimRange(stage.GetPrimAtPath(rs_path)):
            if p.GetName() == "Camera_OmniVision_OV9782_Color":
                ov_path = str(p.GetPath())
                break

        if ov_path:
            from pxr import Vt
            cp  = stage.GetPrimAtPath(ov_path)
            xf  = UsdGeom.Xformable(cp)
            existing = [op.GetOpName() for op in xf.GetOrderedXformOps()]
            rop = xf.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat, opSuffix="extra")
            rop.Set(float(_CAM_EXTRA_RPY[2]))
            cp.GetAttribute("xformOpOrder").Set(
                Vt.TokenArray(existing + [rop.GetOpName()])
            )
            self._wrist_cam = WristCamera.from_existing_prim(
                prim_path=ov_path, resolution=_CAM_RES)
        else:
            self._wrist_cam = WristCamera(
                parent_prim_path=rs_path,
                name=f"{self.name}_wrist",
                resolution=_CAM_RES,
                rpy_deg=_CAM_EXTRA_RPY,
            )
        print(f"[{self.name}] 카메라: {self._wrist_cam._prim_path}")

    def _disable_rs_physics(self):
        if not self._rs_path:
            return
        stage = omni.usd.get_context().get_stage()
        root  = stage.GetPrimAtPath(self._rs_path)
        if not root.IsValid():
            return
        for p in Usd.PrimRange(root):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

    # ══════════════════════════════════════════════════════════════════
    #  유틸리티
    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _import_urdf(urdf_path: str, fix_base: bool):
        _, import_cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        import_cfg.merge_fixed_joints           = False
        import_cfg.convex_decomp                = True
        import_cfg.import_inertia_tensor        = True
        import_cfg.fix_base                     = fix_base
        import_cfg.distance_scale               = 1.0  # 항상 1.0 — 스케일은 root ScaleOp로 처리
        import_cfg.default_drive_type           = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_cfg.default_drive_strength       = 1e10
        import_cfg.default_position_drive_damping = 1e5
        _, artic_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_cfg,
            get_articulation_root=True,
        )
        if artic_path is None:
            raise RuntimeError(f"URDF import 실패: {urdf_path}")
        robot_root = artic_path.rsplit("/", 1)[0] or artic_path
        return robot_root, artic_path

    @staticmethod
    def _find_prim(root_path: str, name: str):
        stage = omni.usd.get_context().get_stage()
        root  = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return None
        for p in Usd.PrimRange(root):
            if p.GetName() == name:
                return str(p.GetPath())
        return None

    @staticmethod
    def _find_joint_index(robot, jname: str, fallback: int = 0) -> int:
        for i, n in enumerate(robot.dof_names):
            if n == jname or n.endswith(jname):
                return i
        return fallback

    @staticmethod
    def _find_joint_indices(robot, jnames):
        return np.array([M0609Agent._find_joint_index(robot, n, i)
                         for i, n in enumerate(jnames)])

    # ══════════════════════════════════════════════════════════════════
    #  동기화 / 제어
    # ══════════════════════════════════════════════════════════════════

    def _aruco_to_world(self, det, cam_path):
        if det.rvec is None or det.tvec is None:
            return None
        Rcm, _ = cv2.Rodrigues(det.rvec)
        T_cm = np.eye(4)
        T_cm[:3,:3] = Rcm
        T_cm[:3, 3] = det.tvec.reshape(3)
        T_wg_gl = _get_world_T(cam_path)
        T_wg_cv = T_wg_gl @ _T_GL2CV
        return (T_wg_cv @ T_cm)[:3, 3]

    def _apply_ee(self, target_pos, ori=None):
        actions = self._cspace.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=ori,
        )
        self._robot.apply_action(actions)

    def _attach_cube(self):
        """
        흡착: physics/scene graph 변경 없이 EE 오프셋만 기록.
        매 step set_world_pose() 로 큐브를 EE에 붙임 → tensor view 안전.
        """
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        cube_pos, _  = self._cube.get_world_pose()
        R_ee = _quat_wxyz_to_R(ee_q)
        self._grab_offset_local = R_ee.T @ (cube_pos - ee_pos)
        print(f"[{self.name}] 흡착 완료")

    def _update_gripped_cube(self):
        """
        매 physics step 호출.
        physics_sim_view 의 set_world_pose / set_linear_velocity 는
        tensor view 내부 write API 이므로 view invalidation 없음.
        """
        if self._grab_offset_local is None:
            return
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        R_ee   = _quat_wxyz_to_R(ee_q)
        target = ee_pos + R_ee @ self._grab_offset_local
        self._cube.set_world_pose(position=target)
        self._cube.set_linear_velocity(np.zeros(3))
        self._cube.set_angular_velocity(np.zeros(3))

    def _detach_cube(self):
        """
        해제: 추적만 중단 — 큐브가 현재 위치에서 중력으로 낙하.
        physics/scene graph 변경 없음 → tensor view 안전.
        """
        self._grab_offset_local = None
        print(f"[{self.name}] 흡착 해제 (cube 낙하)")

    def _display_label(self) -> str:
        if self._state == "PICK_AND_PLACE":
            if self._cur_ev <= 3:
                return "Picking..."
            if self._cur_ev <= 6:
                return "Moving..."
            return "Placing..."
        return "Detecting..."

    def _reset_for_next_cycle(self):
        """배치 완료 후 팔 복귀 → 대기 사이클 시작."""
        self._gripped            = False
        self._grab_offset_local  = None
        self._servo_hold_z       = None
        self._servo_hold_ori     = None
        self._pick_world_pos     = None
        self._prev_ev            = -1
        self._cur_ev             = -1
        self._det                = None
        self._pp.reset()
        self._state = "RETURN_TO_WATCH"
        print(f"[{self.name}] → RETURN_TO_WATCH")

    # ══════════════════════════════════════════════════════════════════
    #  상태머신
    # ══════════════════════════════════════════════════════════════════

    def _run_fsm(self):
        robot  = self._robot
        joints = robot.get_joint_positions()
        ee_pos, ee_ori = robot.end_effector.get_world_pose()
        cur_xy = ee_pos[:2].copy()

        rgb = self._wrist_cam.get_rgb() if self._wrist_cam else None
        det = None
        if rgb is not None:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            det = self._tracker.detect(bgr)
        self._det = det

        # ── MOVE_TO_HOME ─────────────────────────────────────────────
        if self._state == "MOVE_TO_HOME":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)
            err = np.max(np.abs(joints[self._home_idx] - self._home_pos))
            if err < self._home_tol:
                self._state = "Detecting"
                print(f"[{self.name}] → Detecting")

        # ── Detecting ────────────────────────────────────────────────
        # HOME_DEG 자세 그대로 유지하며 ArUco 감지 대기.
        # 다른 각도로 이동하지 않음 — 발견 즉시 SERVO 진입.
        elif self._state == "Detecting":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)
            if det is not None and det.found:
                self._servo_hold_z   = float(ee_pos[2])
                self._servo_hold_ori = ee_ori.copy()
                self._servo.reset()
                self._state = "SERVO"
                print(f"[{self.name}] Detecting → SERVO 발견")

        # ── RETURN_TO_WATCH ──────────────────────────────────────────
        # 픽 앤 플레이스 완료 후 홈 복귀 → Detecting 재개
        elif self._state == "RETURN_TO_WATCH":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)
            err = np.max(np.abs(joints[self._home_idx] - self._home_pos))
            if err < self._home_tol:
                self._state = "Detecting"
                print(f"[{self.name}] 홈 복귀 완료 → Detecting")

        # ── SERVO ─────────────────────────────────────────────────────
        elif self._state == "SERVO":
            if det is not None:
                tgt_xy, _ = self._servo.update(cur_xy, det)
            else:
                self._servo.reset()
                tgt_xy = cur_xy.copy()

            tgt_xy[0] = np.clip(tgt_xy[0], *self._ws_x)
            tgt_xy[1] = np.clip(tgt_xy[1], *self._ws_y)
            self._apply_ee(np.array([tgt_xy[0], tgt_xy[1], self._servo_hold_z]),
                           self._servo_hold_ori)

            if self._servo.is_locked() and det is not None:
                mw = self._aruco_to_world(det, self._wrist_cam._prim_path)
                if mw is None:
                    self._servo.reset()
                else:
                    self._pick_world_pos = np.array([mw[0], mw[1],
                                                     mw[2] - _ARUCO_Z_OFF])
                    self._pp.reset()
                    self._prev_ev = -1
                    self._state   = "PICK_AND_PLACE"
                    print(f"[{self.name}] → PICK_AND_PLACE  pick={self._pick_world_pos.round(3)}")

        # ── PICK_AND_PLACE ────────────────────────────────────────────
        elif self._state == "PICK_AND_PLACE":
            actions = self._pp.forward(
                picking_position=self._pick_world_pos,
                placing_position=self._goal_pos,
                current_joint_positions=joints,
                end_effector_offset=self._ee_offset,
            )
            robot.apply_action(actions)
            ev = getattr(self._pp, "_event", -1)
            self._cur_ev = ev

            if not self._gripped:
                # EE ↔ 큐브 거리 < 그리퍼 도달 범위 → 흡착
                dist = float(np.linalg.norm(ee_pos - self._pick_world_pos))
                if dist < self._attach_reach:
                    self._attach_cube()
                    self._gripped = True
                    print(f"[{self.name}] 흡착! dist={dist:.3f}m")
            else:
                # EE XY ↔ 목표 XY 거리 < 임계 → 해제
                xy_dist = float(np.linalg.norm(ee_pos[:2] - self._goal_pos[:2]))
                if xy_dist < _DETACH_XY_TOL:
                    self._detach_cube()
                    self._gripped = False

            self._prev_ev = ev
            if self._pp.is_done():
                print(f"[{self.name}] 배치 완료 → Detecting 재시작")
                self._reset_for_next_cycle()


# ══════════════════════════════════════════════════════════════════════
#  모듈 레벨 헬퍼
# ══════════════════════════════════════════════════════════════════════

def simulation_app_update():
    omni.kit.app.get_app().update()


def _quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])



def _get_world_T(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim  = stage.GetPrimAtPath(prim_path)
    mat   = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    return np.array(mat, dtype=np.float64).T


