"""
main_isaac/robots/m0609_agent.py
=================================
Doosan M0609 + OnRobot RG2 에이전트.
m0609_pick_place_visual.py 로직을 BaseRobotAgent 로 캡슐화.

상태머신:
    MOVE_TO_HOME → Detecting (joint_5 회전)
    → SEARCH → SERVO → PICK_AND_PLACE → DONE
"""
import sys
import os
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
import carb
import cv2
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
# RobotAssembler 는 extension 수동 enable 후에만 import 가능 → setup() 내부에서 지연 import

import robot_config as C
from .base_robot import BaseRobotAgent

# 외부 모듈 경로 추가
if C.M0609_SRC_DIR not in sys.path:
    sys.path.insert(0, C.M0609_SRC_DIR)

from m0609_rmpflow_controller import RMPFlowController
from m0609_pick_place_controller import PickPlaceController
from realsense_mount import attach_realsense_d455
from wrist_camera import WristCamera
from aruco_tracker import ArucoTracker
from visual_servo_controller import VisualServoController
from camera_viewer import CameraViewer

# ── 고정 파라미터 ─────────────────────────────────────────────────────
_EE_LINK       = "link_6"
_GRIP_BASE     = "angle_bracket"
_GRIP_GRASP    = "gripper_body"

_CAM_T         = (0.0, 0.045, 0.05)
_CAM_RPY       = (0.0, -90.0, 90.0)
_CAM_RES       = (640, 480)
_CAM_EXTRA_RPY = (0.0, 0.0, 90.0)
_CAM_FX, _CAM_FY = 500.0, 500.0
_CAM_CX, _CAM_CY = _CAM_RES[0] / 2.0, _CAM_RES[1] / 2.0
_DIST_COEFFS   = [0.0] * 12

_HOME_JOINTS   = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
_HOME_DEG      = np.array([0.0, 0.0, 70.0, 0.0, 0.0, 0.0])
_HOME_TOL_DEG  = 1.0
_SPIN_J5_DEG   = 90.0
_SPIN_DUR      = 4.0
_CTRL_DT       = 1.0 / 60.0

_LIFT_RATE     = 0.03
_LIFT_Z_MAX    = 0.75
_SERVO_PX2WLD  = np.array([[0.0, -1.0], [-1.0, 0.0]])

_EE_INIT_H     = 0.25
_EE_OFFSET     = np.array([0.0, 0.0, 0.2])
_EVENTS_DT     = [0.008, 0.005, 0.02, 0.02, 0.005, 0.01, 0.005, 0.05, 0.008, 0.08]

_CUBE_EDGE     = 0.05
_ARUCO_ID      = 1
_ARUCO_PLANE   = 0.045
_ARUCO_TEX_R   = 600 / 720
_ARUCO_LEN     = _ARUCO_PLANE * _ARUCO_TEX_R
_ARUCO_Z_OFF   = _CUBE_EDGE / 2.0 + 0.001

_T_GL2CV       = np.diag([1.0, -1.0, -1.0, 1.0])

# physics step 마다 상태머신을 실행하면 과부하 → 10 step 마다 한 번 실행 (50 Hz)
_FSM_EVERY     = 10


class M0609Agent(BaseRobotAgent):
    """M0609 로봇 에이전트 (ArUco 시각 서보 + 픽 앤 플레이스)."""

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        spawn    = self.spawn_xyz
        cube_xyz = np.array(self.cfg.get("cube_xyz",
                            (spawn[0]+0.4, spawn[1]+0.2, _CUBE_EDGE/2)))
        goal_xyz = np.array(self.cfg.get("goal_xyz",
                            (spawn[0]+0.55, spawn[1]-0.35, 0.0)))

        stage = omni.usd.get_context().get_stage()

        # URDF import
        robot_root, _ = self._import_urdf(C.M0609_URDF, fix_base=True)
        grip_root,  _ = self._import_urdf(C.RG2_URDF,   fix_base=False)
        self._robot_root = robot_root

        # EE / 그리퍼 경로 검색
        robot_ee = (self._find_prim(robot_root, _EE_LINK)
                    or f"{robot_root}/{_EE_LINK}")
        grip_base = (self._find_prim(grip_root, _GRIP_BASE)
                     or f"{grip_root}/{_GRIP_BASE}")

        # RobotAssembler extension enable + 지연 import
        _mgr = omni.kit.app.get_app().get_extension_manager()
        _mgr.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
        from isaacsim.robot_setup.assembler import RobotAssembler  # noqa: PLC0415

        # RobotAssembler 로 결합
        assembler = RobotAssembler()
        assembler.begin_assembly(stage, robot_root, robot_ee,
                                 grip_root, grip_base,
                                 "Gripper", f"{self.name}_rg2")
        assembler.assemble()
        assembler.finish_assemble()
        print(f"[{self.name}] 어셈블리 완료")

        # 스폰 위치는 world.reset() + robot.initialize() 이후 set_world_pose 로 적용
        # (fix_base FixedJoint 앵커가 origin 에 박히기 때문에 여기서 XformOp 은 무의미)

        robot_ee = self._find_prim(robot_root, _EE_LINK)
        self._grip_body_path = self._find_prim(robot_root, _GRIP_GRASP)

        # 그리퍼 드라이브 강도 설정
        for jn in ["finger_joint", "right_inner_knuckle_joint"]:
            jp = self._find_prim(robot_root, jn)
            if jp:
                jp_prim = stage.GetPrimAtPath(jp)
                for dt in ["angular", "linear"]:
                    drv = UsdPhysics.DriveAPI.Get(jp_prim, dt)
                    if drv:
                        drv.GetMaxForceAttr().Set(100.0)
                        drv.GetStiffnessAttr().Set(1000.0)
                        drv.GetDampingAttr().Set(50.0)

        for _ in range(10):
            simulation_app_update()

        # ParallelGripper + SingleManipulator
        gripper = ParallelGripper(
            end_effector_prim_path=robot_ee,
            joint_prim_names=["finger_joint", "right_inner_knuckle_joint"],
            joint_opened_positions=np.array([0.0, 0.0]),
            joint_closed_positions=np.array([0.8, 0.8]),
            action_deltas=np.array([-0.5, -0.5]),
        )
        self._robot = self.world.scene.add(
            SingleManipulator(
                prim_path=robot_root,
                name=self.name,
                end_effector_prim_path=robot_ee,
                gripper=gripper,
            )
        )

        # 마찰 재질
        cube_mat = PhysicsMaterial(
            prim_path=f"/World/PhysMat_{self.name}_cube",
            static_friction=1.2, dynamic_friction=1.0, restitution=0.0)
        finger_mat = PhysicsMaterial(
            prim_path=f"/World/PhysMat_{self.name}_finger",
            static_friction=4.0, dynamic_friction=3.0, restitution=0.0)

        for ln in ["left_inner_finger","right_inner_finger",
                   "left_inner_knuckle","right_inner_knuckle",
                   "left_outer_knuckle","right_outer_knuckle"]:
            lp = self._find_prim(robot_root, ln)
            if lp:
                SingleGeometryPrim(prim_path=lp,
                                   name=f"{self.name}_geom_{ln}").apply_physics_material(
                    finger_mat)

        # 큐브 + ArUco 마커
        self._cube = self.world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{self.name}_cube",
                name=f"{self.name}_cube",
                position=cube_xyz,
                scale=np.array([_CUBE_EDGE, _CUBE_EDGE, _CUBE_EDGE]),
                color=np.array([0.85, 0.85, 0.85]),
                mass=0.01,
                physics_material=cube_mat,
            )
        )
        self.world.scene.add(
            VisualCuboid(
                prim_path=f"/World/{self.name}_goal",
                name=f"{self.name}_goal",
                position=goal_xyz,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        self._goal_pos    = goal_xyz
        self._marker_path = f"/World/{self.name}_marker"
        tex_path = os.path.join(C.ARUCO_TEXTURE_DIR, f"aruco_id{_ARUCO_ID}.png")
        _add_aruco_plane(stage, self._marker_path, tex_path, _ARUCO_PLANE,
                         (cube_xyz[0], cube_xyz[1], cube_xyz[2] + _ARUCO_Z_OFF))

        # 카메라 설치
        self._rs_path   = None
        self._wrist_cam = None
        self._setup_camera(stage, robot_root)

        # 그립 조인트 경로 (잡기/놓기 시 FixedJoint)
        self._grip_joint = f"/World/{self.name}_grip_joint"

        # 내부 상태 초기화
        self._init_state(spawn)
        print(f"[{self.name}] setup 완료  spawn={spawn}  cube={cube_xyz}  goal={goal_xyz}")

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        # RealSense physics 비활성화 (reset 이후 재적용)
        if self._rs_path:
            stage = omni.usd.get_context().get_stage()
            for p in Usd.PrimRange(stage.GetPrimAtPath(self._rs_path)):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

        self._robot.initialize()

        # fix_base FixedJoint 앵커를 포함해 physics 상태를 원하는 위치로 이동
        self._robot.set_world_pose(
            position=np.array(self.spawn_xyz, dtype=np.float64),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),   # w,x,y,z — 회전 없음
        )

        self._robot.gripper.initialize(
            physics_sim_view=self.world.physics_sim_view,
            articulation_apply_action_func=self._robot.apply_action,
            get_joint_positions_func=self._robot.get_joint_positions,
            set_joint_positions_func=self._robot.set_joint_positions,
            dof_names=self._robot.dof_names,
        )
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions)

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
            marker_length=_ARUCO_LEN, target_id=_ARUCO_ID, K=K)
        self._servo = VisualServoController(
            image_size=_CAM_RES, pixel_to_world_xy=_SERVO_PX2WLD)
        self._viewer = CameraViewer(enabled=True)

        # RMPFlow / PickPlace 컨트롤러
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
            end_effector_initial_height=_EE_INIT_H,
            events_dt=_EVENTS_DT,
            urdf_path=C.M0609_URDF,
            robot_description_path=C.M0609_DESC_YAML,
            rmpflow_config_path=C.M0609_RMPFLOW_CFG,
            end_effector_frame_name=_EE_LINK,
        )

        # home joint 인덱스 캐시
        self._home_idx = self._find_joint_indices(self._robot, _HOME_JOINTS)
        self._home_pos = np.deg2rad(_HOME_DEG)
        self._home_tol = np.deg2rad(_HOME_TOL_DEG)
        self._j5_idx   = self._find_joint_index(self._robot, "joint_5", 4)

        self._state = "MOVE_TO_HOME"
        print(f"[{self.name}] post_reset 완료")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, _dt: float) -> None:
        if not hasattr(self, "_robot") or self._robot is None:
            return
        self._phys_cnt += 1
        # ArUco 마커를 큐브 위로 동기화 (매 step)
        self._sync_marker()
        # 상태머신은 _FSM_EVERY step 마다
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
                det = self._det   # 상태머신에서 저장한 최신 검출
            label = self._display_label()
            self._viewer.update(rgb, det, state_str=label)

    # ── 내부 초기화 ──────────────────────────────────────────────────

    def _init_state(self, spawn):
        self._phys_cnt          = 0
        self._state             = "MOVE_TO_HOME"
        self._spin_start_joints = None
        self._spin_elapsed      = 0.0
        self._spin_last_log     = -1
        self._servo_hold_z      = None
        self._servo_hold_ori    = None
        self._search_start_xy   = None
        self._search_ori        = None
        self._search_z          = None
        self._pick_world_pos    = None
        self._prev_ev           = -1
        self._cur_ev            = -1
        self._det               = None
        # EE workspace — spawn 기준 상대 범위
        ox, oy = spawn[0], spawn[1]
        self._ws_x = (ox + 0.2, ox + 0.6)
        self._ws_y = (oy - 0.5, oy + 0.5)

    def _setup_camera(self, stage, robot_root):
        parent = self._find_prim(robot_root, _GRIP_BASE)
        if parent is None:
            carb.log_warn(f"[{self.name}] camera parent 없음")
            return
        rs_path = attach_realsense_d455(
            parent_prim_path=parent,
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
                Vt.TokenArray(existing + [rop.GetOpName()]))
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

    # ── 유틸리티 ─────────────────────────────────────────────────────

    @staticmethod
    def _import_urdf(urdf_path: str, fix_base: bool):
        _, import_cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        import_cfg.merge_fixed_joints           = False
        import_cfg.convex_decomp                = True
        import_cfg.import_inertia_tensor        = True
        import_cfg.fix_base                     = fix_base
        import_cfg.distance_scale               = 1.0
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

    # ── 동기화 ───────────────────────────────────────────────────────

    def _sync_marker(self):
        if self._cube is None:
            return
        cube_pos, cube_q = self._cube.get_world_pose()
        R_c = _quat_wxyz_to_R(cube_q)
        mpos = cube_pos + R_c @ np.array([0.0, 0.0, _ARUCO_Z_OFF])
        stage = omni.usd.get_context().get_stage()
        _set_marker_pose(stage, self._marker_path, mpos, cube_q)

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
        stage = omni.usd.get_context().get_stage()
        _attach_fixed_joint(stage, self._grip_joint,
                            self._grip_body_path,
                            f"/World/{self.name}_cube")

    def _detach_cube(self):
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(self._grip_joint).IsValid():
            stage.RemovePrim(self._grip_joint)
            print(f"[{self.name}] 조인트 해제")

    def _display_label(self) -> str:
        if self._state == "DONE":
            return "Placing Success!!"
        if self._state == "PICK_AND_PLACE":
            if self._cur_ev <= 3:
                return "Picking..."
            if self._cur_ev <= 6:
                return "Moving..."
            return "Placing Success!!"
        return "Detecting..."

    # ── 상태머신 ─────────────────────────────────────────────────────

    def _run_fsm(self):
        robot  = self._robot
        joints = robot.get_joint_positions()
        ee_pos, ee_ori = robot.end_effector.get_world_pose()
        cur_xy = ee_pos[:2].copy()

        # 카메라 프레임 취득 + ArUco 검출
        rgb = self._wrist_cam.get_rgb() if self._wrist_cam else None
        det = None
        if rgb is not None:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            det = self._tracker.detect(bgr)
        self._det = det

        # ── MOVE_TO_HOME ────────────────────────────────────────────
        if self._state == "MOVE_TO_HOME":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)
            err = np.max(np.abs(joints[self._home_idx] - self._home_pos))
            if err < self._home_tol:
                self._spin_start_joints = None
                self._spin_elapsed      = 0.0
                self._state = "Detecting"
                print(f"[{self.name}] → Detecting")

        # ── Detecting (joint_5 +90° 회전으로 시야 확보) ──────────────
        elif self._state == "Detecting":
            if self._spin_start_joints is None:
                self._spin_start_joints = joints.copy()
            self._spin_elapsed = min(self._spin_elapsed + _CTRL_DT, _SPIN_DUR)
            prog = min(self._spin_elapsed / _SPIN_DUR, 1.0)
            j5t  = np.array([self._spin_start_joints[self._j5_idx]
                              + np.deg2rad(_SPIN_J5_DEG) * prog])
            robot.set_joint_positions(j5t,
                                      joint_indices=np.array([self._j5_idx]))
            if self._spin_elapsed >= _SPIN_DUR:
                self._state = "SEARCH"
                print(f"[{self.name}] → SEARCH")

        # ── SEARCH ──────────────────────────────────────────────────
        elif self._state == "SEARCH":
            if det is not None and det.found:
                self._servo_hold_z   = float(ee_pos[2])
                self._servo_hold_ori = ee_ori.copy()
                self._servo.reset()
                self._state = "SERVO"
                print(f"[{self.name}] → SERVO  hold_z={self._servo_hold_z:.3f}")
            else:
                if self._search_start_xy is None:
                    self._search_start_xy = cur_xy.copy()
                    self._search_ori      = ee_ori.copy()
                    self._search_z        = float(ee_pos[2])
                if self._search_z >= _LIFT_Z_MAX:
                    print(f"[{self.name}] SEARCH 실패 — 종료")
                    self._state = "DONE"
                    return
                self._search_z = min(self._search_z + _LIFT_RATE * _CTRL_DT, _LIFT_Z_MAX)
                lift = np.array([self._search_start_xy[0],
                                 self._search_start_xy[1],
                                 self._search_z])
                self._apply_ee(lift, self._search_ori)

        # ── SERVO ────────────────────────────────────────────────────
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

        # ── PICK_AND_PLACE ───────────────────────────────────────────
        elif self._state == "PICK_AND_PLACE":
            actions = self._pp.forward(
                picking_position=self._pick_world_pos,
                placing_position=self._goal_pos,
                current_joint_positions=joints,
                end_effector_offset=_EE_OFFSET,
            )
            robot.apply_action(actions)
            ev = getattr(self._pp, "_event", -1)
            self._cur_ev = ev

            if ev == 4 and self._prev_ev == 3:   # lift 시작 → 큐브 부착
                self._attach_cube()
            elif ev == 8 and self._prev_ev == 7:  # open 완료 → 큐브 해제
                self._detach_cube()

            self._prev_ev = ev
            if self._pp.is_done():
                print(f"[{self.name}] DONE ✓")
                self._state = "DONE"


# ── 모듈 레벨 헬퍼 ────────────────────────────────────────────────────

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


def _set_marker_pose(stage, path, pos, q_wxyz):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        t = op.GetOpType()
        if t == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        elif t == UsdGeom.XformOp.TypeOrient:
            op.Set(Gf.Quatf(float(q_wxyz[0]), float(q_wxyz[1]),
                            float(q_wxyz[2]), float(q_wxyz[3])))


def _add_aruco_plane(stage, prim_path, texture_path, size, position):
    plane = UsdGeom.Mesh.Define(stage, prim_path)
    plane.CreatePointsAttr([(-0.5,-0.5,0),(0.5,-0.5,0),(0.5,0.5,0),(-0.5,0.5,0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0,1,2,3])
    plane.CreateExtentAttr([(-0.5,-0.5,0),(0.5,0.5,0)])
    plane.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(plane).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying).Set(
        [Gf.Vec2f(0,0),Gf.Vec2f(1,0),Gf.Vec2f(1,1),Gf.Vec2f(0,1)])
    xf = UsdGeom.Xformable(plane)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    xf.AddOrientOp().Set(Gf.Quatf(1,0,0,0))
    xf.AddScaleOp().Set(Gf.Vec3f(size, size, size))

    mp = prim_path + "_mat"
    mat  = UsdShade.Material.Define(stage, mp)
    shdr = UsdShade.Shader.Define(stage, mp+"/Shader")
    shdr.CreateIdAttr("UsdPreviewSurface")
    shdr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    uvr = UsdShade.Shader.Define(stage, mp+"/UVReader")
    uvr.CreateIdAttr("UsdPrimvarReader_float2")
    uvr.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uvr.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, mp+"/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        uvr.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shdr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shdr.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(plane.GetPrim()).Bind(mat)


def _attach_fixed_joint(stage, joint_path, link_path, cube_path):
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    lp = stage.GetPrimAtPath(link_path)
    cp = stage.GetPrimAtPath(cube_path)
    if not lp.IsValid() or not cp.IsValid():
        return
    lxf  = UsdGeom.Xformable(lp).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    cxf  = UsdGeom.Xformable(cp).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rel  = cxf * lxf.GetInverse()
    rp   = rel.ExtractTranslation()
    rq   = rel.ExtractRotationQuat()
    im   = rq.GetImaginary()
    jnt  = UsdPhysics.FixedJoint.Define(stage, joint_path)
    jnt.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    jnt.CreateBody1Rel().SetTargets([Sdf.Path(cube_path)])
    jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(rp))
    jnt.CreateLocalRot0Attr().Set(Gf.Quatf(rq.GetReal(),
                                            float(im[0]),float(im[1]),float(im[2])))
    jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
    jnt.CreateLocalRot1Attr().Set(Gf.Quatf(1,0,0,0))
    print(f"[grip_joint] attached: {cube_path} → {link_path}")
