"""
main_isaac/robots/spot_agent.py
================================
Boston Dynamics Spot + OnRobot RG2 에이전트.
spot_pick.py 로직을 BaseRobotAgent 로 캡슐화.

상태머신:
    WALKING → NAVIGATE_TO_CUBE → LOWER → GRASP
    → RAISE → RETURN_HOME → RELEASE → DONE
"""
import sys
import numpy as np
import carb
import omni.usd
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf, UsdPhysics, Usd

from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path

try:
    from isaacsim.core.utils.types import ArticulationAction
except ImportError:
    from omni.isaac.core.utils.types import ArticulationAction

try:
    from omni.isaac.robot_policy.examples.robots import SpotFlatTerrainPolicy
except ImportError:
    from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

import cv2
import robot_config as C
from ..base_robot import BaseRobotAgent

# 외부 모듈 경로 추가
if C.SPOT_SRC_DIR not in sys.path:
    sys.path.insert(0, C.SPOT_SRC_DIR)
from realsense_mount import attach_realsense_d455
from wrist_camera import WristCamera

# ── 고정 파라미터 ─────────────────────────────────────────────────────
_GRIPPER_ROT_OFFSET  = R.from_euler("xyz", [110.0, 0.0, 90.0], degrees=True)
_GRIPPER_OFF_NORMAL  = np.array([0.30, 0.0, 0.0], dtype=np.float64)
_GRIPPER_OFF_LOW     = np.array([0.55, 0.0, -0.72], dtype=np.float64)

_CAM_OFFSET_T        = (0.0, 0.045, 0.05)
_CAM_OFFSET_RPY      = (0.0, -90.0, -90.0)
_CAM_RES             = (640, 480)
_CAM_EXTRA_RPY       = (0.0, 0.0, 90.0)

_ROTATOR_LINKS = {
    "right_outer_knuckle": +1,
    "right_inner_knuckle": +1,
    "left_outer_knuckle":  -1,
    "left_inner_knuckle":  -1,
}
_FOLLOWER_LINKS = {
    "right_inner_finger": "right_outer_knuckle",
    "left_inner_finger":  "left_outer_knuckle",
}
_OPEN_ANGLE     = 0.7
_ANIM_STEPS     = 250

_CROUCH_DEG = {
    "fl_hx":  23.1,  "fl_hy":  68.3,  "fl_kn": -99.8,
    "fr_hx": -23.1,  "fr_hy":  68.3,  "fr_kn": -99.8,
    "hl_hx":  27.0,  "hl_hy":  63.11, "hl_kn": -86.11,
    "hr_hx": -27.0,  "hr_hy":  63.11, "hr_kn": -86.11,
}

_CUBE_SCALE     = 0.05
_BLUE_LOWER     = np.array([100, 120,  80])
_BLUE_UPPER     = np.array([130, 255, 255])
_MIN_AREA       = 300
_CENTER_TOL     = 0.25
_STOP_DIST      = 0.65
_APPROACH_DIST  = 1.2
_HOME_DIST      = 0.45
_LOWER_STEPS    = 300
_RAISE_STEPS    = 300
_DETECT_EVERY   = 10
_Kp             = 1.6
_LOOK_AHEAD     = 0.55
_SPEED          = 0.55
_WARMUP         = 10
_STABILIZE      = 1000


class SpotAgent(BaseRobotAgent):
    """Spot 로봇 에이전트 (순찰 + 큐브 픽업)."""

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        assets_root  = get_assets_root_path()
        spot_usd     = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
        gripper_path = f"/World/{self.name}_Gripper"   # Spot 자식이 아닌 World 직속
        cube_path    = f"/World/{self.name}_Cube"
        spawn        = self.spawn_xyz
        cube_xyz     = tuple(self.cfg.get("cube_xyz", (spawn[0]+3, spawn[1], _CUBE_SCALE/2)))

        # Spot
        self._spot = SpotFlatTerrainPolicy(
            prim_path=f"/World/{self.name}",
            name=self.name,
            usd_path=spot_usd,
            position=np.array(spawn, dtype=np.float64),
        )

        # 그리퍼 (USD 파일 직접 참조)
        self._gripper_path    = gripper_path
        self._gripper_ab_path = f"{gripper_path}/angle_bracket"
        gxf = define_prim(gripper_path, "Xform")
        gxf.GetReferences().AddReference(C.SPOT_GRIPPER_USD)
        self._remove_gripper_physics()

        # 블루 큐브
        self._cube_path = cube_path
        cube_prim = define_prim(cube_path, "Cube")
        self._cube_xf = UsdGeom.Xformable(cube_prim)
        self._cube_xf.ClearXformOpOrder()
        self._cube_t_op = self._cube_xf.AddTranslateOp()
        self._cube_t_op.Set(Gf.Vec3d(*cube_xyz))
        self._cube_xf.AddScaleOp().Set(Gf.Vec3f(_CUBE_SCALE, _CUBE_SCALE, _CUBE_SCALE))
        cube_prim.CreateAttribute("primvars:displayColor",
                                  Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(0, 0.2, 1)])
        UsdPhysics.CollisionAPI.Apply(cube_prim)
        self._cube_rb = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        self._cube_rb.GetRigidBodyEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(cube_prim).GetMassAttr().Set(0.01)

        # 카메라 (setup 단계에서 prim 생성; 초기화는 post_reset)
        self._rs_path    = None
        self._wrist_cam  = None
        self._setup_realsense()

        # 상태 초기화
        self._init_internal_state(spawn)
        print(f"[{self.name}] setup 완료  spawn={spawn}  cube={cube_xyz}")

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        self._disable_rs_physics()

        stage = self.world.stage
        root  = stage.GetPrimAtPath(self._gripper_path)
        if root.IsValid():
            xf = UsdGeom.Xformable(root)
            xf.ClearXformOpOrder()
            self._gripper_t_op = xf.AddTranslateOp()
            self._gripper_o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

        self._init_finger_links()

        if self._wrist_cam is not None:
            self._wrist_cam.initialize()

        print(f"[{self.name}] post_reset 완료")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, dt: float) -> None:
        # 워밍업
        if not self._initialized:
            self._warmup_cnt += 1
            if self._warmup_cnt < _WARMUP:
                return
            self._spot.initialize()
            self._init_crouch_joints()
            self._initialized = True
            return

        # 안정화
        if not self._stable:
            self._stab_cnt += 1
            self._spot.forward(dt, np.zeros(3))
            self._sync_gripper()
            if self._stab_cnt >= _STABILIZE:
                self._stable = True
                print(f"[{self.name}] 안정화 완료 → 주행 시작")
            return

        # 실행
        cmd = self._run_fsm()
        self._spot.forward(dt, cmd)
        self._apply_crouch_blend()
        self._sync_gripper()
        self._update_gripper_anim()
        if self._gripped:
            self._sync_cube_to_gripper()

    # ── 내부 초기화 ──────────────────────────────────────────────────

    def _init_internal_state(self, spawn):
        self._gripper_t_op  = None
        self._gripper_o_op  = None
        self._cur_g_off     = _GRIPPER_OFF_NORMAL.copy()
        self._g_world_pos   = np.zeros(3)
        self._g_world_rot   = R.identity()
        self._finger_data   = {}
        self._ganim_state   = "idle"
        self._ganim_step    = 0
        self._crouch_idx    = None
        self._crouch_tgt    = None
        self._lower_start   = None
        self._state         = "WALKING"
        self._state_step    = 0
        self._cube_nav      = None
        self._gripped       = False
        self._det_cnt       = 0
        self._warmup_cnt    = 0
        self._stab_cnt      = 0
        self._initialized   = False
        self._stable        = False
        ox, oy = spawn[0], spawn[1]
        self._waypoints = [
            np.array([ox + 3.0, oy + 0.0]),
            np.array([ox + 3.0, oy - 1.5]),
            np.array([ox + 0.0, oy - 1.5]),
            np.array([ox + 0.0, oy + 0.0]),
        ]
        self._wp_idx  = 0
        self._home_xy = np.array([ox, oy])

    def _remove_gripper_physics(self):
        for prim in self.world.stage.Traverse():
            if not str(prim.GetPath()).startswith(self._gripper_path):
                continue
            for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI,
                        UsdPhysics.CollisionAPI, UsdPhysics.MassAPI):
                if prim.HasAPI(api):
                    prim.RemoveAPI(api)
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)

    def _setup_realsense(self):
        try:
            ab = self.world.stage.GetPrimAtPath(self._gripper_ab_path)
            if not ab.IsValid():
                carb.log_warn(f"[{self.name}] angle_bracket 없음")
                return
            rs_path = attach_realsense_d455(
                parent_prim_path=self._gripper_ab_path,
                child_name="realsense_d455",
                translation=_CAM_OFFSET_T,
                rpy_deg=_CAM_OFFSET_RPY,
            )
            self._rs_path = rs_path

            stage   = omni.usd.get_context().get_stage()
            ov_path = None
            rs_prim = stage.GetPrimAtPath(rs_path)
            if rs_prim.IsValid():
                for p in Usd.PrimRange(rs_prim):
                    if p.GetName() == "Camera_OmniVision_OV9782_Color":
                        ov_path = str(p.GetPath())
                        break

            if ov_path:
                from pxr import Vt
                cam_prim = stage.GetPrimAtPath(ov_path)
                cam_xf   = UsdGeom.Xformable(cam_prim)
                existing = [op.GetOpName() for op in cam_xf.GetOrderedXformOps()]
                rot_op   = cam_xf.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat,
                                                opSuffix="extra")
                rot_op.Set(float(_CAM_EXTRA_RPY[2]))
                cam_prim.GetAttribute("xformOpOrder").Set(
                    Vt.TokenArray(existing + [rot_op.GetOpName()]))
                self._wrist_cam = WristCamera.from_existing_prim(
                    prim_path=ov_path, resolution=_CAM_RES)
            else:
                self._wrist_cam = WristCamera(
                    parent_prim_path=rs_path,
                    name=f"{self.name}_wrist_rgb",
                    resolution=_CAM_RES,
                    rpy_deg=_CAM_EXTRA_RPY,
                )
            print(f"[{self.name}] 카메라: {self._wrist_cam._prim_path}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] RealSense 설정 실패: {e}")

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

    def _init_crouch_joints(self):
        try:
            dof = list(self._spot.robot.dof_names)
            idx, tgt = [], []
            for name, deg in _CROUCH_DEG.items():
                i = next(
                    (i for i, n in enumerate(dof)
                     if n == name or n.endswith(f"/{name}") or n.endswith(f"_{name}")),
                    -1)
                if i >= 0:
                    idx.append(i)
                    tgt.append(np.deg2rad(deg))
            self._crouch_idx = np.array(idx, dtype=int)
            self._crouch_tgt = np.array(tgt, dtype=np.float64)
            print(f"[{self.name}] crouch 관절 {len(idx)}개 매핑 완료")
        except Exception as e:
            carb.log_warn(f"[{self.name}] crouch 초기화 실패: {e}")

    def _init_finger_links(self):
        stage = self.world.stage

        def _load(link_name):
            p = stage.GetPrimAtPath(f"{self._gripper_path}/{link_name}")
            if not p.IsValid():
                return None
            xf  = UsdGeom.Xformable(p)
            mat = xf.GetLocalTransformation(Usd.TimeCode.Default())
            tr  = np.array(mat.ExtractTranslation())
            rq  = mat.ExtractRotationQuat()
            im  = rq.GetImaginary()
            xf.ClearXformOpOrder()
            t_op = xf.AddTranslateOp()
            t_op.Set(Gf.Vec3d(*map(float, tr)))
            o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            o_op.Set(rq)
            return {"t_op": t_op, "o_op": o_op,
                    "base_t": tr,
                    "base_q": np.array([im[0], im[1], im[2], rq.GetReal()])}

        for ln, sign in _ROTATOR_LINKS.items():
            d = _load(ln)
            if d:
                d["sign"] = sign
                self._finger_data[ln] = d

        for ln, parent in _FOLLOWER_LINKS.items():
            d = _load(ln)
            if d is None or parent not in self._finger_data:
                continue
            pd  = self._finger_data[parent]
            pr0 = R.from_quat(pd["base_q"])
            d["parent"]   = parent
            d["rel_in_p"] = pr0.inv().apply(d["base_t"] - pd["base_t"])
            self._finger_data[ln] = d

    # ── 그리퍼 제어 ──────────────────────────────────────────────────

    def _set_finger_angle(self, angle: float):
        for ln, sign in _ROTATOR_LINKS.items():
            if ln not in self._finger_data:
                continue
            d   = self._finger_data[ln]
            rot = R.from_quat(d["base_q"]) * R.from_euler("y", sign * angle)
            fq  = rot.as_quat()
            d["o_op"].Set(Gf.Quatd(float(fq[3]),
                                    float(fq[0]), float(fq[1]), float(fq[2])))
            d["_cur"] = rot

        for ln in _FOLLOWER_LINKS:
            if ln not in self._finger_data:
                continue
            d  = self._finger_data[ln]
            pd = self._finger_data.get(d["parent"])
            if pd is None:
                continue
            pr = pd.get("_cur", R.from_quat(pd["base_q"]))
            d["t_op"].Set(Gf.Vec3d(*map(float,
                pd["base_t"] + pr.apply(d["rel_in_p"]))))

    def _trigger_close(self):
        if self._ganim_state == "idle" and self._finger_data:
            self._ganim_state = "closing"
            self._ganim_step  = 0

    def _trigger_open(self):
        if self._ganim_state == "idle" and self._finger_data:
            self._ganim_state = "opening"
            self._ganim_step  = 0

    def _update_gripper_anim(self):
        if self._ganim_state == "idle":
            return
        self._ganim_step += 1
        if self._ganim_state == "opening":
            t = min(self._ganim_step / _ANIM_STEPS, 1.0)
            self._set_finger_angle(t * _OPEN_ANGLE)
            if self._ganim_step >= _ANIM_STEPS:
                self._ganim_state = "idle"
                self._ganim_step  = 0
        elif self._ganim_state == "closing":
            t = 1.0 - min(self._ganim_step / _ANIM_STEPS, 1.0)
            self._set_finger_angle(t * _OPEN_ANGLE)
            if self._ganim_step >= _ANIM_STEPS:
                self._set_finger_angle(0.0)
                self._ganim_state = "idle"
                self._ganim_step  = 0

    # ── 자세 동기화 ──────────────────────────────────────────────────

    def _apply_crouch_blend(self):
        if self._crouch_idx is None or len(self._crouch_idx) == 0:
            return
        if self._state == "LOWER":
            t = min(self._state_step / _LOWER_STEPS, 1.0)
        elif self._state == "GRASP":
            t = 1.0
        elif self._state == "RAISE":
            t = 1.0 - min(self._state_step / _RAISE_STEPS, 1.0)
        else:
            return
        try:
            if self._lower_start is None:
                all_pos = self._spot.robot.get_joint_positions()
                self._lower_start = all_pos[self._crouch_idx].copy()
            tgt = self._lower_start * (1.0 - t) + self._crouch_tgt * t
            self._spot.robot.apply_action(ArticulationAction(
                joint_positions=tgt,
                joint_indices=self._crouch_idx,
            ))
        except Exception as e:
            carb.log_warn(f"[{self.name}] crouch blend 실패: {e}")

    def _sync_gripper(self):
        if self._gripper_t_op is None:
            return
        try:
            pos, quat = self._spot.robot.get_world_pose()
            rot  = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            gpos = pos + rot.apply(self._cur_g_off)
            grot = rot * _GRIPPER_ROT_OFFSET
            self._g_world_pos = gpos
            self._g_world_rot = grot
            self._gripper_t_op.Set(Gf.Vec3d(*map(float, gpos)))
            q = grot.as_quat()
            self._gripper_o_op.Set(Gf.Quatd(float(q[3]),
                                              float(q[0]), float(q[1]), float(q[2])))
        except Exception as e:
            carb.log_warn(f"[{self.name}] gripper sync 실패: {e}")

    @property
    def _cur_g_off(self):
        return self.__cur_g_off

    @_cur_g_off.setter
    def _cur_g_off(self, v):
        self.__cur_g_off = v

    # ── 큐브 제어 ────────────────────────────────────────────────────

    def _set_cube_kinematic(self, enabled: bool):
        self._cube_rb.GetRigidBodyEnabledAttr().Set(not enabled)

    def _set_cube_pos(self, pos: np.ndarray):
        self._cube_t_op.Set(Gf.Vec3d(*map(float, pos)))

    def _get_cube_world(self) -> np.ndarray:
        mat = self._cube_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(mat.ExtractTranslation())

    def _get_grip_center(self) -> np.ndarray:
        stage = self.world.stage
        pts = []
        for ln in ("right_inner_finger", "left_inner_finger"):
            p = stage.GetPrimAtPath(f"{self._gripper_path}/{ln}")
            if p.IsValid():
                mat = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                pts.append(np.array(mat.ExtractTranslation()))
        return np.mean(pts, axis=0) if pts else self._g_world_pos.copy()

    def _sync_cube_to_gripper(self):
        if not self._gripped:
            return
        pos    = self._get_grip_center()
        pos[2] = max(float(pos[2]), _CUBE_SCALE / 2)
        self._set_cube_pos(pos)

    # ── 탐지 ─────────────────────────────────────────────────────────

    def _detect_blue(self, frame: np.ndarray):
        if frame is None or frame.size == 0:
            return False, 0.0, 0
        bgr  = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, _BLUE_LOWER, _BLUE_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return False, 0.0, 0
        lg   = max(cs, key=cv2.contourArea)
        area = int(cv2.contourArea(lg))
        if area < _MIN_AREA:
            return False, 0.0, area
        M  = cv2.moments(lg)
        cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else frame.shape[1] // 2
        return True, (cx / frame.shape[1]) - 0.5, area

    def _try_detect(self):
        if self._wrist_cam is None:
            return
        try:
            rgb = self._wrist_cam.get_rgb()
            if rgb is None or rgb.size == 0:
                return
            found, cx_r, area = self._detect_blue(rgb)
            if found and abs(cx_r) < _CENTER_TOL:
                cw = self._get_cube_world()
                self._cube_nav  = cw[:2].copy()
                self._state     = "NAVIGATE_TO_CUBE"
                self._state_step = 0
                print(f"[{self.name}] 큐브 발견 {cw.round(3)}  area={area}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] detect 오류: {e}")

    # ── 내비게이션 ───────────────────────────────────────────────────

    def _get_pos_yaw(self):
        pos, quat = self._spot.robot.get_world_pose()
        yaw = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz")[2]
        return pos, yaw

    def _nav_toward(self, tgt_xy: np.ndarray, speed: float = None) -> np.ndarray:
        speed = speed or _SPEED
        try:
            pos, yaw = self._get_pos_yaw()
            dist     = np.linalg.norm(pos[:2] - tgt_xy)
            vx       = speed * min(1.0, dist / _APPROACH_DIST)
            ey       = np.arctan2(tgt_xy[1]-pos[1], tgt_xy[0]-pos[0]) - yaw
            ey       = (ey + np.pi) % (2*np.pi) - np.pi
            return np.array([vx, 0.0, float(np.clip(_Kp * ey, -0.7, 0.7))])
        except Exception:
            return np.zeros(3)

    def _waypoint_cmd(self) -> np.ndarray:
        try:
            pos, yaw = self._get_pos_yaw()
            tgt = self._waypoints[self._wp_idx]
            if np.linalg.norm(pos[:2] - tgt) < _LOOK_AHEAD:
                self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)
                tgt = self._waypoints[self._wp_idx]
                print(f"[{self.name}] 웨이포인트 → {self._wp_idx}  {tgt}")
            ey = np.arctan2(tgt[1]-pos[1], tgt[0]-pos[0]) - yaw
            ey = (ey + np.pi) % (2*np.pi) - np.pi
            return np.array([_SPEED, 0.0, float(np.clip(_Kp * ey, -0.7, 0.7))])
        except Exception:
            return np.zeros(3)

    # ── 상태머신 ─────────────────────────────────────────────────────

    def _run_fsm(self) -> np.ndarray:
        self._state_step += 1
        cmd = np.zeros(3)

        if self._state == "WALKING":
            cmd = self._waypoint_cmd()
            self._det_cnt += 1
            if self._det_cnt >= _DETECT_EVERY:
                self._det_cnt = 0
                self._try_detect()

        elif self._state == "NAVIGATE_TO_CUBE":
            if self._cube_nav is None:
                self._state = "WALKING"
                return cmd
            cmd = self._nav_toward(self._cube_nav)
            try:
                pos, _ = self._get_pos_yaw()
                if np.linalg.norm(pos[:2] - self._cube_nav) < _STOP_DIST:
                    self._lower_start = None
                    self._state      = "LOWER"
                    self._state_step = 0
                    print(f"[{self.name}] → LOWER")
            except Exception:
                pass

        elif self._state == "LOWER":
            if self._state_step >= _LOWER_STEPS:
                self._state      = "GRASP"
                self._state_step = 0
                self._trigger_close()
                print(f"[{self.name}] → GRASP")

        elif self._state == "GRASP":
            if self._ganim_state == "idle":
                self._gripped = True
                self._set_cube_kinematic(True)
                self._sync_cube_to_gripper()
                self._state      = "RAISE"
                self._state_step = 0
                print(f"[{self.name}] → RAISE")

        elif self._state == "RAISE":
            self._sync_cube_to_gripper()
            if self._state_step >= _RAISE_STEPS:
                self._lower_start = None
                self._state      = "RETURN_HOME"
                self._state_step = 0
                print(f"[{self.name}] → RETURN_HOME")

        elif self._state == "RETURN_HOME":
            cmd = self._nav_toward(self._home_xy)
            self._sync_cube_to_gripper()
            try:
                pos, _ = self._get_pos_yaw()
                if np.linalg.norm(pos[:2] - self._home_xy) < _HOME_DIST:
                    self._state      = "RELEASE"
                    self._state_step = 0
                    self._trigger_open()
                    print(f"[{self.name}] → RELEASE")
            except Exception:
                pass

        elif self._state == "RELEASE":
            self._sync_cube_to_gripper()
            if self._ganim_state == "idle":
                self._gripped = False
                self._set_cube_kinematic(False)
                self._state = "DONE"
                print(f"[{self.name}] DONE ✓")

        return cmd
