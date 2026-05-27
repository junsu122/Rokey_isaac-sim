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
import math
import numpy as np
import carb
import omni.usd
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, UsdPhysics, Usd

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
if C.USE_REALSENSE:
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

# 가끔 못 일어남
# _CROUCH_DEG = {
#     "fl_hx":  23.1,  "fl_hy":  68.3,  "fl_kn": -99.8,
#     "fr_hx": -23.1,  "fr_hy":  68.3,  "fr_kn": -99.8,
#     "hl_hx":  27.0,  "hl_hy":  63.11, "hl_kn": -86.11,
#     "hr_hx": -27.0,  "hr_hy":  63.11, "hr_kn": -86.11,
# }

# 앞으로 고꾸라짐
# _CROUCH_DEG = {
#     "fl_hx":  22.2,  "fl_hy":  51.57,  "fl_kn": -86.11,
#     "fr_hx": -22.2,  "fr_hy":  51.57,  "fr_kn": -86.11,
#     "hl_hx":  16.5,  "hl_hy":  16.2, "hl_kn": -50.4,
#     "hr_hx": -16.5,  "hr_hy":  16.2, "hr_kn": -50.4,
# }

# 한쪽으로 기운다
# _CROUCH_DEG = {
#     "fl_hx":  10.0,  "fl_hy":  51.57,  "fl_kn": -112.4,
#     "fr_hx": -10.0,  "fr_hy":  51.57,  "fr_kn": -112.4,
#     "hl_hx":  1.3,  "hl_hy":  63.11, "hl_kn": -86.11,
#     "hr_hx": -1.3,  "hr_hy":  63.11, "hr_kn": -86.11,
# }

# 안정적인듯?
_CROUCH_DEG = {
    "fl_hx":  5.73,  "fl_hy":  70.1,  "fl_kn": -120.9,
    "fr_hx": -5.73,  "fr_hy":  70.1,  "fr_kn": -120.9,
    "hl_hx":  20.0,  "hl_hy":  63.11, "hl_kn": -86.11,
    "hr_hx": -20.0,  "hr_hy":  63.11, "hr_kn": -86.11,
}

_MIN_AREA           = 20     # small/far click-spawn boxes still need to be detected
_GOAL_ZONE_HALF     = 1.5    # 목표 영역 반변 길이 (m) — 3×3 m 초록 사각형 기준
_STOP_DIST      = 0.65
_APPROACH_DIST  = 1.2
_HOME_DIST      = 0.75
_LOWER_STEPS    = 300
_RAISE_STEPS    = 300
_DETECT_EVERY   = 10
_Kp             = 1.6
_LOOK_AHEAD     = 0.55
_SPEED          = 0.55
_WARMUP         = 10
_STABILIZE      = 1000
_AVOID_DIST      = 2.0   # 다른 Spot 와 이 거리(m) 이내이면 하위 우선순위 Spot 이 정지
_DETOUR_SIDE     = 2.5   # 우회 시 측방 오프셋 [m]
_DETOUR_FWD      = 2.0   # 우회 시 진행 방향 전방 오프셋 [m] — 정지 로봇을 지나쳐 가는 거리
_DETOUR_COOLDOWN = 300   # 우회 완료 후 재우회 검사 전 대기 FSM 틱 수 (≈ 6 s)

# ── pod 섹션 금지 구역 (Spot 진입 불가) ──────────────────────────────────
# 각 섹션 pod 좌표 범위 + 넓은 마진 (로봇 반폭 + 회전/흔들림 여유)
# Section A: cy=10.0, Section B: cy=0.0, Section C: cy=-10.0
# pod x: ±2.8 → 마진 포함 ±5.2
# pod y: ±2.0 from cy → 마진 포함 cy±3.8, leaving marker lanes reachable
_POD_ZONE_HALF_X = 5.2
_POD_ZONE_CENTERS_Y = (10.0, 0.0, -10.0)
_POD_ZONE_HALF_Y = 3.8
_WALL_SAFE_Y = 14.6
_CONVEYOR_ZONES = [
    (12.2, 20.8, -10.95, -10.05),
    (12.2, 20.8,  -0.45,   0.45),
    (12.2, 20.8,  10.05,  10.95),
    (-20.0, -11.8, -0.45, 0.45),
    (-13.8, -11.8, -3.2, -1.5),
    (-13.8, -11.8,  1.5,  3.2),
    (-12.2, -10.4, -8.7, -6.9),
    (-10.4,  -8.6, -0.9,  0.9),
]


def _clamp_clear_of_pods(xy: np.ndarray) -> np.ndarray:
    """목표점이 pod/컨베이어 금지 구역 안에 있으면 가장 가까운 외곽으로 밀어낸다."""
    x, y = float(xy[0]), max(-_WALL_SAFE_Y, min(_WALL_SAFE_Y, float(xy[1])))
    for cy in _POD_ZONE_CENTERS_Y:
        if abs(x) < _POD_ZONE_HALF_X and abs(y - cy) < _POD_ZONE_HALF_Y:
            # x 방향으로 가장 짧은 이탈 경로 사용
            safe_x = math.copysign(_POD_ZONE_HALF_X + 0.2, x if x != 0 else 1.0)
            return np.array([safe_x, y], dtype=np.float64)
    for x0, x1, y0, y1 in _CONVEYOR_ZONES:
        if x0 < x < x1 and y0 < y < y1:
            candidates = [
                np.array([x0 - 0.35, y], dtype=np.float64),
                np.array([x1 + 0.35, y], dtype=np.float64),
                np.array([x, y0 - 0.35], dtype=np.float64),
                np.array([x, y1 + 0.35], dtype=np.float64),
            ]
            return min(candidates, key=lambda p: float(np.linalg.norm(p - xy[:2])))
    return xy


def _segment_hits_pod_zone(a: np.ndarray, b: np.ndarray) -> bool:
    """True if a straight Spot route would cross any pod/conveyor no-go box."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    zones = [
        (-_POD_ZONE_HALF_X, _POD_ZONE_HALF_X,
         cy - _POD_ZONE_HALF_Y, cy + _POD_ZONE_HALF_Y)
        for cy in _POD_ZONE_CENTERS_Y
    ] + _CONVEYOR_ZONES
    for min_x, max_x, min_y, max_y in zones:
        if max(ax, bx) < min_x or min(ax, bx) > max_x:
            continue
        if max(ay, by) < min_y or min(ay, by) > max_y:
            continue
        if min_x < ax < max_x and min_y < ay < max_y:
            return True
        if min_x < bx < max_x and min_y < by < max_y:
            return True
        steps = max(8, int(np.linalg.norm(b[:2] - a[:2]) / 0.25))
        for t in np.linspace(0.0, 1.0, steps):
            x = ax + (bx - ax) * float(t)
            y = ay + (by - ay) * float(t)
            if min_x < x < max_x and min_y < y < max_y:
                return True
    return False


def _pod_safe_detour(pos_xy: np.ndarray, target_xy: np.ndarray) -> np.ndarray:
    """Return an outside-edge detour when direct travel would cross pods."""
    target_xy = _clamp_clear_of_pods(np.array(target_xy, dtype=np.float64))
    if not _segment_hits_pod_zone(pos_xy, target_xy):
        return target_xy
    side_seed = pos_xy[0] if abs(pos_xy[0]) > 0.2 else target_xy[0]
    safe_x = math.copysign(_POD_ZONE_HALF_X + 0.6, side_seed if side_seed != 0 else 1.0)
    if abs(pos_xy[0]) < _POD_ZONE_HALF_X + 0.3:
        return np.array([safe_x, float(pos_xy[1])], dtype=np.float64)
    return np.array([safe_x, target_xy[1]], dtype=np.float64)


class SpotAgent(BaseRobotAgent):
    """Spot 로봇 에이전트 (순찰 + 큐브 픽업)."""

    # 모든 SpotAgent 인스턴스의 클래스 레벨 레지스트리 (상호 충돌 회피용)
    _registry: list = []

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        assets_root  = get_assets_root_path()
        spot_usd     = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
        gripper_path = f"/World/{self.name}_Gripper"
        spawn        = self.spawn_xyz

        # ArUco ID → 목표 XY 매핑 (robot_config 에서 로드)
        # 키를 int로 변환 (JSON 은 str 키로 올 수 있음)
        raw_goals = self.cfg.get("aruco_goals", {})
        self._aruco_goals = {int(k): np.array(v, dtype=np.float64)
                             for k, v in raw_goals.items()}

        # ArUco 검출기 (ID 검출 전용, pose 추정 불필요)
        _dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._aruco_detector = cv2.aruco.ArucoDetector(
            _dict, cv2.aruco.DetectorParameters())

        # Spot
        self._spot = SpotFlatTerrainPolicy(
            prim_path=f"/World/{self.name}",
            name=self.name,
            usd_path=spot_usd,
            position=np.array(spawn, dtype=np.float64),
        )

        # 그리퍼
        self._gripper_path    = gripper_path
        self._gripper_ab_path = f"{gripper_path}/angle_bracket"
        gxf = define_prim(gripper_path, "Xform")
        gxf.GetReferences().AddReference(C.SPOT_GRIPPER_USD)
        self._remove_gripper_physics()

        # 카메라
        self._rs_path    = None
        self._wrist_cam  = None
        if C.USE_REALSENSE:
            self._setup_realsense()

        # 상태 초기화
        self._init_internal_state(spawn)

        # 클래스 레지스트리에 등록 (충돌 회피 로직에서 다른 Spot 위치 참조)
        if self not in SpotAgent._registry:
            SpotAgent._registry.append(self)

        print(f"[{self.name}] setup 완료  spawn={spawn}  "
              f"aruco_goals={list(self._aruco_goals.keys())}")

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

        self._compute_astar_patrol()
        print(f"[{self.name}] post_reset 완료")

    def _compute_astar_patrol(self) -> None:
        """순찰 웨이포인트를 _waypoints 에 저장.

        A* 스무딩을 사용하지 않음: 스무더가 pod 장애물을 인식하지 못해
        직선 단축 경로를 만들면서 pod 구역을 통과하기 때문.
        robot_config.py 의 웨이포인트는 pod 구역 외곽(x=±4.5)을 따르도록
        수동 설계되어 있으므로 그대로 사용.
        """
        config_wps = self.cfg.get("waypoints", [])
        if not config_wps:
            return
        self._waypoints = [
            np.array([float(wp[0]), float(wp[1])], dtype=np.float64)
            for wp in config_wps
        ]
        print(f"[{self.name}] 순찰 경로 설정 완료 — {len(self._waypoints)} 웨이포인트 "
              f"(A* 스킵, pod 외곽 경로 직접 적용)")

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
            self._sync_autobox_to_gripper()

    # ── 내부 초기화 ──────────────────────────────────────────────────

    def _init_internal_state(self, spawn):
        self._gripper_t_op      = None
        self._gripper_o_op      = None
        self._cur_g_off         = _GRIPPER_OFF_NORMAL.copy()
        self._g_world_pos       = np.zeros(3)
        self._g_world_rot       = R.identity()
        self._finger_data       = {}
        self._ganim_state       = "idle"
        self._ganim_step        = 0
        self._crouch_idx        = None
        self._crouch_tgt        = None
        self._lower_start       = None
        self._state             = "WALKING"
        self._state_step        = 0
        self._cube_nav          = None   # 이동할 박스 XY
        self._gripped           = False
        self._detected_aruco_id = None   # 감지된 ArUco ID
        self._goal_xy           = None   # ID에 대응하는 목표 XY
        self._grip_box_path     = None   # 잡고 있는 AutoBox prim 경로
        self._delivered_box_paths = set()
        self._pickup_cooldown   = 0
        self._det_cnt           = 0
        self._warmup_cnt        = 0
        self._stab_cnt          = 0
        self._initialized       = False
        self._stable            = False
        self._active_detour     = None   # 현재 추적 중인 우회 목표점 (np.ndarray or None)
        self._detour_cooldown   = 0      # 우회 완료 후 재검사 전 대기 카운터
        ox, oy = spawn[0], spawn[1]
        # robot_config.py 의 "waypoints" 키로 경로 지정 가능.
        # 미지정 시 스폰 주변 기본 사각형 경로 사용.
        if "waypoints" in self.cfg and self.cfg["waypoints"]:
            self._waypoints = [np.array(wp, dtype=np.float64)
                               for wp in self.cfg["waypoints"]]
        else:
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

    # ── AutoBox 제어 ─────────────────────────────────────────────────

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

    def _find_nearest_autobox(self, spot_xy: np.ndarray,
                               max_dist: float = 6.0,
                               skip_goal_zones: bool = False):
        """스팟 XY 기준 가장 가까운 미픽업 박스 prim 경로와 XY를 반환."""
        stage = omni.usd.get_context().get_stage()
        best_path, best_xy, best_dist = None, None, max_dist
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not (
                path.startswith("/World/AutoBox_") or
                path.startswith("/World/DynamicBoxes/") or
                path.startswith("/World/MinimapClickBoxes/")
            ):
                continue
            if prim.GetCustomDataByKey("spot_delivered"):
                continue
            if path in self._delivered_box_paths:
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            # 이미 kinematic(다른 로봇이 잡은 것) 이면 스킵
            if UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():
                continue
            mat = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            T   = np.array(mat, dtype=np.float64).T
            xy  = T[:2, 3]
            if skip_goal_zones and self._is_in_goal_zone(xy):
                continue
            d   = float(np.linalg.norm(xy - spot_xy))
            if d < best_dist:
                best_dist, best_path, best_xy = d, path, xy.copy()
        return best_path, best_xy

    @staticmethod
    def _set_box_collision(prim, enabled: bool) -> None:
        """AutoBox 하위 전체 collision 활성/비활성."""
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                child.GetAttribute("physics:collisionEnabled").Set(enabled)

    def _attach_nearest_autobox(self):
        """
        스팟 현재 위치 기준 가장 가까운 AutoBox를 kinematic으로 전환.
        collision 도 비활성화 → kinematic 박스가 Spot 충돌 메시를 관통할 때
        물리 엔진이 가하는 충격력(Spot 날아가는 현상)을 방지.
        """
        try:
            pos, _ = self._get_pos_yaw()
            path, _ = self._find_nearest_autobox(
                pos[:2], max_dist=2.0, skip_goal_zones=True)
            if path is None:
                print(f"[{self.name}] 근처 AutoBox 없음 (2m 내)")
                return
            stage = omni.usd.get_context().get_stage()
            prim  = stage.GetPrimAtPath(path)
            # kinematic 전환 + collision 비활성화
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
            self._set_box_collision(prim, False)
            self._grip_box_path = path
            print(f"[{self.name}] AutoBox 흡착 (collision OFF): {path}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] attach 오류: {e}")

    def _sync_autobox_to_gripper(self):
        """잡은 AutoBox를 그리퍼 중심으로 이동."""
        if not self._gripped or self._grip_box_path is None:
            return
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(self._grip_box_path)
        if not prim.IsValid():
            self._grip_box_path = None
            return
        pos    = self._get_grip_center()
        pos[2] = max(float(pos[2]), 0.05)
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
                break

    def _detach_autobox(self):
        """AutoBox physics 복원 + collision 재활성화 → 낙하."""
        if self._grip_box_path is None:
            return
        released_path = self._grip_box_path
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(released_path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            self._set_box_collision(prim, True)          # collision 복원
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)
            prim.SetCustomDataByKey("spot_delivered", True)
        self._delivered_box_paths.add(released_path)
        print(f"[{self.name}] AutoBox 해제 (collision ON → 낙하): {released_path}")
        self._grip_box_path = None

    # ── ArUco 탐지 ────────────────────────────────────────────────────

    def _is_in_goal_zone(self, xy: np.ndarray) -> bool:
        """XY 위치가 aruco_goals 중 어느 목표 영역 안에 있는지 확인."""
        for goal_xy in self._aruco_goals.values():
            if (abs(xy[0] - goal_xy[0]) < _GOAL_ZONE_HALF and
                    abs(xy[1] - goal_xy[1]) < _GOAL_ZONE_HALF):
                return True
        return False

    def _is_goal_occupied(self, goal_xy: np.ndarray,
                          ignore_path: str = None) -> bool:
        """True if a box is already sitting in this marker zone."""
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if ignore_path is not None and path == ignore_path:
                continue
            if not (
                path.startswith("/World/AutoBox_") or
                path.startswith("/World/DynamicBoxes/") or
                path.startswith("/World/MinimapClickBoxes/")
            ):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            mat = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            T = np.array(mat, dtype=np.float64).T
            xy = T[:2, 3]
            if (abs(xy[0] - goal_xy[0]) < _GOAL_ZONE_HALF and
                    abs(xy[1] - goal_xy[1]) < _GOAL_ZONE_HALF):
                return True
        return False

    def _select_goal_for_box(self, aruco_id: int, box_path: str = None) -> np.ndarray:
        """Use the requested ArUco marker if empty, otherwise choose an empty marker."""
        preferred = self._aruco_goals.get(aruco_id)
        if preferred is not None and not self._is_goal_occupied(preferred, box_path):
            return preferred
        for _, goal_xy in sorted(self._aruco_goals.items()):
            if not self._is_goal_occupied(goal_xy, box_path):
                return goal_xy
        return preferred if preferred is not None else self._home_xy

    def _detect_aruco(self, frame: np.ndarray):
        """카메라 프레임에서 가장 큰 ArUco 마커 ID를 반환. 없으면 None."""
        if frame is None or frame.size == 0:
            return None
        bgr  = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return None
        # 면적이 가장 큰 마커 선택
        areas = [
            float((corners[i][0].max(axis=0) - corners[i][0].min(axis=0)).prod())
            for i in range(len(ids))
        ]
        best = int(np.argmax(areas))
        aruco_id = int(ids[best][0])
        if aruco_id in self._aruco_goals and areas[best] > _MIN_AREA:
            return aruco_id
        return None

    def _try_detect_aruco(self):
        """WALKING 중 ArUco 탐지 → 박스 위치 확인 후 NAVIGATE_TO_CUBE 전환."""
        if self._wrist_cam is None:
            return
        try:
            rgb = self._wrist_cam.get_rgb()
            aruco_id = self._detect_aruco(rgb)
            if aruco_id is None:
                return
            # 가장 가까운 AutoBox 위치 확인
            pos, _ = self._get_pos_yaw()
            box_path, box_xy = self._find_nearest_autobox(pos[:2], skip_goal_zones=True)
            if box_xy is None:
                return
            self._detected_aruco_id = aruco_id
            self._goal_xy           = self._select_goal_for_box(aruco_id, box_path)
            self._cube_nav          = box_xy
            self._state             = "NAVIGATE_TO_CUBE"
            self._state_step        = 0
            print(f"[{self.name}] ArUco ID={aruco_id} 감지 "
                  f"박스={box_xy.round(2)}  목표={self._goal_xy.round(2)}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] detect 오류: {e}")

    def _try_find_box_direct(self):
        """Fallback: pick the nearest unplaced box and send it to an A-side marker."""
        try:
            pos, _ = self._get_pos_yaw()
            box_path, box_xy = self._find_nearest_autobox(
                pos[:2], max_dist=8.0, skip_goal_zones=True)
            if box_xy is None:
                return
            self._detected_aruco_id = None
            self._goal_xy           = self._select_goal_for_box(0, box_path)
            self._cube_nav          = box_xy
            self._state             = "NAVIGATE_TO_CUBE"
            self._state_step        = 0
            print(f"[{self.name}] Box 직접 감지  박스={box_xy.round(2)} "
                  f"목표={self._goal_xy.round(2)}")
        except Exception as e:
            carb.log_warn(f"[{self.name}] direct box detect 오류: {e}")

    # ── 내비게이션 ───────────────────────────────────────────────────

    def _get_pos_yaw(self):
        pos, quat = self._spot.robot.get_world_pose()
        yaw = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz")[2]
        return pos, yaw

    def _set_nearest_patrol_waypoint(self) -> None:
        try:
            pos, _ = self._get_pos_yaw()
            dists = [float(np.linalg.norm(wp - pos[:2])) for wp in self._waypoints]
            self._wp_idx = int(np.argmin(dists))
            self._active_detour = None
            self._detour_cooldown = 0
        except Exception:
            pass

    def _nav_toward(self, tgt_xy: np.ndarray, speed: float = None) -> np.ndarray:
        speed = speed or _SPEED
        try:
            pos, yaw = self._get_pos_yaw()
            tgt_xy = _pod_safe_detour(pos[:2], np.array(tgt_xy, dtype=np.float64))
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
            # pod 섹션 안에 있으면 즉시 탈출 목표로 향함
            escape = _clamp_clear_of_pods(pos[:2])
            if not np.allclose(escape, pos[:2], atol=0.05):
                ey = np.arctan2(escape[1]-pos[1], escape[0]-pos[0]) - yaw
                ey = (ey + np.pi) % (2*np.pi) - np.pi
                return np.array([_SPEED, 0.0, float(np.clip(_Kp * ey, -0.7, 0.7))])
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

    # ── 상호 충돌 회피 ────────────────────────────────────────────────

    def _should_yield(self) -> bool:
        """True 이면 이 Spot 이 정지해야 함 (상위 우선순위 Spot 가 _AVOID_DIST 이내).

        레지스트리에 먼저 등록된 Spot (낮은 인덱스) 이 상위 우선순위.
        하위 우선순위 Spot 만 정지하므로 두 Spot 이 동시에 멈추는 교착 없음.
        """
        try:
            my_idx = SpotAgent._registry.index(self)
            my_pos, _ = self._get_pos_yaw()
            for other in SpotAgent._registry:
                if other is self:
                    continue
                other_idx = SpotAgent._registry.index(other)
                if other_idx >= my_idx:
                    continue  # 동등하거나 하위 우선순위 → 양보 불필요
                if not other._initialized or not other._stable:
                    continue
                other_pos, _ = other._get_pos_yaw()
                dist = float(np.linalg.norm(my_pos[:2] - other_pos[:2]))
                if dist < _AVOID_DIST:
                    return True
        except Exception:
            pass
        return False

    def _compute_detour_around_lower(self) -> np.ndarray | None:
        """고우선순위 Spot이 정지한 하위 우선순위 Spot을 측방+전방으로 우회할 목표 지점 계산.

        우회 목표 = 정지 로봇 위치 + 측방(_DETOUR_SIDE) + 진행 방향 전방(_DETOUR_FWD).
        '측방만' 우회하면 다음 웨이포인트로 돌아오다 다시 정지 로봇에 걸려 무한 루프가 되므로
        전방 성분을 추가해 정지 로봇을 확실히 지나치도록 한다.
        """
        try:
            my_idx = SpotAgent._registry.index(self)
            my_pos, _ = self._get_pos_yaw()
            for other in SpotAgent._registry:
                if other is self:
                    continue
                other_idx = SpotAgent._registry.index(other)
                if other_idx <= my_idx:
                    continue  # 동급 또는 고우선순위 → 우회 대상 아님
                if not other._initialized or not other._stable:
                    continue
                other_pos, _ = other._get_pos_yaw()
                dist = float(np.linalg.norm(my_pos[:2] - other_pos[:2]))
                if dist < _AVOID_DIST:
                    # 현재 진행 방향 (다음 웨이포인트 기준)
                    next_wp = self._waypoints[self._wp_idx]
                    tgt_dir = next_wp - my_pos[:2]
                    nd = np.linalg.norm(tgt_dir)
                    tgt_dir = tgt_dir / nd if nd > 0.01 else np.array([1.0, 0.0])
                    # 진행 방향 수직 중 내 현재 위치 방향과 일치하는 쪽 선택
                    perp1 = np.array([-tgt_dir[1], tgt_dir[0]])
                    perp2 = -perp1
                    diff  = my_pos[:2] - other_pos[:2]
                    perp  = perp1 if np.dot(perp1, diff) >= np.dot(perp2, diff) else perp2
                    # 우회 목표: 측방으로 비켜 + 진행 방향으로 지나쳐
                    detour = other_pos[:2] + perp * _DETOUR_SIDE + tgt_dir * _DETOUR_FWD
                    detour = _clamp_clear_of_pods(
                        np.array([detour[0], detour[1]], dtype=np.float64))
                    return detour
        except Exception:
            pass
        return None

    # ── 상태머신 ─────────────────────────────────────────────────────

    def _run_fsm(self) -> np.ndarray:
        self._state_step += 1
        cmd = np.zeros(3)

        # 하위 우선순위: 상위 우선순위 Spot이 근접하면 완전 정지 대기
        if self._state == "WALKING" and self._should_yield():
            return np.zeros(3)

        if self._state == "WALKING":
            if self._pickup_cooldown > 0:
                self._pickup_cooldown -= 1
            # 고우선순위: 정지 중인 하위 우선순위 Spot 우회 (persistent detour + cooldown)
            if self._detour_cooldown > 0:
                # 우회 완료 후 쿨다운 중 — 일반 주행으로 멀어지도록 대기
                self._detour_cooldown -= 1
            elif self._active_detour is not None:
                # 활성 우회 목표 추적 중
                pos, _ = self._get_pos_yaw()
                if np.linalg.norm(pos[:2] - self._active_detour) <= _LOOK_AHEAD:
                    # 우회 목표 도착 → 쿨다운 진입, 일반 주행으로 복귀
                    self._active_detour   = None
                    self._detour_cooldown = _DETOUR_COOLDOWN
                else:
                    return self._nav_toward(self._active_detour)
            else:
                # 우회 필요 여부 검사 (쿨다운 없을 때만)
                detour = self._compute_detour_around_lower()
                if detour is not None:
                    self._active_detour = detour
                    return self._nav_toward(detour)

            cmd = self._waypoint_cmd()
            if self._pickup_cooldown <= 0:
                self._det_cnt += 1
                if self._det_cnt >= _DETECT_EVERY:
                    self._det_cnt = 0
                    self._try_detect_aruco()           # ArUco ID 탐지

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
                self._attach_nearest_autobox()     # AutoBox 흡착
                self._sync_autobox_to_gripper()
                self._state      = "RAISE"
                self._state_step = 0
                print(f"[{self.name}] → RAISE")

        elif self._state == "RAISE":
            self._sync_autobox_to_gripper()
            if self._state_step >= _RAISE_STEPS:
                self._lower_start = None
                goal = self._goal_xy if self._goal_xy is not None else self._home_xy
                self._state      = "NAVIGATE_TO_GOAL"
                self._state_step = 0
                print(f"[{self.name}] → NAVIGATE_TO_GOAL  목표={goal.round(2)}")

        elif self._state == "NAVIGATE_TO_GOAL":
            # ArUco ID 에 해당하는 목표로 이동, 없으면 홈으로
            goal = self._goal_xy if self._goal_xy is not None else self._home_xy
            cmd  = self._nav_toward(goal)
            self._sync_autobox_to_gripper()
            try:
                pos, _ = self._get_pos_yaw()
                if (np.linalg.norm(pos[:2] - goal) < _HOME_DIST or
                        self._state_step > 2000):
                    self._state      = "RELEASE"
                    self._state_step = 0
                    self._trigger_open()
                    print(f"[{self.name}] → RELEASE  at {goal.round(2)}")
            except Exception:
                pass

        elif self._state == "RELEASE":
            self._sync_autobox_to_gripper()
            if self._ganim_state == "idle" or self._state_step > _ANIM_STEPS + 80:
                self._gripped           = False
                self._detach_autobox()             # AutoBox 해제
                self._detected_aruco_id = None
                self._goal_xy           = None
                self._cube_nav          = None
                self._state             = "WALKING"
                self._state_step        = 0
                self._pickup_cooldown   = 500
                self._set_nearest_patrol_waypoint()
                print(f"[{self.name}] 배치 완료 → WALKING")

        return cmd

    # ── 미니맵용 위치 조회 ────────────────────────────────────────────
    def get_world_xy(self) -> tuple:
        """(x, y, heading_rad) — 미니맵용."""
        if not self._initialized or not self._stable:
            return (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]),
                    math.radians(self.spawn_yaw))
        try:
            pos, yaw = self._get_pos_yaw()
            return (float(pos[0]), float(pos[1]), float(yaw))
        except Exception:
            return (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]),
                    math.radians(self.spawn_yaw))
