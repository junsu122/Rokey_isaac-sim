"""
spot_pick.py
============
- 팔 없는 Spot + SpotFlatTerrainPolicy 보행
- 그리퍼가 Spot 앞에 부착 (Spot 추종)
- RealSense 카메라로 파란 큐브 감지
- 완전한 픽앤플레이스 상태 머신:
    WALKING → NAVIGATE_TO_CUBE → LOWER(고개 숙이기)
    → GRASP(잡기) → RAISE(고개 들기)
    → RETURN_HOME(귀환) → RELEASE(내려놓기) → DONE
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "exts": ["omni.isaac.ros2_bridge", "omni.isaac.core_nodes", "omni.graph.action"]
})

import sys
import time
import numpy as np
import carb
import omni.usd
import omni.kit.app
import cv2
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf, UsdPhysics, Usd

try:
    from isaacsim.core.utils.types import ArticulationAction
except ImportError:
    from omni.isaac.core.utils.types import ArticulationAction

sys.path.insert(0, "/home/rokey/dev_ws/isaac_sim/src/spot_test")
from realsense_mount import attach_realsense_d455
from wrist_camera import WristCamera

try:
    from omni.isaac.robot_policy.examples.robots import SpotFlatTerrainPolicy
except ImportError:
    from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# ================================================================================
# 경로 / 카메라 설정
# ================================================================================
assets_root  = get_assets_root_path()
SPOT_USD     = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
GRIPPER_USD  = "/home/rokey/dev_ws/isaac_sim/src/onrobot_rg2/urdf/onrobot_rg2/onrobot_rg2.usd"
GRIPPER_PATH = "/World/Spot/Gripper"

GRIPPER_ROT_OFFSET = R.from_euler("xyz", [110.0, 0.0, 90.0], degrees=True)

# 그리퍼 오프셋: 정상 위치 vs 낮은 위치(큐브 파지용)
# 값이 맞지 않으면 여기만 조정
GRIPPER_OFFSET_NORMAL = np.array([0.30, 0.0, -0.65], dtype=np.float64)
GRIPPER_OFFSET_LOW    = np.array([0.55, 0.0, -0.72], dtype=np.float64)

GRIPPER_ANGLE_BRACKET = f"{GRIPPER_PATH}/angle_bracket"
CAM_OFFSET_T         = (0.0, 0.045, 0.05)
CAM_OFFSET_RPY       = (0.0, -90.0, -90.0)
CAM_RES              = (640, 480)
CAM_SENSOR_EXTRA_RPY = (0.0, 0.0, 90.0)

# 그리퍼 finger 링크
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
GRIPPER_OPEN_ANGLE = 0.7
GRIPPER_ANIM_STEPS = 250   # 500Hz 기준 0.5초

# ── Spot 쪼그리기 자세 (스크린샷 관절값, 단위: 도) ────────────────────────────
# LOWER 진입 시 이 자세로 보간 → RAISE 시 역방향 복원
CROUCH_JOINTS_DEG = {
    "fl_hx":  23.1,   "fl_hy":  68.3,  "fl_kn": -99.8,
    "fr_hx": -23.1,   "fr_hy":  68.3,  "fr_kn": -99.8,
    "hl_hx":   27.0,  "hl_hy":  63.11, "hl_kn": -86.11,
    "hr_hx":  -27.0,  "hr_hy":  63.11, "hr_kn": -86.11,
}

# 런타임에 채워지는 쪼그리기 관련 전역변수
_crouch_indices  = None   # 관절 인덱스 배열
_crouch_targets  = None   # 목표값 배열 (rad)
_lower_start_pos = None   # LOWER 진입 시점 캡처값 (복원 기준)

# ================================================================================
# 픽앤플레이스 파라미터
# ================================================================================
CUBE_PATH        = "/World/BlueCube"
CUBE_SCALE       = 0.05

BLUE_HSV_LOWER   = np.array([100, 120,  80])
BLUE_HSV_UPPER   = np.array([130, 255, 255])
DETECT_MIN_AREA  = 300
DETECT_CENTER_TOL = 0.25   # 화면 중앙 허용 편차 비율

CUBE_STOP_DIST   = 0.65   # 큐브로부터 이 거리에서 정지
CUBE_APPROACH_DIST = 1.2  # 이 거리부터 감속

HOME_POS         = np.array([0.0, 0.0])
HOME_ARRIVE_DIST = 0.45

LOWER_ANIM_STEPS = 300   # 그리퍼 내리기/올리기 스텝 수 (0.6초)
RAISE_ANIM_STEPS = 300

DETECT_INTERVAL  = 10    # 감지 주기 (physics 스텝)

# ================================================================================
# World
# ================================================================================
my_world = World(stage_units_in_meters=1.0, physics_dt=1/500, rendering_dt=1/50)

prim = define_prim("/World/Ground", "Xform")
prim.GetReferences().AddReference(
    assets_root + "/Isaac/Environments/Grid/default_environment.usd"
)
##########내가하는 맵 가져와서 쓸때#########################################
# prim.GetReferences().AddReference("/home/rokey/Downloads/my_map.usd")
#######################################################################
light = define_prim("/World/DistantLight", "DistantLight")
light.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

# 장애물 콘 (큐브와 다른 위치)
obs = define_prim("/World/Cone", "Cone")
obs_xf = UsdGeom.Xformable(obs)
obs_xf.ClearXformOpOrder()
obs_xf.AddTranslateOp().Set(Gf.Vec3d(2.5, 1.5, 0.15))
obs_xf.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
obs.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
    [Gf.Vec3f(1.0, 0.35, 0.0)]
)
UsdPhysics.CollisionAPI.Apply(obs)

# ================================================================================
# 파란 큐브 스폰 (Spot 앞 2m)
# ================================================================================
cube_prim = define_prim(CUBE_PATH, "Cube")
_cube_xf  = UsdGeom.Xformable(cube_prim)
_cube_xf.ClearXformOpOrder()
_cube_translate_op = _cube_xf.AddTranslateOp()
_cube_translate_op.Set(Gf.Vec3d(2.0, 0.0, CUBE_SCALE / 2))
_cube_xf.AddScaleOp().Set(Gf.Vec3f(CUBE_SCALE, CUBE_SCALE, CUBE_SCALE))
cube_prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
    [Gf.Vec3f(0.0, 0.2, 1.0)]
)
UsdPhysics.CollisionAPI.Apply(cube_prim)
_cube_rb = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
_cube_rb.GetRigidBodyEnabledAttr().Set(True)
_cube_mass = UsdPhysics.MassAPI.Apply(cube_prim)
_cube_mass.GetMassAttr().Set(0.01)   # 단위: kg, 원하는 값으로 변경

# ================================================================================
# Spot
# ================================================================================
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    usd_path=SPOT_USD,
    position=np.array([0.0, 0.0, 0.65]),
)

# ================================================================================
# 그리퍼 스폰
# ================================================================================
gripper_xform = define_prim(GRIPPER_PATH, "Xform")
gripper_xform.GetReferences().AddReference(GRIPPER_USD)

# ================================================================================
# 그리퍼 physics 제거
# ================================================================================
def _remove_gripper_physics():
    stage   = my_world.stage
    removed = 0
    joints_to_deactivate = []
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if not path_str.startswith(GRIPPER_PATH):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI); removed += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI);        removed += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI);        removed += 1
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI);             removed += 1
        if prim.IsA(UsdPhysics.Joint) or prim.GetTypeName() in (
            "PhysicsRevoluteJoint", "PhysicsPrismaticJoint",
            "PhysicsFixedJoint",    "PhysicsSphericalJoint", "Joint",
            "RevoluteJoint",        "PrismaticJoint",
            "FixedJoint",           "SphericalJoint",
        ):
            joints_to_deactivate.append(prim.GetPath())
    for path in joints_to_deactivate:
        p = stage.GetPrimAtPath(path)
        if p.IsValid():
            p.SetActive(False); removed += 1
    print(f"[Gripper] physics {removed}개 제거/비활성화 완료")

# ================================================================================
# 그리퍼 XformOp + Spot 추종 동기화 (동적 오프셋 사용)
# ================================================================================
_gripper_translate_op   = None
_gripper_orient_op      = None
_current_gripper_offset = GRIPPER_OFFSET_NORMAL.copy()
_gripper_world_pos      = np.zeros(3)   # 매 스텝 캐시
_gripper_world_rot      = R.identity()  # 매 스텝 캐시 (그리퍼 orientation)

# 파지점 기준 큐브 위치 오프셋 (월드 프레임, 단위: m)
# 핑거팁 중점에서 추가로 이동할 필요가 있을 때만 조정
CUBE_GRIP_OFFSET = np.array([0.0, 0.0, 0.0])

def _init_gripper_xform():
    global _gripper_translate_op, _gripper_orient_op
    stage = my_world.stage
    root  = stage.GetPrimAtPath(GRIPPER_PATH)
    if not root.IsValid():
        carb.log_warn("[Gripper] prim 없음")
        return
    xf = UsdGeom.Xformable(root)
    xf.ClearXformOpOrder()
    _gripper_translate_op = xf.AddTranslateOp()
    _gripper_orient_op    = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    print("[Gripper] XformOp 초기화 완료")

def _sync_gripper_pose():
    global _gripper_world_pos, _gripper_world_rot
    if _gripper_translate_op is None:
        return
    if not hasattr(spot, "robot") or spot.robot is None:
        return
    try:
        body_pos, body_quat = spot.robot.get_world_pose()
        rot          = R.from_quat([body_quat[1], body_quat[2],
                                    body_quat[3], body_quat[0]])
        offset_world = rot.apply(_current_gripper_offset)
        gripper_pos  = body_pos + offset_world
        _gripper_world_pos = gripper_pos
        _gripper_world_rot = rot * GRIPPER_ROT_OFFSET   # 핑거팁 방향 계산용

        _gripper_translate_op.Set(Gf.Vec3d(float(gripper_pos[0]),
                                            float(gripper_pos[1]),
                                            float(gripper_pos[2])))
        qf = _gripper_world_rot.as_quat()
        _gripper_orient_op.Set(Gf.Quatd(float(qf[3]), float(qf[0]),
                                         float(qf[1]), float(qf[2])))
    except Exception as e:
        carb.log_warn(f"[Gripper] pose 동기화 실패: {e}")

# ================================================================================
# 큐브 유틸
# ================================================================================
def _set_cube_kinematic(enabled: bool):
    """enabled=True → 물리 비활성화(위치 수동 제어), False → 물리 복원"""
    _cube_rb.GetRigidBodyEnabledAttr().Set(not enabled)

def _set_cube_pos(pos: np.ndarray):
    _cube_translate_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

def _get_cube_world_pos() -> np.ndarray:
    mat = _cube_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())

def _get_grip_center_world_pos() -> np.ndarray:
    """right_inner_finger / left_inner_finger USD prim의 실제 월드 좌표 중점을 반환.
    두 prim의 ComputeLocalToWorldTransform()을 사용하므로 offset 추정이 필요 없음.
    prim이 없으면 gripper root 위치를 폴백으로 반환."""
    stage = my_world.stage
    positions = []
    for link_name in ("right_inner_finger", "left_inner_finger"):
        p = stage.GetPrimAtPath(f"{GRIPPER_PATH}/{link_name}")
        if p.IsValid():
            mat = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            positions.append(np.array(mat.ExtractTranslation()))
    if positions:
        return np.mean(positions, axis=0) + CUBE_GRIP_OFFSET
    return _gripper_world_pos + CUBE_GRIP_OFFSET   # 폴백

def _sync_cube_to_gripper():
    """실제 핑거팁 prim 월드 좌표로 큐브를 이동.
    지면 아래로 내려가지 않도록 Z를 클램핑."""
    if not _is_gripped:
        return
    grip_pos    = _get_grip_center_world_pos()
    # grip_pos[0] = max(float(grip_pos[0])+10)
    grip_pos[2] = max(float(grip_pos[2]), CUBE_SCALE / 2)
    _set_cube_pos(grip_pos)

# ================================================================================
# RealSense 부착
# ================================================================================
_realsense_prim_path = None
_wrist_camera        = None

def _find_prim_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root  = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for p in Usd.PrimRange(root):
        if p.GetName() == name:
            return str(p.GetPath())
    return None

def _setup_realsense():
    global _realsense_prim_path, _wrist_camera
    ab_prim = my_world.stage.GetPrimAtPath(GRIPPER_ANGLE_BRACKET)
    if not ab_prim.IsValid():
        carb.log_warn(f"[Camera] angle_bracket 없음: {GRIPPER_ANGLE_BRACKET}")
        return
    _realsense_prim_path = attach_realsense_d455(
        parent_prim_path=GRIPPER_ANGLE_BRACKET,
        child_name="realsense_d455",
        translation=CAM_OFFSET_T,
        rpy_deg=CAM_OFFSET_RPY,
    )
    _OV_CAM_NAME = "Camera_OmniVision_OV9782_Color"
    ov_cam_path  = _find_prim_by_name(_realsense_prim_path, _OV_CAM_NAME)
    if ov_cam_path:
        stage    = omni.usd.get_context().get_stage()
        cam_prim = stage.GetPrimAtPath(ov_cam_path)
        cam_xf   = UsdGeom.Xformable(cam_prim)
        existing = [op.GetOpName() for op in cam_xf.GetOrderedXformOps()]
        rot_op   = cam_xf.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat, opSuffix="extra")
        rot_op.Set(float(CAM_SENSOR_EXTRA_RPY[2]))
        from pxr import Vt
        cam_prim.GetAttribute("xformOpOrder").Set(
            Vt.TokenArray(existing + [rot_op.GetOpName()])
        )
        _wrist_camera = WristCamera.from_existing_prim(prim_path=ov_cam_path, resolution=CAM_RES)
    else:
        _wrist_camera = WristCamera(
            parent_prim_path=_realsense_prim_path,
            name="wrist_rgb", resolution=CAM_RES, rpy_deg=CAM_SENSOR_EXTRA_RPY,
        )
    print(f"[Camera] WristCamera prim = {_wrist_camera._prim_path}")

def _disable_realsense_physics_post_reset():
    if not _realsense_prim_path:
        return
    stage = omni.usd.get_context().get_stage()
    for p in Usd.PrimRange(stage.GetPrimAtPath(_realsense_prim_path)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

# ================================================================================
# Spot 쪼그리기 관절 제어
# ================================================================================
def _init_crouch_joints():
    """spot.initialize() 이후 호출 — DOF 이름으로 관절 인덱스·목표값 설정."""
    global _crouch_indices, _crouch_targets
    try:
        dof_names = list(spot.robot.dof_names)
        print(f"[Crouch] Spot DOF names: {dof_names}")
        indices, targets = [], []
        for name, deg in CROUCH_JOINTS_DEG.items():
            # 정확히 일치하거나 접미사로 포함된 이름을 검색
            idx = next(
                (i for i, n in enumerate(dof_names)
                 if n == name or n.endswith(f"/{name}") or n.endswith(f"_{name}")),
                -1,
            )
            if idx >= 0:
                indices.append(idx)
                targets.append(np.deg2rad(deg))
            else:
                carb.log_warn(f"[Crouch] 관절 없음: {name}")
        _crouch_indices = np.array(indices, dtype=int)
        _crouch_targets = np.array(targets, dtype=np.float64)
        print(f"[Crouch] {len(indices)}개 관절 매핑 완료")
    except Exception as e:
        carb.log_warn(f"[Crouch] 초기화 실패: {e}")

def _apply_crouch_blend():
    """
    spot.forward() 이후 호출 — policy 출력 위에 crouch drive target을 덮어씀.

    set_joint_positions() (물리 무시, 넘어짐) 대신 apply_action() 사용:
    PhysX PD 제어가 살아있어 자연스럽게 자세 전환하고 균형을 유지함.
    _state / _state_step 을 직접 읽어 t 를 계산하므로 인자 없음.
    """
    global _lower_start_pos
    if _crouch_indices is None or len(_crouch_indices) == 0:
        return

    if _state == "LOWER":
        t = min(_state_step / LOWER_ANIM_STEPS, 1.0)
    elif _state == "GRASP":
        t = 1.0
    elif _state == "RAISE":
        t = 1.0 - min(_state_step / RAISE_ANIM_STEPS, 1.0)
    else:
        return

    try:
        if _lower_start_pos is None:
            # LOWER 최초 진입 시 현재 관절 위치를 기준으로 캡처
            all_pos = spot.robot.get_joint_positions()
            _lower_start_pos = all_pos[_crouch_indices].copy()

        target = _lower_start_pos * (1.0 - t) + _crouch_targets * t

        # apply_action() 으로 drive target 설정 → PD 제어기가 토크를 계산해 균형 유지
        spot.robot.apply_action(ArticulationAction(
            joint_positions=target,
            joint_indices=_crouch_indices,
        ))
    except Exception as e:
        carb.log_warn(f"[Crouch] blend 실패: {e}")

# ================================================================================
# 그리퍼 손가락 애니메이션 (FK 기반)
# ================================================================================
_finger_link_data   = {}
_gripper_anim_state = "idle"   # idle | opening | closing
_gripper_anim_step  = 0

def _init_finger_links():
    global _finger_link_data
    stage = my_world.stage

    def _load(link_name):
        path = f"{GRIPPER_PATH}/{link_name}"
        p    = stage.GetPrimAtPath(path)
        if not p.IsValid():
            carb.log_warn(f"[Gripper] finger link 없음: {path}")
            return None
        xf    = UsdGeom.Xformable(p)
        mat   = xf.GetLocalTransformation(Usd.TimeCode.Default())
        trans = np.array(mat.ExtractTranslation())
        rot_q = mat.ExtractRotationQuat()
        img   = rot_q.GetImaginary()
        xf.ClearXformOpOrder()
        t_op = xf.AddTranslateOp()
        t_op.Set(Gf.Vec3d(*map(float, trans)))
        o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        o_op.Set(rot_q)
        return {"translate_op": t_op, "orient_op": o_op,
                "base_trans": trans,
                "base_quat": np.array([img[0], img[1], img[2], rot_q.GetReal()])}

    for link_name, axis_sign in _ROTATOR_LINKS.items():
        d = _load(link_name)
        if d:
            d["axis_sign"] = axis_sign
            _finger_link_data[link_name] = d
    for link_name, parent_name in _FOLLOWER_LINKS.items():
        d = _load(link_name)
        if d is None or parent_name not in _finger_link_data:
            continue
        pd = _finger_link_data[parent_name]
        parent_rot0 = R.from_quat(pd["base_quat"])
        d["parent_name"]         = parent_name
        d["rel_trans_in_parent"] = parent_rot0.inv().apply(
            d["base_trans"] - pd["base_trans"])
        _finger_link_data[link_name] = d
    print(f"[Gripper] finger links 초기화: {list(_finger_link_data.keys())}")

def _set_finger_angle(angle: float):
    for link_name, axis_sign in _ROTATOR_LINKS.items():
        if link_name not in _finger_link_data:
            continue
        data    = _finger_link_data[link_name]
        cur_rot = R.from_quat(data["base_quat"]) * R.from_euler("y", axis_sign * angle)
        fq = cur_rot.as_quat()
        data["orient_op"].Set(Gf.Quatd(float(fq[3]),
                                        float(fq[0]), float(fq[1]), float(fq[2])))
        data["_cur_rot"] = cur_rot
    for link_name in _FOLLOWER_LINKS:
        if link_name not in _finger_link_data:
            continue
        data = _finger_link_data[link_name]
        pd   = _finger_link_data.get(data["parent_name"])
        if pd is None:
            continue
        parent_rot = pd.get("_cur_rot", R.from_quat(pd["base_quat"]))
        new_trans  = pd["base_trans"] + parent_rot.apply(data["rel_trans_in_parent"])
        data["translate_op"].Set(Gf.Vec3d(*map(float, new_trans)))

def _trigger_close():
    global _gripper_anim_state, _gripper_anim_step
    if _gripper_anim_state == "idle" and _finger_link_data:
        _gripper_anim_state = "closing"
        _gripper_anim_step  = 0
        print("[Gripper] 닫기 시작")

def _trigger_open():
    global _gripper_anim_state, _gripper_anim_step
    if _gripper_anim_state == "idle" and _finger_link_data:
        _gripper_anim_state = "opening"
        _gripper_anim_step  = 0
        print("[Gripper] 열기 시작")

def _update_gripper_animation():
    global _gripper_anim_state, _gripper_anim_step
    if _gripper_anim_state == "idle":
        return
    _gripper_anim_step += 1
    if _gripper_anim_state == "opening":
        t = min(_gripper_anim_step / GRIPPER_ANIM_STEPS, 1.0)
        _set_finger_angle(t * GRIPPER_OPEN_ANGLE)
        if _gripper_anim_step >= GRIPPER_ANIM_STEPS:
            _gripper_anim_state = "idle"
            _gripper_anim_step  = 0
            print("[Gripper] 열기 완료")
    elif _gripper_anim_state == "closing":
        t = 1.0 - min(_gripper_anim_step / GRIPPER_ANIM_STEPS, 1.0)
        _set_finger_angle(t * GRIPPER_OPEN_ANGLE)
        if _gripper_anim_step >= GRIPPER_ANIM_STEPS:
            _set_finger_angle(0.0)
            _gripper_anim_state = "idle"
            _gripper_anim_step  = 0
            print("[Gripper] 닫기 완료")

# ================================================================================
# 파란 큐브 감지
# ================================================================================
def _detect_blue_cube(frame: np.ndarray):
    if frame is None or frame.size == 0:
        return False, 0.0, 0
    bgr  = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_HSV_LOWER, BLUE_HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0, 0
    largest = max(contours, key=cv2.contourArea)
    area    = int(cv2.contourArea(largest))
    if area < DETECT_MIN_AREA:
        return False, 0.0, area
    M  = cv2.moments(largest)
    cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else frame.shape[1] // 2
    cx_ratio = (cx / frame.shape[1]) - 0.5
    return True, cx_ratio, area

# ================================================================================
# 보행 유틸
# ================================================================================
Kp                  = 1.6
look_ahead_distance = 0.55
target_speed        = 0.55

waypoints = [
    np.array([3.0,  0.0]),
    np.array([3.0, -1.5]),
    np.array([0.0, -1.5]),
    np.array([0.0,  0.0]),
]
wp_idx = 0

def _get_spot_pos_yaw():
    pos, quat = spot.robot.get_world_pose()
    yaw = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz")[2]
    return pos, yaw

def _navigate_toward(target_xy: np.ndarray, speed: float = None) -> np.ndarray:
    """target_xy 방향으로 이동 명령 생성."""
    speed = speed or target_speed
    try:
        pos, yaw = _get_spot_pos_yaw()
        dist     = np.linalg.norm(pos[:2] - target_xy)
        vx       = speed * min(1.0, dist / CUBE_APPROACH_DIST)
        err_yaw  = np.arctan2(target_xy[1] - pos[1], target_xy[0] - pos[0]) - yaw
        err_yaw  = (err_yaw + np.pi) % (2 * np.pi) - np.pi
        return np.array([vx, 0.0, float(np.clip(Kp * err_yaw, -0.7, 0.7))])
    except Exception:
        return np.zeros(3)

def _waypoint_command() -> np.ndarray:
    global wp_idx
    try:
        pos, yaw = _get_spot_pos_yaw()
        tgt = waypoints[wp_idx]
        if np.linalg.norm(pos[:2] - tgt) < look_ahead_distance:
            wp_idx = (wp_idx + 1) % len(waypoints)
            tgt    = waypoints[wp_idx]
            print(f"[Spot] 웨이포인트 → {wp_idx} {tgt}")
        err_yaw = np.arctan2(tgt[1] - pos[1], tgt[0] - pos[0]) - yaw
        err_yaw = (err_yaw + np.pi) % (2 * np.pi) - np.pi
        return np.array([target_speed, 0.0, float(np.clip(Kp * err_yaw, -0.7, 0.7))])
    except Exception:
        return np.zeros(3)

# ================================================================================
# 픽앤플레이스 상태 머신
# ================================================================================
_state           = "WALKING"
_state_step      = 0
_cube_nav_pos    = None    # 감지 시점 큐브 2D 위치 (내비게이션 목표)
_is_gripped      = False
_detect_counter  = 0

def _try_detect_and_navigate():
    """카메라로 감지해서 NAVIGATE_TO_CUBE 전이. WALKING 상태에서만 호출."""
    global _state, _state_step, _cube_nav_pos
    if _wrist_camera is None:
        return
    try:
        rgb = _wrist_camera.get_rgb()   # (H, W, 3) or None
        if rgb is None or rgb.size == 0:
            return
        detected, cx_ratio, area = _detect_blue_cube(rgb)
        if detected and abs(cx_ratio) < DETECT_CENTER_TOL:
            # 큐브 월드 위치 조회 (USD transform)
            cube_world = _get_cube_world_pos()
            _cube_nav_pos = cube_world[:2].copy()
            print(f"[Detect] 큐브 발견! pos={cube_world.round(3)}, area={area}")
            _state      = "NAVIGATE_TO_CUBE"
            _state_step = 0
    except Exception as e:
        carb.log_warn(f"[Detect] 오류: {e}")

def _run_state_machine() -> np.ndarray:
    """매 스텝 호출. 반환: Spot velocity command [vx, vy, wz]"""
    global _state, _state_step, _is_gripped, _current_gripper_offset, _detect_counter, _lower_start_pos

    _state_step += 1
    cmd = np.zeros(3)

    # ── WALKING: 웨이포인트 순회 + 주기적 감지 ─────────────────────────────
    if _state == "WALKING":
        cmd = _waypoint_command()
        _detect_counter += 1
        if _detect_counter >= DETECT_INTERVAL:
            _detect_counter = 0
            _try_detect_and_navigate()

    # ── NAVIGATE_TO_CUBE: 큐브 위치로 직접 이동 ────────────────────────────
    elif _state == "NAVIGATE_TO_CUBE":
        if _cube_nav_pos is None:
            _state = "WALKING"
            return cmd
        cmd = _navigate_toward(_cube_nav_pos)
        try:
            pos, _ = _get_spot_pos_yaw()
            dist   = np.linalg.norm(pos[:2] - _cube_nav_pos)
            if dist < CUBE_STOP_DIST:
                print(f"[State] NAVIGATE_TO_CUBE → LOWER (dist={dist:.2f}m)")
                _lower_start_pos = None   # 이번 LOWER 시작 시점 재캡처
                _state      = "LOWER"
                _state_step = 0
        except Exception:
            pass

    # ── LOWER: 정지 (관절 쪼그리기는 on_physics_step에서 blend) ──────────────
    # 그리퍼 오프셋은 변경하지 않음 → Spot 몸통 하강에 따라 자연스럽게 내려감
    elif _state == "LOWER":
        cmd = np.zeros(3)
        if _state_step >= LOWER_ANIM_STEPS:
            print("[State] LOWER → GRASP")
            _state      = "GRASP"
            _state_step = 0
            _trigger_close()

    # ── GRASP: 정지 + 그리퍼 닫기 완료 대기 (관절은 on_physics_step에서 유지)
    elif _state == "GRASP":
        cmd = np.zeros(3)
        if _gripper_anim_state == "idle":
            _is_gripped = True
            _set_cube_kinematic(True)
            _sync_cube_to_gripper()   # 파지 즉시 그리퍼 앞에 배치
            print("[State] GRASP → RAISE")
            _state      = "RAISE"
            _state_step = 0

    # ── RAISE: 정지 (관절 복원은 on_physics_step에서 blend) ──────────────────
    # 그리퍼 오프셋은 변경하지 않음 → 몸통 복원에 따라 자연스럽게 올라감
    elif _state == "RAISE":
        cmd = np.zeros(3)
        _sync_cube_to_gripper()
        if _state_step >= RAISE_ANIM_STEPS:
            _lower_start_pos = None
            print("[State] RAISE → RETURN_HOME")
            _state      = "RETURN_HOME"
            _state_step = 0

    # ── RETURN_HOME: 원점으로 귀환, 큐브 동기화 ────────────────────────────
    elif _state == "RETURN_HOME":
        cmd = _navigate_toward(HOME_POS)
        _sync_cube_to_gripper()   # 이동 중에도 그리퍼에 붙어서 이동
        try:
            pos, _ = _get_spot_pos_yaw()
            dist   = np.linalg.norm(pos[:2] - HOME_POS)
            if dist < HOME_ARRIVE_DIST:
                print("[State] RETURN_HOME → RELEASE")
                _state      = "RELEASE"
                _state_step = 0
                _trigger_open()
        except Exception:
            pass

    # ── RELEASE: 정지 + 그리퍼 열기, 완료 시 물리 복원 → 현재 위치에서 낙하
    elif _state == "RELEASE":
        cmd = np.zeros(3)
        _sync_cube_to_gripper()   # 열리는 동안도 그리퍼 위치 유지
        if _gripper_anim_state == "idle":
            _is_gripped = False
            _set_cube_kinematic(False)   # 물리 복원 → 그리퍼 앞 위치에서 낙하
            print("[State] RELEASE → DONE ✓")
            _state = "DONE"

    # ── DONE ────────────────────────────────────────────────────────────────
    elif _state == "DONE":
        cmd = np.zeros(3)

    return cmd

# ================================================================================
# 초기화 시퀀스
# ================================================================================
print("USD 에셋 로드 중...")
for _ in range(250):
    omni.kit.app.get_app().update()
time.sleep(2.0)

_remove_gripper_physics()
_setup_realsense()

my_world.reset()
_disable_realsense_physics_post_reset()

for _ in range(10):
    omni.kit.app.get_app().update()

_init_gripper_xform()
_init_finger_links()

if _wrist_camera is not None:
    _wrist_camera.initialize()
    print("[Camera] WristCamera 초기화 완료")

# ================================================================================
# physics 콜백
# ================================================================================
WARMUP_STEPS    = 10
STABILIZE_STEPS = 1000

warmup_count    = 0
stabilize_count = 0
initialized     = False
is_stable       = False

def on_physics_step(step_size: float) -> None:
    global warmup_count, stabilize_count, initialized, is_stable

    if not initialized:
        warmup_count += 1
        if warmup_count < WARMUP_STEPS:
            return
        spot.initialize()
        _init_crouch_joints()   # DOF 이름 확인 후 관절 인덱스 설정
        initialized = True
        print("[Spot] 초기화 완료")
        return

    if not is_stable:
        stabilize_count += 1
        spot.forward(step_size, np.zeros(3))
        _sync_gripper_pose()
        if stabilize_count >= STABILIZE_STEPS:
            is_stable = True
            print("[Spot] 안정화 완료 → 주행 시작")
        return

    cmd = _run_state_machine()
    spot.forward(step_size, cmd)      # 항상 실행 → policy가 균형 유지
    _apply_crouch_blend()             # policy 이후에 drive target 덮어쓰기
    _sync_gripper_pose()
    _update_gripper_animation()

my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# ================================================================================
# 메인 루프
# ================================================================================
try:
    while simulation_app.is_running():
        my_world.step(render=True)
finally:
    my_world.clear()
    simulation_app.close()
    print("시뮬레이션 종료.")
