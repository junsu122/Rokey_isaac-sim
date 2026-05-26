"""
spot_gripper.py
===============
- 팔 없는 Spot + SpotFlatTerrainPolicy로 정상 보행
- OnRobot RG2 그리퍼를 월드 고정 위치에 배치 (Spot 추종 없음)
- m0609_pick_place_visual.py 방식으로 angle_bracket에 RealSense D455 부착
- 웨이포인트 도착마다 그리퍼 열기/닫기 애니메이션
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
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf, UsdPhysics, Usd

# m0609_aruco_detect 의 realsense_mount / wrist_camera 재사용
sys.path.insert(0, "/home/rokey/dev_ws/isaac_sim/src/m0609_aruco_detect")
from realsense_mount import attach_realsense_d455
from wrist_camera import WristCamera

try:
    from omni.isaac.robot_policy.examples.robots import SpotFlatTerrainPolicy
except ImportError:
    from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# ================================================================================
# 경로 설정
# ================================================================================
assets_root = get_assets_root_path()
SPOT_USD    = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
GRIPPER_USD = "/home/rokey/dev_ws/isaac_sim/src/onrobot_rg2/urdf/onrobot_rg2/onrobot_rg2.usd"
GRIPPER_PATH = "/World/Spot/Gripper"

# 그리퍼 Spot 몸통 기준 오프셋 (로컬 좌표, 미터) — Stage에서 확인한 값
GRIPPER_OFFSET     = np.array([0.3, 0.0, -0.65], dtype=np.float64)
GRIPPER_ROT_OFFSET = R.from_euler("xyz", [110.0, 0.0, 90.0], degrees=True)

# RealSense D455 마운트 설정 (m0609_pick_place_visual.py 와 동일)
GRIPPER_ANGLE_BRACKET = f"{GRIPPER_PATH}/angle_bracket"
CAM_OFFSET_T         = (0.0, 0.045, 0.05)   # angle_bracket 기준 오프셋 (m)
CAM_OFFSET_RPY       = (0.0, -90.0, -90.0)   # 카메라 방향 (deg)
CAM_RES              = (640, 480)
CAM_SENSOR_EXTRA_RPY = (0.0, 0.0, 90.0)     # 내장 OmniVision 센서 추가 회전

# 그리퍼 링크 분류
# ── 직접 회전체: gripper_body에 직결, Y축 기준 회전
_ROTATOR_LINKS = {
    "right_outer_knuckle": +1,   # finger_joint,              mult=+1
    "right_inner_knuckle": +1,   # right_inner_knuckle_joint, mult=+1
    "left_outer_knuckle":  -1,   # left_outer_knuckle_joint,  mult=-1
    "left_inner_knuckle":  -1,   # left_inner_knuckle_joint,  mult=-1
}
# ── 추종체: 4-bar linkage 특성으로 rotation=rest 고정, translation만 FK 계산
_FOLLOWER_LINKS = {
    "right_inner_finger": "right_outer_knuckle",
    "left_inner_finger":  "left_outer_knuckle",
}
GRIPPER_OPEN_ANGLE = 0.7   # 최대 열림 각도 (rad)
GRIPPER_ANIM_STEPS = 250   # 열기/닫기 스텝 수 (500Hz 기준 약 0.5초)
GRIPPER_HOLD_STEPS = 500   # 열린 상태 유지 스텝 수 (약 1.0초)

# ================================================================================
# World
# ================================================================================
my_world = World(stage_units_in_meters=1.0, physics_dt=1/500, rendering_dt=1/50)

prim = define_prim("/World/Ground", "Xform")
prim.GetReferences().AddReference(
    assets_root + "/Isaac/Environments/Grid/default_environment.usd"
)

light = define_prim("/World/DistantLight", "DistantLight")
light.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

obs = define_prim("/World/Cone", "Cone")
obs_xf = UsdGeom.Xformable(obs)
obs_xf.ClearXformOpOrder()
obs_xf.AddTranslateOp().Set(Gf.Vec3d(3.0, 0.0, 0.15))
obs_xf.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
obs.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
    [Gf.Vec3f(1.0, 0.35, 0.0)]
)
UsdPhysics.CollisionAPI.Apply(obs)

# ================================================================================
# 1) Spot (팔 없음)
# ================================================================================
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    usd_path=SPOT_USD,
    position=np.array([0.0, 0.0, 0.65]),
)

# ================================================================================
# 2) OnRobot RG2 그리퍼 — physics 없이 시각 모델로만 스폰
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
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI);  removed += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI);         removed += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI);         removed += 1
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI);              removed += 1
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
            p.SetActive(False);  removed += 1

    print(f"[Gripper] physics API + joint {removed}개 제거/비활성화 완료")

# ================================================================================
# 그리퍼 고정 위치 초기화 (Spot 추종 없음)
# ================================================================================
_gripper_translate_op = None
_gripper_orient_op    = None

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
    """매 스텝마다 Spot 몸통 pose + 오프셋으로 그리퍼 위치 업데이트"""
    if _gripper_translate_op is None:
        return
    if not hasattr(spot, "robot") or spot.robot is None:
        return
    try:
        body_pos, body_quat = spot.robot.get_world_pose()
        # Isaac Sim quat: [w, x, y, z] → scipy: [x, y, z, w]
        rot          = R.from_quat([body_quat[1], body_quat[2],
                                    body_quat[3], body_quat[0]])
        offset_world = rot.apply(GRIPPER_OFFSET)
        gripper_pos  = body_pos + offset_world

        _gripper_translate_op.Set(Gf.Vec3d(float(gripper_pos[0]),
                                            float(gripper_pos[1]),
                                            float(gripper_pos[2])))
        final_rot = rot * GRIPPER_ROT_OFFSET
        qf = final_rot.as_quat()  # scipy: [x, y, z, w]
        _gripper_orient_op.Set(Gf.Quatd(float(qf[3]), float(qf[0]),
                                         float(qf[1]), float(qf[2])))
    except Exception as e:
        carb.log_warn(f"[Gripper] pose 동기화 실패: {e}")

# ================================================================================
# RealSense D455 — angle_bracket에 부착 (m0609_pick_place_visual.py 방식)
# ================================================================================
_realsense_prim_path = None
_wrist_camera        = None

def _find_prim_by_name(root_path: str, name: str) -> str | None:
    stage = omni.usd.get_context().get_stage()
    root  = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for p in Usd.PrimRange(root):
        if p.GetName() == name:
            return str(p.GetPath())
    return None

def _setup_realsense():
    """angle_bracket 에 RealSense D455 USD를 자식으로 부착하고 WristCamera 준비."""
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
    print(f"[Camera] RealSense 부착: {_realsense_prim_path}")

    # 내장 OmniVision 카메라 탐색
    _OV_CAM_NAME = "Camera_OmniVision_OV9782_Color"
    ov_cam_path  = _find_prim_by_name(_realsense_prim_path, _OV_CAM_NAME)

    if ov_cam_path:
        print(f"[Camera] 내장 카메라 발견: {ov_cam_path}")
        # 센서 추가 회전 적용 (yaw 보정)
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
        print(f"[Camera] 내장 카메라 없음 — 커스텀 센서 생성")
        _wrist_camera = WristCamera(
            parent_prim_path=_realsense_prim_path,
            name="wrist_rgb",
            resolution=CAM_RES,
            rpy_deg=CAM_SENSOR_EXTRA_RPY,
        )
    print(f"[Camera] WristCamera prim = {_wrist_camera._prim_path}")

def _disable_realsense_physics_post_reset():
    """world.reset() 후 RealSense USD 내부 physics 비활성화."""
    if not _realsense_prim_path:
        return
    stage = omni.usd.get_context().get_stage()
    for p in Usd.PrimRange(stage.GetPrimAtPath(_realsense_prim_path)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

# ================================================================================
# 그리퍼 열기/닫기 애니메이션 (FK 기반)
# ================================================================================
_finger_link_data   = {}
_gripper_anim_state = "idle"
_gripper_anim_step  = 0

def _init_finger_links():
    global _finger_link_data
    stage = my_world.stage

    def _load(link_name):
        path = f"{GRIPPER_PATH}/{link_name}"
        p = stage.GetPrimAtPath(path)
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

def trigger_gripper_open_close():
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
            _gripper_anim_state = "holding"
            _gripper_anim_step  = 0

    elif _gripper_anim_state == "holding":
        if _gripper_anim_step >= GRIPPER_HOLD_STEPS:
            _gripper_anim_state = "closing"
            _gripper_anim_step  = 0
            print("[Gripper] 닫기 시작")

    elif _gripper_anim_state == "closing":
        t = 1.0 - min(_gripper_anim_step / GRIPPER_ANIM_STEPS, 1.0)
        _set_finger_angle(t * GRIPPER_OPEN_ANGLE)
        if _gripper_anim_step >= GRIPPER_ANIM_STEPS:
            _set_finger_angle(0.0)
            _gripper_anim_state = "idle"
            _gripper_anim_step  = 0
            print("[Gripper] 닫기 완료")

# ================================================================================
# 초기화 시퀀스
# ================================================================================
print("USD 에셋 로드 중...")
for _ in range(250):
    omni.kit.app.get_app().update()
time.sleep(2.0)

# 물리 제거 (그리퍼 전체)
_remove_gripper_physics()

# RealSense 부착 (물리 제거 이후 — angle_bracket 이미 존재)
_setup_realsense()

my_world.reset()

# reset 후 RealSense 내부 physics 재확인 비활성화
_disable_realsense_physics_post_reset()

for _ in range(10):
    omni.kit.app.get_app().update()

_init_gripper_xform()
_init_finger_links()

if _wrist_camera is not None:
    _wrist_camera.initialize()
    print("[Camera] WristCamera 초기화 완료")

# ================================================================================
# 경로 추종 (Spot)
# ================================================================================
Kp                  = 1.6
look_ahead_distance = 0.55
target_speed        = 0.55

waypoints = [
    np.array([4.0,  0.0]),
    np.array([4.0, -1.5]),
    np.array([0.0, -1.5]),
    np.array([0.0,  0.0]),
]
wp_idx  = 0
command = np.zeros(3)

def _update_command():
    global wp_idx
    try:
        pos, quat = spot.robot.get_world_pose()
        yaw = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz")[2]
        tgt = waypoints[wp_idx]
        if np.linalg.norm(pos[:2] - tgt) < look_ahead_distance:
            wp_idx = (wp_idx + 1) % len(waypoints)
            tgt    = waypoints[wp_idx]
            print(f"[Spot] 웨이포인트 → {wp_idx} {tgt}")
            trigger_gripper_open_close()
        err_yaw    = np.arctan2(tgt[1] - pos[1], tgt[0] - pos[0]) - yaw
        err_yaw    = (err_yaw + np.pi) % (2 * np.pi) - np.pi
        command[:] = [target_speed, 0.0, float(np.clip(Kp * err_yaw, -0.7, 0.7))]
    except Exception:
        command[:] = 0.0

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

    _update_command()
    spot.forward(step_size, command)
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