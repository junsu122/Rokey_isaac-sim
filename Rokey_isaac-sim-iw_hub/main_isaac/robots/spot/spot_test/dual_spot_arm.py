from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "exts": ["omni.isaac.ros2_bridge", "omni.isaac.core_nodes", "omni.graph.action"]
})

import carb
import time
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf, UsdPhysics

from isaacsim.sensors.camera import SingleViewDepthSensorAsset

try:
    from omni.isaac.robot_policy.examples.robots import SpotFlatTerrainPolicy
except ImportError:
    from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# ================================================================================
# 관절 수 상수
#   spot_with_arm.usd 기준: 다리 12개(앞) + 팔 7개(뒤) = 19개
#   관절 순서가 다른 경우 ARM_DEFAULT_POS 배열 순서도 함께 조정하세요.
# ================================================================================
NUM_LEG_DOF = 12
NUM_ARM_DOF = 7
TOTAL_DOF   = NUM_LEG_DOF + NUM_ARM_DOF  # 19

# 팔 관절 이름 (config_loader Warning 디버깅용 참고)
# sh0, sh1, el0, el1, wr0, wr1, f1x 순서
ARM_JOINT_NAMES = ["arm0_sh0", "arm0_sh1", "arm0_el0", "arm0_el1",
                   "arm0_wr0", "arm0_wr1", "arm0_f1x"]

# 팔 기본 자세 — 안전하게 접힌 상태
ARM_DEFAULT_POS = np.array([0.0, -1.2, 1.4, 0.0, 0.8, 0.0, 0.0], dtype=np.float32)
ARM_STIFFNESS   = np.array([3000.0] * NUM_ARM_DOF, dtype=np.float32)
ARM_DAMPING     = np.array([100.0]  * NUM_ARM_DOF, dtype=np.float32)
ARM_MAX_VEL     = np.array([3.0]    * NUM_ARM_DOF, dtype=np.float32)
ARM_MAX_EFFORT  = np.array([200.0]  * NUM_ARM_DOF, dtype=np.float32)
ARM_MAX_ACC     = np.array([10.0]   * NUM_ARM_DOF, dtype=np.float32)


# ================================================================================
# 헬퍼: 12-DOF <-> 19-DOF 변환 (NumPy / PyTorch 공통)
# ================================================================================
def _pad_to_full(field, arm_defaults: np.ndarray):
    """12-DOF -> 19-DOF 확장. shape (12,) 또는 (N,12) -> (19,) 또는 (N,19)"""
    if field is None:
        return None
    if isinstance(field, list):
        field = np.array(field, dtype=np.float32)

    is_torch = type(field).__name__ == "Tensor"
    if is_torch:
        import torch
        defaults = torch.tensor(arm_defaults, device=field.device, dtype=field.dtype)
        if field.ndim == 1:
            return torch.cat([field, defaults])
        return torch.cat([field, defaults.unsqueeze(0).expand(field.shape[0], -1)], dim=1)
    else:
        field = np.asarray(field, dtype=np.float32)
        if field.ndim == 1:
            return np.concatenate([field, arm_defaults])
        return np.concatenate([field, np.tile(arm_defaults, (field.shape[0], 1))], axis=1)


def _trim_to_leg(field):
    """19-DOF -> 12-DOF (다리 관절만 추출). (19,)->(12,) / (N,19)->(N,12)"""
    if field is None:
        return None
    if type(field).__name__ == "Tensor":
        return field[..., :NUM_LEG_DOF]
    return np.asarray(field)[..., :NUM_LEG_DOF]


# ================================================================================
# ArticulationViewProxy
#   policy_controller.py 가 내부적으로 호출하는 articulation_view 메서드를
#   모두 가로채어 12 <-> 19 DOF 변환을 수행합니다.
# ================================================================================
class ArticulationViewProxy:
    def __init__(self, original_view):
        object.__setattr__(self, "_view", original_view)

    @property
    def num_dof(self) -> int:
        return NUM_LEG_DOF  # RL 정책에게는 12개처럼 보임

    # ---- 읽기: 19 -> 12 ----
    def get_joint_positions(self, *a, **kw):
        return _trim_to_leg(self._view.get_joint_positions(*a, **kw))

    def get_joint_velocities(self, *a, **kw):
        return _trim_to_leg(self._view.get_joint_velocities(*a, **kw))

    # ---- 쓰기: 12 -> 19 ----
    def set_max_joint_velocities(self, v, *a, **kw):
        return self._view.set_max_joint_velocities(_pad_to_full(v, ARM_MAX_VEL), *a, **kw)

    def set_max_joint_accelerations(self, v, *a, **kw):
        return self._view.set_max_joint_accelerations(_pad_to_full(v, ARM_MAX_ACC), *a, **kw)

    def set_gains(self, stiffness, damping, *a, **kw):
        return self._view.set_gains(
            _pad_to_full(stiffness, ARM_STIFFNESS),
            _pad_to_full(damping,   ARM_DAMPING),
            *a, **kw
        )

    def set_max_efforts(self, v, *a, **kw):
        return self._view.set_max_efforts(_pad_to_full(v, ARM_MAX_EFFORT), *a, **kw)

    def set_joint_position_targets(self, v, *a, **kw):
        return self._view.set_joint_position_targets(_pad_to_full(v, ARM_DEFAULT_POS), *a, **kw)

    def set_joint_velocities(self, v, *a, **kw):
        return self._view.set_joint_velocities(
            _pad_to_full(v, np.zeros(NUM_ARM_DOF, dtype=np.float32)), *a, **kw
        )

    def set_joint_efforts(self, v, *a, **kw):
        return self._view.set_joint_efforts(
            _pad_to_full(v, np.zeros(NUM_ARM_DOF, dtype=np.float32)), *a, **kw
        )

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_view"), name)

    def __setattr__(self, name, value):
        if name == "_view":
            object.__setattr__(self, name, value)
        else:
            setattr(object.__getattribute__(self, "_view"), name, value)


# ================================================================================
# ArticulationControllerProxy
#   RL 정책이 apply_action()으로 내리는 12-DOF 명령을 19-DOF로 확장합니다.
# ================================================================================
class ArticulationControllerProxy:
    def __init__(self, original_controller):
        object.__setattr__(self, "_controller", original_controller)

    def apply_action(self, action, *a, **kw):
        if action is None:
            return
        zeros7 = np.zeros(NUM_ARM_DOF, dtype=np.float32)
        if getattr(action, "joint_positions",  None) is not None:
            action.joint_positions  = _pad_to_full(action.joint_positions,  ARM_DEFAULT_POS)
        if getattr(action, "joint_velocities", None) is not None:
            action.joint_velocities = _pad_to_full(action.joint_velocities, zeros7)
        if getattr(action, "joint_efforts",    None) is not None:
            action.joint_efforts    = _pad_to_full(action.joint_efforts,    zeros7)
        return self._controller.apply_action(action, *a, **kw)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_controller"), name)

    def __setattr__(self, name, value):
        if name == "_controller":
            object.__setattr__(self, name, value)
        else:
            setattr(object.__getattribute__(self, "_controller"), name, value)


# ================================================================================
# CustomArmSpot
#   SpotFlatTerrainPolicy(12-DOF RL)를 spot_with_arm(19-DOF) 위에서 실행합니다.
# ================================================================================
class CustomArmSpot(SpotFlatTerrainPolicy):
    OFFICIAL_ARM_USD = (
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/5.1/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd"
    )

    def __init__(self, prim_path: str, name: str, robot_id: int,
                 position=None, orientation=None):
        super().__init__(
            prim_path=prim_path,
            name=name,
            usd_path=self.OFFICIAL_ARM_USD,
            position=position,
            orientation=orientation,
        )
        self.robot_id            = robot_id
        self.realsense_prim_path = f"/World/Intel_RealSense_D455_Spot{robot_id}"
        self.realsense_asset     = None
        self._ros2_nodes         = []   # 종료 시 cleanup을 위해 보관
        self._render_products    = []   # 종료 시 cleanup을 위해 보관
        self._ros2_attached      = False
        self._initialized        = False

    # ------------------------------------------------------------------
    # initialize()
    # ------------------------------------------------------------------
    def initialize(self, physics_sim_view=None):
        if self._initialized:
            return

        # 1) robot ArticulationView 먼저 생성
        if hasattr(self, "robot") and self.robot is not None:
            if not getattr(self.robot, "_is_initialized", False):
                self.robot.initialize(physics_sim_view)

            # 2) _articulation_view -> 프록시 교체
            raw_view = getattr(self.robot, "_articulation_view", None)
            if raw_view is not None and not isinstance(raw_view, ArticulationViewProxy):
                self.robot._articulation_view = ArticulationViewProxy(raw_view)
                print(f"[Spot {self.robot_id}] robot._articulation_view -> Proxy 교체")

        # CustomArmSpot 자신의 _articulation_view도 교체
        raw_self = getattr(self, "_articulation_view", None)
        if raw_self is not None and not isinstance(raw_self, ArticulationViewProxy):
            self._articulation_view = ArticulationViewProxy(raw_self)

        # 3) 부모 initialize 실행 (내부에서 set_max_joint_velocities 등이 호출됨)
        super().initialize(physics_sim_view)

        # 4) ArticulationController -> 프록시 교체
        raw_ctrl = getattr(self, "_articulation_controller", None)
        if raw_ctrl is not None and not isinstance(raw_ctrl, ArticulationControllerProxy):
            self._articulation_controller = ArticulationControllerProxy(raw_ctrl)
            print(f"[Spot {self.robot_id}] _articulation_controller -> Proxy 교체")

        # 5) [Fix] arm0_* Warning 해결: 팔 관절 초기 위치 강제 설정
        self._set_arm_initial_pose()

        self._initialized = True
        print(f"[Spot {self.robot_id}] 초기화 완료")

    def _set_arm_initial_pose(self):
        """
        [Fix] config_loader가 arm0_* 기본값을 못 찾아 0으로 두는 문제를 해결합니다.
        팔 관절 인덱스(12~18)에 ARM_DEFAULT_POS를 직접 설정합니다.
        """
        try:
            real_view = getattr(self.robot, "_articulation_view", None)
            if real_view is None:
                return
            # 프록시 안의 원본 뷰를 꺼냄
            if isinstance(real_view, ArticulationViewProxy):
                real_view = object.__getattribute__(real_view, "_view")

            cur = real_view.get_joint_positions()
            if cur is None:
                return
            cur = np.asarray(cur, dtype=np.float32).flatten()  # (19,)
            cur[NUM_LEG_DOF:] = ARM_DEFAULT_POS                # 팔 7개 덮어쓰기
            real_view.set_joint_position_targets(cur.reshape(1, -1))
            print(f"[Spot {self.robot_id}] 팔 초기 자세 적용: {ARM_DEFAULT_POS.tolist()}")
        except Exception as e:
            carb.log_warn(f"[Spot {self.robot_id}] 팔 초기 자세 설정 실패 (무시 가능): {e}")

    # ------------------------------------------------------------------
    # forward()
    # ------------------------------------------------------------------
    def forward(self, step_size: float, command: np.ndarray):
        """RL 정책 한 스텝 실행. command = [vx, vy, wz]"""
        parent_cls = type(self).__mro__[1]  # SpotFlatTerrainPolicy
        if hasattr(parent_cls, "advance"):
            parent_cls.advance(self, step_size, command)
        elif hasattr(parent_cls, "forward"):
            parent_cls.forward(self, step_size, command)
        else:
            carb.log_warn(f"[Spot {self.robot_id}] forward/advance 메서드 없음")

    # ------------------------------------------------------------------
    # attach_realsense_ros2_streams()
    # ------------------------------------------------------------------
    def attach_realsense_ros2_streams(self):
        if self._ros2_attached:
            return

        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

        self.realsense_asset = SingleViewDepthSensorAsset(
            prim_path=self.realsense_prim_path,
            asset_path=asset_path
        )
        self.realsense_asset.initialize()

        try:
            import omni.replicator.core as rep
            from omni.isaac.core_nodes.scripts.utils import set_targets

            rp_attr = getattr(self.realsense_asset, "_render_product_path", None)
            if rp_attr is not None:
                render_product = rp_attr
            else:
                lens_prim = f"{self.realsense_prim_path}/RSD455/Camera_Pseudo_Depth"
                render_product = rep.create.render_product(lens_prim, resolution=(640, 480))

            self._render_products.append(render_product)
            topic_prefix = f"/spot{self.robot_id}"
            frame_id     = f"spot{self.robot_id}_realsense_frame"

            rgb_node = rep.utils.create_node(
                node_type_id="omni.isaac.ros2_bridge.ROS2PublishImage",
                attributes={
                    "inputs:topicName": f"{topic_prefix}/camera/image_raw",
                    "inputs:frameId":   frame_id,
                },
            )
            set_targets(node=rgb_node, attribute="inputs:renderProductPath",
                        targets=render_product)
            self._ros2_nodes.append(rgb_node)

            pc_node = rep.utils.create_node(
                node_type_id="omni.isaac.ros2_bridge.ROS2PublishPointCloud",
                attributes={
                    "inputs:topicName": f"{topic_prefix}/camera/point_cloud",
                    "inputs:frameId":   frame_id,
                },
            )
            set_targets(node=pc_node, attribute="inputs:renderProductPath",
                        targets=render_product)
            self._ros2_nodes.append(pc_node)

            self._ros2_attached = True
            print(f"[Spot {self.robot_id}] ROS2 토픽 활성화: {topic_prefix}/camera/...")

        except Exception as e:
            carb.log_error(f"[Spot {self.robot_id}] ROS2 센서 연결 실패: {e}")

    def detach_realsense_ros2_streams(self):
        """
        [Fix] 종료 전 Replicator 노드/렌더 프로덕트를 명시적으로 정리합니다.
        - "Could not find category Replicator:Annotators" Warning 억제
        - "There was an error running python" 크래시 방지
        """
        if not self._ros2_attached:
            return
        try:
            import omni.replicator.core as rep
            for node in self._ros2_nodes:
                try:
                    rep.utils.destroy_node(node)
                except Exception:
                    pass
            for rp in self._render_products:
                try:
                    rep.utils.destroy_render_product(rp)
                except Exception:
                    pass
            self._ros2_nodes.clear()
            self._render_products.clear()
            self._ros2_attached = False
            print(f"[Spot {self.robot_id}] ROS2 스트림 정리 완료")
        except Exception as e:
            carb.log_warn(f"[Spot {self.robot_id}] ROS2 정리 중 오류 (무시): {e}")

    def update_camera_pose(self):
        stage = World.instance().stage
        camera_prim = stage.GetPrimAtPath(self.realsense_prim_path)
        if not camera_prim.IsValid():
            return

        # SpotFlatTerrainPolicy는 get_world_pose()를 직접 갖지 않음
        # 위치 정보는 내부 self.robot (ArticulationRobot) 에 있음
        if not hasattr(self, "robot") or self.robot is None:
            return
        body_pos, body_quat = self.robot.get_world_pose()
        # Isaac Sim 쿼터니언: [w,x,y,z] -> scipy: [x,y,z,w]
        r = R.from_quat([body_quat[1], body_quat[2], body_quat[3], body_quat[0]])
        camera_pos = body_pos + r.apply(np.array([0.45, 0.0, 0.12]))

        xformable = UsdGeom.Xformable(camera_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(
            Gf.Vec3d(float(camera_pos[0]), float(camera_pos[1]), float(camera_pos[2]))
        )
        yaw_deg = float(np.degrees(r.as_euler("xyz")[2]))
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, yaw_deg))


# ================================================================================
# 월드 생성
# ================================================================================
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)
assets_root_path = get_assets_root_path()

# 바닥
prim = define_prim("/World/Ground", "Xform")
prim.GetReferences().AddReference(
    assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
)

# 조명
light_prim = define_prim("/World/DistantLight", "DistantLight")
light_prim.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

# 장애물 콘
obs = define_prim("/World/TestObstacleCone", "Cone")
obs_xform = UsdGeom.Xformable(obs)
obs_xform.ClearXformOpOrder()
obs_xform.AddTranslateOp().Set(Gf.Vec3d(2.5, 1.0, 0.15))
obs_xform.AddScaleOp().Set(Gf.Vec3f(0.25, 0.25, 0.45))
obs.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
    [Gf.Vec3f(1.0, 0.35, 0.0)]
)
UsdPhysics.CollisionAPI.Apply(obs)

# ================================================================================
# 로봇 2대 스폰
# ================================================================================
spot1 = CustomArmSpot(
    prim_path="/World/Spot_1", name="Spot1", robot_id=1,
    position=np.array([0.0, 0.0, 0.65])
)
spot2 = CustomArmSpot(
    prim_path="/World/Spot_2", name="Spot2", robot_id=2,
    position=np.array([0.0, 2.0, 0.65])
)

import omni.kit.app
print("USD 에셋 및 신경망 가중치 로드 중...")
for _ in range(250):
    omni.kit.app.get_app().update()
time.sleep(3.0)

# 순서 중요: reset 먼저 -> 센서 연결
my_world.reset()
spot1.attach_realsense_ros2_streams()
spot2.attach_realsense_ros2_streams()

# ================================================================================
# 경로 추종 파라미터
# ================================================================================
Kp                  = 1.6
look_ahead_distance = 0.55
target_speed        = 0.55

spot1_path = [
    np.array([4.0,  0.0]),
    np.array([4.0, -1.5]),
    np.array([0.0, -1.5]),
    np.array([0.0,  0.0]),
]
spot1_idx, spot1_command = 0, np.zeros(3)

spot2_path = [
    np.array([4.0, 2.0]),
    np.array([4.0, 3.5]),
    np.array([0.0, 3.5]),
    np.array([0.0, 2.0]),
]
spot2_idx, spot2_command = 0, np.zeros(3)

# ================================================================================
# physics 콜백
# ================================================================================
first_step   = True
reset_needed = False


def on_physics_step(step_size: float) -> None:
    global first_step, reset_needed

    if first_step:
        spot1.initialize()
        spot2.initialize()
        first_step = False
        return

    if reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step   = True
        return

    spot1.update_camera_pose()
    spot2.update_camera_pose()
    spot1.forward(step_size, spot1_command)
    spot2.forward(step_size, spot2_command)


my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)


def compute_command(robot, path: list, idx: int, command: np.ndarray) -> int:
    # SpotFlatTerrainPolicy는 get_world_pose()가 없음 → 내부 robot 객체 사용
    pos, quat = robot.robot.get_world_pose()
    yaw = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz")[2]
    tgt = path[idx]
    if np.linalg.norm(pos[:2] - tgt) < look_ahead_distance:
        idx = (idx + 1) % len(path)
        tgt = path[idx]
        print(f"[{robot.name}] 웨이포인트 -> 인덱스 {idx} {tgt}")
    err_yaw    = np.arctan2(tgt[1] - pos[1], tgt[0] - pos[0]) - yaw
    err_yaw    = (err_yaw + np.pi) % (2 * np.pi) - np.pi
    command[:] = [target_speed, 0.0, float(np.clip(Kp * err_yaw, -0.7, 0.7))]
    return idx


# ================================================================================
# 메인 루프
# ================================================================================
try:
    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_playing():
            spot1_idx = compute_command(spot1, spot1_path, spot1_idx, spot1_command)
            spot2_idx = compute_command(spot2, spot2_path, spot2_idx, spot2_command)
finally:
    # [Fix] 종료 시 Replicator 노드 먼저 정리 -> 크래시 방지
    print("종료 중: ROS2 스트림 정리...")
    spot1.detach_realsense_ros2_streams()
    spot2.detach_realsense_ros2_streams()
    my_world.clear()
    simulation_app.close()
    print("시뮬레이션 종료.")