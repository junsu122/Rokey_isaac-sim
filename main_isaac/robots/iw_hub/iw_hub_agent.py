"""
main_isaac/robots/iw_hub/iw_hub_agent.py
==========================================
IW Hub 에이전트 — spawn + ActionGraph 설정 전담.
이동 제어는 iw_hub_movement ROS2 패키지가 담당한다.

동작 방식 (우선순위):
  1차 시도: iw_hub_v2.usda 의 reference ActionGraph 가 OmniGraph 에
            인식됐으면 그대로 재사용하고 로봇별 토픽 이름만 덮어쓴다.
            (fabricCacheBacking = "StageWithoutHistory" 로 설정된 경우)

  fallback: reference ActionGraph 가 인식되지 않으면 현재 stage 편집
            레이어에 ActionGraph 를 og.Controller 로 직접 생성한다.
            (이전의 "Shared" backing 방식에서의 workaround)

ROS2 토픽 (robot_name = iw_hub_01 / iw_hub_02):
  sub  /{robot_name}/cmd_vel        geometry_msgs/Twist
  sub  /{robot_name}/lift_cmd       sensor_msgs/JointState
  pub  /{robot_name}/odom           nav_msgs/Odometry
  pub  /{robot_name}/tf             tf2_msgs/TFMessage
"""
import math
import numpy as np
import carb
import omni.usd
import omni.graph.core as og
from pxr import Usd, UsdGeom, Sdf, Gf

import robot_config as C
from ..base_robot import BaseRobotAgent

_SENSORS_REL = "iw_hub_sensors"

try:
    from isaacsim.core.prims import SingleArticulation
except Exception:
    SingleArticulation = None


def _set_rel_targets(stage, prim_path: str, rel_name: str, targets: list):
    rel = stage.GetPrimAtPath(prim_path).GetRelationship(rel_name)
    if rel:
        rel.SetTargets([Sdf.Path(t) for t in targets])
    else:
        carb.log_warn(f"[IwHubAgent] rel not found: {prim_path}.{rel_name}")


def _configure_topics_usd(stage, graph_path: str, robot_name: str):
    """
    world.reset() 이전, USD 레이어에 직접 토픽 이름을 기록한다.
    OmniGraph가 reset() 내부 step에서 Publisher를 생성할 때 이 값을 읽으므로
    타이밍 문제 없이 올바른 토픽으로 초기화된다.
    """
    def _set(node_rel, attr_name, val):
        prim = stage.GetPrimAtPath(f"{graph_path}/{node_rel}")
        if not prim.IsValid():
            carb.log_warn(f"[IwHubAgent] prim 없음: {graph_path}/{node_rel}")
            return
        attr = prim.GetAttribute(attr_name)
        if attr:
            attr.Set(val)
        else:
            prim.CreateAttribute(attr_name, type(val))
            prim.GetAttribute(attr_name).Set(val)

    _set("ros2_subscribe_twist",        "inputs:topicName",     f"/{robot_name}/cmd_vel")
    _set("ros2_subscribe_joint_state",  "inputs:topicName",     f"/{robot_name}/lift_cmd")
    _set("ros2_publish_odometry",       "inputs:topicName",     f"/{robot_name}/odom")
    _set("ros2_publish_odometry",       "inputs:chassisFrameId",f"{robot_name}/base_link")
    _set("ros2_publish_odometry",       "inputs:odomFrameId",   f"{robot_name}/odom")
    _set("ros2_publish_transform_tree", "inputs:topicName",     f"/{robot_name}/tf")
    carb.log_info(f"[IwHubAgent] {robot_name} USD 레이어 토픽 설정 완료")


def _build_action_graph(robot_root: str, robot_name: str):
    """
    현재 stage 편집 레이어에 ActionGraph 를 직접 생성한다.
    reference ActionGraph 가 OmniGraph 에 인식되지 않을 때의 fallback.
    """
    sensor_prim = f"{robot_root}/{_SENSORS_REL}"
    graph_path  = f"{robot_root}/ActionGraph"

    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path":     graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("on_playback_tick",            "omni.graph.action.OnPlaybackTick"),
                ("ros2_context",                "isaacsim.ros2.bridge.ROS2Context"),
                ("isaac_read_simulation_time",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ros2_subscribe_twist",        "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("break_3_vector",              "omni.graph.nodes.BreakVector3"),
                ("break_3_vector_01",           "omni.graph.nodes.BreakVector3"),
                ("differential_controller",     "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("articulation_controller",     "isaacsim.core.nodes.IsaacArticulationController"),
                ("ros2_subscribe_joint_state",  "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("articulation_controller_01",  "isaacsim.core.nodes.IsaacArticulationController"),
                ("isaac_compute_odometry_node", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("ros2_publish_odometry",       "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("on_playback_tick.outputs:tick",                  "ros2_subscribe_twist.inputs:execIn"),
                ("ros2_context.outputs:context",                   "ros2_subscribe_twist.inputs:context"),
                ("ros2_subscribe_twist.outputs:linearVelocity",    "break_3_vector.inputs:tuple"),
                ("ros2_subscribe_twist.outputs:angularVelocity",   "break_3_vector_01.inputs:tuple"),
                ("on_playback_tick.outputs:tick",                  "differential_controller.inputs:execIn"),
                ("break_3_vector.outputs:x",                       "differential_controller.inputs:linearVelocity"),
                ("break_3_vector_01.outputs:z",                    "differential_controller.inputs:angularVelocity"),
                ("on_playback_tick.outputs:tick",                  "articulation_controller.inputs:execIn"),
                ("differential_controller.outputs:velocityCommand","articulation_controller.inputs:velocityCommand"),
                ("on_playback_tick.outputs:tick",                  "ros2_subscribe_joint_state.inputs:execIn"),
                ("ros2_context.outputs:context",                   "ros2_subscribe_joint_state.inputs:context"),
                ("on_playback_tick.outputs:tick",                  "articulation_controller_01.inputs:execIn"),
                ("ros2_subscribe_joint_state.outputs:jointNames",      "articulation_controller_01.inputs:jointNames"),
                ("ros2_subscribe_joint_state.outputs:positionCommand", "articulation_controller_01.inputs:positionCommand"),
                ("on_playback_tick.outputs:tick",                  "isaac_compute_odometry_node.inputs:execIn"),
                ("isaac_compute_odometry_node.outputs:execOut",    "ros2_publish_odometry.inputs:execIn"),
                ("ros2_context.outputs:context",                   "ros2_publish_odometry.inputs:context"),
                ("isaac_compute_odometry_node.outputs:angularVelocity","ros2_publish_odometry.inputs:angularVelocity"),
                ("isaac_compute_odometry_node.outputs:linearVelocity", "ros2_publish_odometry.inputs:linearVelocity"),
                ("isaac_compute_odometry_node.outputs:orientation",    "ros2_publish_odometry.inputs:orientation"),
                ("isaac_compute_odometry_node.outputs:position",       "ros2_publish_odometry.inputs:position"),
                ("isaac_read_simulation_time.outputs:simulationTime",  "ros2_publish_odometry.inputs:timeStamp"),
                ("on_playback_tick.outputs:tick",                  "ros2_publish_transform_tree.inputs:execIn"),
                ("ros2_context.outputs:context",                   "ros2_publish_transform_tree.inputs:context"),
                ("isaac_read_simulation_time.outputs:simulationTime",  "ros2_publish_transform_tree.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ros2_subscribe_twist.inputs:topicName",        f"/{robot_name}/cmd_vel"),
                ("ros2_subscribe_joint_state.inputs:topicName",  f"/{robot_name}/lift_cmd"),
                ("differential_controller.inputs:maxLinearSpeed", 1.8),
                ("differential_controller.inputs:wheelRadius",    0.08),
                ("differential_controller.inputs:wheelDistance",  0.58),
                ("articulation_controller.inputs:jointNames",     ["left_wheel_joint", "right_wheel_joint"]),
                ("ros2_publish_odometry.inputs:topicName",        f"/{robot_name}/odom"),
                ("ros2_publish_odometry.inputs:chassisFrameId",   f"{robot_name}/base_link"),
                ("ros2_publish_odometry.inputs:odomFrameId",      f"{robot_name}/odom"),
                ("ros2_publish_transform_tree.inputs:topicName",  f"/{robot_name}/tf"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    _set_rel_targets(stage, f"{graph_path}/articulation_controller",
                     "inputs:targetPrim",  [sensor_prim])
    _set_rel_targets(stage, f"{graph_path}/articulation_controller_01",
                     "inputs:targetPrim",  [sensor_prim])
    _set_rel_targets(stage, f"{graph_path}/isaac_compute_odometry_node",
                     "inputs:chassisPrim", [sensor_prim])
    _set_rel_targets(stage, f"{graph_path}/ros2_publish_transform_tree",
                     "inputs:targetPrims", [sensor_prim])

    carb.log_info(f"[IwHubAgent] {robot_name} ActionGraph(fallback) 생성 완료 → {graph_path}")
    return graph


class IwHubAgent(BaseRobotAgent):
    """IW Hub 스폰 + ActionGraph 설정 에이전트."""

    _ROBOTS_PATH = "/World/Robots"

    def setup(self) -> None:
        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath(self._ROBOTS_PATH):
            UsdGeom.Xform.Define(stage, self._ROBOTS_PATH)

        prim_path = f"{self._ROBOTS_PATH}/{self.name}"
        self._prim_path = prim_path
        self._articulation_path = f"{prim_path}/{_SENSORS_REL}"
        self._articulation = None

        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(C.IW_HUB_USD)

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, self.spawn_yaw))
        xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

        stage.Load(prim_path)

        # world.reset() 이전에 USD 레이어에 토픽 이름 기록
        # → reset() 내부 step에서 ROS2 Publisher가 올바른 토픽으로 생성됨
        graph_path = f"{prim_path}/{_SENSORS_REL}/ActionGraph"
        _configure_topics_usd(stage, graph_path, self.name)

        if SingleArticulation is not None:
            try:
                self._articulation = self.world.scene.add(
                    SingleArticulation(
                        prim_path=self._articulation_path,
                        name=f"{self.name}_articulation",
                    )
                )
            except Exception as e:
                carb.log_warn(f"[IwHubAgent] {self.name} articulation wrapper 생성 실패: {e}")

        carb.log_info(f"[IwHubAgent] {self.name} 스폰 완료  "
                      f"xyz={self.spawn_xyz}  yaw={self.spawn_yaw}°")

    def post_reset(self) -> None:
        stage = omni.usd.get_context().get_stage()

        stage.Load(self._prim_path)

        if self._articulation is not None:
            try:
                self._articulation.initialize()
                carb.log_info(f"[IwHubAgent] {self.name} articulation pose source ready")
            except Exception as e:
                carb.log_warn(f"[IwHubAgent] {self.name} articulation initialize 실패: {e}")
                self._articulation = None
        else:
            carb.log_warn(f"[IwHubAgent] {self.name} articulation wrapper 없음 → USD transform fallback 사용")

        ref_graph_path = f"{self._prim_path}/{_SENSORS_REL}/ActionGraph"

        graph = og.get_graph_by_path(ref_graph_path)
        if graph is not None:
            carb.log_info(f"[IwHubAgent] {self.name} reference ActionGraph 인식됨 "
                          f"(토픽은 setup()에서 USD 레이어에 기록 완료)")
            return

        # fallback: reference ActionGraph 미인식 → 편집 레이어에 직접 생성
        carb.log_warn(f"[IwHubAgent] {self.name} reference ActionGraph 미인식 "
                      f"→ fallback(편집 레이어 직접 생성)")

        ref_graph = stage.GetPrimAtPath(ref_graph_path)
        if ref_graph and ref_graph.IsValid():
            ref_graph.SetActive(False)

        _build_action_graph(self._prim_path, self.name)

    def on_physics_step(self, dt: float) -> None:
        pass

    # ── 미니맵 인터페이스 ─────────────────────────────────────────────

    @property
    def mission_state(self) -> int:
        """ROS2 제어 로봇 — 내부 상태 머신 없음. 미니맵 표시용 기본값."""
        return 0

    def get_world_xy(self) -> tuple:
        """(x, y, heading_rad) 반환. 미니맵용."""
        if self._articulation is not None:
            try:
                pos, quat = self._articulation.get_world_pose()
                q = np.array(quat, dtype=np.float64)
                # Isaac core pose quaternions are wxyz.
                w, x, y, z = q
                hdg = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                )
                return (float(pos[0]), float(pos[1]), float(hdg))
            except Exception:
                pass

        try:
            stage = omni.usd.get_context().get_stage()
            cache = UsdGeom.XformCache()
            candidates = [
                stage.GetPrimAtPath(f"{self._prim_path}/{_SENSORS_REL}"),
                stage.GetPrimAtPath(f"{self._prim_path}/{_SENSORS_REL}/chassis"),
            ]
            root = stage.GetPrimAtPath(self._prim_path)
            if root.IsValid():
                sensors = None
                chassis = None
                for p in Usd.PrimRange(root):
                    name = p.GetName()
                    if name == _SENSORS_REL:
                        sensors = p
                    elif name == "chassis":
                        chassis = p
                candidates.extend([sensors, chassis, root])

            for prim in candidates:
                if prim is None or not prim.IsValid():
                    continue
                mat = cache.GetLocalToWorldTransform(prim)
                tr = mat.ExtractTranslation()
                m = np.array(mat, dtype=np.float64).T
                hdg = math.atan2(float(m[1, 0]), float(m[0, 0]))
                return (float(tr[0]), float(tr[1]), hdg)
        except Exception:
            pass
        return (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]), 0.0)
