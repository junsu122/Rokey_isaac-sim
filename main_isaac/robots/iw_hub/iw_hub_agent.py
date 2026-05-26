"""
main_isaac/robots/iw_hub/iw_hub_agent.py
==========================================
IW Hub 에이전트 — spawn + ActionGraph 설정 + 자율 미션 FSM.

미션 FSM (mission_state):
  0 WAITING     M0609 complete 신호 complete_threshold 회 수신 대기
  1 LIFTING     리프트 업 (LIFT_STEPS FSM 틱 대기)
  2 GOTO_SECTION 섹션 목표 위치로 이동
  3 LOWERING    리프트 다운 (LIFT_STEPS FSM 틱 대기)
  4 GOTO_HOME   홈 위치로 복귀

ROS2 토픽 (robot_name = iw_hub_01 / iw_hub_02 / iw_hub_03):
  sub  /{robot_name}/cmd_vel        geometry_msgs/Twist
  sub  /{robot_name}/lift_cmd       sensor_msgs/JointState
  pub  /{robot_name}/odom           nav_msgs/Odometry
  pub  /{robot_name}/tf             tf2_msgs/TFMessage
"""
import math
import threading
import carb
import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, Sdf, Gf

import robot_config as C
from ..base_robot import BaseRobotAgent

# ── ROS2 (선택 사항) ──────────────────────────────────────────────────
try:
    import sys as _sys
    for _p in [
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
    ]:
        if _p not in _sys.path:
            _sys.path.insert(0, _p)
    import rclpy
    _ROS2_AVAILABLE = True
    print("[IwHub] rclpy import 성공")
except Exception as _e:
    print(f"[IwHub] rclpy import 실패: {_e}")
    _ROS2_AVAILABLE = False

_ros2_node = None


def _get_ros2_node():
    global _ros2_node
    if not _ROS2_AVAILABLE:
        return None
    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        pass
    if _ros2_node is None:
        _ros2_node = rclpy.create_node("isaac_iw_hub_node")
        print("[IwHub] ROS2 노드 생성: isaac_iw_hub_node")
    return _ros2_node

_SENSORS_REL  = "iw_hub_sensors"
_CORRIDOR_XY  = (-6.0, 1.5)   # 픽업 복귀 후 섹션 진입 중간점
_SPOT_WAIT_DIST = 2.0


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
    """IW Hub 스폰 + ActionGraph 설정 + 자율 미션 FSM."""

    _ROBOTS_PATH = "/World/Robots"

    # ── 미션 상수 ─────────────────────────────────────────────────────
    LIFT_UP         = 0.30    # 리프트 올림 위치 [m]
    LIFT_DOWN       = 0.0     # 리프트 내림 위치 [m]
    LIFT_STEPS      = 200     # 리프트 대기 FSM 틱 (200 * PUB_EVERY = 2000 physics steps ≈ 4s) — 천천히 올려 포드 넘어짐 방지
    NAV_TOL         = 0.20    # 도착 허용 오차 [m]
    MAX_V           = 0.8     # 최대 직진 속도 [m/s]
    MAX_W           = 1.5     # 최대 회전 속도 [rad/s]
    KP_W            = 2.5     # 회전 P게인
    PUB_EVERY       = 10      # cmd_vel 발행 주기 (physics step 수)
    COMPLETE_NEEDED = 3       # 출발 조건: complete 신호 횟수
    DOCK_TOL        = 0.08    # pod place/pick 시 정밀 정렬 허용 오차 [m]
    DOCK_KP         = 0.65
    DOCK_KI         = 0.015
    DOCK_KD         = 0.18
    DOCK_MAX_V      = 0.22

    # ── setup ────────────────────────────────────────────────────────
    def setup(self) -> None:
        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath(self._ROBOTS_PATH):
            UsdGeom.Xform.Define(stage, self._ROBOTS_PATH)

        prim_path = f"{self._ROBOTS_PATH}/{self.name}"
        self._prim_path = prim_path

        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(C.IW_HUB_USD)

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, self.spawn_yaw))
        xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

        stage.Load(prim_path)

        graph_path = f"{prim_path}/{_SENSORS_REL}/ActionGraph"
        _configure_topics_usd(stage, graph_path, self.name)

        carb.log_info(f"[IwHubAgent] {self.name} 스폰 완료  "
                      f"xyz={self.spawn_xyz}  yaw={self.spawn_yaw}°")

        # ── 공통 상태 초기화 ─────────────────────────────────────────
        self._complete_count  = 0
        self._complete_lock   = threading.Lock()
        self._section_name    = self.cfg.get("section", "A")
        self._complete_topic  = self.cfg.get("complete_topic", "/m0609_A/work")
        self._complete_signal = self.cfg.get("complete_signal", "A_complete")
        self._complete_needed = int(self.cfg.get("complete_threshold", self.COMPLETE_NEEDED))
        self._drop_idx        = 0
        self._fsm_step        = 0
        self._physics_step    = 0
        self._home_xy         = (float(self.spawn_xyz[0]), float(self.spawn_xyz[1]))
        self._nav_target      = None
        self._ros2_cmd_pub    = None
        self._ros2_lift_pub   = None
        self._path_wps        = []
        self._path_wp_idx     = 0

        # ── 직접구동 핸들 (post_reset → _init_direct_drive 에서 채워짐) ──
        self._articulation  = None
        self._lw_idx        = None
        self._rw_idx        = None
        self._lift_idx      = None
        self._lw_joint_path = None
        self._rw_joint_path = None

        # ── odom 피드백 (ROS2 구독, move_to_point.py 와 동일 방식) ──────
        self._odom_x     = float(self.spawn_xyz[0])
        self._odom_y     = float(self.spawn_xyz[1])
        self._odom_yaw   = 0.0
        self._odom_fresh = False

        # ── 모드 분기 ────────────────────────────────────────────────
        self._mode = self.cfg.get("mode", "standard")

        if self._mode == "pickup":
            # pickup 모드: 포드스택 → 리프트업 → 중간점 → 슬롯 배달
            px, py = self.cfg.get("pickup_xyz", (0.0, 0.0))[:2]
            self._pickup_xy      = (float(px), float(py))
            self._pickup_state   = "WAITING"
            self._turn_tgt_yaw   = None
            self._delivery_slots = self._build_delivery_slots()
            self._delivery_idx   = 0
            self._backout_x      = float(_CORRIDOR_XY[0])
            self._dock_target    = None
            self._dock_pid       = {"axis": None, "err_i": 0.0, "prev_err": 0.0}
            self.mission_state   = -1   # pickup 모드에서는 표준 FSM 미사용
        else:
            self.mission_state   = 0    # 0=WAITING

    # ── post_reset ───────────────────────────────────────────────────
    def post_reset(self) -> None:
        stage = omni.usd.get_context().get_stage()
        stage.Load(self._prim_path)

        # reference ActionGraph 의 topicName 이 "/cmd_vel" (USD 하드코딩) 이므로
        # 항상 비활성화하고, 로봇별 topic을 가진 fallback ActionGraph 를 사용한다.
        ref_graph_path = f"{self._prim_path}/{_SENSORS_REL}/ActionGraph"
        ref_prim = stage.GetPrimAtPath(ref_graph_path)
        if ref_prim and ref_prim.IsValid():
            ref_prim.SetActive(False)
            print(f"[{self.name}] reference ActionGraph 비활성화 (USD topic=/cmd_vel 고정)")

        fallback_path = f"{self._prim_path}/ActionGraph"
        if og.get_graph_by_path(fallback_path) is None:
            _build_action_graph(self._prim_path, self.name)
            print(f"[{self.name}] fallback ActionGraph 생성: topic=/{self.name}/cmd_vel")
        else:
            print(f"[{self.name}] fallback ActionGraph 이미 존재")

        self._setup_ros2()
        self._init_direct_drive()

    # ── 직접구동 초기화 ───────────────────────────────────────────────
    def _init_direct_drive(self) -> None:
        """바퀴·리프트 조인트 직접 제어 초기화."""
        from pxr import Usd, UsdPhysics as _UsdPhysics
        art_path = f"{self._prim_path}/{_SENSORS_REL}"
        stage = omni.usd.get_context().get_stage()

        # ── Articulation 객체 초기화 ──────────────────────────────────
        try:
            try:
                from isaacsim.core.prims import SingleArticulation
                art = SingleArticulation(prim_path=art_path, name=f"{self.name}_art")
            except (ImportError, AttributeError):
                from omni.isaac.core.articulations import Articulation
                art = Articulation(prim_path=art_path, name=f"{self.name}_art")
            art.initialize()
            self._articulation = art
            dof = list(art.dof_names)
            self._lw_idx   = next((i for i, n in enumerate(dof) if "left_wheel"  in n), None)
            self._rw_idx   = next((i for i, n in enumerate(dof) if "right_wheel" in n), None)
            self._lift_idx = next((i for i, n in enumerate(dof) if "lift"        in n), None)
            print(f"[{self.name}] 직접구동 초기화 완료: dof={dof} "
                  f"lw={self._lw_idx} rw={self._rw_idx} lift={self._lift_idx}")
        except Exception as e:
            print(f"[{self.name}] 직접구동 초기화 실패 (ROS2/ActionGraph 유지): {e}")

        # ── 조인트 USD 경로 탐색 (USD DriveAPI 폴백용) ──────────────────
        sensors_prim = stage.GetPrimAtPath(art_path)
        if sensors_prim.IsValid():
            for prim in Usd.PrimRange(sensors_prim):
                if "Joint" not in prim.GetTypeName():
                    continue
                n = prim.GetName().lower()
                if "left" in n and "wheel" in n:
                    self._lw_joint_path = str(prim.GetPath())
                elif "right" in n and "wheel" in n:
                    self._rw_joint_path = str(prim.GetPath())
            print(f"[{self.name}] 조인트 경로: lw={self._lw_joint_path} rw={self._rw_joint_path}")

    # ── ROS2 설정 ─────────────────────────────────────────────────────
    def _setup_ros2(self) -> None:
        node = _get_ros2_node()
        if node is None:
            print(f"[{self.name}] ROS2 없음 — 자율 미션 비활성화")
            return
        try:
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import JointState
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

            _isaac_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
            )

            self._ros2_cmd_pub  = node.create_publisher(Twist,      f"/{self.name}/cmd_vel",  10)
            self._ros2_lift_pub = node.create_publisher(JointState, f"/{self.name}/lift_cmd", 10)

            # odom 구독 — move_to_point.py 와 동일 방식 (BEST_EFFORT QoS)
            def _on_odom(msg):
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                self._odom_x   = float(p.x)
                self._odom_y   = float(p.y)
                siny = 2.0 * (o.w * o.z + o.x * o.y)
                cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                self._odom_yaw   = math.atan2(siny, cosy)
                self._odom_fresh = True
            node.create_subscription(Odometry, f"/{self.name}/odom", _on_odom, _isaac_qos)

            complete_signal = self._complete_signal
            def _on_complete(msg):
                if msg.data == complete_signal:
                    with self._complete_lock:
                        self._complete_count += 1
                    print(f"[{self.name}] complete 수신 "
                          f"#{self._complete_count}/{self._complete_needed}")
            node.create_subscription(String, self._complete_topic, _on_complete, 10)
            print(f"[{self.name}] ROS2 설정 완료  odom=/{self.name}/odom  완료신호={self._complete_topic}")
        except Exception as e:
            print(f"[{self.name}] ROS2 설정 실패: {e}")

    # ── 위치 읽기 ─────────────────────────────────────────────────────
    def _get_xy_hdg(self) -> tuple:
        """(x, y, heading_rad) — odom 구독 우선 (move_to_point.py 와 동일), XformCache 폴백."""
        # 1순위: odom 피드백 (ActionGraph가 publish, 실시간 갱신됨)
        if self._odom_fresh:
            # odom 은 로봇 기준 로컬 프레임 (시작점 = 0,0).
            # 내비게이션 타겟은 월드 좌표이므로 spawn 오프셋을 더해 월드 좌표로 변환한다.
            wx = float(self.spawn_xyz[0]) + self._odom_x
            wy = float(self.spawn_xyz[1]) + self._odom_y
            self._hdg_debug = getattr(self, "_hdg_debug", 0) + 1
            if self._hdg_debug % 100 == 1:
                print(f"[{self.name}] odom(world) pos=({wx:.2f},{wy:.2f}) "
                      f"yaw={math.degrees(self._odom_yaw):.1f}°")
            return wx, wy, self._odom_yaw
        # 2순위: USD XformCache (odom 아직 미수신 시 초기 추정)
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(f"{self._prim_path}/{_SENSORS_REL}")
            if not prim.IsValid():
                prim = stage.GetPrimAtPath(self._prim_path)
            if not prim.IsValid():
                return float(self.spawn_xyz[0]), float(self.spawn_xyz[1]), 0.0
            cache = UsdGeom.XformCache()
            mat   = cache.GetLocalToWorldTransform(prim)
            tr    = mat.ExtractTranslation()
            rot   = mat.ExtractRotationMatrix()
            yaw   = math.atan2(float(rot[1][0]), float(rot[0][0]))
            return float(tr[0]), float(tr[1]), yaw
        except Exception:
            return float(self.spawn_xyz[0]), float(self.spawn_xyz[1]), 0.0

    def get_world_xy(self) -> tuple:
        """(x, y, heading_rad) — 미니맵용."""
        return self._get_xy_hdg()

    def _get_prim_xy(self, prim_path: str) -> tuple | None:
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None
            cache = UsdGeom.XformCache()
            tr = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
            return float(tr[0]), float(tr[1])
        except Exception:
            return None

    def _spot_too_close(self) -> bool:
        """Pause IW Hub commands while a Spot is close to the hub body."""
        try:
            hx, hy, _ = self._get_xy_hdg()
            for name in ("Spot_01", "Spot_02"):
                pos = self._get_prim_xy(f"/World/{name}")
                if pos is None:
                    continue
                sx, sy = pos
                if math.hypot(sx - hx, sy - hy) < _SPOT_WAIT_DIST:
                    self._spot_wait_dbg = getattr(self, "_spot_wait_dbg", 0) + 1
                    if self._spot_wait_dbg % 100 == 1:
                        print(f"[{self.name}] Spot nearby, waiting  "
                              f"spot={name} pos=({sx:.2f},{sy:.2f})")
                    return True
        except Exception:
            pass
        return False

    # ── 발행 헬퍼 ─────────────────────────────────────────────────────
    def _publish_cmd_vel(self, lv: float, av: float) -> None:
        """ROS2 → fallback ActionGraph 경유 바퀴 제어 (검증된 유일한 경로)."""
        self._cmd_call_count = getattr(self, "_cmd_call_count", 0) + 1
        dbg = self._cmd_call_count <= 5 or self._cmd_call_count % 200 == 0

        if self._ros2_cmd_pub is None:
            if dbg:
                print(f"[{self.name}] !! cmd_vel #{self._cmd_call_count} ROS2 pub 없음 — 이동 불가!")
            return
        if (abs(lv) > 1e-4 or abs(av) > 1e-4) and self._spot_too_close():
            lv, av = 0.0, 0.0
        try:
            from geometry_msgs.msg import Twist
            msg = Twist()
            msg.linear.x  = float(lv)
            msg.angular.z = float(av)
            self._ros2_cmd_pub.publish(msg)
            if dbg:
                print(f"[{self.name}] cmd_vel #{self._cmd_call_count} "
                      f"lv={lv:.3f} av={av:.3f} published ✓")
        except Exception as e:
            if dbg:
                print(f"[{self.name}] cmd_vel ROS2 publish 실패: {e}")

    def _publish_lift(self, pos: float) -> None:
        """ROS2 → ActionGraph 경유 리프트 위치 제어."""
        if self._ros2_lift_pub is None:
            return
        try:
            from sensor_msgs.msg import JointState
            msg = JointState()
            msg.name     = ["lift_joint"]
            msg.position = [float(pos)]
            self._ros2_lift_pub.publish(msg)
        except Exception:
            pass

    # ── 네비게이션 ─────────────────────────────────────────────────────
    def _plan_path_to(self, tx: float, ty: float) -> None:
        """A* 경로 계획. 결과를 _path_wps 에 저장."""
        try:
            from path_planner import get_planner
            planner = get_planner()
            if planner is not None:
                x, y, _ = self._get_xy_hdg()
                self._path_wps = planner.plan((x, y), (tx, ty),
                                              agent_name=self.name)
            else:
                self._path_wps = [(tx, ty)]
        except Exception as e:
            carb.log_warn(f"[{self.name}] A* 경로 계획 실패: {e}")
            self._path_wps = [(tx, ty)]
        self._path_wp_idx = 0
        print(f"[{self.name}] A* 경로 계획 — {len(self._path_wps)}개 웨이포인트")

    @staticmethod
    def _angle_err(target: float, current: float) -> float:
        """각도 오차를 [-π, π] 로 정규화."""
        return (target - current + math.pi) % (2 * math.pi) - math.pi

    def _reset_dock_pid(self, axis: str = None) -> None:
        self._dock_pid = {"axis": axis, "err_i": 0.0, "prev_err": 0.0}

    def _dock_axis_pid(self, target: float, axis: str,
                       align_far: bool = False) -> bool:
        """PID-controlled close approach in minimap/odom coordinates."""
        x, y, hdg = self._get_xy_hdg()
        pos = x if axis == "x" else y
        err = target - pos
        if abs(err) <= self.DOCK_TOL:
            self._publish_cmd_vel(0.0, 0.0)
            self._reset_dock_pid(axis)
            return True

        pid = getattr(self, "_dock_pid", {"axis": None, "err_i": 0.0, "prev_err": 0.0})
        if pid.get("axis") != axis:
            pid = {"axis": axis, "err_i": 0.0, "prev_err": err}

        dt = self.PUB_EVERY * 0.002  # PHYSICS_DT is 1/500 in robot_config.py
        pid["err_i"] = max(-0.35, min(0.35, pid["err_i"] + err * dt))
        derr = (err - pid["prev_err"]) / max(dt, 1e-6)
        pid["prev_err"] = err
        self._dock_pid = pid

        odom_v = (self.DOCK_KP * err +
                  self.DOCK_KI * pid["err_i"] +
                  self.DOCK_KD * derr)
        odom_v = max(-self.DOCK_MAX_V, min(self.DOCK_MAX_V, odom_v))

        axis_component = math.cos(hdg) if axis == "x" else math.sin(hdg)

        # Align only while still far from the slot/inside the aisle. Near target, never spin.
        if align_far and abs(err) > 0.45 and abs(axis_component) < 0.85:
            if axis == "x":
                target_yaw = 0.0 if err >= 0.0 else math.pi
            else:
                target_yaw = math.pi / 2.0 if err >= 0.0 else -math.pi / 2.0
            yaw_err = self._angle_err(target_yaw, hdg)
            av = max(-0.6, min(0.6, self.KP_W * yaw_err))
            self._publish_cmd_vel(0.0, av)
            self._reset_dock_pid(axis)
            return False

        # During close docking, never spin. Use current heading projection only.
        if abs(axis_component) < 0.2:
            lv = 0.0
        else:
            lv = odom_v / axis_component
            lv = max(-self.DOCK_MAX_V, min(self.DOCK_MAX_V, lv))
        self._publish_cmd_vel(float(lv), 0.0)
        return False

    def _nav_along_path(self) -> bool:
        """_path_wps 를 순서대로 따라 이동. 최종 목표 도착 시 True."""
        if not self._path_wps:
            return True
        tx, ty = self._path_wps[self._path_wp_idx]
        x, y, hdg = self._get_xy_hdg()
        dist = math.hypot(tx - x, ty - y)
        if dist <= self.NAV_TOL:
            if self._path_wp_idx < len(self._path_wps) - 1:
                self._path_wp_idx += 1
                return False
            self._publish_cmd_vel(0.0, 0.0)
            return True
        err_hdg = self._angle_err(math.atan2(ty - y, tx - x), hdg)
        lv = 0.0 if abs(err_hdg) > 0.5 else self.MAX_V * min(1.0, dist / 1.5)
        av = max(-self.MAX_W, min(self.MAX_W, self.KP_W * err_hdg))
        self._publish_cmd_vel(lv, av)
        return False

    def _nav_to(self, tx: float, ty: float) -> bool:
        """단일 웨이포인트 이동. 가까운 뒤쪽 목표는 180도 회전 대신 후진."""
        x, y, hdg = self._get_xy_hdg()
        dx, dy    = tx - x, ty - y
        dist      = math.hypot(dx, dy)
        if dist <= self.NAV_TOL:
            self._publish_cmd_vel(0.0, 0.0)
            return True

        target_hdg = math.atan2(dy, dx)
        fwd_err = self._angle_err(target_hdg, hdg)
        rev_err = self._angle_err(target_hdg + math.pi, hdg)
        use_reverse = abs(fwd_err) > 2.2 and abs(rev_err) < 0.5

        err_hdg = rev_err if use_reverse else fwd_err
        speed = self.MAX_V * min(1.0, dist / 1.5)
        if use_reverse:
            speed *= -0.6
        lv = 0.0 if abs(err_hdg) > 0.5 else speed
        av = max(-self.MAX_W, min(self.MAX_W, self.KP_W * err_hdg))
        self._publish_cmd_vel(lv, av)
        return False

    def _drive_axis_to(self, tx: float, ty: float, axis: str) -> bool:
        """Move on one axis only, choosing forward or reverse with the smaller yaw change.

        On exact 90-degree ties, prefer the positive yaw turn and drive backward.
        That avoids the repeated negative-yaw spin seen when going to the first slot.
        """
        x, y, hdg = self._get_xy_hdg()
        if axis == "x":
            delta = tx - x
            forward_yaw = 0.0 if delta >= 0.0 else math.pi
            reverse_yaw = math.pi if delta >= 0.0 else 0.0
        else:
            delta = ty - y
            forward_yaw = math.pi / 2.0 if delta >= 0.0 else -math.pi / 2.0
            reverse_yaw = -math.pi / 2.0 if delta >= 0.0 else math.pi / 2.0

        if abs(delta) <= self.NAV_TOL:
            self._publish_cmd_vel(0.0, 0.0)
            return True

        fwd_err = self._angle_err(forward_yaw, hdg)
        rev_err = self._angle_err(reverse_yaw, hdg)
        use_reverse = (
            abs(rev_err) < abs(fwd_err) - 1e-3 or
            abs(abs(rev_err) - abs(fwd_err)) <= 1e-3 and rev_err > fwd_err
        )
        err_hdg = rev_err if use_reverse else fwd_err
        lv = self.MAX_V * 0.6 * min(1.0, abs(delta) / 1.0)
        if use_reverse:
            lv = -lv
        if abs(err_hdg) > 0.5:
            lv = 0.0
        av = max(-self.MAX_W, min(self.MAX_W, self.KP_W * err_hdg))
        self._publish_cmd_vel(lv, av)
        return False

    def _get_drop_pos(self) -> tuple:
        """섹션 슬롯 01 배치 위치 (x, y) — IW Hub 배달 전용 예약 슬롯."""
        from robot_config import SECTION_PODS
        positions = SECTION_PODS.get(self._section_name, [])
        if not positions:
            return (0.0, 0.0)
        # 슬롯 01 (인덱스 0): world_setup 에서 비워 둔 IW Hub 전용 위치
        x, y, _ = positions[0]
        return float(x), float(y)

    def _get_signal_count(self) -> int:
        """work_signals (직접 Python) 과 ROS2 콜백 중 더 큰 값 반환."""
        try:
            import work_signals as _ws
            direct = _ws.get(self._section_name)
        except Exception:
            direct = 0
        with self._complete_lock:
            ros2_cnt = self._complete_count
        return max(direct, ros2_cnt)

    def _reset_signal_count(self) -> None:
        """work_signals 와 ROS2 카운터 모두 초기화."""
        try:
            import work_signals as _ws
            _ws.reset(self._section_name)
        except Exception:
            pass
        with self._complete_lock:
            self._complete_count = 0

    # ── 픽업 모드 전용 헬퍼 ──────────────────────────────────────────

    def _build_delivery_slots(self) -> list:
        """Section B 배달 슬롯 목록 (SECTION_PODS 기준 모든 위치)."""
        from robot_config import SECTION_PODS
        positions = SECTION_PODS.get(self._section_name, [])
        return [(float(p[0]), float(p[1])) for p in positions]

    def _place_slot_target(self, tx: float, ty: float) -> tuple:
        """Loaded pod placement target. Use the exact SECTION_PODS slot."""
        return tx, ty

    def _slot_aisle_x(self, slot_x: float) -> float:
        """Target slot 옆의 통로 x. 마지막 docking 전까지 pod 중심선 이동을 피한다."""
        if slot_x < 0.0:
            return slot_x - 1.4
        if slot_x > 0.0:
            return slot_x - 1.4
        return slot_x - 1.4

    def _approach_section_slot(self, tx: float, ty: float,
                               yfirst: bool = True,
                               final_y_tol: float = None) -> bool:
        """Move through an aisle, then dock exactly into the requested slot."""
        aisle_x = self._slot_aisle_x(tx)
        x, y, _ = self._get_xy_hdg()
        # The aisle is only an entry gate. Once the hub has passed the aisle
        # toward the slot, never drive back to the aisle; continue X-only.
        before_aisle = x < aisle_x - self.NAV_TOL if tx >= aisle_x else x > aisle_x + self.NAV_TOL

        if yfirst:
            if abs(y - ty) > self.NAV_TOL:
                return self._drive_axis_to(x, ty, "y")
            if before_aisle:
                return self._drive_axis_to(aisle_x, ty, "x")
        else:
            if before_aisle:
                return self._drive_axis_to(aisle_x, y, "x")
            if abs(y - ty) > self.NAV_TOL:
                return self._drive_axis_to(aisle_x, ty, "y")

        old_tol = self.NAV_TOL
        y_tol = self.NAV_TOL if final_y_tol is None else float(final_y_tol)
        self.NAV_TOL = self.DOCK_TOL
        try:
            x, y, _ = self._get_xy_hdg()
            if abs(y - ty) > y_tol:
                return self._drive_axis_to(x, ty, "y")
            return self._dock_x_into_slot(tx, ty)
        finally:
            self.NAV_TOL = old_tol

    def _dock_x_into_slot(self, tx: float, ty: float) -> bool:
        """Final placement is X-only. Do not touch yaw or Y here."""
        x, _, hdg = self._get_xy_hdg()
        dx = tx - x
        if abs(dx) <= self.DOCK_TOL:
            self._publish_cmd_vel(0.0, 0.0)
            return True

        facing_plus_x = math.cos(hdg) >= 0.0
        need_plus_x = dx >= 0.0
        speed = max(0.18, min(0.45, abs(dx) * 0.45))
        lv = speed if facing_plus_x == need_plus_x else -speed
        self._publish_cmd_vel(lv, 0.0)
        return False

    def _nav_axis_aligned(self, tx: float, ty: float, yfirst: bool = False) -> bool:
        """축 정렬 이동 (대각선 금지). yfirst=True 이면 Y→X, 기본은 X→Y. 도착 시 True."""
        x, y, _ = self._get_xy_hdg()
        if yfirst:
            if abs(ty - y) > self.NAV_TOL:
                return self._drive_axis_to(x, ty, "y")
            if abs(tx - x) > self.NAV_TOL:
                return self._drive_axis_to(tx, ty, "x")
        else:
            if abs(tx - x) > self.NAV_TOL:
                return self._drive_axis_to(tx, y, "x")
            if abs(ty - y) > self.NAV_TOL:
                return self._drive_axis_to(tx, ty, "y")
        self._publish_cmd_vel(0.0, 0.0)
        return True

    def _turn_to_heading(self, target_yaw: float) -> bool:
        """목표 헤딩으로 제자리 회전. 오차<0.1rad 시 True."""
        _, _, hdg = self._get_xy_hdg()
        err = self._angle_err(target_yaw, hdg)
        if abs(err) < 0.1:
            self._publish_cmd_vel(0.0, 0.0)
            return True
        self._publish_cmd_vel(0.0, max(-self.MAX_W, min(self.MAX_W, self.KP_W * err)))
        return False

    def _drive_along_x(self, tx: float) -> bool:
        """X축 이동 전용 래퍼."""
        _, y, _ = self._get_xy_hdg()
        return self._drive_axis_to(tx, y, "x")

    def _run_lift_phase(self, up: bool) -> bool:
        """리프트 올리기(up=True) 또는 내리기(up=False) 시퀀스. 완료 시 True."""
        self._fsm_step += 1
        t = min(self._fsm_step / self.LIFT_STEPS, 1.0)
        self._publish_lift(t * self.LIFT_UP if up else (1.0 - t) * self.LIFT_UP)
        return self._fsm_step >= self.LIFT_STEPS

    def _get_next_delivery_slot(self) -> tuple:
        if not self._delivery_slots:
            return self._home_xy
        return self._delivery_slots[self._delivery_idx % len(self._delivery_slots)]

    # ── 픽업 모드 FSM ────────────────────────────────────────────────
    # 시나리오 (X or Y 한 축씩 이동, 불필요한 180° 회전 없음):
    #   spawn(-6.45,1.5) → GOTO_PICKUP(-7.9,1.5) [X축 후진]
    #   → LIFTING → GOTO_INTERM(-6.0,1.5) [X축 전진/후진]
    #   → GOTO_SLOT(slot01, Y-first) → LOWERING
    #   → BACKOUT_DROP → GOTO_NEXT_POD → LIFTING_OUT
    #   → BACKOUT_PICK → GOTO_CORR_RETURN → GOTO_SUPPLY → LOWERING_OUT
    #   → RETREAT → WAITING
    def _run_pickup_fsm(self) -> None:
        cnt       = self._get_signal_count()
        px, py    = self._pickup_xy      # (-7.9, 1.5)
        cx, cy    = _CORRIDOR_XY         # (-6.0, 1.5)

        if self._pickup_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] WAITING  신호={cnt}/{self._complete_needed}  "
                      f"pos=({x:.2f},{y:.2f})")
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self._reset_dock_pid()
                self._pickup_state = "GOTO_PICKUP"
                self._fsm_step     = 0
                print(f"[{self.name}] WAITING → GOTO_PICKUP  pickup={self._pickup_xy}")

        elif self._pickup_state == "GOTO_PICKUP":
            # X축만 이동. 동향이면 후진, 서향이면 전진 (180° 회전 불필요)
            if self._drive_along_x(px):
                self._reset_dock_pid()
                self._pickup_state = "LIFTING"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_PICKUP → LIFTING")

        elif self._pickup_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_INTERM"
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING → GOTO_INTERM  interm={_CORRIDOR_XY}")

        elif self._pickup_state == "GOTO_INTERM":
            # X축만 이동. 동향이면 전진, 서향이면 후진 (180° 회전 불필요)
            if self._drive_along_x(cx):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SLOT"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_INTERM → GOTO_SLOT  slot={self._get_next_delivery_slot()}")

        elif self._pickup_state == "GOTO_SLOT":
            raw_tx, raw_ty = self._get_next_delivery_slot()
            tx, ty = self._place_slot_target(raw_tx, raw_ty)
            if self._approach_section_slot(tx, ty, yfirst=True,
                                           final_y_tol=0.25):
                mx, my, _ = self._get_xy_hdg()
                if abs(mx - tx) > self.DOCK_TOL:
                    print(f"[{self.name}] GOTO_SLOT minimap guard  "
                          f"pos=({mx:.2f},{my:.2f}) target=({tx:.2f},{ty:.2f})")
                    return
                # 드롭 위치에서 2m 후진할 목표 X 저장
                self._backout_x    = self._slot_aisle_x(tx)
                self._dock_target  = (tx, ty)
                self._reset_dock_pid()
                self._pickup_state = "LOWERING"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SLOT → LOWERING  at=({tx:.2f},{ty:.2f})")

        elif self._pickup_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._drop_idx    += 1
                self._delivery_idx = (self._delivery_idx + 1) % max(1, len(self._delivery_slots))
                self._reset_dock_pid()
                self._pickup_state = "BACKOUT_DROP"
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING → BACKOUT_DROP  "
                      f"target_x={self._backout_x:.2f}  (배달 #{self._drop_idx})")

        elif self._pickup_state == "BACKOUT_DROP":
            # 방금 내려놓은 포드에서 먼저 X축으로 이탈한 뒤 다음 pod 로 접근
            if self._drive_along_x(self._backout_x):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_NEXT_POD"
                self._fsm_step     = 0
                print(f"[{self.name}] BACKOUT_DROP → GOTO_NEXT_POD  "
                      f"next={self._get_next_delivery_slot()}")

        elif self._pickup_state == "GOTO_NEXT_POD":
            tx, ty = self._get_next_delivery_slot()
            if self._approach_section_slot(tx, ty, yfirst=True):
                self._backout_x    = self._slot_aisle_x(tx)
                self._dock_target  = (tx, ty)
                self._reset_dock_pid()
                self._pickup_state = "LIFTING_OUT"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_NEXT_POD → LIFTING_OUT  at=({tx:.2f},{ty:.2f})")

        elif self._pickup_state == "LIFTING_OUT":
            if self._run_lift_phase(up=True):
                self._reset_dock_pid()
                self._pickup_state = "BACKOUT_PICK"
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING_OUT → BACKOUT_PICK  target_x={self._backout_x:.2f}")

        elif self._pickup_state == "BACKOUT_PICK":
            if self._drive_along_x(self._backout_x):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_CORR_RETURN"
                self._fsm_step     = 0
                print(f"[{self.name}] BACKOUT_PICK → GOTO_CORR_RETURN  corridor={_CORRIDOR_XY}")

        elif self._pickup_state == "GOTO_CORR_RETURN":
            if self._nav_axis_aligned(cx, cy, yfirst=True):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SUPPLY"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_CORR_RETURN → GOTO_SUPPLY  supply={self._pickup_xy}")

        elif self._pickup_state == "GOTO_SUPPLY":
            if self._nav_axis_aligned(px, py):
                self._reset_dock_pid()
                self._pickup_state = "LOWERING_OUT"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SUPPLY → LOWERING_OUT")

        elif self._pickup_state == "LOWERING_OUT":
            if self._run_lift_phase(up=False):
                self._delivery_idx = (self._delivery_idx + 1) % max(1, len(self._delivery_slots))
                self._reset_dock_pid()
                self._pickup_state = "RETREAT"
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING_OUT → RETREAT")

        elif self._pickup_state == "RETREAT":
            hx, hy = self._home_xy
            if self._nav_axis_aligned(hx, hy):
                self._reset_dock_pid()
                self._pickup_state = "WAITING"
                self._fsm_step     = 0
                print(f"[{self.name}] RETREAT → WAITING  (cycle #{self._drop_idx})")

    # ── 표준 모드 FSM ────────────────────────────────────────────────
    # 상태:
    #   0 WAITING      M0609 완료 신호 대기
    #   1 LIFTING      리프트 업 (LIFT_STEPS FSM 틱)
    #   2 GOTO_SECTION 섹션 슬롯 01 로 이동
    #   3 LOWERING     리프트 다운 (LIFT_STEPS FSM 틱)
    #   4 MOVE_OUT     슬롯 01 에서 벗어나는 이동 (홈 방향으로 1m)
    #   5 GOTO_HOME    홈(PodStack) 으로 복귀

    def _run_fsm(self) -> None:
        cnt = self._get_signal_count()

        if self.mission_state == 0:              # WAITING
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                # 리프트는 LIFTING 상태에서 서서히 올림 — 여기서 즉시 발행하지 않음
                self.mission_state = 1
                self._fsm_step     = 0
                print(f"[{self.name}] WAITING → LIFTING  (신호 {cnt}회 수신)")

        elif self.mission_state == 1:            # LIFTING
            self._fsm_step += 1
            # 서서히 리프트 올림: 0 → LIFT_UP (포드 넘어짐 방지)
            t = min(self._fsm_step / self.LIFT_STEPS, 1.0)
            self._publish_lift(t * self.LIFT_UP)
            if self._fsm_step >= self.LIFT_STEPS:
                self._nav_target   = self._get_drop_pos()
                self._plan_path_to(*self._nav_target)
                self.mission_state = 2
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING → GOTO_SECTION({self._section_name}) "
                      f"슬롯01={self._nav_target}")

        elif self.mission_state == 2:            # GOTO_SECTION
            if self._nav_along_path():
                # 리프트는 LOWERING 상태에서 서서히 내림 — 여기서 즉시 발행하지 않음
                self.mission_state = 3
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SECTION → LOWERING")

        elif self.mission_state == 3:            # LOWERING
            self._fsm_step += 1
            # 서서히 리프트 내림: LIFT_UP → 0 (포드 넘어짐 방지)
            t = min(self._fsm_step / self.LIFT_STEPS, 1.0)
            self._publish_lift((1.0 - t) * self.LIFT_UP)
            if self._fsm_step >= self.LIFT_STEPS:
                self._drop_idx += 1
                # 슬롯 위치에서 홈 방향으로 1m 물러나는 중간 웨이포인트 계산
                sx, sy = self._get_drop_pos()
                hx, hy = self._home_xy
                dist = math.hypot(hx - sx, hy - sy)
                if dist > 0.1:
                    ratio = min(1.0, 1.0 / dist)
                    mx = sx + (hx - sx) * ratio
                    my = sy + (hy - sy) * ratio
                else:
                    mx, my = hx, hy
                self._nav_target = (mx, my)
                self._plan_path_to(mx, my)
                self.mission_state = 4
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING → MOVE_OUT  "
                      f"중간목표=({mx:.2f},{my:.2f})")

        elif self.mission_state == 4:            # MOVE_OUT
            if self._nav_along_path():
                self._nav_target = self._home_xy
                self._plan_path_to(*self._home_xy)
                self.mission_state = 5
                self._fsm_step     = 0
                print(f"[{self.name}] MOVE_OUT → GOTO_HOME")

        elif self.mission_state == 5:            # GOTO_HOME
            if self._nav_along_path():
                self.mission_state = 0
                print(f"[{self.name}] GOTO_HOME → WAITING (배달 #{self._drop_idx})")

    # ── physics 콜백 ─────────────────────────────────────────────────
    def on_physics_step(self, dt: float) -> None:
        self._physics_step += 1
        # 직접구동 지연 초기화: post_reset 에서 실패한 경우 physics 안정화 후 재시도
        if self._articulation is None and self._physics_step % 100 == 5:
            self._init_direct_drive()
        # odom 콜백 처리: FSM과 같은 주기(PUB_EVERY)로 spin — 항상 최신 heading 확보
        if _ROS2_AVAILABLE and _ros2_node is not None and self._physics_step % self.PUB_EVERY == 0:
            try:
                rclpy.spin_once(_ros2_node, timeout_sec=0)
            except Exception:
                pass
        if self._physics_step % self.PUB_EVERY != 0:
            return
        if self._mode == "pickup":
            self._run_pickup_fsm()
        else:
            self._run_fsm()
