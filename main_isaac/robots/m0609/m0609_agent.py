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

try:
    import sys as _sys
    for _p in [
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
    ]:
        if _p not in _sys.path:
            _sys.path.insert(0, _p)
    import rclpy
    from std_msgs.msg import String as _RosString
    _ROS2_AVAILABLE = True
    print("[M0609] rclpy import 성공")
except Exception as _e:
    print(f"[M0609] rclpy import 실패: {_e}")
    _ROS2_AVAILABLE = False

_ros2_node = None

def _get_ros2_node():
    """모듈 공유 ROS2 노드를 반환. 최초 1회만 init/create."""
    global _ros2_node
    if not _ROS2_AVAILABLE:
        return None
    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        pass  # 이미 초기화됨 (Isaac Sim ROS bridge 등)
    if _ros2_node is None:
        _ros2_node = rclpy.create_node("isaac_m0609_node")
        print("[M0609] ROS2 노드 생성: isaac_m0609_node")
    return _ros2_node


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
_HOME_DEG      = np.array([110.0, 0.0, 70.0, 0.0, 100.0, 0.0])  # joint_4 -90° → EE 아래 향함
_HOME_TOL_DEG  = 5.0
_SERVO_PX2WLD  = np.array([[0.0, -1.0], [-1.0, 0.0]])

_EE_INIT_H     = 0.25
_EE_OFFSET     = np.array([0.0, 0.0, 0.11])
_EVENTS_DT     = [0.008, 0.005, 0.02, 0.02, 0.005, 0.01, 0.005, 0.05, 0.008, 0.08]

# ── 흡착/해제 근접 임계값 ─────────────────────────────────────────────
# EE(link_6)와 큐브 픽업 위치 사이 거리가 이 값 이하일 때 큐브를 흡착
# 그리퍼 총 길이(_SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT) + 여유 ← 수정 가능
_ATTACH_REACH   = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT + 0.045   # [m]

_CUBE_EDGE     = 0.05
_ARUCO_LEN     = 0.045 * (600 / 720)   # ≈ 0.0375 m  (USDA 박스 고정 크기 기준)
_ARUCO_Z_OFF   = _CUBE_EDGE / 2.0 + 0.001  # 0.026 m

# 새 pick 로직 파라미터
_PICK_ABOVE_H  = 0.5    # [m] 마커 위 접근 높이               ← 수정 가능
_APPROACH_TOL  = 0.15   # [m] RMPFlow 목표 도달 판정 거리    ← 수정 가능
_MOVEL_STEPS   = 60     # MOVEL 보간 스텝 수                  ← 수정 가능

# 시각 서보 하강 (SERVO_DESCEND) 파라미터 — multi_targets 모드 전용
_SERVO_DZ      = 0.005  # [m/FSM tick] Z 하강 속도 (FSM 50Hz → 0.25 m/s)
_SERVO_GAIN    = 0.0008 # [m/px] 픽셀 오차 → XY 보정 게인


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
        box_type  = self.cfg.get("box_type", "red_id1")
        multi_cfg = self.cfg.get("multi_targets", None)

        def _aruco_len_from_wh(wh):
            if wh is not None:
                return min(wh[0], wh[1]) * 0.9 * (600.0 / 720.0)
            return _ARUCO_LEN

        if multi_cfg:
            self._multi_targets = [
                {
                    "aruco_id" : _ARUCO_ID_FROM_BOX[t["box_type"]],
                    "goal_pos" : np.array(t["goal_xyz"]),
                    "aruco_len": _aruco_len_from_wh(t.get("aruco_box_wh")),
                }
                for t in multi_cfg
            ]
            self._id_to_goal  = {t["aruco_id"]: t["goal_pos"] for t in self._multi_targets}
            self._aruco_id    = self._multi_targets[0]["aruco_id"]
            self._goal_pos    = self._multi_targets[0]["goal_pos"]
            self._aruco_len   = self._multi_targets[0]["aruco_len"]
            self._aruco_z_off = 0.002  # AutoSpawnPanel 박스 공통
            box_type          = multi_cfg[0]["box_type"]
        else:
            self._multi_targets = None
            self._id_to_goal    = None
            self._aruco_id      = _ARUCO_ID_FROM_BOX.get(box_type, 1)
            self._goal_pos      = np.array(self.cfg["goal_xyz"])
            # ArUco marker_length = 박스 짧은 면 × 0.9 (plane 크기) × (600/720) (quiet zone 제외)
            wh = self.cfg.get("aruco_box_wh", None)
            if wh is not None:
                self._aruco_len   = _aruco_len_from_wh(wh)
                self._aruco_z_off = 0.002
            else:
                self._aruco_len   = _ARUCO_LEN
                self._aruco_z_off = _ARUCO_Z_OFF

        # ScaleOp 이 물리에도 적용되어 팔 길이가 scale 배가 됨.
        # RMPFlow 는 1x URDF 로 IK 계산 → physics EE = 2*rmpflow_target - base.
        # _apply_ee 에서 rmpflow_target = base + (target - base) / scale 로 보정.
        self._scale       = scale
        self._base_pos    = np.array(spawn, dtype=np.float64)
        self._ee_init_h   = _EE_INIT_H
        self._ee_offset   = _EE_OFFSET
        self._attach_reach = _ATTACH_REACH * scale
        # ★ robot_config.py 에서 "pad_reach" 키로 직접 지정 가능.
        #   미지정 시 gripper 상수 × scale 로 자동 계산.
        self._pad_reach    = float(self.cfg.get(
            "pad_reach",
            (_SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT) * scale
        ))
        self._pick_offset  = np.array(
            self.cfg.get("pick_xyz_offset", [0.0, 0.0, 0.0]), dtype=np.float64
        )
        wp = self.cfg.get("waypoint_xyz", None)
        self._waypoint_xyz = np.array(wp, dtype=np.float64) if wp is not None else None
        self._movel_steps       = int(self.cfg.get("movel_steps", _MOVEL_STEPS))
        self._home_return_steps = int(self.cfg.get("home_return_steps", 250))
        self._approach_tol      = float(self.cfg.get("approach_tol", _APPROACH_TOL))
        self._servo_dz          = float(self.cfg.get("servo_dz", _SERVO_DZ))
        j1_lim = self.cfg.get("joint1_limits_deg", None)
        if j1_lim:
            self._j1_lo = np.deg2rad(j1_lim[0])
            self._j1_hi = np.deg2rad(j1_lim[1])
        else:
            self._j1_lo = None
            self._j1_hi = None

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

        # ── 받침대 큐브 (선택) ──────────────────────────────────────
        pedestal = self.cfg.get("pedestal", None)
        if pedestal:
            self._build_pedestal(stage, spawn, pedestal)

        # ── 흡착 그리퍼 형상 (ScaleOp 로 자동 확대됨) ──────────────
        self._suction_path, cam_mount_path = self._build_suction_gripper(stage, robot_ee)
        self._grip_body_path = self._suction_path
        print(f"[{self.name}] 흡착 그리퍼 생성 완료: {self._suction_path}")

        # ── world_setup 에서 로드된 ArUco USDA 박스 참조 ───────────
        # 박스는 BoxSpawner가 동적으로 /World/DynamicBoxes/ 에 스폰하므로
        # 해당 경로가 존재할 때만 래핑 (없으면 None으로 폴백)
        self._cube = None
        if not self._multi_targets:
            box_prim_path = f"/World/ArUcoBoxes/{box_type}"
            _check_stage = omni.usd.get_context().get_stage()
            if _check_stage.GetPrimAtPath(box_prim_path).IsValid():
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

        self._pick_count = 0   # 누적 픽앤플레이스 횟수
        self._work_complete_count = int(self.cfg.get("work_complete_count", 3))
        self._pending_wait      = False  # 완료 publish 후 WAITING 전환 예약
        self._start_signal_count = 0    # X_start 수신 누적 횟수 (로봇별)
        self._wait_signal_count  = 0    # WAITING 진입 시점의 _start_signal_count 값

        # ROS2 publisher + subscriber: /{robot_name}/work
        self._ros2_pub = None
        print(f"[{self.name}] ROS2 available={_ROS2_AVAILABLE}, multi={self._multi_targets}")
        if _ROS2_AVAILABLE and not self._multi_targets:
            node = _get_ros2_node()
            if node is not None:
                _topic  = f"/{self.name[0].lower()}{self.name[1:]}/work"
                _suffix = self.name.split("_")[-1]   # "A" / "B" / "C"
                _expect = f"{_suffix}_start"

                self._ros2_pub = node.create_publisher(_RosString, _topic, 10)

                def _make_cb(robot, expect):
                    def _cb(msg):
                        if msg.data == expect:
                            robot._start_signal_count += 1
                            print(f"[{robot.name}] {_topic} ← '{msg.data}' 수신  (재개 #{robot._start_signal_count})")
                    return _cb

                node.create_subscription(_RosString, _topic, _make_cb(self, _expect), 10)
                print(f"[{self.name}] ROS2 publisher+subscriber: {_topic}  (완료:{self._work_complete_count}회 / 재개:'{_expect}')")

        self._init_state(spawn)
        print(f"[{self.name}] setup 완료  spawn={spawn}  yaw={yaw_deg}°  "
              f"goal={self._goal_pos}")

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

        if self._multi_targets:
            self._trackers = {
                t["aruco_id"]: ArucoTracker(
                    marker_length=t["aruco_len"], target_id=t["aruco_id"], K=K
                )
                for t in self._multi_targets
            }
            self._tracker = next(iter(self._trackers.values()))
        else:
            self._trackers = None
            self._tracker = ArucoTracker(
                marker_length=self._aruco_len, target_id=self._aruco_id, K=K)
        self._servo = VisualServoController(
            image_size=_CAM_RES, pixel_to_world_xy=_SERVO_PX2WLD)
        self._viewer = CameraViewer(enabled=False)  # 카메라 창 비활성화

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
        home_deg = np.array(self.cfg.get("home_deg", _HOME_DEG), dtype=np.float64)
        self._home_pos = np.deg2rad(home_deg)
        if self._j1_lo is not None:
            self._home_pos[0] = np.clip(self._home_pos[0], self._j1_lo, self._j1_hi)
        self._home_tol = np.deg2rad(_HOME_TOL_DEG)

        self._state = "MOVE_TO_HOME"
        print(f"[{self.name}] post_reset 완료  home_deg={np.rad2deg(self._home_pos).round(1)}")

    # ── on_physics_step ──────────────────────────────────────────────
    def on_physics_step(self, _dt: float) -> None:
        if not hasattr(self, "_robot") or self._robot is None:
            return
        self._phys_cnt += 1
        if self._gripped:
            self._update_gripped_box()
        # ROS2 노드 spin (500틱 = 1초마다, 첫 번째 로봇만 실행)
        if _ROS2_AVAILABLE and _ros2_node is not None and self._phys_cnt % 500 == 0:
            rclpy.spin_once(_ros2_node, timeout_sec=0)
        # 홈 복귀: 물리 500Hz 로 관절 보간 (FSM 주기와 별도)
        if self._state == "RETURN_TO_HOME":
            self._step_home_return()
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

    def _build_pedestal(self, stage, spawn_xyz, size) -> None:
        """spawn_xyz 바로 아래에 고정 큐브 받침대를 생성."""
        w, d, h = float(size[0]), float(size[1]), float(size[2])
        cx = float(spawn_xyz[0])
        cy = float(spawn_xyz[1])
        cz = float(spawn_xyz[2]) - h / 2.0
        prim_path = f"/World/pedestal_{self.name}"
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
        xf.AddScaleOp().Set(Gf.Vec3f(w, d, h))
        cube.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.35, 0.35, 0.38)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rb.GetKinematicEnabledAttr().Set(True)
        print(f"[{self.name}] 받침대 생성: {prim_path}  "
              f"center=({cx:.3f},{cy:.3f},{cz:.3f})  size={size}")

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
        self._phys_cnt           = 0
        self._state              = "MOVE_TO_HOME"
        self._pick_world_pos     = None
        self._pick_above_xyz     = None
        self._goal_above_xyz     = None
        self._gripped            = False
        self._grip_prim_path     = None
        self._grab_offset_local  = None
        self._grab_R_rel         = None   # EE→박스 상대 회전 (흡착 시 기록)
        self._pick_ori_q         = None   # 그립 완료 시 EE 방향 (이후 운동에서 유지)
        self._movel_wps          = None
        self._movel_step         = 0
        self._det                = None
        self._home_ee_pos        = None   # MOVE_TO_HOME 완료 시 기록
        self._home_return_wps    = None   # 홈 복귀 관절 보간 웨이포인트
        self._home_return_step   = 0
        self._active_aruco_id    = None
        self._servo_target_xyz   = None  # SERVO_DESCEND 현재 목표 위치
        self._detect_cooldown    = 0     # Detecting 진입 후 감지 대기 틱 수
        self._approach_start_cnt = 0     # APPROACH_ABOVE 진입 시 phys_cnt

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
        # scale 보정: physics EE = base + scale*(rmpflow_target - base)
        # ∴ rmpflow_target = base + (target - base) / scale
        t = np.asarray(target_pos, dtype=np.float64)
        if self._scale != 1.0:
            t = self._base_pos + (t - self._base_pos) / self._scale
        actions = self._cspace.forward(
            target_end_effector_position=t,
            target_end_effector_orientation=ori,
        )
        if self._j1_lo is not None and actions.joint_positions is not None:
            jp = np.array(actions.joint_positions, dtype=np.float64)
            j1 = int(self._home_idx[0])
            jp[j1] = np.clip(jp[j1], self._j1_lo, self._j1_hi)
            actions = ArticulationAction(
                joint_positions=jp,
                joint_velocities=actions.joint_velocities,
                joint_efforts=actions.joint_efforts,
            )
        self._robot.apply_action(actions)

    def _step_home_return(self) -> None:
        """물리 500Hz 마다 호출: 관절 보간으로 홈 각도로 부드럽게 복귀."""
        if self._home_return_wps is None:
            return
        if self._home_return_step < len(self._home_return_wps):
            self._robot.apply_action(
                ArticulationAction(
                    joint_positions=self._home_return_wps[self._home_return_step],
                    joint_indices=self._home_idx,
                )
            )
            self._home_return_step += 1

    def _start_movel(self, start: np.ndarray, end: np.ndarray,
                     steps: int = _MOVEL_STEPS) -> None:
        """world frame 직선 보간 시작."""
        self._movel_wps  = np.linspace(start, end, self._movel_steps
                                        if steps == _MOVEL_STEPS else steps)
        self._movel_step = 0

    def _step_movel(self, ori=None) -> bool:
        """웨이포인트 한 칸 전진. ori 지정 시 EE 방향 고정. 완료 시 True 반환."""
        if self._movel_wps is None or self._movel_step >= len(self._movel_wps):
            return True
        self._apply_ee(self._movel_wps[self._movel_step], ori=ori)
        self._movel_step += 1
        return self._movel_step >= len(self._movel_wps)

    def _find_nearest_autobox(self, near_pos: np.ndarray,
                               max_dist: float = 1.5) -> "str | None":
        """
        near_pos 의 XY 반경 max_dist 이내 AutoBox prim 경로를 반환.
        AutoSpawnPanel 이 생성한 /World/AutoBox_XXXX 를 탐색.
        """
        stage = omni.usd.get_context().get_stage()
        best_path = None
        best_dist = max_dist
        world = stage.GetPrimAtPath("/World")
        cache = UsdGeom.XformCache()
        for prim in world.GetChildren() if world.IsValid() else []:
            path = str(prim.GetPath())
            if not path.startswith("/World/AutoBox_"):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            mat = cache.GetLocalToWorldTransform(prim)
            T   = np.array(mat, dtype=np.float64).T
            pos = T[:3, 3]
            # XY 거리만 비교 (높이 차이 무시)
            d = float(np.linalg.norm(pos[:2] - near_pos[:2]))
            if d < best_dist:
                best_dist = d
                best_path = path
        return best_path

    def _find_box_by_aruco_id(self, aruco_id: int):
        """ArUco ID에 해당하는 AutoBox 윗면(ArUco plane) 세계 좌표를 USD stage에서 직접 조회."""
        target_tex = f"aruco_id{aruco_id}.png"
        stage = omni.usd.get_context().get_stage()
        best_pos = None
        best_z   = -1e9

        world = stage.GetPrimAtPath("/World")
        cache = UsdGeom.XformCache()
        for prim in world.GetChildren() if world.IsValid() else []:
            path = str(prim.GetPath())
            if not path.startswith("/World/AutoBox_"):
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            tex_prim = stage.GetPrimAtPath(f"{path}/aruco_mat/Texture")
            if not tex_prim.IsValid():
                continue
            attr = tex_prim.GetAttribute("inputs:file")
            if not attr or target_tex not in str(attr.Get()):
                continue
            mat = cache.GetLocalToWorldTransform(prim)
            T   = np.array(mat, dtype=np.float64).T
            pos = T[:3, 3]
            # 박스 높이: /box 자식 prim 의 xformOp:scale Z
            bh = 0.15
            box_child = stage.GetPrimAtPath(f"{path}/box")
            if box_child.IsValid():
                xf = UsdGeom.Xformable(box_child)
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                        val = op.Get()
                        if val is not None:
                            bh = float(val[2])
                        break
            aruco_z = float(pos[2]) + bh / 2.0 + 0.002
            if aruco_z > best_z:
                best_z   = aruco_z
                best_pos = np.array([float(pos[0]), float(pos[1]), aruco_z])

        return best_pos

    def _attach_autobox(self, prim_path: str,
                        ideal_ee_pos: "np.ndarray | None" = None) -> bool:
        """
        AutoBox 를 kinematic 으로 전환하고 EE 오프셋 기록.
        이후 _update_gripped_box() 가 매 step USD translate 를 갱신.
        """
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return False
        # 물리 kinematic 전환 (중력 비활성화, USD 트랜스폼으로 제어)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
        # 박스 현재 월드 위치
        mat     = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        T       = np.array(mat, dtype=np.float64).T
        box_pos = T[:3, 3]
        # EE → 박스 위치/회전 오프셋 (EE 로컬 좌표계)
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        R_ee = _quat_wxyz_to_R(ee_q)
        # ideal_ee_pos: MOVEL 완료 시 물리 지연 때문에 ee_pos가 아직 목표에 미달.
        # pick_world_pos (계획된 위치)를 기준으로 오프셋을 계산해 일관성 확보.
        ee_ref = np.asarray(ideal_ee_pos, dtype=np.float64) \
                 if ideal_ee_pos is not None else ee_pos
        # 박스 월드 회전 추출 (USD transform, scale 정규화)
        R_box = T[:3, :3].copy()
        for i in range(3):
            n = np.linalg.norm(R_box[:, i])
            if n > 1e-9:
                R_box[:, i] /= n
        self._grip_prim_path    = prim_path
        self._grab_offset_local = R_ee.T @ (box_pos - ee_ref)
        self._grab_R_rel        = R_ee.T @ R_box
        self._gripped           = True
        print(f"[{self.name}] 흡착: {prim_path}")
        return True

    def _update_gripped_box(self):
        """
        매 physics step: 흡착된 AutoBox 를 EE 오프셋 위치로 이동.
        kinematic rigid body 는 USD translate 를 physics 가 직접 읽음.
        """
        if not self._gripped or self._grip_prim_path is None \
                or self._grab_offset_local is None:
            return
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        R_ee         = _quat_wxyz_to_R(ee_q)
        target       = ee_pos + R_ee @ self._grab_offset_local

        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(self._grip_prim_path)
        if not prim.IsValid():
            self._gripped        = False
            self._grip_prim_path = None
            return
        # 박스 새 회전: R_box = R_ee @ R_rel
        if self._grab_R_rel is not None:
            R_box_new = R_ee @ self._grab_R_rel
            q_wxyz = _R_to_quat_wxyz(R_box_new)
        else:
            q_wxyz = None

        # kinematic body: translate + orient op 갱신
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            ot = op.GetOpType()
            if ot == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(target[0]),
                                float(target[1]),
                                float(target[2])))
            elif ot == UsdGeom.XformOp.TypeOrient and q_wxyz is not None:
                w, x, y, z = q_wxyz
                op.Set(Gf.Quatf(float(w), float(x), float(y), float(z)))

    def _detach_autobox(self):
        """
        흡착 해제: kinematic → dynamic 전환 → 중력으로 낙하.
        """
        if self._grip_prim_path is None:
            self._gripped = False
            return
        stage = omni.usd.get_context().get_stage()
        prim  = stage.GetPrimAtPath(self._grip_prim_path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)
        print(f"[{self.name}] 흡착 해제 → 낙하: {self._grip_prim_path}")
        self._grip_prim_path    = None
        self._grab_offset_local = None
        self._gripped           = False

    def _display_label(self) -> str:
        if self._state == "WAITING":
            suffix = self.name.split("_")[-1]
            return f"Waiting {suffix}_start"

        labels = {
            "MOVE_TO_HOME"     : "Moving to Home",
            "Detecting"        : "Detecting...",
            "APPROACH_ABOVE"   : "Approaching Box",
            "SERVO_DESCEND"    : "Servo Picking...",
            "DESCEND_TO_GRIP"  : "Picking...",
            "LIFT_AFTER_GRIP"  : "Lifting...",
            "MOVE_TO_WAYPOINT" : "To Waypoint",
            "MOVE_TO_GOAL"     : "Moving to Goal",
            "DESCEND_TO_PLACE" : "Placing...",
            "RETRACT_PLACE"    : "Retracting",
            "RETURN_TO_HOME"   : "Returning Home",
        }
        return labels.get(self._state, self._state)

    def _reset_for_next_cycle(self):
        """배치 완료 후 팔 복귀 → 대기 사이클 시작."""
        self._gripped            = False
        self._grip_prim_path     = None
        self._grab_offset_local  = None
        self._grab_R_rel         = None
        self._pick_ori_q         = None
        self._pick_world_pos     = None
        self._pick_above_xyz     = None
        self._goal_above_xyz     = None
        self._movel_wps          = None
        self._movel_step         = 0
        self._det                = None
        self._active_aruco_id    = None
        # 홈 복귀 후 감지 쿨다운 (방금 내려놓은 박스 즉시 재감지 방지)
        self._detect_cooldown    = int(self.cfg.get("detect_cooldown_ticks", 100))
        # 현재 관절 → 홈 관절 선형 보간 (물리 500Hz 에서 실행)
        cur = self._robot.get_joint_positions()
        self._home_return_wps  = np.linspace(
            cur[self._home_idx], self._home_pos, self._home_return_steps
        )
        self._home_return_step = 0
        self._state = "RETURN_TO_HOME"
        print(f"[{self.name}] → RETURN_TO_HOME ({self._home_return_steps}스텝 보간)")

    # ══════════════════════════════════════════════════════════════════
    #  상태머신
    # ══════════════════════════════════════════════════════════════════

    def _run_fsm(self):
        robot  = self._robot
        joints = robot.get_joint_positions()
        ee_pos, _ = robot.end_effector.get_world_pose()

        rgb = self._wrist_cam.get_rgb() if self._wrist_cam else None
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb is not None else None
        det = None
        if bgr is not None:
            if self._trackers:
                for t_info in self._multi_targets:
                    d = self._trackers[t_info["aruco_id"]].detect(bgr)
                    if d is not None and d.found:
                        det = d
                        self._active_aruco_id = t_info["aruco_id"]
                        self._tracker = self._trackers[self._active_aruco_id]
                        break
            else:
                det = self._tracker.detect(bgr)
        self._det = det

        # ── MOVE_TO_HOME ─────────────────────────────────────────────
        if self._state == "MOVE_TO_HOME":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)
            err = np.max(np.abs(joints[self._home_idx] - self._home_pos))
            # 허용오차 이내 OR 5초(250 FSM 틱) 타임아웃 → Detecting 진입
            timeout = self._phys_cnt > 250 * _FSM_EVERY
            if err < self._home_tol or timeout:
                if timeout and err >= self._home_tol:
                    print(f"[{self.name}] MOVE_TO_HOME 타임아웃 (err={np.rad2deg(err):.1f}°) → Detecting")
                self._home_ee_pos = ee_pos.copy()
                self._state = "Detecting"
                print(f"[{self.name}] → Detecting  home_ee={ee_pos.round(3)}")

        # ── Detecting ────────────────────────────────────────────────
        # HOME_DEG 자세 그대로 유지하며 ArUco 감지 대기.
        # 다른 각도로 이동하지 않음 — 발견 즉시 SERVO 진입.
        elif self._state == "Detecting":
            robot.set_joint_positions(self._home_pos,
                                      joint_indices=self._home_idx)

            # ── 홈 복귀 후 쿨다운 (방금 내려놓은 박스 즉시 재감지 방지) ──
            if self._detect_cooldown > 0:
                self._detect_cooldown -= 1
                return

            # ── 진단 로그: 50 FSM 틱마다 검출 현황 출력 ─────────────
            if self._phys_cnt % (50 * _FSM_EVERY) == 0:
                if bgr is None:
                    print(f"[{self.name}][Detecting] 카메라 프레임 없음 (wrist_cam 미초기화?)")
                elif det is None:
                    target_ids = ([t["aruco_id"] for t in self._multi_targets]
                                  if self._multi_targets else [self._aruco_id])
                    print(f"[{self.name}][Detecting] 목표 IDs={target_ids} 검출 안됨")
                elif not det.found:
                    print(f"[{self.name}][Detecting] 검출된 IDs={det.all_detected_ids}, 목표 ID={self._aruco_id} 없음")
                else:
                    act_id = self._active_aruco_id if self._trackers else self._aruco_id
                    print(f"[{self.name}][Detecting] 목표 ID={act_id} 검출 중...")

            if det is not None and det.found:
                if self._id_to_goal is not None and self._active_aruco_id is not None:
                    self._goal_pos = self._id_to_goal[self._active_aruco_id]
                act_id = self._active_aruco_id if self._trackers else self._aruco_id

                # 1순위: USD stage에서 박스 위치 직접 조회 (multi_targets/AutoBox 전용)
                mw = self._find_box_by_aruco_id(act_id) if (self._multi_targets and act_id is not None) else None
                # fallback: _aruco_to_world (solvePnP 기반)
                if mw is None:
                    mw = self._aruco_to_world(det, self._wrist_cam._prim_path)
                # pick_xyz_offset 보정 (robot_config.py 에서 조절)
                if mw is not None:
                    mw = mw + self._pick_offset

                # ── 진단 로그: 1초마다 1회 출력 (50Hz 스팸 방지) ─────
                if self._phys_cnt % (50 * _FSM_EVERY) == 0:
                    if mw is None:
                        print(f"[{self.name}][DIAG] 검출 성공(ID={act_id}) but mw=None "
                              f"(tvec={'ok' if det.tvec is not None else 'None'}) "
                              f"— stage 탐색 실패, solvePnP도 실패")
                    else:
                        print(f"[{self.name}][DIAG] 검출 성공 ID={act_id} "
                              f"mw={np.round(mw,3)}  goal={np.round(self._goal_pos,3)}")

                if mw is not None:
                    # 박스 윗면 z (ArUco plane - z_off)
                    box_top_z = mw[2] - self._aruco_z_off
                    # EE(link_6)는 박스 윗면보다 pad_reach 만큼 위에 있어야
                    # 흡착 패드 끝이 박스 윗면에 닿음
                    # robot_config.py 의 "pad_reach" 키로 직접 조절 가능
                    pick_z               = box_top_z + self._pad_reach
                    self._pick_world_pos = np.array([mw[0], mw[1], pick_z])
                    self._pick_above_xyz = np.array([mw[0], mw[1], pick_z + _PICK_ABOVE_H])
                    self._goal_above_xyz = np.array([self._goal_pos[0],
                                                     self._goal_pos[1],
                                                     self._goal_pos[2] + _PICK_ABOVE_H])
                    self._state = "APPROACH_ABOVE"
                    self._approach_start_cnt = self._phys_cnt
                    print(f"[{self.name}] Detecting → APPROACH_ABOVE  pick={self._pick_world_pos.round(3)}  above={self._pick_above_xyz.round(3)}")

        # ── RETURN_TO_HOME ───────────────────────────────────────────
        # 관절 보간은 on_physics_step 의 _step_home_return() 에서 500Hz 로 처리.
        # FSM 은 수렴 확인만 담당.
        elif self._state == "RETURN_TO_HOME":
            err = np.max(np.abs(joints[self._home_idx] - self._home_pos))
            if err < self._home_tol:
                self._home_return_wps = None
                if self._pending_wait:
                    self._state = "WAITING"
                    _suffix = self.name.split("_")[-1]
                    print(f"[{self.name}] 홈 복귀 완료 → WAITING  (/{self.name[0].lower()}{self.name[1:]}/work 에서 '{_suffix}_start' 대기)")
                else:
                    self._state = "Detecting"
                    print(f"[{self.name}] 홈 복귀 완료 → Detecting")

        # ── WAITING ──────────────────────────────────────────────────
        # 완료 publish 후 X_start 신호 수신 대기
        elif self._state == "WAITING":
            if self._start_signal_count > self._wait_signal_count:
                self._pending_wait = False
                self._state = "Detecting"
                _suffix = self.name.split("_")[-1]
                print(f"[{self.name}] ★ '{_suffix}_start' 수신 → Detecting 재개")

        # ── APPROACH_ABOVE ───────────────────────────────────────────
        # RMPFlow 로 픽업 위치 0.5 m 위까지 이동
        elif self._state == "APPROACH_ABOVE":
            self._apply_ee(self._pick_above_xyz)
            dist = np.linalg.norm(ee_pos - self._pick_above_xyz)
            elapsed = self._phys_cnt - self._approach_start_cnt
            # 50 FSM 틱마다 EE 위치 vs 목표 출력
            if self._phys_cnt % (50 * _FSM_EVERY) == 0:
                print(f"[{self.name}][APPROACH] dist={dist:.3f}m  EE={np.round(ee_pos,3)}")
            # 10초 (500 FSM 틱) 타임아웃 → 홈 복귀
            if elapsed > 500 * _FSM_EVERY:
                print(f"[{self.name}][APPROACH] 타임아웃 (dist={dist:.3f}m) → RETURN_TO_HOME")
                self._reset_for_next_cycle()
                return
            if dist < self._approach_tol:
                # 박스 위 도달 시점 EE 방향 기록 → 하강/이후 상태에서 유지
                _, self._pick_ori_q = robot.end_effector.get_world_pose()
                if self._multi_targets:
                    self._servo_target_xyz = ee_pos.copy()
                    self._state = "SERVO_DESCEND"
                    print(f"[{self.name}] APPROACH_ABOVE → SERVO_DESCEND  EE={np.round(ee_pos,3)}")
                else:
                    self._start_movel(ee_pos, self._pick_world_pos)
                    self._state = "DESCEND_TO_GRIP"
                    print(f"[{self.name}] APPROACH_ABOVE → DESCEND_TO_GRIP  EE={np.round(ee_pos,3)}")

        # ── SERVO_DESCEND ─────────────────────────────────────────────
        # multi_targets 전용: ArUco 중심 추적하며 Z 하강
        elif self._state == "SERVO_DESCEND":
            if det is not None and det.found:
                px_err = np.array([det.cx - _CAM_CX, det.cy - _CAM_CY])
                xy_corr = _SERVO_PX2WLD @ px_err * _SERVO_GAIN
                self._servo_target_xyz[0] += xy_corr[0]
                self._servo_target_xyz[1] += xy_corr[1]
            self._servo_target_xyz[2] -= self._servo_dz
            self._apply_ee(self._servo_target_xyz, ori=self._pick_ori_q)
            if self._phys_cnt % (50 * _FSM_EVERY) == 0:
                print(f"[{self.name}][SERVO_DESCEND] z={self._servo_target_xyz[2]:.3f}"
                      f"  det={'O' if det and det.found else 'X'}")
            if self._servo_target_xyz[2] <= self._pick_world_pos[2]:
                box_path = self._find_nearest_autobox(self._pick_world_pos, max_dist=1.5)
                if box_path:
                    self._attach_autobox(box_path, ideal_ee_pos=self._pick_world_pos)
                    print(f"[{self.name}] SERVO_DESCEND → LIFT_AFTER_GRIP  (흡착 성공)")
                else:
                    print(f"[{self.name}] SERVO_DESCEND → LIFT_AFTER_GRIP  (흡착 실패: AutoBox 없음)")
                self._start_movel(ee_pos, self._pick_above_xyz)
                self._state = "LIFT_AFTER_GRIP"

        # ── DESCEND_TO_GRIP ──────────────────────────────────────────
        # 픽업 위치까지 직선 하강 (MOVEL)
        elif self._state == "DESCEND_TO_GRIP":
            done = self._step_movel(ori=self._pick_ori_q)   # 하강 중 EE 방향 유지
            if done:
                box_path = self._find_nearest_autobox(self._pick_world_pos, max_dist=1.5)
                if box_path:
                    self._attach_autobox(box_path, ideal_ee_pos=self._pick_world_pos)
                    print(f"[{self.name}] DESCEND_TO_GRIP → LIFT_AFTER_GRIP  (흡착 성공)")
                else:
                    print(f"[{self.name}] DESCEND_TO_GRIP → LIFT_AFTER_GRIP  (흡착 실패: AutoBox 없음  pos={np.round(self._pick_world_pos,3)})")
                self._start_movel(ee_pos, self._pick_above_xyz)
                self._state = "LIFT_AFTER_GRIP"

        # ── LIFT_AFTER_GRIP ──────────────────────────────────────────
        # 픽업 후 위로 올라가기 — 그립 방향 유지
        elif self._state == "LIFT_AFTER_GRIP":
            done = self._step_movel(ori=self._pick_ori_q)
            if done:
                if self._waypoint_xyz is not None:
                    self._state = "MOVE_TO_WAYPOINT"
                    print(f"[{self.name}] LIFT 완료 → MOVE_TO_WAYPOINT  wp={self._waypoint_xyz.round(3)}")
                else:
                    self._state = "MOVE_TO_GOAL"
                    print(f"[{self.name}] LIFT 완료 → MOVE_TO_GOAL  goal_above={self._goal_above_xyz.round(3)}")

        # ── MOVE_TO_WAYPOINT ─────────────────────────────────────────
        # waypoint_xyz 경유 후 MOVE_TO_GOAL — 그립 방향 유지
        elif self._state == "MOVE_TO_WAYPOINT":
            self._apply_ee(self._waypoint_xyz, ori=self._pick_ori_q)
            dist = np.linalg.norm(ee_pos - self._waypoint_xyz)
            if dist < self._approach_tol:
                self._state = "MOVE_TO_GOAL"
                print(f"[{self.name}] 웨이포인트 도달 → MOVE_TO_GOAL  goal_above={self._goal_above_xyz.round(3)}")

        # ── MOVE_TO_GOAL ─────────────────────────────────────────────
        # RMPFlow 로 목표 위치 위까지 이동 — 그립 방향 유지
        elif self._state == "MOVE_TO_GOAL":
            self._apply_ee(self._goal_above_xyz, ori=self._pick_ori_q)
            dist = np.linalg.norm(ee_pos - self._goal_above_xyz)
            if dist < self._approach_tol:
                goal_xyz = np.array(self._goal_pos, dtype=np.float64)
                self._start_movel(ee_pos, goal_xyz)
                self._state = "DESCEND_TO_PLACE"
                print(f"[{self.name}] MOVE_TO_GOAL 도달 → DESCEND_TO_PLACE")

        # ── DESCEND_TO_PLACE ─────────────────────────────────────────
        # 목표 위치까지 직선 하강 후 해제 — 그립 방향 유지
        elif self._state == "DESCEND_TO_PLACE":
            done = self._step_movel(ori=self._pick_ori_q)
            if done:
                self._detach_autobox()          # kinematic → dynamic → 낙하
                self._start_movel(ee_pos, self._goal_above_xyz)
                self._state = "RETRACT_PLACE"
                print(f"[{self.name}] PLACE 완료 → RETRACT_PLACE")

        # ── RETRACT_PLACE ────────────────────────────────────────────
        # 플레이스 후 위로 후퇴 — 방향 제약 없이 자연스럽게
        elif self._state == "RETRACT_PLACE":
            done = self._step_movel()           # ori 없음: 홈 복귀에 자연스러운 경로
            if done:
                self._pick_count += 1
                print(f"[{self.name}] ★ 픽앤플레이스 완료 #{self._pick_count} → RETURN_TO_HOME")

                if self._pick_count >= self._work_complete_count and self._ros2_pub is not None:
                    suffix = self.name.split("_")[-1]   # "A" / "B" / "C"
                    _msg = _RosString()
                    _msg.data = f"{suffix}_complete"
                    self._ros2_pub.publish(_msg)
                    print(f"[{self.name}] ★ ROS2 publish → '{_msg.data}'")
                    # work_signals: 3회 픽앤플레이스 완료 시에만 IW Hub 에 신호 전달
                    try:
                        import work_signals as _ws
                        _suffix_s = self.name.split("_")[-1]
                        _ws.signal(_suffix_s)
                        print(f"[{self.name}] work_signals.signal('{_suffix_s}') "
                              f"→ 누적 {_ws.get(_suffix_s)}")
                    except Exception as _e:
                        carb.log_warn(f"[{self.name}] work_signals 오류: {_e}")
                    self._pick_count = 0
                    # wait_after_complete=False 이면 WAITING 없이 계속 픽앤플레이스
                    if self.cfg.get("wait_after_complete", True):
                        self._pending_wait = True
                        self._wait_signal_count = self._start_signal_count  # 제출 시점 기록
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


def _R_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → (w, x, y, z) quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return np.array([0.25 / s,
                         (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s,
                         (R[1, 0] - R[0, 1]) * s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s,
                         0.25 * s,
                         (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s,
                         (R[0, 1] + R[1, 0]) / s,
                         0.25 * s,
                         (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[1, 0] - R[0, 1]) / s,
                         (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s,
                         0.25 * s])



def _get_world_T(prim_path: str) -> np.ndarray:
    """
    prim 의 local→world 변환을 반환.
    부모 ScaleOp(예: scale=2.0)가 rotation 컬럼에 흡수되어 있으므로
    각 컬럼을 정규화해 순수 rotation + translation 만 추출.
    정규화하지 않으면 tvec(미터 단위)에 scale 이 곱해져
    _aruco_to_world 가 2배 먼 위치를 반환하는 버그가 발생함.
    """
    stage = omni.usd.get_context().get_stage()
    prim  = stage.GetPrimAtPath(prim_path)
    mat   = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    T = np.array(mat, dtype=np.float64).T
    # rotation 컬럼의 스케일 제거 (컬럼 노름 = scale 값)
    for i in range(3):
        col_norm = np.linalg.norm(T[:3, i])
        if col_norm > 1e-9:
            T[:3, i] /= col_norm
    return T
