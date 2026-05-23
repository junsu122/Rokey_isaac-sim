"""
test_m0609_place.py
===================
M0609 Pick & Place 테스트 스크립트.

실행:
    cd /home/rokey/Rokey_isaac-sim
    python test_m0609_place.py

동작 순서:
    1. HOME 관절각 이동
    2. 접근 위치 → 픽업 위치 (MOVEL)
    3. 큐브 흡착 (kinematic 추적)
    4. JOINT_MID1 → JOINT_MID2 관절 이동
    5. Place 접근 → Place 목표 (MOVEL)  ← z 높이 순환
    6. 큐브 해제
    7. Place 후퇴 → JOINT_BACK 이동 → 반복
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
import carb
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import Gripper
from isaacsim.core.utils.types import ArticulationAction

# ──────────────────────────────────────────────────────────────────────
#  ★ 수정 가능한 파라미터
# ──────────────────────────────────────────────────────────────────────

# 파일 경로
WAREHOUSE_USD = "/home/rokey/Rokey_isaac-sim/main_isaac/usd/warehouse_v7_test_ver5.usda"
POD_USD    = "/home/rokey/gidong_ws/scene/pod/pod_stack_4.usda"
ROBOT_URDF = ("/home/rokey/dev_ws/main_isaac/robots/m0609"
              "/m0609_aruco_detect/doosan-robot2/urdf/m0609_isaac_sim.urdf")
_SRC_DIR   = "/home/rokey/dev_ws/main_isaac/robots/m0609/m0609_aruco_detect"
RMPFLOW_CFG  = _SRC_DIR + "/m0609_rmpflow_common.yaml"
DESC_YAML    = _SRC_DIR + "/m0609_rg2_description.yaml"

# 스폰 설정
POD_XYZ         = (-12.0, 7.5, 0.0)      # Pod USD 위치 [m]
ROBOT_XYZ       = (-13.4, 6.8, 0.0)      # 로봇 스폰 위치 [m]
ROBOT_YAW_DEG   = -90.0                   # 로봇 yaw [deg]
ROBOT_SCALE     = 2.0                     # 로봇 시각 스케일 배율

# 큐브 설정
CUBE_XYZ   = np.array([-12.0, 6.0, 0.9]) # 픽업 위치 = 큐브 초기 위치
CUBE_SCALE = np.array([0.2, 0.2, 0.2])   # 큐브 크기 [m]

# 홈 관절각 [deg]
HOME_DEG      = np.array([0.0,  0.0,  90.0, 0.0,  90.0, 0.0])

# 경유 관절각 [deg]
JOINT1_TURN_DEG = np.array([-90.0, 0.0,  90.0, 0.0,  90.0, 0.0])  # HOME 후 joint_1 -90°
JOINT_MID1_DEG  = np.array([  0.0, 0.0,   0.0, 0.0,   0.0, 0.0])
JOINT_MID2_DEG  = np.array([  0.0, 0.0, -90.0, 0.0, -90.0, 0.0])
JOINT_BACK_DEG  = np.array([  0.0, 0.0,   0.0, 0.0,   0.0, 0.0])

# EE 이동 위치 [world frame, m]
APPROACH_PRE_XYZ  = np.array([-11.6, 6.0, 0.5])  # 픽업 전 접근
PICK_XYZ          = np.array([-12.0, 6.0, 0.5])  # 픽업

PLACE_APPROACH_X  = -13.0    # Place 접근 X
PLACE_TARGET_X    = -12.1    # Place 목표 X
PLACE_Y           = 7.5      # Place Y
PLACE_Z_LIST      = [0.36, 0.65, 0.91]   # 1·2·3번째 Place Z

# 관절 수렴 허용 오차 [deg]
JOINT_TOL_DEG = 2.0

# MOVEL 보간 스텝 수 (FSM tick 기준)
MOVEL_STEPS = 80

# 흡착 근접 거리 [m]
ATTACH_REACH = 0.15

# ──────────────────────────────────────────────────────────────────────
#  더미 그리퍼 (관절 없음 — kinematic 흡착)
# ──────────────────────────────────────────────────────────────────────

class NoOpGripper(Gripper):
    def __init__(self, ee_path):
        super().__init__(end_effector_prim_path=ee_path)
    def initialize(self, physics_sim_view=None, **kw):
        super().initialize(physics_sim_view=physics_sim_view)
    def post_reset(self): pass
    def open(self): pass
    def close(self): pass
    def set_default_state(self, *a, **kw): pass
    def get_default_state(self, *a, **kw): return None
    def forward(self, *a, **kw): return ArticulationAction()


# ──────────────────────────────────────────────────────────────────────
#  헬퍼
# ──────────────────────────────────────────────────────────────────────

def _quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


# ──────────────────────────────────────────────────────────────────────
#  메인 에이전트
# ──────────────────────────────────────────────────────────────────────

class PlaceTestAgent:
    def __init__(self, world: World):
        self.world = world
        self._robot   = None
        self._cube    = None
        self._state   = "INIT"

        # 내부 상태
        self._gripped           = False
        self._grab_offset_local = None
        self._movel_wps         = None
        self._movel_ori         = None
        self._movel_step        = 0
        self._cycle_idx         = 0          # 0→1→2→0 순환
        self._phys_cnt          = 0
        self._home_pos          = None
        self._home_idx          = None
        self._cspace            = None
        self._cubes_in_scene    = []         # 배치된 큐브 목록

    # ── 씬 구성 ──────────────────────────────────────────────────────

    def setup(self):
        stage = omni.usd.get_context().get_stage()

        # 창고 맵 로드
        warehouse = define_prim("/World/Warehouse", "Xform")
        warehouse.GetReferences().AddReference(WAREHOUSE_USD)
        print(f"[Test] 창고 맵 로드 완료: {WAREHOUSE_USD}")

        # Pod USD 로드
        pod_prim = define_prim("/World/Pod", "Xform")
        pod_prim.GetReferences().AddReference(POD_USD)
        xf = UsdGeom.Xformable(pod_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*POD_XYZ))
        print(f"[Test] Pod 로드 완료: {POD_XYZ}")

        # URDF 임포트
        _, import_cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        import_cfg.merge_fixed_joints             = False
        import_cfg.convex_decomp                  = True
        import_cfg.import_inertia_tensor          = True
        import_cfg.fix_base                       = True
        import_cfg.distance_scale                 = 1.0
        import_cfg.default_drive_type            = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_cfg.default_drive_strength        = 1e10
        import_cfg.default_position_drive_damping = 1e5

        _, artic_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=ROBOT_URDF,
            import_config=import_cfg,
            get_articulation_root=True,
        )
        if artic_path is None:
            raise RuntimeError("URDF import 실패")
        robot_root = artic_path.rsplit("/", 1)[0] or artic_path

        # scale × translate × rotateZ 적용
        root_prim = stage.GetPrimAtPath(robot_root)
        xf = UsdGeom.Xformable(root_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*ROBOT_XYZ))
        xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(ROBOT_YAW_DEG)
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(ROBOT_SCALE, ROBOT_SCALE, ROBOT_SCALE))

        # EE 경로 검색
        robot_ee = self._find_prim(robot_root, "link_6") or f"{robot_root}/link_6"

        for _ in range(10):
            omni.kit.app.get_app().update()

        gripper = NoOpGripper(robot_ee)
        self._robot = self.world.scene.add(
            SingleManipulator(
                prim_path=artic_path,
                name="m0609_test",
                end_effector_prim_path=robot_ee,
                gripper=gripper,
            )
        )

        # 첫 번째 큐브 미리 생성
        self._spawn_cube()
        print("[Test] setup 완료")

    def _spawn_cube(self):
        """픽업 위치에 새 큐브 생성."""
        idx = len(self._cubes_in_scene)
        cube = self.world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/TestCube_{idx}",
                name=f"test_cube_{idx}",
                position=CUBE_XYZ.copy(),
                scale=CUBE_SCALE,
                color=np.array([0.8, 0.4, 0.1]),
                mass=0.05,
            )
        )
        self._cube = cube
        self._cubes_in_scene.append(cube)
        print(f"[Test] 큐브 {idx+1} 생성 at {CUBE_XYZ}")
        return cube

    # ── post_reset ───────────────────────────────────────────────────

    def post_reset(self):
        self._robot.initialize()
        yaw_rad = np.deg2rad(ROBOT_YAW_DEG)
        c, s = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
        self._robot.set_world_pose(
            position=np.array(ROBOT_XYZ, dtype=np.float64),
            orientation=np.array([c, 0.0, 0.0, s]),
        )
        self._robot.gripper.initialize(
            physics_sim_view=self.world.physics_sim_view,
            articulation_apply_action_func=self._robot.apply_action,
        )

        # RMPFlow 컨트롤러
        from m0609_rmpflow_controller import RMPFlowController
        self._cspace = RMPFlowController(
            name="test_rmpflow",
            robot_articulation=self._robot,
            urdf_path=ROBOT_URDF,
            robot_description_path=DESC_YAML,
            rmpflow_config_path=RMPFLOW_CFG,
            end_effector_frame_name="link_6",
        )

        dof_names = self._robot.dof_names
        self._home_idx = np.array([
            next((i for i, n in enumerate(dof_names)
                  if n.endswith(f"joint_{k+1}")), k)
            for k in range(6)
        ])
        self._home_pos = np.deg2rad(HOME_DEG)

        # ── 초기 붕괴 방지 ──────────────────────────────────────────
        # world.reset() 후 drive 명령 없이 physics step이 돌면
        # 중력에 의해 로봇이 즉시 무너짐.
        # set_joint_positions 로 HOME 에 텔레포트(1회)해서 drive target 초기화.
        self._robot.set_joint_positions(
            self._home_pos, joint_indices=self._home_idx)

        self._state = "MOVE_TO_HOME"
        print("[Test] post_reset 완료 → MOVE_TO_HOME")

    # ── physics 콜백 ─────────────────────────────────────────────────

    def on_physics_step(self, dt):
        self._phys_cnt += 1
        if self._gripped:
            self._update_cube()
        if self._phys_cnt % 10 != 0:
            return
        self._run_fsm()

    # ── kinematic 큐브 추적 ──────────────────────────────────────────

    def _attach(self):
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        cube_pos, _  = self._cube.get_world_pose()
        R = _quat_wxyz_to_R(ee_q)
        self._grab_offset_local = R.T @ (cube_pos - ee_pos)
        self._gripped = True
        print("[Test] 흡착!")

    def _update_cube(self):
        if self._grab_offset_local is None:
            return
        ee_pos, ee_q = self._robot.end_effector.get_world_pose()
        R = _quat_wxyz_to_R(ee_q)
        self._cube.set_world_pose(position=ee_pos + R @ self._grab_offset_local)
        self._cube.set_linear_velocity(np.zeros(3))
        self._cube.set_angular_velocity(np.zeros(3))

    def _detach(self):
        self._grab_offset_local = None
        self._gripped = False
        print("[Test] 해제!")

    # ── EE 제어 ──────────────────────────────────────────────────────

    def _apply_ee(self, pos, ori=None):
        actions = self._cspace.forward(
            target_end_effector_position=pos,
            target_end_effector_orientation=ori,
        )
        self._robot.apply_action(actions)

    # ── MOVEL ────────────────────────────────────────────────────────

    def _start_movel(self, start_pos, end_pos, ori=None, steps=MOVEL_STEPS):
        """world frame 직선 보간 시작."""
        self._movel_wps  = np.linspace(start_pos, end_pos, steps)
        self._movel_ori  = ori
        self._movel_step = 0

    def _step_movel(self) -> bool:
        """웨이포인트 한 칸 전진. 완료 시 True."""
        if self._movel_step >= len(self._movel_wps):
            return True
        self._apply_ee(self._movel_wps[self._movel_step], self._movel_ori)
        self._movel_step += 1
        return self._movel_step >= len(self._movel_wps)

    # ── 관절 이동 ─────────────────────────────────────────────────────

    def _set_joints(self, deg_array):
        # apply_action → PD drive 경유 → 실제 서보 이동
        # set_joint_positions 는 physics 직접 쓰기(텔레포트)이므로 사용 금지
        self._robot.apply_action(
            ArticulationAction(joint_positions=np.deg2rad(deg_array))
        )

    def _joint_err(self, deg_array) -> float:
        joints = self._robot.get_joint_positions()
        return float(np.max(np.abs(
            joints[self._home_idx] - np.deg2rad(deg_array))))

    # ── 현재 Place Z ─────────────────────────────────────────────────

    @property
    def _place_z(self) -> float:
        return PLACE_Z_LIST[self._cycle_idx % len(PLACE_Z_LIST)]

    # ── FSM ──────────────────────────────────────────────────────────

    def _run_fsm(self):
        ee_pos, ee_ori = self._robot.end_effector.get_world_pose()
        tol = np.deg2rad(JOINT_TOL_DEG)

        # ── MOVE_TO_HOME ─────────────────────────────────────────────
        if self._state == "MOVE_TO_HOME":
            self._set_joints(HOME_DEG)
            if self._joint_err(HOME_DEG) < tol:
                self._state = "JOINT1_TURN"
                print(f"[Test] HOME 완료 → JOINT1_TURN  cycle={self._cycle_idx}")

        # ── JOINT1_TURN: joint_1 → -90° (나머지 HOME 유지) ───────────
        elif self._state == "JOINT1_TURN":
            self._set_joints(JOINT1_TURN_DEG)
            if self._joint_err(JOINT1_TURN_DEG) < tol:
                self._state = "APPROACH_PRE"
                print("[Test] JOINT1_TURN 완료 → APPROACH_PRE")

        # ── APPROACH_PRE: EE → (-11.6, 6.0, 0.5) ────────────────────
        elif self._state == "APPROACH_PRE":
            self._apply_ee(APPROACH_PRE_XYZ)
            dist = np.linalg.norm(ee_pos - APPROACH_PRE_XYZ)
            if dist < 0.05:
                # APPROACH_PRE 도달 → MOVEL로 픽업 위치까지 직선 이동
                self._start_movel(APPROACH_PRE_XYZ, PICK_XYZ, ori=ee_ori)
                self._state = "MOVEL_PICK"
                print("[Test] APPROACH_PRE 도달 → MOVEL_PICK")

        # ── MOVEL_PICK: MOVEL → (-12.0, 6.0, 0.5) ───────────────────
        elif self._state == "MOVEL_PICK":
            done = self._step_movel()
            if done:
                dist = np.linalg.norm(ee_pos - PICK_XYZ)
                if dist < ATTACH_REACH:
                    self._attach()
                    self._state = "JOINT_MID1"
                    print("[Test] PICK 완료 → JOINT_MID1")

        # ── JOINT_MID1: 관절 (0,0,0,0,0,0) ──────────────────────────
        elif self._state == "JOINT_MID1":
            self._set_joints(JOINT_MID1_DEG)
            if self._joint_err(JOINT_MID1_DEG) < tol:
                self._state = "JOINT_MID2"
                print("[Test] JOINT_MID1 완료 → JOINT_MID2")

        # ── JOINT_MID2: 관절 (0,0,-90,0,-90,0) ──────────────────────
        elif self._state == "JOINT_MID2":
            self._set_joints(JOINT_MID2_DEG)
            if self._joint_err(JOINT_MID2_DEG) < tol:
                z = self._place_z
                approach = np.array([PLACE_APPROACH_X, PLACE_Y, z])
                self._start_movel(ee_pos, approach, ori=ee_ori)
                self._state = "MOVEL_PLACE_APPROACH"
                print(f"[Test] JOINT_MID2 완료 → MOVEL_PLACE_APPROACH  z={z}")

        # ── MOVEL_PLACE_APPROACH: EE → (-13, 7.5, z) ─────────────────
        elif self._state == "MOVEL_PLACE_APPROACH":
            done = self._step_movel()
            if done:
                z   = self._place_z
                src = np.array([PLACE_APPROACH_X, PLACE_Y, z])
                dst = np.array([PLACE_TARGET_X,   PLACE_Y, z])
                self._start_movel(src, dst, ori=self._movel_ori)
                self._state = "MOVEL_PLACE"
                print(f"[Test] Place 접근 완료 → MOVEL_PLACE  z={z}")

        # ── MOVEL_PLACE: MOVEL → (-12.1, 7.5, z) ────────────────────
        elif self._state == "MOVEL_PLACE":
            done = self._step_movel()
            if done:
                self._detach()
                self._cycle_idx += 1

                # 다음 사이클을 위한 큐브 스폰
                self._spawn_cube()

                # Place 후퇴로 이동
                z   = PLACE_Z_LIST[(self._cycle_idx - 1) % len(PLACE_Z_LIST)]
                src = np.array([PLACE_TARGET_X,   PLACE_Y, z])
                dst = np.array([PLACE_APPROACH_X, PLACE_Y, z])
                self._start_movel(src, dst, ori=self._movel_ori)
                self._state = "MOVEL_PLACE_RETRACT"
                print(f"[Test] PLACE 완료 (cycle={self._cycle_idx}) → MOVEL_PLACE_RETRACT")

        # ── MOVEL_PLACE_RETRACT: (-12.1,7.5,z) → (-13,7.5,z) ────────
        elif self._state == "MOVEL_PLACE_RETRACT":
            done = self._step_movel()
            if done:
                self._state = "JOINT_BACK"
                print("[Test] 후퇴 완료 → JOINT_BACK")

        # ── JOINT_BACK: 관절 (0,0,0,0,0,0) ──────────────────────────
        elif self._state == "JOINT_BACK":
            self._set_joints(JOINT_BACK_DEG)
            if self._joint_err(JOINT_BACK_DEG) < tol:
                self._state = "MOVE_TO_HOME"
                print("[Test] JOINT_BACK 완료 → MOVE_TO_HOME (다음 사이클)")

    # ── 유틸 ─────────────────────────────────────────────────────────

    @staticmethod
    def _find_prim(root_path, name):
        stage = omni.usd.get_context().get_stage()
        root  = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return None
        for p in Usd.PrimRange(root):
            if p.GetName() == name:
                return str(p.GetPath())
        return None


# ──────────────────────────────────────────────────────────────────────
#  엔트리포인트
# ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # M0609_SRC_DIR를 sys.path에 추가 (RMPFlow 컨트롤러 임포트)
    _SRC = "/home/rokey/dev_ws/main_isaac/robots/m0609/m0609_aruco_detect"
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1 / 500,
        rendering_dt=1 / 50,
    )

    agent = PlaceTestAgent(world)
    agent.setup()

    print("[Test] 씬 로드 중...")
    for _ in range(300):
        omni.kit.app.get_app().update()

    world.reset()
    for _ in range(30):
        omni.kit.app.get_app().update()

    agent.post_reset()

    world.add_physics_callback("place_test", agent.on_physics_step)

    print("[Test] 시뮬레이션 시작")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    finally:
        world.clear()
        simulation_app.close()
        print("[Test] 종료")
