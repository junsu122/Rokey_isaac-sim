"""
test_m0609_place.py
===================
M0609 Pick & Place 테스트 스크립트.
m0609_agent.py 와 동일한 로봇 로드 방식 (흡착 그리퍼 포함, 카메라 제외).

실제 물리 도달 범위 기준 좌표 설계:
    로봇 베이스: (-13.5, 6.8, 0.0)  yaw=-90°  scale=2.0
    실측 EE @ HOME: (-13.488, 6.064, 0.848)
    실측 최대 도달: ~1.55 m

동작 순서:
    1. HOME
    2. APPROACH_PICK  (RMPFlow → 큐브 위 안전 높이)
    3. MOVEL_PICK     (위에서 아래로 → 큐브 픽업)
    4. LIFT           (RMPFlow → 큐브 위 안전 높이)
    5. APPROACH_PLACE (RMPFlow → 선반 위 안전 높이)
    6. MOVEL_PLACE    (위에서 아래로 → 선반)
    7. RETRACT        (MOVEL → 선반 위로 후퇴)
    8. HOME → 반복
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

# ── robot_config.py (main_isaac 와 동일한 경로 공유) ─────────────────
_MAIN_ISAAC = "/home/rokey/Rokey_isaac-sim/main_isaac"
if _MAIN_ISAAC not in sys.path:
    sys.path.insert(0, _MAIN_ISAAC)
import robot_config as C

if C.M0609_SRC_DIR not in sys.path:
    sys.path.insert(0, C.M0609_SRC_DIR)

# ──────────────────────────────────────────────────────────────────────
#  ★ 수정 가능한 파라미터
# ──────────────────────────────────────────────────────────────────────

# 파일 경로
WAREHOUSE_USD = C.WAREHOUSE_USD
POD_USD       = "/home/rokey/Rokey_isaac-sim/main_isaac/usd/pod_stack_4_v2.usda"
ROBOT_URDF    = C.M0609_URDF
RMPFLOW_CFG   = C.M0609_RMPFLOW_CFG
DESC_YAML     = C.M0609_DESC_YAML

# 스폰
POD_XYZ       = (-12.0, 7.5, 0.0)
ROBOT_XYZ     = (-13.2, 6.8, 0.0)
ROBOT_YAW_DEG = -90.0
ROBOT_SCALE   = 2.0

# ── 큐브 ─────────────────────────────────────────────────────────────
# 로봇 베이스(-13.5, 6.8, 0)에서 ~1.33m → 실측 도달 범위 내
CUBE_XYZ   = np.array([-12.5, 7.0, 0.82])
CUBE_SCALE = np.array([0.12,  0.12,  0.12])

# ── 홈 관절각 ────────────────────────────────────────────────────────
HOME_DEG = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

# ── 픽업 (위에서 아래로) ─────────────────────────────────────────────
#   PICK_ABOVE: 큐브 정상부 위  (~1.57m, 최대 도달 범위 내)
#   PICK_XYZ  : 큐브 상단       (~1.33m)
PICK_ABOVE_XYZ = np.array([-12.5, 7.0, 1.30])
PICK_XYZ       = np.array([-12.5, 7.0, 0.87])

# ── 플레이스 (위에서 아래로) ─────────────────────────────────────────
#   PLACE_ABOVE: 선반 위 안전 높이  (~1.5m)
#   PLACE_XY   : 선반 XY 위치
#   PLACE_Z_LIST: 3단 선반 Z (낮은 → 중간 → 높은)
PLACE_ABOVE_XYZ = np.array([-12.5, 7.3, 1.10])
PLACE_XY        = np.array([-12.5, 7.3])
PLACE_Z_LIST    = [0.45, 0.75, 1.05]

# ── 제어 상수 ────────────────────────────────────────────────────────
APPROACH_TOL  = 0.10   # RMPFlow 목표 도달 판정 거리 [m]
ATTACH_REACH  = 0.22   # 흡착 임계 거리 [m]  (EE → 큐브 중심)
MOVEL_STEPS   = 60     # MOVEL 보간 스텝
JOINT_TOL_DEG = 2.0    # 관절각 수렴 허용 오차 [deg]

# ══════════════════════════════════════════════════════════════════════
#  흡착 그리퍼 형상 (m0609_agent.py 와 동일)
# ══════════════════════════════════════════════════════════════════════
_SUCTION_STEM_RADIUS  = 0.022
_SUCTION_STEM_HEIGHT  = 0.060
_SUCTION_PAD_RADIUS   = 0.045
_SUCTION_PAD_HEIGHT   = 0.012
_SUCTION_RIM_RADIUS   = 0.048
_SUCTION_RIM_HEIGHT   = 0.004
_SUCTION_MOUNT_OFFSET = (0.0, 0.0, 0.0)
_SUCTION_COLOR_BODY   = Gf.Vec3f(0.30, 0.30, 0.30)
_SUCTION_COLOR_PAD    = Gf.Vec3f(0.10, 0.10, 0.10)
_SUCTION_COLOR_RIM    = Gf.Vec3f(0.05, 0.05, 0.05)

_EE_LINK     = "link_6"
_HOME_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


# ──────────────────────────────────────────────────────────────────────
#  더미 그리퍼
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


def _find_joint_index(robot, jname: str, fallback: int = 0) -> int:
    for i, n in enumerate(robot.dof_names):
        if n == jname or n.endswith(jname):
            return i
    return fallback


# ──────────────────────────────────────────────────────────────────────
#  메인 에이전트
# ──────────────────────────────────────────────────────────────────────

class PlaceTestAgent:
    def __init__(self, world: World):
        self.world = world
        self._robot        = None
        self._cube         = None
        self._state        = "INIT"
        self._suction_path = None

        self._gripped           = False
        self._grab_offset_local = None
        self._movel_wps         = None
        self._movel_ori         = None
        self._movel_step        = 0
        self._cycle_idx         = 0
        self._phys_cnt          = 0
        self._home_pos          = None
        self._home_idx          = None
        self._cspace            = None

    # ── 씬 구성 ──────────────────────────────────────────────────────

    def setup(self):
        stage = omni.usd.get_context().get_stage()

        warehouse = define_prim("/World/Warehouse", "Xform")
        warehouse.GetReferences().AddReference(WAREHOUSE_USD)
        print(f"[Test] 창고 맵 로드: {WAREHOUSE_USD}")

        pod_prim = define_prim("/World/Pod", "Xform")
        pod_prim.GetReferences().AddReference(POD_USD)
        xf = UsdGeom.Xformable(pod_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*POD_XYZ))
        print(f"[Test] Pod 로드: {POD_XYZ}")

        # ── URDF import (m0609_agent.py 완전 동일) ───────────────────
        _, import_cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        import_cfg.merge_fixed_joints             = False
        import_cfg.convex_decomp                  = True
        import_cfg.import_inertia_tensor          = True
        import_cfg.fix_base                       = True
        import_cfg.distance_scale                 = 1.0
        import_cfg.default_drive_type             = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_cfg.default_drive_strength         = 1e10
        import_cfg.default_position_drive_damping = 1e5

        _, artic_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=ROBOT_URDF,
            import_config=import_cfg,
            get_articulation_root=True,
        )
        if artic_path is None:
            raise RuntimeError(f"URDF import 실패: {ROBOT_URDF}")
        robot_root = artic_path.rsplit("/", 1)[0] or artic_path

        target_root = "/World/m0609_test"
        if robot_root != target_root:
            omni.kit.commands.execute("MovePrim",
                                      path_from=robot_root,
                                      path_to=target_root)
            artic_path = target_root + artic_path[len(robot_root):]
            robot_root = target_root
        self._robot_root = robot_root

        root_prim = stage.GetPrimAtPath(robot_root)
        xf = UsdGeom.Xformable(root_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*ROBOT_XYZ))
        if abs(ROBOT_YAW_DEG) > 1e-6:
            xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(ROBOT_YAW_DEG)
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(ROBOT_SCALE, ROBOT_SCALE, ROBOT_SCALE))

        robot_ee = self._find_prim(robot_root, _EE_LINK) or f"{artic_path}/{_EE_LINK}"

        for _ in range(10):
            omni.kit.app.get_app().update()

        self._suction_path = self._build_suction_gripper(stage, robot_ee)
        print(f"[Test] 흡착 그리퍼 생성: {self._suction_path}")

        gripper = NoOpGripper(robot_ee)
        self._robot = self.world.scene.add(
            SingleManipulator(
                prim_path=artic_path,
                name="m0609_test",
                end_effector_prim_path=robot_ee,
                gripper=gripper,
            )
        )

        # ── 픽업 받침대 (정적 충돌체) ──────────────────────────────────
        # 큐브가 낙하하지 않도록 CUBE_XYZ 아래에 정적 박스를 배치.
        # 큐브 bottom = CUBE_XYZ[2] - CUBE_SCALE[2]/2
        _table_top = float(CUBE_XYZ[2] - CUBE_SCALE[2] / 2.0)   # 0.76 m
        _table_prim = define_prim("/World/PickTable", "Cube")
        UsdGeom.Cube(_table_prim).CreateSizeAttr(1.0)
        _xf = UsdGeom.Xformable(_table_prim)
        _xf.ClearXformOpOrder()
        _xf.AddTranslateOp().Set(Gf.Vec3d(
            float(CUBE_XYZ[0]), float(CUBE_XYZ[1]), _table_top / 2.0))
        _xf.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, _table_top))
        UsdPhysics.CollisionAPI.Apply(_table_prim)
        _table_prim.CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
        ).Set([Gf.Vec3f(0.55, 0.40, 0.25)])
        print(f"[Test] 픽업 받침대 생성 (top z={_table_top:.3f} m)")

        # ── 큐브 (setup 시 1회만 생성) ─────────────────────────────────
        self._spawn_cube()
        print("[Test] setup 완료")

    # ── 흡착 그리퍼 (m0609_agent.py 동일, 카메라 제외) ──────────────

    def _build_suction_gripper(self, stage, ee_path: str) -> str:
        root_path = f"{ee_path}/suction_gripper"
        root_xf = UsdGeom.Xform.Define(stage, root_path)
        xf = UsdGeom.Xformable(root_xf.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*_SUCTION_MOUNT_OFFSET))

        def _cyl(name, radius, height, z_center, color):
            p = f"{root_path}/{name}"
            cyl = UsdGeom.Cylinder.Define(stage, p)
            cyl.CreateRadiusAttr(float(radius))
            cyl.CreateHeightAttr(float(height))
            cyl.CreateAxisAttr("Z")
            c_xf = UsdGeom.Xformable(cyl.GetPrim())
            c_xf.ClearXformOpOrder()
            c_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(z_center)))
            cyl.GetPrim().CreateAttribute(
                "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray,
            ).Set([color])

        _cyl("stem", _SUCTION_STEM_RADIUS, _SUCTION_STEM_HEIGHT,
             _SUCTION_STEM_HEIGHT / 2.0, _SUCTION_COLOR_BODY)
        pad_z = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT / 2.0
        _cyl("pad",  _SUCTION_PAD_RADIUS,  _SUCTION_PAD_HEIGHT,
             pad_z, _SUCTION_COLOR_PAD)
        rim_z = _SUCTION_STEM_HEIGHT + _SUCTION_PAD_HEIGHT - _SUCTION_RIM_HEIGHT / 2.0
        _cyl("rim",  _SUCTION_RIM_RADIUS,  _SUCTION_RIM_HEIGHT,
             rim_z, _SUCTION_COLOR_RIM)
        return root_path

    def _disable_suction_physics(self):
        if not self._suction_path:
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

    def _spawn_cube(self):
        """최초 1회만 씬에 추가. 이후 사이클엔 위치만 리셋."""
        if self._cube is not None:
            # world.reset() 이후엔 scene.add() 불가 → 위치·속도만 리셋
            self._cube.set_world_pose(position=CUBE_XYZ.copy())
            self._cube.set_linear_velocity(np.zeros(3))
            self._cube.set_angular_velocity(np.zeros(3))
            print(f"[Test] 큐브 위치 리셋 at {CUBE_XYZ}")
            return

        cube = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/TestCube",
                name="test_cube",
                position=CUBE_XYZ.copy(),
                scale=CUBE_SCALE,
                color=np.array([0.8, 0.4, 0.1]),
                mass=0.05,
            )
        )
        self._cube = cube
        print(f"[Test] 큐브 생성 at {CUBE_XYZ}")

    # ── post_reset ───────────────────────────────────────────────────

    def post_reset(self):
        self._disable_suction_physics()

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

        from m0609_rmpflow_controller import RMPFlowController
        self._cspace = RMPFlowController(
            name="test_rmpflow",
            robot_articulation=self._robot,
            urdf_path=ROBOT_URDF,
            robot_description_path=DESC_YAML,
            rmpflow_config_path=RMPFLOW_CFG,
            end_effector_frame_name=_EE_LINK,
        )

        self._home_idx = np.array([
            _find_joint_index(self._robot, jn, i)
            for i, jn in enumerate(_HOME_JOINTS)
        ])
        self._home_pos = np.deg2rad(HOME_DEG)
        self._robot.set_joint_positions(self._home_pos, joint_indices=self._home_idx)

        self._state    = "MOVE_TO_HOME"
        self._phys_cnt = 0
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

    def _start_movel(self, start_pos, end_pos, ori=None, steps=MOVEL_STEPS):
        self._movel_wps  = np.linspace(start_pos, end_pos, steps)
        self._movel_ori  = ori
        self._movel_step = 0

    def _step_movel(self) -> bool:
        if self._movel_step >= len(self._movel_wps):
            return True
        self._apply_ee(self._movel_wps[self._movel_step], self._movel_ori)
        self._movel_step += 1
        return self._movel_step >= len(self._movel_wps)

    def _set_joints(self, deg_array):
        self._robot.set_joint_positions(
            np.deg2rad(deg_array), joint_indices=self._home_idx)

    def _joint_err(self, deg_array) -> float:
        joints = self._robot.get_joint_positions()
        return float(np.max(np.abs(
            joints[self._home_idx] - np.deg2rad(deg_array))))

    @property
    def _place_xyz(self) -> np.ndarray:
        z = PLACE_Z_LIST[self._cycle_idx % len(PLACE_Z_LIST)]
        return np.array([PLACE_XY[0], PLACE_XY[1], z])

    # ── FSM ──────────────────────────────────────────────────────────

    def _run_fsm(self):
        ee_pos, ee_ori = self._robot.end_effector.get_world_pose()
        tol = np.deg2rad(JOINT_TOL_DEG)

        # ── MOVE_TO_HOME ─────────────────────────────────────────────
        if self._state == "MOVE_TO_HOME":
            self._set_joints(HOME_DEG)
            if self._joint_err(HOME_DEG) < tol:
                self._state = "APPROACH_PICK"
                print(f"[Test] HOME → APPROACH_PICK  cycle={self._cycle_idx}")

        # ── APPROACH_PICK: EE → 큐브 위 안전 높이 ────────────────────
        elif self._state == "APPROACH_PICK":
            self._apply_ee(PICK_ABOVE_XYZ)
            if np.linalg.norm(ee_pos - PICK_ABOVE_XYZ) < APPROACH_TOL:
                self._start_movel(ee_pos, PICK_XYZ, ori=ee_ori)
                self._state = "MOVEL_PICK"
                print("[Test] APPROACH_PICK 도달 → MOVEL_PICK")

        # ── MOVEL_PICK: 위에서 아래로 큐브까지 ───────────────────────
        elif self._state == "MOVEL_PICK":
            done = self._step_movel()
            if done:
                cube_pos, _ = self._cube.get_world_pose()
                dist = np.linalg.norm(ee_pos - cube_pos)
                if dist < ATTACH_REACH:
                    self._attach()
                else:
                    print(f"[Test] 흡착 실패 (EE-큐브 거리={dist:.3f}m > {ATTACH_REACH}m)")
                self._state = "LIFT_FROM_PICK"
                print("[Test] MOVEL_PICK 완료 → LIFT_FROM_PICK")

        # ── LIFT_FROM_PICK: 픽 후 위로 ───────────────────────────────
        elif self._state == "LIFT_FROM_PICK":
            self._apply_ee(PICK_ABOVE_XYZ)
            if np.linalg.norm(ee_pos - PICK_ABOVE_XYZ) < APPROACH_TOL:
                self._state = "APPROACH_PLACE"
                print("[Test] LIFT 완료 → APPROACH_PLACE")

        # ── APPROACH_PLACE: 선반 위 안전 높이로 이동 ─────────────────
        elif self._state == "APPROACH_PLACE":
            self._apply_ee(PLACE_ABOVE_XYZ)
            if np.linalg.norm(ee_pos - PLACE_ABOVE_XYZ) < APPROACH_TOL:
                place_xyz = self._place_xyz
                self._start_movel(ee_pos, place_xyz, ori=ee_ori)
                self._state = "MOVEL_PLACE"
                print(f"[Test] APPROACH_PLACE 도달 → MOVEL_PLACE  z={place_xyz[2]}")

        # ── MOVEL_PLACE: 위에서 아래로 선반까지 ─────────────────────
        elif self._state == "MOVEL_PLACE":
            done = self._step_movel()
            if done:
                self._detach()
                self._cycle_idx += 1
                self._spawn_cube()   # 위치 리셋만 (재생성 아님)
                self._start_movel(ee_pos, PLACE_ABOVE_XYZ, ori=ee_ori)
                self._state = "MOVEL_RETRACT"
                print(f"[Test] PLACE 완료 (cycle={self._cycle_idx}) → MOVEL_RETRACT")

        # ── MOVEL_RETRACT: 선반 위로 후퇴 ────────────────────────────
        elif self._state == "MOVEL_RETRACT":
            done = self._step_movel()
            if done:
                self._state = "MOVE_TO_HOME"
                print("[Test] RETRACT 완료 → MOVE_TO_HOME")

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
    agent.post_reset()

    # warmup 전 콜백 등록 → warmup 중 FSM이 HOME 위치 유지
    world.add_physics_callback("place_test", agent.on_physics_step)

    print("[Test] Warmup 중...")
    for _ in range(150):
        world.step(render=False)

    print("[Test] 시뮬레이션 시작")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    finally:
        world.clear()
        simulation_app.close()
        print("[Test] 종료")
