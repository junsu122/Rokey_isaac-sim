from isaacsim import SimulationApp

# 1. 시뮬레이션 앱 초기화 (화면을 보며 테스트하기 위해 headless=False 설정)
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
# 🌟 쿼터니언 변환을 위한 SciPy 라이브러리 임포트
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf

# ================================================================================
# 🐕 [구조 수정] 월드 리셋(Reset) 이후 안전하게 카메라를 붙이도록 설계한 클래스
# ================================================================================

class CustomSpot(SpotFlatTerrainPolicy):
    def __init__(self, prim_path, name, position=None, orientation=None):
        # 부모 클래스(NVIDIA Spot)의 기본 생성자를 먼저 호출하여 로봇을 띄웁니다.
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation)
        self.camera_prim_path = f"{prim_path}/body/spot_head_camera"

    def attach_head_camera(self):
        """ 🌟 [최종 수정] 빈 카메라 틀에 렌더링에 필요한 필수 USD 속성을 주입합니다. """
        # 1. body 링크 하위에 카메라 프림 생성
        define_prim(self.camera_prim_path, "Camera")
        
        from pxr import UsdGeom, Sdf, Usd
        stage = World.instance().stage
        camera_prim_obj = stage.GetPrimAtPath(self.camera_prim_path)
        
        # ----------------------------------------------------------------------------
        # 🎯 [핵심 추가] 카메라이 정상 작동(렌더링)하기 위한 필수 USD 스키마 속성 주입
        # ----------------------------------------------------------------------------
        # 아이작 심 렌더러가 이 프림을 '진짜 카메라 센서'로 인식하도록 타입을 명시합니다.
        camera_prim_obj.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token).Set("perspective")
        
        # 렌즈 속성 설정 (화각 및 초점 거리 지정 - 안 넣어주면 0이 되어 검게 보일 수 있음)
        camera_prim_obj.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(50.0)
        camera_prim_obj.CreateAttribute("horizontalAperture", Sdf.ValueTypeNames.Float).Set(20.955)
        camera_prim_obj.CreateAttribute("verticalAperture", Sdf.ValueTypeNames.Float).Set(15.222)
        camera_prim_obj.CreateAttribute("clippingRange", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.01, 10000.0))
        
        # 2. 변환 매트릭스 설정 (오일러 좌표계)
        xformable = UsdGeom.Xformable(camera_prim_obj)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        rotate_op = xformable.AddRotateXYZOp()
        
        # 스팟 머리 겉면에 위치 고정 (준수님이 확인하신 그 위치)
        translate_op.Set(Gf.Vec3d(0.4, 0.0, -0.01))
        rotate_op.Set(Gf.Vec3f(90.0, 0.0, -90.0))  # 정면 응시
        
        # 가이드라인 항상 보기
        camera_prim_obj.CreateAttribute("guideVisibility", Sdf.ValueTypeNames.Token).Set("always")
        
        print(f"✅ [카메라 렌더링 속성 주입 완료]: {self.camera_prim_path}")

    def get_camera_path(self):
        return self.camera_prim_path

# ================================================================================
# ⚙️ 시뮬레이션 앱 환경 구성 및 콜백 등록
# ================================================================================

first_step = True
reset_needed = False

# 물리 엔진 스텝 콜백 함수
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        spot.initialize()
        print("Policy 관절 순서:", spot.robot._articulation_view.dof_names)
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        spot.forward(step_size, base_command)

# 시뮬레이션 월드 세팅 (500Hz 물리, 50Hz 렌더링)
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# 기본 그리드 맵 환경 로드
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# 🐕 카메라 기반 커스텀 스팟 인스턴스 선언
spot = CustomSpot(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([0, 0, 0.8]),
)

# 🚨 [치트키] 월드를 완전히 초기화해서 로봇 구조를 고정시킨 다음에 카메라 수술을 진행합니다!
my_world.reset()
spot.attach_head_camera()  # 🌟 리셋 직후에 좌표를 주입해야 덮어써 지지 않습니다.

my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)
spot_cam_path = spot.get_camera_path()

# ================================================================================
# 🎯 글로벌 패스 및 제어 변수 설정
# ================================================================================
base_command = np.zeros(3)
current_state = "STABILIZE" 

Kp = 1.2   
Ki = 0.05  
Kd = 0.1   

dt = 1 / 50  
stabilize_timer = 0

global_path = [
    np.array([5.0, 0.0]),   
    np.array([5.0, 5.0]),   
    np.array([0.0, 5.0]),   
    np.array([0.0, 0.0])    
]
target_waypoint_idx = 0     
look_ahead_distance = 0.45  
target_speed = 0.8         

print("🚀 Pure Pursuit 기반 글로벌 패스 추종 무한 사각형 주행을 시작합니다!")

# ================================================================================
# 🕹️ 메인 시뮬레이션 루프
# ================================================================================
while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_stopped():
        reset_needed = True
        current_state = "STABILIZE" 
        stabilize_timer = 0
        target_waypoint_idx = 0
        
    if my_world.is_playing():
        current_pos, current_quat = spot.robot.get_world_pose()
        
        quat_xyzw = [current_quat[1], current_quat[2], current_quat[3], current_quat[0]]
        r = R.from_quat(quat_xyzw)
        current_yaw = r.as_euler('xyz')[2]
        
        # ----------------------------------------------------------------------------
        # 🤖 상태 머신 및 경로 추종 제어 시퀀스
        # ----------------------------------------------------------------------------
        if current_state == "STABILIZE":
            base_command = np.array([0.0, 0.0, 0.0])
            if stabilize_timer >= 150:  
                current_state = "TRACKING"
                print("✅ [안정화 완료] 글로벌 패스 Pure Pursuit 주행을 시작합니다.")
            else:
                stabilize_timer += 1
                
        elif current_state == "TRACKING":
            target_pt = global_path[target_waypoint_idx]
            distance_to_target = np.linalg.norm(current_pos[:2] - target_pt)
            
            if distance_to_target < look_ahead_distance:
                target_waypoint_idx = (target_waypoint_idx + 1) % len(global_path)
                target_pt = global_path[target_waypoint_idx]
                print(f"🎯 [Waypoint 갱신] {target_waypoint_idx + 1}번째 목표점({target_pt[0]}, {target_pt[1]})으로 전환합니다.")
            
            target_yaw = np.arctan2(target_pt[1] - current_pos[1], target_pt[0] - current_pos[0])
            
            yaw_error = target_yaw - current_yaw
            if yaw_error > np.pi: yaw_error -= 2 * np.pi
            elif yaw_error < -np.pi: yaw_error += 2 * np.pi
            
            turn_speed = Kp * yaw_error
            turn_speed = np.clip(turn_speed, -0.6, 0.6)
            
            if abs(yaw_error) > (35.0 * np.pi / 180.0):
                base_command = np.array([0.05, 0.0, turn_speed])
            else:
                base_command = np.array([target_speed, 0.0, turn_speed])

simulation_app.close()