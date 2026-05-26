from isaacsim import SimulationApp

# 1. 시뮬레이션 앱 초기화
simulation_app = SimulationApp({"headless": False, "exts": ["omni.isaac.ros2_bridge", "omni.isaac.core_nodes"]})

import carb
import time
import sys
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from scipy.spatial.transform import Rotation as R
from pxr import Gf, UsdGeom, Sdf, UsdPhysics

# Isaac Sim 5.X 오피셜 고수준 센서 자산 패키지
from isaacsim.sensors.camera import SingleViewDepthSensorAsset

# ================================================================================
# 🐕 [센서 데이터 무결성 보정] 물리 및 버퍼 필터링이 강화된 커스텀 스팟 클래스
# ================================================================================
class CustomSpot(SpotFlatTerrainPolicy):
    def __init__(self, prim_path, name, position=None, orientation=None):
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation)
        
        self.realsense_hardware_path = "/World/Intel_RealSense_D455"
        self.realsense_asset = None
        self.render_product_path = None
        self.log_timer = 0 # 디버깅 로그용 타이머

    def attach_realsense_ros2_streams(self):
        """ 월드 최상위에 독립적으로 리얼센스를 생성하고 ROS2 팩토리를 강제 동기화합니다. """
        assets_root_path = get_assets_root_path()
        stage = World.instance().stage
        
        # 1. 독립된 경로에 리얼센스 D455 자산 빌드
        asset_path = assets_root_path + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        self.realsense_asset = SingleViewDepthSensorAsset(
            prim_path=self.realsense_hardware_path, 
            asset_path=asset_path
        )
        self.realsense_asset.initialize()
        
        import omni.kit.app
        for _ in range(20):
            omni.kit.app.get_app().update()
        time.sleep(0.5)
        
        # 2. 📡 [ROS2 토픽 빌드]
        try:
            import omni.replicator.core as rep
            from omni.isaac.core_nodes.scripts.utils import set_targets
            
            if hasattr(self.realsense_asset, "_render_product_path"):
                render_product_path = self.realsense_asset._render_product_path
            else:
                render_product_path = self.realsense_asset.render_product_path
            
            # 📸 [image_raw]
            rgb_pub_node = rep.utils.create_node(
                node_type_id="omni.isaac.ros2_bridge.ROS2PublishImage",
                attributes={
                    "inputs:topicName": "/camera/image_raw",
                    "inputs:frameId": "realsense_lens_frame"
                }
            )
            set_targets(node=rgb_pub_node, attribute="inputs:renderProductPath", targets=render_product_path)
            
            # 🛰️ [point_cloud]
            pc_pub_node = rep.utils.create_node(
                node_type_id="omni.isaac.ros2_bridge.ROS2PublishPointCloud",
                attributes={
                    "inputs:topicName": "/camera/point_cloud",
                    "inputs:frameId": "realsense_lens_frame"
                }
            )
            set_targets(node=pc_pub_node, attribute="inputs:renderProductPath", targets=render_product_path)
            
            self.render_product_path = render_product_path
            print("📡 [ROS2 Core 연결 대성공] 5.X 토픽 허브 개설 완료!")
            
        except Exception as e:
            print(f"⚠️ ROS2 자산 결합 중 에러 발생: {e}")

    def update_camera_pose(self):
        """ 매 물리 프레임마다 스팟 이마 위치로 카메라 포즈 동기화 """
        stage = World.instance().stage
        camera_prim = stage.GetPrimAtPath(self.realsense_hardware_path)
        if not camera_prim.IsValid():
            return
            
        body_pos, body_quat = self.robot.get_world_pose()
        quat_xyzw = [body_quat[1], body_quat[2], body_quat[3], body_quat[0]]
        r = R.from_quat(quat_xyzw)
        
        local_offset = np.array([0.45, 0.0, 0.08])
        rotated_offset = r.apply(local_offset)
        target_camera_pos = body_pos + rotated_offset
        
        xformable = UsdGeom.Xformable(camera_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(float(target_camera_pos[0]), float(target_camera_pos[1]), float(target_camera_pos[2])))
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(np.degrees(r.as_euler('xyz')[2]))))

    def get_front_depth_data(self):
        """ 👁️ 알고리즘 제어용 3D Depth 데이터 긁어오기 """
        try:
            path_target = getattr(self, "render_product_path", None)
            if path_target is None:
                return None
            import omni.syntheticdata.sensors as sensors
            return sensors.get_depth_linear(path_target)
        except Exception:
            return None

# ================================================================================
# ⚙️ 시뮬레이션 앱 환경 구성 및 콜백 등록
# ================================================================================
first_step = True
reset_needed = False

def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        spot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        spot.update_camera_pose()
        spot.forward(step_size, base_command)

my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)
assets_root_path = get_assets_root_path()

# 기본 배경 빌드
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# 조명
light_prim_path = "/World/DistantLight"
light_prim = define_prim(light_prim_path, "DistantLight")
light_prim.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
light_xform = UsdGeom.Xformable(light_prim)
light_xform.ClearXformOpOrder()
light_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

# ================================================================================
# 📐 [장애물 콘 생성 및 물리 충돌체 강제 주입 세션]
# ================================================================================
obstacle_prim_path = "/World/TestObstacleCone"
obstacle_prim = define_prim(obstacle_prim_path, "Cone")
obstacle_xform = UsdGeom.Xformable(obstacle_prim)
obstacle_xform.ClearXformOpOrder()
obs_translate_op = obstacle_xform.AddTranslateOp()
obs_scale_op = obstacle_xform.AddScaleOp()
obs_translate_op.Set(Gf.Vec3d(2.3, 0.0, 0.15)) # 거리를 살짝 당겨 정렬
obs_scale_op.Set(Gf.Vec3f(0.25, 0.25, 0.45)) # 크기를 살짝 키워 인지율 상향
obstacle_prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(1.0, 0.35, 0.0)])

# 🔥 [핵심 패치] 콘 에셋에 센서가 반사될 Collider 물리 성질을 명시적으로 부여합니다.
stage = World.instance().stage
cone_prim_obj = stage.GetPrimAtPath(obstacle_prim_path)
UsdPhysics.CollisionAPI.Apply(cone_prim_obj) # 충돌 겉표면 락 바인딩

# 로봇 생성 및 마운트
spot = CustomSpot(prim_path="/World/Spot", name="Spot", position=np.array([0, 0, 0.8]))
spot.attach_realsense_ros2_streams() 
my_world.reset()

my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# ================================================================================
# 🎯 글로벌 패스 및 제어 변수 설정
# ================================================================================
base_command = np.zeros(3)
current_state = "STABILIZE" 

Kp = 1.2   
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
target_speed = 0.7         

obstacle_detected = False
obstacle_avoid_timer = 0
AVOID_DURATION = 75 

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
        obstacle_detected = False
        obstacle_avoid_timer = 0
        
    if my_world.is_playing():
        current_pos, current_quat = spot.robot.get_world_pose()
        
        quat_xyzw = [current_quat[1], current_quat[2], current_quat[3], current_quat[0]]
        r = R.from_quat(quat_xyzw)
        current_yaw = r.as_euler('xyz')[2]
        
        if current_state == "STABILIZE":
            base_command = np.array([0.0, 0.0, 0.0])
            if stabilize_timer >= 150:  
                current_state = "TRACKING"
                print("✅ [안정화 완료] RealSense D455 데이터 모니터링 주행을 가동합니다.")
            else:
                stabilize_timer += 1
                
        elif current_state == "TRACKING":
            if not obstacle_detected:
                depth_matrix = spot.get_front_depth_data()
                
                if (depth_matrix is not None) and (hasattr(depth_matrix, "shape")) and (len(depth_matrix.shape) == 2):
                    h, w = depth_matrix.shape
                    ch, cw = h // 2, w // 2
                    
                    # 화각 중앙 윈도우 스캔 구역 추출
                    center_depth_zone = depth_matrix[ch-40:ch+40, cw-40:cw+40]
                    
                    # 🌟 [보정 필터] 무한대(inf), NaN 값을 완벽히 소거하고 유효 미터 단위 거리만 발라냅니다.
                    valid_depths = center_depth_zone[
                        np.isfinite(center_depth_zone) & 
                        (center_depth_zone > 0.05) & 
                        (center_depth_zone < 10.0)
                    ]
                    
                    # 실시간 모니터링용 디버깅 프린트 (50프레임마다 터미널에 실측 거리를 찍어줍니다.)
                    spot.log_timer += 1
                    if spot.log_timer % 50 == 0:
                        if len(valid_depths) > 0:
                            print(f"📊 [정면 센서 피드] 포착된 물체 최소 실측 거리: {np.min(valid_depths):.2f}m (유효 포인트: {len(valid_depths)}개)")
                        else:
                            print("📊 [정면 센서 피드] 뻥 뚫림 (시야 내 장애물 없음)")
                    
                    if len(valid_depths) > 0:
                        min_distance = np.min(valid_depths)
                        # 콘에 충돌체를 입혔으므로 스팟이 접근할 때 미터 값이 정밀하게 좁혀집니다.
                        if min_distance < 1.7:
                            obstacle_detected = True
                            obstacle_avoid_timer = AVOID_DURATION
                            print(f"\n⚠️ [장애물 확정 감지!] 전방 {min_distance:.2f}m 지점 콘 충돌체 인지 -> 우회 기동 수행\n")

            if obstacle_detected:
                obstacle_avoid_timer -= 1
                base_command = np.array([0.25, 0.0, 0.65]) # 우회 기동 선회 속도 상향
                if obstacle_avoid_timer <= 0:
                    obstacle_detected = False
                    print("✅ [회피 완료] 경로로 재진입합니다.")
            else:
                # 패스 추종 제어 로직
                target_pt = global_path[target_waypoint_idx]
                distance_to_target = np.linalg.norm(current_pos[:2] - target_pt)
                if distance_to_target < look_ahead_distance:
                    target_waypoint_idx = (target_waypoint_idx + 1) % len(global_path)
                    target_pt = global_path[target_waypoint_idx]
                
                target_yaw = np.arctan2(target_pt[1] - current_pos[1], target_pt[0] - current_pos[0])
                yaw_error = target_yaw - current_yaw
                if yaw_error > np.pi: yaw_error -= 2 * np.pi
                elif yaw_error < -np.pi: yaw_error += 2 * np.pi
                
                turn_speed = np.clip(Kp * yaw_error, -0.6, 0.6)
                if abs(yaw_error) > (35.0 * np.pi / 180.0):
                    base_command = np.array([0.05, 0.0, turn_speed])
                else:
                    base_command = np.array([target_speed, 0.0, turn_speed])

print("🏁 시뮬레이션 종료.")
my_world.clear() 
simulation_app.close()