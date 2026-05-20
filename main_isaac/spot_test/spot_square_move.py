from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
# 🌟 쿼터니언을 오일러 각도로 변환하기 위한 툴 임포트
from scipy.spatial.transform import Rotation as R

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
        spot.forward(step_size, base_command)

my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 500, rendering_dt=1 / 50)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([0, 0, 0.8]),
)
my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# ================================================================================
# 🎯 PID 제어 게인 및 변수 설정 (3초 안정화 추가 버전)
# ================================================================================
base_command = np.zeros(3)

# 💡 상태 머신에 "STABILIZE" 상태를 첫 단계로 추가합니다.
current_state = "STABILIZE" 

start_position = None
target_yaw = 0.0

# PID 게인 설정
Kp = 1.2   
Ki = 0.05  
Kd = 0.1   

# PID 계산용 변수
prev_yaw_error = 0.0
raw_yaw_error_integral = 0.0
dt = 1 / 50  

# ⏱️ 안정화 타이머용 변수 (50Hz 환경에서 3초 = 150 스텝) // 회전 바퀴 수 체크
stabilize_timer = 0
side_counter = 0                    # 💡 현재 사각형의 몇 번째 변을 돌고 있는지 카운트 (0~3)
is_correcting_printed = False  
initial_yaw = None                  # 처음 출발하는 방향값 저장 // 추후 원점 복귀했을때 비교용

print("🚀 3초 안정화 후 PID 사각형 주행 미션을 시작합니다!")

while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_stopped():
        reset_needed = True
        current_state = "STABILIZE" # 리셋 시 안정화 단계로 복귀
        stabilize_timer = 0
        side_counter = 0
        raw_yaw_error_integral = 0.0
        prev_yaw_error = 0.0
        initial_yaw = None # 리셋 시 초기화
        
    if my_world.is_playing():
        current_pos, current_quat = spot.robot.get_world_pose()
        
        # SciPy 기반 현재 Yaw 각도 계산
        quat_xyzw = [current_quat[1], current_quat[2], current_quat[3], current_quat[0]]
        r = R.from_quat(quat_xyzw)
        current_yaw = r.as_euler('xyz')[2]
        
        # ================================================================================
        # 🕹️ 상태 머신 (State Machine)
        # ================================================================================
        
        # [0단계] ⏱️ 3초간 제자리 안정화 구간 (요청하신 기능)
        if current_state == "STABILIZE":
            base_command = np.array([0.0, 0.0, 0.0]) # 아무 명령도 주지 않음
            
            if stabilize_timer == 0:
                print("⏳ [안정화] 로봇이 자세를 잡을 수 있도록 3초간 대기합니다...")
                
            if stabilize_timer >= 150: # 150 스텝 = 3초 도달 시
                current_state = "INIT"
                print("✅ [안정화 완료] 사각형 주행 시스템을 가동합니다.")

            else:
                stabilize_timer += 1
        
        # [1단계] 주행 기준점 초기화
        elif current_state == "INIT":
            start_position = np.copy(current_pos)

            # 🌟 최초 1단계(첫 바퀴 시작할 때)의 절대 방향을 영구 저장합니다.
            if initial_yaw is None:
                initial_yaw = current_yaw
                print(f"📸 [방향 기억] 최초 시작 방향({initial_yaw * 180 / np.pi:.1f}도)을 기억했습니다.")

            current_state = "FORWARD"
            print("▶️ [전진] 2미터 전진을 시작합니다.")
            
        # [2단계] 5m 전진 구간
        elif current_state == "FORWARD":
            distance_traveled = np.linalg.norm(current_pos[:2] - start_position[:2])
            
            if distance_traveled >= 2.0:
                base_command = np.array([0.0, 0.0, 0.0])
                current_state = "TURN_START"
                print(f"📍 2m 도달 완료. 회전 제어기로 전환합니다.")
            else:
                base_command = np.array([1.0, 0.0, 0.0])
                
# [3단계] 회전 목표 각도 계산
        elif current_state == "TURN_START":
            target_yaw = current_yaw + (90.0 * np.pi / 180.0)
            
            # 각도 범위 보정 (-pi ~ pi)
            if target_yaw > np.pi: target_yaw -= 2 * np.pi
            elif target_yaw < -np.pi: target_yaw += 2 * np.pi
            
            prev_yaw_error = target_yaw - current_yaw
            if prev_yaw_error > np.pi: prev_yaw_error -= 2 * np.pi
            elif prev_yaw_error < -np.pi: prev_yaw_error += 2 * np.pi
            raw_yaw_error_integral = 0.0
            is_correcting_printed = False
            
            current_state = "TURNING"
            print(f"🔄 [PID 회전] 목표 각도 {target_yaw * 180 / np.pi:.1f}도로 회전을 시작합니다.")
            
        # [4단계] PID 기반 반시계 90도 회전 및 분기 처리
        elif current_state == "TURNING":
            yaw_error = target_yaw - current_yaw
            if yaw_error > np.pi: yaw_error -= 2 * np.pi
            elif yaw_error < -np.pi: yaw_error += 2 * np.pi
            
            P_term = Kp * yaw_error
            raw_yaw_error_integral += yaw_error * dt
            raw_yaw_error_integral = np.clip(raw_yaw_error_integral, -0.5, 0.5)
            I_term = Ki * raw_yaw_error_integral
            
            yaw_rate = (yaw_error - prev_yaw_error) / dt
            D_term = Kd * yaw_rate
            
            turn_speed = P_term + I_term + D_term
            turn_speed = np.clip(turn_speed, -0.8, 0.8)
            
            # 회전 완료 조건 (각도 마진 3.5도 이내 및 회전 속도가 안정화되었을 때)
            if abs(yaw_error) < (1.5 * np.pi / 180.0) and abs(yaw_rate) < 0.05:
                base_command = np.array([0.0, 0.0, 0.0])
                
                # 회전이 끝났으므로 변 카운트 하나 증가 (0 -> 1 -> 2 -> 3)
                side_counter += 1
                
                # 🎯 [핵심 수정] 4번째 회전까지 끝나서 한 바퀴를 다 돌았다면 (side_counter == 4)
                if side_counter >= 4:
                    current_state = "ALIGN_ORIGIN"
                    print("🎯 [한 바퀴 완료] 원점(0,0) 복귀 및 위치 오차 보정을 시작합니다.")
                else:
                    # 아직 한 바퀴가 안 끝났다면 다음 변 전진 준비
                    start_position = np.copy(current_pos) 
                    current_state = "FORWARD"              
                    print(f"▶️ [{side_counter + 1}번째 변 전환] 다음 변으로 전진합니다.")
            else:
                base_command = np.array([0.0, 0.0, turn_speed])
                if yaw_error < 0 and not is_correcting_printed:
                    print("🔄 [재보정] 목표각도를 넘어, 각도를 반대로 보정 중에 있습니다.")
                    is_correcting_printed = True
                
            prev_yaw_error = yaw_error

        # 🎯 [5단계] 통합 상태: 원점 위치 및 최종 방향 동시 정렬 (Simultaneous Homing)
        elif current_state == "ALIGN_ORIGIN":
            # 1. 원점(0,0)까지의 남은 거리 오차 계산
            origin_error = np.linalg.norm(current_pos[:2] - np.array([0.0, 0.0]))
            
            # 2. 최초 시작했던 정방향(initial_yaw)과의 최종 각도 오차 계산 및 보정 (-pi ~ pi)
            final_heading_error = initial_yaw - current_yaw
            if final_heading_error > np.pi: final_heading_error -= 2 * np.pi
            elif final_heading_error < -np.pi: final_heading_error += 2 * np.pi
            
            # 🛑 [종료 조건] 원점 거리 오차가 10cm 이내이고, '동시에' 처음 방향 오차도 1.5도 이내일 때
            if origin_error < 0.1 and abs(final_heading_error) < (1.5 * np.pi / 180.0):
                base_command = np.array([0.0, 0.0, 0.0])
                
                # 모든 기준점을 절대 원점 기준으로 완벽하게 리셋 후 다음 바퀴 시작
                side_counter = 0
                start_position = np.array([0.0, 0.0, current_pos[2]])
                current_state = "FORWARD"
                print(f"✨ [동시 보정 완료] 위치({origin_error:.3f}m)와 방향({final_heading_error * 180 / np.pi:.1f}도) 정렬 성공! 다음 바퀴를 시작합니다.")
            
            else:
                # 3. 실시간으로 원점(0,0)을 바라보기 위한 각도(벡터) 계산
                target_origin_yaw = np.arctan2(0.0 - current_pos[1], 0.0 - current_pos[0])
                
                yaw_error_to_origin = target_origin_yaw - current_yaw
                if yaw_error_to_origin > np.pi: yaw_error_to_origin -= 2 * np.pi
                elif yaw_error_to_origin < -np.pi: yaw_error_to_origin += 2 * np.pi
                
                # 4. 상황에 따른 전진(X) 및 회전(Z) 명령 동시 생성
                if origin_error >= 0.1:
                    # 💡 [아직 원점에 도달하지 못한 경우]
                    # 원점 방향을 조준하면서 앞으로 걸어갑니다. (위치와 각도 동시 제어)
                    turn_speed = Kp * yaw_error_to_origin
                    turn_speed = np.clip(turn_speed, -0.5, 0.5)
                    
                    # 만약 원점 방향과 몸의 방향이 30도 이상 너무 많이 틀어져 있다면, 
                    # 발이 꼬이지 않게 제자리 회전 비중을 높이고 전진 속도를 줄입니다.
                    if abs(yaw_error_to_origin) > (30.0 * np.pi / 180.0):
                        base_command = np.array([0.0, 0.0, turn_speed])
                    else:
                        base_command = np.array([0.15, 0.0, turn_speed])
                        
                else:
                    # 💡 [원점 근처(10cm 이내)에는 왔으나, 최종 방향(initial_yaw)이 아직 안 맞은 경우]
                    # 자리는 잡았으니 전진(X)은 멈추고, 제자리 회전만 해서 처음 정방향을 맞춥니다.
                    realign_turn_speed = Kp * final_heading_error
                    realign_turn_speed = np.clip(realign_turn_speed, -0.4, 0.4)
                    base_command = np.array([0.0, 0.0, realign_turn_speed])

simulation_app.close()