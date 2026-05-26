from isaacsim import SimulationApp

# 1. 시뮬레이션 앱 초기화 (Depth 버퍼 획득을 위해 headless=False 설정)
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from omni.isaac.sensor import Camera as IsaacCamera
from pxr import Gf, UsdGeom, Sdf

# ================================================================================
# ⚙️ 시뮬레이션 월드 및 인프라 기본 세팅
# ================================================================================
my_world = World(stage_units_in_meters=1.0)

# 기본 바닥 그리드 맵 로드
get_assets_root_path = lambda: "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/5.1"
prim = define_prim("/World/Ground", "Xform")
asset_path = get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# 조명 추가 (카메라 센서 렌더링용)
light_prim = define_prim("/World/DistantLight", "DistantLight")
light_prim.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

# 📐 카메라 정면 2m 거리에 테스트용 주황색 콘 배치
cone_prim = define_prim("/World/TestCone", "Cone")
cone_xform = UsdGeom.Xformable(cone_prim)
cone_xform.ClearXformOpOrder()
cone_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.2)) # X=2.0m
cone_xform.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.4))
cone_prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(1.0, 0.4, 0.0)])

# 🚨 [매우 중요] 월드를 먼저 완전히 리셋하여 스테이지 물리 뼈대를 다 잡은 뒤 센서를 선언합니다.
my_world.reset()

# ================================================================================
# 📸 RealSense 카메라 센서 단독 바인딩 (크래시 방지 최적화 버전)
# ================================================================================
camera_prim_path = "/World/MyRealSenseCamera"

# 🎯 [핵심 수정 1]: 충돌을 유발하던 이전의 define_prim(..., "Camera") 문장을 완전히 삭제했습니다.
# IsaacCamera 내부에서 렌더 파이프라인과 완벽 동기화되는 프림을 스스로 스폰하도록 일임합니다.
realsense_sensor = IsaacCamera(
    prim_path=camera_prim_path,
    name="realsense_test",
    resolution=(640, 480)
)

# 렌더 엔진 등록 및 depth 데이터 스트림 추출 활성화
realsense_sensor.initialize()
realsense_sensor.add_depth_to_frame()

# 카메라의 위치/회전 상태 안전 주입 (원점에서 정면 X축 방향 응시)
realsense_sensor.set_local_pose(
    position=np.array([0.0, 0.0, 0.3]),      # 바닥에서 30cm 높이
    orientation=np.array([1.0, 0.0, 0.0, 0.0]) # 정면 주시 쿼터니언
)

print("\n🚀 RealSense 카메라 센서 가동! 10프레임마다 센터 거리를 출력합니다...\n")

# ================================================================================
# 🕹️ 데이터 모니터링 루프
# ================================================================================
frame_count = 0

while simulation_app.is_running():
    # 매 스텝 물리/렌더링 버퍼 동기화 수행
    my_world.step(render=True)
    
    if my_world.is_playing():
        frame_count += 1
        
        # 실시간 데이터 버퍼 수신
        frame = realsense_sensor.get_current_frame()
        
        if "depth" in frame and frame["depth"] is not None:
            depth_matrix = frame["depth"]
            
            # 정중앙 픽셀 거리값 스크리닝 (640x480 행렬의 정중앙 좌표)
            center_distance = depth_matrix[240, 320]
            
            if frame_count % 10 == 0:
                print(f"👁️ [RealSense 데이터 정상 수신] 정면 장애물까지의 거리: {center_distance:.3f}m")
        else:
            if frame_count % 10 == 0:
                print("⏳ 센서가 렌더링 파이프라인 버퍼를 대기하고 있습니다...")

# ================================================================================
# 🧹 [핵심 수정 2] 세그멘테이션 폴트를 차단하는 완전 종결 청소 파트
# ================================================================================
print("🏁 주행 테스트 완료. 메모리 클린업을 집행합니다.")

# 1. 렌더 파이프라인에서 카메라 래퍼를 강제로 언바인딩하여 Vulkan 드라이버의 더블 프리 현상을 막습니다.
if realsense_sensor is not None:
    realsense_sensor.post_reset()

# 2. 월드 하위 프림 메모리를 다 걷어내고 앱을 닫습니다.
my_world.clear()
simulation_app.close()

print("🎉 [종료 완료] 크래시 현상 없이 깔끔하게 스크립트가 탈출되었습니다.")