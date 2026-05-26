from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane

# ============================================================
# Scene 준비
# ============================================================
world = World(stage_units_in_meters=1.0)

GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]))
DynamicCuboid(prim_path="/World/Cube",
    position=np.array([-.5, -.2, 1.0]),
    scale=np.array([.5, .5, .5]),
    color=np.array([.2, .3, 0.]))

# ============================================================
# Contact Sensor 생성 (Wrapper 방식, 6-7 슬라이드)
# ============================================================
from isaacsim.sensors.physics import ContactSensor

sensor = ContactSensor(
    prim_path="/World/Cube/Contact_Sensor",
    name="Contact_Sensor",
    frequency=60,
    translation=np.array([0, 0, 0]),
    min_threshold=0,
    max_threshold=10000000,
    radius=-1
)

# ============================================================
# Contact Report API 수동 부착 (6-8 슬라이드)
# ============================================================
import omni
from pxr import PhysxSchema

stage = omni.usd.get_context().get_stage()
parent_prim = stage.GetPrimAtPath("/World/Cube")
contact_report = PhysxSchema.PhysxContactReportAPI.Apply(parent_prim)
contact_report.CreateThresholdAttr(0.0)

# ============================================================
# 시뮬레이션 시작 + 센서 인터페이스 준비
# ============================================================
world.reset()

# 방법 1, 3에서 사용하는 저수준 인터페이스
from isaacsim.sensors.physics import _sensor
_contact_sensor_interface = _sensor.acquire_contact_sensor_interface()

SENSOR_PATH = "/World/Cube/Contact_Sensor"

# ============================================================
# 시뮬레이션 루프 — 세 가지 방법을 모두 호출하여 비교
# ============================================================
try:
    for i in range(300):
        world.step(render=True)

        # 100프레임(약 1.67초)마다 한 번씩만 비교 출력
        if i % 100 == 0:
            print(f"\n========== Frame {i} ==========")

            # ----------------------------------------------------
            # 방법 1: get_sensor_reading() — 권장
            # ----------------------------------------------------
            reading = _contact_sensor_interface.get_sensor_reading(
                SENSOR_PATH, use_latest_data=True
            )
            print("\n[방법 1] get_sensor_reading() — 권장 API")
            print(f"  in_contact : {reading.in_contact}")
            print(f"  value      : {reading.value:.4f} N")
            print(f"  time       : {reading.time:.4f} s")
            print(f"  is_valid   : {reading.is_valid}")

            # ----------------------------------------------------
            # 방법 2: get_current_frame() — Wrapper 객체의 메서드
            # ----------------------------------------------------
            frame = sensor.get_current_frame()
            print("\n[방법 2] get_current_frame() — 통일 인터페이스")
            print(f"  반환 타입  : {type(frame).__name__}")
            print(f"  keys       : {list(frame.keys())}")
            print(f"  in_contact : {frame['in_contact']}")
            print(f"  force      : {frame['force']:.4f} N")
            print(f"  time       : {frame['time']:.4f} s")
            print(f"  number_of_contacts : {frame['number_of_contacts']}")

            # ----------------------------------------------------
            # 방법 3: get_contact_sensor_raw_data() — Deprecated
            # ----------------------------------------------------
            raw = _contact_sensor_interface.get_contact_sensor_raw_data(SENSOR_PATH)
            print("\n[방법 3] get_contact_sensor_raw_data() — Deprecated")
            print(f"  반환 타입  : {type(raw).__name__}")
            print(f"  접촉 이벤트 수: {len(raw)}")
            if len(raw) > 0:
                # raw는 numpy structured array — 첫 번째 이벤트만 샘플로 출력
                event = raw[0]
                print(f"  [첫 이벤트] time={event['time']:.4f}, "
                      f"body0='{event['body0']}', body1='{event['body1']}'")
                print(f"             impulse={event['impulse']}")
                print(f"             normal ={event['normal']}")
                print(f"             position={event['position']}")

finally:
    print("\nSimulation finished")
    # simulation_app.close()는 종료 크래시 회피를 위해 생략