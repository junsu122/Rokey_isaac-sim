"""
main_isaac/main.py
==================
여러 로봇을 단일 창고 환경에서 동시 구동하는 진입점.

실행:
    python /path/to/main_isaac/main.py
    또는
    cd /path/to/main_isaac && python main.py

로봇 추가 방법:
    1. robot_config.py 의 ROBOT_REGISTRY 에 항목 추가
    2. 새 로봇 타입이면 robots/ 에 에이전트 클래스 작성 후 _AGENT_CLASSES 에 등록
"""
import os, sys

# Isaac Sim 내장 ROS2 (Python 3.11용) 경로 설정.
# LD_LIBRARY_PATH는 프로세스 시작 전에만 적용되므로,
# 미설정 시 환경변수를 추가하고 자신을 재실행한다.
_BRIDGE = "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble"
if os.path.isdir(_BRIDGE) and _BRIDGE + "/lib" not in os.environ.get("LD_LIBRARY_PATH", ""):
    _env = os.environ.copy()
    _env["LD_LIBRARY_PATH"] = _BRIDGE + "/lib:" + _env.get("LD_LIBRARY_PATH", "")
    _env["PYTHONPATH"]      = _BRIDGE + "/rclpy:" + _env.get("PYTHONPATH", "")
    _env["ROS_DISTRO"]      = "humble"
    _env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.execve(sys.executable, [sys.executable] + sys.argv, _env)

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "exts": [
        "omni.isaac.ros2_bridge",
        "omni.isaac.core_nodes",
        "omni.graph.action",
        "isaacsim.robot_setup.assembler",
        "isaacsim.robot.wheeled_robots",
    ],
})

import sys
import os

# main_isaac/ 디렉토리를 sys.path 에 추가 → 어느 경로에서 실행해도 로컬 모듈 인식
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import time
import carb
import omni.kit.app
from isaacsim.core.api import World

from robot_config import PHYSICS_DT, RENDERING_DT, ROBOT_REGISTRY
from world_setup import setup_warehouse
from auto_spawn_panel import AutoSpawnPanel

#########################################################로봇 추가 부분
from robots.spot.spot_agent       import SpotAgent
from robots.m0609.m0609_agent     import M0609Agent
from robots.drone.drone_agent     import DroneAgent
from robots.iw_hub.iw_hub_agent   import IwHubAgent
#####################################################################

# ── 새 로봇 타입을 추가하면 여기에 등록 ────────────────────────────────
_AGENT_CLASSES = {

    ##############################바로 위에서 추가한 부분을 불러오고 이름을 지정합니다
    "spot"  : SpotAgent,
    "m0609" : M0609Agent,
    "drone" : DroneAgent,
    "iw_hub": IwHubAgent,
    ########################################################################
}

# ── 월드 생성 ─────────────────────────────────────────────────────────
my_world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=RENDERING_DT,
)

setup_warehouse(my_world)

# ── USD 로드 대기 ─────────────────────────────────────────────────────
print("[main] 씬 로드 중...")
for _ in range(300):
    omni.kit.app.get_app().update()
time.sleep(1.0)

# ── 에이전트 생성 + setup ────────────────────────────────────────────
agents = []
for cfg in ROBOT_REGISTRY:
    robot_type = cfg["type"]
    if robot_type not in _AGENT_CLASSES:
        raise ValueError(f"알 수 없는 로봇 타입: '{robot_type}'  "
                         f"(등록된 타입: {list(_AGENT_CLASSES)})")
    agent = _AGENT_CLASSES[robot_type](cfg, my_world)
    agent.setup()
    agents.append(agent)
    print(f"[main] 에이전트 등록 완료 — {cfg['name']} ({robot_type})  "
          f"spawn={cfg['spawn_xyz']}")

print(f"\n[main] 총 {len(agents)}개 로봇 로드 완료\n")

# ── 월드 리셋 ─────────────────────────────────────────────────────────
my_world.reset()

for _ in range(60):
    omni.kit.app.get_app().update()

for agent in agents:
    agent.post_reset()

# ── AutoSpawnPanel 초기화 (world.reset() 이후에 생성) ─────────────────
spawn_panel = AutoSpawnPanel(my_world)

# ── physics 콜백 ──────────────────────────────────────────────────────
_step_count  = 0
_GLOBAL_WARM = 30   # 전 에이전트 공통 워밍업 (physics 안정 대기)


def _on_physics_step(dt: float) -> None:
    global _step_count
    _step_count += 1
    if _step_count < _GLOBAL_WARM:
        return
    for agent in agents:
        try:
            agent.on_physics_step(dt)
        except Exception as e:
            carb.log_warn(f"[{agent.name}] on_physics_step 오류: {e}")


my_world.add_physics_callback("multi_robot_step", callback_fn=_on_physics_step)

# ── 메인 루프 ─────────────────────────────────────────────────────────
print("[main] 시뮬레이션 시작")
_frame = 0
try:
    while simulation_app.is_running():
        my_world.step(render=True)
        for agent in agents:
            agent.on_render_step()
        _frame += 1
        if _frame % 60 == 0:
            spawn_panel.tick()
finally:
    my_world.clear()
    simulation_app.close()
    print("[main] 시뮬레이션 종료.")
