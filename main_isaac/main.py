"""
main_isaac/main.py
==================
여러 로봇을 단일 창고 환경에서 동시 구동하는 진입점.

실행:
    cd /home/rokey/dev_ws/main_isaac
    python main.py

로봇 추가 방법:
    1. config.py 의 ROBOT_REGISTRY 에 항목 추가
    2. 새 로봇 타입이면 robots/ 에 에이전트 클래스 작성 후 _AGENT_CLASSES 에 등록
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "exts": [
        "omni.isaac.ros2_bridge",
        "omni.isaac.core_nodes",
        "omni.graph.action",
        "isaacsim.robot_setup.assembler",
    ],
})

import time
import carb
import omni.kit.app
from isaacsim.core.api import World

from robot_config import PHYSICS_DT, RENDERING_DT, ROBOT_REGISTRY
from world_setup import setup_warehouse
from robots.spot_agent import SpotAgent
from robots.m0609_agent import M0609Agent

# ── 새 로봇 타입을 추가하면 여기에 등록 ────────────────────────────────
_AGENT_CLASSES = {
    "spot" : SpotAgent,
    "m0609": M0609Agent,
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

for _ in range(30):
    omni.kit.app.get_app().update()

for agent in agents:
    agent.post_reset()

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
try:
    while simulation_app.is_running():
        my_world.step(render=True)
        for agent in agents:
            agent.on_render_step()
finally:
    my_world.clear()
    simulation_app.close()
    print("[main] 시뮬레이션 종료.")
