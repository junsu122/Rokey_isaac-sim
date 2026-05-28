"""
spot_with_arm.usd 관절 순서 진단 스크립트
- initialize() 없이 USD stage에서 직접 관절 이름/순서를 읽습니다.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.kit.app
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from pxr import UsdPhysics, Usd

ARM_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd"
)

world = World(stage_units_in_meters=1.0)
prim = define_prim("/World/Spot", "Xform")
prim.GetReferences().AddReference(ARM_USD)

print("USD 로딩 대기 중...")
for _ in range(200):
    omni.kit.app.get_app().update()

# ── USD stage에서 관절 직접 파싱 ──────────────────────────
stage = world.stage
root_path = "/World/Spot"

joints = []
for p in stage.TraverseAll():
    path_str = str(p.GetPath())
    if not path_str.startswith(root_path + "/"):
        continue
    # RevoluteJoint 또는 PrismaticJoint 타입인 것만
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        # ArticulationRoot 바로 아래 관절만 (body1 기준)
        joint_name = p.GetName()
        # body0, body1 타겟 읽기
        body1_targets = p.GetRelationship("physics:body1").GetTargets() if p.HasRelationship("physics:body1") else []
        body1 = str(body1_targets[0]).split("/")[-1] if body1_targets else "?"
        joints.append((path_str, joint_name, body1))

# path depth 기준 정렬 (articulation joint order는 depth-first)
joints.sort(key=lambda x: x[0])

print("\n" + "="*70)
print(f"발견된 관절 수: {len(joints)}")
print("="*70)
print(f"{'idx':>4}  {'관절 이름':<25}  {'연결 바디(body1)':<20}")
print("-"*70)
for i, (path, name, body1) in enumerate(joints):
    print(f"{i:>4}  {name:<25}  {body1:<20}")
print("="*70)

# 다리/팔 분류
print("\n[분류]")
for i, (path, name, body1) in enumerate(joints):
    n = name.lower()
    if any(k in n for k in ["arm", "sh0", "sh1", "el0", "el1", "wr0", "wr1", "f1x"]):
        tag = "ARM"
    elif any(k in n for k in ["fl_", "fr_", "hl_", "hr_", "hx", "hy", "kn"]):
        tag = "LEG"
    else:
        tag = "???"
    print(f"  [{i:>2}] {tag}  {name}")

world.clear()
simulation_app.close()