"""
iw.hub 물류센터용 Isaac Sim Action Graph (수정판)
Isaac Sim 4.2 / 4.5 공식 문서 기준 노드 타입 사용

실행: Window > Script Editor > 붙여넣기 > Run
      실행 전 iw_hub_sensors.usd 씬에 로드 필요
"""

import omni.graph.core as og
import omni.kit.commands
import carb
from pxr import Usd
from omni.isaac.core.utils.stage import get_current_stage

# =====================================================================
# 설정값 — 환경에 맞게 수정
# =====================================================================
ROBOT_PRIM_PATH = "/World/iw_hub"   # Stage에서 실제 Prim Path 확인

WHEEL_RADIUS    = 0.1     # m  (Stage > iw_hub > chassis > left_wheel > Physics > Radius)
WHEEL_DISTANCE  = 0.555   # m  (좌우 바퀴 중심 간 거리)
MAX_LIN_SPEED   = 2.2     # m/s
MAX_ANG_SPEED   = 1.5     # rad/s

CMD_VEL_TOPIC   = "/cmd_vel"
ODOM_TOPIC      = "/odom"
CLOCK_TOPIC     = "/clock"
TF_TOPIC        = "/tf"
ROS2_DOMAIN     = 0

DRIVE_GRAPH_PATH  = "/World/IWHub_DriveGraph"
SENSOR_GRAPH_PATH = "/World/IWHub_SensorGraph"


# =====================================================================
# 유틸: 로봇에서 조인트 이름 자동 탐지
# =====================================================================
def detect_wheel_joints(robot_path):
    stage = get_current_stage()
    root  = stage.GetPrimAtPath(robot_path)

    if not root.IsValid():
        print(f"[ERROR] 로봇 Prim을 찾을 수 없습니다: {robot_path}")
        print("        Stage 패널에서 iw_hub 의 실제 경로를 확인해 ROBOT_PRIM_PATH를 수정하세요.")
        return None, None

    joints = []
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "PhysicsRevoluteJoint":
            joints.append(prim.GetName())

    if not joints:
        print("[WARN] RevoluteJoint를 찾지 못했습니다. iw_hub.usd 또는 iw_hub_sensors.usd 로드 확인.")
        return None, None

    print(f"[INFO] 발견된 RevoluteJoint: {joints}")

    left  = next((j for j in joints if "left"  in j.lower()), joints[0])
    right = next((j for j in joints if "right" in j.lower()), joints[1] if len(joints) > 1 else joints[0])

    print(f"[INFO] 사용할 조인트 → 왼쪽: '{left}', 오른쪽: '{right}'")
    return left, right


# =====================================================================
# 유틸: 기존 그래프 삭제
# =====================================================================
def delete_graph(path):
    stage = get_current_stage()
    if stage.GetPrimAtPath(path).IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[path])
        print(f"[INFO] 기존 그래프 삭제: {path}")


# =====================================================================
# 1. Drive Control Graph
#    /cmd_vel → Differential Controller → Articulation Controller
#
#  [노드 타입] Isaac Sim 4.5 공식 확인값:
#    isaacsim.ros2.bridge.ROS2SubscribeTwist
#    isaacsim.robot.wheeled_robots.DifferentialController
#    isaacsim.core.nodes.IsaacArticulationController
# =====================================================================
def create_drive_graph(left_joint, right_joint):
    delete_graph(DRIVE_GRAPH_PATH)

    try:
        (graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": DRIVE_GRAPH_PATH,
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick",      "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Ctx",     "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubTwist",    "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("BreakLin",    "omni.graph.nodes.BreakVector3"),
                    ("BreakAng",    "omni.graph.nodes.BreakVector3"),
                    ("DiffCtrl",    "isaacsim.robot.wheeled_robots.DifferentialController"),
                    ("ArticCtrl",   "isaacsim.core.nodes.IsaacArticulationController"),
                    ("LeftToken",   "omni.graph.nodes.ConstantToken"),
                    ("RightToken",  "omni.graph.nodes.ConstantToken"),
                    ("JointArr",    "omni.graph.nodes.MakeArray"),
                ],
                og.Controller.Keys.CONNECT: [
                    # ── Exec 체인 ─────────────────────────────────────────
                    # OnTick → SubTwist → ArticCtrl 순서로 실행
                    ("OnTick.outputs:tick",       "SubTwist.inputs:execIn"),
                    ("SubTwist.outputs:execOut",  "ArticCtrl.inputs:execIn"),

                    # ── ROS2 Context ──────────────────────────────────────
                    ("ROS2Ctx.outputs:context",   "SubTwist.inputs:context"),

                    # ── Twist 메시지 → 속도 분해 ──────────────────────────
                    # linearVelocity(vectord[3])의 x 성분 → DiffCtrl linear
                    ("SubTwist.outputs:linearVelocity",  "BreakLin.inputs:tuple"),
                    # angularVelocity(vectord[3])의 z 성분 → DiffCtrl angular
                    ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                    ("BreakLin.outputs:x", "DiffCtrl.inputs:linearVelocity"),
                    ("BreakAng.outputs:z", "DiffCtrl.inputs:angularVelocity"),

                    # ── DiffCtrl → ArticCtrl ──────────────────────────────
                    ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),

                    # ── 조인트 이름 배열 ──────────────────────────────────
                    ("LeftToken.inputs:value",  "JointArr.inputs:input0"),
                    ("RightToken.inputs:value", "JointArr.inputs:input1"),
                    ("JointArr.outputs:array",  "ArticCtrl.inputs:jointNames"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ROS2 설정
                    ("ROS2Ctx.inputs:domain_id",    ROS2_DOMAIN),
                    ("SubTwist.inputs:topicName",   CMD_VEL_TOPIC),

                    # 조인트 이름
                    ("LeftToken.inputs:value",      left_joint),
                    ("RightToken.inputs:value",     right_joint),
                    ("JointArr.inputs:arraySize",   2),

                    # DifferentialController 파라미터 (iw.hub 사양)
                    ("DiffCtrl.inputs:wheelRadius",     WHEEL_RADIUS),
                    ("DiffCtrl.inputs:wheelDistance",   WHEEL_DISTANCE),
                    ("DiffCtrl.inputs:maxLinearSpeed",  MAX_LIN_SPEED),
                    ("DiffCtrl.inputs:maxAngularSpeed", MAX_ANG_SPEED),

                    # ArticulationController 로봇 경로
                    ("ArticCtrl.inputs:robotPath", ROBOT_PRIM_PATH),
                ],
            },
        )
        print(f"[OK] Drive Control Graph 생성: {DRIVE_GRAPH_PATH}")
        return graph

    except Exception as e:
        print(f"[ERROR] Drive Graph 생성 실패: {e}")
        print("       Isaac Sim 4.2 사용 중이라면 아래 노드 이름으로 변경:")
        print("         isaacsim.ros2.bridge.* → omni.isaac.ros2_bridge.*")
        print("         isaacsim.robot.wheeled_robots.* → omni.isaac.wheeled_robots.*")
        print("         isaacsim.core.nodes.* → omni.isaac.core_nodes.*")
        return None


# =====================================================================
# 2. Sensor Graph
#    Clock / Odometry / TF 발행
# =====================================================================
def create_sensor_graph():
    delete_graph(SENSOR_GRAPH_PATH)

    try:
        (graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": SENSOR_GRAPH_PATH,
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick",      "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Ctx",     "isaacsim.ros2.bridge.ROS2Context"),
                    ("SimTime",     "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PubClock",    "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PubOdom",     "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PubTF",       "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    # ── Exec 체인 ─────────────────────────────────────────
                    ("OnTick.outputs:tick",       "SimTime.inputs:execIn"),
                    ("SimTime.outputs:execOut",   "PubClock.inputs:execIn"),
                    ("OnTick.outputs:tick",       "PubOdom.inputs:execIn"),
                    ("OnTick.outputs:tick",       "PubTF.inputs:execIn"),

                    # ── ROS2 Context ──────────────────────────────────────
                    ("ROS2Ctx.outputs:context",   "PubClock.inputs:context"),
                    ("ROS2Ctx.outputs:context",   "PubOdom.inputs:context"),
                    ("ROS2Ctx.outputs:context",   "PubTF.inputs:context"),

                    # ── 시뮬레이션 시간 → Clock ───────────────────────────
                    ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ROS2Ctx.inputs:domain_id",      ROS2_DOMAIN),

                    ("PubClock.inputs:topicName",     CLOCK_TOPIC),

                    ("PubOdom.inputs:topicName",      ODOM_TOPIC),
                    ("PubOdom.inputs:robotPath",      ROBOT_PRIM_PATH),
                    ("PubOdom.inputs:odomFrameId",    "odom"),
                    ("PubOdom.inputs:chassisFrameId", "base_link"),

                    ("PubTF.inputs:topicName",        TF_TOPIC),
                    ("PubTF.inputs:targetPrims",      [ROBOT_PRIM_PATH]),
                ],
            },
        )
        print(f"[OK] Sensor Graph 생성: {SENSOR_GRAPH_PATH}")
        return graph

    except Exception as e:
        print(f"[ERROR] Sensor Graph 생성 실패: {e}")
        return None


# =====================================================================
# 메인 실행
# =====================================================================
def main():
    print("=" * 60)
    print("  iw.hub Action Graph 생성 시작")
    print(f"  Robot: {ROBOT_PRIM_PATH}")
    print("=" * 60)

    # 조인트 자동 탐지
    left, right = detect_wheel_joints(ROBOT_PRIM_PATH)
    if left is None:
        print(f"[WARN] 자동 탐지 실패 → 기본값 사용")
        left  = "left_wheel"
        right = "right_wheel"

    # 그래프 생성
    drive_ok  = create_drive_graph(left, right)
    sensor_ok = create_sensor_graph()

    print()
    print("=" * 60)
    if drive_ok and sensor_ok:
        print("  완료! 다음 단계:")
        print()
        print("  1. Play ▶ 클릭")
        print()
        print("  2. 터미널에서 이동 테스트:")
        print("     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\")
        print("       '{linear: {x: 0.5}, angular: {z: 0.0}}'")
        print()
        print("  3. 바퀴가 안 움직이면:")
        print("     → Script Editor에서 iwhub_inspect.py 실행")
        print("     → 출력된 Joint Name 확인 후 상단 설정값 수정")
    else:
        print("  [ERROR] 일부 그래프 생성에 실패했습니다.")
        print("  Isaac Sim 버전에 따라 노드 이름을 아래와 같이 바꿔보세요:")
        print()
        print("  ┌─────────────────────────────────────────────────────┐")
        print("  │  Isaac Sim 4.5 (현재 스크립트)                      │")
        print("  │    isaacsim.ros2.bridge.*                           │")
        print("  │    isaacsim.robot.wheeled_robots.*                  │")
        print("  │    isaacsim.core.nodes.*                            │")
        print("  ├─────────────────────────────────────────────────────┤")
        print("  │  Isaac Sim 4.2 (구버전으로 변경 시)                  │")
        print("  │    omni.isaac.ros2_bridge.*                         │")
        print("  │    omni.isaac.wheeled_robots.*                      │")
        print("  │    omni.isaac.core_nodes.*                          │")
        print("  └─────────────────────────────────────────────────────┘")
    print("=" * 60)


main()
