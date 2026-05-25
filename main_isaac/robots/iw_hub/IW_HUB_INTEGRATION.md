# IW Hub 통합 작업 기록

날짜: 2026-05-22 ~ 2026-05-23  
작업자: Rokey

---

## 개요

IW Hub 로봇 2대를 Isaac Sim에 통합하고 ROS2로 이동 제어하는 과정에서 발생한 문제와 해결 과정을 기록한다.

---

## 1. 최종 아키텍처

```
Isaac Sim (run_sim.sh)
└── main_isaac/main.py
    ├── world_setup.py        → warehouse_v7.usda 로드
    ├── IwHubAgent            → iw_hub_v2.usda reference + ActionGraph 설정
    │   ├── iw_hub_01  (0.0,  0.0, 0.0)
    │   └── iw_hub_02  (0.0,  2.0, 0.0)
    └── (M0609, Spot, Drone 등 기타 로봇)

ROS2 (iw_hub_movement 패키지)
├── /iw_hub_01/cmd_vel  →  Isaac Sim 구독
├── /iw_hub_01/odom     ←  Isaac Sim 발행
├── /iw_hub_02/cmd_vel  →  Isaac Sim 구독
└── /iw_hub_02/odom     ←  Isaac Sim 발행
```

---

## 2. 문제 1 — ActionGraph가 Isaac Sim에 인식되지 않음

### 현상

`iw_hub_v2.usda`를 reference로 로드했을 때, 그 안의 ActionGraph가 OmniGraph 런타임에 등록되지 않아 ROS2 토픽이 발행/구독되지 않음.

### 원인 분석

`iw_hub_v2.usda`와 컨베이어 벨트(`warehouse_v7.usda`)의 ActionGraph 방식 비교:

| 항목 | 컨베이어 벨트 | IW Hub (수정 전) |
|---|---|---|
| `fabricCacheBacking` | `"StageWithoutHistory"` | `"Shared"` |
| 로드 방식 | warehouse USD에 직접 내장 | reference arc로 로드 |
| OmniGraph 인식 | ✓ 정상 | ✗ 미인식 |

**근본 원인**: `fabricCacheBacking = "Shared"` 그래프는 OmniGraph 런타임이 Fabric 소유권을 직접 초기화해야 하는데, reference arc(read-only 레이어)에서 로드되면 소유권을 주장할 수 없어 Graph Registry 등록 자체가 스킵된다.

`"StageWithoutHistory"`는 Fabric 소유권 없이 USD 스테이지 데이터를 그대로 사용하므로 reference arc에서도 정상 동작한다.

### 해결

`iw_hub_v2.usda` 수정:

```diff
- token fabricCacheBacking = "Shared"
+ token fabricCacheBacking = "StageWithoutHistory"
```

---

## 3. 문제 2 — 멀티 로봇 토픽 이름 충돌

### 현상

`iw_hub_v2.usda` 안에 토픽 이름이 하드코딩되어 있어 두 로봇이 같은 토픽을 사용함.

```
iw_hub_01, iw_hub_02 모두 → /cmd_vel, /lift_cmd, /chassis/odom
```

### 원인

USD 파일에 고정값으로 박혀 있는 기본값:

```
ros2_subscribe_twist.inputs:topicName     = "/cmd_vel"
ros2_subscribe_joint_state.inputs:topicName = "/lift_cmd"
ros2_publish_odometry.inputs:topicName    = "/chassis/odom"
ros2_publish_odometry.inputs:chassisFrameId = "base_link"
ros2_publish_odometry.inputs:odomFrameId  = "odom"
```

### 해결

`IwHubAgent.setup()`에서 `stage.GetAttribute().Set()`으로 USD 편집 레이어에 직접 덮어씀:

```python
_configure_topics_usd(stage, graph_path, self.name)
```

결과:
```
iw_hub_01 → /iw_hub_01/cmd_vel, /iw_hub_01/odom, /iw_hub_01/tf
iw_hub_02 → /iw_hub_02/cmd_vel, /iw_hub_02/odom, /iw_hub_02/tf
```

---

## 4. 문제 3 — 토픽 설정 타이밍 오류 (핵심 버그)

### 현상

토픽 이름을 `post_reset()`에서 `og.Controller.attribute().set()`으로 설정했으나 ROS2 통신이 여전히 안 됨.

### 원인

```
[잘못된 순서]
setup()          → 아무것도 안 함
world.reset()    → 내부 step 실행
                   → ROS2 Publisher 생성 (topicName = "/chassis/odom" 읽음)
                   → Publisher가 "/chassis/odom"으로 고정됨 ←
post_reset()     → og.Controller.attribute().set("/iw_hub_01/odom")
                   → Fabric 값은 바뀌지만 Publisher는 재생성되지 않음
                   → 효과 없음
```

두 API의 차이:

| API | 쓰는 위치 | 효과 |
|---|---|---|
| `og.Controller.attribute().set()` | Fabric 캐시 (런타임) | Publisher 이미 생성된 후라 무효 |
| `stage.GetAttribute().Set()` | USD 편집 레이어 | `world.reset()` 이전에 확정 → Publisher 생성 시 올바른 값 읽음 |

### 해결

토픽 설정을 `post_reset()`에서 `setup()`으로 이동하고, API를 USD 직접 쓰기로 교체:

```
[올바른 순서]
setup()          → stage.GetAttribute().Set() 으로 USD 레이어 덮어씀
                   → "/iw_hub_01/odom" 확정
world.reset()    → 내부 step → Publisher 생성
                   → topicName 읽음 = "/iw_hub_01/odom" ✓
post_reset()     → graph 인식 여부 확인만
main loop        → world.step() → 정상 발행
```

---

## 5. 최종 파일 변경 목록

### `iw_hub_v2.usda`

```diff
- token fabricCacheBacking = "Shared"
+ token fabricCacheBacking = "StageWithoutHistory"
```

### `iw_hub_agent.py`

| 함수 | 변경 내용 |
|---|---|
| `_configure_topics_usd()` | 신규 추가. `stage.GetAttribute().Set()` 방식으로 USD 레이어에 토픽 기록 |
| `setup()` | `_configure_topics_usd()` 호출 추가 (world.reset() 이전 시점) |
| `post_reset()` | `_configure_topics()` 제거. graph 인식 여부 확인만 수행 |
| fallback | reference ActionGraph 미인식 시 `_build_action_graph()`로 편집 레이어에 직접 생성 |

---

## 6. 실행 방법

### Isaac Sim 실행

```bash
cd /home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim
./run_sim.sh
```

### ROS2 이동 노드 실행

```bash
# 빌드 (코드 변경 시)
cd /home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim/main_isaac/robots/iw_hub
colcon build --packages-select iw_hub_movement

# 실행
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch iw_hub_movement iw_hub.launch.py
```

### 단독 명령

```bash
# 웨이포인트 이동
ros2 run iw_hub_movement axis_nav --ros-args \
    -p robot_name:=iw_hub_01 -p waypoint:=STACK_1

# 좌표 직접 지정
ros2 run iw_hub_movement move_to_point --ros-args \
    -p robot_name:=iw_hub_01 -p target_x:=-12.0 -p target_y:=7.35
```

---

## 7. 정상 동작 확인 로그

```
# Isaac Sim 로그
[IwHubAgent] iw_hub_01 스폰 완료  xyz=(0.0, 0.0, 0.0)  yaw=0.0°
[IwHubAgent] iw_hub_01 USD 레이어 토픽 설정 완료
[IwHubAgent] iw_hub_02 스폰 완료  xyz=(0.0, 2.0, 0.0)  yaw=0.0°
[IwHubAgent] iw_hub_02 USD 레이어 토픽 설정 완료
...
[IwHubAgent] iw_hub_01 reference ActionGraph 인식됨 (토픽은 setup()에서 USD 레이어에 기록 완료)
[IwHubAgent] iw_hub_02 reference ActionGraph 인식됨 (토픽은 setup()에서 USD 레이어에 기록 완료)

# ROS2 로그
[iw_hub_01] AxisNav 시작 → WAIT_1  axis_order=xy
[iw_hub_02] AxisNav 시작 → WAIT_2  axis_order=xy
[axis_nav_02]: 목표 도달
```

---

## 8. 웨이포인트 목록

`iw_hub_movement/models.py` 기준:

| 이름 | 좌표 (x, y) | 설명 |
|---|---|---|
| `WAIT_1` | (-10.0, 7.0) | 대기 위치 1 |
| `WAIT_2` | (-10.0, 0.0) | 대기 위치 2 |
| `WAIT_3` | (-10.0, -7.0) | 대기 위치 3 |
| `STACK_1` | (-12.0, 7.35) | PodStack_01 픽업 |
| `STACK_2` | (-10.3, 0.0) | PodStack_02 픽업 |
| `STACK_3` | (-12.0, -7.5) | PodStack_03 픽업 |
| `UNLOAD_1` | (11.8, 9.6) | M0609_A 언로드 |
| `UNLOAD_2` | (11.8, -0.4) | M0609_B 언로드 |
| `UNLOAD_3` | (11.8, -10.4) | M0609_C 언로드 |
