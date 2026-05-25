# IW Hub Server Task Automation Plan

이 문서는 서버 통합 이후 IW Hub 로봇을 장기 실행 상태로 두고, 서버가 내려주는 pod/stack 작업을 계속 수행하게 만들기 위한 계획이다.

목표는 ROS2와 Isaac Sim을 한 번 실행하면 끄지 않고 유지한 상태에서, 서버가 작업을 전달할 때마다 IW Hub가 다음 흐름을 반복 수행하는 것이다.

```text
server task
  -> iw_hub task manager
  -> idle robot 선택
  -> source stack / pod 위치로 이동
  -> lift up
  -> unload 또는 목적지로 이동
  -> lift down
  -> 다음 작업 대기
```

## 1. 현재 코드 기준 구조

### Isaac Sim 쪽

- `main_isaac/robots/iw_hub/iw_hub_agent.py`
  - IW Hub USD spawn 담당.
  - 로봇별 ROS2 topic을 Isaac Sim ActionGraph에 연결한다.
  - 주요 topic:
    - `/iw_hub_01/cmd_vel`
    - `/iw_hub_01/lift_cmd`
    - `/iw_hub_01/odom`
    - `/iw_hub_01/tf`
    - `/iw_hub_02/cmd_vel`
    - `/iw_hub_02/lift_cmd`
    - `/iw_hub_02/odom`
    - `/iw_hub_02/tf`

### ROS2 쪽

- `main_isaac/robots/iw_hub/src/smart_factory/smart_factory/robot1_stack_sequence.py`
  - 현재 핵심 실행 노드.
  - `STACK -> lift up -> UNLOAD -> lift down -> WAIT` 형태의 순차 작업을 수행한다.
  - axis 직접 제어와 Nav2 제어를 모두 지원한다.
  - 로봇 간 peer safety, place reservation, grid reservation 로직이 들어 있다.

- `main_isaac/robots/iw_hub/src/smart_factory/smart_factory/robot2_stack_sequence.py`
  - robot1 sequence를 robot2 기본값으로 실행하는 래퍼 성격.

- `main_isaac/robots/iw_hub/src/smart_factory/smart_factory/task_manager_node.py`
  - 현재는 sample world/task를 만들어 간단히 plan을 publish하는 수준.
  - 서버 작업 큐, robot assignment, task state 관리 기능은 아직 약하다.

- `main_isaac/robots/iw_hub/src/smart_factory/smart_factory/axis_nav_to_place.py`
  - `WAIT`, `STACK`, `UNLOAD` 좌표 정의.
  - 현재 좌표는 정적 dictionary인 `PLACES` / `PLACE_CANDIDATES`로 관리된다.

- `main_isaac/robots/iw_hub/src/smart_factory/smart_factory/shelf_transport_planner.py`
  - shelf/slot/footprint 예약 개념이 이미 있다.
  - 나중에 pod/stack 점유 상태 관리로 확장하기 좋은 기반이다.

- `main_isaac/robot_config.py`
  - Isaac Sim spawn 좌표, `POD_STACKS`, IW Hub no-go zone 등 실제 warehouse 좌표의 기준 파일.

## 2. 최종 목표 구조

서버는 로봇을 직접 조종하지 않는다. 서버는 "무엇을 옮길지"만 전달하고, ROS2 쪽이 로봇 선택, 이동, lift, 상태 보고를 담당한다.

```text
Server
  -> smart_factory_server_bridge
      -> /smart_factory/task_request
          -> smart_factory_task_manager
              -> /smart_factory/iw_hub_01/task
              -> /smart_factory/iw_hub_02/task
                  -> iw_hub_task_executor
                      -> Nav2 or axis movement
                      -> /iw_hub_XX/cmd_vel
                      -> /iw_hub_XX/lift_cmd
                      <- /iw_hub_XX/odom
                      <- /iw_hub_XX/tf
```

권장 역할 분리:

- `server_bridge`
  - 서버 API, WebSocket, MQTT, HTTP 등을 ROS2 topic/action으로 변환.

- `task_manager`
  - 작업 큐 관리.
  - idle robot 선택.
  - stack/pod/unload 점유 상태 관리.
  - 작업 assign / cancel / retry 관리.

- `iw_hub_task_executor`
  - 각 로봇마다 하나씩 실행.
  - task를 받으면 이동/lift sequence 수행.
  - 작업이 끝나면 `IDLE`로 돌아가 다음 task 대기.

## 3. 서버 작업 데이터 형식 초안

초기 개발에서는 custom ROS2 message/action을 바로 만들기보다 `std_msgs/String` JSON으로 시작하는 것을 추천한다.

예시:

```json
{
  "task_id": "job_0001",
  "type": "move_pod",
  "robot_id": "auto",
  "source": {
    "type": "stack",
    "name": "STACK_3",
    "pose": [-9.7, -8.9, 0.0]
  },
  "intermediate": {
    "type": "unload",
    "name": "UNLOAD_1",
    "pose": [4.0, -13.0, 0.0]
  },
  "destination": {
    "type": "stack",
    "name": "STACK_1",
    "pose": [-12.8, 9.0, 0.0]
  },
  "priority": 10
}
```

초기 테스트용 publish:

```bash
ros2 topic pub /smart_factory/task_request std_msgs/msg/String \
"{data: '{\"task_id\":\"job_0001\",\"type\":\"move_pod\",\"robot_id\":\"auto\",\"source\":{\"type\":\"stack\",\"name\":\"STACK_3\",\"pose\":[-9.7,-8.9,0.0]},\"intermediate\":{\"type\":\"unload\",\"name\":\"UNLOAD_1\",\"pose\":[4.0,-13.0,0.0]},\"destination\":{\"type\":\"stack\",\"name\":\"STACK_1\",\"pose\":[-12.8,9.0,0.0]},\"priority\":10}'}"
```

동작이 안정되면 다음처럼 ROS2 action으로 승격할 수 있다.

```text
smart_factory_msgs/action/TransportPod.action
```

## 4. 필요한 상태 모델

서버가 좌표를 주더라도 ROS2 쪽에서도 최소한의 상태를 알고 있어야 중복 작업과 충돌을 막을 수 있다.

`smart_factory/models.py`에 추가 후보:

```python
@dataclass
class Pod:
    pod_id: str
    pose: Pose2D
    current_slot: str | None
    state: str  # at_stack, carried, at_unload, unknown


@dataclass
class StackSlot:
    name: str
    pose: Pose2D
    occupied_by_pod: str | None = None
    reserved_by_task: str | None = None


@dataclass
class UnloadSlot:
    name: str
    pose: Pose2D
    occupied_by_pod: str | None = None
    reserved_by_task: str | None = None


@dataclass
class TransportTask:
    task_id: str
    source_name: str
    source_pose: Pose2D
    unload_name: str | None
    unload_pose: Pose2D | None
    destination_name: str
    destination_pose: Pose2D
    priority: int = 0
    assigned_robot: str | None = None
    status: str = "waiting"
```

초기에는 상태를 메모리에만 들고 시작해도 된다. 이후 서버 DB나 local yaml/json snapshot과 동기화하면 된다.

## 5. 가장 중요한 변경: sequence를 executor로 전환

현재 `robot1_stack_sequence.py`는 실행 시 `--stack-target`, `--unload-target`, `--wait-target`을 받아 한 번의 고정 sequence를 수행한다.

서버 통합 후에는 노드가 종료되지 않고 다음 상태로 계속 돌아야 한다.

```text
IDLE
  -> TASK_ACCEPTED
  -> MOVE_TO_SOURCE_STACK
  -> LIFT_UP_AT_SOURCE
  -> MOVE_TO_UNLOAD
  -> LIFT_DOWN_AT_UNLOAD
  -> optional BACK_OUT_FROM_UNLOAD
  -> MOVE_TO_DEST_SOURCE_OR_EMPTY_POD
  -> LIFT_UP
  -> MOVE_TO_DEST_STACK
  -> LIFT_DOWN_AT_DEST
  -> REPORT_DONE
  -> IDLE
```

초기 MVP는 단순화해서 아래 흐름부터 구현한다.

```text
IDLE
  -> receive task
  -> MOVE_TO_SOURCE
  -> LIFT_UP
  -> MOVE_TO_DESTINATION
  -> LIFT_DOWN
  -> REPORT_DONE
  -> IDLE
```

이후 unload 중간 경유가 필요한 task type을 추가한다.

## 6. 동적 좌표 지원

현재 이동 목표는 `axis_nav_to_place.py`의 `PLACES`에 고정되어 있다.

서버가 임의 pod 좌표와 목적지를 주려면 다음 중 하나가 필요하다.

### 추천: PlaceRegistry 추가

정적 좌표는 기존 `PLACES`를 유지하고, 서버 task에서 받은 좌표만 runtime registry에 넣는다.

예:

```python
class PlaceRegistry:
    def __init__(self):
        self.static_places = dict(PLACES)
        self.dynamic_places = {}

    def set_dynamic_place(self, name: str, pose: Pose2D) -> None:
        self.dynamic_places[name] = (pose.x, pose.y)

    def resolve(self, name: str) -> tuple[float, float]:
        if name in self.dynamic_places:
            return self.dynamic_places[name]
        return self.static_places[name]
```

이렇게 하면 기존 `build_axis_route()`와 sequence 로직을 많이 재사용할 수 있다.

### 대안: target_pose 직접 이동

`_step_move()`가 `target_name` 대신 `Pose2D`를 받을 수 있게 바꾼다.

장점:
- 서버 좌표를 바로 쓸 수 있다.

단점:
- 기존 `PLACES`, target prefix, place reservation 로직과 연결하기 위해 수정 범위가 커진다.

## 7. task_manager 확장 계획

`task_manager_node.py`에 추가할 기능:

1. `/smart_factory/task_request` 구독
2. JSON parse 및 validation
3. 작업 큐 저장
4. robot 상태 구독
   - `/smart_factory/robot1_stack_sequence_status`
   - `/smart_factory/robot2_stack_sequence_status`
5. idle robot 선택
6. 로봇별 task topic publish
   - `/smart_factory/iw_hub_01/task`
   - `/smart_factory/iw_hub_02/task`
7. task status publish
   - `/smart_factory/task_status`
8. stack/pod/unload reservation 관리

robot 선택 기준 MVP:

```text
1. 지정 robot_id가 있으면 해당 로봇
2. robot_id == auto면 IDLE 로봇 중 source_pose까지 가장 가까운 로봇
3. 둘 다 busy면 queue에서 대기
```

## 8. server_bridge 계획

새 파일 후보:

```text
main_isaac/robots/iw_hub/src/smart_factory/smart_factory/server_bridge.py
```

역할:

- 서버에서 task 수신.
- task를 `/smart_factory/task_request`로 publish.
- `/smart_factory/task_status`, robot status를 서버로 전송.

초기에는 실제 서버가 없어도 테스트할 수 있게 bridge 없이 ROS2 topic publish로 먼저 구현한다.

서버 방식 후보:

- HTTP polling
- WebSocket
- MQTT
- Redis queue

추천 순서:

1. ROS2 topic JSON만으로 local MVP 구현
2. HTTP or WebSocket bridge 추가
3. 서버 ACK / retry / cancel 추가

## 9. launch 구조

현재:

- `iw_hub_nav2_bringup.launch.py`
- `nav2_reserved_sequences.launch.py`
- `task_manager.launch.py`

서버 자동화용 launch 후보:

```text
factory_runtime.launch.py
```

포함할 노드:

```text
iw_hub_nav2_bringup
smart_factory_task_manager
smart_factory_server_bridge
iw_hub_01_task_executor
iw_hub_02_task_executor
robot_pose_monitor optional
```

중요:

- executor는 launch 직후 바로 움직이면 안 된다.
- 시작 상태는 항상 `IDLE`.
- task가 들어올 때만 sequence를 시작한다.

## 10. 구현 순서

### Phase 1: local task-driven MVP

1. `models.py`에 `TransportTask`, `StackSlot`, `Pod`, `UnloadSlot` 추가.
2. `robot1_stack_sequence.py`를 기반으로 `iw_hub_task_executor.py` 생성.
3. executor가 `/smart_factory/iw_hub_XX/task`를 구독하게 한다.
4. task를 받으면 `source -> lift up -> destination -> lift down -> IDLE` 수행.
5. JSON topic publish로 단일 로봇 테스트.

### Phase 2: task_manager queue

1. `task_manager_node.py`가 `/smart_factory/task_request`를 구독한다.
2. 작업 queue를 관리한다.
3. idle robot을 선택한다.
4. robot별 task topic으로 assign한다.
5. 완료/실패 status를 publish한다.

### Phase 3: unload 경유와 stack 상태

1. task type을 추가한다.
   - `move_pod`
   - `move_pod_via_unload`
   - `relocate_empty_pod`
2. unload slot 점유/reservation 관리.
3. empty stack / occupied stack 상태 관리.
4. 작업 성공 시 pod 위치 상태 업데이트.

### Phase 4: multi-robot 안정화

1. 기존 peer safety, grid reservation 유지.
2. task_manager에서도 같은 stack/unload 중복 assign 방지.
3. Nav2 cancel/replan 동작 확인.
4. 두 로봇이 동시에 작업을 받을 때 status/log 검증.

### Phase 5: real server bridge

1. 서버 API 결정.
2. `server_bridge.py` 구현.
3. task ACK, done, failed, canceled 상태 송신.
4. 네트워크 끊김 시 retry 정책 추가.

## 11. 테스트 시나리오

### 단일 로봇 기본

```text
task: STACK_3 -> UNLOAD_1
expected:
  iw_hub_01 moves to STACK_3
  lift up
  moves to UNLOAD_1
  lift down
  status done
  executor returns IDLE
```

### 반복 작업

```text
task1: STACK_3 -> UNLOAD_1
task2: STACK_2 -> UNLOAD_2
expected:
  task1 done
  same ROS2 nodes stay alive
  task2 starts without relaunch
```

### auto robot 선택

```text
task: robot_id=auto
expected:
  task_manager chooses idle robot nearest to source
```

### 두 로봇 충돌 회피

```text
task1: iw_hub_01 STACK_1 -> UNLOAD_1
task2: iw_hub_02 STACK_3 -> UNLOAD_3
expected:
  both execute
  place/grid reservation prevents direct conflict
```

### busy robot queue

```text
task1 assigned to iw_hub_01
task2 assigned to iw_hub_01 while task1 running
expected:
  task2 waits in queue
  task2 starts after task1 done
```

## 12. 주의할 점

- `robot_config.py`의 `POD_STACKS` 좌표와 `axis_nav_to_place.py`의 `STACK` 좌표는 서로 맞아야 한다.
- `iw_hub_nav2_bringup.launch.py`의 map to odom transform도 spawn pose와 맞아야 한다.
- executor는 작업 완료 후 반드시 lift를 down 상태로 유지하고 `IDLE`을 publish해야 한다.
- 서버가 같은 pod/stack을 동시에 두 번 assign하지 않도록 task_manager에서 reservation이 필요하다.
- 처음부터 서버까지 붙이지 말고 ROS2 topic JSON으로 먼저 검증한다.

## 13. MVP 완료 기준

다음이 되면 1차 목표 완료로 본다.

```text
1. Isaac Sim 실행
2. ROS2/Nav2 launch 실행
3. iw_hub executor는 IDLE 대기
4. ros2 topic pub으로 task JSON 전송
5. iw_hub가 source로 이동
6. lift up
7. destination으로 이동
8. lift down
9. done status publish
10. 노드 종료 없이 다음 task 수신 가능
```

이 상태까지 만들면 이후 서버 통합은 bridge 문제로 분리할 수 있다.

