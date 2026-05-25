# Smart Factory 0525 Notes

2026-05-25 작업 메모.

## 목표

Isaac Sim에서 IW Hub가 Nav2로 이동할 때 다음 증상을 줄이는 것이 목표다.

- stack/unload 목표로 가지 않고 반대 방향 또는 벽 쪽으로 이동
- 로봇이 자기 위치를 잘못 아는 듯한 동작
- 두 로봇이 마주보거나 동선이 겹칠 때 충돌 위험
- PodStack 근처 no-go zone 때문에 stack 접근이 불안정

서버 통합은 제외하고, 현재 로컬 ROS2/Nav2/sequence 구조 안에서 개선했다.

## 좌표계 결론

권장 방식은 `map/world` 기준이다.

```text
현재 위치: TF로 map 기준 변환
목표 위치: map/world 좌표
Nav2 goal frame: map
```

중요한 TF 체인:

```text
map -> odom -> base_link
```

`odom -> base_link`는 로봇이 움직이며 계속 변한다.  
`map -> odom`은 시작 시 spawn/world 위치에 맞춰 고정하는 것이 기본이다.

의심한 문제:

```text
Isaac spawn 위치는 (-8, -14)
하지만 /odom은 시작 기준이라 (0, 0)
map -> odom static TF도 (0, 0)
=> Nav2가 map 기준 로봇 위치를 잘못 판단
```

## 변경 1: Nav2 map -> odom 보정

파일:

- `launch/iw_hub_nav2_bringup.launch.py`
- `launch/nav2_reserved_sequences.launch.py`

현재 IW Hub spawn 기준으로 map/odom origin을 맞췄다.

```python
iw_hub_01: (-8.0, -14.0, 1.57079632679)
iw_hub_02: (-10.0, -14.0, 1.57079632679)
```

주의:

`yaw=1.5708`은 spawn yaw 90도를 반영한 값이다.  
Isaac에서 `/iw_hub_01/odom --once` 했을 때 시작 yaw가 이미 90도로 나오면 `yaw`는 `0.0`이어야 할 수 있다.

확인 명령:

```bash
ros2 topic echo /iw_hub_01/odom --once
ros2 run tf2_ros tf2_echo map iw_hub_01/base_link
ros2 run tf2_ros tf2_echo map iw_hub_01/odom
```

시작 직후 `map -> iw_hub_01/base_link`가 `(-8, -14)` 근처면 좌표계가 맞는 것이다.

## 변경 2: Nav2 active goal 중 cancel/pause

파일:

- `smart_factory/robot1_stack_sequence.py`

기존 문제:

Nav2 goal을 보낸 뒤에는 goal이 끝날 때까지 reservation/grid conflict를 적극적으로 중간 개입하지 못했다.

변경 후:

```text
Nav2 goal active 중
-> peer safety / place reservation / grid reservation 재검사
-> 충돌 위험 발견
-> active Nav2 goal cancel
-> cancel 완료 대기
-> 안전해지면 현재 위치 기준으로 route 재생성 또는 우회 route 사용
```

추가된 상태:

```python
nav_cancel_future
nav_cancel_reason
nav_replan_route
```

추가된 흐름:

```text
_nav2_cancel_plan()
_request_nav2_cancel()
_finish_nav2_cancel()
```

## 변경 3: Nav2 우회 route 2차 구현

서버 통합 없이 현재 가지고 있는 정보만 사용했다.

사용 정보:

- `peer_pose`
- `peer_reservation.cell`
- `peer_reservation.next_cell`
- `avoidance_role`

역할:

```text
robot1: yield
robot2: evade
```

동작:

```text
마주보고 오는 peer 감지
-> evader가 bypass route 생성
-> Nav2 goal cancel
-> cancel 완료 후 bypass route로 교체
-> bypass waypoint를 Nav2 goal로 전송
```

grid 충돌도 처리한다.

```text
상대가 내 next cell에 있음
서로 cell을 맞바꿈(edge swap)
상대 next cell과 내 next cell이 겹침
-> grid bypass route 생성
```

사용하는 우회 함수:

```python
_build_left_bypass_route()
_build_grid_reverse_right_bypass_route()
```

Nav2 route axis가 `direct`일 때도 우회 계산이 되도록 dominant axis를 추정한다.

```python
_dominant_axis()
_replace_route_axis()
```

## 변경 4: Nav2 reserved launch에서 회피 활성화

파일:

- `launch/nav2_reserved_sequences.launch.py`

변경:

```text
robot1 avoidance_role = yield
robot2 avoidance_role = evade
peer safety stop enabled
```

즉 너무 가까워지거나 grid conflict가 생기면 한쪽은 멈추고, evader는 가능한 경우 우회 route를 만든다.

## 변경 5: Reservation을 중심 cell + 주변 margin으로 확장

파일:

- `smart_factory/robot1_stack_sequence.py`

기존:

```text
cell=현재 중심 cell
next=다음 중심 cell
```

변경:

```text
cell=현재 중심 cell
next=다음 중심 cell
cells=현재 중심 주변 margin 포함 점유 cells
next_cells=다음 중심 주변 margin 포함 점유 cells
```

기본 margin:

```text
--grid-reservation-margin-cells 1
```

즉 중심 cell 기준 3x3 영역을 점유 영역으로 본다.

예시:

```text
cells=-1,-1|-1,0|-1,1|0,-1|0,0|0,1|1,-1|1,0|1,1
```

충돌 판단은 set 교집합 기반으로 확장했다.

```text
내 next_cells ∩ peer cells != empty
=> 상대 점유 영역으로 들어가려 함

내 next_cells ∩ peer next_cells != empty
=> 같은 영역을 동시에 예약

내 current_cells ∩ peer next_cells &&
내 next_cells ∩ peer cells
=> edge swap
```

기존 `cell/next`만 보내는 메시지도 계속 호환된다.

## 남은 구상: Stack approach waypoint

문제:

PodStack과 no-go zone이 너무 가깝다. 특히 PodStack_01은 no-go zone 너머에 있어 바로 목표만 주면 접근이 불안정할 수 있다.

향후 방향:

```text
STACK_1 이동:
현재 위치 -> STACK_1_APPROACH -> STACK_1

STACK_2 이동:
현재 위치 -> STACK_2_APPROACH -> STACK_2

STACK_3 이동:
현재 위치 -> STACK_3_APPROACH -> STACK_3
```

좌표는 나중에 Isaac에서 실제 공간을 보고 정한다.

주의:

- approach waypoint는 no-go zone 밖이어야 한다.
- Nav2 map에서 free cell이어야 한다.
- 최종 stack 진입 방향과 맞아야 한다.

## 런타임 확인 명령

좌표계:

```bash
ros2 topic echo /iw_hub_01/odom --once
ros2 run tf2_ros tf2_echo map iw_hub_01/base_link
ros2 run tf2_ros tf2_echo iw_hub_01/odom iw_hub_01/base_link
```

cmd_vel 충돌:

```bash
ros2 topic info /iw_hub_01/cmd_vel -v
```

reservation 메시지:

```bash
ros2 topic echo /smart_factory/robot1_stack_reservation
ros2 topic echo /smart_factory/robot2_stack_reservation
```

status:

```bash
ros2 topic echo /smart_factory/robot1_stack_sequence_status
ros2 topic echo /smart_factory/robot2_stack_sequence_status
```

기대 status 예:

```text
nav2=cancel_requested
nav2=canceled
grid_occupied_next:bypass
peer_head_on:bypass
peer_safety_wait
```

## 검증

이 컴퓨터에는 Isaac Sim이 없어서 시뮬레이션 검증은 못 했다.

로컬에서 한 검증:

```bash
python3 -m py_compile ...
```

통과.

더미 상태로 확인한 것:

- Nav2 cancel 상태 전환
- head-on bypass route 생성
- grid occupied-next bypass route 생성
- `cells/next_cells` reservation 포맷 생성 및 파싱
- footprint-margin 교집합 충돌 판단

## 주의 사항

- 현재 변경은 Nav2 모드 중심이다.
- axis 모드는 기존 직접 cmd_vel 제어 흐름을 최대한 유지했다.
- 우회 route는 실제 Isaac에서 공간이 충분한지 확인해야 한다.
- `grid_reservation_margin_cells=1`은 안전하지만 좁은 통로에서는 과하게 막을 수 있다.
- yaw 보정값은 실제 `/odom` 시작 orientation 확인 후 조정해야 한다.
