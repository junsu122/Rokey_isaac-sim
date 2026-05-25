# IW Hub Nav2 / Odom / TF Notes

이 문서는 IW Hub가 Isaac Sim에서 stack/unload 이동 중 반대 방향으로 가거나, 벽 쪽으로 계속 가거나, 자기 위치를 제대로 파악하지 못하는 증상을 분석하기 위해 정리한 메모다.

## 핵심 결론

현재 의심되는 가장 큰 원인은 `cmd_vel` 자체보다 좌표계 정합 문제다.

특히 아래 상태라면 Nav2가 로봇 위치를 잘못 판단할 수 있다.

```text
Isaac 실제 spawn 위치:
iw_hub_01 = (-8, -14)

/iw_hub_01/odom:
시작 기준 상대 좌표라서 (0, 0) 근처로 시작

map -> iw_hub_01/odom static TF:
(0, 0, 0)으로 고정

결과:
Nav2가 map 기준 로봇 위치를 (-8, -14)가 아니라 (0, 0) 근처로 판단
```

이 경우 Nav2 goal이 맞아도 경로가 틀어져 반대 방향, 벽 방향, 비정상 회전 같은 증상이 생길 수 있다.

## Odom과 TF 의미

`odom`은 보통 로봇의 시작점 기준 이동량을 나타내는 좌표계다.

```text
odom -> base_link
```

이 값은 로봇이 움직이면 계속 바뀌는 것이 정상이다.

반면 `map -> odom`은 map/world 좌표와 odom 좌표를 연결하는 보정값이다. 일반 주행 중에는 보통 고정된다.

```text
map -> odom
```

전체 위치는 TF 체인으로 계산된다.

```text
map -> base_link = map -> odom + odom -> base_link
```

예:

```text
시작:
map -> odom = (-8, -14)
odom -> base_link = (0, 0)
map -> base_link = (-8, -14)

STACK_1 도착 후:
map -> odom = (-8, -14)          # 고정
odom -> base_link = (-4.8, 23.0) # 이동하면서 변경
map -> base_link = (-12.8, 9.0)
```

## Isaac IW Hub Odom 방식

현재 IW Hub의 `/odom`은 실제 로봇 바퀴 encoder tick을 적분해서 계산하는 방식이라기보다 Isaac Sim 내부의 chassis pose/velocity를 읽어 ROS odom으로 publish하는 방식에 가깝다.

관련 구성:

```text
IsaacComputeOdometry
-> ROS2PublishOdometry
-> /iw_hub_01/odom
```

즉 흐름은 대략 다음과 같다.

```text
Isaac Sim 물리 시뮬레이션에서 chassis pose 계산
-> IsaacComputeOdometry
-> ROS2PublishOdometry
-> /iw_hub_01/odom
```

다만 `/odom` pose가 Isaac world 기준인지, 로봇 시작점 기준인지는 실제 실행 시 `/iw_hub_01/odom` 시작값을 확인해야 한다.

## Map 기준 방식 권장

현재 프로젝트의 목표 좌표는 창고 world/map 기준으로 쓰이고 있다.

예:

```text
WAIT_1  = (-8.0, -14.0)
STACK_1 = (-12.8, 9.0)
UNLOAD_1 = (4.0, -13.0)
```

따라서 Nav2를 쓸 때는 아래 방식이 가장 자연스럽다.

```text
현재 위치:
TF로 map 기준 위치 계산

목표 위치:
map/world 기준 좌표 그대로 사용

좌표 연결:
map -> odom -> base_link
```

서버가 `STACK_1에서 들어서 UNLOAD_1에 놓아라` 같은 명령을 줄 때도 서버는 map 기준 장소명 또는 map 기준 좌표만 주면 된다. 로봇이 움직이며 현재 위치가 바뀌는 것은 odom과 TF가 계속 갱신한다.

서버가 직접 로봇 기준 좌표로 목표를 계속 변환해서 보내면 좌표계가 섞여 더 위험해질 수 있다.

## 시작 기준 좌표 방식도 가능한가

가능하다. 하지만 이 경우 목표 좌표도 시작 기준 odom 좌표로 변환해야 한다.

예:

```text
robot start in map = (-8.0, -14.0)
STACK_1 in map = (-12.8, 9.0)

STACK_1 in odom-start frame:
x = -12.8 - (-8.0) = -4.8
y = 9.0 - (-14.0) = 23.0
```

즉 현재 위치를 odom 기준으로 쓰면서 목표를 map 기준 `(-12.8, 9.0)`으로 주면 안 된다.

이 방식은 axis 직접 제어에는 가능하지만, Nav2에는 map 기준 TF를 맞추는 방식이 더 권장된다.

## Axis와 Nav2 실행 관계

현재 `robot1_stack_sequence.py`는 이동 제어 방식을 옵션으로 나눈다.

```text
--motion-controller axis
--motion-controller nav2
```

기본값은 `axis`다.

### Axis 모드

```text
robot1_stack_sequence
-> compute_axis_nav_command()
-> /iw_hub_01/cmd_vel
-> Isaac IW Hub 이동
```

특징:

- Nav2를 쓰지 않는다.
- sequence 노드가 직접 `/iw_hub_01/cmd_vel`을 publish한다.
- x축 먼저, y축 먼저 같은 축 기반 경로를 직접 따라간다.

### Nav2 모드

Nav2 launch에서는 sequence 노드에 아래 옵션을 준다.

```text
--motion-controller nav2
--nav2-action-name /iw_nav_1/navigate_to_pose
--nav2-goal-frame map
```

흐름:

```text
robot1_stack_sequence
-> /iw_nav_1/navigate_to_pose goal 전송
-> Nav2 planner/controller
-> /iw_nav_1/cmd_vel
-> cmd_vel_relay
-> /iw_hub_01/cmd_vel
-> Isaac IW Hub 이동
```

Nav2 모드에서는 같은 sequence 노드 안에서 직접 `/iw_hub_01/cmd_vel`을 publish하지 않는다.

하지만 별도의 axis 노드, move_to_point 노드, 다른 서버가 동시에 `/iw_hub_01/cmd_vel`을 publish하면 명령이 충돌할 수 있다.

## 확인 명령

Isaac/ROS 실행 중, 로봇이 움직이기 전에 먼저 확인한다.

```bash
ros2 topic echo /iw_hub_01/odom --once
```

시작 직후 position이 `(0, 0)` 근처면 odom이 시작점 기준일 가능성이 크다.

```bash
ros2 run tf2_ros tf2_echo map iw_hub_01/base_link
```

시작 직후 이 값은 Isaac spawn 위치 `(-8, -14)` 근처여야 정상이다.

```bash
ros2 run tf2_ros tf2_echo iw_hub_01/odom iw_hub_01/base_link
```

이 값은 시작 직후 `(0, 0)` 근처여도 정상일 수 있다.

```bash
ros2 run tf2_ros tf2_echo map iw_hub_01/odom
```

현재 static TF가 `(0, 0, 0)`으로 나온다면, odom이 시작점 기준인 경우 Nav2 위치 판단이 틀릴 수 있다.

```bash
ros2 topic info /iw_hub_01/cmd_vel -v
```

`/iw_hub_01/cmd_vel` publisher가 여러 개면 axis/Nav2/서버 명령이 충돌할 수 있다.

## 의심이 맞을 때 1차 수정 방향

만약 `/odom`이 시작점 기준이고, `map -> odom`이 `(0, 0, 0)`으로 되어 있다면 Nav2 launch의 static TF를 spawn 위치에 맞춰야 한다.

현재 의심되는 수정 예:

```python
_static_tf("iw_hub_01_map_to_odom", -8.0, -14.0, yaw, "iw_hub_01/odom")
_static_tf("iw_hub_02_map_to_odom", -10.0, -14.0, yaw, "iw_hub_02/odom")
```

`yaw` 값은 `/odom` 시작 orientation이 어떤 기준인지 보고 결정해야 한다.

- `/odom` 시작 yaw가 0에 가깝고 Isaac spawn yaw가 90도라면 `yaw = 1.5708`일 가능성이 있다.
- `/odom` 시작 yaw가 이미 90도라면 `yaw = 0.0`일 가능성이 있다.

수정 후 다시 확인한다.

```bash
ros2 run tf2_ros tf2_echo map iw_hub_01/base_link
```

시작 직후 값이 Isaac spawn 위치와 맞으면 Nav2 좌표계 정합이 맞은 것이다.

## 서버 명령 방식

서버가 작업 명령을 줄 때 권장 방식:

```text
robot_id: iw_hub_01
pickup: STACK_1
dropoff: UNLOAD_1
```

또는:

```text
pickup: map 기준 (-12.8, 9.0)
dropoff: map 기준 (4.0, -13.0)
```

서버가 직접 `/cmd_vel`을 계속 publish하는 방식이면 Nav2와 동시에 켜면 안 된다.

원칙:

```text
한 로봇의 /iw_hub_01/cmd_vel publisher는 한 시점에 하나만
```

Nav2를 쓸 때:

```text
서버 -> 목표 좌표/action/service
Nav2 -> cmd_vel
```

서버가 속도 명령을 직접 줄 때:

```text
서버 -> /iw_hub_01/cmd_vel
Nav2는 끄기
```
