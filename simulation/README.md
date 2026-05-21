# iw.hub 물류센터용 Isaac Sim Action Graph

idealworks iw.hub AMR 로봇을 Isaac Sim 4.2 / 4.5 에서 ROS2로 제어하기 위한 Action Graph 자동 생성 스크립트입니다.

---

## 파일 구성

```
warehouse_sim/
├── iwhub_inspect.py       # Step 1: 로봇 내부 구조 확인 (조인트·LiDAR 경로 출력)
├── iwhub_action_graph.py  # Step 2: Action Graph 생성
└── create_warehouse.py    # 물류센터 씬 생성 스크립트
```

---

## 사전 요구사항

| 항목 | 버전 / 조건 |
|------|------------|
| NVIDIA Isaac Sim | 4.2 또는 4.5 |
| ROS2 | Humble 또는 Iron |
| GPU | RTX 시리즈 (LiDAR RTX 렌더링 필요) |
| OS | Ubuntu 22.04 |

---

## 사용 방법

### Step 0 — Extension 활성화

Isaac Sim 상단 메뉴:

```
Window > Extensions
```

검색창에 아래 두 Extension을 찾아 **Enable** 토글 ON:

- `isaacsim.ros2.bridge` (Isaac Sim 4.5)  
  또는 `omni.isaac.ros2_bridge` (Isaac Sim 4.2)
- `omni.isaac.wheeled_robots`

---

### Step 1 — iw.hub 로봇 씬에 로드

Isaac Sim 상단 메뉴:

```
Create > Isaac > Robots > Wheeled Robots > Idealworks > iw.hub (with sensors)
```

> **sensors 버전**을 선택해야 LiDAR와 카메라가 함께 로드됩니다.

로드 후 Stage 패널에서 Prim Path 확인:

```
/World/iw_hub   ← 기본값 (다를 경우 스크립트에서 수정)
```

---

### Step 2 — 조인트 이름 확인 (`iwhub_inspect.py`)

Isaac Sim 메뉴:

```
Window > Script Editor
```

`iwhub_inspect.py` 파일 전체 내용을 복사해 붙여넣고 **Run** 클릭.

출력 예시:

```
============================================================
[iw.hub 구조 분석]  /World/iw_hub
============================================================

[RevoluteJoint (바퀴/관절)]
  Joint Name : 'left_wheel'
  Full Path  :  /World/iw_hub/chassis/left_wheel

  Joint Name : 'right_wheel'
  Full Path  :  /World/iw_hub/chassis/right_wheel

[LiDAR Sensor]
  /World/iw_hub/chassis/sick_lidar

============================================================
[iwhub_action_graph.py 에 복사할 값]
============================================================
LEFT_WHEEL_JOINT  = "left_wheel"
RIGHT_WHEEL_JOINT = "right_wheel"
lidar_prim_path   = "/World/iw_hub/chassis/sick_lidar"
============================================================
```

---

### Step 3 — Action Graph 설정값 수정 (`iwhub_action_graph.py`)

파일 상단 설정 블록을 Step 2 출력값으로 맞춰 수정:

```python
# 씬에 로드된 iw.hub 로봇의 Prim Path
ROBOT_PRIM_PATH = "/World/iw_hub"

# Step 2에서 확인한 조인트 이름
LEFT_WHEEL_JOINT  = "left_wheel"
RIGHT_WHEEL_JOINT = "right_wheel"

# iw.hub 물리 파라미터 (USD 파일 실측값)
WHEEL_RADIUS   = 0.1      # m
WHEEL_DISTANCE = 0.555    # m
MAX_LINEAR_SPEED  = 2.2   # m/s
```

> **WHEEL_RADIUS / WHEEL_DISTANCE 확인 방법**  
> Stage > iw_hub > chassis > left_wheel 선택 → Property 패널 → Physics > Radius

---

### Step 4 — Action Graph 생성

Script Editor에서 `iwhub_action_graph.py` 전체 내용을 붙여넣고 **Run** 클릭.

성공 시 Stage 패널에 아래 3개 그래프가 생성됩니다:

```
/World/IWHub_DriveGraph    ← cmd_vel 수신 → 바퀴 구동
/World/IWHub_SensorGraph   ← Clock / Odom / TF 발행
/World/IWHub_LidarGraph    ← LiDAR PointCloud 발행
```

---

### Step 5 — 시뮬레이션 시작 및 ROS2 테스트

Isaac Sim 하단 **Play ▶** 버튼 클릭 후 터미널에서:

```bash
# 발행 중인 토픽 확인
ros2 topic list

# 예상 출력:
# /clock
# /cmd_vel
# /odom
# /scan
# /point_cloud
# /tf
```

```bash
# 로봇 전진 테스트 (선속도 0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

```bash
# 회전 테스트 (각속도 0.3 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
```

```bash
# LiDAR 데이터 확인
ros2 topic echo /scan --once

# 오도메트리 확인
ros2 topic echo /odom --once
```

---

## 생성되는 Action Graph 구조

```
[IWHub_DriveGraph]
  ROS2Context
  OnPlaybackTick ──→ ROS2SubscribeTwist (/cmd_vel)
                       │ linear.x ──→ DifferentialController ──→ ArticulationController
                       └ angular.z ─╯  (wheel_r, dist, max_v)    (left_wheel, right_wheel)

[IWHub_SensorGraph]
  OnPlaybackTick ──→ IsaacReadSimulationTime ──→ ROS2PublishClock (/clock)
                 ──→ ROS2PublishOdometry (/odom, base_link → odom)
                 ──→ ROS2PublishTransformTree (/tf)

[IWHub_LidarGraph]
  OnPlaybackTick ──→ IsaacReadLidarPointCloud ──→ ROS2PublishPointCloud (/point_cloud)
```

---

## 트러블슈팅

### 바퀴가 움직이지 않는다

```
원인: 조인트 이름 불일치
해결: iwhub_inspect.py 다시 실행 → 출력된 Joint Name 으로 수정
```

### `/scan` 토픽이 없다

```
원인: iw_hub_sensors.usd 가 아닌 iw_hub.usd 를 로드했거나 LiDAR 경로 불일치
해결: iwhub_inspect.py 의 [LiDAR Sensor] 출력값으로 lidar_prim_path 수정
```

### `Extension not found` 에러

```
원인: ROS2 Bridge Extension 비활성화
해결: Window > Extensions > isaacsim.ros2.bridge 검색 후 Enable
```

### 오도메트리가 튄다 (값이 이상하다)

```
원인: WHEEL_RADIUS 또는 WHEEL_DISTANCE 값이 실제와 다름
해결: Stage > iw_hub > chassis > left_wheel > Property > Physics > Radius 값 확인 후 수정
```

### Isaac Sim 4.2 에서 노드 타입 에러

```python
# 4.2는 구버전 네임스페이스 사용 - 아래와 같이 변경
"omni.isaac.ros2_bridge.ROS2Context"          # 4.2 ✓
"isaacsim.ros2.bridge.ROS2Context"            # 4.5 ✓ (자동 호환)

"omni.isaac.wheeled_robots.DifferentialController"   # 4.2 ✓
"isaacsim.robot.wheeled_robots.DifferentialController" # 4.5 ✓
```

---

## ROS2 Nav2 연동 (선택사항)

Nav2 스택과 연동하려면 추가로 아래가 필요합니다:

```bash
# Nav2 실행 (iw.hub 파라미터 파일 별도 준비 필요)
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=iwhub_nav2_params.yaml \
  use_sim_time:=true
```

Isaac Sim에서 `/use_sim_time` 설정:

```
Edit > Preferences > ROS2 > Use Sim Time: ON
```

---

## 참고 자료

- [Isaac Sim 4.5 ROS2 Drive TurtleBot 튜토리얼](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
- [Isaac Sim Robot Assets - iw.hub](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_robots.html)
- [idealworks GitHub](https://github.com/idealworks)
- [OmniGraph 튜토리얼](https://docs.isaacsim.omniverse.nvidia.com/latest/omnigraph/omnigraph_tutorial.html)
