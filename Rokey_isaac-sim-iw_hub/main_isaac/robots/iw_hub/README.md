# IW Hub 통합 모듈

IW Hub 로봇의 Isaac Sim 스폰 및 ROS2 이동 제어를 담당하는 모듈이다.

---

## 주요 파일 역할

| 파일 | 담당 역할 |
| :--- | :--- |
| **`iw_hub_agent.py`** | Isaac Sim 내 로봇 스폰 및 ActionGraph(ROS2 토픽) 설정 |
| **`iw_hub_v2.usda`** | IW Hub 로봇 USD 모델 |
| **`src/iw_hub_movement/`** | ROS2 이동 제어 패키지 |

---

## 디렉토리 구조

```text
iw_hub/
├── iw_hub_agent.py              # Isaac Sim 에이전트
├── iw_hub_v2.usda               # 로봇 USD 모델
├── src/
│   └── iw_hub_movement/         # ROS2 패키지 루트
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       │   └── iw_hub.launch.py # 두 로봇 동시 실행 launch 파일
│       └── iw_hub_movement/     # 실제 노드 코드
│           ├── models.py        # Pose2D, WAYPOINTS 정의
│           ├── move_to_point.py # 직선 이동 노드
│           └── axis_nav.py      # X→Y 축 정렬 이동 노드
├── build/                       # colcon 빌드 결과 (자동 생성)
└── install/                     # colcon 설치 결과 (자동 생성)
```

---

## ROS2 토픽

| 방향 | 토픽 | 타입 |
| :--- | :--- | :--- |
| ROS2 → Isaac Sim | `/iw_hub_01/cmd_vel` | `geometry_msgs/Twist` |
| ROS2 → Isaac Sim | `/iw_hub_02/cmd_vel` | `geometry_msgs/Twist` |
| ROS2 → Isaac Sim | `/iw_hub_01/lift_cmd` | `sensor_msgs/JointState` |
| ROS2 → Isaac Sim | `/iw_hub_02/lift_cmd` | `sensor_msgs/JointState` |
| Isaac Sim → ROS2 | `/iw_hub_01/odom` | `nav_msgs/Odometry` |
| Isaac Sim → ROS2 | `/iw_hub_02/odom` | `nav_msgs/Odometry` |
| Isaac Sim → ROS2 | `/iw_hub_01/tf` | `tf2_msgs/TFMessage` |
| Isaac Sim → ROS2 | `/iw_hub_02/tf` | `tf2_msgs/TFMessage` |

---

## 빌드

```bash
cd /home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim/main_isaac/robots/iw_hub
colcon build --packages-select iw_hub_movement
```

빌드 후 `install/setup.bash` 가 생성된다. `main.py` 실행 시 자동으로 source 된다.

---

## 실행

### Isaac Sim 실행 (자동으로 ROS2 노드 함께 시작)

```bash
cd /home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim
isaac-python main_isaac/main.py
```

### ROS2 노드 단독 실행

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# launch 파일로 두 로봇 동시 실행
ros2 launch iw_hub_movement iw_hub.launch.py

# 웨이포인트 이름으로 이동
ros2 run iw_hub_movement axis_nav --ros-args \
    -p robot_name:=iw_hub_01 -p waypoint:=STACK_1

# 직접 좌표로 이동
ros2 run iw_hub_movement move_to_point --ros-args \
    -p robot_name:=iw_hub_01 -p target_x:=-12.0 -p target_y:=7.35
```

---

## 웨이포인트 목록

`src/iw_hub_movement/iw_hub_movement/models.py` 의 `WAYPOINTS` 에서 수정한다.

| 이름 | 설명 |
| :--- | :--- |
| `WAIT_1` / `WAIT_2` / `WAIT_3` | 로봇 대기 위치 |
| `STACK_1` / `STACK_2` / `STACK_3` | Pod Stack 픽업 위치 |
| `UNLOAD_1` / `UNLOAD_2` / `UNLOAD_3` | M0609 언로드 위치 |

---

## 스폰 위치 변경

`robot_config.py` 의 `ROBOT_REGISTRY` 에서 수정한다.

```python
{
    "type"      : "iw_hub",
    "name"      : "iw_hub_01",
    "spawn_xyz" : (0.0, 0.0, 0.0),   # ★ 여기를 수정
    "spawn_yaw" : 0.0,
},
```
