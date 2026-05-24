# Integration Branch — 프로젝트 전체 개요

브랜치: `integration` (origin/5.24 기준)  
Isaac Sim 5.1.0 기반 다중 로봇 통합 물류 시뮬레이션

---

## 폴더 구조

```
Rokey_isaac-sim/
├── main_isaac/                  ← 시뮬레이션 진입점 + 로봇 에이전트
│   ├── main.py                  ← 메인 실행 파일
│   ├── robot_config.py          ← 로봇 스폰 좌표 / 파라미터 설정
│   ├── world_setup.py           ← 창고 USD 로드 + 조명 + ArUco 박스 배치
│   ├── auto_spawn_panel.py      ← 런타임 박스 동적 스폰 패널 (omni.ui)
│   ├── usd/                     ← 창고 + 소품 USD 파일
│   │   ├── warehouse_v7_1.usda  ← 창고 메인 씬 (현재 사용)
│   │   ├── pod_stack_4_v2.usda  ← 팟 스택 소품
│   │   └── pot_v1.usda          ← 팟 소품
│   ├── aruco_marker_box/        ← ArUco 텍스처 박스 USD + 스포너
│   │   └── usd/                 ← green_id0 / red_id1 / blue_id2 USDA
│   └── robots/
│       ├── base_robot.py        ← BaseRobotAgent 추상 클래스
│       ├── m0609/               ← 두산 M0609 협동로봇
│       ├── spot/                ← Boston Dynamics Spot
│       ├── drone/               ← Iris 쿼드로터 (Pegasus Simulator)
│       └── iw_hub/              ← iw.hub AMR (ROS2 제어)
├── m0609/                       ← 루트 레벨 M0609 (단독 테스트용)
├── spot_robot/                  ← Spot 단독 테스트 스크립트 모음
│   ├── spot_test/               ← spot_pick, spot_square_move 등
│   ├── pick_and_place_test/     ← m0609 단독 pick&place 테스트
│   └── camera_test/             ← RealSense / LiDAR 테스트
├── Rokey_isaac-sim-iw_hub/      ← iw_hub 브랜치 스냅샷 (참조용)
├── run_sim.sh                   ← 시뮬레이션 실행 스크립트
└── m0609_test.usd               ← M0609 단독 USD 테스트 파일
```

---

## 실행 방법

```bash
cd /home/rokey/dev_ws/isaac_sim/src/Rokey_isaac-sim
./run_sim.sh
```

`run_sim.sh`는 Isaac Sim 내장 Python 3.11 환경에서 `main_isaac/main.py`를 실행한다.  
ROS2 humble rclpy(3.11용)를 시스템 rclpy(3.10)보다 우선 로드하도록 `PYTHONPATH`를 설정한다.

---

## 시스템 아키텍처

```
run_sim.sh
└── main_isaac/main.py
    ├── world_setup.py         → warehouse_v7_1.usda + ArUco 박스 + Pod Stack 로드
    ├── AutoSpawnPanel         → 런타임 박스 동적 스폰 (spawn_queue.json 감시)
    └── 에이전트 (robot_config.ROBOT_REGISTRY 기준 생성)
        ├── M0609Agent ×4      → ArUco 시각 서보 + 진공 흡착 픽앤플레이스
        ├── SpotAgent          → 웨이포인트 순찰 + RG2 그리퍼 픽앤플레이스
        ├── DroneAgent         → 키보드/조이스틱 비행 + 깊이 카메라 HUD
        └── IwHubAgent         → AMR 스폰 + ROS2 ActionGraph 설정

ROS2 (iw_hub_movement 패키지)
├── /iw_hub_01/cmd_vel → Isaac Sim 구독
├── /iw_hub_01/odom    ← Isaac Sim 발행
├── /iw_hub_02/cmd_vel → Isaac Sim 구독
└── /iw_hub_02/odom    ← Isaac Sim 발행
```

---

## 에이전트 상세

### M0609Agent (`main_isaac/robots/m0609/m0609_agent.py`)

두산 M0609 6축 협동로봇 + 진공 흡착 그리퍼.

**상태 머신**

```
MOVE_TO_HOME → SEARCH (joint_5 회전으로 ArUco 탐색)
    → SERVO (비주얼 서보로 접근)
    → PICK_AND_PLACE (흡착 + 목표로 이동)
    → DONE → (n회 반복 후) WAITING
```

**주요 의존 모듈** (`m0609_aruco_detect/`)

| 파일 | 역할 |
|---|---|
| `aruco_tracker.py` | OpenCV ArUco DICT_6X6 마커 감지 |
| `visual_servo_controller.py` | EE와 마커 중심 정렬 제어 |
| `m0609_rmpflow_controller.py` | RMPFlow 기반 관절 경로 생성 |
| `m0609_pick_place_controller.py` | 픽앤플레이스 시퀀스 제어 |
| `realsense_mount.py` | RealSense D455 카메라 부착 |
| `wrist_camera.py` | 손목 카메라 이미지 스트림 |

**로봇 설정 파라미터** (`robot_config.py`)

| 파라미터 | 설명 |
|---|---|
| `spawn_xyz` | 스폰 위치 (m) |
| `spawn_yaw` | 초기 방향각 (deg) |
| `goal_xyz` | 물품을 내려놓을 목표 위치 |
| `scale` | 그리퍼 + 큐브 크기 배율 |
| `box_type` | 집을 ArUco 박스 종류 (`green_id0` / `red_id1` / `blue_id2`) |
| `aruco_box_wh` | 박스 w×h (m), ArUco 탐지 영역 계산용 |
| `waypoint_xyz` | LIFT 후 GOAL 전 경유 웨이포인트 |
| `pick_z_offset` | 픽 Z 추가 오프셋 (음수=더 아래) |
| `work_complete_count` | N회 픽앤플레이스 후 ROS2 complete 발행 → WAITING |
| `pad_reach` | EE→흡착패드 끝 거리 (m) |
| `movel_steps` | MOVEL 보간 수 (낮을수록 빠름) |

**ROS2 인터페이스**

| 토픽 | 방향 | 타입 | 내용 |
|---|---|---|---|
| `/m0609/work` | 구독 | `std_msgs/String` | `"A_start"` 수신 시 WAITING → 재개 |

**현재 등록된 M0609 인스턴스**

| 이름 | 스폰 위치 | 담당 박스 | 목표 위치 |
|---|---|---|---|
| `M0609_A` | (-12.07, 7.92, 0.93) | blue_id2 | (-12.7, 9.00, 1.3) |
| `M0609_B` | (-9.45, 0.79, 0.93) | red_id1 | (-8.2, 1.4, 1.3) |
| `M0609_C` | (-10.45, -7.80, 0.91) | green_id0 | (-9.7, -8.9, 1.3) |
| `M0609_3way` | (-14.8, 0.5, 2.295) | 3종 전환 | 종류별 다른 목표 |

> `M0609_3way`: 3종 ArUco 마커를 순서대로 인식해 각각 다른 목표로 이송하는 특수 인스턴스

---

### SpotAgent (`main_isaac/robots/spot/spot_agent.py`)

Boston Dynamics Spot + OnRobot RG2 그리퍼.

**상태 머신**

```
WALKING (웨이포인트 순찰) → NAVIGATE_TO_CUBE (ArUco 감지 시 접근)
    → LOWER → GRASP → RAISE → RETURN_HOME → RELEASE → DONE
```

**주요 기능**
- `SpotFlatTerrainPolicy` 기반 4족 보행
- 손목 카메라(RealSense D455)로 ArUco 마커 탐지
- RG2 그리퍼로 박스 집기 → ArUco ID별 목표 좌표로 이송

**설정 파라미터** (`robot_config.py`)

| 파라미터 | 설명 |
|---|---|
| `waypoints` | 순찰 경로 `[(x, y), ...]` |
| `aruco_goals` | ArUco ID → 목표 XY `{0: (x, y), 1: (x, y), ...}` |

**테스트 스크립트** (`spot_robot/spot_test/`)

| 파일 | 역할 |
|---|---|
| `spot_pick.py` | ArUco 감지 + 픽앤플레이스 단독 테스트 |
| `spot_square_move.py` | 정사각형 경로 이동 테스트 |
| `spot_global_path.py` | 전역 경로 추종 테스트 |
| `spot_obstacle.py` | 장애물 회피 테스트 |
| `dual_spot_arm.py` | Spot 2대 동시 제어 테스트 |
| `wrist_camera.py` | 손목 카메라 스트림 테스트 |
| `realsense_mount.py` | RealSense 부착 테스트 |

---

### DroneAgent (`main_isaac/robots/drone/drone_agent.py`)

Iris 쿼드로터 (Pegasus Simulator 기반).

**상태 머신**

```
WARMUP → FLYING (HybridController 키보드/조이스틱 비행)
```

**주요 기능**
- `PegasusInterface` + `Multirotor` 물리 모델
- `HybridController`: 키보드 + 조이스틱 혼합 입력
- `SoftwareDepthCamera`: 소프트웨어 깊이 추정
- `DroneHUD`: OpenCV 기반 비행 정보 오버레이
- `FrustumDrawer`: 카메라 시야각 시각화

**설정 파라미터**

| 파라미터 | 설명 |
|---|---|
| `environment` | 씬 환경 이름 (기본: `"Black Gridroom"`) |
| `takeoff_alt` | 이륙 목표 고도 m (기본: 1.5) |

---

### IwHubAgent (`main_isaac/robots/iw_hub/iw_hub_agent.py`)

iw.hub AMR — 스폰 및 ROS2 ActionGraph 설정.

**동작 방식**
1. `iw_hub_v2.usda`를 reference arc로 로드
2. ActionGraph의 `fabricCacheBacking = "StageWithoutHistory"` 설정으로 OmniGraph 인식
3. `setup()`에서 `stage.GetAttribute().Set()`으로 USD 레이어에 토픽 이름 직접 기록
4. `world.reset()` 시 ROS2 Publisher가 올바른 토픽으로 생성됨

**ROS2 인터페이스**

| 토픽 | 방향 | 타입 |
|---|---|---|
| `/{robot_name}/cmd_vel` | 구독 | `geometry_msgs/Twist` |
| `/{robot_name}/lift_cmd` | 구독 | `sensor_msgs/JointState` |
| `/{robot_name}/odom` | 발행 | `nav_msgs/Odometry` |
| `/{robot_name}/tf` | 발행 | `tf2_msgs/TFMessage` |

**iw_hub_movement ROS2 패키지 웨이포인트**

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

---

## 씬 구성 (`world_setup.py`)

### 창고
- USD: `warehouse_v7_1.usda`
- 스케일: 미터 단위 (`stage_units_in_meters=1.0`)

### ArUco 마커 박스 (kinematic)

| 종류 | 위치 (x, y, z) | M0609 담당 |
|---|---|---|
| `green_id0` | (-13.0, 9.5, 1.0) | M0609_A |
| `red_id1` | (-7.88, 1.64, 1.0) | M0609_B |
| `blue_id2` | (-9.63, -9.38, 1.0) | M0609_C |

### Pod Stack (소품)

| 이름 | 위치 (x, y, z) |
|---|---|
| `PodStack_01` | (-12.8, 9.0, 0.0) |
| `PodStack_02` | (-8.2, 1.5, 0.0) |
| `PodStack_03` | (-9.7, -8.9, 0.0) |

---

## AutoSpawnPanel (`main_isaac/auto_spawn_panel.py`)

런타임에 동적으로 물리 박스를 생성하는 omni.ui 패널.

- `~/Downloads/spawn_queue.json` 파일을 감시해 박스 스폰 요청 처리
- `~/Downloads/label_sizes.json` — 라벨별 박스 크기 설정
- `~/Downloads/zone_config.json` — 존별 위치/색상 설정
- ArUco 텍스처(`aruco_id0.png` ~ `aruco_id9.png`)를 박스 윗면에 적용
- `PhysicsRigidBodyAPI` 적용 → 물리 시뮬레이션 대상

---

## 시뮬레이션 파라미터

| 항목 | 값 |
|---|---|
| Physics 주기 | 500 Hz (2ms) |
| Rendering 주기 | 50 Hz (20ms) |
| 전역 워밍업 | 30 스텝 후 에이전트 활성화 |
| RealSense 활성화 | `USE_REALSENSE = True` |

---

## M0609 단독 테스트 (`spot_robot/pick_and_place_test/`)

| 파일 | 역할 |
|---|---|
| `m0609_pick_place_fixed_target.py` | 고정 좌표 픽앤플레이스 테스트 |
| `m0609_pick_place_controller.py` | 컨트롤러 단독 동작 테스트 |
| `m0609_rmpflow_controller.py` | RMPFlow 단독 동작 테스트 |

---

## 카메라 테스트 (`spot_robot/camera_test/`)

| 파일 | 역할 |
|---|---|
| `Camera.py` | 기본 카메라 스트림 테스트 |
| `pinhole.py` | Pinhole 카메라 모델 테스트 |
| `fisheye.py` | Fisheye 카메라 모델 테스트 |
| `rotating_lidar_rtx.py` | RTX 기반 회전 LiDAR 테스트 |
| `contact_sensor.py` | 접촉 센서 테스트 |

---

## 기술 스택

| 분류 | 기술 |
|---|---|
| 시뮬레이션 | NVIDIA Isaac Sim 5.1.0 |
| 로봇 제어 | Python, RMPFlow, IsaacSim Articulation API |
| 비전 | OpenCV, ArUco (DICT_6X6), RealSense D455 |
| 드론 | Pegasus Simulator (Iris Multirotor) |
| AMR | iw.hub + ROS2 OmniGraph ActionGraph |
| ROS2 | Humble, `geometry_msgs`, `nav_msgs`, `sensor_msgs` |
| USD | OpenUSD, PhysX, PhysicsRigidBodyAPI |
