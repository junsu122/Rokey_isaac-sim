# 🏭 Isaac Sim 기반 스마트 물류공장 디지털 트윈
# main_isaac 안에 있는 파일들만 확인하시면 됩니다!!!

> **NVIDIA Isaac Sim** 환경에서 4종의 로봇이 협력하여 구현하는 **물류 자동화 디지털 트윈** 프로젝트

---

## 🎬 전체 시연 영상

> ⬇️ 전체영상 입니다

https://github.com/user-attachments/assets/1119c8ba-7b85-417a-b820-afd7819bdf98

```
📹 유튜브 링크
```

---

## 🗺️ 프로젝트 개요

| 항목 | 내용 |
|:---|:---|
| 🧪 **시뮬레이터** | NVIDIA Isaac Sim 4.x (Omniverse 기반) |
| 🐍 **언어** | Python 3 (`isaac-python` 환경) |
| 🤖 **로봇 수** | 4종 (M0609 × 3, IW Hub × 2, Spot × 2, Drone × 1) |
| 📡 **통신** | ROS2 (IW Hub 제어), OpenCV (ArUco 비전) |
| 🏗️ **환경** | 커스텀 창고 USD (`warehouse_v7_1.usda`) |

본 프로젝트는 실제 물류 공장을 모사한 **디지털 트윈** 환경을 구축하고, 여러 이기종 로봇이 **자율 협력**하는 스마트 물류 시스템을 시뮬레이션합니다.

---

## 🤖 로봇 역할 소개

### 🦾 Doosan M0609 — 물류 분류 & 적재 담당


> ArUco 마커를 인식하여 물품을 정밀하게 집고 목표 위치에 적재하는 **산업용 6축 협동로봇**

**주요 기능:**
- 🔍 손목 장착 **RealSense D455 카메라**로 ArUco 마커 실시간 감지
- 🎯 **시각 서보 컨트롤러(Visual Servo)** 를 통해 마커 위치로 정밀 유도
- 💨 **진공 흡착 그리퍼(Suction Gripper)** 로 박스 픽업 (FixedJoint 방식)
- 🧠 **RMPFlow + 직선 보간(MOVEL)** 으로 충돌 없는 경로 생성
- 🔄 ArUco ID(0/1/2) 에 따라 서로 다른 목표 위치로 분류 적재

**상태머신 흐름:**
```
MOVE_TO_HOME → Detecting → APPROACH_ABOVE → DESCEND_TO_GRIP
→ LIFT_AFTER_GRIP → MOVE_TO_GOAL → DESCEND_TO_PLACE → RETRACT_PLACE → (반복)
```

**인스턴스:**
| 이름 | 위치 | 담당 박스 |
|:---|:---|:---|
| `M0609_A` | 서측 스테이션 A | 🔵 id2 |
| `M0609_B` | 서측 스테이션 B | 🔴 id1 |
| `M0609_C` | 남측 스테이션 C | 🟢 id0 |

---

### 🚛 IW Hub — 물류 이동 & 적재 선반 이동 담당

> 창고 내부를 자율 주행하며 물품과 적재 선반(Pod)을 운반하는 **물류 이송 로봇**

**주요 기능:**
- 🗺️ **ROS2 `/cmd_vel` 토픽** 으로 외부에서 이동 명령 수신
- 📦 Pod Stack(선반)을 들어 올려 원하는 위치로 이송
- 📡 `/odom` 및 `/tf` 토픽으로 **주행 상태 퍼블리시**
- 🔧 **OmniGraph ActionGraph** 로 ROS2 브릿지 자동 구성

**ROS2 토픽 (robot_name = `iw_hub_01` / `iw_hub_02`):**
| 방향 | 토픽 | 메시지 타입 |
|:---|:---|:---|
| 수신 | `/{robot_name}/cmd_vel` | `geometry_msgs/Twist` |
| 수신 | `/{robot_name}/lift_cmd` | `sensor_msgs/JointState` |
| 송신 | `/{robot_name}/odom` | `nav_msgs/Odometry` |
| 송신 | `/{robot_name}/tf` | `tf2_msgs/TFMessage` |

**인스턴스:**
| 이름 | 스폰 위치 |
|:---|:---|
| `iw_hub_01` | (0.0, -2.0, 0.0) |
| `iw_hub_02` | (0.0,  2.0, 0.0) |

---

### 🐕 Boston Dynamics Spot — 장애물(낙하 물건) 정리 담당

> 사족보행 로봇이 창고 내부를 순찰하며 **떨어진 물건을 자율 인식하고 지정 위치로 이동**시키는 로봇

**주요 기능:**
- 🦮 **SpotFlatTerrainPolicy** 기반 사족보행 자율 이동
- 📍 설정된 웨이포인트를 순환하며 창고 순찰
- 🔍 손목 장착 카메라로 **ArUco 마커 감지** → 박스 식별
- 🤏 **OnRobot RG2 그리퍼** 로 박스 파지 및 운반
- 🗂️ ArUco ID 별 목표 위치로 박스 분류 배치

**상태머신 흐름:**
```
WALKING → NAVIGATE_TO_CUBE → LOWER → GRASP → RAISE → RETURN_HOME → RELEASE
```

**순찰 웨이포인트 (robot_config.py 에서 자유 설정):**
```python
"waypoints": [
    (4.15, 7.0),   # wp0
    (1.15, 7.0),   # wp1
    (1.15, 13.0),  # wp2
    (4.15, 13.0),  # wp3
]
```

**인스턴스:**
| 이름 | 순찰 구역 |
|:---|:---|
| `Spot_01` | 창고 동측 통로 |
| `Spot_02` | 창고 서측 통로 |

---

### 🚁 Drone (Iris Quadrotor) — 특정 물품 반송 & 배송 담당

> **Pegasus Simulator** 기반의 쿼드로터 드론으로 특정 물품을 공중으로 신속하게 반송·배송

**주요 기능:**
- 🎮 **키보드 / 조이스틱** 입력으로 실시간 비행 제어
- 📷 **소프트웨어 깊이 카메라(SoftwareDepthCamera)** 로 장애물 감지
- 🖥️ **HUD(DroneHUD)** 로 비행 상태 실시간 시각화
- 🛫 자동 이륙 후 HybridController 로 고도 유지

**상태머신 흐름:**
```
WARMUP → FLYING (HybridController 제어)
```

---

## 📁 프로젝트 구조

```
Rokey_isaac-sim/
└── main_isaac/
    ├── 🚀 main.py                    # 시뮬레이션 진입점 (모든 로봇 통합 실행)
    ├── ⚙️  robot_config.py            # 로봇 스폰 좌표 / 설정 레지스트리
    ├── 🏗️  world_setup.py             # 창고 씬 로드 (맵 + 조명 + 박스 스폰)
    ├── 📦 auto_spawn_panel.py         # 자동 박스 스폰 패널
    │
    ├── aruco_marker_box/              # ArUco 마커 박스 USD 에셋 생성
    │   ├── aruco_box_spawner.py
    │   ├── create_usda_assets.py
    │   └── generate_usda_text.py
    │
    ├── robots/
    │   ├── 🧩 base_robot.py           # 모든 로봇 에이전트 추상 베이스 클래스
    │   │
    │   ├── m0609/
    │   │   ├── m0609_agent.py         # M0609 에이전트 (ArUco 비전 + 픽앤플레이스)
    │   │   └── m0609_aruco_detect/
    │   │       ├── aruco_tracker.py           # ArUco 마커 검출
    │   │       ├── visual_servo_controller.py # 시각 서보 제어
    │   │       ├── m0609_rmpflow_controller.py# RMPFlow 경로 생성
    │   │       ├── m0609_pick_place_controller.py
    │   │       ├── wrist_camera.py            # 손목 카메라
    │   │       ├── realsense_mount.py         # RealSense D455 장착
    │   │       └── camera_viewer.py           # OpenCV 카메라 뷰어
    │   │
    │   ├── spot/
    │   │   └── spot_agent.py          # Spot 에이전트 (순찰 + ArUco 픽업)
    │   │
    │   ├── drone/
    │   │   ├── drone_agent.py         # Drone 에이전트 (비행 + HUD)
    │   │   ├── drone_deps/
    │   │   │   ├── controller.py      # HybridController (키보드/조이스틱)
    │   │   │   ├── depth_camera.py    # 소프트웨어 깊이 카메라
    │   │   │   ├── hud.py             # 비행 HUD
    │   │   │   └── drone_config.py    # 드론 파라미터
    │   │   └── pegasus_simulator/     # Pegasus Sim (쿼드로터 물리)
    │   │
    │   └── iw_hub/
    │       ├── iw_hub_agent.py        # IW Hub 에이전트 (ROS2 브릿지)
    │       └── iw_hub_v2.usda         # IW Hub USD 에셋
    │
    └── usd/
        └── warehouse_v7_1.usda        # 창고 환경 에셋
```

---

## ⚙️ 아키텍처

```
<img width="562" height="551" alt="아이작심아키텍처" src="https://github.com/user-attachments/assets/25194336-9c51-4423-aeaa-6e05859ec577" />



```

**실행 루프:**
- **Physics Callback** (500 Hz): 각 에이전트의 `on_physics_step(dt)` 호출 → 로봇 제어
- **Render Loop** (~50 Hz): 각 에이전트의 `on_render_step()` 호출 → 카메라 뷰어 업데이트
- **AutoSpawnPanel** (60프레임마다): 새 박스 자동 스폰

---

## 🚀 실행 방법

### 1️⃣ 사전 요구사항 설치

```bash
# Isaac Sim 4.x 설치 (NVIDIA Omniverse 런처 사용)
# ROS2 Humble 설치 (IW Hub 사용 시)
sudo apt install ros-humble-desktop
```

### 2️⃣ 레포지토리 클론

```bash
git clone https://github.com/your-repo/Rokey_isaac-sim.git
cd Rokey_isaac-sim
```

### 3️⃣ 시뮬레이션 실행

```bash
# Isaac Sim 전용 Python 환경으로 실행
isaac-python main_isaac/main.py
```

---

## 🛠️ 로봇 추가 방법

새로운 로봇을 추가하려면 아래 세 단계만 따르면 됩니다.

**1. `BaseRobotAgent` 를 상속하는 에이전트 클래스 작성**
```python
# main_isaac/robots/my_robot/my_robot_agent.py
from ..base_robot import BaseRobotAgent

class MyRobotAgent(BaseRobotAgent):
    def setup(self): ...
    def post_reset(self): ...
    def on_physics_step(self, dt): ...
```

**2. `main.py` 의 `_AGENT_CLASSES` 에 등록**
```python
from robots.my_robot.my_robot_agent import MyRobotAgent

_AGENT_CLASSES = {
    "my_robot": MyRobotAgent,
    # ... 기존 로봇들
}
```

**3. `robot_config.py` 의 `ROBOT_REGISTRY` 에 스폰 설정 추가**
```python
ROBOT_REGISTRY = [
    {
        "type"     : "my_robot",
        "name"     : "MyRobot_01",
        "spawn_xyz": (0.0, 0.0, 0.0),
        "spawn_yaw": 0.0,
    },
]
```

---

## 📦 의존성

### 🔧 핵심 의존성

| 패키지 | 버전 | 용도 |
|:---|:---|:---|
| **NVIDIA Isaac Sim** | 4.x | 물리 시뮬레이션 + 렌더링 |
| **Omniverse USD** | - | 씬 구성 및 에셋 |
| **Python** | 3.10+ | 메인 언어 (`isaac-python`) |

### 🐍 Python 패키지 (isaac-python 환경)

| 패키지 | 용도 |
|:---|:---|
| `numpy` | 수치 연산 (행렬, 벡터, 좌표 변환) |
| `opencv-python (cv2)` | ArUco 마커 검출, 카메라 이미지 처리 |
| `scipy` | 회전 변환 (`Rotation`), 수치 해석 |
| `omni.isaac.core` | Isaac Sim World, Articulation, Prim API |
| `omni.graph.core` | OmniGraph ActionGraph (IW Hub ROS2 브릿지) |
| `pxr (USD)` | USD 씬 편집 (UsdGeom, UsdPhysics, Sdf) |
| `isaacsim.robot.policy` | Spot FlatTerrain 보행 정책 |
| `isaacsim.asset.importer.urdf` | URDF → USD 변환 (M0609) |
| `pegasus.simulator` | 쿼드로터 드론 물리 (Drone) |

### 📡 ROS2 (IW Hub 전용)

| 패키지 | 용도 |
|:---|:---|
| **ROS2 Humble** | IW Hub 이동 명령 통신 |
| `geometry_msgs` | `/cmd_vel` Twist 메시지 |
| `nav_msgs` | `/odom` Odometry 메시지 |
| `sensor_msgs` | `/lift_cmd` JointState 메시지 |
| `omni.isaac.ros2_bridge` | Isaac Sim ↔ ROS2 브릿지 |

### 🛸 Pegasus Simulator (Drone 전용)

```bash
# Pegasus Simulator 설치 (main_isaac/robots/drone/pegasus_simulator/ 에 포함)
# 별도 설치 불필요 — 로컬 경로로 자동 로드됨
```

---

## ⚙️ 주요 설정값 (robot_config.py)

```python
# 시뮬레이션 타이밍
PHYSICS_DT   = 1 / 500   # 물리 엔진: 500 Hz
RENDERING_DT = 1 / 50    # 렌더링:   50 Hz

# RealSense D455 활성화 여부
USE_REALSENSE = True      # False 로 설정 시 시작 속도 향상

# 로봇 스폰 위치는 ROBOT_REGISTRY 에서 수정
# ArUco 박스 위치는 ARUCO_BOXES 에서 수정
# Pod Stack 위치는 POD_STACKS 에서 수정
```

---

## 🧑‍💻 팀원

| 이름 | 담당 로봇 / 역할 |
|:---|:---|
| 팀원 A | 🦾 M0609 (물류 분류 & 적재) |
| 팀원 B | 🚛 IW Hub (물류 이동) |
| 팀원 C | 🐕 Spot (장애물 정리) |
| 팀원 D | 🚁 Drone (배송) |

---

## 📝 라이선스

본 프로젝트는 두산 Rokey isaac-sim 교육 목적으로 제작되었습니다.

---

<div align="center">
  <strong>🏭 Isaac Sim 기반 스마트 물류공장 디지털 트윈</strong><br/>
  <i>NVIDIA Isaac Sim × ROS2 × ArUco Vision × Pegasus Simulator</i>
</div>
