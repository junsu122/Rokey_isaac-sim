# Rokey Isaac-Sim 프로젝트 전체 정리

> **브랜치**: `jintaek`  
> **기술 스택**: NVIDIA Isaac Sim 5.1 · ROS2 Humble · Firebase Firestore · React 19 · Python 3.10  
> **목적**: ArUco 마커 기반 물류센터 자동화 (AMR · 협동로봇 · 드론 통합 제어)

---

## 디렉토리 구조

```
Rokey_isaac-sim/
├── DB/                          # Firebase Firestore 관리 (Python)
├── robot/                       # ArUco 인식 + ROS2 브릿지
│   ├── config/                  # 설정 파일 (YAML, JSON)
│   ├── docs/                    # 아키텍처 문서
│   ├── markers/                 # ArUco 마커 이미지 (PNG)
│   ├── ros_bridge/              # ROS2 ↔ Firebase 브릿지
│   ├── tests/                   # 테스트 · 시뮬레이션 스크립트
│   ├── urdf/                    # 로봇 URDF 모델
│   └── utils/                   # 유틸리티 모듈
├── simulation/                  # Isaac Sim 물류센터 환경
├── UI/                          # React 모니터링 대시보드
│   └── src/
│       ├── components/          # UI 컴포넌트
│       └── hooks/               # Firebase 커스텀 훅
└── main_isaac/                  # iw_hub ROS2 패키지 (설치본)
    └── robots/iw_hub/
        └── install/smart_factory/   # smart_factory ROS2 패키지
```

---

## 1. DB/ — Firebase Firestore 관리

Firebase Admin SDK로 Firestore 데이터를 초기화·관리·모니터링한다.

### 파일 목록

| 파일 | 역할 |
|------|------|
| `firebase_manager.py` | Firebase 앱 초기화, UTC 타임스탬프 유틸리티 |
| `robot_status.py` | 로봇 3종 상태 관리 (AMR · 드론 · 협동로봇) |
| `task_manager.py` | 작업 생성·완료·상태 추적 |
| `inventory.py` | 구획(sections) · 물품(products) · 배송(items) 관리 |
| `item_tracker.py` | 배송 인스턴스 등록 및 상태 변화 추적 |
| `navigation.py` | AMR 네비게이션 목표 등록 및 도착 확인 |
| `setup_inventory.py` | 초기 구획·물품 마스터 데이터 등록 |
| `seed_example_data.py` | 예시 데이터 삽입 |
| `monitor.py` | Firestore 실시간 터미널 모니터링 |
| `reset_inventory.py` | Firestore 전체 초기화 |
| `test_connection.py` | Firebase 연결 확인 |

### Firestore 컬렉션 구조

```
robots/
  amr_001          AMR 로봇 상태 (위치, 배터리, 화물 상태)
  drone_001        드론 상태 (고도, 배터리, 임무)
  m0609            협동로봇 암 상태 (관절, EE 포즈, 작업)

sections/          구획 마스터 (A-1, A-2, A-3, B-1, B-2)
products/          물품 마스터 (Apple Watch, Galaxy Tab, ...)
items/             배송 인스턴스 (등록→감지→이송→배달)
tasks/             작업 지시서 (pending→in_progress→completed)
navigation/        AMR 이동 목표 (목표 구획, 좌표, 상태)
```

### 주요 상태 열거형

```python
# robot_status.py
AMRState  : charging | operating | empty | loading | transporting | unloading
DroneState: taking_off | flying | hovering | landing
ArmState  : idle | picking | placing

# task_manager.py
TaskStatus    : pending | in_progress | completed | failed
DeliveryStatus: registered | waiting | detected | in_transit | delivered | returned
```

### 실행 명령

```bash
python3 DB/test_connection.py        # 연결 확인
python3 DB/setup_inventory.py        # 초기 데이터 등록
python3 DB/seed_example_data.py      # 예시 데이터 삽입
python3 DB/monitor.py --watch        # 실시간 감시
python3 DB/monitor.py --robots       # 로봇 상태만
python3 DB/reset_inventory.py        # 전체 초기화
```

---

## 2. robot/ — 로봇 인식·제어 시스템

ArUco 마커 인식, Firebase 연동, ROS2 브릿지를 담당한다.

### 2.1 메인 진입점

| 파일 | 역할 |
|------|------|
| `isaac_aruco_main.py` | 전체 시스템 진입점 (Isaac Sim / 웹캠 / Mock) |
| `create_scene.py` | Isaac Sim 물리 환경 구성 (테이블, 박스, 카메라, 로봇) |
| `scene_setup.py` | 씬 초기화 및 구성 |

```bash
python3 robot/isaac_aruco_main.py                        # Isaac Sim 모드
python3 robot/isaac_aruco_main.py --webcam 0             # 웹캠 + Firebase
python3 robot/isaac_aruco_main.py --webcam 0 --ros       # 웹캠 + Firebase + ROS2
python3 robot/isaac_aruco_main.py --webcam 0 --no-firebase  # 웹캠만
python3 robot/isaac_aruco_main.py --mock                 # Mock 테스트
```

### 2.2 config/ — 설정 파일

#### `object_registry.yaml` — ArUco 마커 정의

| ID 범위 | 역할 | 내용 |
|---------|------|------|
| 0–4 | `item` | 제품 마커 (Apple Watch, Galaxy Tab, MacBook Pro, AirPods, Kindle) |
| 10–14 | `section` | 구획 마커 (A-1, A-2, A-3, B-1, B-2) |
| 20–22 | `destination` | 배송지 마커 (Gangnam, Seocho, Guro Digital) |

```yaml
markers:
  0:  { role: item,        label: "Apple Watch",    target_section: "A-1", destination: "Gangnam",      marker_size: 0.04 }
  1:  { role: item,        label: "Galaxy Tab",     target_section: "A-2", destination: "Seocho",       marker_size: 0.04 }
  2:  { role: item,        label: "MacBook Pro",    target_section: "A-3", destination: "Guro Digital", marker_size: 0.04 }
  3:  { role: item,        label: "AirPods",        target_section: "A-1", destination: "Gangnam",      marker_size: 0.04 }
  4:  { role: item,        label: "Kindle",         target_section: "A-2", destination: "Seocho",       marker_size: 0.04 }
  10: { role: section,     label: "Section A-1",    position: [-0.4, 0.3] }
  11: { role: section,     label: "Section A-2",    position: [ 0.0, 0.3] }
  12: { role: section,     label: "Section A-3",    position: [ 0.4, 0.3] }
  13: { role: section,     label: "Section B-1",    position: [-0.4,-0.3] }
  14: { role: section,     label: "Section B-2",    position: [ 0.0,-0.3] }
  20: { role: destination, label: "Dest-Gangnam",   position: [ 1.5, 0.5] }
  21: { role: destination, label: "Dest-Seocho",    position: [ 1.5, 0.0] }
  22: { role: destination, label: "Dest-Guro",      position: [ 1.5,-0.5] }

camera:
  width: 1280,  height: 720
  fx: 958.8,    fy: 958.8
  cx: 640.0,    cy: 360.0

aruco:
  dictionary: "DICT_4X4_50"
  corner_refinement_method: "CORNER_REFINE_SUBPIX"
```

#### `ros_topics.yaml` — ROS2 토픽 이름 설정

### 2.3 utils/ — 유틸리티 모듈

| 파일 | 주요 내용 |
|------|----------|
| `aruco_detector.py` | OpenCV ArUco 검출 (`DetectedMarker` 데이터클래스 반환) |
| `isaac_camera.py` | Isaac Sim 카메라 연동 |
| `video_camera.py` | 웹캠 연동 |

**`DetectedMarker` 구조**:
```python
@dataclass
class DetectedMarker:
    marker_id: int
    role: str           # "item" | "section" | "destination" | "unknown"
    label: str
    corners: np.ndarray
    center: tuple       # (px, py)
    rvec: np.ndarray    # 회전 벡터
    tvec: np.ndarray    # 이동 벡터 (3D 위치)
    distance: float     # 카메라까지 거리 [m]
```

### 2.4 ros_bridge/ — ROS2 ↔ Firebase 브릿지

| 파일 | 역할 |
|------|------|
| `run_bridge.py` | 전체 브릿지 실행 진입점 |
| `amr_bridge.py` | AMR ↔ Firebase 양방향 동기화 |
| `drone_bridge.py` | 드론 ↔ Firebase 양방향 동기화 |
| `arm_bridge.py` | M0609 협동로봇 ↔ Firebase 동기화 |
| `aruco_bridge.py` | ArUco 검출 결과 ROS2 발행 |

**ROS2 토픽 흐름**:

```
Isaac Sim → Firebase:
  /amr_001/odom             →  robots/amr_001.position
  /amr_001/battery_state    →  robots/amr_001.battery
  /drone_001/odom           →  robots/drone_001.position
  /m0609/joint_states       →  robots/m0609.joints
  /m0609/end_effector_pose  →  robots/m0609.ee_pose

Firebase → Isaac Sim:
  navigation/amr_001.target →  /amr_001/goal (PoseStamped)
  tasks/*/task_command       →  /m0609/task_command (String)
  /aruco/detections          →  ROS2 ArUco 검출 결과
```

```bash
python3 robot/ros_bridge/run_bridge.py                     # 전체
python3 robot/ros_bridge/run_bridge.py --amr-only          # AMR만
python3 robot/ros_bridge/run_bridge.py --arm-only          # 협동로봇만
python3 robot/ros_bridge/run_bridge.py --update-interval 1.0
```

### 2.5 markers/ — ArUco 마커 이미지

| 파일명 | 내용 |
|--------|------|
| `marker_0_Apple Watch.png` | 제품 마커 ID 0 |
| `marker_1_Galaxy Tab.png` | 제품 마커 ID 1 |
| `marker_2_MacBook Pro.png` | 제품 마커 ID 2 |
| `marker_3_AirPods.png` | 제품 마커 ID 3 |
| `marker_4_Kindle.png` | 제품 마커 ID 4 |
| `marker_10_Section A-1.png` ~ `marker_14_Section B-2.png` | 구획 마커 |
| `marker_20_Dest-Gangnam.png` ~ `marker_22_Dest-Guro.png` | 배송지 마커 |

### 2.6 tests/ — 테스트 스크립트

| 파일 | 역할 |
|------|------|
| `test_aruco_detector.py` | ArUco 검출기 단위 테스트 |
| `simulate_robots.py` | 로봇 상태 시뮬레이션 |
| `simulate_robots_simple.py` | 간단한 로봇 시뮬레이션 |
| `simulate_full_demo.py` | 전체 워크플로우 데모 시뮬레이션 |

### 2.7 docs/ — 문서

| 파일 | 내용 |
|------|------|
| `ARCHITECTURE.md` | 전체 시스템 아키텍처 및 동작 흐름 |
| `database_structure.md` | Firestore 스키마 상세 설명 |
| `ros_bridge_guide.md` | ROS2-Firebase 브릿지 사용 가이드 |

---

## 3. simulation/ — Isaac Sim 물류센터 환경

NVIDIA Isaac Sim 기반 iw.hub AMR 로봇 물류센터 시뮬레이션.

| 파일 | 역할 |
|------|------|
| `iwhub_action_graph.py` | Action Graph 자동 생성 (cmd_vel, odom, tf, lidar) |
| `iwhub_inspect.py` | iw.hub 로봇 내부 구조 확인 (조인트, LiDAR 경로) |
| `create_warehouse.py` | 물류센터 씬 생성 스크립트 |
| `README.md` | iw.hub 사용 가이드 |

### Action Graph 구조

```
IWHub_DriveGraph:
  ROS2Context
  → ROS2SubscribeTwist (/cmd_vel)
  → DifferentialController
  → ArticulationController (left_wheel, right_wheel)

IWHub_SensorGraph:
  IsaacReadSimulationTime
  → ROS2PublishClock (/clock)
  → ROS2PublishOdometry (/odom)
  → ROS2PublishTransformTree (/tf)

IWHub_LidarGraph:
  IsaacReadLidarPointCloud
  → ROS2PublishPointCloud (/point_cloud)
```

### 로봇 파라미터

```python
ROBOT_PRIM_PATH  = "/World/iw_hub"
WHEEL_RADIUS     = 0.1      # m
WHEEL_DISTANCE   = 0.555    # m
MAX_LIN_SPEED    = 2.2      # m/s
MAX_ANG_SPEED    = 1.5      # rad/s
```

### Isaac Sim 씬 파라미터 (create_scene.py)

```python
TABLE_SIZE    = [1.2, 0.6, 0.05]         # 테이블 1.2 × 0.6 m
BOX_SIZE      = [0.08, 0.08, 0.08]       # 박스 8 cm 큐브
CAMERA_POS    = [0.0, 0.0, 0.9]          # 카메라 수직 0.9 m 위
BIN_POSITIONS = {
    0: [0.5,  0.3, 0.01],   # 강남
    1: [0.5,  0.0, 0.01],   # 서초
    2: [0.5, -0.3, 0.01],   # 구로
}
```

---

## 4. UI/ — React 모니터링 대시보드

Firebase Firestore를 실시간으로 구독하는 웹 모니터링 UI.

### 기술 스택

| 항목 | 내용 |
|------|------|
| 프레임워크 | React 19 + TypeScript |
| 번들러 | Vite |
| 스타일 | Tailwind CSS |
| DB | Firebase Web SDK (Firestore onSnapshot) |

### 파일 구조

```
UI/src/
├── App.tsx                # 메인 앱 컴포넌트
├── firebase.ts            # Firebase 웹 SDK 초기화
├── types.ts               # TypeScript 타입 정의
├── components/
│   ├── AmazonLogo.tsx     # 로고 컴포넌트
│   ├── BatteryBar.tsx     # 배터리 잔량 바
│   ├── InventoryPanel.tsx # 재고 현황 패널
│   ├── RobotCard.tsx      # 로봇 상태 카드
│   ├── StatusBadge.tsx    # 상태 뱃지
│   └── WarehouseMap.tsx   # 창고 평면도 SVG 맵
└── hooks/
    ├── useRobotFleet.ts   # 로봇 상태 실시간 구독 훅
    └── useInventory.ts    # 재고 실시간 구독 훅
```

### 실행

```bash
cd UI
cp .env.example .env.local   # Firebase 웹 설정 입력
npm install
npm run dev                  # http://localhost:5173
```

---

## 5. main_isaac/robots/iw_hub/ — smart_factory ROS2 패키지

iw.hub 2대를 활용한 스마트 팩토리 자동화 ROS2 패키지 (설치본).

### smart_factory 패키지 노드 목록

| 파일 | 역할 |
|------|------|
| `dispatcher.py` | 작업 분배기 (태스크 → 로봇 할당) |
| `task_manager_node.py` | 작업 관리 ROS2 노드 |
| `grid_planner.py` | 격자 기반 경로 계획 |
| `graph.py` | 토폴로지 그래프 경로 탐색 |
| `reservation.py` | 경로 예약 충돌 방지 |
| `footprint_reservation.py` | 로봇 풋프린트 기반 예약 |
| `occupancy_grid.py` | 점유 격자 맵 관리 |
| `axis_nav_to_place.py` | 축 정렬 네비게이션 |
| `reserved_axis_nav.py` | 예약 기반 축 네비게이션 |
| `robot_axis_nav_to_xy.py` | XY 좌표 축 정렬 이동 |
| `move_to_point.py` | 단일 포인트 이동 노드 |
| `aruco_alignment.py` | ArUco 마커 정렬 제어 |
| `pose_estimator.py` | 로봇 자세 추정 |
| `robot_pose_monitor.py` | 로봇 자세 실시간 모니터링 |
| `current_pose_node.py` | 현재 자세 발행 노드 |
| `shelf_geometry.py` | 선반 형상 계산 |
| `shelf_transport_planner.py` | 선반 이송 경로 계획 |
| `shelf_experiment.py` | 선반 실험 시나리오 |
| `robot_defaults.py` | 로봇 기본 설정값 |
| `models.py` | 공통 데이터 모델 |
| `two_robot_reservation_demo.py` | 2대 로봇 예약 데모 |
| `two_robot_reservation_follower.py` | 2대 로봇 팔로워 |
| `robot1_stack_sequence.py` | 로봇1 스택 시퀀스 |
| `robot2_stack_sequence.py` | 로봇2 스택 시퀀스 |
| `sample_world.py` | 샘플 월드 구성 |
| `demo.py` | 데모 시나리오 |

### Launch 파일

```
smart_factory/share/smart_factory/launch/task_manager.launch.py
```

---

## 6. 전체 데이터 흐름

```
카메라 (Isaac Sim / 웹캠)
  ↓
aruco_detector.py (OpenCV ArUco 인식)
  DetectedMarker { id, role, label, 3D위치, 거리 }
  ↓
역할별 처리
  item 마커    → TaskManager.create_task()  → tasks/{id}
               → ArmManager.set_detected_item()
  section 마커 → AMRManager.set_localization() (위치 보정)
  dest 마커    → TaskManager.complete_task() → items/{id} 업데이트
  ↓
Firebase Firestore 업데이트
  (robots/, items/, tasks/, navigation/)
  ↓
ROS2 브릿지 (Firestore 변경 감지)
  → /m0609/task_command  (협동로봇에 작업 지시)
  → /amr_001/goal        (AMR에 이동 목표)
  ↓
Isaac Sim 로봇 동작
  M0609 암: 픽업 → 배치
  AMR: 목표 구획으로 이동 → 화물 수령 → 배송지 이동
  드론: 모니터링 비행
  ↓
로봇 상태 피드백
  /amr_001/odom       → AMRBridge  → robots/amr_001.position
  /m0609/joint_states → ArmBridge  → robots/m0609.joints
  ↓
UI 대시보드
  Firestore onSnapshot → React 실시간 갱신
```

---

## 7. 빠른 시작 가이드

### 환경 준비

```bash
# Firebase 서비스 계정 키 배치
# Firebase Console → 프로젝트 설정 → 서비스 계정 → 새 비공개 키 생성
cp <다운로드한_키>.json robot/config/serviceAccountKey.json
```

### DB 초기화

```bash
python3 DB/test_connection.py        # 연결 확인
python3 DB/setup_inventory.py        # 구획·물품 마스터 등록
python3 DB/seed_example_data.py      # 예시 데이터 (선택)
```

### 웹 대시보드

```bash
cd UI
cp .env.example .env.local           # Firebase 웹 앱 키 입력
npm install && npm run dev           # http://localhost:5173
```

### ArUco 인식 실행

```bash
# 웹캠 + Firebase 연동
python3 robot/isaac_aruco_main.py --webcam 0

# ROS2 토픽 발행 포함
python3 robot/isaac_aruco_main.py --webcam 0 --ros

# Isaac Sim 내부 실행
python3 robot/isaac_aruco_main.py
```

### ROS2 브릿지

```bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
python3 robot/ros_bridge/run_bridge.py
```

### iw_hub 스마트 팩토리

```bash
cd main_isaac/robots/iw_hub
source install/setup.bash
ros2 launch smart_factory task_manager.launch.py
```

### Firestore 모니터링

```bash
python3 DB/monitor.py --watch        # 전체 실시간
python3 DB/monitor.py --robots       # 로봇 상태만
python3 DB/monitor.py --tasks        # 작업 현황만
python3 DB/monitor.py --items        # 배송 물품만
```

---

## 8. Firebase 프로젝트 정보

| 항목 | 값 |
|------|-----|
| 프로젝트 ID | `rokey-factory-base` |
| Firebase Console | https://console.firebase.google.com/project/rokey-factory-base |
| Firestore DB | https://console.firebase.google.com/project/rokey-factory-base/firestore |

---

## 9. 기술 스택 요약

| 영역 | 기술 |
|------|------|
| 시뮬레이션 | NVIDIA Isaac Sim 5.1 |
| 로봇 제어 | ROS2 Humble |
| 물품 인식 | Python, OpenCV, ArUco (DICT_4X4_50) |
| 데이터베이스 | Firebase Firestore |
| 백엔드 | Python 3.10, Firebase Admin SDK |
| 프론트엔드 | React 19, TypeScript, Vite, Tailwind CSS |
| ROS2 토픽 | Odometry, BatteryState, JointState, PoseStamped, Twist, String |
| 경로 계획 | 격자 기반(Grid), 토폴로지 그래프, 예약 기반 충돌 방지 |
