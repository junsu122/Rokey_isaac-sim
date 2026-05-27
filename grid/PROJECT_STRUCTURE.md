# Rokey Isaac-Sim 프로젝트 구조 문서

## 프로젝트 개요

NVIDIA Isaac Sim 기반 창고 자동화 다중 로봇 시뮬레이션.
Spot 사족보행 로봇, Doosan M0609 산업용 팔, IW Hub 모바일 매니퓰레이터, Iris 쿼드로터가 협동하여 Pod 스택 이동, 상자 집기/놓기, 섹션 배송을 수행한다.

- 물리 시뮬레이션: 500 Hz
- 렌더링: 50 Hz
- 총 Python 소스: 135개 파일 / 약 3,000+ 라인
- 데이터 크기: ~189 MB (main_isaac 기준)

---

## 디렉토리 트리

```
Rokey_isaac-sim/
├── README.md                        # 프로젝트 한국어 설명
├── run_sim.sh                       # 실행 래퍼 스크립트
├── m0609_test.usd                   # 레거시 USD 에셋
│
├── main_isaac/                      # ★ 메인 애플리케이션
│   ├── main.py                      # 진입점 · 메인 루프
│   ├── robot_config.py              # 전체 로봇/환경 설정
│   ├── world_setup.py               # 씬 초기화 · 박스 스포너
│   ├── path_planner.py              # A* 경로 계획 (싱글턴)
│   ├── control_center.py            # Isaac↔GUI 브릿지
│   ├── external_control_center.py   # Tkinter GUI (별도 프로세스)
│   ├── minimap.py                   # 2D 창고 시각화 생성기
│   ├── minimap_process.py           # OpenCV 표시 프로세스
│   ├── auto_spawn_panel.py          # 인터랙티브 박스 생성 UI
│   ├── work_signals.py              # M0609 → IW Hub 신호 채널
│   │
│   └── robots/
│       ├── base_robot.py            # 추상 기반 클래스
│       ├── spot/
│       │   └── spot_agent.py        # Spot 사족보행 에이전트
│       ├── m0609/
│       │   └── m0609_agent.py       # M0609 산업용 팔 에이전트
│       ├── iw_hub/
│       │   └── iw_hub_agent.py      # IW Hub 모바일 베이스 에이전트
│       └── drone/
│           └── drone_agent.py       # Iris 쿼드로터 에이전트
│
├── Rokey_isaac-sim-iw_hub/          # 백업/서브브랜치 (참조용)
├── spot_robot/                      # 레거시 Spot 테스트 스크립트
├── m0609/                           # 레거시 M0609 테스트 스크립트
└── robots/                          # 레거시 로봇 테스트 스크립트
```

---

## 파일별 상세

### 진입점 및 실행

#### `run_sim.sh`
- ROS2 Humble 라이브러리 충돌 해결 (시스템 Python 3.10 vs Isaac Python 3.11)
- Isaac Sim 내부 rclpy 경로를 PYTHONPATH 앞에 삽입
- 실행: `bash run_sim.sh`

#### `main_isaac/main.py` (163줄)
- Isaac Sim 앱 초기화 (headless=False, 1280×720)
- ROS2 브릿지 extension 로드
- `ROBOT_REGISTRY` 읽어 에이전트 인스턴스 생성
- `setup()` → `world.reset()` → `post_reset()` → 메인 루프
- 메인 루프: 500 Hz `on_physics_step()`, 50 Hz `on_render_step()`

---

### 설정

#### `main_isaac/robot_config.py` (328줄)

| 상수 | 값 | 설명 |
|------|----|------|
| PHYSICS_DT | 1/500 | 물리 타임스텝 (s) |
| RENDERING_DT | 1/50 | 렌더링 타임스텝 (s) |
| USE_REALSENSE | True | RealSense D455 카메라 사용 여부 |
| WAREHOUSE_USD | warehouse_v7_1.usda | 창고 3D 모델 |

**환경 구조:**
- `POD_STACKS`: 고정 Pod 위치 4개 (홈/배송 구역)
- `SECTION_PODS`: A/B/C 섹션 각 3×4 그리드 (슬롯 01은 IW Hub 배송 전용)
- `ARUCO_BOXES`: green_id0, red_id1, blue_id2 박스 3종

**`ROBOT_REGISTRY`**: 로봇 스폰 마스터 설정 (아래 로봇 목록 참조)

---

### 씬 초기화

#### `main_isaac/world_setup.py` (210줄)

- `setup_warehouse(world)`: 창고 USD 로드, 조명 생성, Pod 그리드 배치
- `BoxSpawner` 클래스: 컨베이어 (-16, 0) 위치에 20초마다 ArUco 박스 자동 스폰 (green→red→blue 순환)

---

### 경로 계획

#### `main_isaac/path_planner.py` (237줄)

- 싱글턴 패턴: `get_planner()` 로 전역 인스턴스 공유
- 격자 해상도: 0.5 m/셀
- 월드 범위: x ∈ [-18, 24], y ∈ [-18, 18]
- 정적 장애물: 창고 외벽 4면
- 동적 장애물: 다른 로봇 위치 (회피 반경 2셀)

주요 메서드:
```python
plan(start_xy, goal_xy, agent_name)   # A* 단일 경로
plan_patrol(waypoints, agent_name)     # 순환 순찰 경로
```

---

### 로봇 에이전트

#### `main_isaac/robots/base_robot.py` (56줄)

```python
class BaseRobotAgent(ABC):
    def setup(self)              # USD prim 생성 (world.reset() 이전)
    def post_reset(self)         # 컨트롤러 초기화 (world.reset() 이후)
    def on_physics_step(self, dt) # 500 Hz 호출
    def on_render_step(self)     # 50 Hz 호출 (선택)
```

---

#### `main_isaac/robots/spot/spot_agent.py` (400+줄)

**역할:** Boston Dynamics Spot 순찰 + ArUco 마커 박스 집기

FSM 상태:
```
WALKING → NAVIGATE_TO_CUBE → LOWER → GRASP → RAISE → RETURN_HOME
```

주요 파라미터:
```python
_STOP_DIST    = 0.65 m    # 목표 도달 판정
_APPROACH_DIST = 1.2 m    # 접근 시작 거리
_SPEED        = 0.55 m/s  # 보행 속도
_AVOID_DIST   = 2.0 m     # 충돌 회피 거리
```

컨트롤:
- 본체: `SpotFlatTerrainPolicy` (위치/방향 명령)
- 팔: 6-DOF + RG2 그리퍼 (knuckle/follower 관절)
- 비전: RealSense D455 (ArUco 마커 검출, 최소 면적 300 px²)

---

#### `main_isaac/robots/m0609/m0609_agent.py` (700+줄)

**역할:** Doosan M0609 6-DOF 팔 + 진공 흡착 그리퍼로 ArUco 박스 집기/놓기

진공 그리퍼 구조:
- 스템: Ø44 mm, 높이 60 mm
- 패드: Ø90 mm (흡착면)
- 림: Ø96 mm 고무 립

FSM 상태:
```
MOVE_TO_HOME → DETECTING → SEARCH → SERVO → PICK_AND_PLACE → DONE
```

컨트롤:
- `RMPflow`: 모션 플래닝 + RRT 충돌 회피
- `PickPlaceController`: 집기/놓기 상태 머신
- `VisualServoController`: ArUco 마커 추적 하강

로봇 인스턴스 (robot_config):
| 이름 | 섹션 | 박스 종류 | 완료 횟수 |
|------|------|-----------|-----------|
| M0609_A | A | blue_id2 | 3 |
| M0609_B | B | red_id1 | 1 |
| M0609_C | C | green_id0 | 3 |
| M0609_3way | 컨베이어 | 3종 모두 | 3 |

---

#### `main_isaac/robots/iw_hub/iw_hub_agent.py` (550+줄)

**역할:** 모바일 매니퓰레이터로 Pod 스택 배송

ROS2 토픽:
- Subscribe: `/{robot}/cmd_vel` (Twist), `/{robot}/lift_cmd` (JointState)
- Publish: `/{robot}/odom` (Odometry), `/{robot}/tf` (TFMessage)

FSM 상태:
```
WAITING(0) → LIFTING(1) → GOTO_SECTION(2) → LOWERING(3) → GOTO_HOME(4)
```

운용 모드:
- **배송 모드** (hub_01, hub_03): M0609 완료 신호 수신 → 리프트 → 섹션 배송 → 귀환
- **픽업 모드** (hub_02): 컨베이어 픽업 → 리프트 → 섹션 배송 → 귀환

---

#### `main_isaac/robots/drone/drone_agent.py` (600+줄)

**역할:** Iris 쿼드로터 자율 Pod 섹션 배송

프레임워크: Pegasus Simulator

FSM 상태:
```
IDLE → TAKEOFF → FLY_PICK → DESCEND_PICK → GRAB
     → ASCEND_PICK → FLY_DROP → DESCEND_DROP → RELEASE → REASCEND → [반복/DONE]
```

주요 파라미터:
```python
_HOVER_ALT     = 2.5 m    # 순항 고도
_GRAB_ALT      = 1.5 m    # 픽업 고도
_NAV_TOL_XY    = 0.5 m    # 수평 도달 허용 오차
_GRAB_WAIT     = 40 steps # 파지 전 안정화 대기
```

조작: 키보드 (T=이륙, L=착륙, WASD=이동, QE=요), 조이스틱 지원

---

### 통신 및 시각화

#### `main_isaac/work_signals.py` (31줄)

M0609 → IW Hub 직접 Python 신호 채널 (ROS2 타이밍 문제 우회)

```python
signal(section)   # M0609: 집기 완료 시 카운터 증가
get(section)      # IW Hub: 누적 카운트 읽기
reset(section)    # IW Hub: 미션 전 카운터 초기화
```

#### `main_isaac/control_center.py` (300+줄)

- `external_control_center.py`를 서브프로세스(시스템 Python3)로 실행
- 송신: 로봇 카메라 피드 (~4 FPS), 로봇 상태/위치
- 수신: 박스 스폰 명령, 드론 수동 웨이포인트

#### `main_isaac/external_control_center.py` (300+줄)

Tkinter GUI (시스템 Python3):
- 좌측: 로봇 카메라 피드 (스크롤 그리드)
- 우측 탭1: Box Spawn (위치/크기/ArUco ID 조작)
- 우측 탭2: Drone (수동 웨이포인트 입력, 상태 표시)

#### `main_isaac/minimap.py` / `minimap_process.py` (670+줄 / 139줄)

실시간 2D 창고 탑뷰 시각화:
- 월드 범위: x ∈ [-18, 24], y ∈ [-18, 18] → 캔버스 900×660 px
- 렌더링: 벽(회색), 컨베이어(청록), Pod(주황/초록/보라), 로봇 아이콘
- 우클릭: 월드 좌표에 Pod 스폰
- `minimap_process.py`: 별도 OpenCV 프로세스 (Isaac Sim GUI 충돌 방지)

#### `main_isaac/auto_spawn_panel.py` (600+줄)

- ArUco 텍스처 오버레이 포함 물리 박스 생성 (`_create_box_with_aruco()`)
- JSON 큐 파일 (`~/Downloads/spawn_queue.json`) 기반 스폰 자동화

---

## 아키텍처 흐름

```
┌────────────────────────────── main.py (Isaac Sim, Python 3.11) ─────┐
│                                                                       │
│  World (500 Hz physics / 50 Hz rendering)                            │
│  ├─ SpotAgent      (순찰 + ArUco 집기)                               │
│  ├─ M0609Agent     (ArUco 비주얼 서보 집기/놓기)                     │
│  ├─ IwHubAgent     (Pod 배송 FSM + ROS2 브릿지)                      │
│  └─ DroneAgent     (자율 섹션 배송)                                  │
│                                                                       │
│  WarehousePathPlanner  (A* 싱글턴)                                   │
│  BoxSpawner            (20초마다 컨베이어 박스 스폰)                  │
│  ControlCenter         (카메라/상태 → GUI 브릿지)                    │
│  Minimap               (2D 탑뷰 → OpenCV 프로세스 브릿지)            │
│                                                                       │
└──┬─────────────────────┬────────────────────────┬────────────────────┘
   │ stdin/stdout         │ stdin/stdout            │ ROS2 Topics
   ▼                      ▼                         ▼
external_control_center  minimap_process           IW Hub 외부 컨트롤러
(Tkinter GUI)            (OpenCV 탑뷰)
```

**통신 방식 요약:**
| 방식 | 사용처 |
|------|--------|
| 직접 Python 호출 | 에이전트 내부 |
| `work_signals.py` (threading.Lock) | M0609 → IW Hub |
| ROS2 토픽 | IW Hub ↔ 외부 컨트롤러 |
| stdin/stdout 바이너리 프레임 | Isaac → Tkinter GUI, Isaac → Minimap |

---

## 실행 방법

```bash
# 방법 1 (권장)
bash /home/rokey/Rokey_isaac-sim/run_sim.sh

# 방법 2
isaac-python /home/rokey/Rokey_isaac-sim/main_isaac/main.py
```

실행 시 열리는 창:
1. Isaac Sim 3D 뷰 (기본 카메라)
2. Control Center (로봇 카메라 + 박스 스폰 UI)
3. Minimap (2D 탑뷰, 우클릭으로 Pod 스폰)

---

## 주요 설정 변경 위치

| 변경 목적 | 파일 | 수정 대상 |
|-----------|------|-----------|
| 로봇 스폰 위치 | `robot_config.py` | `ROBOT_REGISTRY[].spawn_xyz` |
| 물리/렌더링 속도 | `robot_config.py` | `PHYSICS_DT`, `RENDERING_DT` |
| 카메라 사용 여부 | `robot_config.py` | `USE_REALSENSE` |
| ArUco 타겟 | `robot_config.py` | `multi_targets` 배열 |
| 창고 3D 모델 | `world_setup.py` | `WAREHOUSE_USD` 경로 |
| M0609 집기 파라미터 | `m0609_agent.py` | `work_complete_count`, `servo_dz` |
| 경로 계획 회피 반경 | `path_planner.py` | `ROBOT_R` |
