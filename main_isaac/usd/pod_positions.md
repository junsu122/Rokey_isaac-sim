# Pod 위치 목록 (warehouse_v7_1.usda 기준)

좌표계: World 원점(0,0,0), 단위 m  
그리드: dx=3.5m (열 간격), dy=3.0m (행 간격), cols×rows = 3×3

---

## PodStack (픽업 공급 스택)

| 이름         | x      | y     | z   | 담당 로봇     |
|-------------|--------|-------|-----|--------------|
| PodStack_01 | -12.8  |  9.0  | 0.0 | iw_hub_01 (Section A) |
| PodStack_02 |  -8.2  |  1.55 | 0.0 | iw_hub_02 (Section B pickup) |
| PodStack_03 |  -9.65 | -8.9  | 0.0 | iw_hub_03 (Section C) |
| PodStack_04 |  12.0  | 14.0  | 0.0 | 드론 배달 목적지 |

---

## Section A 슬롯 (중심 0.0, 10.0)

| 슬롯 | x    | y    | z   |
|------|------|------|-----|
| A-01 | -3.5 |  7.0 | 0.0 |
| A-02 |  0.0 |  7.0 | 0.0 |
| A-03 |  3.5 |  7.0 | 0.0 |
| A-04 | -3.5 | 10.0 | 0.0 |
| A-05 |  0.0 | 10.0 | 0.0 |
| A-06 |  3.5 | 10.0 | 0.0 |
| A-07 | -3.5 | 13.0 | 0.0 |
| A-08 |  0.0 | 13.0 | 0.0 |
| A-09 |  3.5 | 13.0 | 0.0 |

iw_hub_01 배달 목표: **A-01 (-3.5, 7.0)**

---

## Section B 슬롯 (중심 0.0, 0.0)

| 슬롯 | x    | y    | z   |
|------|------|------|-----|
| B-01 | -3.5 | -3.0 | 0.0 |
| B-02 |  0.0 | -3.0 | 0.0 |
| B-03 |  3.5 | -3.0 | 0.0 |
| B-04 | -3.5 |  0.0 | 0.0 |
| B-05 |  0.0 |  0.0 | 0.0 |
| B-06 |  3.5 |  0.0 | 0.0 |
| B-07 | -3.5 |  3.0 | 0.0 |
| B-08 |  0.0 |  3.0 | 0.0 |
| B-09 |  3.5 |  3.0 | 0.0 |

iw_hub_02 배달 목표: **B-01 (-3.5, -3.0)** 방향부터 순차 배달

---

## Section C 슬롯 (중심 0.0, -10.0)

| 슬롯 | x    | y     | z   |
|------|------|-------|-----|
| C-01 | -3.5 | -13.0 | 0.0 |
| C-02 |  0.0 | -13.0 | 0.0 |
| C-03 |  3.5 | -13.0 | 0.0 |
| C-04 | -3.5 | -10.0 | 0.0 |
| C-05 |  0.0 | -10.0 | 0.0 |
| C-06 |  3.5 | -10.0 | 0.0 |
| C-07 | -3.5 |  -7.0 | 0.0 |
| C-08 |  0.0 |  -7.0 | 0.0 |
| C-09 |  3.5 |  -7.0 | 0.0 |

iw_hub_03 배달 목표: **출하 컨베이어 옆 순차 배치** (아래 참고)

### Section C 출하 배달 위치 (출하 컨베이어 옆)

| 배달 순서 | x | y | 비고 |
|----------|-----|-----|------|
| 1번째 | 14.5 | -12.0 | `drop_idx=0` |
| 2번째 | 12.0 | -12.0 | `drop_idx=1` |
| 3번째 |  9.5 | -12.0 | `drop_idx=2` |
| 4번째 |  7.0 | -12.0 | `drop_idx=3` |
| 5번째 |  4.5 | -12.0 | `drop_idx=4` |
| n번째 | `14.5 - (n-1)×2.5` | -12.0 | x가 2.5씩 감소 |

### Section C 슬롯 전체

| 슬롯 | x | y | z | pod | 내용물 |
|------|-----|-------|-----|-----|--------|
| C-01 | -3.5 | -13.0 | 0.0 | 없음 | — (IW Hub 배달 예약) |
| C-02 |  0.0 | -13.0 | 0.0 | **빈 pod** | 없음 |
| C-03 |  3.5 | -13.0 | 0.0 | **빈 pod** | 없음 |
| C-04 | -3.5 | -10.0 | 0.0 | **빈 pod** | 없음 |
| C-05 |  0.0 | -10.0 | 0.0 | **빈 pod** | 없음 |
| C-06 |  3.5 | -10.0 | 0.0 | **빈 pod** | 없음 |
| C-07 | -3.5 |  -7.0 | 0.0 | pod | ArUco 박스 3단 적층 (드론 픽업) |
| C-08 |  0.0 |  -7.0 | 0.0 | pod | ArUco 박스 3단 적층 (드론 픽업) |
| C-09 |  3.5 |  -7.0 | 0.0 | pod | ArUco 박스 3단 적층 (드론 픽업) |

**빈 pod (내용물 없음): C-02 ~ C-06 총 5개**

---

## IW Hub 스폰 위치

| 로봇        | spawn x | spawn y | spawn z  | yaw | 모드      |
|------------|---------|---------|----------|-----|-----------|
| iw_hub_01  | -12.8   |  13.0   | -0.14    | 0°  | section_a |
| iw_hub_02  |  -6.45  |   1.5   | -0.14    | 0°  | pickup    |
| iw_hub_03  |  -9.65  | -11.0   | -0.14    | 0°  | section_c |

---

# FSM 수정 가이드

## 파일 구조

```
main_isaac/robots/iw_hub/
├── iw_hub_agent.py          ← 주 에이전트 (상수, 이동 함수, FSM 진입점)
├── fsm/
│   ├── __init__.py
│   ├── section_a.py         ← iw_hub_01 전용 FSM (Section A)
│   ├── section_c.py         ← iw_hub_03 전용 FSM (Section C)
│   ├── pickup.py            ← iw_hub_02 전용 FSM (pickup 모드)
│   └── standard.py          ← 그 외 iw_hub FSM
└── ...
```

로봇 스폰/모드 설정: `main_isaac/robot_config.py` → `ROBOT_REGISTRY`

---

## 이동 함수 종류

### `_drive_minimap_axis_with_heading(target, axis, heading, tol, fast=False)`
heading 방향으로 먼저 **회전**한 뒤 해당 축으로 주행.  
`abs(yaw_err) > 0.15` 이면 회전만, 이하이면 직진.

```python
# 예: 남쪽(-π/2)을 보면서 y=9.0까지 이동
self._drive_minimap_axis_with_heading(9.0, "y", -math.pi/2, 0.04)
```

| 파라미터 | 설명 |
|---------|------|
| `target` | 목표 좌표 (m) |
| `axis`   | `"x"` 또는 `"y"` |
| `heading`| 이동 방향 (rad). 동=0, 북=π/2, 서=π, 남=-π/2 |
| `tol`    | 도달 허용 오차 (m). 보통 0.04~0.06 |
| `fast`   | `True`이면 1m 이상 구간에서 고속(FAST_ROUTE_V) 사용 |

### `_drive_minimap_axis_no_turn(target, axis, tol)`
현재 heading **그대로** 해당 축으로 주행 (회전 없음).  
pod 집고 spawn 위치로 후진할 때 사용.

```python
# 예: 회전 없이 y=13.0으로 후진
self._drive_minimap_axis_no_turn(13.0, "y", 0.06)
```

### 허용 오차 기준

| 용도 | 값 | 상수 |
|------|-----|------|
| Pod 픽업 진입 (PodStack 앞) | 0.04 | `POD_TOL` |
| 일반 경로 이동 | 0.06 | `ROUTE_TOL` |
| 정밀 정렬 (슬롯 배치) | 0.03~0.04 | - |

---

## Section A/C FSM 상태 흐름

### Section A (`fsm/section_a.py`, iw_hub_01)

```
WAITING
  ↓ 신호 수신
GOTO_POD      _drive_minimap_axis_with_heading(POD_Y=9.0,  "y", -π/2, POD_TOL)
  ↓
LIFTING       _run_lift_phase(up=True)
  ↓
GOTO_CORRIDOR _drive_minimap_axis_no_turn(HOME_Y=13.0, "y", ROUTE_TOL)  ← 후진, yaw 유지
  ↓
GOTO_SLOT_X   _drive_minimap_axis_with_heading(DROP_X=-3.5, "x", 0.0,  ROUTE_TOL, fast=True)
  ↓
GOTO_SLOT_Y   _drive_minimap_axis_with_heading(DROP_Y=7.0,  "y", -π/2, ROUTE_TOL)
  ↓
LOWERING      _run_lift_phase(up=False)
  ↓
GOTO_RETURN_Y _drive_minimap_axis_with_heading(HOME_Y=13.0, "y", π/2,  ROUTE_TOL, fast=True)
  ↓
GOTO_HOME_X   _drive_minimap_axis_with_heading(HOME_X=-12.7,"x", π,    ROUTE_TOL, fast=True)
  ↓
WAITING
```

### Section C (`fsm/section_c.py`, iw_hub_03)

```
WAITING
  ↓ 신호 수신
GOTO_POD      _drive_minimap_axis_with_heading(POD_Y=-8.9,  "y", π/2,  POD_TOL)
  ↓
LIFTING       _run_lift_phase(up=True)
  ↓
GOTO_SPAWN_Y  _drive_minimap_axis_no_turn(HOME_Y=-11.0, "y", ROUTE_TOL) ← 후진, yaw 유지
  ↓
GOTO_SLOT_X   _drive_minimap_axis_with_heading(DROP_X=-3.5,  "x", 0.0,  ROUTE_TOL, fast=True)
  ↓
GOTO_SLOT_Y   _drive_minimap_axis_with_heading(SLOT_Y=-13.0, "y", -π/2, ROUTE_TOL)
  ↓
LOWERING      _run_lift_phase(up=False)
  ↓
GOTO_RETURN_Y _drive_minimap_axis_with_heading(HOME_Y=-11.0, "y", π/2,  ROUTE_TOL, fast=True)
  ↓
GOTO_HOME_X   _drive_minimap_axis_with_heading(HOME_X=-9.7,  "x", π,    ROUTE_TOL, fast=True)
  ↓
WAITING
```

---

## 상태 추가 방법

1. `fsm/section_a.py` (또는 `section_c.py`) 에서 원하는 위치에 `elif` 블록 추가

```python
elif self._sa_state == "NEW_STATE":
    if self._drive_minimap_axis_with_heading(TARGET, "y", HEADING, TOL):
        self._sa_state = "NEXT_STATE"
        self._fsm_step = 0
        print(f"[{self.name}] NEW_STATE → NEXT_STATE")
```

2. 이전 상태의 전환 목적지를 `"NEW_STATE"`로 변경

```python
elif self._sa_state == "PREV_STATE":
    if self._run_lift_phase(up=True):
        self._sa_state = "NEW_STATE"   # ← 여기
        ...
```

3. 재빌드 불필요 — Isaac Sim 재시작만으로 적용됨

---

## 주요 튜닝 상수 (`iw_hub_agent.py`)

| 상수 | 기본값 | 설명 |
|------|--------|------|
| `MAX_W` | 0.4 rad/s | 최대 회전 속도 |
| `KP_W`  | 1.2 | 회전 P게인 |
| `FAST_ROUTE_V` | ~3.5 m/s | fast=True 시 최대 직진 속도 |
| yaw 허용오차 | 0.15 rad | 이 이하면 직진 시작 |

## 스폰/좌표 변경 방법

`main_isaac/robot_config.py` → `ROBOT_REGISTRY` 내 해당 로봇 딕셔너리 수정  
`spawn_xyz`, `spawn_yaw` 변경 시 FSM 파일 상수(`HOME_X`, `HOME_Y` 등)도 함께 수정할 것
