# DB — Firebase 연동 노드

Isaac Sim ↔ Firebase Firestore 간 데이터 동기화를 담당하는 Python 스크립트 모음.

---

## 파일 구성

| 파일 | 역할 | 실행 시점 |
|---|---|---|
| `init_db.py` | Firestore 초기 데이터 기록 (섹션·Pod 생성) | 시뮬 시작 전 1회 |
| `reset_and_setup.py` | 기존 컬렉션 삭제 후 init_db 재실행 | 완전 초기화 시 |
| `trigger_bridge.py` | ROS2 ↔ Firestore 통합 브릿지 | 시뮬 실행 중 상시 |

---

## 실행 순서

```bash
# 1. DB 초기화 (시뮬 최초 실행 또는 초기화 필요 시)
python3 DB/init_db.py

# 2. 브릿지 노드 실행 (시뮬과 함께 상시 구동)
source /opt/ros/humble/setup.bash
python3 DB/trigger_bridge.py

# 3. 시뮬레이션 실행
./run_sim.sh
```

---

## Firestore 스키마

```
sections/{A|B|C}
  ├── section_id        : "A" | "B" | "C"
  ├── package_size      : "Big" | "Medium" | "Small"
  ├── pod_amount        : 20
  ├── robots
  │   ├── m0609  : { robot_name, state: "working"|"wait" }
  │   └── iw_hub : { robot_name, state: "working"|"wait", location: {x, y} }
  └── last_updated

sections/{A|B|C}/pods/{pod_01 ~ pod_20}
  ├── pod_id   : "pod_01" ~ "pod_20"
  ├── state    : "empty" | "filling" | "full" | "moving"
  └── location : { x, y }
```

---

## trigger_bridge.py 동작

### 구독 (ROS2 → Firestore)

| 토픽 | 메시지 | 처리 |
|---|---|---|
| `/{m0609}/work` | `"A_complete"` 등 | pod: filling → full, m0609 state → wait |
| `/{iw_hub}/odom` | Odometry | phase별 pod state 전환 |
| `/{iw_hub}/work_done` | String | m0609 재개 신호 발행 |

### 발행 (Firestore 트리거 → ROS2)

| 토픽 | 메시지 | 조건 |
|---|---|---|
| `/{M0609}/work` | `"work_start"` | iw_hub swap 완료 후 |
| `/{iw_hub}/command` | JSON (swap) | full pod 감지 시 |

### Pod 상태 전환 (iw_hub odom 기반)

```
filling ──[m0609 complete]──▶ full
full    ──[iw_hub belt 도착]──▶ moving
moving  ──[iw_hub 격자 도착]──▶ full  (location 업데이트)
empty   ──[iw_hub empty 픽업]──▶ moving
moving  ──[iw_hub belt 재도착]──▶ filling  (location = belt)
```

### swap 커맨드 포맷

```json
{
  "action":           "swap",
  "full_pod_id":      "pod_03",
  "pickup_pos":       { "x": -12.8, "y": 9.0 },
  "return_pos":       { "x": 0.75,  "y": 7.3 },
  "empty_pod_id":     "pod_07",
  "empty_pickup_pos": { "x": -0.75, "y": 8.7 },
  "deliver_pos":      { "x": -12.8, "y": 9.0 }
}
```

---

## 섹션별 로봇 매핑

| Section | M0609 | iw_hub | package_size |
|---|---|---|---|
| A | M0609_A | iw_hub_01 | Big |
| B | M0609_B | iw_hub_02 | Medium |
| C | M0609_C | iw_hub_03 | Small |

---

## 미확정 좌표 (TODO)

아래 값은 실제 시뮬 실행 후 확인 필요:

```python
CONVEYOR_WAIT_POS = {
    "A": {"x": -12.8, "y":  9.0},
    "B": {"x":  -7.9, "y":  1.5},
    "C": {"x":  -9.7, "y": -8.6},
}
WAIT_RADIUS         = 0.5   # iw_hub wait 판정 반경
GRID_ARRIVAL_RADIUS = 0.3   # pod 격자 도착 판정 반경
```
