# Firestore 데이터베이스 구조

Firebase 프로젝트: `rokey-factory-base`

---

## 전체 컬렉션 구성

```
Firestore
├── robots/         ← 로봇 실시간 상태 (3대)
├── sections/       ← 창고 구획 마스터 (A-1, B-2 등)
├── products/       ← 물품 마스터 (ArUco 마커와 연결)
├── items/          ← 배송 인스턴스 (물품이 이동하는 과정 추적)
├── tasks/          ← 로봇에게 내리는 작업 지시서
└── navigation/     ← AMR 이동 목표 및 도착 확인
```

---

## 1. robots/

로봇 3대의 실시간 상태를 저장합니다.

### 문서 ID
| ID | 종류 |
|---|---|
| `amr_001` | 자율주행 로봇 (AMR) |
| `drone_001` | 드론 |
| `m0609` | 두산 협동로봇 암 |

---

### robots/amr_001 (자율주행 로봇)

```
robots/amr_001
  ├─ robot_id        : "amr_001"
  ├─ type            : "amr"
  ├─ battery         : 100.0              ← 배터리 잔량 (%)
  ├─ charge_status   : "operating"        ← 충전 상태
  ├─ cargo_status    : "empty"            ← 물품 적재 상태
  ├─ position        : {x, y, yaw}        ← 현재 위치
  ├─ speed           : 0.0                ← 현재 속도 (m/s)
  ├─ current_task    : "task_xxx"         ← 담당 중인 작업 ID
  ├─ error_code      : null
  ├─ last_updated    : timestamp
  │
  ├─ localization                         ← 가장 최근 ArUco 위치 인식 결과
  │    ├─ marker_id     : 10
  │    ├─ label         : "Section A-1"
  │    ├─ estimated_pos : {x, y, yaw}
  │    ├─ distance      : 0.452           ← 마커까지 거리 (m)
  │    └─ detected_at   : timestamp
  │
  └─ localization_history : [...]         ← 위치 인식 이력 (최대 10건)
```

**charge_status 값**
| 값 | 의미 |
|---|---|
| `operating` | 운행 중 |
| `charging` | 충전 중 |

**cargo_status 값**
| 값 | 의미 |
|---|---|
| `empty` | 빈 카트 |
| `loading` | 수납 중 (M0609가 박스 올리는 중) |
| `transporting` | 목적지를 향해 이동 중 |
| `unloading` | 목적지에서 물품 내려놓는 중 |

---

### robots/drone_001 (드론)

```
robots/drone_001
  ├─ robot_id        : "drone_001"
  ├─ type            : "drone"
  ├─ battery         : 100.0
  ├─ charge_status   : "operating"
  ├─ cargo_status    : "empty"
  ├─ position        : {x, y, z}          ← 3D 위치
  ├─ altitude        : 0.0                ← 현재 고도 (m)
  ├─ heading         : 0.0                ← 방향 (0~360도, 0=북)
  ├─ speed           : 0.0
  ├─ current_task    : null
  ├─ localization    : { ... }            ← AMR과 동일한 구조
  └─ localization_history : [...]
```

---

### robots/m0609 (두산 협동로봇 암)

```
robots/m0609
  ├─ robot_id        : "m0609"
  ├─ type            : "arm"
  ├─ status          : "idle"             ← 로봇 동작 상태
  ├─ gripper         : "open"             ← 그리퍼 상태
  ├─ position        : {x, y, z}          ← 엔드이펙터 위치
  ├─ joints          : [0,0,0,0,0,0]      ← 6축 관절값 (degrees)
  ├─ battery         : 100.0
  ├─ current_task    : null
  ├─ error_code      : null
  ├─ last_updated    : timestamp
  │
  ├─ detected_item                        ← 가장 최근 ArUco 인식 물품
  │    ├─ marker_id   : 0
  │    ├─ label       : "Apple Watch"
  │    ├─ category    : "item"
  │    ├─ position_xyz: [0.1, 0.0, 0.5]
  │    ├─ item_id     : "ITEM-xxxx"
  │    └─ detected_at : timestamp
  │
  └─ detection_history : [...]            ← 인식 이력 (최대 10건)
```

**status 값**
| 값 | 의미 |
|---|---|
| `idle` | 대기 중 |
| `picking` | 물품 집는 중 |
| `placing` | 물품 내려놓는 중 |
| `moving` | 이동 중 |
| `error` | 에러 |

---

## 2. sections/

창고 구획(위치)의 마스터 데이터입니다. `setup_inventory.py` 실행 시 yaml에서 읽어 등록됩니다.

```
sections/A-1
  ├─ section_id    : "A-1"
  ├─ description   : "Section A-1"
  ├─ position      : {x: -0.4, y: 0.3, z: 0.0}   ← 실제 공간 좌표
  ├─ capacity      : 5                              ← 최대 수용 물품 수
  ├─ current_count : 0                              ← 현재 보관 물품 수
  ├─ is_active     : true
  └─ created_at    : timestamp
```

**등록된 구획 목록**
| section_id | 위치 (x, y) | ArUco 마커 ID |
|---|---|---|
| A-1 | (-0.4, 0.3) | 10 |
| A-2 | (0.0, 0.3) | 11 |
| A-3 | (0.4, 0.3) | 12 |
| B-1 | (-0.4, -0.3) | 13 |
| B-2 | (0.0, -0.3) | 14 |

---

## 3. products/

ArUco 마커와 연결된 물품 마스터 데이터입니다.

```
products/PROD-xxxxxx
  ├─ product_id  : "PROD-6644C8"
  ├─ name        : "Apple Watch"
  ├─ marker_id   : 0                  ← ArUco 마커 ID
  ├─ section     : "A-1"              ← 정렬 구획 (중간 경유지)
  ├─ destination : "Gangnam"          ← 최종 배송지
  ├─ weight      : 0.0
  ├─ size        : {w: 0.08, d: 0.08, h: 0.08}
  ├─ description : "Apple Watch — dest: Gangnam"
  ├─ is_active   : true
  └─ created_at  : timestamp
```

**등록된 물품 목록**
| 마커 ID | 물품명 | 정렬 구획 | 최종 배송지 |
|---|---|---|---|
| 0 | Apple Watch | A-1 | Gangnam |
| 3 | AirPods | A-1 | Gangnam |
| 1 | Galaxy Tab | A-2 | Seocho |
| 4 | Kindle | A-2 | Seocho |
| 2 | MacBook Pro | A-3 | Guro Digital |

> 같은 배송지로 가는 물품은 같은 구획에 정렬됩니다. B-1, B-2는 미사용 (물품 추가 시 활용).

---

## 4. items/

물품이 카메라에 인식된 순간부터 배송 완료까지의 과정을 추적하는 "배송 인스턴스"입니다.
마커가 인식될 때마다 자동으로 생성됩니다.

```
items/ITEM-xxxxxxxx
  ├─ item_id       : "ITEM-2836C439"
  ├─ product_id    : "PROD-6644C8"    ← products/ 참조
  ├─ name          : "Apple Watch"
  ├─ marker_id     : 0
  ├─ section       : "A-1"            ← 정렬 구획 (중간 경유지)
  ├─ destination   : "Gangnam"        ← 최종 배송지
  ├─ status        : "detected"       ← 현재 배송 상태
  ├─ position_xyz  : [0.1, 0.0, 0.5] ← 마지막 감지 위치
  ├─ assigned_robot: "m0609"
  ├─ current_task  : "task_xxx"
  ├─ registered_at : timestamp
  ├─ detected_at   : timestamp
  └─ delivered_at  : null             ← 배송 완료 시 기록
```

**status 흐름**
```
detected  →  in_transit  →  delivered
                         →  returned (반품)
```

| status | 의미 |
|---|---|
| `detected` | 카메라가 마커 인식, 배송 시작 전 |
| `in_transit` | 로봇이 운반 중 |
| `delivered` | 배송 완료 |
| `returned` | 반품 처리 |

---

## 5. tasks/

로봇에게 내리는 작업 지시서입니다.
물품 마커가 인식되면 **M0609(픽업)** 과 **AMR(배달)** 두 개의 작업이 동시에 생성됩니다.

```
tasks/task_xxxxxxxx (M0609 픽업 작업)
  ├─ task_id      : "task_49051116"
  ├─ item_id      : "ITEM-2836C439"  ← items/ 참조
  ├─ marker_id    : 0
  ├─ destination  : "A-1"            ← 정렬 구획 (물품을 올려놓을 위치)
  ├─ robot_id     : "m0609"
  ├─ status       : "pending"
  ├─ created_at   : timestamp
  ├─ started_at   : null
  └─ completed_at : null

tasks/task_xxxxxxxx (AMR 배달 작업)
  ├─ task_id      : "task_80d3b2d7"
  ├─ item_id      : "ITEM-2836C439"  ← items/ 참조
  ├─ marker_id    : 0
  ├─ destination  : "Gangnam"        ← 최종 배송지
  ├─ robot_id     : "amr_001"
  ├─ status       : "pending"
  ├─ created_at   : timestamp
  ├─ started_at   : null
  └─ completed_at : null
```

> **M0609 태스크 `destination`** = 정렬 구획 (`A-1`)  — 물품을 올려놓을 중간 위치
> **AMR 태스크 `destination`** = 최종 배송지 (`Gangnam`) — 물품을 최종적으로 전달할 위치

**status 흐름**
```
pending  →  in_progress  →  completed
                         →  failed
```

| status | 의미 |
|---|---|
| `pending` | 로봇 할당 대기 중 |
| `in_progress` | 로봇이 수행 중 |
| `completed` | 완료 |
| `failed` | 실패 |

---

## 6. navigation/

AMR의 이동 목표와 도착 확인을 관리합니다.
구획(A-1 등) 뿐만 아니라 최종 배송지(Gangnam 등)도 목표로 설정할 수 있습니다.

```
navigation/amr_001
  ├─ amr_id           : "amr_001"
  ├─ current_target   : "A-1"              ← 현재 이동 목표 (구획 또는 배송지)
  ├─ target_position  : {x: -0.4, y: 0.3} ← 목표의 실제 좌표
  ├─ assigned_item_id : "ITEM-xxxx"        ← 운반 중인 물품 ID
  ├─ status           : "navigating"       ← 네비게이션 상태
  ├─ confirmed_section: null               ← 마커로 확인된 위치
  └─ updated_at       : timestamp
```

**NavigationManager가 알고 있는 위치 목록**
| 이름 | 종류 | 배송지 | 좌표 (x, y) | ArUco 마커 ID |
|---|---|---|---|---|
| A-1 | 정렬 구획 | Gangnam | (-0.4, 0.3) | 10 |
| A-2 | 정렬 구획 | Seocho | (0.0, 0.3) | 11 |
| A-3 | 정렬 구획 | Guro Digital | (0.4, 0.3) | 12 |
| B-1 | 정렬 구획 | (미사용) | (-0.4, -0.3) | 13 |
| B-2 | 정렬 구획 | (미사용) | (0.0, -0.3) | 14 |
| Gangnam | 최종 배송지 | — | (1.5, 0.5) | 20 |
| Seocho | 최종 배송지 | — | (1.5, 0.0) | 21 |
| Guro Digital | 최종 배송지 | — | (1.5, -0.5) | 22 |

**status 흐름**
```
idle  →  navigating  →  arrived  →  (unloading 후) idle
```

| status | 의미 |
|---|---|
| `idle` | 대기 중 |
| `navigating` | 목표를 향해 이동 중 |
| `arrived` | 마커 인식으로 도착 확인됨 |

---

## 전체 흐름 요약

```
1. 카메라가 item 마커 인식 (예: ID=0, Apple Watch)
       │
       ├── items/          문서 생성    section="A-1"  destination="Gangnam"
       ├── tasks/          문서 생성 ×2
       │     task_A  robot_id="m0609"   destination="A-1"      (ARM 픽업)
       │     task_B  robot_id="amr_001" destination="Gangnam"  (AMR 최종 배달)
       ├── robots/m0609        status = "picking"
       ├── robots/amr_001      cargo_status = "loading"
       └── navigation/amr_001  current_target="A-1", status="navigating"

2. AMR이 이동하다 section 마커 인식 (예: ID=10, Section A-1)
       │
       ├── robots/amr_001      localization 업데이트 (위치 보정)
       └── navigation/amr_001  status="arrived", confirmed_section="A-1"
           robots/amr_001      cargo_status="unloading"

3. destination 마커 인식 (예: ID=20, Gangnam)
       │
       ├── robots/amr_001      cargo_status="unloading"
       └── navigation/amr_001  status="idle", current_target=null

4. 배송 완료 처리
       ├── items/ITEM-xxx      status="delivered", delivered_at=timestamp
       ├── tasks/task_B        status="completed"
       └── navigation/amr_001  status="idle"
```

---

## 마커 역할 구분

| 마커 ID 범위 | 역할 | 설명 |
|---|---|---|
| 0 ~ 9 | `item` | 물품 박스에 부착 — 인식 시 tasks, items 생성 |
| 10 ~ 19 | `section` | 구획 선반/바닥에 부착 — AMR 도착 확인용 |
| 20 ~ 29 | `destination` | 최종 배송지에 부착 — 최종 배송 완료 확인용 |

---

## 데이터 모니터링

```bash
# 전체 현황 한 번 출력
python3 DB/monitor.py

# 실시간 모니터링 (변경될 때마다 출력)
python3 DB/monitor.py --watch

# 항목별 조회
python3 DB/monitor.py --robots
python3 DB/monitor.py --items
python3 DB/monitor.py --tasks
python3 DB/monitor.py --stats

# 인식/위치 이력 조회
python3 DB/monitor.py --history arm m0609
python3 DB/monitor.py --history amr amr_001
```

## 데이터 초기화

```bash
# 전체 초기화 후 재등록
python3 DB/reset_inventory.py
python3 DB/setup_inventory.py
```
