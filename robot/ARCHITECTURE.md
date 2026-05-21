# Rokey Factory — 시스템 아키텍처

## 전체 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Isaac Sim (시뮬레이션)                        │
│                                                                     │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌────────────────┐  │
│  │  AMR     │   │  드론    │   │ M0609 암 │   │  카메라        │  │
│  │ amr_001  │   │drone_001 │   │  m0609   │   │ (ArUco 감지)   │  │
│  └────┬─────┘   └────┬─────┘   └────┬─────┘   └───────┬────────┘  │
│       │ /odom        │ /pose        │ /state           │ 영상       │
│       │ /battery     │ /battery     │ /joints          │            │
└───────┼──────────────┼──────────────┼──────────────────┼────────────┘
        │              │              │                  │
        ▼              ▼              ▼                  ▼
┌───────────────────────────────────────────────────────────────────┐
│                      ROS2 토픽 레이어                               │
│                                                                   │
│  /amr_001/odom          /drone_001/pose        /m0609/state       │
│  /amr_001/battery_state /drone_001/battery     /m0609/joints      │
│  /amr_001/goal ◄──────  /drone_001/goal ◄────  /m0609/cmd ◄────   │
│  (Firebase → AMR)       (Firebase → 드론)      (Firebase → 암)    │
└───────────────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────────────┐
│                   ROS2-Firebase 브릿지 (run_bridge.py)              │
│                                                                   │
│  AMRBridge     ─── /amr_001/odom 구독   → Firestore 위치 저장      │
│  DroneBridge   ─── /drone_001/pose 구독 → Firestore 위치 저장      │
│  ArmBridge     ─── /m0609/state 구독    → Firestore 상태 저장      │
│                                                                   │
│  (양방향) Firestore navigation/ 변경 감지 → /goal 토픽 발행         │
│                                                                   │
│  실행:  python3 ros_bridge/run_bridge.py                           │
│  옵션:  --amr-only / --drone-only / --arm-only                     │
│         --update-interval 0.5  (Firebase 쓰기 최소 간격, 초)        │
└───────────────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────────────┐
│                  Firebase Firestore (rokey-factory-base)            │
│                                                                   │
│  robots/                      navigation/                         │
│    amr_001/                     amr_001/ ── current_target        │
│      position: {x, y, yaw}               ── status               │
│      battery, charge_status              ── goal_position         │
│      cargo_status, speed                                          │
│      current_task                       tasks/                    │
│      localization (ArUco 위치 보정)       task_xxx/ ── item_id     │
│                                                  ── destination   │
│    drone_001/                                    ── robot_id      │
│      position: {x, y, z}                         ── status       │
│      altitude, heading, speed                                     │
│      battery, cargo_status             sections/                  │
│      current_task                        A-1/ ── position {x,y,z}│
│      localization                        A-2/    ── section_id    │
│                                          A-3/                     │
│    m0609/                                B-1/                     │
│      status (idle/picking/placing)       B-2/                     │
│      gripper (open/closed)                                        │
│      position: {x, y, z}              inventory/                  │
│      joints: [j1~j6]                    Apple Watch               │
│      detected_item (ArUco 인식 결과)     Galaxy Tab                │
│      current_task                        MacBook Pro              │
│                                          AirPods                  │
│                                          Kindle                   │
└───────────────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────────────┐
│              모니터링 대시보드 (UI)                  │
│              React + TypeScript + Vite  →  http://localhost:5173   │
│                                                                   │
│  useRobotFleet()  ── Firestore onSnapshot ── 실시간 갱신            │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  창고 평면도 (2D SVG 맵)                                       │ │
│  │                                                              │ │
│  │  [ A-1 ]  [ A-2 ]  [ A-3 ]        ┌──────┐                 │ │
│  │                          [암고정]   │ 강남 │                 │ │
│  │  [ B-1 ]  [ B-2 ]                  ├──────┤                 │ │
│  │                                    │ 서초 │                 │ │
│  │    🚗 AMR 위치 (실시간)             ├──────┤                 │ │
│  │    🚁 드론 위치 (실시간)            │ 구로 │                 │ │
│  │    🦾 암 (고정, x=1.0 y=0.0)       └──────┘                 │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │
│  │ AMR 카드    │  │ 드론 카드   │  │ M0609 로봇암 카드        │   │
│  │ 배터리      │  │ 배터리      │  │ 배터리                   │   │
│  │ 상태 뱃지   │  │ 상태 뱃지   │  │ 상태 뱃지               │   │
│  │ 보유 물품   │  │ 보유 물품   │  │ 인식 물품 (ArUco)        │   │
│  │ 현재 작업   │  │ 현재 작업   │  │ 그리퍼 상태             │   │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘   │
└───────────────────────────────────────────────────────────────────┘
```

---

## ArUco 마커 역할 분류

| 마커 ID | 역할 | 내용 |
|--------|------|------|
| 0 ~ 4  | `item` | 제품 박스 부착 — Apple Watch, Galaxy Tab, MacBook Pro, AirPods, Kindle |
| 10 ~ 14 | `section` | 선반/바닥 부착 — A-1, A-2, A-3, B-1, B-2 |
| 20 ~ 22 | `destination` | 배송지 마킹 — 강남, 서초, 구로 디지털 |

마커 인식 흐름:
```
카메라 영상
  → aruco_detector.py (OpenCV)
  → on_detected() 콜백
  → item 마커:  TaskManager.create()  +  ArmManager.set_detected_item()
  → section 마커: AMRManager.set_localization() (위치 보정)
  → destination 마커: TaskManager.complete()
```

---

## 로봇별 동작 시나리오

```
1. 카메라가 item 마커(예: MacBook Pro) 인식
2. TaskManager → tasks/ 에 작업 생성 (destination: "Guro Digital")
3. ArmBridge → M0609 암 픽업 시작 (status: picking)
4. AMR에 이동 명령 → navigation/amr_001 업데이트
5. AMRBridge → /amr_001/goal 토픽 발행
6. AMR 이동 중 section 마커(A-3) 인식 → 위치 보정
7. AMR 목적지 도착 → destination 마커(구로) 인식
8. TaskManager.complete() → 작업 완료
9. 대시보드에서 전 과정 실시간 모니터링
```

---

## 주요 파일 구조

```
robot/
├── robot_main.py          # 메인 진입점 (ArUco 감지 루프)
├── config/
│   ├── object_registry.yaml     # 마커 역할/위치 정의 (단일 진실 소스)
│   └── serviceAccountKey.json   # Firebase Admin SDK 키 (git 제외)
├── utils/
│   ├── aruco_detector.py        # OpenCV ArUco 감지
│   ├── isaac_camera.py          # Isaac Sim 카메라 연동
│   └── video_camera.py          # 웹캠 연동
├── DB/
│   ├── firebase_manager.py      # Firebase 초기화
│   ├── robot_status.py          # AMR/드론/암 상태 관리 (Firestore)
│   ├── task_manager.py          # 작업 생성/완료 관리
│   ├── navigation.py            # AMR 이동 명령 관리
│   ├── inventory.py             # 재고 관리
│   └── item_tracker.py          # 물품 추적
├── ros_bridge/
│   ├── run_bridge.py            # 브릿지 통합 실행 진입점
│   ├── amr_bridge.py            # AMR ↔ Firebase 양방향 브릿지
│   ├── drone_bridge.py          # 드론 ↔ Firebase 양방향 브릿지
│   ├── arm_bridge.py            # 암 ↔ Firebase 양방향 브릿지
│   └── aruco_bridge.py          # ArUco 인식 결과 ROS 브릿지
└── UI/         # 모니터링 대시보드
    ├── src/
    │   ├── firebase.ts          # Firebase 웹 SDK 초기화
    │   ├── hooks/useRobotFleet.ts  # Firestore 실시간 리스너
    │   └── components/
    │       ├── WarehouseMap.tsx  # 창고 2D SVG 맵
    │       ├── RobotCard.tsx    # 로봇 상태 카드
    │       ├── BatteryBar.tsx   # 배터리 바
    │       └── StatusBadge.tsx  # 상태 뱃지
    └── .env.local               # Firebase 웹 설정값 (git 제외)
```

---

## 실행 명령

```bash
# 1. Firebase 재고 초기화 (최초 1회)
python3 DB/reset_inventory.py && python3 DB/setup_inventory.py

# 2. ROS2-Firebase 브릿지 실행 (Isaac Sim 실행 후)
python3 ros_bridge/run_bridge.py
python3 ros_bridge/run_bridge.py --amr-only          # AMR만
python3 ros_bridge/run_bridge.py --update-interval 1.0  # 쓰기 간격 조정

# 3. 모니터링 대시보드 실행
cd UI && npm run dev
# → http://localhost:5173

# 4. Firestore 실시간 모니터링 (터미널용)
python3 DB/monitor.py --watch

# 5. 웹캠 테스트 (Isaac Sim 없이)
python3 robot_main.py --webcam 0
```

---

## 위치 좌표 수정 방법

실제 환경 구성 후 아래 파일의 상수만 변경하면 됩니다.

| 수정 대상 | 파일 | 상수 |
|---------|------|------|
| 섹션 위치 (A-1 ~ B-2) | `UI/src/components/WarehouseMap.tsx` | `SECTIONS` 배열 |
| 배송지 위치 | `UI/src/components/WarehouseMap.tsx` | `DESTINATIONS` 배열 |
| 로봇 암 고정 위치 | `UI/src/components/WarehouseMap.tsx` | `ARM_WORLD` |
| 마커별 실제 좌표 | `config/object_registry.yaml` | 각 마커의 `position` |
