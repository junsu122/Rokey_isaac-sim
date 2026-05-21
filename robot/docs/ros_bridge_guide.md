# ROS2-Firebase 통신 브릿지 가이드

## 전체 통신 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                        Isaac Sim (ROS2)                         │
│                                                                 │
│  AMR ──/amr_001/odom──────────────────────────────────────────► │
│       ◄─/amr_001/goal──────────────────────────────────────────  │
│                                                                 │
│  Drone ──/drone_001/odom─────────────────────────────────────►  │
│         ◄─/drone_001/pose_command──────────────────────────────  │
│                                                                 │
│  M0609 ──/m0609/joint_states──────────────────────────────────► │
│         ──/m0609/end_effector_pose────────────────────────────► │
│         ◄─/m0609/task_command─────────────────────────────────  │
│         ──/m0609/task_done────────────────────────────────────► │
└────────────────────────────┬────────────────────────────────────┘
                             │ ROS2 Topics
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                   ros_bridge/run_bridge.py                      │
│                                                                 │
│   AMRBridge    ──────────────────────────────────────────────── │
│   DroneBridge  ──────────────────────────────────────────────── │
│   ArmBridge    ──────────────────────────────────────────────── │
│   ArucoBridge  ──────────────────────────────────────────────── │
└────────────────────────────┬────────────────────────────────────┘
                             │ Firebase Admin SDK
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                   Firebase Firestore                            │
│                                                                 │
│   robots/    items/    tasks/    navigation/    sections/       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 파일 구조

```
ros_bridge/
├── __init__.py
├── amr_bridge.py        ← AMR 양방향 브릿지
├── drone_bridge.py      ← 드론 양방향 브릿지
├── arm_bridge.py        ← M0609 암 양방향 브릿지
├── aruco_bridge.py      ← ArUco 검출 결과 ROS2 발행
└── run_bridge.py        ← 전체 실행 엔트리포인트

config/
└── ros_topics.yaml      ← 토픽 이름 설정
```

---

## ROS2 토픽 목록

### Isaac Sim → Firebase (구독)

| 토픽 | 메시지 타입 | 용도 |
|------|------------|------|
| `/amr_001/odom` | `nav_msgs/Odometry` | AMR 위치/속도 → Firebase |
| `/amr_001/battery_state` | `sensor_msgs/BatteryState` | AMR 배터리 → Firebase |
| `/amr_001/cmd_vel` | `geometry_msgs/Twist` | AMR 속도 로깅 |
| `/drone_001/odom` | `nav_msgs/Odometry` | 드론 위치/고도 → Firebase |
| `/drone_001/battery_state` | `sensor_msgs/BatteryState` | 드론 배터리 → Firebase |
| `/m0609/joint_states` | `sensor_msgs/JointState` | 암 관절값 → Firebase |
| `/m0609/end_effector_pose` | `geometry_msgs/PoseStamped` | 엔드이펙터 위치 → Firebase |
| `/m0609/task_done` | `std_msgs/String` (JSON) | 작업 완료 신호 → Firebase |

### Firebase → Isaac Sim (발행)

| 토픽 | 메시지 타입 | 트리거 조건 |
|------|------------|------------|
| `/amr_001/goal` | `geometry_msgs/PoseStamped` | `navigation/amr_001.status = "navigating"` |
| `/amr_001/firebase_status` | `std_msgs/String` (JSON) | navigation 문서 변경 시 |
| `/drone_001/pose_command` | `geometry_msgs/PoseStamped` | 드론 이동 명령 시 |
| `/drone_001/firebase_status` | `std_msgs/String` (JSON) | 로봇 상태 변경 시 |
| `/m0609/task_command` | `std_msgs/String` (JSON) | `tasks/` 에 pending 작업 생성 시 |
| `/m0609/firebase_status` | `std_msgs/String` (JSON) | 로봇 상태 변경 시 |
| `/aruco/detections` | `std_msgs/String` (JSON) | ArUco 마커 인식 시 |

---

## 메시지 형식

### `/m0609/task_command` (발행)
```json
{
  "task_id":     "task_49051116",
  "item_id":     "ITEM-2836C439",
  "marker_id":   0,
  "destination": "A-1",
  "action":      "pick"
}
```

### `/m0609/task_done` (수신)
```json
{
  "task_id": "task_49051116",
  "result":  "success"
}
```

### `/aruco/detections` (발행)
```json
{
  "marker_id":     0,
  "role":          "item",
  "label":         "Apple Watch",
  "position_xyz":  [0.1, 0.0, 0.5],
  "target_section": "A-1",
  "destination":   "Gangnam",
  "timestamp":     1716012345.123
}
```

### `/amr_001/firebase_status` (발행)
```json
{
  "status": "navigating",
  "target": "A-1"
}
```

---

## 실행 방법

### 1. 브릿지 전체 실행
```bash
# 터미널 1: ROS2-Firebase 브릿지
python3 ros_bridge/run_bridge.py

# 터미널 2: ArUco 감지 + Firebase + ROS2 발행
python3 robot_main.py --webcam 0 --ros
```

### 2. 특정 로봇만 실행
```bash
python3 ros_bridge/run_bridge.py --amr-only
python3 ros_bridge/run_bridge.py --arm-only
python3 ros_bridge/run_bridge.py --drone-only
```

### 3. 업데이트 주기 조정
```bash
# Firebase 업데이트 최소 간격을 1초로 설정 (기본값: 0.5초)
python3 ros_bridge/run_bridge.py --update-interval 1.0
```

---

## Isaac Sim OmniGraph 설정

Isaac Sim에서 ROS2 Bridge를 활성화하려면 OmniGraph에서 아래 노드를 추가해야 합니다.

### AMR 설정
```
ROS2 Publish Odometry
  → topic name: /amr_001/odom
  → frameId: odom

ROS2 Subscribe PoseStamped
  ← topic name: /amr_001/goal
```

### M0609 설정
```
ROS2 Publish Joint State
  → topic name: /m0609/joint_states
  → targetPrims: /World/m0609

ROS2 Subscribe String
  ← topic name: /m0609/task_command

ROS2 Publish String
  → topic name: /m0609/task_done
```

### 드론 설정
```
ROS2 Publish Odometry
  → topic name: /drone_001/odom

ROS2 Subscribe PoseStamped
  ← topic name: /drone_001/pose_command
```

---

## 데이터 흐름 예시

### 물품 인식 후 AMR 이동까지

```
1. ArUco 카메라가 ID=0 (Apple Watch) 인식
       ↓
2. robot_main.py → on_detected()
       ↓ Firebase 쓰기
3. navigation/amr_001
     status = "navigating"
     current_target = "A-1"
       ↓ Firestore on_snapshot 발동
4. AMRBridge._on_nav_change()
       ↓ ROS2 발행
5. /amr_001/goal  →  PoseStamped(x=-0.4, y=0.3)
       ↓ Isaac Sim 수신
6. AMR가 (-0.4, 0.3)으로 이동 시작

7. A-1 섹션 마커 (ID=10) 인식
       ↓ Firebase 쓰기
8. navigation/amr_001  status = "arrived"
   robots/amr_001       cargo_status = "unloading"
```

### M0609 작업 처리

```
1. tasks/ 에 pending 작업 생성
     task_xxx  robot_id="m0609"  destination="A-1"
       ↓ Firestore on_snapshot 발동
2. ArmBridge._on_task_change()
       ↓ ROS2 발행
3. /m0609/task_command  →  {"action":"pick", "task_id":"task_xxx", ...}
       ↓ Isaac Sim 수신
4. M0609가 픽업 동작 수행

5. 완료 후 Isaac Sim 발행
   /m0609/task_done  →  {"task_id":"task_xxx", "result":"success"}
       ↓ ArmBridge._on_task_done()
6. tasks/task_xxx  status = "completed"
   robots/m0609    status = "idle"
```
