# Bridge Interface 설정 문서

`section_bridge.py` 토픽/인터페이스 조정 가이드.  
실제 로봇 이름 / 토픽이 바뀌면 이 문서를 먼저 수정한 뒤 코드에 반영한다.

---

## 섹션 ↔ 로봇 매핑

`section_bridge.py` 상단 `SECTION_ROBOTS` 딕셔너리로 관리한다.

```python
SECTION_ROBOTS = {
    "A": {"m0609": "M0609_A",  "iw_hub": "iw_hub_01"},
    "B": {"m0609": "M0609_B",  "iw_hub": "iw_hub_02"},
    "C": {"m0609": "M0609_C",  "iw_hub": "iw_hub_03"},
}
```

로봇 이름을 변경하면 구독 토픽 경로가 자동으로 바뀐다.

---

## ROS2 구독 토픽

### m0609 상태

| 항목 | 내용 |
|---|---|
| 토픽 | `/{m0609_name}/state` |
| 타입 | `std_msgs/String` |
| 허용 값 | `"working"` \| `"stop"` |
| Firestore 경로 | `sections/{id}/robots.m0609.state` |

**예시**
```
토픽: /M0609_A/state
데이터: "working"
```

---

### iw_hub 상태

| 항목 | 내용 |
|---|---|
| 토픽 | `/{iw_hub_name}/state` |
| 타입 | `std_msgs/String` |
| 허용 값 | `"working"` \| `"stop"` |
| Firestore 경로 | `sections/{id}/robots.iw_hub.state` |

**예시**
```
토픽: /iw_hub_01/state
데이터: "stop"
```

---

### iw_hub 위치 (odom)

| 항목 | 내용 |
|---|---|
| 토픽 | `/{iw_hub_name}/odom` |
| 타입 | `nav_msgs/Odometry` |
| 업데이트 주기 | 0.5초 (ODOM_UPDATE_INTERVAL) |
| 사용 필드 | `pose.pose.position.x`, `.y` |
| Firestore 경로 | `sections/{id}/robots.iw_hub.location` |

---

### Pod 상태 업데이트

| 항목 | 내용 |
|---|---|
| 토픽 | `/section_{id}/pod_update` |
| 타입 | `std_msgs/String` (JSON) |
| 허용 state 값 | `"full"` \| `"empty"` \| `"filling"` \| `"moving"` |
| Firestore 경로 | `sections/{id}/pods/{pod_id}` |

**JSON 형식**
```json
{
  "pod_id": "pod_01",
  "state": "full",
  "location": {"x": 1.0, "y": 2.0}
}
```
> `location` 은 선택 항목. 생략하면 위치는 유지하고 state만 업데이트.

**예시**
```
토픽: /section_A/pod_update
데이터: {"pod_id": "pod_03", "state": "filling"}
```

---

## 조정 가능한 파라미터

`section_bridge.py` 상단에서 수정한다.

| 변수 | 기본값 | 설명 |
|---|---|---|
| `ODOM_UPDATE_INTERVAL` | `0.5` | iw_hub 위치 Firestore 쓰기 최소 간격 (초) |
| `VALID_ROBOT_STATES` | `{"working", "stop"}` | m0609 / iw_hub 허용 state |
| `VALID_POD_STATES` | `{"full", "empty", "filling", "moving"}` | Pod 허용 state |

---

## 향후 추가 예정 인터페이스

| 기능 | 토픽 (예정) | 타입 | 비고 |
|---|---|---|---|
| m0609 작업 완료 횟수 | `/{m0609_name}/complete_count` | `std_msgs/Int32` | work_complete_count 연동 |
| iw_hub 목표 웨이포인트 | `/{iw_hub_name}/waypoint` | `std_msgs/String` | STACK/WAIT/UNLOAD |
| Pod 위치 갱신 (iw_hub odom 연동) | 자동 계산 | — | odom에서 추론 |
| Firestore → ROS2 명령 발행 | — | — | 역방향 브릿지 추가 시 |
