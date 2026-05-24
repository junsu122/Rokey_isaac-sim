# Firestore Schema

---

## 컬렉션 구조

```
sections/               ← 섹션별 전체 상태 (A, B, C)
└── {section_id}/
    ├── (문서 필드)
    └── pods/           ← 섹션 내 Pod 목록 (서브컬렉션)
        └── {pod_id}
```

---

## sections/{section_id}

섹션 하나의 상태. section_id = `"A"` | `"B"` | `"C"`

```
sections/A
  ├── section_id     : "A"
  ├── package_size   : "Big" | "Medium" | "Small"
  ├── pod_amount     : 20                        ← 총 Pod 수
  │
  ├── robots
  │    ├── m0609
  │    │    └── state    : "working" | "stop"
  │    │
  │    └── iw_hub
  │         ├── state    : "working" | "stop"
  │         └── location : {x: 0.0, y: 0.0}     ← 현재 좌표 (m)
  │
  └── last_updated   : timestamp
```

---

## sections/{section_id}/pods/{pod_id}

Pod 하나의 상태. pod_id = `"pod_01"` ~ `"pod_20"` 등

```
sections/A/pods/pod_01
  ├── pod_id    : "pod_01"
  ├── state     : "full" | "empty" | "filling" | "moving"
  └── location  : {x: 0.0, y: 0.0}              ← 현재 좌표 (m)
```

**state 의미**

| state | 의미 |
|---|---|
| `empty` | Pod 비어 있음 |
| `filling` | M0609가 물품 채우는 중 |
| `full` | 물품 가득 참 |
| `moving` | iw_hub가 이송 중 |

---

## 섹션별 고정 할당

| Section | m0609 | iw_hub | package_size |
|---|---|---|---|
| A | M0609_A | iw_hub_A | Big |
| B | M0609_B | iw_hub_B | Medium |
| C | M0609_C | iw_hub_C | Small |

> 각 로봇은 소속 Section 에만 귀속되며 다른 Section 으로 이동하지 않는다.
