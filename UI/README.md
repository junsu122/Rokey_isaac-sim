# UI — 물류센터 모니터링 대시보드

React 19 + TypeScript + Vite + Firebase Realtime 기반 웹 대시보드.

## 실행

```bash
cd UI
npm install
npm run dev       # 개발 서버 (http://localhost:5173)
npm run build     # 프로덕션 빌드
```

`.env.local` 에 Firebase 설정이 있어야 한다.

---

## 탭 구성

| 탭 | 내용 |
|---|---|
| 전체 현황 | Section A/B/C 카드 + Pod 전체 그리드 |
| 창고 맵 | SVG 탑뷰 맵 — Pod 위치(상태별 색상), iw_hub 실시간 위치, M0609 고정 위치 |
| Pod 현황 | 섹션 탭별 Pod 상태 그리드 (호버 시 좌표 툴팁) |
| 로봇 현황 | 섹션별 M0609 + iw_hub 상태 카드 (iw_hub 실시간 좌표 포함) |

---

## 파일 구조

```
UI/src/
├── App.tsx                      # 탭 라우팅 + 헤더/푸터
├── types.ts                     # 데이터 타입 정의
├── firebase.ts                  # Firebase 초기화
├── hooks/
│   └── useSections.ts           # sections + pods 실시간 구독
└── components/
    ├── SectionCard.tsx           # 섹션 요약 카드 (로봇 상태 + Pod 바)
    ├── PodPanel.tsx              # Pod 상태 그리드 (섹션 탭 포함)
    ├── WarehouseMap.tsx          # SVG 창고 탑뷰 맵
    ├── RobotPanel.tsx            # 로봇 상태 카드 패널
    ├── StatusBadge.tsx           # 상태 뱃지 (working/wait/stop)
    └── AmazonLogo.tsx            # 헤더 로고
```

---

## 데이터 흐름

```
Firebase Firestore
  sections/{A|B|C}          → SectionCard, RobotPanel
  sections/{A|B|C}/pods/*   → PodPanel, WarehouseMap
```

`useSections` 훅이 두 컬렉션을 동시 구독하고 `SectionData[]` 로 합쳐서 반환한다.  
모든 컴포넌트는 이 훅 하나에서 데이터를 받는다.

---

## 창고 맵 좌표계

minimap.py 와 동일한 월드 좌표계를 사용한다.

- x 범위: -18 ~ 24 m
- y 범위: -18 ~ 18 m
- SVG viewBox: 840 × 720 (1 m = 20 px)

| 구역 | x | y |
|---|---|---|
| Section A | -4.9 ~ 4.9 | 6.1 ~ 13.9 |
| Section B | -4.9 ~ 4.9 | -3.9 ~ 3.9 |
| Section C | -4.9 ~ 4.9 | -13.9 ~ -6.1 |

M0609 고정 위치 (robot_config.py 기준):

| 로봇 | x | y |
|---|---|---|
| M0609_A | -12.07 | 7.92 |
| M0609_B | -9.45 | 0.79 |
| M0609_C | -10.45 | -7.80 |
