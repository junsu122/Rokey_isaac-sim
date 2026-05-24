# Warehouse Dashboard

Firebase + React 기반 물류센터 실시간 모니터링 대시보드

---

## 링크

| 항목 | URL |
|------|-----|
| **UI (개발 서버)** | http://localhost:5173 |
| **UI (프리뷰 서버)** | http://localhost:4173 |
| **Firebase 콘솔** | https://console.firebase.google.com/project/rokey-factory-base |
| **Firestore 데이터베이스** | https://console.firebase.google.com/project/rokey-factory-base/firestore |
| **Firebase 프로젝트 설정** | https://console.firebase.google.com/project/rokey-factory-base/settings/general |

---

## 시작하기

### 1. 의존성 설치

```bash
npm install
```

### 2. 환경 변수 설정

```bash
cp .env.example .env.local
```

`.env.local` 파일을 열어 Firebase 콘솔에서 복사한 값으로 채워주세요.

> **Firebase 콘솔 경로:**  
> 프로젝트 설정 → 일반 → 내 앱 → 웹 앱 구성 (SDK 설정 및 구성)

```env
VITE_FIREBASE_API_KEY=...
VITE_FIREBASE_AUTH_DOMAIN=...
VITE_FIREBASE_PROJECT_ID=...
VITE_FIREBASE_STORAGE_BUCKET=...
VITE_FIREBASE_MESSAGING_SENDER_ID=...
VITE_FIREBASE_APP_ID=...
```

### 3. 개발 서버 실행

```bash
npm run dev
```

브라우저에서 http://localhost:5173 접속

---

## 주요 명령어

```bash
npm run dev      # 개발 서버 실행  →  http://localhost:5173
npm run build    # 프로덕션 빌드  →  dist/ 폴더 생성
npm run preview  # 빌드 결과 미리보기  →  http://localhost:4173
npm run lint     # 코드 린트 검사
```

---

## 프로젝트 구조

```
warehouse-dashboard/
├── src/
│   ├── components/
│   │   ├── WarehouseMap.tsx     # 창고 실시간 맵 (SVG)
│   │   ├── RobotCard.tsx        # 로봇 상태 카드
│   │   ├── InventoryPanel.tsx   # 재고 및 배송 현황
│   │   ├── BatteryBar.tsx       # 배터리 표시 바
│   │   ├── StatusBadge.tsx      # 상태 뱃지
│   │   └── AmazonLogo.tsx       # 헤더 로고
│   ├── hooks/
│   │   ├── useRobotFleet.ts     # Firestore 로봇 상태 실시간 구독
│   │   └── useInventory.ts      # Firestore 재고/배송 실시간 구독
│   ├── firebase.ts              # Firebase 초기화
│   ├── types.ts                 # TypeScript 타입 정의
│   └── App.tsx                  # 루트 컴포넌트
├── .env.example                 # 환경 변수 템플릿 (공유용)
├── .env.local                   # 실제 환경 변수 (git 제외)
└── vite.config.ts               # Vite 설정 (@/ 경로 별칭)
```

---

## Firestore 컬렉션 구조

### `robots` 컬렉션

| 문서 ID | 설명 |
|---------|------|
| `amr_001` | 자율주행 로봇 (AMR) |
| `drone_001` | 드론 |
| `m0609` | 두산 M0609 협동로봇 |

### `products` 컬렉션

물품 마스터 데이터 (`marker_id`, `name` 등)

### `items` 컬렉션

개별 물품 인스턴스 (`detected_at`, `status`, `section`, `destination` 등)

---

## 기술 스택

| 분류 | 기술 |
|------|------|
| UI 프레임워크 | React 19 + TypeScript 6 |
| 빌드 도구 | Vite 8 |
| 스타일 | Tailwind CSS v4 |
| 데이터베이스 | Firebase Firestore (실시간 구독) |
| 경로 별칭 | `@/` → `src/` |
