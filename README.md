# Warehouse Robotics System

Firebase + ROS2 + Isaac Sim 기반 물류센터 자동화 시스템

---

## 폴더 구조

```
code/
├── UI/           # React 모니터링 대시보드 (웹)
├── DB/           # Firebase Firestore 관리 스크립트 (Python)
├── robot/        # ArUco 마커 인식 + 로봇 제어 (Python)
├── simulation/   # Isaac Sim 물류센터 시뮬레이션
└── README.md
```

---

## 각 폴더 역할

### UI/ — 웹 대시보드
- React + TypeScript + Firebase 실시간 연동
- 로봇 위치, 배터리, 재고 현황 모니터링
- 실행: `cd UI && npm install && npm run dev` → http://localhost:5173
- 상세: [UI/README.md](UI/README.md)

### DB/ — 데이터베이스 관리
- Firebase Admin SDK 기반 Firestore 관리 스크립트
- 서비스 계정 키 필요 (`robot/config/serviceAccountKey.json`)

| 스크립트 | 역할 |
|----------|------|
| `DB/setup_inventory.py` | 초기 재고 데이터 등록 |
| `DB/seed_example_data.py` | 예시 데이터 삽입 |
| `DB/reset_inventory.py` | Firestore 전체 초기화 |
| `DB/monitor.py` | 실시간 터미널 모니터링 |
| `DB/test_connection.py` | Firebase 연결 확인 |

```bash
# code/ 루트에서 실행
python3 DB/test_connection.py
python3 DB/setup_inventory.py
python3 DB/seed_example_data.py
python3 DB/monitor.py --watch
```

### robot/ — 로봇 인식·제어
- ArUco 마커 기반 물품 분류 인식
- ROS2 브릿지 (AMR / 드론 / 협동로봇)
- Firebase 연동으로 인식 결과 Firestore에 기록

```bash
cd robot
python3 isaac_aruco_main.py --webcam 0
```

### simulation/ — Isaac Sim 시뮬레이션
- NVIDIA Isaac Sim 4.5 기반 물류센터 가상 환경
- iw.hub AMR Action Graph 포함
- 상세: [simulation/README.md](simulation/README.md)

---

## 빠른 시작

### 1. Firebase 서비스 계정 키 설정
```bash
# Firebase 콘솔 → 프로젝트 설정 → 서비스 계정 → 새 비공개 키 생성
# 다운로드한 JSON 파일을 아래 경로에 배치
robot/config/serviceAccountKey.json
```

### 2. DB 초기화
```bash
python3 DB/test_connection.py      # 연결 확인
python3 DB/setup_inventory.py      # 초기 데이터 등록
python3 DB/seed_example_data.py    # 예시 데이터 삽입
```

### 3. 웹 대시보드 실행
```bash
cd UI
cp .env.example .env.local         # Firebase 웹 앱 설정값 입력
npm install
npm run dev
```

### 4. 로봇 인식 실행
```bash
cd robot
python3 isaac_aruco_main.py --webcam 0
```

---

## Firebase 링크

| 항목 | URL |
|------|-----|
| Firebase 콘솔 | https://console.firebase.google.com/project/rokey-factory-base |
| Firestore DB | https://console.firebase.google.com/project/rokey-factory-base/firestore |

---

## 기술 스택

| 분류 | 기술 |
|------|------|
| 웹 UI | React 19, TypeScript, Vite, Tailwind CSS |
| 데이터베이스 | Firebase Firestore |
| 로봇 인식 | Python, OpenCV, ArUco |
| 로봇 제어 | ROS2, Firebase Admin SDK |
| 시뮬레이션 | NVIDIA Isaac Sim 4.5 |
