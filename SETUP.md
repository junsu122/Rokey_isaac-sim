# 개발 환경 세팅 가이드

다른 PC에서 이 프로젝트를 시작할 때 참고한다.

---

## 전제 조건

| 항목 | 버전 |
|---|---|
| NVIDIA Isaac Sim | 5.1.0 |
| ROS2 | Humble |
| Python | 3.11 (Isaac Sim 내장) / 3.10 (시스템) |
| Node.js | 18+ |
| OS | Ubuntu 22.04 |

---

## 1. 레포지토리 클론

```bash
cd ~/dev_ws/isaac_sim/src
git clone https://github.com/junsu122/Rokey_isaac-sim.git
cd Rokey_isaac-sim
git checkout integration
```

---

## 2. Firebase 서비스 계정 키 설정

> `serviceAccountKey.json`은 보안 파일이라 git에 포함되지 않는다.  
> Firebase 콘솔에서 직접 발급받아야 한다.

```
Firebase 콘솔 → rokey-factory-base → 프로젝트 설정
→ 서비스 계정 → 새 비공개 키 생성 → JSON 다운로드
```

다운로드한 파일을 아래 경로에 배치:

```bash
cp ~/Downloads/rokey-factory-base-*.json DB/serviceAccountKey.json
```

---

## 3. Python 패키지 설치

```bash
# firebase-admin (DB 스크립트 / 브릿지 노드용)
sudo pip3 install firebase-admin
```

---

## 4. Firestore 초기화 (최초 1회)

```bash
cd ~/dev_ws/isaac_sim/src/Rokey_isaac-sim
python3 DB/reset_and_setup.py
```

출력 예시:
```
[Firebase] 연결 완료
[reset]   robots/  → 0개 문서 삭제
[setup]   sections/A 등록
[setup]   sections/A/pods  20개 등록
...
[done] Firestore 업데이트 완료.
```

---

## 5. UI 설정

### 5-1. 패키지 설치

```bash
cd UI
npm install
```

### 5-2. Firebase 웹 앱 설정

```bash
cp .env.example .env.local
```

`.env.local`을 열어 Firebase 콘솔 값 입력:

```
Firebase 콘솔 → rokey-factory-base → 프로젝트 설정
→ 내 앱 → 웹 앱 → SDK 설정 및 구성
```

```env
VITE_FIREBASE_API_KEY=...
VITE_FIREBASE_AUTH_DOMAIN=rokey-factory-base.firebaseapp.com
VITE_FIREBASE_PROJECT_ID=rokey-factory-base
VITE_FIREBASE_STORAGE_BUCKET=rokey-factory-base.firebasestorage.app
VITE_FIREBASE_MESSAGING_SENDER_ID=...
VITE_FIREBASE_APP_ID=...
```

### 5-3. 개발 서버 실행

```bash
cd UI
npm run dev
# → http://localhost:5173
```

---

## 6. ROS2 iw_hub_movement 패키지 빌드

```bash
source /opt/ros/humble/setup.bash

cd ~/dev_ws/isaac_sim/src/Rokey_isaac-sim/main_isaac/robots/iw_hub
colcon build --packages-select iw_hub_movement

source install/setup.bash
```

---

## 7. Isaac Sim 실행

```bash
cd ~/dev_ws/isaac_sim/src/Rokey_isaac-sim
./run_sim.sh
```

---

## 8. Firebase 브릿지 노드 실행 (시뮬 실행 후)

```bash
source /opt/ros/humble/setup.bash
python3 DB/section_bridge.py
```

---

## 디렉토리 구조 요약

```
Rokey_isaac-sim/
├── main_isaac/          ← Isaac Sim 시뮬레이션 진입점
├── spot_robot/          ← Spot 단독 테스트 스크립트
├── DB/                  ← Firebase 관리 스크립트 + 브릿지 노드
│   ├── schema.md        ← Firestore schema 문서
│   ├── BRIDGE_SETTINGS.md ← ROS2 토픽 인터페이스 설정
│   ├── reset_and_setup.py ← Firestore 초기화
│   ├── section_bridge.py  ← ROS2 → Firestore 브릿지
│   └── serviceAccountKey.json  ← ★ 직접 배치 필요 (git 제외)
├── UI/                  ← React 웹 대시보드
│   └── .env.local       ← ★ 직접 작성 필요 (git 제외)
├── run_sim.sh           ← Isaac Sim 실행 스크립트
├── INTEGRATION_OVERVIEW.md ← 전체 시스템 문서
└── SETUP.md             ← 이 파일
```

---

## 실행 순서 요약

```
1. python3 DB/reset_and_setup.py   # Firestore 초기화 (최초 1회)
2. ./run_sim.sh                    # Isaac Sim 실행
3. python3 DB/section_bridge.py    # Firebase 브릿지 (별도 터미널)
4. cd UI && npm run dev            # 웹 대시보드 (별도 터미널)
```
