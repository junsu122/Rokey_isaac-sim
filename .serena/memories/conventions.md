# 코드 컨벤션

## 좌표/설정 관리
- **모든 좌표·상수는 `robot_config.py` 단일 파일**에서 관리. 에이전트 파일에 하드코딩 금지.
- 로봇 등록: `ROBOT_REGISTRY` 리스트에 딕셔너리로 추가

## 로봇 에이전트 구조
- 모든 에이전트는 `base_robot.py`의 베이스 클래스 상속
- `setup()`: 초기화 (스폰 후 1회)
- `post_reset()`: 씬 리셋 후 호출
- `update()`: 매 렌더 스텝 호출 (50Hz)
- FSM 상태는 문자열(pickup 모드) 또는 정수(표준 모드)로 추적

## 출력
- `print(f"[{self.name}] ...")` 형식으로 로봇 이름 prefix
- `sys.stderr.write()`: isaac-python에서만 stdout이 억제될 때 사용

## 웨이포인트 튜플 형식
`(x, y, yaw_deg, tolerance [, reverse_bool])`
- yaw_deg: 목표 헤딩 (현재는 대부분 0 = 동향)
- tolerance: NAV_TOL(0.20) 또는 DOCK_TOL(0.03)
- reverse_bool: True이면 후진으로 이동 (선택 옵션)

## 주석
- 한국어 인라인 주석 사용
- WHY가 비자명한 경우에만 주석 작성
