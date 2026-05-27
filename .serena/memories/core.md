# Rokey_isaac-sim — 프로젝트 핵심 맵

## 목적
NVIDIA Isaac Sim 5.1.0 기반 창고 자동화 시뮬레이션.
여러 로봇(IW Hub AMR, M0609 매니퓰레이터, 드론, Spot)이 협업하는 멀티로봇 시스템.

## 주 진입점
- `main_isaac/main.py` — 시뮬레이션 메인 루프, 모든 로봇 에이전트 생성
- `main_isaac/robot_config.py` — **단일 진실 소스**: 스폰 좌표, 섹션 NAV 맵, 벨트 슬롯 등 모든 상수

## 디렉토리 구조
```
main_isaac/
  main.py              # 엔트리포인트
  robot_config.py      # 전체 설정 상수 (ROBOT_REGISTRY, SECTION_NAV, BELT_DELIVERY_SLOTS…)
  world_setup.py       # USD 씬 로드, Pod Stack 스폰, BoxSpawner
  robots/
    base_robot.py      # 로봇 에이전트 베이스 클래스
    iw_hub/
      iw_hub_agent.py  # IW Hub AMR — 핵심 FSM 로직 (가장 복잡한 파일)
    m0609/
      m0609_agent.py   # M0609 매니퓰레이터 에이전트
    drone/
      drone_agent.py   # 드론 에이전트
    spot/
      spot_agent.py    # Spot 로봇 에이전트
  minimap.py           # OpenCV 탑뷰 미니맵 (별도 프로세스로 파이프 전송)
  path_planner.py      # A* 경로 계획 (IW Hub 표준 모드용)
  work_signals.py      # ROS2 완료 신호 카운터
  control_center.py    # 외부 제어 UI 브리지
grid/
  warehouse_layout.py  # matplotlib 창고 레이아웃 시각화 (개발용)
```

## 좌표계
- 단위: 미터(m), 원점 = warehouse_v3.usd 월드 원점
- 창고 범위: x=-18~24, y=-18~18 (minimap 기준)
- 섹션 경계(붉은 선): `mem:warehouse_layout` 참조
- 모든 좌표 변경은 `robot_config.py` 한 곳에서만 수정

## 관련 메모리
- 창고 레이아웃·섹션 좌표: `mem:warehouse_layout`
- IW Hub FSM 상세: `mem:iw_hub_fsm`
- 실행/빌드 명령: `mem:suggested_commands`
- 기술 스택: `mem:tech_stack`
