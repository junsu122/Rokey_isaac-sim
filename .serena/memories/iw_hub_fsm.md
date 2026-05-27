# IW Hub FSM — main_isaac/robots/iw_hub/iw_hub_agent.py

## 모드 분기 (cfg["mode"])
- `"standard"` (기본, hub_01/03): 섹션 슬롯에 pod 배달
- `"pickup"` (hub_02): pod 픽업 → 출하 벨트 → 섹션 보충 순환

## Pickup 모드 FSM 상태 순환
```
WAITING
  → GOTO_PICKUP       pickup_xy(-7.9,1.5)로 후진
  → LIFTING           _run_lift_phase(up=True)
  → GOTO_BELT         _make_deliver_path() — inter-zone corridor 경유
  → ALIGN_LOWER       _final_align() 정밀 정렬
  → LOWERING          _run_lift_phase(up=False) — 벨트에 pod 놓기
  → BACKOUT_BELT      _make_belt_backout_path() — corridor → home
  → RESTOCK_GOTO_ENTRY  섹션 B entry(-4.9,-1.7)
  → RESTOCK_DOCK        _corridor_dock(rx,ry)
  → RESTOCK_LIFT        섹션 pod 들기
  → RESTOCK_BACKOUT     _corridor_backout(rx,ry)
  → RESTOCK_PLACE       entry 경유 → picking place(-7.9,1.5)
  → RESTOCK_LOWER       picking place에 pod 내려놓기
  → WAITING
```

## 표준 모드 FSM (mission_state int)
0=WAITING → 1=LIFTING → 2=GOTO_ENTRY → 3=DOCK_IN → 4=LOWERING → 5=BACKOUT → 6=GOTO_HOME → 0

## 핵심 이동 함수
- `_follow_path(waypoints)`: 순차 웨이포인트 이동, 각 웨이포인트 = (x, y, yaw_deg, tol [, reverse])
- `_drive_straight_to(tx, ty)`: 목표까지 heading 보정하며 직진, 속도 ramp MAX_V*dist/1.5
- `_corridor_dock(tx, ty)`: h_corr[0] → col_corr_x → slot_y → 최종 x 도킹 (4단계)
- `_corridor_backout(tx, ty)`: col_corr_x → h_corr[0] 후퇴 (2단계)
- `_final_align(tx, ty)`: step0=헤딩정렬, step1=y보정(남향:직진/동향:회전), step2=x보정

## 배달 경로 설계 (hub_02 기준)
- `_make_deliver_path(tx, ty)`: 3 웨이포인트
  1. (entry_x-0.5, corridor_y=5.0) — 섹션 서쪽에서 A-B corridor 정렬
  2. (tx, 5.0) — corridor 따라 동향 직진 (section B 완전 우회)
  3. (tx, 0.0) — 벨트로 남진
- `_make_belt_backout_path`: (tx, 5.0) → (hx, hy)
- `_get_corridor_y()`: {"A":14.5, "B":5.0, "C":-5.0}

## 주요 상수 (클래스 변수)
- MAX_V = 1.5 m/s, MAX_W = 1.5 rad/s
- NAV_TOL = 0.20 m, DOCK_TOL = 0.03 m
- LIFT_STEPS = 200 틱

## 슬롯 인덱스
- `_belt_slot_idx`: BELT_DELIVERY_SLOTS 순환 (0~3)
- `_restock_slot_idx`: SECTION_PODS[sec][1:] 순환 (slot02~09)
