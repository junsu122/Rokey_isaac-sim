# 세션 로그

## 1. 프로젝트 컨텍스트

- **브랜치**: `sequence` (temp 브랜치에서 분기)
- **목적**: iw_hub 운반 시퀀스 구현 + 창고 레이아웃 정리

---

## 2. 창고 레이아웃 변경사항

### Pod 그리드 변경
- **기존**: 3×4 = 12개, dx=2.8, dy=2.0
- **변경**: 3×3 = 9개, dx=3.5, dy=3.0
- 제거된 pod는 empty pod
- Full pod (박스 적재): 슬롯 7~9 (마지막 행)
- 슬롯 01: iw_hub 배달 예약 (빈 상태)

### Section 센터 좌표
```
Sec A: (0.0,  10.0)
Sec B: (0.0,   0.0)   ← 기존 -0.2에서 변경
Sec C: (0.0, -10.0)
```

### Slot 01 위치 (iw_hub 배달 목적지)
```
Sec A slot 01: (-3.5,  7.0)
Sec B slot 01: (-3.5, -3.0)
Sec C slot 01: (-3.5, -13.0)
```

### Staging Platform 위치 (USD 실측)
```
Stage (3way):  (-14.25,  0.0)
Stage N (A):   (-11.32,  8.66)
Stage W (B):   ( -8.70,  0.04)
Stage S (C):   (-11.20, -8.55)
```

### PodStack 위치
```
PodStack_01: (-12.8,  9.0)   # Sec A
PodStack_02: ( -8.2,  1.5)   # Sec B
PodStack_03: ( -9.7, -8.9)   # Sec C
PodStack_04: ( 12.0, 14.0)   # 드론 배달 목적지
```

---

## 3. iw_hub 설정 변경

### Spawn / Yaw
```
iw_hub_01: spawn (-12.8, 14.0, -0.14)  yaw =  90°
iw_hub_02: spawn ( -6.45, 1.5, -0.14)  yaw =   0°  (변경 없음)
iw_hub_03: spawn ( -9.7,-13.0, -0.14)  yaw = -90°
```

### 속도 설정 (axis_nav.py / move_to_point.py)
```
MAX_V: 1.5 → 3.5 m/s
MAX_W: 2.0 → 0.8 rad/s
```

### Pod 픽업 오프셋
- pod center까지 **0.4m** 추가 전진 후 lift

---

## 4. iw_hub_01 Waypoint 초안 (미확정)

```
원칙: yaw 회전 최소화, X→Y 축 이동

1. spawn  (-12.8, 14.0)  yaw=90°  대기
2. 후진   (-12.8,  8.6)           PodStack_01 픽업 (0.4m 오버)
3. lift_up
4. 후진   (-12.8, 14.0)           pod 들고 복귀
5. +X 이동(-3.5,  14.0)           통로 이동
6. -Y 이동(-3.5,   7.0)           slot 01 배달 위치
7. lift_down
8. +Y 이동(-3.5,  14.0)           복귀
9. -X 이동(-12.8, 14.0)           대기 위치 복귀
```

iw_hub_02, iw_hub_03 waypoint는 **미확정** — 다음 세션에서 계속

---

## 5. GNOME 로그인 문제 (다른 노트북)

### 증상
- GDM3 active(running) 상태인데 로그인 GUI 안 뜸
- 백라이트만 켜짐

### 원인 파악
- Xvfb가 `:20` 가상 디스플레이에서 실행 중
- GDM이 물리 디스플레이(Xorg tty1) 대신 Xvfb(:20)에 붙음
- `.xsession-errors`에서 `DISPLAY=:20` 확인

### 조치
- `/etc/gdm3/custom.conf`에 `InitialVT=2` 추가 → 로그인은 됐으나 로그인 후 검은 화면
- Xvfb 실행 원인: PAM 세션(PPID=1023, sd-pam)
- `~/.config/autostart/`, `~/.config/systemd/user/` 없음
- `~/.profile`, `~/.bashrc` 에 Xvfb 등록 여부 **미확인** ← 다음에 확인 필요

### 다음 확인 사항
```bash
grep -l "Xvfb" /home/rokey/.profile /home/rokey/.bashrc /home/rokey/.bash_profile
grep "Xvfb" /home/rokey/.profile
grep "Xvfb" /home/rokey/.bashrc
```

---

## 6. 수정된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `main_isaac/robot_config.py` | _make_grid rows=3, dx=3.5, dy=3.0 / Sec B 센터 (0,0) / iw_hub spawn 좌표 |
| `main_isaac/world_setup.py` | _SECTION_BOX_SLOTS [7,8,9] |
| `main_isaac/draw_warehouse.py` | 동일 반영 + staging platform 추가 |
| `main_isaac/robots/iw_hub/src/iw_hub_movement/iw_hub_movement/axis_nav.py` | MAX_V=3.5, MAX_W=0.8 |
| `main_isaac/robots/iw_hub/src/iw_hub_movement/iw_hub_movement/move_to_point.py` | MAX_V=3.5, MAX_W=0.8 |

---

## 7. 다음 세션 할 일

1. [ ] iw_hub_02, iw_hub_03 waypoint 확정
2. [ ] sequence FSM 노드 구현 (2단계 fine docking 포함)
3. [ ] GNOME Xvfb 원인 파일 확인 및 제거
4. [ ] integration 브랜치와 sequence 브랜치 통합 계획
