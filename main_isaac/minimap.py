"""
main_isaac/minimap.py
======================
창고 씬의 실시간 탑뷰(top-down) 미니맵.

기능:
  - IW Hub, M0609, ArUco 박스, Pod Stack 위치 실시간 표시
  - IW Hub 미션 상태 라벨 표시 (GOTO_STACK / LIFTING / GOTO_DROP / LOWERING)
  - 미니맵 우클릭 → Spot gripper용 소형 ArUco 박스 동적 스폰
  - 빨간 배달 라인(x = -7.5) 표시

Isaac Sim 번들 Python 은 cv2 GUI를 지원하지 않으므로
시스템 Python3 프로세스(minimap_process.py)로 이미지를 파이프 전송한다.
subprocess.stdout 을 통해 스폰 이벤트를 역방향으로 수신한다.
"""
from __future__ import annotations

import os
import math
import struct
import queue
import subprocess
import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf

import robot_config as C

# ── 표시 경로 ─────────────────────────────────────────────────────────
_SCRIPT = Path(__file__).parent / "minimap_process.py"

# ── 월드 좌표 표시 범위 ────────────────────────────────────────────────
_WX0, _WX1 = -18.0, 24.0
_WY0, _WY1 = -18.0, 18.0

# ── 이미지 크기 [px] ────────────────────────────────────────────────────
_IW, _IH = 900, 660

# ── 빨간 배달 라인 x 좌표 [m] ────────────────────────────────────────────
_RED_LINE_X = -7.5

# ── 색상 (BGR) ─────────────────────────────────────────────────────────
_C_BG        = (28,  28,  28)
_C_GRID      = (50,  50,  50)
_C_WALL      = (90,  90,  90)
_C_ROOM      = (45,  45,  60)
_C_CONV      = (40,  80, 100)
_C_PODSTACK  = (30, 130, 200)
_C_POD_CLICK = (60, 180, 255)   # 클릭으로 스폰된 Pod
_C_AUTOBOX   = (60, 210, 120)
_C_M0609     = (200, 100,  60)
_C_IWHUB     = (0,  220, 220)
_C_IWHUB_DIR = (0,  120, 120)
_C_SPOT      = (80, 220, 130)   # Spot (연녹색)
_C_SPOT_DIR  = (40, 140,  70)   # Spot 방향 화살표
_C_DRONE     = (50, 200, 255)   # 드론 (황청색)
_C_DRONE_ALT = (30, 130, 180)   # 드론 윤곽
_C_GREEN_BOX = (40, 180,  40)
_C_RED_BOX   = (40,  40, 200)
_C_BLUE_BOX  = (200,  80,  40)
_C_LABEL     = (220, 220, 220)
_C_AXIS_X    = (60,  60, 200)
_C_AXIS_Y    = (60, 160,  60)
_C_RED_LINE  = (60,  60, 220)   # 빨간 배달 라인
_C_SEC_A_POD  = (0,  165, 255)  # Section A pods (orange)
_C_SEC_B_POD  = (60, 200,  60)  # Section B pods (green)
_C_SEC_C_POD  = (200,  60, 255) # Section C pods (purple)
_C_SEC_BORDER = (80,   80, 220) # section boundary outline (kept distinct from wall)

# ── 미션 상태 라벨 ────────────────────────────────────────────────────
_STATE_LABELS = {0: "WAIT", 1: "LIFT↑", 2: "→SEC", 3: "LOWER↓", 4: "→HOME"}

# ── 창고 레이아웃 ──────────────────────────────────────────────────────
_OUTER_WALL = (-16.65, 16.65, -16.65, 16.65)

_SORTING_ROOMS = [
    (16.5, 23.0,  5.1, 15.9),
    (16.5, 23.0, -5.3,  5.3),
    (16.5, 23.0,-15.9, -5.1),
]

_MANIP_ROOMS = [
    (-16.5, -8.0,  3.5, 10.0),
    (-16.5, -8.0, -3.5,  3.5),
    (-16.5, -8.0,-10.0, -3.5),
]

_CONV_CENTERS = [
    (13.5, -10.5), (15.5, -10.5), (17.5, -10.5), (19.5, -10.5),
    (13.5,   0.0), (15.5,   0.0), (17.5,   0.0), (19.5,   0.0),
    (13.5,  10.5), (15.5,  10.5), (17.5,  10.5), (19.5,  10.5),
    (-15.0, 0.0), (-17.0, 0.0), (-19.0, 0.0),
    (-12.8, -2.35), (-12.8,  2.35),
    (-11.3, -7.8),
    (-9.5,  0.0),
]

# (sec_name, x_min, x_max, y_min, y_max, color)
_SECTION_BOUNDS = [
    ("A", -4.9,  4.9,   6.1,  13.9, _C_SEC_A_POD),
    ("B", -4.9,  4.9,  -3.9,   3.9, _C_SEC_B_POD),
    ("C", -4.9,  4.9, -13.9,  -6.1, _C_SEC_C_POD),
]


# ══════════════════════════════════════════════════════════════════════

def _w2p(wx: float, wy: float) -> tuple:
    px = int((wx - _WX0) / (_WX1 - _WX0) * _IW)
    py = int((1.0 - (wy - _WY0) / (_WY1 - _WY0)) * _IH)
    return px, py


def _clamp_px(p: tuple) -> tuple:
    return (max(0, min(_IW - 1, p[0])), max(0, min(_IH - 1, p[1])))


def _clean_env() -> dict:
    keep = {
        'HOME', 'USER', 'USERNAME', 'LOGNAME',
        'DISPLAY', 'XAUTHORITY', 'WAYLAND_DISPLAY',
        'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS',
        'LANG', 'LC_ALL', 'LC_CTYPE', 'TZ', 'TERM',
    }
    env = {k: v for k, v in os.environ.items() if k in keep}
    env.setdefault('PATH', '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin')
    return env


# ══════════════════════════════════════════════════════════════════════

class Minimap:
    """
    실시간 창고 탑뷰 미니맵.
    우클릭으로 Isaac Sim 에 Pod Stack 을 동적으로 스폰한다.
    """

    def __init__(self, agents: list):
        self._agents     = agents
        self._proc: Optional[subprocess.Popen] = None
        self._send_q: queue.Queue              = queue.Queue(maxsize=2)
        self._spawn_q: queue.Queue             = queue.Queue()
        self._send_thread:   Optional[threading.Thread] = None
        self._stdout_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None
        self._static_bg: Optional[np.ndarray]  = None
        self._frame_skip = 0
        self._click_pods: list = []
        self._pod_counter = 0

        # 정적 배경은 USD 스테이지와 무관하게 즉시 생성 가능
        try:
            self._static_bg = self._build_static_bg()
        except Exception as e:
            print(f"[Minimap] 정적 배경 생성 실패: {e}")
            import traceback; traceback.print_exc()

        # 서브프로세스 즉시 시작
        self._start_process()

    # ── 서브프로세스 시작 ────────────────────────────────────────────

    def _start_process(self) -> bool:
        """minimap_process.py 를 새로 시작한다. 실패하면 False 반환."""
        try:
            self._proc = subprocess.Popen(
                ['/usr/bin/python3', str(_SCRIPT)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=_clean_env(),
            )
            print(f"[Minimap] 서브프로세스 시작 — PID {self._proc.pid}  스크립트: {_SCRIPT}")
            # 송신 스레드: 큐에서 꺼내 stdin 에 씀
            self._send_q = queue.Queue(maxsize=2)
            self._send_thread = threading.Thread(target=self._sender, daemon=True)
            self._send_thread.start()
            # stdout/stderr 리더
            self._stdout_thread = threading.Thread(target=self._stdout_reader, daemon=True)
            self._stdout_thread.start()
            self._stderr_thread = threading.Thread(target=self._stderr_reader, daemon=True)
            self._stderr_thread.start()
            return True
        except Exception as e:
            print(f"[Minimap] 서브프로세스 시작 실패: {e}")
            import traceback; traceback.print_exc()
            return False

    def _ensure_alive(self) -> bool:
        """프로세스가 살아있으면 True. 죽어있으면 재시작 시도."""
        if self._proc is not None and self._proc.poll() is None:
            return True
        print("[Minimap] 서브프로세스 종료 감지 — 재시작 시도")
        return self._start_process()

    # ── stdout 수신 스레드 (스폰 이벤트) ─────────────────────────────

    def _stdout_reader(self) -> None:
        """minimap_process.py 의 stdout 을 읽어 스폰 요청을 큐에 쌓는다."""
        try:
            for raw_line in self._proc.stdout:
                line = raw_line.decode(errors="ignore").strip()
                if line.startswith("SPAWN "):
                    parts = line.split()
                    if len(parts) == 3:
                        wx, wy = float(parts[1]), float(parts[2])
                        self._spawn_q.put_nowait((wx, wy))
        except Exception:
            pass

    def _stderr_reader(self) -> None:
        """minimap_process.py 의 stderr 를 읽어 출력한다 (디버그용)."""
        try:
            for raw_line in self._proc.stderr:
                line = raw_line.decode(errors="ignore").rstrip()
                if line:
                    print(f"[Minimap-proc] {line}")
        except Exception:
            pass

    # ── 정적 배경 ────────────────────────────────────────────────────

    def _build_static_bg(self) -> np.ndarray:
        img = np.full((_IH, _IW, 3), _C_BG, dtype=np.uint8)

        # 격자
        for gx in range(-16, 25, 4):
            cv2.line(img, _clamp_px(_w2p(gx, _WY0)),
                     _clamp_px(_w2p(gx, _WY1)), _C_GRID, 1, cv2.LINE_AA)
            px, py0 = _w2p(gx, _WY0)
            cv2.putText(img, f"{gx}", _clamp_px((px + 2, py0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (120, 120, 120), 1, cv2.LINE_AA)
        for gy in range(-16, 19, 4):
            cv2.line(img, _clamp_px(_w2p(_WX0, gy)),
                     _clamp_px(_w2p(_WX1, gy)), _C_GRID, 1, cv2.LINE_AA)
            px0, py = _w2p(_WX0, gy)
            cv2.putText(img, f"{gy}", _clamp_px((px0 + 4, py - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (120, 120, 120), 1, cv2.LINE_AA)

        # 서측 조작실
        for (x0, x1, y0, y1) in _MANIP_ROOMS:
            cv2.rectangle(img, _w2p(x0, y1), _w2p(x1, y0), _C_ROOM, -1)
            cv2.rectangle(img, _w2p(x0, y1), _w2p(x1, y0), (70, 70, 90), 1)

        # 동측 선별실
        for (x0, x1, y0, y1) in _SORTING_ROOMS:
            cv2.rectangle(img, _w2p(x0, y1), _w2p(x1, y0), _C_ROOM, -1)
            cv2.rectangle(img, _w2p(x0, y1), _w2p(x1, y0), (70, 70, 90), 1)

        # 외벽
        x0, x1, y0, y1 = _OUTER_WALL
        cv2.rectangle(img, _w2p(x0, y0), _w2p(x1, y1), _C_WALL, 2)

        # 컨베이어 트랙
        for cx, cy in _CONV_CENTERS:
            cv2.rectangle(img,
                          _w2p(cx - 0.9, cy - 0.35),
                          _w2p(cx + 0.9, cy + 0.35),
                          _C_CONV, -1)

        # 빨간 배달 라인 (x = _RED_LINE_X)
        lp1 = _clamp_px(_w2p(_RED_LINE_X, _WY0))
        lp2 = _clamp_px(_w2p(_RED_LINE_X, _WY1))
        cv2.line(img, lp1, lp2, _C_RED_LINE, 2, cv2.LINE_AA)
        rlx, _ = _w2p(_RED_LINE_X, 0)
        cv2.putText(img, "DROP LINE", (rlx + 4, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, _C_RED_LINE, 1, cv2.LINE_AA)

        # Section A / B / C 구역 경계
        for sec_name, sx0, sx1, sy0, sy1, sec_col in _SECTION_BOUNDS:
            p1 = _w2p(sx0, sy1)   # NW corner (image top-left)
            p2 = _w2p(sx1, sy0)   # SE corner (image bottom-right)
            cv2.rectangle(img, p1, p2, sec_col, 2, cv2.LINE_AA)
            cx_px, cy_px = _w2p((sx0 + sx1) / 2, (sy0 + sy1) / 2)
            cv2.putText(img, f"Sec {sec_name}", (cx_px - 28, cy_px + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, sec_col, 1, cv2.LINE_AA)

        # PodStack_04 — 드론 배달 목적지 (12, 14) 강조 표시
        for pod in C.POD_STACKS:
            if pod["name"] == "PodStack_04":
                px, py = _w2p(pod["xyz"][0], pod["xyz"][1])
                cv2.circle(img, (px, py), 13, (0, 200, 255), 2, cv2.LINE_AA)
                cv2.putText(img, "DELIVERY", (px - 28, py - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 200, 255), 1, cv2.LINE_AA)

        # M0609 베이스 위치 (고정)
        for cfg in C.ROBOT_REGISTRY:
            if cfg["type"] != "m0609":
                continue
            bx, by = cfg["spawn_xyz"][0], cfg["spawn_xyz"][1]
            px, py = _w2p(bx, by)
            _draw_robot_square(img, px, py, 9, _C_M0609)
            cv2.putText(img, cfg["name"][-1], (px - 4, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, _C_LABEL, 1, cv2.LINE_AA)

        # 원점 축
        ox, oy = _w2p(0, 0)
        ex, _  = _w2p(1.5, 0)
        _,  ey = _w2p(0, 1.5)
        cv2.arrowedLine(img, (ox, oy), (ex, oy), _C_AXIS_X, 2, cv2.LINE_AA, tipLength=0.3)
        cv2.arrowedLine(img, (ox, oy), (ox, ey), _C_AXIS_Y, 2, cv2.LINE_AA, tipLength=0.3)
        cv2.putText(img, "X", (ex + 3, oy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, _C_AXIS_X, 1)
        cv2.putText(img, "Y", (ox + 3, ey - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, _C_AXIS_Y, 1)

        for txt, wx, wy in [("N", 0, 17), ("S", 0, -17), ("W", -17, 0), ("E", 22, 0)]:
            p = _w2p(wx, wy)
            cv2.putText(img, txt, _clamp_px((p[0] - 4, p[1] + 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        return img

    # ── 동적 레이어 ──────────────────────────────────────────────────

    def _draw_dynamic(self, base: np.ndarray) -> np.ndarray:
        img = base.copy()
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()

        # ── A* 계획 경로 시각화 ───────────────────────────────────────
        try:
            from path_planner import get_planner
            planner = get_planner()
            if planner is not None:
                _path_agent_colors = {}
                for ag in self._agents:
                    t = ag.cfg.get("type")
                    if t == "iw_hub":
                        _path_agent_colors[ag.name] = _C_IWHUB
                    elif t == "spot":
                        _path_agent_colors[ag.name] = _C_SPOT
                for aname, path in planner.get_agent_paths().items():
                    if len(path) < 2:
                        continue
                    col = _path_agent_colors.get(aname, (160, 160, 160))
                    fade = tuple(max(0, c - 80) for c in col)
                    for i in range(len(path) - 1):
                        p1 = _clamp_px(_w2p(path[i][0],   path[i][1]))
                        p2 = _clamp_px(_w2p(path[i+1][0], path[i+1][1]))
                        cv2.line(img, p1, p2, fade, 1, cv2.LINE_AA)
                    for wp in path[1:-1]:
                        wpx, wpy = _clamp_px(_w2p(wp[0], wp[1]))
                        cv2.circle(img, (wpx, wpy), 2, col, -1, cv2.LINE_AA)
        except Exception:
            pass

        # 동적 ArUco 박스 (BoxSpawner → /World/DynamicBoxes/)
        _box_colors = {"green_id0": _C_GREEN_BOX, "red_id1": _C_RED_BOX, "blue_id2": _C_BLUE_BOX}
        for prim in stage.Traverse():
            bpath = str(prim.GetPath())
            if not bpath.startswith("/World/DynamicBoxes/") or bpath.count("/") != 3:
                continue
            try:
                tr  = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                px, py = _w2p(float(tr[0]), float(tr[1]))
                pname = prim.GetName()
                col, box_id = (200, 200, 200), "?"
                for btype, bcol in _box_colors.items():
                    if pname.startswith(btype):
                        col    = bcol
                        box_id = btype.split("id")[-1]
                        break
                cv2.circle(img, (px, py), 9, col, -1, cv2.LINE_AA)
                cv2.circle(img, (px, py), 9, (240, 240, 240), 1, cv2.LINE_AA)
                cv2.putText(img, box_id, (px - 4, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception:
                pass

        # 동적 AutoBox / Pod Stack / Shelf 계열
        seen_pods = set()
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.startswith("/World/AutoBox_") and path.count("/") == 2:
                tr = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                px, py = _w2p(float(tr[0]), float(tr[1]))
                cv2.rectangle(img, (px - 5, py - 5), (px + 5, py + 5), _C_AUTOBOX, -1)
                cv2.rectangle(img, (px - 5, py - 5), (px + 5, py + 5), (220, 255, 220), 1)
                cv2.putText(img, "B", (px - 3, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 0, 0), 1, cv2.LINE_AA)
            elif path.startswith("/World/PodStacks/") and path.count("/") == 3:
                seen_pods.add(path)
                tr = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                px, py = _w2p(float(tr[0]), float(tr[1]))
                if "ClickPod" in path:
                    col = _C_POD_CLICK
                elif "/Sec_A_" in path:
                    col = _C_SEC_A_POD
                elif "/Sec_B_" in path:
                    col = _C_SEC_B_POD
                elif "/Sec_C_" in path:
                    col = _C_SEC_C_POD
                else:
                    col = _C_PODSTACK
                cv2.rectangle(img, (px - 7, py - 7), (px + 7, py + 7), col, -1)
                cv2.rectangle(img, (px - 7, py - 7), (px + 7, py + 7), (200, 220, 255), 1)
                cv2.putText(img, prim.GetName()[-2:], (px - 6, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, _C_LABEL, 1, cv2.LINE_AA)
            elif ("Shelf" in path or "shelf" in path) and path.count("/") <= 3:
                tr = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                px, py = _w2p(float(tr[0]), float(tr[1]))
                cv2.rectangle(img, (px - 9, py - 6), (px + 9, py + 6), _C_PODSTACK, 1)

        # old click list fallback, in case a prim was removed or renamed externally
        for wx, wy, prim_path in self._click_pods:
            if prim_path in seen_pods:
                continue
            px, py = _w2p(wx, wy)
            cv2.rectangle(img, (px - 7, py - 7), (px + 7, py + 7), _C_POD_CLICK, -1)
            cv2.rectangle(img, (px - 7, py - 7), (px + 7, py + 7), (255, 220, 100), 1)

        # IW Hub 로봇
        for agent in self._agents:
            if agent.cfg.get("type") != "iw_hub":
                continue
            try:
                try:
                    x, y, hdg = agent.get_world_xy()
                except Exception:
                    x, y = agent.spawn_xyz[0], agent.spawn_xyz[1]
                    hdg  = math.radians(agent.spawn_yaw)

                px, py = _w2p(x, y)
                dl  = 18
                epx = int(px + dl * math.cos(hdg))
                epy = int(py - dl * math.sin(hdg))

                cv2.circle(img, (px, py), 11, _C_IWHUB, -1, cv2.LINE_AA)
                cv2.circle(img, (px, py), 11, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.arrowedLine(img, (px, py), (epx, epy), _C_IWHUB_DIR,
                                2, cv2.LINE_AA, tipLength=0.4)
                cv2.putText(img, agent.name[-2:],
                            (px - 7, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1, cv2.LINE_AA)

                # 미션 상태 라벨 (속성이 없을 수 있으므로 getattr 사용)
                state_str = _STATE_LABELS.get(getattr(agent, "mission_state", None), "")
                cv2.putText(img, state_str, (px - 20, py - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 200), 1, cv2.LINE_AA)
            except Exception as e:
                print(f"[Minimap] IW Hub {agent.name} 그리기 오류: {e}")

        # ── 드론 ─────────────────────────────────────────────────────
        for agent in self._agents:
            if agent.cfg.get("type") != "drone":
                continue
            try:
                try:
                    x, y, hdg, alt = agent.get_world_xy()
                except Exception:
                    x, y = agent.spawn_xyz[0], agent.spawn_xyz[1]
                    hdg, alt = 0.0, 0.0

                px, py = _w2p(x, y)

                # X자 로터 암 표시 (드론 심볼)
                arm = 11
                for ang in (45, 135):
                    rad = math.radians(ang)
                    ca, sa = math.cos(rad), math.sin(rad)
                    p1 = (int(px + arm * ca), int(py - arm * sa))
                    p2 = (int(px - arm * ca), int(py + arm * sa))
                    cv2.line(img, p1, p2, _C_DRONE, 3, cv2.LINE_AA)
                cv2.circle(img, (px, py), 5, _C_DRONE, -1, cv2.LINE_AA)
                cv2.circle(img, (px, py), 5, (255, 255, 255), 1, cv2.LINE_AA)

                # 방향 화살표
                dl  = 16
                epx = int(px + dl * math.cos(hdg))
                epy = int(py - dl * math.sin(hdg))
                cv2.arrowedLine(img, (px, py), (epx, epy), _C_DRONE_ALT,
                                2, cv2.LINE_AA, tipLength=0.4)

                # 이름 + 고도 + 미션 상태 라벨
                _drone_ms = getattr(agent, "_mission_state", "")
                _ms_short = {"IDLE":"IDLE","TAKEOFF":"↑","FLY_PICK":"→PICK",
                             "DESCEND_PICK":"↓PICK","GRAB":"GRAB","ASCEND_PICK":"↑",
                             "FLY_DROP":"→DROP","RELEASE":"REL","DONE":"DONE"}.get(_drone_ms,"")
                lbl = f"{agent.name[-2:]} z{alt:.1f} {_ms_short}"
                cv2.putText(img, lbl, (px - 10, py + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, _C_DRONE, 1, cv2.LINE_AA)
            except Exception as e:
                print(f"[Minimap] Drone {agent.name} 그리기 오류: {e}")

        # ── Spot 로봇 ─────────────────────────────────────────────────
        _SPOT_STATE_LABELS = {
            "WALKING"          : "WALK",
            "NAVIGATE_TO_CUBE" : "→BOX",
            "LOWER"            : "LOWER",
            "GRASP"            : "GRASP",
            "RAISE"            : "RAISE",
            "NAVIGATE_TO_GOAL" : "→GOAL",
            "RELEASE"          : "REL",
        }
        for agent in self._agents:
            if agent.cfg.get("type") != "spot":
                continue
            try:
                try:
                    x, y, hdg = agent.get_world_xy()
                except Exception:
                    x, y = agent.spawn_xyz[0], agent.spawn_xyz[1]
                    hdg  = math.radians(agent.spawn_yaw)

                px, py = _w2p(x, y)

                # 삼각형 심볼 (방향 화살표)
                tri = 13
                tip = (int(px + tri * math.cos(hdg)),
                       int(py - tri * math.sin(hdg)))
                bl  = (int(px + tri * 0.6 * math.cos(hdg + 2.4)),
                       int(py - tri * 0.6 * math.sin(hdg + 2.4)))
                br  = (int(px + tri * 0.6 * math.cos(hdg - 2.4)),
                       int(py - tri * 0.6 * math.sin(hdg - 2.4)))
                pts = np.array([[tip, bl, br]], dtype=np.int32)
                cv2.fillPoly(img, pts, _C_SPOT)
                cv2.polylines(img, pts, True, (220, 255, 220), 1, cv2.LINE_AA)

                # 방향 화살표
                dl  = 18
                epx = int(px + dl * math.cos(hdg))
                epy = int(py - dl * math.sin(hdg))
                cv2.arrowedLine(img, (px, py), (epx, epy), _C_SPOT_DIR,
                                2, cv2.LINE_AA, tipLength=0.35)

                # 이름 라벨
                cv2.putText(img, agent.name[-2:], (px - 7, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 0, 0), 1, cv2.LINE_AA)

                # 상태 라벨
                state_str = _SPOT_STATE_LABELS.get(getattr(agent, "_state", ""), "")
                cv2.putText(img, state_str, (px - 18, py - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, _C_SPOT, 1, cv2.LINE_AA)
            except Exception as e:
                print(f"[Minimap] Spot {agent.name} 그리기 오류: {e}")

        # 타이틀 / 범례
        cv2.putText(img, "Warehouse Minimap  [우클릭: Pod 스폰]", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)
        _draw_legend(img)

        return img

    # ── 공개 API ──────────────────────────────────────────────────────

    def update(self) -> None:
        """렌더 루프에서 매 step 호출. 6 프레임마다 전송 (~8 Hz)."""
        self._frame_skip += 1
        if self._frame_skip % 6 != 0:
            return
        if not self._ensure_alive():
            return
        if self._static_bg is None:
            return

        # 스폰 요청 처리
        while not self._spawn_q.empty():
            try:
                wx, wy = self._spawn_q.get_nowait()
                self._spawn_pod(wx, wy)
            except queue.Empty:
                break

        try:
            frame = self._draw_dynamic(self._static_bg)
        except Exception as e:
            print(f"[Minimap] _draw_dynamic 오류: {e}")
            import traceback; traceback.print_exc()
            return
        h, w  = frame.shape[:2]
        data  = struct.pack('<HH', h, w) + frame.tobytes()
        header = bytes([0x01]) + struct.pack('<I', len(data))
        try:
            self._send_q.put_nowait([header + data])
        except queue.Full:
            pass

    def close(self) -> None:
        if self._proc is None:
            return
        try:
            self._send_q.put_nowait([bytes([0xFF]) + struct.pack('<I', 0)])
        except queue.Full:
            pass
        try:
            self._send_q.put_nowait(None)  # sentinel to stop sender thread
        except queue.Full:
            pass
        if self._proc.poll() is None:
            try:
                self._proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._proc.kill()

    # ── 우클릭 ArUco 박스 스폰 ───────────────────────────────────────

    def _spawn_pod(self, wx: float, wy: float) -> None:
        """우클릭 좌표에 Spot gripper용 소형 ArUco 박스를 스폰한다."""
        try:
            from auto_spawn_panel import _create_box_with_aruco
            stage = omni.usd.get_context().get_stage()
            self._pod_counter += 1
            prim_path = f"/World/MinimapClickBoxes/ClickBox_{self._pod_counter:03d}"

            root = stage.GetPrimAtPath("/World/MinimapClickBoxes")
            if not root or not root.IsValid():
                from pxr import UsdGeom as _UG
                _UG.Xform.Define(stage, "/World/MinimapClickBoxes")

            # Small enough for Spot gripper; ArUco ID cycles 0,1,2.
            bw, bd, bh = 0.16, 0.16, 0.10
            aruco_id = (self._pod_counter - 1) % 3
            colors = {
                0: (0.2, 0.8, 0.2),
                1: (0.8, 0.2, 0.2),
                2: (0.2, 0.4, 0.9),
            }
            _create_box_with_aruco(
                prim_path=prim_path,
                x_m=float(wx), y_m=float(wy), z_m=bh / 2.0,
                bw=bw, bd=bd, bh=bh,
                color_rgb=colors[aruco_id],
                mass=0.5,
                orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                aruco_id=aruco_id,
            )

            self._click_pods.append((wx, wy, prim_path))
            print(f"[Minimap] ArUco box 스폰: {prim_path}  "
                  f"id={aruco_id}  wx={wx:.2f}  wy={wy:.2f}")
        except Exception as e:
            print(f"[Minimap] ArUco box 스폰 실패: {e}")

    # ── 내부 전송 스레드 ─────────────────────────────────────────────

    def _sender(self) -> None:
        while True:
            item = self._send_q.get()
            if item is None:
                break
            if self._proc is None or self._proc.poll() is not None:
                break
            try:
                for raw in item:
                    self._proc.stdin.write(raw)
                self._proc.stdin.flush()
            except BrokenPipeError:
                break


# ── 공통 그리기 헬퍼 ─────────────────────────────────────────────────

def _draw_robot_square(img, cx, cy, half, color):
    cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), color, -1)
    cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), (220, 220, 220), 1)


def _draw_legend(img: np.ndarray) -> None:
    items = [
        (_C_IWHUB,      "IW Hub"),
        (_C_SPOT,       "Spot"),
        (_C_DRONE,      "Drone"),
        (_C_M0609,      "M0609"),
        (_C_PODSTACK,   "Pod Stack"),
        (_C_POD_CLICK,  "Spawned Pod"),
        (_C_SEC_A_POD,  "Sec A Pods"),
        (_C_SEC_B_POD,  "Sec B Pods"),
        (_C_SEC_C_POD,  "Sec C Pods"),
        (_C_AUTOBOX,    "AutoBox"),
        (_C_GREEN_BOX,  "ArUco Green"),
        (_C_RED_BOX,    "ArUco Red"),
        (_C_BLUE_BOX,   "ArUco Blue"),
        (_C_CONV,       "Conveyor"),
        (_C_RED_LINE,   "Drop Line"),
    ]
    lx, ly = _IW - 115, 14
    for col, label in items:
        cv2.rectangle(img, (lx, ly - 7), (lx + 12, ly + 3), col, -1)
        cv2.putText(img, label, (lx + 16, ly + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, _C_LABEL, 1, cv2.LINE_AA)
        ly += 16
