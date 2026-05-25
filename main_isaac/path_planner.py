"""
path_planner.py
================
창고 A* 경로 계획 모듈 (중앙 집중식).

모든 이동 로봇(IW Hub, Spot)이 이 모듈의 전역 플래너 인스턴스를 통해
경로를 계획한다.  다른 로봇의 현재 위치를 동적 장애물로 반영한다.

사용법:
    # main.py 에서 1회 초기화
    from path_planner import init_planner
    planner = init_planner(agents)

    # 에이전트에서 경로 요청
    from path_planner import get_planner
    path = get_planner().plan((sx, sy), (gx, gy), agent_name="iw_hub_01")
    # → [(x0,y0), (x1,y1), ...] 월드 좌표 리스트
"""
from __future__ import annotations

import heapq
import math
from typing import List, Tuple, Optional

# ── 전역 싱글턴 ───────────────────────────────────────────────────────
_global_planner: Optional["WarehousePathPlanner"] = None


def init_planner(agents: list) -> "WarehousePathPlanner":
    global _global_planner
    _global_planner = WarehousePathPlanner(agents)
    return _global_planner


def get_planner() -> Optional["WarehousePathPlanner"]:
    return _global_planner


# ── 타입 별칭 ──────────────────────────────────────────────────────────
XY = Tuple[float, float]


class WarehousePathPlanner:
    """
    그리드 기반 A* 경로 계획기.

    정적 장애물: 창고 외벽 (외벽 내부는 모두 이동 가능).
    동적 장애물: 다른 에이전트의 현재 위치 주변 격자 셀.
    """

    # ── 월드 좌표 범위 ─────────────────────────────────────────────────
    WX0, WX1 = -18.0, 24.0
    WY0, WY1 = -18.0, 18.0

    # ── 격자 해상도 ────────────────────────────────────────────────────
    GRID_RES = 0.5   # 셀 당 미터

    # ── 동적 장애물 회피 반경 (격자 셀 수) ──────────────────────────────
    ROBOT_R = 2

    # ── 정적 장애물 (x_min, x_max, y_min, y_max) ──────────────────────
    _WALLS: List[Tuple[float, float, float, float]] = [
        (-18.0, -16.4, -18.0,  18.0),   # 서쪽 외벽
        ( 16.4,  24.0, -18.0,  18.0),   # 동쪽 외벽
        (-18.0,  24.0,  16.4,  18.0),   # 북쪽 외벽
        (-18.0,  24.0, -18.0, -16.4),   # 남쪽 외벽
    ]

    def __init__(self, agents: list = None) -> None:
        self._agents = agents or []
        self._paths: dict = {}   # agent_name → List[XY] (미니맵 표시용)

        gw = int((self.WX1 - self.WX0) / self.GRID_RES) + 2
        gh = int((self.WY1 - self.WY0) / self.GRID_RES) + 2
        self._gw, self._gh = gw, gh

        # 정적 장애물 비트맵
        self._static: List[List[bool]] = [[False] * gw for _ in range(gh)]
        for x0, x1, y0, y1 in self._WALLS:
            for gy in range(max(0, self._gy(y0) - 1), min(gh, self._gy(y1) + 2)):
                for gx in range(max(0, self._gx(x0) - 1), min(gw, self._gx(x1) + 2)):
                    self._static[gy][gx] = True

        print(f"[PathPlanner] 초기화 완료 — 격자 {gw}×{gh}  에이전트 {len(self._agents)}개")

    def set_agents(self, agents: list) -> None:
        self._agents = agents

    def get_agent_paths(self) -> dict:
        """미니맵용: 현재 저장된 경로 스냅샷 반환."""
        return dict(self._paths)

    # ── 좌표 변환 ──────────────────────────────────────────────────────
    def _gx(self, wx: float) -> int:
        return max(0, min(self._gw - 1, int((wx - self.WX0) / self.GRID_RES)))

    def _gy(self, wy: float) -> int:
        return max(0, min(self._gh - 1, int((wy - self.WY0) / self.GRID_RES)))

    def _w2g(self, wx: float, wy: float) -> Tuple[int, int]:
        return self._gx(wx), self._gy(wy)

    def _g2w(self, gx: int, gy: int) -> XY:
        return (gx * self.GRID_RES + self.WX0 + self.GRID_RES * 0.5,
                gy * self.GRID_RES + self.WY0 + self.GRID_RES * 0.5)

    # ── 셀 통과 가능 여부 ──────────────────────────────────────────────
    def _free(self, gx: int, gy: int,
              dyn: List[Tuple[int, int]] = None) -> bool:
        if not (0 <= gx < self._gw and 0 <= gy < self._gh):
            return False
        if self._static[gy][gx]:
            return False
        if dyn:
            r = self.ROBOT_R
            for ox, oy in dyn:
                if abs(gx - ox) <= r and abs(gy - oy) <= r:
                    return False
        return True

    # ── A* 경로 계획 ──────────────────────────────────────────────────
    def plan(self, start: XY, goal: XY,
             agent_name: str = None) -> List[XY]:
        """
        start → goal 의 A* 최단 경로 반환.
        agent_name 이 지정되면 해당 에이전트 외 나머지를 동적 장애물로 사용.
        결과는 _paths[agent_name] 에 저장 (미니맵 표시용).
        """
        # 동적 장애물 수집
        dyn: List[Tuple[int, int]] = []
        for ag in self._agents:
            if agent_name and ag.name == agent_name:
                continue
            try:
                if hasattr(ag, "get_world_xy"):
                    pos = ag.get_world_xy()
                    dyn.append(self._w2g(float(pos[0]), float(pos[1])))
            except Exception:
                pass

        sx, sy = self._w2g(*start)
        gx, gy = self._w2g(*goal)

        if (sx, sy) == (gx, gy):
            path = [goal]
            if agent_name:
                self._paths[agent_name] = path
            return path

        DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_h: list = []
        heapq.heappush(open_h, (0.0, sx, sy))
        came: dict = {}
        g_cost: dict = {(sx, sy): 0.0}

        while open_h:
            _, cx, cy = heapq.heappop(open_h)
            if (cx, cy) == (gx, gy):
                pts, cur = [], (cx, cy)
                while cur in came:
                    pts.append(self._g2w(*cur))
                    cur = came[cur]
                pts.reverse()
                pts.append(goal)
                smooth = self._smooth(pts)
                if agent_name:
                    self._paths[agent_name] = smooth
                return smooth

            for dx, dy in DIRS:
                nx, ny = cx + dx, cy + dy
                if not self._free(nx, ny, dyn):
                    continue
                ng = g_cost[(cx, cy)] + math.hypot(dx, dy)
                if (nx, ny) not in g_cost or ng < g_cost[(nx, ny)]:
                    came[(nx, ny)] = (cx, cy)
                    g_cost[(nx, ny)] = ng
                    h = math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_h, (ng + h, nx, ny))

        # 경로 없음 — 직선 반환
        path = [goal]
        if agent_name:
            self._paths[agent_name] = path
        return path

    # ── 경로 스무딩 (직선 가시성 기반) ───────────────────────────────────
    def _smooth(self, path: List[XY]) -> List[XY]:
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._los(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _los(self, a: XY, b: XY) -> bool:
        """두 월드 좌표 사이의 직선 가시성 (정적 장애물만 체크)."""
        ax, ay = self._w2g(*a)
        bx, by = self._w2g(*b)
        steps = max(abs(bx - ax), abs(by - ay))
        if steps == 0:
            return True
        for i in range(steps + 1):
            t = i / steps
            gx = round(ax + t * (bx - ax))
            gy = round(ay + t * (by - ay))
            if not (0 <= gx < self._gw and 0 <= gy < self._gh):
                return False
            if self._static[gy][gx]:
                return False
        return True

    # ── 편의 메서드: 여러 웨이포인트를 A* 로 연결 ────────────────────────
    def plan_patrol(self, waypoints: List[XY],
                    agent_name: str = None) -> List[XY]:
        """
        순환 웨이포인트 리스트를 A* 로 연결한 확장 경로 반환.
        정찰 경로 계획 시 사용.
        """
        if not waypoints:
            return []
        full: List[XY] = []
        n = len(waypoints)
        for i in range(n):
            seg = self.plan(waypoints[i], waypoints[(i + 1) % n], agent_name)
            full.extend(seg[:-1])   # 마지막 점은 다음 세그먼트 첫 점과 겹침
        full.append(waypoints[0])   # 순환 완성
        if agent_name:
            self._paths[agent_name] = full
        return full
