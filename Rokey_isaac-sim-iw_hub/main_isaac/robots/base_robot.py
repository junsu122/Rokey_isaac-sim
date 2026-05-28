"""
main_isaac/robots/base_robot.py
================================
모든 로봇 에이전트의 추상 베이스 클래스.

추가할 로봇은 이 클래스를 상속하고
setup / post_reset / on_physics_step 을 구현하면 됩니다.
"""
from abc import ABC, abstractmethod


class BaseRobotAgent(ABC):
    """
    인터페이스 요약
    ─────────────────────────────────────────────────────
    setup()              world.reset() 전 호출.
                         USD prim 생성, 씬 오브젝트 배치.

    post_reset()         world.reset() 후 호출.
                         Articulation·카메라·컨트롤러 초기화.

    on_physics_step(dt)  physics 콜백 (500 Hz).
                         로봇 제어 루프 실행.

    on_render_step()     렌더 루프 (~50 Hz).  기본 구현: no-op.
                         OpenCV 창 업데이트 등 시각화에 사용.
    ─────────────────────────────────────────────────────
    """

    def __init__(self, cfg: dict, world):
        self.cfg       = cfg
        self.name      = cfg["name"]
        self.world     = world
        self.spawn_xyz = tuple(cfg["spawn_xyz"])
        self.spawn_yaw = float(cfg.get("spawn_yaw", 0.0))

    # ── 필수 구현 ────────────────────────────────────────────────────

    @abstractmethod
    def setup(self) -> None:
        """USD prim 생성 / 씬 오브젝트 초기화 (world.reset() 이전)."""

    @abstractmethod
    def post_reset(self) -> None:
        """world.reset() 이후 Articulation·카메라·컨트롤러 초기화."""

    @abstractmethod
    def on_physics_step(self, dt: float) -> None:
        """매 physics step (500 Hz) 호출."""

    # ── 선택 구현 ────────────────────────────────────────────────────

    def on_render_step(self) -> None:
        """렌더 루프 (~50 Hz). 필요시 서브클래스에서 오버라이드."""
        pass
