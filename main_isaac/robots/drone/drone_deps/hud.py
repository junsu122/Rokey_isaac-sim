"""
drone_control/hud.py
MinimalDroneHUD — 경량 상태 패널.

Depth camera / heatmap 패널을 완전히 제거하여 성능을 최대화한다.
omni.ui 텍스트 라벨만 갱신하므로 Python 이미지 변환 비용이 없다.

사용:
    hud = MinimalDroneHUD(controller)
    # physics step 마다 (HUD_UPDATE_N 주기로 throttle 적용):
    hud.update_status(active_input, is_airborne, drone_pos, target_pos)
"""

import numpy as np
import omni.ui as ui
import carb


class MinimalDroneHUD:
    """
    경량 드론 HUD — 컨트롤 힌트 + 비행 상태 + 목표 입력 패널.
    이미지 처리 없이 텍스트 라벨만 갱신하므로 CPU 부하 최소.

      ┌─ Drone Controls ───────────────────────────────┐
      │  KEYBOARD 단축키                               │
      │  JOYSTICK 단축키                               │
      │  Active input: keyboard / joystick             │
      │  Status: on ground / AIRBORNE alt=1.5m         │
      │  pos (x, y, z)   goal dist: d m                │
      │  ─────────────────────────────────────────     │
      │  MANUAL GOAL  X:___  Y:___  Z:___              │
      │  [Go]  [Land]                                  │
      └────────────────────────────────────────────────┘
    """

    def __init__(self, controller, build_window: bool = True):
        self._ctrl         = controller
        self._lbl_active   = None
        self._lbl_status   = None
        self._lbl_pos      = None
        self._fx = self._fy = self._fz = None
        self._win          = None
        if build_window:
            self._win = ui.Window("Drone Controls", width=420, height=280)
            with self._win.frame:
                self.build_ui()

    # ── UI 구성 ──────────────────────────────────────────────────────

    def build_ui(self):
        with ui.VStack(spacing=3, style={"margin": 8}):

            sh = {"font_size": 13, "color": 0xFFFFFF55}
            sc = {"font_size": 12, "color": 0xFFCCCCCC}
            sy = {"font_size": 12, "color": 0xFF55FFFF}

            # 키보드 힌트
            ui.Label("KEYBOARD  (always active)", style=sh, height=18)
            ui.Label("  T = Takeoff   L = Land", style=sc, height=15)
            ui.Label("  W/S = Fwd/Back   A/D = Strafe", style=sc, height=15)
            ui.Label("  Q/E = Yaw Left/Right   ↑↓ = Altitude", style=sc, height=15)

            ui.Spacer(height=3)

            # 조이스틱 힌트
            ui.Label("JOYSTICK  (overrides KB when sticks pushed)", style=sy, height=16)
            ui.Label("  A/× = Takeoff   B/○ = Land", style=sc, height=15)
            ui.Label("  L-stick = Fwd/Strafe   R-stick = Yaw/Alt", style=sc, height=15)

            ui.Spacer(height=4)
            ui.Line(style={"color": 0xFF444444}, height=2)
            ui.Spacer(height=3)

            # 라이브 상태
            self._lbl_active = ui.Label(
                "Active input: keyboard",
                style={"color": 0xFF88FF88, "font_size": 12}, height=16)
            self._lbl_status = ui.Label(
                "Status: on ground",
                style={"color": 0xFFFFCC44, "font_size": 12}, height=16)
            self._lbl_pos = ui.Label(
                "pos: —   goal dist: —",
                style={"color": 0xFF88CCFF, "font_size": 11}, height=16)

            ui.Spacer(height=4)
            ui.Line(style={"color": 0xFF444444}, height=2)
            ui.Spacer(height=3)

            # 목표 입력
            ui.Label("MANUAL GOAL", style=sh, height=18)

            def _row(label, default):
                with ui.HStack(height=26, spacing=6):
                    ui.Label(label, width=24,
                             style={"color": 0xFFDDDDDD, "font_size": 12})
                    field = ui.FloatField(width=ui.Fraction(1), height=24)
                    field.model.set_value(default)
                return field

            self._fx = _row("X:", 0.0)
            self._fy = _row("Y:", 0.0)
            self._fz = _row("Z:", 1.5)

            with ui.HStack(height=32, spacing=8):
                go   = ui.Button("Go",   height=30,
                                 style={"background_color": 0xFF226622,
                                        "color": 0xFFFFFFFF})
                land = ui.Button("Land", height=30,
                                 style={"background_color": 0xFF662222,
                                        "color": 0xFFFFFFFF})
                go.set_clicked_fn(self._on_go)
                land.set_clicked_fn(self._on_land)

    # ── 버튼 콜백 ────────────────────────────────────────────────────

    def _on_go(self):
        gx = self._fx.model.get_value_as_float()
        gy = self._fy.model.get_value_as_float()
        gz = max(0.1, self._fz.model.get_value_as_float())
        self._ctrl.target_pos  = np.array([gx, gy, gz])
        self._ctrl.is_airborne = True
        self._ctrl.integral    = np.zeros(3)
        carb.log_warn(f"[HUD] Go → ({gx:.2f}, {gy:.2f}, {gz:.2f})")

    def _on_land(self):
        self._ctrl.target_pos[2] = 0.07
        self._ctrl.is_airborne   = False
        carb.log_warn("[HUD] Land")

    # ── 상태 갱신 (physics step 마다 호출 — 텍스트 라벨만 갱신) ───────

    def update_status(self, active_input: str, is_airborne: bool,
                      drone_pos: np.ndarray, target_pos: np.ndarray):
        if self._lbl_active:
            col = 0xFF55FFFF if active_input == "joystick" else 0xFFFFFF55
            self._lbl_active.style = {"color": col, "font_size": 12}
            self._lbl_active.text  = f"Active input: {active_input}"
        if self._lbl_status:
            state = (f"AIRBORNE  alt={drone_pos[2]:.2f} m"
                     if is_airborne else "on ground")
            self._lbl_status.text = f"Status: {state}"
        if self._lbl_pos:
            dist = float(np.linalg.norm(drone_pos - target_pos))
            self._lbl_pos.text = (
                f"pos ({drone_pos[0]:.1f}, {drone_pos[1]:.1f}, {drone_pos[2]:.1f})"
                f"   goal dist: {dist:.2f} m"
            )
