"""
drone_control/hud.py
DroneHUD — single unified omni.ui window containing every panel:

  LEFT  column  — keyboard/joystick control hints, live status, manual goal input
  RIGHT column  — depth camera (grayscale + false-colour) + top-down minimap

Usage (called from app.py each simulation step):
    hud.update_depth(depth_array, altitude)
    hud.update_status(active_input, is_airborne, drone_pos, target_pos)
    hud.update_map(drone_pos, drone_R, target_pos)
"""

import numpy as np
import omni.ui as ui
import cv2
import carb

from drone_config import (
    HUD_DEPTH_W, HUD_DEPTH_H,
    HUD_MAP_W,   HUD_MAP_H,
    MAP_X0, MAP_X1, MAP_Y0, MAP_Y1,
    DEPTH_MAX_M,
)


class DroneHUD:
    """
    One window — no split tabs, no floating panels.

      ┌─ Drone Control Center ──────────────────────────────────────────────┐
      │ LEFT (controls)          │ RIGHT                                    │
      │  KEYBOARD hints          │  DEPTH GRAY   |  DEPTH COLOR             │
      │  JOYSTICK hints          │  depth info label                        │
      │  Active input / Status   │  ─────────────────────────────────────   │
      │  Position / goal dist    │  MINIMAP (top-down grid + drone + goal)  │
      │  ─────────────────────   │  map status label                        │
      │  MANUAL GOAL X/Y/Z       │                                          │
      │  [Go]  [Land]            │                                          │
      └──────────────────────────┴──────────────────────────────────────────┘
    """

    def __init__(self, controller):
        self._ctrl = controller

        # omni.ui image providers (updated every frame)
        self._prov_gray  = ui.ByteImageProvider()
        self._prov_color = ui.ByteImageProvider()
        self._prov_map   = ui.ByteImageProvider()

        # Live labels
        self._lbl_active   = None
        self._lbl_status   = None
        self._lbl_pos      = None
        self._lbl_depth    = None
        self._lbl_map      = None

        # Goal input fields
        self._fx = self._fy = self._fz = None

        self._build()

    # ── UI construction ────────────────────────────────────────────────────── #

    def _build(self):
        left_w  = 400
        right_w = HUD_DEPTH_W * 2 + 30     # 670 px (two depth images side-by-side)
        total_w = left_w + right_w + 32     # ~1102 px
        total_h = HUD_DEPTH_H + HUD_MAP_H + 140

        self._win = ui.Window("Drone Control Center", width=total_w, height=total_h)
        with self._win.frame:
            with ui.HStack(spacing=8, style={"margin": 8}):
                self._build_left(left_w)
                self._build_right()

    def _build_left(self, width):
        sh = {"font_size": 13, "color": 0xFFFFFF55}    # section header
        sc = {"font_size": 12, "color": 0xFFCCCCCC}    # normal line
        sy = {"font_size": 13, "color": 0xFF55FFFF}    # joystick accent

        with ui.ScrollingFrame(
            width=width,
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
        ):
            with ui.VStack(spacing=3):

                # ── Keyboard hints ───────────────────────────────────────── #
                ui.Label("KEYBOARD  (always active)", style=sh, height=18)
                ui.Label("  T = Takeoff   L = Land", style=sc, height=16)
                ui.Label("  W/S = Fwd/Back   A/D = Strafe", style=sc, height=16)
                ui.Label("  Q/E = Yaw   ↑↓ = Altitude", style=sc, height=16)

                ui.Spacer(height=4)

                # ── Joystick hints ───────────────────────────────────────── #
                ui.Label("JOYSTICK  (overrides KB when sticks pushed)", style=sy, height=18)
                ui.Label("  A / × = Takeoff   B / ○ = Land", style=sc, height=16)
                ui.Label("  L-stick = Fwd / Strafe", style=sc, height=16)
                ui.Label("  R-stick = Yaw / Altitude", style=sc, height=16)

                ui.Spacer(height=6)

                # ── Live status ──────────────────────────────────────────── #
                self._lbl_active = ui.Label(
                    "Active input: keyboard",
                    style={"color": 0xFF88FF88, "font_size": 12}, height=16)
                self._lbl_status = ui.Label(
                    "Status: on ground",
                    style={"color": 0xFFFFCC44, "font_size": 12}, height=16)
                self._lbl_pos = ui.Label(
                    "pos: —   goal dist: —",
                    style={"color": 0xFF88CCFF, "font_size": 11}, height=16)

                ui.Spacer(height=6)
                ui.Line(style={"color": 0xFF555555}, height=2)
                ui.Spacer(height=5)

                # ── Manual goal input ────────────────────────────────────── #
                ui.Label("MANUAL GOAL", style=sh, height=18)

                def _row(label, default):
                    with ui.HStack(height=26, spacing=6):
                        ui.Label(label, width=24,
                                 style={"color": 0xFFDDDDDD})
                        field = ui.FloatField(width=ui.Fraction(1), height=24)
                        field.model.set_value(default)
                    return field

                self._fx = _row("X:", 0.0)
                self._fy = _row("Y:", 0.0)
                self._fz = _row("Z:", 1.5)

                with ui.HStack(height=34, spacing=8):
                    go   = ui.Button("Go", height=32,
                                     style={"background_color": 0xFF226622,
                                            "color": 0xFFFFFFFF})
                    land = ui.Button("Land", height=32,
                                     style={"background_color": 0xFF662222,
                                            "color": 0xFFFFFFFF})
                    go.set_clicked_fn(self._on_go)
                    land.set_clicked_fn(self._on_land)

                ui.Spacer(height=8)
                ui.Label("  RGB camera → 'Drone Front Camera' viewport",
                         style={"color": 0xFF777777, "font_size": 11}, height=16)

    def _build_right(self):
        with ui.VStack(spacing=4):

            # ── Depth camera ─────────────────────────────────────────────── #
            ui.Label("DEPTH CAMERA",
                     style={"color": 0xFFFFFF77, "font_size": 13}, height=18)
            with ui.HStack(spacing=6):
                with ui.VStack(spacing=2):
                    ui.Label("GRAY  white=close / black=far",
                             height=16,
                             style={"color": 0xFFBBBBBB, "font_size": 11})
                    ui.ImageWithProvider(self._prov_gray,
                                        width=HUD_DEPTH_W, height=HUD_DEPTH_H)
                with ui.VStack(spacing=2):
                    ui.Label("FALSE COLOR  red=close / blue=far",
                             height=16,
                             style={"color": 0xFFBBBBBB, "font_size": 11})
                    ui.ImageWithProvider(self._prov_color,
                                        width=HUD_DEPTH_W, height=HUD_DEPTH_H)
            self._lbl_depth = ui.Label(
                "waiting…", height=18,
                style={"color": 0xFF88FF88, "font_size": 11})

            ui.Spacer(height=6)
            ui.Line(style={"color": 0xFF555555}, height=2)
            ui.Spacer(height=4)

            # ── Minimap ───────────────────────────────────────────────────── #
            ui.Label("MINIMAP  (top-down)",
                     style={"color": 0xFFFFFF77, "font_size": 13}, height=18)
            ui.ImageWithProvider(self._prov_map,
                                 width=HUD_MAP_W, height=HUD_MAP_H)
            self._lbl_map = ui.Label(
                "—", height=18,
                style={"color": 0xFFFFFF44, "font_size": 11})

    # ── Button callbacks ───────────────────────────────────────────────────── #

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

    # ── Per-frame update methods ───────────────────────────────────────────── #

    @staticmethod
    def _bgr_to_rgba(bgr: np.ndarray) -> list:
        h, w = bgr.shape[:2]
        rgba = np.empty((h, w, 4), dtype=np.uint8)
        rgba[:, :, 0] = bgr[:, :, 2]
        rgba[:, :, 1] = bgr[:, :, 1]
        rgba[:, :, 2] = bgr[:, :, 0]
        rgba[:, :, 3] = 255
        return list(rgba.tobytes())

    def update_depth(self, depth: np.ndarray, altitude: float):
        """Render depth array into the gray + false-colour panels."""
        H, W  = depth.shape
        valid = depth[depth < DEPTH_MAX_M * 0.99]
        d_min = float(np.percentile(valid, 2))  if valid.size > 0 else 0.0
        d_max = float(np.percentile(valid, 98)) if valid.size > 0 else DEPTH_MAX_M
        span  = max(d_max - d_min, 0.01)
        norm  = 1.0 - np.clip((depth - d_min) / span, 0.0, 1.0)
        u8    = (norm * 255).astype(np.uint8)

        big     = cv2.resize(u8, (HUD_DEPTH_W, HUD_DEPTH_H),
                             interpolation=cv2.INTER_LINEAR)
        blurred = cv2.GaussianBlur(big, (3, 3), 0)
        edges   = cv2.dilate(cv2.Canny(blurred, 35, 90),
                             np.ones((2, 2), np.uint8))
        emask   = edges > 0

        gray  = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
        color = cv2.applyColorMap(big, cv2.COLORMAP_TURBO)
        gray[emask]  = (0, 220, 0)
        color[emask] = (0, 220, 0)

        cx, cy = HUD_DEPTH_W // 2, HUD_DEPTH_H // 2
        for img in (gray, color):
            cv2.line(img, (cx - 14, cy), (cx + 14, cy), (0, 255, 255), 2)
            cv2.line(img, (cx, cy - 14), (cx, cy + 14), (0, 255, 255), 2)

        self._prov_gray.set_bytes_data(
            self._bgr_to_rgba(gray),  [HUD_DEPTH_W, HUD_DEPTH_H])
        self._prov_color.set_bytes_data(
            self._bgr_to_rgba(color), [HUD_DEPTH_W, HUD_DEPTH_H])

        if self._lbl_depth:
            cd = float(depth[H // 2, W // 2])
            self._lbl_depth.text = (
                f"centre: {cd:.2f} m  |  range: {d_min:.1f}–{d_max:.1f} m"
                f"  |  alt: {altitude:.2f} m"
            )

    def update_status(self, active_input: str, is_airborne: bool,
                      drone_pos: np.ndarray, target_pos: np.ndarray):
        """Update the live status labels in the left panel."""
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

    def update_map(self, drone_pos: np.ndarray, drone_R=None,
                   target_pos: np.ndarray = None):
        """Redraw the top-down minimap and push to the image provider."""
        W, H = HUD_MAP_W, HUD_MAP_H
        frame = np.full((H, W, 3), 18, dtype=np.uint8)

        def _w2p(wx, wy):
            px = int((wx - MAP_X0) / (MAP_X1 - MAP_X0) * W)
            py = int(H - (wy - MAP_Y0) / (MAP_Y1 - MAP_Y0) * H)
            return (int(np.clip(px, 0, W - 1)),
                    int(np.clip(py, 0, H - 1)))

        # Grid lines every 2 m
        for gx in range(int(MAP_X0), int(MAP_X1) + 1, 2):
            px, _ = _w2p(gx, 0)
            cv2.line(frame, (px, 0), (px, H), (40, 40, 40), 1)
            if gx % 4 == 0:
                cv2.putText(frame, str(gx), (px + 2, H - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (70, 70, 70), 1)
        for gy in range(int(MAP_Y0), int(MAP_Y1) + 1, 2):
            _, py = _w2p(0, gy)
            cv2.line(frame, (0, py), (W, py), (40, 40, 40), 1)
            if gy % 4 == 0 and gy != 0:
                cv2.putText(frame, str(gy), (2, py - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (70, 70, 70), 1)

        # Origin marker
        ox, oy = _w2p(0.0, 0.0)
        cv2.line(frame, (ox - 6, oy), (ox + 6, oy), (70, 70, 70), 2)
        cv2.line(frame, (ox, oy - 6), (ox, oy + 6), (70, 70, 70), 2)

        # Goal marker (green crosshair)
        if target_pos is not None:
            tx, ty = float(target_pos[0]), float(target_pos[1])
            if MAP_X0 <= tx <= MAP_X1 and MAP_Y0 <= ty <= MAP_Y1:
                tpx, tpy = _w2p(tx, ty)
                cv2.line(frame, (tpx - 10, tpy), (tpx + 10, tpy), (0, 200, 0), 2)
                cv2.line(frame, (tpx, tpy - 10), (tpx, tpy + 10), (0, 200, 0), 2)
                cv2.circle(frame, (tpx, tpy), 5, (0, 200, 0), 1)

        # Drone dot
        dpx, dpy = _w2p(float(drone_pos[0]), float(drone_pos[1]))
        cv2.circle(frame, (dpx, dpy), 9, (50, 210, 255), -1)
        cv2.circle(frame, (dpx, dpy), 9, (255, 255, 255), 1)

        # Heading arrow + 70° FOV cone
        if drone_R is not None:
            sx  = W / (MAP_X1 - MAP_X0)
            sy  = H / (MAP_Y1 - MAP_Y0)
            fwd = drone_R.apply(np.array([1.0, 0.0, 0.0]))

            def _dir_px(d):
                v = np.array([d[0] * sx, -d[1] * sy])
                n = np.linalg.norm(v)
                return v / n if n > 0 else v

            fp  = _dir_px(fwd)
            ae  = (int(dpx + fp[0] * 38), int(dpy + fp[1] * 38))
            cv2.arrowedLine(frame, (dpx, dpy), ae, (50, 220, 255), 2,
                            tipLength=0.35)

            half_fov = np.deg2rad(35.0)
            for sign in (-1, 1):
                ca, sa = np.cos(sign * half_fov), np.sin(sign * half_fov)
                edge   = np.array([ca * fwd[0] - sa * fwd[1],
                                   sa * fwd[0] + ca * fwd[1], 0.0])
                ep     = _dir_px(edge)
                fe     = (int(dpx + ep[0] * 55), int(dpy + ep[1] * 55))
                cv2.line(frame, (dpx, dpy), fe, (30, 160, 255), 1)

        cv2.putText(frame, "DRONE", (dpx - 24, dpy + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (50, 210, 255), 1)

        # Convert BGR→RGBA and push
        rgba = np.empty((H, W, 4), dtype=np.uint8)
        rgba[:, :, 0] = frame[:, :, 2]
        rgba[:, :, 1] = frame[:, :, 1]
        rgba[:, :, 2] = frame[:, :, 0]
        rgba[:, :, 3] = 255
        self._prov_map.set_bytes_data(list(rgba.tobytes()), [W, H])

        if self._lbl_map:
            goal_txt = (f"   goal ({target_pos[0]:.1f}, {target_pos[1]:.1f})"
                        if target_pos is not None else "")
            self._lbl_map.text = (
                f"pos ({drone_pos[0]:.1f}, {drone_pos[1]:.1f})"
                f"   alt {drone_pos[2]:.1f} m{goal_txt}"
            )
