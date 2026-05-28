"""
drone_deps/input_monitor.py
DroneInputMonitor — real-time joystick / keyboard input visualiser.

Shows:
  • Left / Right stick axis bars (live, center = 0)
  • Button flash indicators (highlight for 0.4 s after press)
  • Scrolling event log (most recent at top)
"""

import time
import omni.ui as ui


_FLASH_SEC = 0.40   # seconds a button stays highlighted after press

# (code, top_label, bottom_label)
_JOY_BTNS = [
    ("BTN_SOUTH", "A / ×",  "Takeoff"),
    ("BTN_EAST",  "B / ○",  "Land"),
    ("BTN_NORTH", "Y / △",  "Grab"),
    ("BTN_WEST",  "X / □",  "Release"),
]
_KB_BTNS = [
    ("KEY_T", "T",  "Takeoff"),
    ("KEY_L", "L",  "Land"),
    ("KEY_Z", "Z",  "Grab"),
    ("KEY_C", "C",  "Release"),
]

_COLOR_ACTIVE   = 0xFF1A6B1A   # dark green (button active)
_COLOR_INACTIVE = 0xFF2A2A2A   # dark grey  (button idle)
_COLOR_TXT_ON   = 0xFF88FF88   # bright green text
_COLOR_TXT_OFF  = 0xFF777777   # dim text


class DroneInputMonitor:
    """
    Separate omni.ui window that visualises drone joystick / keyboard input.
    Call update() each HUD tick (e.g. every 10 physics steps ≈ 50 Hz).
    """

    _LOG_ROWS = 8

    def __init__(self, controller):
        self._ctrl         = controller
        self._axis_widgets = {}   # axis_key → (pb_model, val_label)
        self._btn_widgets  = {}   # code     → (rect, label)
        self._log_labels   = []

        self._win = ui.Window(
            "Drone Input Monitor",
            width=440, height=420,
            flags=ui.WINDOW_FLAGS_NO_SCROLLBAR,
        )
        with self._win.frame:
            self._build()

    # ── UI construction ───────────────────────────────────────────────

    def _build(self):
        SH = {"font_size": 13, "color": 0xFFFFFF55}   # section header yellow
        SD = {"font_size": 11, "color": 0xFFAAAAAA}   # dim label

        with ui.VStack(spacing=3, style={"margin": 8}):

            # ── Left stick ────────────────────────────────────────────
            ui.Label("LEFT STICK", style=SH, height=17)
            self._make_axis_row("forward_back", "Fwd / Back")
            self._make_axis_row("strafe",       "Strafe    ")

            ui.Spacer(height=4)

            # ── Right stick ───────────────────────────────────────────
            ui.Label("RIGHT STICK", style=SH, height=17)
            self._make_axis_row("yaw",      "Yaw       ")
            self._make_axis_row("altitude", "Altitude  ")

            ui.Spacer(height=6)
            ui.Line(style={"color": 0xFF444444}, height=1)
            ui.Spacer(height=4)

            # ── Joystick buttons ──────────────────────────────────────
            ui.Label("JOYSTICK BUTTONS", style=SH, height=17)
            with ui.HStack(height=40, spacing=5):
                for code, top, bot in _JOY_BTNS:
                    self._make_btn_cell(code, top, bot)

            ui.Spacer(height=3)

            # ── Keyboard buttons ──────────────────────────────────────
            ui.Label("KEYBOARD", style=SH, height=17)
            with ui.HStack(height=40, spacing=5):
                for code, top, bot in _KB_BTNS:
                    self._make_btn_cell(code, top, bot)

            ui.Spacer(height=6)
            ui.Line(style={"color": 0xFF444444}, height=1)
            ui.Spacer(height=4)

            # ── Event log ─────────────────────────────────────────────
            ui.Label("EVENT LOG  (newest first)", style=SH, height=17)
            for _ in range(self._LOG_ROWS):
                lbl = ui.Label("", style=SD, height=14)
                self._log_labels.append(lbl)

    def _make_axis_row(self, key: str, name: str):
        """One row: name + centered progress bar + value label."""
        with ui.HStack(height=20, spacing=6):
            ui.Label(name, width=72,
                     style={"font_size": 11, "color": 0xFFCCCCCC})
            # Single bar: 0.0 = full left (-1), 0.5 = centre (0), 1.0 = full right (+1)
            pb = ui.ProgressBar(height=16)
            pb.model.set_value(0.5)
            val = ui.Label("+0.00", width=40,
                           style={"font_size": 11, "color": _COLOR_TXT_OFF})
        self._axis_widgets[key] = (pb.model, val)

    def _make_btn_cell(self, code: str, top: str, bot: str):
        """Coloured indicator cell that flashes on press."""
        with ui.ZStack(width=ui.Fraction(1), height=38):
            rect  = ui.Rectangle(style={"background_color": _COLOR_INACTIVE,
                                         "border_radius": 5})
            with ui.VStack():
                ui.Spacer()
                t_lbl = ui.Label(top, alignment=ui.Alignment.CENTER,
                                 style={"font_size": 10, "color": _COLOR_TXT_OFF},
                                 height=14)
                b_lbl = ui.Label(bot, alignment=ui.Alignment.CENTER,
                                 style={"font_size": 9,  "color": _COLOR_TXT_OFF},
                                 height=12)
                ui.Spacer()
        self._btn_widgets[code] = (rect, t_lbl, b_lbl)

    # ── Live update ───────────────────────────────────────────────────

    def update(self):
        """Call this at HUD refresh rate (~50 Hz) from on_physics_step."""
        self._update_axes()
        self._update_buttons()
        self._update_log()

    def _update_axes(self):
        try:
            with self._ctrl._axes_lock:
                axes = dict(self._ctrl._axes)
        except Exception:
            return
        for key, (model, lbl) in self._axis_widgets.items():
            v = float(axes.get(key, 0.0))
            model.set_value((v + 1.0) / 2.0)   # -1..+1  →  0..1
            lbl.text  = f"{v:+.2f}"
            lbl.style = {
                "font_size": 11,
                "color": _COLOR_TXT_ON if abs(v) > 0.05 else _COLOR_TXT_OFF,
            }

    def _update_buttons(self):
        now = time.monotonic()
        entries = self._read_log()
        # Most recent press time per code
        recent: dict[str, float] = {}
        for ts, _text, code in entries:
            if code and code not in recent:
                recent[code] = ts

        for code, (rect, t_lbl, b_lbl) in self._btn_widgets.items():
            active = (now - recent.get(code, 0.0)) < _FLASH_SEC
            rect.style  = {"background_color": _COLOR_ACTIVE   if active else _COLOR_INACTIVE,
                            "border_radius": 5}
            t_lbl.style = {"font_size": 10,
                            "color": _COLOR_TXT_ON if active else _COLOR_TXT_OFF}
            b_lbl.style = {"font_size": 9,
                            "color": _COLOR_TXT_ON if active else _COLOR_TXT_OFF}

    def _update_log(self):
        entries = self._read_log()
        now = time.monotonic()
        for i, lbl in enumerate(self._log_labels):
            if i < len(entries):
                ts, text, _code = entries[i]
                ago = now - ts
                lbl.text  = f"-{ago:5.1f}s   {text}"
                lbl.style = {
                    "font_size": 11,
                    "color": 0xFFFFFFFF if ago < 1.0 else 0xFF888888,
                }
            else:
                lbl.text = ""

    def _read_log(self) -> list:
        lock = getattr(self._ctrl, '_input_log_lock', None)
        log  = getattr(self._ctrl, '_input_log', [])
        if lock:
            with lock:
                return list(log)
        return list(log)
