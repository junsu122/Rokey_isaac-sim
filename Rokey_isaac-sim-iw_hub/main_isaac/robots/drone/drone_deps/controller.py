"""
drone_control/controller.py
HybridController — simultaneous keyboard + joystick flight control.

Keyboard:  T=Takeoff  L=Land  W/S=Fwd/Back  A/D=Strafe  Q/E=Yaw  ↑↓=Alt
Joystick:  A/×=Takeoff  B/○=Land  L-stick=Move/Strafe  R-stick=Yaw/Alt

Joystick sticks override keyboard movement when pushed beyond dead zone.
Keyboard takes over when all sticks are centred.
Takeoff / land from either source always work.

NOTE: import this module only after SimulationApp has been started
      (run.py handles this).
"""

import threading
import numpy as np
from scipy.spatial.transform import Rotation
import carb
from pegasus.simulator.logic.backends import Backend

from drone_config import (
    TAKEOFF_ALT, MOVE_SPEED, YAW_RATE_DEG,
    KP, KD, KI, KR, KW, DRONE_MASS,
)

_AXIS_MAX  = 32767.0
_DEAD_ZONE = 0.12


def _norm_axis(raw: int) -> float:
    v = raw / _AXIS_MAX
    if abs(v) < _DEAD_ZONE:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - _DEAD_ZONE) / (1.0 - _DEAD_ZONE)


class HybridController(Backend):
    """Keyboard + joystick simultaneously — no mode switching needed."""

    def __init__(self):
        self._vehicle = None

        self.takeoff_alt = TAKEOFF_ALT
        self.move_speed  = MOVE_SPEED
        self.yaw_rate    = np.deg2rad(YAW_RATE_DEG)

        # State from vehicle
        self.p = np.zeros(3)
        self.R = Rotation.identity()
        self.v = np.zeros(3)
        self.w = np.zeros(3)

        # Flight targets
        self.target_pos  = np.array([0.0, 0.0, 0.07])
        self.target_yaw  = 0.0
        self.is_airborne = False

        # Controller gains
        self.Kp = np.diag([KP, KP, KP])
        self.Kd = np.diag([KD, KD, KD])
        self.Ki = np.diag([KI, KI, KI])
        self.Kr = np.diag([KR, KR, KR])
        self.Kw = np.diag([KW, KW, KW])
        self.integral = np.zeros(3)
        self.m = DRONE_MASS
        self.g = 9.81

        self.input_ref = [0.0, 0.0, 0.0, 0.0]
        self._received_first_state = False
        self.active_input = "keyboard"

        # Keyboard held-key state
        self._keys = {k: False for k in
                      ('forward', 'backward', 'left', 'right',
                       'up', 'down', 'yaw_l', 'yaw_r')}
        self._input_iface = None
        self._keyboard    = None
        self._key_sub     = None

        # Joystick analog axes (updated from background thread)
        self._axes      = {'forward_back': 0.0, 'strafe': 0.0,
                           'yaw': 0.0, 'altitude': 0.0}
        self._axes_lock = threading.Lock()
        self._joy_thread = None

    # ── Backend lifecycle ──────────────────────────────────────────────────── #

    def start(self):
        carb.log_warn("[Ctrl] start() — subscribing keyboard")
        self.integral    = np.zeros(3)
        self.target_pos  = np.array([0.0, 0.0, 0.07])
        self.target_yaw  = 0.0
        self.is_airborne = False

        try:
            import carb.input as _ci
            import omni.appwindow as _aw
            self._input_iface = _ci.acquire_input_interface()
            self._keyboard    = _aw.get_default_app_window().get_keyboard()
            self._key_sub     = self._input_iface.subscribe_to_keyboard_events(
                self._keyboard, self._on_kb_event)
            carb.log_warn("[Ctrl] keyboard OK")
        except Exception as e:
            carb.log_warn(f"[Ctrl] keyboard failed: {e}")

        try:
            import inputs as _inp
            pads = list(getattr(_inp.devices, 'gamepads', []))
            if pads:
                carb.log_warn(f"[Ctrl] gamepad detected: {pads[0]}")
                self._joy_thread = threading.Thread(
                    target=self._joy_loop, daemon=True)
                self._joy_thread.start()
            else:
                carb.log_warn("[Ctrl] no gamepad found — keyboard only")
        except ImportError:
            carb.log_warn("[Ctrl] 'inputs' not installed — keyboard only")

    def stop(self):
        if self._key_sub and self._input_iface:
            self._input_iface.unsubscribe_to_keyboard_events(
                self._keyboard, self._key_sub)
            self._key_sub = None

    def reset(self):
        self.integral    = np.zeros(3)
        self.target_pos  = np.array([0.0, 0.0, 0.07])
        self.target_yaw  = 0.0
        self.is_airborne = False
        for k in self._keys:
            self._keys[k] = False
        with self._axes_lock:
            for k in self._axes:
                self._axes[k] = 0.0

    # ── Vehicle state callbacks ────────────────────────────────────────────── #

    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass

    def update_state(self, state):
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.v = state.linear_velocity
        self.w = state.angular_velocity
        self._received_first_state = True

    def input_reference(self):
        return self.input_ref

    # ── Keyboard event handler ─────────────────────────────────────────────── #

    def _on_kb_event(self, event, *args, **kwargs):
        import carb.input as _ci
        K, KE = _ci.KeyboardInput, _ci.KeyboardEventType
        active = event.type in (KE.KEY_PRESS, KE.KEY_REPEAT)
        press  = event.type == KE.KEY_PRESS
        key    = event.input

        if   key == K.W:    self._keys['forward']  = active
        elif key == K.S:    self._keys['backward'] = active
        elif key == K.A:    self._keys['left']     = active
        elif key == K.D:    self._keys['right']    = active
        elif key == K.Q:    self._keys['yaw_l']    = active
        elif key == K.E:    self._keys['yaw_r']    = active
        elif key == K.UP:   self._keys['up']       = active
        elif key == K.DOWN: self._keys['down']     = active
        elif key == K.T and press: self._cmd_takeoff()
        elif key == K.L and press: self._cmd_land()
        return True

    # ── Joystick background thread ─────────────────────────────────────────── #

    def _joy_loop(self):
        try:
            from inputs import get_gamepad
        except ImportError:
            return
        while True:
            try:
                for ev in get_gamepad():
                    if ev.ev_type == 'Sync':
                        continue
                    c, s = ev.code, ev.state
                    if   c == 'ABS_Y':
                        with self._axes_lock: self._axes['forward_back'] = -_norm_axis(s)
                    elif c == 'ABS_X':
                        with self._axes_lock: self._axes['strafe']       =  _norm_axis(s)
                    elif c == 'ABS_RX':
                        with self._axes_lock: self._axes['yaw']          =  _norm_axis(s)
                    elif c == 'ABS_RY':
                        with self._axes_lock: self._axes['altitude']     = -_norm_axis(s)
                    elif c == 'BTN_SOUTH' and s == 1: self._cmd_takeoff()
                    elif c == 'BTN_EAST'  and s == 1: self._cmd_land()
            except Exception:
                import time; time.sleep(0.5)

    # ── Takeoff / Land commands ────────────────────────────────────────────── #

    def _cmd_takeoff(self):
        if not self.is_airborne:
            self.target_pos[:2] = self.p[:2]
            self.target_pos[2]  = self.takeoff_alt
            self.target_yaw     = self.R.as_euler('ZYX')[0]
            self.is_airborne    = True
            self.integral       = np.zeros(3)
            carb.log_warn(f"[Drone] TAKEOFF → {self.takeoff_alt:.1f} m")

    def _cmd_land(self):
        if self.is_airborne:
            self.target_pos[2] = 0.07
            self.is_airborne   = False
            carb.log_warn("[Drone] LAND")

    # ── Control loop (called every physics step) ───────────────────────────── #

    def update(self, dt: float):
        if not self._received_first_state:
            return

        if self.is_airborne:
            yaw  = self.target_yaw
            fwd  = np.array([ np.cos(yaw),  np.sin(yaw), 0.0])
            rgt  = np.array([ np.sin(yaw), -np.cos(yaw), 0.0])
            step = self.move_speed * dt

            with self._axes_lock:
                fb  = self._axes['forward_back']
                st  = self._axes['strafe']
                ya  = self._axes['yaw']
                alt = self._axes['altitude']

            joy_active = abs(fb) > 0 or abs(st) > 0 or abs(ya) > 0 or abs(alt) > 0

            if joy_active:
                self.active_input = "joystick"
                self.target_pos  += fwd * (fb  * step)
                self.target_pos  += rgt * (st  * step)
                self.target_pos[2] += alt * step
                self.target_yaw   -= ya  * self.yaw_rate * dt
            else:
                if any(self._keys.values()):
                    self.active_input = "keyboard"
                if self._keys['forward']:  self.target_pos += fwd * step
                if self._keys['backward']: self.target_pos -= fwd * step
                if self._keys['right']:    self.target_pos += rgt * step
                if self._keys['left']:     self.target_pos -= rgt * step
                if self._keys['up']:       self.target_pos[2] += step
                if self._keys['down']:     self.target_pos[2] -= step
                if self._keys['yaw_l']:    self.target_yaw += self.yaw_rate * dt
                if self._keys['yaw_r']:    self.target_yaw -= self.yaw_rate * dt

            self.target_pos[2] = max(0.1, self.target_pos[2])

        # Geometric nonlinear position controller
        ep = self.p - self.target_pos
        self.integral += ep * dt
        F_des = (-(self.Kp @ ep) - (self.Kd @ self.v)
                 - (self.Ki @ self.integral)
                 + np.array([0.0, 0.0, self.m * self.g]))

        Z_B = self.R.as_matrix()[:, 2]
        u_1 = F_des @ Z_B
        F_n = np.linalg.norm(F_des)
        if F_n < 1e-6: return

        Z_b_des   = F_des / F_n
        X_c_des   = np.array([np.cos(self.target_yaw), np.sin(self.target_yaw), 0.0])
        Z_cross_X = np.cross(Z_b_des, X_c_des)
        cn = np.linalg.norm(Z_cross_X)
        if cn < 1e-6: return

        Y_b_des = Z_cross_X / cn
        X_b_des = np.cross(Y_b_des, Z_b_des)
        R_des   = np.c_[X_b_des, Y_b_des, Z_b_des]
        R_cur   = self.R.as_matrix()
        eR_mat  = R_des.T @ R_cur - R_cur.T @ R_des
        e_R     = 0.5 * np.array([-eR_mat[1, 2], eR_mat[0, 2], -eR_mat[0, 1]])
        tau     = -(self.Kr @ e_R) - (self.Kw @ self.w)

        if self._vehicle is not None:
            self.input_ref = self._vehicle.force_and_torques_to_velocities(u_1, tau)
