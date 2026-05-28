"""
drone_control/config.py
Tunable parameters — edit here to change flight behaviour without touching other files.
"""
import math

# ── Flight controller ─────────────────────────────────────────────────────── #
TAKEOFF_ALT   = 1.5      # m above ground on takeoff
MOVE_SPEED    = 1.0      # m/s linear movement speed (낮출수록 안정적)
YAW_RATE_DEG  = 90.0     # deg/s yaw rotation speed

# PID / attitude-control gains
KP         = 10.0
KD         = 8.5
KI         = 1.5
KR         = 3.5
KW         = 0.5
DRONE_MASS = 1.5     # kg (must match Iris model)

# Hover trim: rotor speed (rad/s) required to counteract gravity when grounded.
# Derived from: F = ct * omega^2, ct=8.54858e-6 N/(rad/s)^2 (Pegasus Iris default),
# 4 rotors share the load → omega = sqrt(m*g / (4*ct))
HOVER_TRIM = math.sqrt(DRONE_MASS * 9.81 / (4 * 8.54858e-6))  # ≈ 655.9 rad/s

# ── HUD 업데이트 주기 ────────────────────────────────────────────────────── #
# physics 500Hz 에서 HUD 라벨 갱신은 50Hz 이면 충분하다.
HUD_UPDATE_N = 10    # HUD 갱신 주기 (physics steps 기준, 500/10 = 50 Hz)

# ── Front camera (RGB viewport) ───────────────────────────────────────────── #
CAM_FOCAL_LENGTH = 18.0    # mm  (≈70° H-FOV for the Iris body size)
CAM_MOUNT_FWD    = 0.15    # m ahead of body centre
