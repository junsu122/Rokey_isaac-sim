"""
drone_control/config.py
Tunable parameters — edit here to change flight behaviour without touching other files.
"""

# ── Flight controller ─────────────────────────────────────────────────────── #
TAKEOFF_ALT   = 1.5      # m above ground on takeoff
MOVE_SPEED    = 3.0      # m/s linear movement speed
YAW_RATE_DEG  = 90.0     # deg/s yaw rotation speed

# PID / attitude-control gains
KP         = 10.0
KD         = 8.5
KI         = 1.5
KR         = 3.5
KW         = 0.5
DRONE_MASS = 1.5     # kg (must match Iris model)

# ── HUD 업데이트 주기 ────────────────────────────────────────────────────── #
# physics 500Hz 에서 HUD 라벨 갱신은 50Hz 이면 충분하다.
HUD_UPDATE_N = 10    # HUD 갱신 주기 (physics steps 기준, 500/10 = 50 Hz)

# ── Front camera (RGB viewport) ───────────────────────────────────────────── #
CAM_FOCAL_LENGTH = 18.0    # mm  (≈70° H-FOV for the Iris body size)
CAM_MOUNT_FWD    = 0.15    # m ahead of body centre
