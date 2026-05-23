"""
drone_control/config.py
Tunable parameters — edit here to change flight behaviour, display sizes,
map bounds, and drone spawn position without touching any other file.
"""

# ── Flight controller ─────────────────────────────────────────────────────── #
TAKEOFF_ALT   = 1.5      # m above ground on takeoff
MOVE_SPEED    = 3.0      # m/s linear movement speed
YAW_RATE_DEG  = 90.0     # deg/s yaw rotation speed

# PID / attitude-control gains (same scalar applied on all 3 axes)
KP         = 10.0
KD         = 8.5
KI         = 1.5
KR         = 3.5
KW         = 0.5
DRONE_MASS = 1.5     # kg (must match Iris model)

# ── Depth camera (ray-cast sensor) ───────────────────────────────────────── #
DEPTH_RES_W    = 64      # ray-cast grid width  (cols)
DEPTH_RES_H    = 48      # ray-cast grid height (rows)
DEPTH_FOV_DEG  = 70.0    # horizontal field-of-view in degrees
DEPTH_MAX_M    = 15.0    # max sensing range in metres
DEPTH_UPDATE_N = 8       # capture every N simulation steps (performance)

# ── HUD display sizes ────────────────────────────────────────────────────── #
HUD_DEPTH_W = 320        # depth image panel width  (px)
HUD_DEPTH_H = 240        # depth image panel height (px)
HUD_MAP_W   = 480        # minimap image width  (px)
HUD_MAP_H   = 300        # minimap image height (px)

# Minimap world bounds (metres) — widen if you fly further out
MAP_X0, MAP_X1 = -10.0, 10.0
MAP_Y0, MAP_Y1 = -10.0, 10.0

# ── Drone spawn ───────────────────────────────────────────────────────────── #
SPAWN_X = 0.0
SPAWN_Y = 0.0
SPAWN_Z = 0.07   # just above ground

# ── Front camera (RGB viewport) ───────────────────────────────────────────── #
CAM_PRIM_PATH    = "/World/quadrotor/body/FrontCamera"
CAM_FOCAL_LENGTH = 18.0    # mm  (≈70° H-FOV for the Iris body size)
CAM_MOUNT_FWD    = 0.15    # m ahead of body centre
