"""
drone_control/depth_camera.py
SoftwareDepthCamera  — forward-facing depth sensor via PhysX ray-casting.
FrustumDrawer        — 3-D debug overlay showing camera FOV in the viewport.

PhysX raycasts require no GPU render pipeline, making them safe on any
driver/hardware combination (including RTX 5080 where GPU depth bugs exist).
"""

import numpy as np
import carb

from drone_config import (
    DEPTH_RES_W, DEPTH_RES_H, DEPTH_FOV_DEG,
    DEPTH_MAX_M, CAM_MOUNT_FWD,
)


class SoftwareDepthCamera:
    """Forward-facing depth sensor mounted 15 cm in front of the drone body."""

    def __init__(self):
        W, H = DEPTH_RES_W, DEPTH_RES_H
        self.W, self.H = W, H
        self.max_range = DEPTH_MAX_M

        fov_h = np.deg2rad(DEPTH_FOV_DEG)
        fov_v = fov_h * H / W
        h_ang = (np.arange(W) / W - 0.5) * fov_h
        v_ang = (0.5 - np.arange(H) / H) * fov_v

        H_g, V_g = np.meshgrid(h_ang, v_ang)
        rays = np.stack([
             np.cos(V_g) * np.cos(H_g),   # x  (forward in drone body frame)
            -np.cos(V_g) * np.sin(H_g),   # y
             np.sin(V_g),                  # z  (up)
        ], axis=-1)
        norms = np.linalg.norm(rays, axis=-1, keepdims=True)
        self._rays_body  = (rays / norms).reshape(-1, 3)
        self._physx      = None
        self._call_count = 0
        self._last_depth = None

    def initialize(self):
        from omni.physx import get_physx_scene_query_interface
        self._physx = get_physx_scene_query_interface()
        carb.log_warn(f"[DepthCam] {self.W}×{self.H} px PhysX camera ready.")

    def capture(self, drone_pos, drone_R, update_every=8):
        """Return (H, W) float32 depth array in metres. Cached every N calls."""
        self._call_count += 1
        if self._call_count % update_every != 0 and self._last_depth is not None:
            return self._last_depth
        if self._physx is None:
            return np.full((self.H, self.W), self.max_range, dtype=np.float32)

        cam_origin = drone_pos + drone_R.apply(
            np.array([CAM_MOUNT_FWD, 0.0, 0.0]))
        origin     = tuple(float(v) for v in cam_origin)
        rays_world = drone_R.apply(self._rays_body)

        depth_flat = np.full(self.H * self.W, self.max_range, dtype=np.float32)
        for i, ray in enumerate(rays_world):
            res = self._physx.raycast_closest(
                origin,
                (float(ray[0]), float(ray[1]), float(ray[2])),
                self.max_range,
            )
            if res and res.get("hit", False):
                depth_flat[i] = float(res["distance"])

        self._last_depth = depth_flat.reshape(self.H, self.W)
        return self._last_depth


class FrustumDrawer:
    """Draws the depth-camera FOV frustum as 3-D lines in the Isaac Sim viewport."""

    def __init__(self):
        self._draw = None

    def initialize(self):
        for path in ("omni.isaac.debug_draw._debug_draw",
                     "isaacsim.util.debug_draw._debug_draw"):
            try:
                import importlib
                self._draw = importlib.import_module(path).acquire_debug_draw_interface()
                carb.log_warn("[FrustumDrawer] 3-D overlay active.")
                return
            except Exception:
                pass
        carb.log_warn("[FrustumDrawer] debug_draw not available — overlay skipped.")

    def update(self, cam_origin, rays_body, drone_R, cam):
        if self._draw is None:
            return
        self._draw.clear_points()
        self._draw.clear_lines()
        o = tuple(float(v) for v in cam_origin)
        self._draw.draw_points([o], [(1.0, 1.0, 0.0, 1.0)], [14.0])

        idx = [0, cam.W - 1, (cam.H - 1) * cam.W, cam.H * cam.W - 1]
        corners_w = drone_R.apply(rays_body[idx])
        FAR = 2.5
        far_pts = [cam_origin + r * FAR for r in corners_w]
        for fp in far_pts:
            self._draw.draw_lines([o], [tuple(float(v) for v in fp)],
                                  [(0.0, 0.9, 1.0, 0.9)], [3.0])
        for a, b in [(0, 1), (1, 3), (3, 2), (2, 0)]:
            self._draw.draw_lines(
                [tuple(float(v) for v in far_pts[a])],
                [tuple(float(v) for v in far_pts[b])],
                [(0.0, 0.9, 1.0, 0.5)], [2.0])
