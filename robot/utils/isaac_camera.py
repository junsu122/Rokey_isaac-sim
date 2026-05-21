"""
Isaac Sim camera interface wrapper.
실제 Isaac Sim 환경에서 카메라 데이터를 가져오는 래퍼입니다.
Isaac Sim 없이 테스트할 경우 MockIsaacCamera를 사용하세요.
"""

import numpy as np


class IsaacCamera:
    """Isaac Sim RGB 카메라 센서 래퍼."""

    def __init__(self, prim_path: str, width: int = 1280, height: int = 720,
                 frequency: int = 30):
        self.prim_path = prim_path
        self.width = width
        self.height = height
        self.frequency = frequency
        self._camera = None
        self._initialized = False

    def initialize(self):
        # Isaac Sim API import는 실행 시점에 수행 (omni 환경 필요)
        from omni.isaac.sensor import Camera  # type: ignore
        import omni.isaac.core.utils.numpy.rotations as rot_utils  # type: ignore

        self._camera = Camera(
            prim_path=self.prim_path,
            resolution=(self.width, self.height),
            frequency=self.frequency,
        )
        self._camera.initialize()
        self._camera.add_motion_vectors_to_frame()
        self._initialized = True

    def get_rgb(self) -> np.ndarray | None:
        """BGR numpy array (H, W, 3) 반환. Isaac Sim 한 스텝 후 호출해야 합니다."""
        if not self._initialized:
            return None
        rgba = self._camera.get_rgba()
        if rgba is None:
            return None
        # RGBA -> BGR
        bgr = rgba[:, :, [2, 1, 0]]
        return bgr.astype(np.uint8)

    def get_intrinsics(self) -> np.ndarray:
        """카메라 내부 행렬 (3x3) 반환."""
        if self._camera is not None:
            return np.array(self._camera.get_intrinsics_matrix())
        raise RuntimeError("Camera not initialized")


class MockIsaacCamera:
    """Isaac Sim 없이 로컬 테스트용 Mock 카메라."""

    def __init__(self, width: int = 1280, height: int = 720,
                 fx: float = 958.8, fy: float = 958.8):
        self.width = width
        self.height = height
        self._fx, self._fy = fx, fy
        self._cx, self._cy = width / 2.0, height / 2.0

    def initialize(self):
        pass

    def get_rgb(self) -> np.ndarray:
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = (40, 40, 40)
        return frame

    def get_intrinsics(self) -> np.ndarray:
        return np.array([
            [self._fx,       0, self._cx],
            [      0, self._fy, self._cy],
            [      0,       0,       1],
        ], dtype=np.float64)
