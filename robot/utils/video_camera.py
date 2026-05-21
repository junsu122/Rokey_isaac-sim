"""
Video file / webcam camera source for offline testing.
"""

import cv2
import numpy as np


class VideoCamera:
    """비디오 파일 또는 웹캠을 카메라 소스로 사용합니다."""

    def __init__(self, source: str | int):
        self.source = source
        self._cap: cv2.VideoCapture | None = None

    def initialize(self):
        self._cap = cv2.VideoCapture(self.source)
        if not self._cap.isOpened():
            raise RuntimeError(f"카메라/비디오 소스를 열 수 없습니다: {self.source}")

    def get_rgb(self) -> np.ndarray | None:
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        if not ret:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 비디오 루프
            ret, frame = self._cap.read()
        return frame if ret else None

    def get_intrinsics(self) -> np.ndarray:
        w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        f = max(w, h)
        return np.array([
            [f,   0, w / 2],
            [0,   f, h / 2],
            [0,   0,     1],
        ], dtype=np.float64)

    def __del__(self):
        if self._cap:
            self._cap.release()
