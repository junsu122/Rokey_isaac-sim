"""
ArUco 검출기 단위 테스트 - Isaac Sim 없이 실행 가능합니다.
실행: python -m pytest tests/ -v
"""

import sys
from pathlib import Path
import numpy as np
import cv2
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.aruco_detector import ArucoDetector, ClassifiedItem


DICT_NAME = "DICT_4X4_50"
IMG_W, IMG_H = 640, 480
FX = FY = 500.0
CX, CY = IMG_W / 2, IMG_H / 2

K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST = np.zeros((4, 1), dtype=np.float64)

REGISTRY = {
    0: {"label": "강남",           "category": "destination", "description": "강남구 배송", "marker_size": 0.04},
    1: {"label": "서초",           "category": "destination", "description": "서초구 배송", "marker_size": 0.04},
    2: {"label": "구로디지털단지", "category": "destination", "description": "구로구 배송", "marker_size": 0.04},
}


def make_frame_with_marker(marker_id: int, marker_size_px: int = 150) -> np.ndarray:
    """합성 이미지에 ArUco 마커를 그려 반환합니다."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

    frame = np.ones((IMG_H, IMG_W, 3), dtype=np.uint8) * 200
    ox = (IMG_W - marker_size_px) // 2
    oy = (IMG_H - marker_size_px) // 2
    frame[oy:oy + marker_size_px, ox:ox + marker_size_px] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    return frame


@pytest.fixture
def detector():
    return ArucoDetector(
        camera_matrix=K,
        dist_coeffs=DIST,
        aruco_dict_name=DICT_NAME,
        marker_registry=REGISTRY,
    )


def test_classify_known_marker(detector):
    frame = make_frame_with_marker(0)
    results = detector.detect(frame)
    assert len(results) == 1
    assert results[0].marker_id == 0
    assert results[0].label == "강남"
    assert results[0].is_registered is True


def test_classify_unregistered_marker(detector):
    frame = make_frame_with_marker(5)
    results = detector.detect(frame)
    assert len(results) == 1
    assert results[0].marker_id == 5
    assert results[0].is_registered is False
    assert "미등록" in results[0].label


def test_no_marker_empty_frame(detector):
    frame = np.ones((IMG_H, IMG_W, 3), dtype=np.uint8) * 128
    results = detector.detect(frame)
    assert results == []


def test_pose_estimation(detector):
    frame = make_frame_with_marker(0, marker_size_px=200)
    results = detector.detect(frame)
    assert len(results) == 1
    item = results[0]
    assert item.position_xyz is not None, "포즈 추정 실패"
    assert item.position_xyz[2] > 0, "z 값이 양수여야 합니다 (카메라 앞)"


def test_all_labels_classified(detector):
    for marker_id, info in REGISTRY.items():
        frame = make_frame_with_marker(marker_id)
        results = detector.detect(frame)
        assert len(results) == 1
        assert results[0].label == info["label"]
        assert results[0].category == info["category"]


def test_draw_detections_no_crash(detector):
    frame = make_frame_with_marker(1)
    results = detector.detect(frame)
    vis = detector.draw_detections(frame, results)
    assert vis.shape == frame.shape
