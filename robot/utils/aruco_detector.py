"""
ArUco marker detection module.
마커 역할(role)에 따라 다르게 처리합니다.

  role = "item"        → 물품 인식  (박스에 부착, ID 0~9)
  role = "section"     → 구획 인식  (선반/바닥에 부착, ID 10~19)
  role = "destination" → 배송지 확인 (목적지에 부착, ID 20~29)
"""

import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

# 역할별 시각화 색상 (BGR)
_ROLE_COLORS = {
    "item":        (0,  255, 100),   # 초록 — 물품
    "section":     (0,  180, 255),   # 주황 — 구획
    "destination": (255, 50, 50),    # 파랑 — 배송지
    "unknown":     (0,    0, 200),   # 빨강 — 미등록
}


@dataclass
class DetectedMarker:
    marker_id:    int
    role:         str                         # "item" | "section" | "destination" | "unknown"
    label:        str
    info:         dict                        # yaml의 원본 dict
    corners:      np.ndarray                  # (4, 2) pixel coords
    center:       tuple[float, float]
    rvec:         Optional[np.ndarray]
    tvec:         Optional[np.ndarray]
    position_xyz: Optional[tuple[float, float, float]] = None
    is_registered: bool = True

    # 역할별 편의 프로퍼티
    @property
    def destination(self) -> str | None:
        return self.info.get("destination")   # item 마커에서 배송지 반환

    @property
    def section_id(self) -> str | None:
        return self.info.get("section_id")    # section/destination 마커에서 구획ID 반환

    @property
    def is_item(self) -> bool:
        return self.role == "item"

    @property
    def is_section(self) -> bool:
        return self.role == "section"

    @property
    def is_destination(self) -> bool:
        return self.role == "destination"


@dataclass
class ArucoDetector:
    camera_matrix:   np.ndarray
    dist_coeffs:     np.ndarray
    marker_size:     float = 0.04
    aruco_dict_name: str   = "DICT_4X4_50"
    marker_registry: dict  = field(default_factory=dict)

    def __post_init__(self):
        dict_id = getattr(cv2.aruco, self.aruco_dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)

    def detect(self, frame: np.ndarray) -> list[DetectedMarker]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)

        results: list[DetectedMarker] = []
        if ids is None:
            return results

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            mid  = int(marker_id)
            info = self.marker_registry.get(mid)
            is_registered = info is not None

            if info is None:
                info = {
                    "role":        "unknown",
                    "label":       f"미등록 (ID:{mid})",
                    "marker_size": self.marker_size,
                }

            m_size = info.get("marker_size", self.marker_size)
            cx, cy = marker_corners[0].mean(axis=0)
            rvec, tvec = self._estimate_pose(marker_corners, m_size)
            pos_xyz = tuple(tvec.flatten().tolist()) if tvec is not None else None

            results.append(DetectedMarker(
                marker_id=mid,
                role=info.get("role", "unknown"),
                label=info["label"],
                info=info,
                corners=marker_corners[0],
                center=(float(cx), float(cy)),
                rvec=rvec,
                tvec=tvec,
                position_xyz=pos_xyz,
                is_registered=is_registered,
            ))
        return results

    def detect_by_role(self, frame: np.ndarray) -> dict[str, list[DetectedMarker]]:
        """역할별로 분류해서 반환. 예: result["item"], result["section"]"""
        all_markers = self.detect(frame)
        result: dict[str, list[DetectedMarker]] = {
            "item": [], "section": [], "destination": [], "unknown": []
        }
        for m in all_markers:
            result.setdefault(m.role, []).append(m)
        return result

    def _estimate_pose(self, corners, marker_size: float):
        half = marker_size / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, corners[0].astype(np.float32),
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        return (rvec, tvec) if ok else (None, None)

    def draw_detections(self, frame: np.ndarray,
                        detections: list[DetectedMarker]) -> np.ndarray:
        vis = frame.copy()
        for m in detections:
            color = _ROLE_COLORS.get(m.role, _ROLE_COLORS["unknown"])
            cv2.polylines(vis, [m.corners.astype(int)], True, color, 3)

            if m.rvec is not None:
                cv2.drawFrameAxes(
                    vis, self.camera_matrix, self.dist_coeffs,
                    m.rvec, m.tvec,
                    m.info.get("marker_size", self.marker_size) * 0.5,
                )

            cx, cy = int(m.center[0]), int(m.center[1])

            # 역할 태그 + 레이블
            role_tag = {"item": "📦", "section": "📍", "destination": "🏁"}.get(m.role, "?")
            label_text = f"{role_tag} {m.label}"
            (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(vis, (cx - tw//2 - 6, cy - th - 12),
                          (cx + tw//2 + 6, cy + 4), color, -1)
            cv2.putText(vis, label_text, (cx - tw//2, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            if m.position_xyz:
                dist_text = f"{m.position_xyz[2]:.3f}m"
                cv2.putText(vis, dist_text, (cx - tw//2, cy + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        return vis
