"""
ArUco 검출 결과를 ROS2 토픽으로 발행하는 브릿지.

isaac_aruco_main.py가 마커를 인식하면 이 노드가
/aruco/detections 토픽으로 JSON 메시지를 발행합니다.

Isaac Sim의 다른 노드들이 이 토픽을 구독하여 활용할 수 있습니다.
(예: 시각화, 로깅, 커스텀 제어 노드)

발행 메시지 형식 (std_msgs/String, JSON):
{
  "marker_id":    0,
  "role":         "item",
  "label":        "Apple Watch",
  "position_xyz": [0.1, 0.0, 0.5],
  "timestamp":    1716012345.123
}
"""

import json
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))


class ArucoBridge(Node):
    """
    ArUco 검출 결과를 /aruco/detections 토픽으로 발행합니다.
    isaac_aruco_main.py의 on_detected() 콜백에서 직접 호출됩니다.
    """

    def __init__(self):
        super().__init__("aruco_ros_bridge")

        self._pub = self.create_publisher(String, "/aruco/detections", 10)
        self.get_logger().info("[ArucoBridge] 시작 — /aruco/detections 발행 준비")

    def publish_detection(self, marker_id: int, role: str, label: str,
                          position_xyz: tuple | None = None,
                          extra: dict | None = None):
        """
        ArUco 검출 결과를 ROS2 토픽으로 발행합니다.

        Args:
            marker_id    : ArUco 마커 ID
            role         : "item" | "section" | "destination"
            label        : 물품명 또는 구역명
            position_xyz : (x, y, z) 3D 위치 (없으면 None)
            extra        : 추가 정보 dict (target_section, destination 등)
        """
        payload = {
            "marker_id":    marker_id,
            "role":         role,
            "label":        label,
            "position_xyz": list(position_xyz) if position_xyz else None,
            "timestamp":    time.time(),
        }
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(msg)

        self.get_logger().info(
            f"[ArUco] 발행: ID={marker_id}  role={role}  label={label}")
