"""
iw_hub_movement/move_to_point.py
==================================
오도메트리 피드백 기반 목표 지점 이동 노드.

실행:
    ros2 run iw_hub_movement move_to_point --ros-args \
        -p robot_name:=iw_hub_01 -p target_x:=3.0 -p target_y:=5.0
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from iw_hub_movement.models import Pose2D, normalize_angle, yaw_from_quaternion

_ISAAC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class MoveToPointNode(Node):
    """오도메트리 기반 단일 목표 지점 이동 노드."""

    def __init__(self) -> None:
        super().__init__('move_to_point')

        self.declare_parameter('robot_name', 'iw_hub_01')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('max_linear_speed', 3.5)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('distance_tolerance', 0.12)
        self.declare_parameter('yaw_tolerance', 0.15)

        robot_name  = self.get_parameter('robot_name').value
        target_x    = self.get_parameter('target_x').value
        target_y    = self.get_parameter('target_y').value
        self._max_v = self.get_parameter('max_linear_speed').value
        self._max_w = self.get_parameter('max_angular_speed').value
        self._d_tol = self.get_parameter('distance_tolerance').value
        self._y_tol = self.get_parameter('yaw_tolerance').value

        self._target = (target_x, target_y)
        self._pose: Pose2D | None = None
        self._done = False

        self._cmd_pub    = self.create_publisher(Twist,  f'/{robot_name}/cmd_vel',    10)
        self._status_pub = self.create_publisher(String, f'/{robot_name}/nav_status', 10)
        self.create_subscription(Odometry, f'/{robot_name}/odom', self._on_odom, _ISAAC_QOS)
        self.create_timer(0.1, self._on_timer)

        self.get_logger().info(f'[{robot_name}] MoveToPoint 시작 → target={self._target}')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._pose = Pose2D(
            x=p.x, y=p.y,
            yaw=yaw_from_quaternion(o.x, o.y, o.z, o.w),
        )

    def _on_timer(self) -> None:
        if self._pose is None or self._done:
            return

        dx = self._target[0] - self._pose.x
        dy = self._target[1] - self._pose.y
        dist = math.hypot(dx, dy)

        if dist <= self._d_tol:
            self._publish(0.0, 0.0)
            self._done = True
            self.get_logger().info('목표 도달')
            self._publish_status('complete')
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error  = normalize_angle(target_yaw - self._pose.yaw)
        angular_z  = max(-self._max_w, min(self._max_w, 2.0 * yaw_error))

        if abs(yaw_error) > self._y_tol:
            self._publish(0.0, angular_z)
        else:
            linear_x = min(self._max_v, max(0.08, dist * 0.3))
            self._publish(linear_x, angular_z)

        self._publish_status(
            f'dist={dist:.2f} yaw_err={math.degrees(yaw_error):.1f}°')

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
