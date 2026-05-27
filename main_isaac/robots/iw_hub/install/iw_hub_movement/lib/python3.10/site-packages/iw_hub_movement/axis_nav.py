"""
iw_hub_movement/axis_nav.py
============================
X축 → Y축 순서로 정렬 이동하는 노드.
웨이포인트 이름 또는 직접 좌표 지정 가능.

실행 (웨이포인트 이름):
    ros2 run iw_hub_movement axis_nav --ros-args \
        -p robot_name:=iw_hub_01 -p waypoint:=STACK_1

실행 (직접 좌표):
    ros2 run iw_hub_movement axis_nav --ros-args \
        -p robot_name:=iw_hub_01 -p target_x:=-12.0 -p target_y:=7.35
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from iw_hub_movement.models import (
    WAYPOINTS, Pose2D, normalize_angle, yaw_from_quaternion,
)

_ISAAC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_D_TOL = 0.12   # 위치 허용 오차 [m]
_Y_TOL = 0.15   # 방향 허용 오차 [rad]
_MAX_V = 3.5    # 최대 직진 속도 [m/s]
_MAX_W = 0.8    # 최대 회전 속도 [rad/s]


class AxisNavNode(Node):
    """X축 → Y축 순서로 목표 지점까지 이동하는 노드."""

    def __init__(self) -> None:
        super().__init__('axis_nav')

        self.declare_parameter('robot_name', 'iw_hub_01')
        self.declare_parameter('waypoint',   '')       # 웨이포인트 이름 (우선)
        self.declare_parameter('target_x',   0.0)
        self.declare_parameter('target_y',   0.0)
        self.declare_parameter('axis_order', 'xy')     # 'xy' 또는 'yx'

        robot_name  = self.get_parameter('robot_name').value
        waypoint    = self.get_parameter('waypoint').value
        axis_order  = self.get_parameter('axis_order').value

        if waypoint:
            if waypoint not in WAYPOINTS:
                raise ValueError(f"알 수 없는 웨이포인트: '{waypoint}'  "
                                 f"(등록된 목록: {list(WAYPOINTS)})")
            tx, ty = WAYPOINTS[waypoint]
        else:
            tx = self.get_parameter('target_x').value
            ty = self.get_parameter('target_y').value

        self._target     = (tx, ty)
        self._axis_order = axis_order   # 'xy': x 먼저, 'yx': y 먼저
        self._pose: Pose2D | None = None
        self._done = False

        self._cmd_pub    = self.create_publisher(Twist,  f'/{robot_name}/cmd_vel',    10)
        self._status_pub = self.create_publisher(String, f'/{robot_name}/nav_status', 10)
        self.create_subscription(Odometry, f'/{robot_name}/odom', self._on_odom, _ISAAC_QOS)
        self.create_timer(0.1, self._on_timer)

        label = waypoint if waypoint else f'({tx}, {ty})'
        self.get_logger().info(
            f'[{robot_name}] AxisNav 시작 → {label}  axis_order={axis_order}')

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

        dx = abs(self._target[0] - self._pose.x)
        dy = abs(self._target[1] - self._pose.y)

        if dx <= _D_TOL and dy <= _D_TOL:
            self._publish(0.0, 0.0)
            self._done = True
            self.get_logger().info('목표 도달')
            self._publish_status('complete')
            return

        # 축 선택
        if self._axis_order == 'xy':
            active = 'x' if dx > _D_TOL else 'y'
        else:
            active = 'y' if dy > _D_TOL else 'x'

        axis_error = (self._target[0] - self._pose.x) if active == 'x' \
                     else (self._target[1] - self._pose.y)

        if active == 'x':
            target_yaw = 0.0 if axis_error >= 0.0 else math.pi
        else:
            target_yaw = math.pi / 2.0 if axis_error >= 0.0 else -math.pi / 2.0

        yaw_error = normalize_angle(target_yaw - self._pose.yaw)
        angular_z = max(-_MAX_W, min(_MAX_W, 1.5 * yaw_error))

        if abs(yaw_error) > _Y_TOL:
            self._publish(0.0, angular_z)
        else:
            linear_x = min(_MAX_V, max(0.08, abs(axis_error) * 0.3))
            self._publish(linear_x, angular_z)

        self._publish_status(
            f'axis={active} x={self._pose.x:.2f} y={self._pose.y:.2f}')

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
    node = AxisNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
