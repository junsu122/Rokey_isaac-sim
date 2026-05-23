from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import default_cmd_vel_topic, default_odom_topic
from smart_factory.two_robot_reservation_demo import build_corridor_reservation_plan

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    String = object


WorldPoint = tuple[float, float]


@dataclass(frozen=True)
class WaypointSegment:
    robot_1_target: str
    robot_2_target: str
    label: str


def interpolate_corridor_points(start: WorldPoint, end: WorldPoint) -> dict[str, WorldPoint]:
    ax, ay = start
    dx, dy = end
    return {
        "A": start,
        "B": (ax + (dx - ax) / 3.0, ay + (dy - ay) / 3.0),
        "C": (ax + 2.0 * (dx - ax) / 3.0, ay + 2.0 * (dy - ay) / 3.0),
        "D": end,
    }


def build_segments(robot_1_path: list[str], robot_2_path: list[str]) -> list[WaypointSegment]:
    max_steps = max(len(robot_1_path), len(robot_2_path)) - 1
    segments = []
    for index in range(max_steps):
        robot_1_target = _path_at(robot_1_path, index + 1)
        robot_2_target = _path_at(robot_2_path, index + 1)
        segments.append(
            WaypointSegment(
                robot_1_target=robot_1_target,
                robot_2_target=robot_2_target,
                label=f"step {index}: r1->{robot_1_target}, r2->{robot_2_target}",
            )
        )
    return segments


def should_move(path: list[str], index: int) -> bool:
    return _path_at(path, index) != _path_at(path, index + 1)


def distance_between(left: Pose2D, right: Pose2D) -> float:
    return math.hypot(left.x - right.x, left.y - right.y)


def compute_straight_command(
    start_pose: Pose2D,
    current_pose: Pose2D,
    *,
    should_robot_move: bool,
    step_distance: float,
    speed: float,
    distance_tolerance: float,
) -> tuple[float, bool]:
    if not should_robot_move:
        return 0.0, True

    traveled = distance_between(start_pose, current_pose)
    if traveled >= max(0.0, step_distance - distance_tolerance):
        return 0.0, True
    return speed, False


def should_safety_stop(robot_1_pose: Pose2D, robot_2_pose: Pose2D, min_safe_distance: float) -> bool:
    return min_safe_distance > 0.0 and distance_between(robot_1_pose, robot_2_pose) < min_safe_distance


def is_segment_timed_out(segment_started_at: float | None, max_step_duration: float) -> bool:
    if segment_started_at is None or max_step_duration <= 0.0:
        return False
    return time.monotonic() - segment_started_at >= max_step_duration


def compute_drive_command(
    pose: Pose2D,
    target: WorldPoint,
    *,
    max_linear_speed: float,
    max_angular_speed: float,
    distance_tolerance: float,
    yaw_tolerance: float,
) -> tuple[float, float, bool]:
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    distance = math.hypot(dx, dy)
    if distance <= distance_tolerance:
        return 0.0, 0.0, True

    target_yaw = math.atan2(dy, dx)
    yaw_error = normalize_angle(target_yaw - pose.yaw)
    angular_z = _clamp(1.5 * yaw_error, -max_angular_speed, max_angular_speed)

    if abs(yaw_error) > yaw_tolerance:
        return 0.0, angular_z, False

    linear_x = min(max_linear_speed, max(0.08, distance * 0.5))
    return linear_x, angular_z, False


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _path_at(path: list[str], index: int) -> str:
    if index >= len(path):
        return path[-1]
    return path[index]


class TwoRobotReservationFollower(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_two_robot_reservation_follower")
        self.args = args
        self.robot_1_pose: Pose2D | None = None
        self.robot_2_pose: Pose2D | None = None
        self.robot_1_segment_start: Pose2D | None = None
        self.robot_2_segment_start: Pose2D | None = None
        self.segment_started_at: float | None = None
        self.waypoints: dict[str, WorldPoint] | None = None
        self.robot_1_path, self.robot_2_path = build_corridor_reservation_plan()
        self.segments = build_segments(self.robot_1_path, self.robot_2_path)
        self.segment_index = 0

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.robot_1_pub = self.create_publisher(Twist, args.robot_1_cmd_vel, 10)
        self.robot_2_pub = self.create_publisher(Twist, args.robot_2_cmd_vel, 10)
        self.status_pub = self.create_publisher(String, "/smart_factory/reservation_follower", 10)
        self._odom_subscriptions = [
            self.create_subscription(Odometry, args.robot_1_odom, self._on_robot_1_odom, isaac_qos),
            self.create_subscription(Odometry, args.robot_2_odom, self._on_robot_2_odom, isaac_qos),
        ]
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"Following reserved route from {args.robot_1_odom}, {args.robot_2_odom}"
        )

    def _on_robot_1_odom(self, msg: Odometry) -> None:
        self.robot_1_pose = _pose_from_odom(msg)

    def _on_robot_2_odom(self, msg: Odometry) -> None:
        self.robot_2_pose = _pose_from_odom(msg)

    def _on_timer(self) -> None:
        if self.robot_1_pose is None or self.robot_2_pose is None:
            self._publish_status("waiting for both robot odom messages")
            return

        if self.args.straight_only:
            self._on_straight_timer()
            return

        if self.waypoints is None:
            self.waypoints = self._make_waypoints()
            self.get_logger().info(
                "World waypoints: "
                + ", ".join(
                    f"{name}=({point[0]:.3f},{point[1]:.3f})"
                    for name, point in sorted(self.waypoints.items())
                )
            )

        if self.segment_index >= len(self.segments):
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
            self._publish_status("complete")
            return

        segment = self.segments[self.segment_index]
        robot_1_linear, robot_1_angular, robot_1_done = compute_drive_command(
            self.robot_1_pose,
            self.waypoints[segment.robot_1_target],
            max_linear_speed=self.args.speed,
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self.args.distance_tolerance,
            yaw_tolerance=self.args.yaw_tolerance,
        )
        robot_2_linear, robot_2_angular, robot_2_done = compute_drive_command(
            self.robot_2_pose,
            self.waypoints[segment.robot_2_target],
            max_linear_speed=self.args.speed,
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self.args.distance_tolerance,
            yaw_tolerance=self.args.yaw_tolerance,
        )

        self._publish_twists(robot_1_linear, robot_1_angular, robot_2_linear, robot_2_angular)
        self._publish_status(
            f"{segment.label}; r1_done={robot_1_done}; r2_done={robot_2_done}"
        )

        if robot_1_done and robot_2_done:
            self.segment_index += 1
            time.sleep(self.args.segment_pause)

    def _on_straight_timer(self) -> None:
        if self.segment_index >= len(self.segments):
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
            self._publish_status("complete")
            return

        if self.robot_1_segment_start is None or self.robot_2_segment_start is None:
            self.robot_1_segment_start = self.robot_1_pose
            self.robot_2_segment_start = self.robot_2_pose
            self.segment_started_at = time.monotonic()

        segment = self.segments[self.segment_index]
        robot_1_linear, robot_1_done = compute_straight_command(
            self.robot_1_segment_start,
            self.robot_1_pose,
            should_robot_move=should_move(self.robot_1_path, self.segment_index),
            step_distance=self.args.step_distance,
            speed=self.args.speed,
            distance_tolerance=self.args.distance_tolerance,
        )
        robot_2_linear, robot_2_done = compute_straight_command(
            self.robot_2_segment_start,
            self.robot_2_pose,
            should_robot_move=should_move(self.robot_2_path, self.segment_index),
            step_distance=self.args.step_distance,
            speed=self.args.speed,
            distance_tolerance=self.args.distance_tolerance,
        )
        robot_1_traveled = distance_between(self.robot_1_segment_start, self.robot_1_pose)
        robot_2_traveled = distance_between(self.robot_2_segment_start, self.robot_2_pose)
        robot_distance = distance_between(self.robot_1_pose, self.robot_2_pose)
        safety_stop = should_safety_stop(
            self.robot_1_pose,
            self.robot_2_pose,
            self.args.min_safe_distance,
        )
        timed_out = is_segment_timed_out(self.segment_started_at, self.args.max_step_duration)
        if safety_stop:
            robot_1_linear = 0.0
            robot_2_linear = 0.0
        elif timed_out:
            robot_1_linear = 0.0
            robot_2_linear = 0.0
            robot_1_done = True
            robot_2_done = True

        self._publish_twists(robot_1_linear, 0.0, robot_2_linear, 0.0)
        self._publish_status(
            f"{segment.label}; straight_only=true; "
            f"r1_traveled={robot_1_traveled:.3f}; r2_traveled={robot_2_traveled:.3f}; "
            f"robot_distance={robot_distance:.3f}; safety_stop={safety_stop}; "
            f"timed_out={timed_out}; r1_done={robot_1_done}; r2_done={robot_2_done}"
        )

        if robot_1_done and robot_2_done:
            self.segment_index += 1
            self.robot_1_segment_start = None
            self.robot_2_segment_start = None
            self.segment_started_at = None
            time.sleep(self.args.segment_pause)

    def _make_waypoints(self) -> dict[str, WorldPoint]:
        if self.args.a_x is not None and self.args.a_y is not None:
            start = (self.args.a_x, self.args.a_y)
        else:
            start = (self.robot_1_pose.x, self.robot_1_pose.y)

        if self.args.d_x is not None and self.args.d_y is not None:
            end = (self.args.d_x, self.args.d_y)
        else:
            end = (self.robot_2_pose.x, self.robot_2_pose.y)

        return interpolate_corridor_points(start, end)

    def _publish_twists(
        self,
        robot_1_linear: float,
        robot_1_angular: float,
        robot_2_linear: float,
        robot_2_angular: float,
    ) -> None:
        robot_1_command = Twist()
        robot_1_command.linear.x = robot_1_linear
        robot_1_command.angular.z = robot_1_angular
        self.robot_1_pub.publish(robot_1_command)

        robot_2_command = Twist()
        robot_2_command.linear.x = robot_2_linear
        robot_2_command.angular.z = robot_2_angular
        self.robot_2_pub.publish(robot_2_command)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def _pose_from_odom(msg: Odometry) -> Pose2D:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return Pose2D(
        x=position.x,
        y=position.y,
        yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Follow a two-robot reserved corridor route using odom feedback."
    )
    parser.add_argument("--robot-1-odom", default=default_odom_topic(1))
    parser.add_argument("--robot-2-odom", default=default_odom_topic(2))
    parser.add_argument("--robot-1-cmd-vel", default=default_cmd_vel_topic(1))
    parser.add_argument("--robot-2-cmd-vel", default=default_cmd_vel_topic(2))
    parser.add_argument("--a-x", type=float, help="World x for waypoint A. Defaults to robot 1 start x.")
    parser.add_argument("--a-y", type=float, help="World y for waypoint A. Defaults to robot 1 start y.")
    parser.add_argument("--d-x", type=float, help="World x for waypoint D. Defaults to robot 2 start x.")
    parser.add_argument("--d-y", type=float, help="World y for waypoint D. Defaults to robot 2 start y.")
    parser.add_argument("--speed", type=float, default=0.18, help="Max forward speed in m/s.")
    parser.add_argument("--turn-speed", type=float, default=0.6, help="Max turn speed in rad/s.")
    parser.add_argument(
        "--straight-only",
        action="store_true",
        help="Ignore waypoint heading and run the reservation plan as straight-line forward/wait steps.",
    )
    parser.add_argument(
        "--step-distance",
        type=float,
        default=0.6,
        help="Distance in meters for one reserved straight-line step.",
    )
    parser.add_argument(
        "--max-step-duration",
        type=float,
        default=5.0,
        help="Safety timeout in seconds for one straight-line reservation step. Use 0 to disable.",
    )
    parser.add_argument(
        "--min-safe-distance",
        type=float,
        default=0.8,
        help="Stop both robots if their odom positions are closer than this distance. Use 0 to disable.",
    )
    parser.add_argument("--distance-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument("--segment-pause", type=float, default=0.2)
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running this follower.")

    parsed_args = _parse_args()
    rclpy.init(args=args)
    node = TwoRobotReservationFollower(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_twists(0.0, 0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
