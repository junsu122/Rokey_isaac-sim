from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Optional

from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import default_cmd_vel_topic, default_odom_topic
from smart_factory.two_robot_reservation_follower import normalize_angle

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

PLACE_CANDIDATES: dict[str, list[WorldPoint]] = {
    # Warehouse coordinates from robot_config.py (POD_STACKS) and
    # iw_hub_movement/models.py (WAIT, UNLOAD). robot_config.py is the
    # single source of truth — update both files together when positions change.
    "WAIT": [(-8.0, -14.0), (-9.0, -14.0), (-10.0, -14.0)],      # WAIT_1(hub01) / _2(예비) / _3(hub02)
    "STACK": [(-12.8, 9.0), (-8.2, 1.5), (-9.7, -8.9)],           # PodStack_01 / _02 / _03
    "SHELF_STORAGE": [(0.0, -10.0), (0.0, 10.0), (0.0, 0.0)],    # 비활성화 — 창고에 없음
    "UNLOAD": [(4.0, -13.0), (4.0, -3.0), (4.0, 7.0)],            # UNLOAD_1 / _2 / _3
}

PLACES: dict[str, WorldPoint] = {
    name if len(points) == 1 else f"{name}_{index}": point
    for name, points in PLACE_CANDIDATES.items()
    for index, point in enumerate(points, start=1)
}


@dataclass(frozen=True)
class AxisRoute:
    target_name: str
    waypoints: list[WorldPoint]
    axes: list[str]


@dataclass(frozen=True)
class AxisNavCommand:
    linear_x: float
    angular_z: float
    done: bool
    distance: float
    target_yaw: float
    control_yaw: float
    yaw_error: float
    axis_error: float
    crossed_axis_target: bool


def build_axis_route(
    start: WorldPoint,
    target_name: str,
    *,
    axis_order: str = "xy",
) -> AxisRoute:
    normalized_target = target_name.upper()
    target, resolved_target_name = _resolve_place(start, normalized_target)
    if target is None:
        available = ", ".join(sorted((*PLACE_CANDIDATES, *PLACES)))
        raise ValueError(f"Unknown target {target_name!r}. Available targets: {available}")
    if axis_order not in {"xy", "yx"}:
        raise ValueError("axis_order must be 'xy' or 'yx'")

    if axis_order == "xy":
        steps = [((target[0], start[1]), "x"), (target, "y")]
    else:
        steps = [((start[0], target[1]), "y"), (target, "x")]
    waypoints, axes = _deduplicate_steps(start, steps)
    return AxisRoute(target_name=resolved_target_name, waypoints=waypoints, axes=axes)


def _resolve_place(start: WorldPoint, target_name: str) -> tuple[WorldPoint | None, str]:
    if target_name in PLACES:
        return PLACES[target_name], target_name
    if target_name not in PLACE_CANDIDATES:
        return None, target_name

    points = PLACE_CANDIDATES[target_name]
    nearest_index, nearest_point = min(
        enumerate(points, start=1),
        key=lambda item: math.hypot(item[1][0] - start[0], item[1][1] - start[1]),
    )
    if len(points) == 1:
        return nearest_point, target_name
    return nearest_point, f"{target_name}_{nearest_index}"


def _deduplicate_steps(
    start: WorldPoint,
    steps: list[tuple[WorldPoint, str]],
) -> tuple[list[WorldPoint], list[str]]:
    points = []
    axes = []
    previous = start
    for point, axis in steps:
        if not _same_point(previous, point):
            points.append(point)
            axes.append(axis)
            previous = point
    return points, axes


def _same_point(left: WorldPoint, right: WorldPoint, tolerance: float = 1e-3) -> bool:
    return math.isclose(left[0], right[0], abs_tol=tolerance) and math.isclose(
        left[1], right[1], abs_tol=tolerance
    )


def compute_axis_nav_command(
    pose: Pose2D,
    target: WorldPoint,
    *,
    segment_start: WorldPoint,
    active_axis: str,
    max_linear_speed: float,
    max_angular_speed: float,
    distance_tolerance: float,
    yaw_tolerance: float,
    yaw_offset: float = 0.0,
    angular_sign: float = 1.0,
    allow_crossed_axis_target: bool = True,
    axis_aligned_heading: bool = True,
    reverse_motion: bool = False,
) -> AxisNavCommand:
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    distance = math.hypot(dx, dy)
    axis_error = _axis_error(pose, target, active_axis)
    crossed_axis_target = _crossed_axis_target(segment_start, pose, target, active_axis)
    if abs(axis_error) <= distance_tolerance or (
        allow_crossed_axis_target and crossed_axis_target
    ):
        return AxisNavCommand(
            linear_x=0.0,
            angular_z=0.0,
            done=True,
            distance=distance,
            target_yaw=pose.yaw,
            control_yaw=pose.yaw,
            yaw_error=0.0,
            axis_error=axis_error,
            crossed_axis_target=crossed_axis_target,
        )

    target_yaw = (
        _axis_target_yaw(axis_error, active_axis)
        if axis_aligned_heading
        else math.atan2(dy, dx)
    )
    if reverse_motion:
        target_yaw = normalize_angle(target_yaw + math.pi)
    control_yaw = normalize_angle(pose.yaw + yaw_offset)
    yaw_error = normalize_angle(target_yaw - control_yaw)
    angular_z = _clamp(1.5 * yaw_error * angular_sign, -max_angular_speed, max_angular_speed)

    if abs(yaw_error) > yaw_tolerance:
        return AxisNavCommand(
            linear_x=0.0,
            angular_z=angular_z,
            done=False,
            distance=distance,
            target_yaw=target_yaw,
            control_yaw=control_yaw,
            yaw_error=yaw_error,
            axis_error=axis_error,
            crossed_axis_target=crossed_axis_target,
        )

    speed_error = abs(axis_error) if axis_aligned_heading else distance
    linear_x = min(max_linear_speed, max(0.08, speed_error * 0.2))
    if reverse_motion:
        linear_x = -linear_x
    return AxisNavCommand(
        linear_x=linear_x,
        angular_z=angular_z,
        done=False,
        distance=distance,
        target_yaw=target_yaw,
        control_yaw=control_yaw,
        yaw_error=yaw_error,
        axis_error=axis_error,
        crossed_axis_target=crossed_axis_target,
    )


def _axis_error(pose: Pose2D, target: WorldPoint, active_axis: str) -> float:
    if active_axis == "x":
        return target[0] - pose.x
    return target[1] - pose.y


def _axis_target_yaw(axis_error: float, active_axis: str) -> float:
    if active_axis == "x":
        return 0.0 if axis_error >= 0.0 else math.pi
    return math.pi / 2.0 if axis_error >= 0.0 else -math.pi / 2.0


def _crossed_axis_target(
    segment_start: WorldPoint,
    pose: Pose2D,
    target: WorldPoint,
    active_axis: str,
) -> bool:
    if active_axis == "x":
        start_value = segment_start[0]
        current_value = pose.x
        target_value = target[0]
    else:
        start_value = segment_start[1]
        current_value = pose.y
        target_value = target[1]

    direction = target_value - start_value
    if math.isclose(direction, 0.0, abs_tol=1e-6):
        return True
    return (current_value - target_value) * direction > 0.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class AxisNavToPlace(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_axis_nav_to_place")
        self.args = args
        self.pose: Pose2D | None = None
        self.segment_start_pose: Pose2D | None = None
        self.last_progress_pose: Pose2D | None = None
        self.last_progress_time: float | None = None
        self.route: AxisRoute | None = None
        self.waypoint_index = 0

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            args.odom_topic,
            self._on_odom,
            isaac_qos,
        )
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"Axis nav waiting for {args.odom_topic}; target={args.target.upper()}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = _pose_from_odom(msg)

    def _on_timer(self) -> None:
        if self.pose is None:
            self._publish_status("waiting for odom")
            return

        if self.route is None:
            self.route = build_axis_route(
                (self.pose.x, self.pose.y),
                self.args.target,
                axis_order=self.args.axis_order,
            )
            self.get_logger().info(
                f"Route to {self.route.target_name}: "
                + " -> ".join(f"({x:.3f},{y:.3f})" for x, y in self.route.waypoints)
            )

        if self.waypoint_index >= len(self.route.waypoints):
            self._publish_command(0.0, 0.0)
            self._publish_status(f"complete target={self.route.target_name}")
            return

        if self.segment_start_pose is None:
            self.segment_start_pose = self.pose
            self.last_progress_pose = self.pose
            self.last_progress_time = self.get_clock().now().nanoseconds / 1e9

        target = self.route.waypoints[self.waypoint_index]
        active_axis = self.route.axes[self.waypoint_index]
        command = compute_axis_nav_command(
            self.pose,
            target,
            segment_start=(self.segment_start_pose.x, self.segment_start_pose.y),
            active_axis=active_axis,
            max_linear_speed=self.args.speed,
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self.args.distance_tolerance,
            yaw_tolerance=self.args.yaw_tolerance,
            yaw_offset=self.args.yaw_offset,
            angular_sign=self.args.angular_sign,
        )
        # Stall detection is intentionally disabled during Isaac experiments.
        # The robot may pause briefly while physics settles or while another
        # node publishes, and hard-stopping here made navigation look stuck.
        stalled = False
        self._publish_command(command.linear_x, command.angular_z)
        self._publish_status(
            f"target={self.route.target_name}; waypoint={self.waypoint_index + 1}/"
            f"{len(self.route.waypoints)}; x={self.pose.x:.3f}; y={self.pose.y:.3f}; "
            f"yaw={self.pose.yaw:.3f}; axis={active_axis}; goal=({target[0]:.3f},{target[1]:.3f}); "
            f"distance={command.distance:.3f}; target_yaw={command.target_yaw:.3f}; "
            f"control_yaw={command.control_yaw:.3f}; yaw_error={command.yaw_error:.3f}; "
            f"axis_error={command.axis_error:.3f}; crossed={command.crossed_axis_target}; "
            f"linear={command.linear_x:.3f}; angular={command.angular_z:.3f}; "
            f"stalled={stalled}; done={command.done}"
        )

        if command.done:
            self.waypoint_index += 1
            self.segment_start_pose = None
            self.last_progress_pose = None
            self.last_progress_time = None

    def _is_stalled(self, command: AxisNavCommand) -> bool:
        if command.done or command.linear_x <= 0.0 or self.args.stall_timeout <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_progress_pose is None or self.last_progress_time is None:
            self.last_progress_pose = self.pose
            self.last_progress_time = now
            return False

        moved = math.hypot(self.pose.x - self.last_progress_pose.x, self.pose.y - self.last_progress_pose.y)
        if moved >= self.args.stall_distance:
            self.last_progress_pose = self.pose
            self.last_progress_time = now
            return False
        return now - self.last_progress_time >= self.args.stall_timeout

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_pub.publish(command)

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
        description="Navigate to one named place using only x/y-axis-aligned route segments."
    )
    parser.add_argument(
        "--target",
        choices=sorted((*PLACE_CANDIDATES, *PLACES)),
        required=True,
        help=f"Target place: {', '.join(sorted((*PLACE_CANDIDATES, *PLACES)))}.",
    )
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument("--odom-topic", default=default_odom_topic(1))
    parser.add_argument("--cmd-vel-topic", default=default_cmd_vel_topic(1))
    parser.add_argument("--status-topic", default="/smart_factory/axis_nav_status")
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--turn-speed", type=float, default=2.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument("--stall-timeout", type=float, default=0.0)
    parser.add_argument("--stall-distance", type=float, default=0.03)
    parser.add_argument(
        "--yaw-offset",
        type=float,
        default=0.0,
        help="Offset added to odom yaw when the robot forward axis differs from odom yaw.",
    )
    parser.add_argument(
        "--angular-sign",
        type=float,
        choices=[-1.0, 1.0],
        default=1.0,
        help="Use -1 if positive angular.z turns the robot opposite of odom yaw.",
    )
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running axis_nav_to_place.")

    parsed_args = _parse_args()
    rclpy.init(args=args)
    node = AxisNavToPlace(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_command(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
