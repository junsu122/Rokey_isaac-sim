from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Optional

from smart_factory.axis_nav_to_place import (
    AxisRoute,
    PLACES,
    build_axis_route,
    compute_axis_nav_command,
)
from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import (
    default_base_frame,
    default_cmd_vel_topic,
    default_odom_topic,
    default_robot_id,
)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    String = object
    TFMessage = object


@dataclass
class RobotAxisState:
    robot_id: str
    target_name: str
    pose: Pose2D | None = None
    route: AxisRoute | None = None
    waypoint_index: int = 0
    segment_start_pose: Pose2D | None = None
    completed: bool = False
    pose_source: str | None = None
    avoidance_replans: int = 0

    @property
    def active_axis(self) -> str | None:
        if self.route is None or self.waypoint_index >= len(self.route.axes):
            return None
        return self.route.axes[self.waypoint_index]

    @property
    def is_on_reserved_lane(self) -> bool:
        return self.active_axis == "y"


@dataclass(frozen=True)
class ReservationDecision:
    robot_1_allowed: bool
    robot_2_allowed: bool
    reason: str


@dataclass(frozen=True)
class HeadOnConflict:
    axis: str
    distance: float
    robot_1_direction: float
    robot_2_direction: float


def decide_reservations(robot_1: RobotAxisState, robot_2: RobotAxisState) -> ReservationDecision:
    if robot_1.completed and robot_2.completed:
        return ReservationDecision(False, False, "complete")

    if robot_1.target_name == robot_2.target_name:
        if not robot_1.completed:
            return ReservationDecision(True, False, f"{robot_2.robot_id} waits: {robot_1.target_name} reserved")
        return ReservationDecision(False, False, f"{robot_2.target_name} occupied by {robot_1.robot_id}")

    if robot_1.is_on_reserved_lane and robot_2.is_on_reserved_lane:
        if not robot_1.completed:
            return ReservationDecision(True, False, f"{robot_2.robot_id} waits: x=3 lane reserved")
        return ReservationDecision(False, True, f"{robot_1.robot_id} completed lane")

    return ReservationDecision(not robot_1.completed, not robot_2.completed, "free")


def distance_between(left: Pose2D, right: Pose2D) -> float:
    return math.hypot(left.x - right.x, left.y - right.y)


def should_safety_stop(robot_1: RobotAxisState, robot_2: RobotAxisState, min_safe_distance: float) -> bool:
    if min_safe_distance <= 0.0 or robot_1.pose is None or robot_2.pose is None:
        return False
    return distance_between(robot_1.pose, robot_2.pose) < min_safe_distance


def detect_head_on_conflict(
    robot_1: RobotAxisState,
    robot_2: RobotAxisState,
    *,
    lane_tolerance: float,
    trigger_distance: float,
) -> HeadOnConflict | None:
    if (
        robot_1.pose is None
        or robot_2.pose is None
        or robot_1.route is None
        or robot_2.route is None
        or robot_1.completed
        or robot_2.completed
    ):
        return None

    axis = robot_1.active_axis
    if axis is None or axis != robot_2.active_axis:
        return None

    target_1 = robot_1.route.waypoints[robot_1.waypoint_index]
    target_2 = robot_2.route.waypoints[robot_2.waypoint_index]
    direction_1 = _direction_to_target(robot_1.pose, target_1, axis)
    direction_2 = _direction_to_target(robot_2.pose, target_2, axis)
    if direction_1 == 0.0 or direction_2 == 0.0 or direction_1 == direction_2:
        return None

    if _lateral_distance(robot_1.pose, robot_2.pose, axis) > lane_tolerance:
        return None
    if _lateral_distance_to_targets(target_1, target_2, axis) > lane_tolerance:
        return None

    distance = distance_between(robot_1.pose, robot_2.pose)
    if trigger_distance > 0.0 and distance > trigger_distance:
        return None

    robot_2_ahead_of_robot_1 = (
        _axis_value(robot_2.pose, axis) - _axis_value(robot_1.pose, axis)
    ) * direction_1 > 0.0
    robot_1_ahead_of_robot_2 = (
        _axis_value(robot_1.pose, axis) - _axis_value(robot_2.pose, axis)
    ) * direction_2 > 0.0
    if not robot_2_ahead_of_robot_1 or not robot_1_ahead_of_robot_2:
        return None

    if not _axis_ranges_overlap(
        (_axis_value(robot_1.pose, axis), _axis_point_value(target_1, axis)),
        (_axis_value(robot_2.pose, axis), _axis_point_value(target_2, axis)),
        lane_tolerance,
    ):
        return None

    return HeadOnConflict(axis, distance, direction_1, direction_2)


def build_left_bypass_route(
    robot: RobotAxisState,
    other: RobotAxisState,
    *,
    lateral_offset: float,
    pass_distance: float,
) -> AxisRoute | None:
    if robot.pose is None or other.pose is None or robot.route is None:
        return None
    axis = robot.active_axis
    if axis is None or robot.waypoint_index >= len(robot.route.waypoints):
        return None

    original_target = robot.route.waypoints[robot.waypoint_index]
    direction = _direction_to_target(robot.pose, original_target, axis)
    if direction == 0.0:
        return None

    offset = _left_offset(axis, direction, lateral_offset)
    pass_axis_value = _axis_value(other.pose, axis) + direction * pass_distance
    side_point = _offset_point_on_current_axis(robot.pose, original_target, axis, offset)
    pass_side_point = _replace_axis_value(side_point, axis, pass_axis_value)
    pass_lane_point = _replace_lateral_value(pass_side_point, axis, _lateral_point_value(original_target, axis))

    old_points = robot.route.waypoints[robot.waypoint_index:]
    old_axes = robot.route.axes[robot.waypoint_index:]
    waypoints, axes = _deduplicate_route_steps(
        (robot.pose.x, robot.pose.y),
        [
            (side_point, _perpendicular_axis(axis)),
            (pass_side_point, axis),
            (pass_lane_point, _perpendicular_axis(axis)),
            *zip(old_points, old_axes),
        ],
    )
    return AxisRoute(target_name=robot.route.target_name, waypoints=waypoints, axes=axes)


def _axis_value(pose: Pose2D, axis: str) -> float:
    return pose.x if axis == "x" else pose.y


def _axis_point_value(point: tuple[float, float], axis: str) -> float:
    return point[0] if axis == "x" else point[1]


def _lateral_point_value(point: tuple[float, float], axis: str) -> float:
    return point[1] if axis == "x" else point[0]


def _lateral_distance(left: Pose2D, right: Pose2D, axis: str) -> float:
    if axis == "x":
        return abs(left.y - right.y)
    return abs(left.x - right.x)


def _lateral_distance_to_targets(
    left: tuple[float, float],
    right: tuple[float, float],
    axis: str,
) -> float:
    return abs(_lateral_point_value(left, axis) - _lateral_point_value(right, axis))


def _direction_to_target(pose: Pose2D, target: tuple[float, float], axis: str) -> float:
    delta = _axis_point_value(target, axis) - _axis_value(pose, axis)
    if math.isclose(delta, 0.0, abs_tol=1e-6):
        return 0.0
    return math.copysign(1.0, delta)


def _axis_ranges_overlap(
    left: tuple[float, float],
    right: tuple[float, float],
    tolerance: float,
) -> bool:
    left_min, left_max = sorted(left)
    right_min, right_max = sorted(right)
    return max(left_min, right_min) <= min(left_max, right_max) + tolerance


def _perpendicular_axis(axis: str) -> str:
    return "y" if axis == "x" else "x"


def _left_offset(axis: str, direction: float, lateral_offset: float) -> float:
    if axis == "x":
        return math.copysign(abs(lateral_offset), direction)
    return math.copysign(abs(lateral_offset), -direction)


def _offset_point_on_current_axis(
    pose: Pose2D,
    original_target: tuple[float, float],
    axis: str,
    offset: float,
) -> tuple[float, float]:
    if axis == "x":
        return (pose.x, original_target[1] + offset)
    return (original_target[0] + offset, pose.y)


def _replace_axis_value(
    point: tuple[float, float],
    axis: str,
    axis_value: float,
) -> tuple[float, float]:
    if axis == "x":
        return (axis_value, point[1])
    return (point[0], axis_value)


def _replace_lateral_value(
    point: tuple[float, float],
    axis: str,
    lateral_value: float,
) -> tuple[float, float]:
    if axis == "x":
        return (point[0], lateral_value)
    return (lateral_value, point[1])


def _deduplicate_route_steps(
    start: tuple[float, float],
    steps,
) -> tuple[list[tuple[float, float]], list[str]]:
    waypoints = []
    axes = []
    previous = start
    for point, axis in steps:
        if not (
            math.isclose(previous[0], point[0], abs_tol=1e-3)
            and math.isclose(previous[1], point[1], abs_tol=1e-3)
        ):
            waypoints.append(point)
            axes.append(axis)
            previous = point
    return waypoints, axes


class ReservedAxisNav(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_reserved_axis_nav")
        self.args = args
        self.robot_1 = RobotAxisState(args.robot_1_id, args.robot_1_target.upper())
        self.robot_2 = RobotAxisState(args.robot_2_id, args.robot_2_target.upper())
        self.last_wait_log_time: float | None = None

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.robot_1_pub = self.create_publisher(Twist, args.robot_1_cmd_vel, 10)
        self.robot_2_pub = self.create_publisher(Twist, args.robot_2_cmd_vel, 10)
        self.status_pub = self.create_publisher(String, "/smart_factory/reserved_axis_nav_status", 10)
        self._subscriptions = [
            self.create_subscription(Odometry, args.robot_1_odom, self._on_robot_1_odom, isaac_qos),
            self.create_subscription(Odometry, args.robot_2_odom, self._on_robot_2_odom, isaac_qos),
            self.create_subscription(TFMessage, args.robot_1_tf, self._on_robot_1_tf, isaac_qos),
            self.create_subscription(TFMessage, args.robot_2_tf, self._on_robot_2_tf, isaac_qos),
        ]
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"Reserved axis nav: {self.robot_1.robot_id}->{self.robot_1.target_name}, "
            f"{self.robot_2.robot_id}->{self.robot_2.target_name}; "
            f"pose_source={args.pose_source}; "
            f"tf=({args.robot_1_tf}, {args.robot_2_tf}); "
            f"odom=({args.robot_1_odom}, {args.robot_2_odom})"
        )

    def _on_robot_1_odom(self, msg: Odometry) -> None:
        if self.args.pose_source in {"odom", "auto"} and self.robot_1.pose_source != "tf":
            self.robot_1.pose = _pose_from_odom(msg)
            self.robot_1.pose_source = "odom"

    def _on_robot_2_odom(self, msg: Odometry) -> None:
        if self.args.pose_source in {"odom", "auto"} and self.robot_2.pose_source != "tf":
            self.robot_2.pose = _pose_from_odom(msg)
            self.robot_2.pose_source = "odom"

    def _on_robot_1_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(self.robot_1, msg, self.args.robot_1_base_frame, self.args.robot_1_tf)

    def _on_robot_2_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(self.robot_2, msg, self.args.robot_2_base_frame, self.args.robot_2_tf)

    def _on_robot_tf(
        self,
        robot: RobotAxisState,
        msg: TFMessage,
        base_frame: str,
        tf_topic: str,
    ) -> None:
        if self.args.pose_source not in {"tf", "auto"}:
            return
        allow_unqualified_frame = tf_topic != "/tf"
        for transform in msg.transforms:
            if _frame_matches_robot(
                transform.child_frame_id,
                robot.robot_id,
                base_frame,
                allow_unqualified_frame=allow_unqualified_frame,
            ):
                robot.pose = _pose_from_transform(transform)
                robot.pose_source = "tf"
                return

    def _on_timer(self) -> None:
        if self.robot_1.pose is None or self.robot_2.pose is None:
            self._publish_waiting_for_poses()
            return

        self._ensure_routes()
        avoidance_text = self._maybe_apply_head_on_avoidance()
        if should_safety_stop(self.robot_1, self.robot_2, self.args.min_safe_distance):
            robot_distance = distance_between(self.robot_1.pose, self.robot_2.pose)
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
            self._publish_status(
                f"safety_stop robot_distance={robot_distance:.3f} "
                f"min_safe_distance={self.args.min_safe_distance:.3f}"
            )
            return

        decision = decide_reservations(self.robot_1, self.robot_2)

        robot_1_linear, robot_1_angular, robot_1_text = self._step_robot(self.robot_1, decision.robot_1_allowed)
        robot_2_linear, robot_2_angular, robot_2_text = self._step_robot(self.robot_2, decision.robot_2_allowed)
        self._publish_twists(robot_1_linear, robot_1_angular, robot_2_linear, robot_2_angular)
        avoidance_prefix = f"{avoidance_text}; " if avoidance_text else ""
        self._publish_status(f"{avoidance_prefix}reservation={decision.reason}; {robot_1_text}; {robot_2_text}")

    def _ensure_routes(self) -> None:
        for robot in (self.robot_1, self.robot_2):
            if robot.route is None and robot.pose is not None:
                robot.route = build_axis_route(
                    (robot.pose.x, robot.pose.y),
                    robot.target_name,
                    axis_order=self.args.axis_order,
                )
                self.get_logger().info(
                    f"{robot.robot_id} route to {robot.target_name}: "
                    + " -> ".join(f"({x:.3f},{y:.3f})" for x, y in robot.route.waypoints)
                )

    def _maybe_apply_head_on_avoidance(self) -> str:
        if not self.args.enable_head_on_avoidance:
            return ""
        conflict = detect_head_on_conflict(
            self.robot_1,
            self.robot_2,
            lane_tolerance=self.args.head_on_lane_tolerance,
            trigger_distance=self.args.head_on_trigger_distance,
        )
        if conflict is None:
            return ""

        evader = self.robot_2 if self.args.avoidance_evader == "robot_2" else self.robot_1
        other = self.robot_1 if evader is self.robot_2 else self.robot_2
        if evader.avoidance_replans >= self.args.max_avoidance_replans:
            return (
                f"head_on_conflict axis={conflict.axis} distance={conflict.distance:.3f}; "
                f"{evader.robot_id}=avoidance_limit"
            )

        route = build_left_bypass_route(
            evader,
            other,
            lateral_offset=self.args.avoidance_lateral_offset,
            pass_distance=self.args.avoidance_pass_distance,
        )
        if route is None:
            return (
                f"head_on_conflict axis={conflict.axis} distance={conflict.distance:.3f}; "
                f"{evader.robot_id}=avoidance_unavailable"
            )

        evader.route = route
        evader.waypoint_index = 0
        evader.segment_start_pose = None
        evader.avoidance_replans += 1
        self.get_logger().info(
            f"{evader.robot_id} head-on avoidance route: "
            + " -> ".join(f"({x:.3f},{y:.3f})/{axis}" for (x, y), axis in zip(route.waypoints, route.axes))
        )
        return (
            f"head_on_avoidance evader={evader.robot_id} axis={conflict.axis} "
            f"distance={conflict.distance:.3f}"
        )

    def _step_robot(self, robot: RobotAxisState, allowed: bool) -> tuple[float, float, str]:
        if robot.route is None or robot.pose is None:
            return 0.0, 0.0, f"{robot.robot_id}=waiting_route"
        if robot.completed or robot.waypoint_index >= len(robot.route.waypoints):
            robot.completed = True
            return 0.0, 0.0, f"{robot.robot_id}=complete target={robot.target_name}"
        if not allowed:
            return 0.0, 0.0, f"{robot.robot_id}=reserved_wait target={robot.target_name}"

        if robot.segment_start_pose is None:
            robot.segment_start_pose = robot.pose

        target = robot.route.waypoints[robot.waypoint_index]
        active_axis = robot.route.axes[robot.waypoint_index]
        command = compute_axis_nav_command(
            robot.pose,
            target,
            segment_start=(robot.segment_start_pose.x, robot.segment_start_pose.y),
            active_axis=active_axis,
            max_linear_speed=self.args.speed,
            max_angular_speed=self.args.turn_speed,
            distance_tolerance=self.args.distance_tolerance,
            yaw_tolerance=self.args.yaw_tolerance,
            yaw_offset=self.args.yaw_offset,
            angular_sign=self.args.angular_sign,
        )

        if command.done:
            robot.waypoint_index += 1
            robot.segment_start_pose = None
            if robot.waypoint_index >= len(robot.route.waypoints):
                robot.completed = True

        text = (
            f"{robot.robot_id}=move target={robot.target_name} wp={robot.waypoint_index}/"
            f"{len(robot.route.waypoints)} axis={active_axis} "
            f"x={robot.pose.x:.3f} y={robot.pose.y:.3f} source={robot.pose_source} "
            f"linear={command.linear_x:.3f} angular={command.angular_z:.3f} done={command.done}"
        )
        return command.linear_x, command.angular_z, text

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

    def _publish_waiting_for_poses(self) -> None:
        missing = [
            robot.robot_id
            for robot in (self.robot_1, self.robot_2)
            if robot.pose is None
        ]
        text = (
            f"waiting for poses missing={','.join(missing)} "
            f"source={self.args.pose_source} "
            f"tf=({self.args.robot_1_tf},{self.args.robot_2_tf}) "
            f"odom=({self.args.robot_1_odom},{self.args.robot_2_odom})"
        )
        self._publish_status(text)

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if self.last_wait_log_time is None or now - self.last_wait_log_time >= 2.0:
            self.last_wait_log_time = now
            self.get_logger().info(text)

    def _try_publish_stop(self) -> None:
        try:
            self._publish_twists(0.0, 0.0, 0.0, 0.0)
        except Exception as exc:  # ROS may invalidate the context before KeyboardInterrupt is handled.
            self.get_logger().debug(f"Stop publish skipped during shutdown: {exc}")


def _pose_from_odom(msg: Odometry) -> Pose2D:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return Pose2D(
        x=position.x,
        y=position.y,
        yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _pose_from_transform(transform) -> Pose2D:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Pose2D(
        x=translation.x,
        y=translation.y,
        yaw=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _frame_matches_robot(
    child_frame_id: str,
    robot_id: str,
    base_frame: str,
    *,
    allow_unqualified_frame: bool = True,
) -> bool:
    if child_frame_id == base_frame or child_frame_id.endswith(f"/{base_frame}"):
        return True
    if allow_unqualified_frame and child_frame_id in {"chassis", "base_link", "iw_hub_sensors"}:
        return True
    if robot_id not in child_frame_id:
        return False
    frame_tail = child_frame_id.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"base_link", "iw_hub_sensors"}


def _parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Two-robot axis navigation with named place targets and lane reservations."
    )
    parser.add_argument("--robot-1-target", choices=sorted(PLACES), required=True)
    parser.add_argument("--robot-2-target", choices=sorted(PLACES), required=True)
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument("--robot-1-id", default=default_robot_id(1))
    parser.add_argument("--robot-2-id", default=default_robot_id(2))
    parser.add_argument("--robot-1-odom", default=default_odom_topic(1))
    parser.add_argument("--robot-2-odom", default=default_odom_topic(2))
    parser.add_argument("--robot-1-cmd-vel", default=default_cmd_vel_topic(1))
    parser.add_argument("--robot-2-cmd-vel", default=default_cmd_vel_topic(2))
    parser.add_argument("--robot-1-tf", default=f"/{default_robot_id(1)}/tf")
    parser.add_argument("--robot-2-tf", default=f"/{default_robot_id(2)}/tf")
    parser.add_argument("--robot-1-base-frame", default=default_base_frame(1))
    parser.add_argument("--robot-2-base-frame", default=default_base_frame(2))
    parser.add_argument(
        "--pose-source",
        choices=["tf", "odom", "auto"],
        default="tf",
        help="Use tf for world poses, odom for robot-local poses, or auto with tf overriding odom.",
    )
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--turn-speed", type=float, default=2.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.2)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--min-safe-distance",
        type=float,
        default=1.2,
        help="Stop both robots when their odom positions are closer than this distance. Use 0 to disable.",
    )
    parser.add_argument(
        "--enable-head-on-avoidance",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Insert a left-side dogleg route when both robots approach each other on the same axis segment.",
    )
    parser.add_argument(
        "--avoidance-evader",
        choices=["robot_1", "robot_2"],
        default="robot_2",
        help="Robot that inserts the bypass when a head-on conflict is detected.",
    )
    parser.add_argument(
        "--head-on-lane-tolerance",
        type=float,
        default=0.35,
        help="Maximum lateral distance for two same-axis segments to be treated as the same lane.",
    )
    parser.add_argument(
        "--head-on-trigger-distance",
        type=float,
        default=5.0,
        help="Start bypass planning when head-on robots are within this distance. Use 0 to ignore distance.",
    )
    parser.add_argument(
        "--avoidance-lateral-offset",
        type=float,
        default=1.0,
        help="Side-step distance for the temporary bypass lane.",
    )
    parser.add_argument(
        "--avoidance-pass-distance",
        type=float,
        default=1.5,
        help="How far past the other robot the bypass route rejoins the original lane.",
    )
    parser.add_argument(
        "--max-avoidance-replans",
        type=int,
        default=1,
        help="Maximum number of bypass insertions per robot for one navigation run.",
    )
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running reserved_axis_nav.")

    parsed_args = _parse_args(args)
    rclpy.init(args=args)
    node = ReservedAxisNav(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node._try_publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
