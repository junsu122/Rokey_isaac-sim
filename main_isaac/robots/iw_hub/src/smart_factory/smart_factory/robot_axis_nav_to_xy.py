from __future__ import annotations

import argparse
from typing import Optional

from smart_factory.axis_nav_to_place import AxisRoute, compute_axis_nav_command
from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot1_stack_sequence import (
    _build_left_bypass_route,
    _is_peer_head_on_conflict,
    _limit_command_acceleration,
)
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
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    TFMessage = object


def build_xy_axis_route(
    start: tuple[float, float],
    target: tuple[float, float],
    *,
    axis_order: str,
) -> AxisRoute:
    if axis_order == "xy":
        steps = [((target[0], start[1]), "x"), (target, "y")]
    elif axis_order == "yx":
        steps = [((start[0], target[1]), "y"), (target, "x")]
    else:
        raise ValueError("axis_order must be 'xy' or 'yx'")
    waypoints, axes = _deduplicate_route_steps(start, steps)
    return AxisRoute(target_name=f"XY({target[0]:.3f},{target[1]:.3f})", waypoints=waypoints, axes=axes)


def _deduplicate_route_steps(
    start: tuple[float, float],
    steps: list[tuple[tuple[float, float], str]],
) -> tuple[list[tuple[float, float]], list[str]]:
    waypoints = []
    axes = []
    previous = start
    for point, axis in steps:
        if abs(previous[0] - point[0]) > 1e-3 or abs(previous[1] - point[1]) > 1e-3:
            waypoints.append(point)
            axes.append(axis)
            previous = point
    return waypoints, axes


class RobotAxisNavToXY(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(f"smart_factory_{args.robot_id}_axis_nav_to_xy")
        self.args = args
        self.pose: Pose2D | None = None
        self.peer_pose: Pose2D | None = None
        self.pose_source: str | None = None
        self.peer_pose_source: str | None = None
        self.route: AxisRoute | None = None
        self.waypoint_index = 0
        self.segment_start_pose: Pose2D | None = None
        self.avoidance_applied = False
        self.last_command_linear_x: float | None = None
        self.last_command_angular_z: float | None = None
        self.last_command_time: float | None = None

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self._subscriptions = [
            self.create_subscription(Odometry, args.odom_topic, self._on_odom, isaac_qos),
            self.create_subscription(TFMessage, args.tf_topic, self._on_tf, isaac_qos),
            self.create_subscription(Odometry, args.peer_odom_topic, self._on_peer_odom, isaac_qos),
            self.create_subscription(TFMessage, args.peer_tf_topic, self._on_peer_tf, isaac_qos),
        ]
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"{args.robot_id} axis nav to ({args.x:.3f},{args.y:.3f}); "
            f"peer={args.peer_robot_id}; avoidance_role={args.avoidance_role}; "
            f"pose_source={args.pose_source}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self.args.pose_source in {"odom", "auto"} and self.pose_source != "tf":
            self.pose = _pose_from_odom(msg)
            self.pose_source = "odom"

    def _on_tf(self, msg: TFMessage) -> None:
        if self.args.pose_source not in {"tf", "auto"}:
            return
        for transform in msg.transforms:
            if _is_world_to_robot_base(transform, self.args.base_frame):
                self.pose = _pose_from_transform(transform)
                self.pose_source = "tf"
                return

    def _on_peer_odom(self, msg: Odometry) -> None:
        if self.args.peer_pose_source in {"odom", "auto"} and self.peer_pose_source != "tf":
            self.peer_pose = _pose_from_odom(msg)
            self.peer_pose_source = "odom"

    def _on_peer_tf(self, msg: TFMessage) -> None:
        if self.args.peer_pose_source not in {"tf", "auto"}:
            return
        for transform in msg.transforms:
            if _is_world_to_robot_base(transform, self.args.peer_base_frame):
                self.peer_pose = _pose_from_transform(transform)
                self.peer_pose_source = "tf"
                return

    def _on_timer(self) -> None:
        if self.pose is None:
            self._publish_command(0.0, 0.0)
            return
        if self.route is None:
            self.route = build_xy_axis_route(
                (self.pose.x, self.pose.y),
                (self.args.x, self.args.y),
                axis_order=self.args.axis_order,
            )
            self.get_logger().info(
                "Route: "
                + " -> ".join(
                    f"({x:.3f},{y:.3f})/{axis}" for (x, y), axis in zip(self.route.waypoints, self.route.axes)
                )
            )

        if self.waypoint_index >= len(self.route.waypoints):
            self._publish_command(0.0, 0.0)
            return

        if self.segment_start_pose is None:
            self.segment_start_pose = self.pose

        target = self.route.waypoints[self.waypoint_index]
        active_axis = self.route.axes[self.waypoint_index]
        avoidance_action = self._maybe_handle_peer_head_on(target, active_axis)
        if avoidance_action == "yield":
            return
        if avoidance_action == "reroute":
            self.segment_start_pose = self.pose
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
            allow_crossed_axis_target=not (self.avoidance_applied and self.waypoint_index <= 2),
        )
        self._publish_command(command.linear_x, command.angular_z)
        if command.done:
            self.waypoint_index += 1
            self.segment_start_pose = None

    def _maybe_handle_peer_head_on(self, target: tuple[float, float], active_axis: str) -> str:
        if self.args.avoidance_role == "off" or self.peer_pose is None or self.pose is None:
            return ""
        if not _is_peer_head_on_conflict(
            self.pose,
            self.peer_pose,
            target,
            active_axis,
            lane_tolerance=self.args.peer_lane_tolerance,
            trigger_distance=self.args.peer_avoidance_trigger_distance,
            path_margin=self.args.peer_avoidance_path_margin,
            peer_yaw_tolerance=self.args.peer_yaw_tolerance,
        ):
            return ""
        if self.args.avoidance_role == "yield":
            self._publish_command(0.0, 0.0)
            return "yield"
        if self.avoidance_applied or self.route is None:
            return ""
        route = _build_left_bypass_route(
            (self.pose.x, self.pose.y),
            self.peer_pose,
            self.route,
            self.waypoint_index,
            active_axis,
            lateral_offset=self.args.peer_avoidance_lateral_offset,
            pass_distance=self.args.peer_avoidance_pass_distance,
        )
        if route is None:
            return ""
        self.route = route
        self.waypoint_index = 0
        self.segment_start_pose = None
        self.avoidance_applied = True
        self.get_logger().info(
            "Peer avoidance route: "
            + " -> ".join(f"({x:.3f},{y:.3f})/{axis}" for (x, y), axis in zip(route.waypoints, route.axes))
        )
        return "reroute"

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        linear_x, angular_z = _limit_command_acceleration(
            linear_x,
            angular_z,
            previous_linear_x=self.last_command_linear_x,
            previous_angular_z=self.last_command_angular_z,
            dt=0.0 if self.last_command_time is None else now - self.last_command_time,
            linear_accel_limit=self.args.linear_accel_limit,
            angular_accel_limit=self.args.angular_accel_limit,
        )
        self.last_command_linear_x = linear_x
        self.last_command_angular_z = angular_z
        self.last_command_time = now
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_pub.publish(command)

    def _try_publish_stop(self) -> None:
        try:
            self._publish_command(0.0, 0.0)
        except Exception as exc:
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


def _is_world_to_robot_base(transform, base_frame: str) -> bool:
    if transform.header.frame_id != "world":
        return False
    child_frame = transform.child_frame_id
    if child_frame == base_frame or child_frame.endswith(f"/{base_frame}"):
        return True
    frame_tail = child_frame.rsplit("/", maxsplit=1)[-1]
    base_tail = base_frame.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"chassis", "base_link", "iw_hub_sensors", base_tail}


def _parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test axis navigation to an arbitrary XY target.")
    parser.add_argument("--robot-index", type=int, choices=[1, 2], required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument("--pose-source", choices=["tf", "odom", "auto"], default="tf")
    parser.add_argument("--peer-pose-source", choices=["tf", "odom", "auto"], default="tf")
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--turn-speed", type=float, default=1.5)
    parser.add_argument("--distance-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument("--linear-accel-limit", type=float, default=2.0)
    parser.add_argument("--angular-accel-limit", type=float, default=3.0)
    parser.add_argument("--peer-lane-tolerance", type=float, default=0.35)
    parser.add_argument("--peer-avoidance-trigger-distance", type=float, default=2.5)
    parser.add_argument("--peer-avoidance-path-margin", type=float, default=0.5)
    parser.add_argument("--peer-avoidance-lateral-offset", type=float, default=1.0)
    parser.add_argument("--peer-avoidance-pass-distance", type=float, default=1.5)
    parser.add_argument("--peer-yaw-tolerance", type=float, default=0.75)
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parsed = parser.parse_args(args)

    peer_index = 2 if parsed.robot_index == 1 else 1
    parsed.robot_id = default_robot_id(parsed.robot_index)
    parsed.peer_robot_id = default_robot_id(peer_index)
    parsed.odom_topic = default_odom_topic(parsed.robot_index)
    parsed.peer_odom_topic = default_odom_topic(peer_index)
    parsed.tf_topic = f"/{parsed.robot_id}/tf"
    parsed.peer_tf_topic = f"/{parsed.peer_robot_id}/tf"
    parsed.base_frame = default_base_frame(parsed.robot_index)
    parsed.peer_base_frame = default_base_frame(peer_index)
    parsed.cmd_vel_topic = default_cmd_vel_topic(parsed.robot_index)
    parsed.avoidance_role = "yield" if parsed.robot_index == 1 else "evade"
    return parsed


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running robot_axis_nav_to_xy.")

    parsed_args = _parse_args(args)
    rclpy.init(args=args)
    node = RobotAxisNavToXY(parsed_args)
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
