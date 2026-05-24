from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from smart_factory.axis_nav_to_place import (
    AxisRoute,
    PLACES,
    build_axis_route,
    compute_axis_nav_command,
)
from smart_factory.models import Pose2D
from smart_factory.no_go_zones import plan_axis_route_around_zones, route_crosses_no_go
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import (
    default_base_frame,
    default_cmd_vel_topic,
    default_odom_topic,
    default_robot_id,
)

try:
    import rclpy
    from geometry_msgs.msg import Quaternion, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Quaternion = object
    Twist = object
    Odometry = object
    Node = object
    QoSProfile = object
    JointState = object
    String = object
    TFMessage = object

try:
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:
    ActionClient = object
    GoalStatus = object
    NavigateToPose = object


class SequencePhase(Enum):
    MOVE_TO_STACK = "move_to_stack"
    LIFT_UP = "lift_up"
    WAIT_AFTER_LIFT = "wait_after_lift"
    # MOVE_TO_SHELF_STORAGE = "move_to_shelf_storage"  # 비활성화 — 창고에 SHELF_STORAGE 없음
    # STOP_AT_SHELF_STORAGE = "stop_at_shelf_storage"  # 비활성화
    MOVE_TO_UNLOAD_1 = "move_to_unload_1"
    SETTLE_AT_UNLOAD = "settle_at_unload"
    LIFT_DOWN = "lift_down"
    BACK_OUT_FROM_UNLOAD = "back_out_from_unload"
    MOVE_TO_WAIT_1 = "move_to_wait_1"
    COMPLETE = "complete"


@dataclass
class MoveState:
    target_name: str
    route: AxisRoute | None = None
    waypoint_index: int = 0
    segment_start_pose: Pose2D | None = None
    pre_aligned_waypoint_index: int | None = None
    last_pre_align_error: float | None = None
    last_pre_align_time: float | None = None
    avoidance_applied: bool = False
    reverse_avoidance_waypoint_index: int | None = None
    reverse_avoidance_start_pose: Pose2D | None = None
    nav_goal_future: object | None = None
    nav_result_future: object | None = None
    nav_goal_handle: object | None = None
    nav_goal_waypoint_index: int | None = None


@dataclass
class PeerReservationState:
    robot_id: str
    phase: str
    target_name: str
    priority: int
    active: bool
    received_at: float
    cell: tuple[int, int] | None = None
    next_cell: tuple[int, int] | None = None


class Robot1StackSequence(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(f"smart_factory_{args.robot_id}_stack_sequence")
        self.args = args
        self.odom_pose: Pose2D | None = None
        self.tf_pose: Pose2D | None = None
        self.center_pose: Pose2D | None = None
        self.pose_source: str | None = None
        self.pose: Pose2D | None = None
        self.peer_odom_pose: Pose2D | None = None
        self.peer_tf_pose: Pose2D | None = None
        self.peer_pose: Pose2D | None = None
        self.peer_pose_source: str | None = None
        self.last_command_linear_x: float | None = None
        self.last_command_angular_z: float | None = None
        self.last_command_time: float | None = None
        self.peer_reservation: PeerReservationState | None = None
        self.peer_safety_paused = False
        self.phase = SequencePhase.MOVE_TO_STACK
        self.phase_started_at = self._now()
        self.move_state = MoveState(args.stack_target)

        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(JointState, args.lift_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.reservation_pub = self.create_publisher(String, args.reservation_topic, 10)
        self.nav2_client = None
        if args.motion_controller == "nav2":
            if ActionClient is object or NavigateToPose is object:
                raise RuntimeError(
                    "nav2_msgs and rclpy.action are required for --motion-controller nav2"
                )
            self.nav2_client = ActionClient(self, NavigateToPose, args.nav2_action_name)
        self._odom_subscription = self.create_subscription(
            Odometry,
            args.odom_topic,
            self._on_odom,
            isaac_qos,
        )
        self._tf_subscription = self.create_subscription(
            TFMessage,
            args.tf_topic,
            self._on_tf,
            isaac_qos,
        )
        self._peer_odom_subscription = self.create_subscription(
            Odometry,
            args.peer_odom_topic,
            self._on_peer_odom,
            isaac_qos,
        )
        self._peer_tf_subscription = self.create_subscription(
            TFMessage,
            args.peer_tf_topic,
            self._on_peer_tf,
            isaac_qos,
        )
        self._peer_reservation_subscription = self.create_subscription(
            String,
            args.peer_reservation_topic,
            self._on_peer_reservation,
            10,
        )
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)
        self.get_logger().info(
            f"{args.robot_id} stack sequence: {args.stack_target} -> lift up -> wait -> "
            f"{args.unload_target} -> lift down -> "
            f"{args.wait_target}; "
            f"pose_source={args.pose_source}; peer={args.peer_robot_id}; "
            f"avoidance_role={args.avoidance_role}; reservation_priority={args.reservation_priority}; "
            f"motion_controller={args.motion_controller}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_pose = _pose_from_odom(msg)
        if self.args.pose_source == "odom" or (
            self.args.pose_source == "auto" and self.pose_source != "tf"
        ):
            self._set_pose(self.odom_pose, "odom")

    def _on_tf(self, msg: TFMessage) -> None:
        if self.args.pose_source not in {"tf", "auto"}:
            return
        for transform in msg.transforms:
            if _is_world_to_robot_base(transform, self.args.base_frame):
                self.tf_pose = _pose_from_transform(transform)
                self._set_pose(self.tf_pose, "tf")
                return

    def _on_peer_odom(self, msg: Odometry) -> None:
        self.peer_odom_pose = _pose_from_odom(msg)
        if self.args.peer_pose_source == "odom" or (
            self.args.peer_pose_source == "auto" and self.peer_pose_source != "tf"
        ):
            self.peer_pose = self.peer_odom_pose
            self.peer_pose_source = "odom"

    def _on_peer_tf(self, msg: TFMessage) -> None:
        if self.args.peer_pose_source not in {"tf", "auto"}:
            return
        for transform in msg.transforms:
            if _is_world_to_robot_base(transform, self.args.peer_base_frame):
                self.peer_tf_pose = _pose_from_transform(transform)
                self.peer_pose = self.peer_tf_pose
                self.peer_pose_source = "tf"
                return

    def _on_peer_reservation(self, msg: String) -> None:
        reservation = _parse_reservation_status(msg.data, received_at=self._now())
        if reservation is not None and reservation.robot_id != self.args.robot_id:
            self.peer_reservation = reservation

    def _set_pose(self, pose: Pose2D, source: str) -> None:
        self.center_pose = pose
        self.pose = _offset_pose(
            pose,
            offset_x=self.args.tracking_offset_x,
            offset_y=self.args.tracking_offset_y,
        )
        self.pose_source = source

    def _on_timer(self) -> None:
        self._publish_reservation_status()
        if self.pose is None:
            self._publish_stop()
            self._publish_status("waiting for odom")
            return

        if self.phase == SequencePhase.MOVE_TO_STACK:
            if self._step_move(self.args.stack_target):
                self._change_phase(SequencePhase.LIFT_UP)
            return

        if self.phase == SequencePhase.LIFT_UP:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            self._change_phase(SequencePhase.WAIT_AFTER_LIFT)
            return

        if self.phase == SequencePhase.WAIT_AFTER_LIFT:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            if self._elapsed() >= self.args.wait_after_lift:
                self._change_phase(SequencePhase.MOVE_TO_UNLOAD_1)  # SHELF_STORAGE 건너뜀
            else:
                self._publish_status(f"phase={self.phase.value}; waiting={self._elapsed():.2f}")
            return

        # SHELF_STORAGE 비활성화 — 창고에 해당 위치 없음
        # if self.phase == SequencePhase.MOVE_TO_SHELF_STORAGE:
        #     self._publish_lift(self.args.lift_up_position)
        #     if self._step_move(self.args.shelf_storage_target):
        #         self._change_phase(SequencePhase.STOP_AT_SHELF_STORAGE)
        #     return

        # if self.phase == SequencePhase.STOP_AT_SHELF_STORAGE:
        #     self._publish_stop()
        #     self._publish_lift(self.args.lift_up_position)
        #     if self._elapsed() >= self.args.shelf_stop_duration:
        #         self._change_phase(SequencePhase.MOVE_TO_UNLOAD_1)
        #     else:
        #         self._publish_status(f"phase={self.phase.value}; stopped={self._elapsed():.2f}")
        #     return

        if self.phase == SequencePhase.MOVE_TO_UNLOAD_1:
            self._publish_lift(self.args.lift_up_position)
            if self._step_move(self.args.unload_target):
                self._change_phase(SequencePhase.SETTLE_AT_UNLOAD)
            return

        if self.phase == SequencePhase.SETTLE_AT_UNLOAD:
            self._publish_stop()
            self._publish_lift(self.args.lift_up_position)
            if self._elapsed() >= self.args.unload_settle_duration:
                self._change_phase(SequencePhase.LIFT_DOWN)
            else:
                self._publish_status(
                    f"phase={self.phase.value}; settling={self._elapsed():.2f}; "
                    f"lift={self.args.lift_up_position:.3f}"
                )
            return

        if self.phase == SequencePhase.LIFT_DOWN:
            self._publish_stop()
            self._publish_lift(self.args.lift_down_position)
            if self._elapsed() >= self.args.lift_down_hold:
                self._change_phase(SequencePhase.BACK_OUT_FROM_UNLOAD)
            else:
                self._publish_status(
                    f"phase={self.phase.value}; lowering={self._elapsed():.2f}; "
                    f"lift={self.args.lift_down_position:.3f}"
                )
            return

        if self.phase == SequencePhase.BACK_OUT_FROM_UNLOAD:
            self._publish_lift(self.args.lift_down_position)
            if self._elapsed() >= self.args.back_out_duration:
                self._publish_stop()
                self._change_phase(SequencePhase.MOVE_TO_WAIT_1)
            else:
                self._publish_command(-abs(self.args.back_out_speed), 0.0)
                self._publish_status(
                    f"phase={self.phase.value}; backing={self._elapsed():.2f}; "
                    f"speed={-abs(self.args.back_out_speed):.3f}"
                )
            return

        if self.phase == SequencePhase.MOVE_TO_WAIT_1:
            self._publish_lift(self.args.lift_down_position)
            if self._step_move(self.args.wait_target):
                self._change_phase(SequencePhase.COMPLETE)
            return

        self._publish_stop()
        self._publish_lift(self.args.lift_down_position)
        self._publish_status("phase=complete")

    def _step_move(self, target_name: str) -> bool:
        if self.move_state.target_name != target_name:
            self.move_state = MoveState(target_name)

        if self.args.motion_controller != "nav2":
            safety_wait_reason = self._peer_safety_wait_reason()
            if safety_wait_reason:
                self._publish_stop()
                self._publish_status(
                    f"phase={self.phase.value}; target={target_name}; peer_safety_wait={safety_wait_reason}"
                )
                return False

        if self.move_state.route is None:
            route_start_pose = self._pose_for_route_start(target_name)
            self.move_state.route = _build_route_for_sequence_target(
                (route_start_pose.x, route_start_pose.y),
                target_name,
                self.args,
            )
            if self.args.motion_controller == "nav2" and self.args.nav2_expand_grid_steps:
                self.move_state.route = _expand_route_for_grid_reservation(
                    (route_start_pose.x, route_start_pose.y),
                    self.move_state.route,
                    cell_size=self.args.grid_cell_size,
                    origin_x=self.args.grid_origin_x,
                    origin_y=self.args.grid_origin_y,
                )
            self.get_logger().info(
                f"Route to {self.move_state.route.target_name}: "
                + " -> ".join(f"({x:.3f},{y:.3f})" for x, y in self.move_state.route.waypoints)
            )
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; route="
                + "->".join(
                    f"({x:.3f},{y:.3f})/{axis}"
                    for (x, y), axis in zip(self.move_state.route.waypoints, self.move_state.route.axes)
                )
            )

        if self.args.motion_controller == "nav2":
            return self._step_nav2_move(target_name)

        if self.move_state.waypoint_index >= len(self.move_state.route.waypoints):
            self._publish_stop()
            self._publish_status(f"phase={self.phase.value}; target={target_name}; done=True")
            return True

        if self._continue_reverse_right_avoidance(target_name):
            return False

        reservation_wait_reason = self._reservation_wait_reason(target_name)
        if reservation_wait_reason:
            self._publish_stop()
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; reservation_wait={reservation_wait_reason}"
            )
            return False

        grid_action = self._maybe_handle_grid_edge_swap(target_name)
        if grid_action in {"yield", "reverse_right"}:
            return False

        grid_wait_reason = self._grid_reservation_wait_reason()
        if grid_wait_reason:
            self._publish_stop()
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; grid_reservation_wait={grid_wait_reason}"
            )
            return False

        if self.move_state.segment_start_pose is None:
            self.move_state.segment_start_pose = self.pose

        target = self.move_state.route.waypoints[self.move_state.waypoint_index]
        active_axis = self.move_state.route.axes[self.move_state.waypoint_index]
        control_pose = self._pose_for_segment(target_name, active_axis)
        avoidance_action = self._maybe_handle_peer_head_on(target_name, target, active_axis, control_pose)
        if avoidance_action == "yield":
            return False
        if avoidance_action == "reverse_right":
            return False
        if self._should_pre_align_before_approach(target_name):
            if not self._step_pre_align(target_name, target, active_axis, control_pose):
                return False

        command = compute_axis_nav_command(
            control_pose,
            target,
            segment_start=(
                self.move_state.segment_start_pose.x,
                self.move_state.segment_start_pose.y,
            ),
            active_axis=active_axis,
            max_linear_speed=self._speed_for_segment(target_name, active_axis),
            max_angular_speed=self._turn_speed_for_segment(),
            distance_tolerance=self._distance_tolerance_for_segment(target_name, active_axis),
            yaw_tolerance=self.args.yaw_tolerance,
            yaw_offset=self.args.yaw_offset,
            angular_sign=self.args.angular_sign,
            allow_crossed_axis_target=self._allow_crossed_axis_target(target_name, active_axis),
            axis_aligned_heading=self._use_axis_aligned_heading(target_name, active_axis),
            reverse_motion=self._use_reverse_motion_for_segment(),
        )
        self._publish_command(command.linear_x, command.angular_z)
        self._publish_status(
            f"phase={self.phase.value}; target={target_name}; waypoint="
            f"{self.move_state.waypoint_index + 1}/{len(self.move_state.route.waypoints)}; "
            f"odom_x={_format_pose_value(self.odom_pose, 'x')}; "
            f"odom_y={_format_pose_value(self.odom_pose, 'y')}; "
            f"tf_x={_format_pose_value(self.tf_pose, 'x')}; "
            f"tf_y={_format_pose_value(self.tf_pose, 'y')}; source={self.pose_source}; "
            f"track_x={self.pose.x:.3f}; track_y={self.pose.y:.3f}; "
            f"control_x={control_pose.x:.3f}; control_y={control_pose.y:.3f}; axis={active_axis}; "
            f"goal=({target[0]:.3f},{target[1]:.3f}); "
            f"axis_error={command.axis_error:.3f}; tolerance="
            f"{self._distance_tolerance_for_segment(target_name, active_axis):.3f}; "
            f"target_yaw={command.target_yaw:.3f}; control_yaw={command.control_yaw:.3f}; "
            f"yaw_error={command.yaw_error:.3f}; "
            f"linear={command.linear_x:.3f}; angular={command.angular_z:.3f}; "
            f"done={command.done}"
        )

        if command.done:
            self.move_state.waypoint_index += 1
            self.move_state.segment_start_pose = None
        return False

    def _step_nav2_move(self, target_name: str) -> bool:
        if self.move_state.route is None:
            return False
        if self.nav2_client is None:
            raise RuntimeError("Nav2 action client is not initialized")

        if self.move_state.waypoint_index >= len(self.move_state.route.waypoints):
            self._publish_status(f"phase={self.phase.value}; target={target_name}; done=True")
            return True

        if self.move_state.nav_goal_future is not None:
            if not self.move_state.nav_goal_future.done():
                self._publish_status(
                    f"phase={self.phase.value}; target={target_name}; nav2=sending_goal"
                )
                return False

            goal_handle = self.move_state.nav_goal_future.result()
            self.move_state.nav_goal_future = None
            if not goal_handle.accepted:
                self._publish_status(
                    f"phase={self.phase.value}; target={target_name}; nav2=goal_rejected"
                )
                self.move_state.nav_goal_handle = None
                return False

            self.move_state.nav_goal_handle = goal_handle
            self.move_state.nav_result_future = goal_handle.get_result_async()

        if self.move_state.nav_result_future is not None:
            if not self.move_state.nav_result_future.done():
                target = self.move_state.route.waypoints[self.move_state.waypoint_index]
                self._publish_status(
                    f"phase={self.phase.value}; target={target_name}; nav2=active; "
                    f"waypoint={self.move_state.waypoint_index + 1}/"
                    f"{len(self.move_state.route.waypoints)}; "
                    f"goal=({target[0]:.3f},{target[1]:.3f})"
                )
                return False

            result = self.move_state.nav_result_future.result()
            status = getattr(result, "status", None)
            self.move_state.nav_result_future = None
            self.move_state.nav_goal_handle = None
            self.move_state.nav_goal_waypoint_index = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.move_state.waypoint_index += 1
                self.move_state.segment_start_pose = None
                return False

            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; nav2=result_status={status}"
            )
            return False

        reservation_wait_reason = self._reservation_wait_reason(target_name)
        if reservation_wait_reason:
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; reservation_wait={reservation_wait_reason}"
            )
            return False

        grid_wait_reason = self._grid_reservation_wait_reason()
        if grid_wait_reason:
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; grid_reservation_wait={grid_wait_reason}"
            )
            return False

        if not self.nav2_client.wait_for_server(timeout_sec=0.0):
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; nav2=waiting_for_server; "
                f"action={self.args.nav2_action_name}"
            )
            return False

        waypoint_index = self.move_state.waypoint_index
        target = self.move_state.route.waypoints[waypoint_index]
        active_axis = self.move_state.route.axes[waypoint_index]
        yaw = _target_yaw_for_segment(
            self.pose,
            target,
            active_axis,
            axis_aligned_heading=self._use_axis_aligned_heading(target_name, active_axis),
        )
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.args.nav2_goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _quaternion_msg_from_yaw(yaw)

        self.move_state.nav_goal_waypoint_index = waypoint_index
        self.move_state.nav_goal_future = self.nav2_client.send_goal_async(goal)
        self._publish_status(
            f"phase={self.phase.value}; target={target_name}; nav2=goal_sent; "
            f"waypoint={waypoint_index + 1}/{len(self.move_state.route.waypoints)}; "
            f"goal=({target[0]:.3f},{target[1]:.3f}); yaw={yaw:.3f}"
        )
        return False

    def _peer_safety_wait_reason(self) -> str:
        if not self.args.enable_peer_safety_stop or self.peer_pose is None or self.pose is None:
            self.peer_safety_paused = False
            return ""

        distance = math.hypot(self.pose.x - self.peer_pose.x, self.pose.y - self.peer_pose.y)
        if self.peer_safety_paused:
            if distance >= self.args.peer_safety_resume_distance:
                self.peer_safety_paused = False
                return ""
            return (
                f"peer={self.args.peer_robot_id} distance={distance:.3f} "
                f"resume={self.args.peer_safety_resume_distance:.3f}"
            )

        if distance <= self.args.peer_safety_stop_distance:
            wait_reason = self._peer_safety_priority_wait_reason()
            if not wait_reason:
                return ""
            self.peer_safety_paused = True
            return (
                f"peer={self.args.peer_robot_id} distance={distance:.3f} "
                f"stop={self.args.peer_safety_stop_distance:.3f} reason={wait_reason}"
            )
        return ""

    def _peer_safety_priority_wait_reason(self) -> str:
        if self.pose is None:
            return ""
        current_cell = _world_to_grid_cell(
            self.pose.x,
            self.pose.y,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )
        next_cell = _next_grid_cell(
            current_cell,
            self.pose,
            self.move_state.route,
            self.move_state.waypoint_index,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )
        if self.peer_reservation is not None and self.peer_reservation.active:
            if (
                self.args.avoidance_role == "evade"
                and _is_grid_edge_swap(
                    current_cell=current_cell,
                    next_cell=next_cell,
                    peer_cell=self.peer_reservation.cell,
                    peer_next_cell=self.peer_reservation.next_cell,
                )
            ):
                return ""
            reason = _grid_reservation_conflict_reason(
                robot_id=self.args.robot_id,
                priority=self.args.reservation_priority,
                current_cell=current_cell,
                next_cell=next_cell,
                peer=self.peer_reservation,
            )
            if reason:
                return reason
            if self.peer_reservation.cell is not None or self.peer_reservation.next_cell is not None:
                return ""

        if _reservation_has_priority(
            self.args.robot_id,
            self.args.reservation_priority,
            self.args.peer_robot_id,
            self.args.peer_reservation_priority,
        ):
            return ""
        return f"peer={self.args.peer_robot_id} priority={self.args.peer_reservation_priority}"

    def _reservation_wait_reason(self, target_name: str) -> str:
        if not self.args.enable_place_reservation:
            return ""
        if not _is_reserved_place(target_name, self.args.reserved_place_prefixes):
            return ""
        if self.peer_reservation is None or not self.peer_reservation.active:
            return ""
        if self._now() - self.peer_reservation.received_at > self.args.peer_reservation_timeout:
            return ""
        if self.peer_reservation.target_name != target_name:
            return ""
        if _reservation_has_priority(
            self.args.robot_id,
            self.args.reservation_priority,
            self.peer_reservation.robot_id,
            self.peer_reservation.priority,
        ):
            return ""
        return f"peer={self.peer_reservation.robot_id} target={target_name}"

    def _grid_reservation_wait_reason(self) -> str:
        if not self.args.enable_grid_reservation or self.pose is None:
            return ""
        if self.peer_reservation is None or not self.peer_reservation.active:
            return ""
        if self._now() - self.peer_reservation.received_at > self.args.peer_reservation_timeout:
            return ""
        if self.peer_reservation.cell is None and self.peer_reservation.next_cell is None:
            return ""

        current_cell = _world_to_grid_cell(
            self.pose.x,
            self.pose.y,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )
        next_cell = _next_grid_cell(
            current_cell,
            self.pose,
            self.move_state.route,
            self.move_state.waypoint_index,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )

        return _grid_reservation_conflict_reason(
            robot_id=self.args.robot_id,
            priority=self.args.reservation_priority,
            current_cell=current_cell,
            next_cell=next_cell,
            peer=self.peer_reservation,
        )

    def _maybe_handle_peer_head_on(
        self,
        target_name: str,
        target: tuple[float, float],
        active_axis: str,
        control_pose: Pose2D,
    ) -> str:
        if self.args.avoidance_role == "off" or self.peer_pose is None:
            return ""
        if not _is_peer_head_on_conflict(
            control_pose,
            self.peer_pose,
            target,
            active_axis,
            lane_tolerance=self.args.peer_lane_tolerance,
            trigger_distance=self.args.peer_avoidance_trigger_distance,
            path_margin=self.args.peer_avoidance_path_margin,
            peer_yaw_tolerance=self.args.peer_yaw_tolerance,
        ):
            return ""

        distance = math.hypot(control_pose.x - self.peer_pose.x, control_pose.y - self.peer_pose.y)
        if self.args.avoidance_role == "yield":
            self._publish_stop()
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; peer_head_on=yield; "
                f"peer={self.args.peer_robot_id}; distance={distance:.3f}; source={self.peer_pose_source}"
            )
            return "yield"

        self.move_state.segment_start_pose = None
        self.move_state.pre_aligned_waypoint_index = None
        self._publish_reverse_right_avoidance(
            target_name,
            source="peer_head_on",
            detail=(
                f"peer={self.args.peer_robot_id}; distance={distance:.3f}; "
                f"source={self.peer_pose_source}"
            ),
        )
        return "reverse_right"

    def _continue_reverse_right_avoidance(self, target_name: str) -> bool:
        if self.move_state.reverse_avoidance_start_pose is None or self.pose is None:
            return False
        distance = math.hypot(
            self.pose.x - self.move_state.reverse_avoidance_start_pose.x,
            self.pose.y - self.move_state.reverse_avoidance_start_pose.y,
        )
        if distance >= self.args.peer_avoidance_reverse_distance:
            self.move_state.reverse_avoidance_start_pose = None
            return False
        self._publish_reverse_right_avoidance(
            target_name,
            source="peer_head_on",
            detail=f"reverse_distance={distance:.3f}/{self.args.peer_avoidance_reverse_distance:.3f}",
        )
        return True

    def _publish_reverse_right_avoidance(self, target_name: str, *, source: str, detail: str) -> None:
        if self.pose is not None and self.move_state.reverse_avoidance_start_pose is None:
            self.move_state.reverse_avoidance_start_pose = self.pose
        reverse_speed = -abs(self.args.peer_avoidance_speed)
        right_turn = -abs(self.args.peer_avoidance_turn_speed) * self.args.angular_sign
        self._publish_command(reverse_speed, right_turn)
        self._publish_status(
            f"phase={self.phase.value}; target={target_name}; {source}=reverse_right; "
            f"{detail}; linear={reverse_speed:.3f}; angular={right_turn:.3f}"
        )

    def _maybe_handle_grid_edge_swap(self, target_name: str) -> str:
        if (
            not self.args.enable_grid_reservation
            or self.args.avoidance_role == "off"
            or self.pose is None
            or self.move_state.route is None
            or self.move_state.avoidance_applied
            or self.peer_reservation is None
            or not self.peer_reservation.active
        ):
            return ""
        if self._now() - self.peer_reservation.received_at > self.args.peer_reservation_timeout:
            return ""
        if self.peer_reservation.cell is None or self.peer_reservation.next_cell is None:
            return ""

        current_cell = _world_to_grid_cell(
            self.pose.x,
            self.pose.y,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )
        next_cell = _next_grid_cell(
            current_cell,
            self.pose,
            self.move_state.route,
            self.move_state.waypoint_index,
            cell_size=self.args.grid_cell_size,
            origin_x=self.args.grid_origin_x,
            origin_y=self.args.grid_origin_y,
        )
        if not _is_grid_edge_swap(
            current_cell=current_cell,
            next_cell=next_cell,
            peer_cell=self.peer_reservation.cell,
            peer_next_cell=self.peer_reservation.next_cell,
        ):
            return ""

        if self.args.avoidance_role == "yield":
            self._publish_stop()
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; grid_head_on=yield; "
                f"edge={current_cell}->{next_cell}; peer={self.peer_reservation.robot_id}"
            )
            return "yield"

        self.move_state.segment_start_pose = None
        self.move_state.pre_aligned_waypoint_index = None
        self._publish_reverse_right_avoidance(
            target_name,
            source="grid_head_on",
            detail=f"edge={current_cell}->{next_cell}; peer={self.peer_reservation.robot_id}",
        )
        return "reverse_right"

    def _should_pre_align_before_approach(self, target_name: str) -> bool:
        if self.move_state.route is None:
            return False
        return _should_pre_align_before_approach(
            self.args,
            target_name,
            waypoint_index=self.move_state.waypoint_index,
            waypoint_count=len(self.move_state.route.waypoints),
            pre_aligned_waypoint_index=self.move_state.pre_aligned_waypoint_index,
        )

    def _step_pre_align(
        self,
        target_name: str,
        target: tuple[float, float],
        active_axis: str,
        control_pose: Pose2D,
    ) -> bool:
        target_yaw = _target_yaw_for_segment(
            control_pose,
            target,
            active_axis,
            axis_aligned_heading=self._use_axis_aligned_heading(target_name, active_axis),
        )
        control_yaw = _normalize_angle(control_pose.yaw + self.args.yaw_offset)
        yaw_error = _normalize_angle(target_yaw - control_yaw)
        now = self._now()

        derivative = 0.0
        if (
            self.move_state.last_pre_align_error is not None
            and self.move_state.last_pre_align_time is not None
        ):
            dt = max(1e-3, now - self.move_state.last_pre_align_time)
            derivative = _normalize_angle(yaw_error - self.move_state.last_pre_align_error) / dt

        angular_z = (
            self.args.stack_pre_align_kp * yaw_error
            + self.args.stack_pre_align_kd * derivative
        ) * self.args.angular_sign
        angular_z = _clamp(
            angular_z,
            -self.args.stack_pre_align_turn_speed,
            self.args.stack_pre_align_turn_speed,
        )

        self.move_state.last_pre_align_error = yaw_error
        self.move_state.last_pre_align_time = now

        if abs(yaw_error) <= self.args.stack_pre_align_yaw_tolerance:
            self._publish_stop()
            self.move_state.pre_aligned_waypoint_index = self.move_state.waypoint_index
            self.move_state.last_pre_align_error = None
            self.move_state.last_pre_align_time = None
            self._publish_status(
                f"phase={self.phase.value}; target={target_name}; pre_align=complete; "
                f"target_yaw={target_yaw:.3f}; control_yaw={control_yaw:.3f}; "
                f"yaw_error={yaw_error:.3f}"
            )
            return True

        self._publish_command(0.0, angular_z)
        self._publish_status(
            f"phase={self.phase.value}; target={target_name}; pre_align=turning; "
            f"control_x={control_pose.x:.3f}; control_y={control_pose.y:.3f}; "
            f"goal=({target[0]:.3f},{target[1]:.3f}); "
            f"target_yaw={target_yaw:.3f}; control_yaw={control_yaw:.3f}; "
            f"yaw_error={yaw_error:.3f}; yaw_d={derivative:.3f}; angular={angular_z:.3f}"
        )
        return False

    def _speed_for_segment(self, target_name: str, active_axis: str) -> float:
        if self.move_state.avoidance_applied and self.move_state.waypoint_index <= 2:
            return self.args.peer_avoidance_speed
        final_waypoint_index = 0
        if self.move_state.route is not None:
            final_waypoint_index = len(self.move_state.route.waypoints) - 1
        if (
            target_name == self.args.stack_target
            and active_axis == "x"
            and self.move_state.waypoint_index == final_waypoint_index
        ):
            return self.args.stack_approach_speed
        return self.args.speed

    def _turn_speed_for_segment(self) -> float:
        if self.move_state.avoidance_applied and self.move_state.waypoint_index <= 2:
            return self.args.peer_avoidance_turn_speed
        return self.args.turn_speed

    def _distance_tolerance_for_segment(self, target_name: str, active_axis: str) -> float:
        if target_name == self.args.stack_target and active_axis == "y":
            return self.args.stack_lateral_tolerance
        return self.args.distance_tolerance

    def _allow_crossed_axis_target(self, target_name: str, active_axis: str) -> bool:
        if self.move_state.avoidance_applied and self.move_state.waypoint_index <= 2:
            return False
        return not (target_name == self.args.stack_target and active_axis == "y")

    def _use_axis_aligned_heading(self, target_name: str, active_axis: str) -> bool:
        return not (target_name == self.args.stack_target and active_axis == "x")

    def _use_reverse_motion_for_segment(self) -> bool:
        return self.move_state.reverse_avoidance_waypoint_index == self.move_state.waypoint_index

    def _pose_for_route_start(self, target_name: str) -> Pose2D:
        if target_name == self.args.stack_target and self.center_pose is not None:
            return self.center_pose
        return self.pose

    def _pose_for_segment(self, target_name: str, active_axis: str) -> Pose2D:
        if target_name == self.args.stack_target and active_axis == "y" and self.center_pose is not None:
            return self.center_pose
        return self.pose

    def _change_phase(self, phase: SequencePhase) -> None:
        self.phase = phase
        self.phase_started_at = self._now()
        self.move_state = MoveState(_target_for_phase(phase, self.args))
        self._publish_status(f"phase={self.phase.value}")

    def _elapsed(self) -> float:
        return self._now() - self.phase_started_at

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        now = self._now()
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

    def _publish_stop(self) -> None:
        self._publish_command(0.0, 0.0)

    def _publish_lift(self, position: float) -> None:
        command = JointState()
        command.name = [self.args.lift_joint_name]
        command.position = [position]
        self.lift_pub.publish(command)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_reservation_status(self) -> None:
        msg = String()
        cell = None
        next_cell = None
        if self.pose is not None:
            cell = _world_to_grid_cell(
                self.pose.x,
                self.pose.y,
                cell_size=self.args.grid_cell_size,
                origin_x=self.args.grid_origin_x,
                origin_y=self.args.grid_origin_y,
            )
            next_cell = _next_grid_cell(
                cell,
                self.pose,
                self.move_state.route,
                self.move_state.waypoint_index,
                cell_size=self.args.grid_cell_size,
                origin_x=self.args.grid_origin_x,
                origin_y=self.args.grid_origin_y,
            )
        msg.data = _format_reservation_status(
            robot_id=self.args.robot_id,
            phase=self.phase.value,
            target_name=_target_for_phase(self.phase, self.args),
            priority=self.args.reservation_priority,
            active=self.phase != SequencePhase.COMPLETE,
            cell=cell,
            next_cell=next_cell,
        )
        self.reservation_pub.publish(msg)

    def _try_publish_stop(self) -> None:
        try:
            self._publish_stop()
        except Exception as exc:  # ROS may invalidate the context before shutdown handling.
            self.get_logger().debug(f"Stop publish skipped during shutdown: {exc}")


def _target_for_phase(phase: SequencePhase, args: argparse.Namespace | None = None) -> str:
    if phase in {
        SequencePhase.MOVE_TO_STACK,
        SequencePhase.LIFT_UP,
        SequencePhase.WAIT_AFTER_LIFT,
    }:
        return getattr(args, "stack_target", "STACK_1")
    # SHELF_STORAGE 비활성화
    # if phase in {SequencePhase.MOVE_TO_SHELF_STORAGE, SequencePhase.STOP_AT_SHELF_STORAGE}:
    #     return getattr(args, "shelf_storage_target", "SHELF_STORAGE_1")
    if phase in {
        SequencePhase.MOVE_TO_UNLOAD_1,
        SequencePhase.SETTLE_AT_UNLOAD,
        SequencePhase.LIFT_DOWN,
        SequencePhase.BACK_OUT_FROM_UNLOAD,
    }:
        return getattr(args, "unload_target", "UNLOAD_1")
    if phase == SequencePhase.MOVE_TO_WAIT_1:
        return getattr(args, "wait_target", "WAIT_1")
    return ""


def _build_route_for_sequence_target(
    start: tuple[float, float],
    target_name: str,
    args: argparse.Namespace,
) -> AxisRoute:
    if target_name == args.stack_target and args.stack_y_align_x_offset > 0.0:
        target = PLACES[target_name]
        approach_direction = target[0] - start[0]
        if math.isclose(approach_direction, 0.0, abs_tol=1e-6):
            pre_align_x = target[0] + args.stack_y_align_x_offset
        else:
            pre_align_x = target[0] - math.copysign(
                args.stack_y_align_x_offset,
                approach_direction,
            )
        waypoints, axes = _deduplicate_route_steps(
            start,
            [
                ((pre_align_x, start[1]), "x"),
                ((pre_align_x, target[1]), "y"),
                (target, "x"),
            ],
        )
        if route_crosses_no_go(start, waypoints):
            detour = plan_axis_route_around_zones(
                start,
                target,
                axis_order=_axis_order_for_sequence_target(target_name, args),
            )
            if detour is not None:
                waypoints, axes = detour
        return AxisRoute(target_name=target_name, waypoints=waypoints, axes=axes)

    return build_axis_route(
        start,
        target_name,
        axis_order=_axis_order_for_sequence_target(target_name, args),
    )


def _expand_route_for_grid_reservation(
    start: tuple[float, float],
    route: AxisRoute,
    *,
    cell_size: float,
    origin_x: float,
    origin_y: float,
) -> AxisRoute:
    if cell_size <= 0.0:
        return route

    expanded_steps: list[tuple[tuple[float, float], str]] = []
    current_point = start
    current_cell = _world_to_grid_cell(
        current_point[0],
        current_point[1],
        cell_size=cell_size,
        origin_x=origin_x,
        origin_y=origin_y,
    )

    for target, active_axis in zip(route.waypoints, route.axes):
        target_cell = _world_to_grid_cell(
            target[0],
            target[1],
            cell_size=cell_size,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        while current_cell != target_cell:
            next_cell = _step_cell_toward(current_cell, target_cell, active_axis)
            if next_cell == current_cell:
                break
            waypoint = _cell_center(
                next_cell,
                cell_size=cell_size,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            expanded_steps.append((waypoint, active_axis))
            current_cell = next_cell
            current_point = waypoint
        expanded_steps.append((target, active_axis))
        current_point = target
        current_cell = target_cell

    waypoints, axes = _deduplicate_route_steps(start, expanded_steps)
    return AxisRoute(target_name=route.target_name, waypoints=waypoints, axes=axes)


def _axis_order_for_sequence_target(target_name: str, args: argparse.Namespace) -> str:
    if target_name == args.stack_target:
        return args.stack_axis_order
    if target_name == args.unload_target:
        return args.unload_axis_order
    if target_name == args.wait_target:
        return args.wait_axis_order
    return args.axis_order


def _deduplicate_route_steps(
    start: tuple[float, float],
    steps: list[tuple[tuple[float, float], str]],
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


def _axis_error(pose: Pose2D, target: tuple[float, float], active_axis: str) -> float:
    if active_axis == "x":
        return target[0] - pose.x
    return target[1] - pose.y


def _axis_target_yaw(axis_error: float, active_axis: str) -> float:
    if active_axis == "x":
        return 0.0 if axis_error >= 0.0 else math.pi
    return math.pi / 2.0 if axis_error >= 0.0 else -math.pi / 2.0


def _target_yaw_for_segment(
    pose: Pose2D,
    target: tuple[float, float],
    active_axis: str,
    *,
    axis_aligned_heading: bool,
) -> float:
    if axis_aligned_heading:
        return _axis_target_yaw(_axis_error(pose, target, active_axis), active_axis)
    return math.atan2(target[1] - pose.y, target[0] - pose.x)


def _should_pre_align_before_approach(
    args: argparse.Namespace,
    target_name: str,
    *,
    waypoint_index: int,
    waypoint_count: int,
    pre_aligned_waypoint_index: int | None,
) -> bool:
    if waypoint_count <= 0 or pre_aligned_waypoint_index == waypoint_index:
        return False
    if target_name == args.stack_target:
        return args.stack_pre_align
    if target_name == args.unload_target and waypoint_index == waypoint_count - 1:
        return args.unload_pre_align
    return False


def _is_peer_head_on_conflict(
    pose: Pose2D,
    peer_pose: Pose2D,
    target: tuple[float, float],
    active_axis: str,
    *,
    lane_tolerance: float,
    trigger_distance: float,
    path_margin: float,
    peer_yaw_tolerance: float,
) -> bool:
    direction = _direction_to_target(pose, target, active_axis)
    if direction == 0.0:
        return False
    if _lateral_distance(pose, peer_pose, active_axis) > lane_tolerance:
        return False
    peer_axis_distance = _axis_distance(pose, peer_pose, active_axis) * direction
    target_axis_distance = abs(_axis_point_value(target, active_axis) - _axis_value(pose, active_axis))
    if peer_axis_distance <= 0.0:
        return False
    if peer_axis_distance > target_axis_distance + max(0.0, path_margin):
        return False
    distance = math.hypot(pose.x - peer_pose.x, pose.y - peer_pose.y)
    if trigger_distance > 0.0 and distance > trigger_distance:
        return False

    peer_expected_yaw = _axis_target_yaw(-direction, active_axis)
    peer_yaw_error = abs(_normalize_angle(peer_expected_yaw - peer_pose.yaw))
    return peer_yaw_error <= peer_yaw_tolerance


def _build_left_bypass_route(
    start: tuple[float, float],
    peer_pose: Pose2D,
    route: AxisRoute,
    waypoint_index: int,
    active_axis: str,
    *,
    lateral_offset: float,
    pass_distance: float,
) -> AxisRoute | None:
    if waypoint_index >= len(route.waypoints):
        return None
    original_target = route.waypoints[waypoint_index]
    direction = _direction_from_start_to_target(start, original_target, active_axis)
    if direction == 0.0:
        return None

    offset = _left_offset(active_axis, direction, lateral_offset)
    pass_axis_value = _axis_value(peer_pose, active_axis) + direction * pass_distance
    side_point = _offset_point_on_current_axis(start, original_target, active_axis, offset)
    pass_side_point = _replace_axis_value(side_point, active_axis, pass_axis_value)
    pass_lane_point = _replace_lateral_value(
        pass_side_point,
        active_axis,
        _lateral_point_value(original_target, active_axis),
    )

    old_points = route.waypoints[waypoint_index:]
    old_axes = route.axes[waypoint_index:]
    waypoints, axes = _deduplicate_route_steps(
        start,
        [
            (side_point, _perpendicular_axis(active_axis)),
            (pass_side_point, active_axis),
            (pass_lane_point, _perpendicular_axis(active_axis)),
            *zip(old_points, old_axes),
        ],
    )
    return AxisRoute(target_name=route.target_name, waypoints=waypoints, axes=axes)


def _direction_to_target(pose: Pose2D, target: tuple[float, float], active_axis: str) -> float:
    return _direction_from_start_to_target((pose.x, pose.y), target, active_axis)


def _direction_from_start_to_target(
    start: tuple[float, float],
    target: tuple[float, float],
    active_axis: str,
) -> float:
    delta = _axis_point_value(target, active_axis) - _axis_point_value(start, active_axis)
    if math.isclose(delta, 0.0, abs_tol=1e-6):
        return 0.0
    return math.copysign(1.0, delta)


def _axis_value(pose: Pose2D, active_axis: str) -> float:
    return pose.x if active_axis == "x" else pose.y


def _axis_point_value(point: tuple[float, float], active_axis: str) -> float:
    return point[0] if active_axis == "x" else point[1]


def _lateral_point_value(point: tuple[float, float], active_axis: str) -> float:
    return point[1] if active_axis == "x" else point[0]


def _axis_distance(left: Pose2D, right: Pose2D, active_axis: str) -> float:
    if active_axis == "x":
        return right.x - left.x
    return right.y - left.y


def _lateral_distance(left: Pose2D, right: Pose2D, active_axis: str) -> float:
    if active_axis == "x":
        return abs(left.y - right.y)
    return abs(left.x - right.x)


def _perpendicular_axis(active_axis: str) -> str:
    return "y" if active_axis == "x" else "x"


def _left_offset(active_axis: str, direction: float, lateral_offset: float) -> float:
    if active_axis == "x":
        return math.copysign(abs(lateral_offset), direction)
    return math.copysign(abs(lateral_offset), -direction)


def _offset_point_on_current_axis(
    start: tuple[float, float],
    original_target: tuple[float, float],
    active_axis: str,
    offset: float,
) -> tuple[float, float]:
    if active_axis == "x":
        return (start[0], original_target[1] + offset)
    return (original_target[0] + offset, start[1])


def _replace_axis_value(
    point: tuple[float, float],
    active_axis: str,
    axis_value: float,
) -> tuple[float, float]:
    if active_axis == "x":
        return (axis_value, point[1])
    return (point[0], axis_value)


def _replace_lateral_value(
    point: tuple[float, float],
    active_axis: str,
    lateral_value: float,
) -> tuple[float, float]:
    if active_axis == "x":
        return (point[0], lateral_value)
    return (lateral_value, point[1])


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


def _quaternion_msg_from_yaw(yaw: float):
    orientation = Quaternion()
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)
    return orientation


def _is_world_to_robot_base(transform, base_frame: str) -> bool:
    if transform.header.frame_id != "world":
        return False
    child_frame = transform.child_frame_id
    if child_frame == base_frame or child_frame.endswith(f"/{base_frame}"):
        return True
    frame_tail = child_frame.rsplit("/", maxsplit=1)[-1]
    base_tail = base_frame.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"chassis", "base_link", "iw_hub_sensors", base_tail}


def _format_pose_value(pose: Pose2D | None, attr_name: str) -> str:
    if pose is None:
        return "nan"
    return f"{getattr(pose, attr_name):.3f}"


def _offset_pose(pose: Pose2D, *, offset_x: float, offset_y: float) -> Pose2D:
    if offset_x == 0.0 and offset_y == 0.0:
        return pose
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=pose.x + offset_x * cos_yaw - offset_y * sin_yaw,
        y=pose.y + offset_x * sin_yaw + offset_y * cos_yaw,
        yaw=pose.yaw,
    )


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _limit_command_acceleration(
    linear_x: float,
    angular_z: float,
    *,
    previous_linear_x: float | None,
    previous_angular_z: float | None,
    dt: float,
    linear_accel_limit: float,
    angular_accel_limit: float,
) -> tuple[float, float]:
    if previous_linear_x is None or previous_angular_z is None or dt <= 0.0:
        return linear_x, angular_z
    return (
        _slew_rate_limit(linear_x, previous_linear_x, linear_accel_limit, dt),
        _slew_rate_limit(angular_z, previous_angular_z, angular_accel_limit, dt),
    )


def _slew_rate_limit(target: float, previous: float, rate_limit: float, dt: float) -> float:
    if rate_limit <= 0.0:
        return target
    max_delta = rate_limit * dt
    return previous + _clamp(target - previous, -max_delta, max_delta)


def _format_reservation_status(
    *,
    robot_id: str,
    phase: str,
    target_name: str,
    priority: int,
    active: bool,
    cell: tuple[int, int] | None = None,
    next_cell: tuple[int, int] | None = None,
) -> str:
    active_text = "1" if active else "0"
    text = (
        f"robot={robot_id};phase={phase};target={target_name};"
        f"priority={priority};active={active_text}"
    )
    if cell is not None:
        text += f";cell={cell[0]},{cell[1]}"
    if next_cell is not None:
        text += f";next={next_cell[0]},{next_cell[1]}"
    return text


def _parse_reservation_status(text: str, *, received_at: float) -> PeerReservationState | None:
    fields = {}
    for part in text.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()
    if not {"robot", "phase", "target", "priority", "active"} <= fields.keys():
        return None
    try:
        priority = int(fields["priority"])
    except ValueError:
        return None
    cell = _parse_grid_cell(fields.get("cell", ""))
    next_cell = _parse_grid_cell(fields.get("next", ""))
    return PeerReservationState(
        robot_id=fields["robot"],
        phase=fields["phase"],
        target_name=fields["target"],
        priority=priority,
        active=fields["active"] in {"1", "true", "True", "active"},
        received_at=received_at,
        cell=cell,
        next_cell=next_cell,
    )


def _parse_grid_cell(text: str) -> tuple[int, int] | None:
    if not text:
        return None
    parts = text.split(",", 1)
    if len(parts) != 2:
        return None
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return None


def _world_to_grid_cell(
    x: float,
    y: float,
    *,
    cell_size: float,
    origin_x: float,
    origin_y: float,
) -> tuple[int, int]:
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    return (
        math.floor((x - origin_x) / cell_size),
        math.floor((y - origin_y) / cell_size),
    )


def _next_grid_cell(
    current_cell: tuple[int, int],
    pose: Pose2D,
    route: AxisRoute | None,
    waypoint_index: int,
    *,
    cell_size: float,
    origin_x: float,
    origin_y: float,
) -> tuple[int, int]:
    if route is None or waypoint_index >= len(route.waypoints):
        return current_cell
    target = route.waypoints[waypoint_index]
    active_axis = route.axes[waypoint_index]
    target_cell = _world_to_grid_cell(
        target[0],
        target[1],
        cell_size=cell_size,
        origin_x=origin_x,
        origin_y=origin_y,
    )
    if target_cell == current_cell:
        return current_cell

    current_x, current_y = current_cell
    if active_axis == "x":
        direction = 1 if target_cell[0] > current_x else -1
        return current_x + direction, current_y
    direction = 1 if target_cell[1] > current_y else -1
    return current_x, current_y + direction


def _step_cell_toward(
    current_cell: tuple[int, int],
    target_cell: tuple[int, int],
    active_axis: str,
) -> tuple[int, int]:
    current_x, current_y = current_cell
    target_x, target_y = target_cell
    if active_axis == "x" and current_x != target_x:
        return current_x + _sign(target_x - current_x), current_y
    if active_axis == "y" and current_y != target_y:
        return current_x, current_y + _sign(target_y - current_y)
    if current_x != target_x:
        return current_x + _sign(target_x - current_x), current_y
    if current_y != target_y:
        return current_x, current_y + _sign(target_y - current_y)
    return current_cell


def _is_grid_edge_swap(
    *,
    current_cell: tuple[int, int],
    next_cell: tuple[int, int],
    peer_cell: tuple[int, int] | None,
    peer_next_cell: tuple[int, int] | None,
) -> bool:
    return peer_next_cell == current_cell and peer_cell == next_cell


def _build_grid_reverse_right_bypass_route(
    start: tuple[float, float],
    route: AxisRoute,
    waypoint_index: int,
    *,
    current_cell: tuple[int, int],
    next_cell: tuple[int, int],
    peer_cell: tuple[int, int],
    peer_next_cell: tuple[int, int],
    cell_size: float,
    origin_x: float,
    origin_y: float,
) -> AxisRoute | None:
    if waypoint_index >= len(route.waypoints):
        return None
    active_axis = route.axes[waypoint_index]
    axis_direction = _grid_axis_direction(current_cell, next_cell, active_axis)
    if axis_direction == 0:
        return None

    blocked_cells = {peer_cell, peer_next_cell}
    for lateral_direction in _right_then_left_lateral_directions(active_axis, axis_direction):
        side_cell = _offset_grid_cell(current_cell, _perpendicular_axis(active_axis), lateral_direction)
        pass_cell = _offset_grid_cell(
            _offset_grid_cell(next_cell, active_axis, axis_direction),
            _perpendicular_axis(active_axis),
            lateral_direction,
        )
        rejoin_cell = _offset_grid_cell(next_cell, active_axis, axis_direction)
        if {side_cell, pass_cell, rejoin_cell} & blocked_cells:
            continue

        side_point = _cell_center(
            side_cell,
            cell_size=cell_size,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        pass_point = _cell_center(
            pass_cell,
            cell_size=cell_size,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        rejoin_point = _cell_center(
            rejoin_cell,
            cell_size=cell_size,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        old_points = route.waypoints[waypoint_index:]
        old_axes = route.axes[waypoint_index:]
        waypoints, axes = _deduplicate_route_steps(
            start,
            [
                (side_point, _perpendicular_axis(active_axis)),
                (pass_point, active_axis),
                (rejoin_point, _perpendicular_axis(active_axis)),
                *zip(old_points, old_axes),
            ],
        )
        return AxisRoute(target_name=route.target_name, waypoints=waypoints, axes=axes)
    return None


def _grid_axis_direction(
    current_cell: tuple[int, int],
    next_cell: tuple[int, int],
    active_axis: str,
) -> int:
    if active_axis == "x":
        return _sign(next_cell[0] - current_cell[0])
    return _sign(next_cell[1] - current_cell[1])


def _right_then_left_lateral_directions(active_axis: str, axis_direction: int) -> tuple[int, int]:
    if active_axis == "x":
        right = -axis_direction
    else:
        right = axis_direction
    return right, -right


def _offset_grid_cell(
    cell: tuple[int, int],
    active_axis: str,
    direction: int,
) -> tuple[int, int]:
    if active_axis == "x":
        return cell[0] + direction, cell[1]
    return cell[0], cell[1] + direction


def _cell_center(
    cell: tuple[int, int],
    *,
    cell_size: float,
    origin_x: float,
    origin_y: float,
) -> tuple[float, float]:
    return (
        origin_x + (cell[0] + 0.5) * cell_size,
        origin_y + (cell[1] + 0.5) * cell_size,
    )


def _sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _grid_reservation_conflict_reason(
    *,
    robot_id: str,
    priority: int,
    current_cell: tuple[int, int],
    next_cell: tuple[int, int],
    peer: PeerReservationState,
) -> str:
    peer_cell = peer.cell
    peer_next = peer.next_cell
    has_priority = _reservation_has_priority(robot_id, priority, peer.robot_id, peer.priority)

    if peer_next == current_cell and peer_cell == next_cell:
        if has_priority:
            return ""
        return f"peer={peer.robot_id} edge_swap={current_cell}->{next_cell}"
    if peer_cell == next_cell:
        return f"peer={peer.robot_id} occupies_next={next_cell}"
    if peer_next == next_cell and not has_priority:
        return f"peer={peer.robot_id} reserves_next={next_cell}"
    return ""


def _is_reserved_place(target_name: str, prefixes: str) -> bool:
    target = target_name.upper()
    return any(
        target == prefix or target.startswith(f"{prefix}_")
        for prefix in _parse_reserved_place_prefixes(prefixes)
    )


def _parse_reserved_place_prefixes(prefixes: str) -> list[str]:
    return [prefix.strip().upper() for prefix in prefixes.split(",") if prefix.strip()]


def _reservation_has_priority(
    robot_id: str,
    priority: int,
    peer_robot_id: str,
    peer_priority: int,
) -> bool:
    if priority != peer_priority:
        return priority < peer_priority
    return robot_id <= peer_robot_id


def _parse_args(
    args: Optional[list[str]] = None,
    *,
    default_robot_index: int = 1,
    default_stack_target: str = "STACK_1",
    default_shelf_storage_target: str = "SHELF_STORAGE_1",
    default_unload_target: str = "UNLOAD_1",
    default_wait_target: str = "WAIT_1",
    default_status_topic: str | None = None,
) -> argparse.Namespace:
    robot_id = default_robot_id(default_robot_index)
    default_peer_index = 2 if default_robot_index == 1 else 1
    default_peer_id = default_robot_id(default_peer_index)
    default_avoidance_role = "yield" if default_robot_index == 1 else "evade"
    default_reservation_priority = default_robot_index
    stack_targets = sorted(name for name in PLACES if name.startswith("STACK_"))
    shelf_storage_targets = sorted(name for name in PLACES if name.startswith("SHELF_STORAGE_"))
    unload_targets = sorted(name for name in PLACES if name.startswith("UNLOAD_"))
    parser = argparse.ArgumentParser(
        description="Run a robot through stack pickup, shelf stop, unload dropoff, and wait."
    )
    parser.add_argument("--robot-id", default=robot_id)
    parser.add_argument("--odom-topic", default=default_odom_topic(default_robot_index))
    parser.add_argument("--tf-topic", default=f"/{robot_id}/tf")
    parser.add_argument("--base-frame", default=default_base_frame(default_robot_index))
    parser.add_argument("--peer-robot-id", default=default_peer_id)
    parser.add_argument("--peer-odom-topic", default=default_odom_topic(default_peer_index))
    parser.add_argument("--peer-tf-topic", default=f"/{default_peer_id}/tf")
    parser.add_argument("--peer-base-frame", default=default_base_frame(default_peer_index))
    parser.add_argument(
        "--peer-pose-source",
        choices=["tf", "odom", "auto"],
        default="tf",
        help="Use this source for the other robot pose used by local avoidance.",
    )
    parser.add_argument(
        "--pose-source",
        choices=["tf", "odom", "auto"],
        default="tf",
        help="Use tf for world poses, odom for robot-local odometry, or auto with tf overriding odom.",
    )
    parser.add_argument("--cmd-vel-topic", default=default_cmd_vel_topic(default_robot_index))
    parser.add_argument("--lift-topic", default=f"/{robot_id}/lift_cmd")
    parser.add_argument(
        "--status-topic",
        default=default_status_topic
        or f"/smart_factory/robot{default_robot_index}_stack_sequence_status",
    )
    parser.add_argument(
        "--reservation-topic",
        default=f"/smart_factory/robot{default_robot_index}_stack_reservation",
    )
    parser.add_argument(
        "--peer-reservation-topic",
        default=f"/smart_factory/robot{default_peer_index}_stack_reservation",
    )
    parser.add_argument("--stack-target", choices=stack_targets, default=default_stack_target)
    parser.add_argument(
        "--shelf-storage-target",
        choices=shelf_storage_targets,
        default=default_shelf_storage_target,
    )
    parser.add_argument("--unload-target", choices=unload_targets, default=default_unload_target)
    parser.add_argument("--axis-order", choices=["xy", "yx"], default="xy")
    parser.add_argument(
        "--stack-axis-order",
        choices=["xy", "yx"],
        default="yx",
        help="Use yx to align left/right before the final straight stack approach.",
    )
    parser.add_argument(
        "--unload-axis-order",
        choices=["xy", "yx"],
        default="yx",
        help="Use yx so UNLOAD_2 can be approached without passing through UNLOAD_1.",
    )
    parser.add_argument(
        "--wait-axis-order",
        choices=["xy", "yx"],
        default="xy",
        help="Use xy so the final move to wait is along y — robot ends facing ±y (z=90 orientation).",
    )
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--stack-approach-speed", type=float, default=0.5)
    parser.add_argument(
        "--motion-controller",
        choices=["axis", "nav2"],
        default="axis",
        help="Use the existing direct axis controller or send each reserved waypoint to Nav2.",
    )
    parser.add_argument(
        "--nav2-action-name",
        default=f"/{robot_id}/navigate_to_pose",
        help="NavigateToPose action name for this robot's Nav2 instance.",
    )
    parser.add_argument(
        "--nav2-goal-frame",
        default="map",
        help="Frame id used for Nav2 goals. Use world only if your Nav2 stack is configured that way.",
    )
    parser.add_argument(
        "--nav2-expand-grid-steps",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Split long axis route segments into grid-cell-sized Nav2 goals so reservations stay local.",
    )
    parser.add_argument(
        "--linear-accel-limit",
        type=float,
        default=2.0,
        help="Maximum linear.x command change in m/s^2. Use 0 to disable command ramping.",
    )
    parser.add_argument(
        "--angular-accel-limit",
        type=float,
        default=3.0,
        help="Maximum angular.z command change in rad/s^2. Use 0 to disable command ramping.",
    )
    parser.add_argument(
        "--stack-y-align-x-offset",
        type=float,
        default=0.0,
        help="Run stack lateral y alignment this many meters before the final stack x target; 0 keeps y-first routing.",
    )
    parser.add_argument(
        "--stack-pre-align",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Stop and PD-align yaw before the final x-axis stack approach.",
    )
    parser.add_argument("--stack-pre-align-kp", type=float, default=1.2)
    parser.add_argument("--stack-pre-align-kd", type=float, default=0.12)
    parser.add_argument("--stack-pre-align-turn-speed", type=float, default=0.45)
    parser.add_argument("--stack-pre-align-yaw-tolerance", type=float, default=0.035)
    parser.add_argument(
        "--unload-pre-align",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Stop and PD-align yaw before the final unload approach.",
    )
    parser.add_argument(
        "--stack-lateral-tolerance",
        type=float,
        default=0.08,
        help="Y-axis tolerance used before the final stack approach.",
    )
    parser.add_argument("--turn-speed", type=float, default=1.5)
    parser.add_argument("--distance-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--enable-place-reservation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish/subscribe simple place reservations and wait when a higher-priority peer owns the same target.",
    )
    parser.add_argument(
        "--reservation-priority",
        type=int,
        default=default_reservation_priority,
        help="Lower number wins when two robots reserve the same target.",
    )
    parser.add_argument(
        "--peer-reservation-priority",
        type=int,
        default=default_peer_index,
        help="Priority expected for the peer robot; lower number wins safety-stop tie decisions.",
    )
    parser.add_argument(
        "--reserved-place-prefixes",
        default="STACK,SHELF_STORAGE,UNLOAD",
        help="Comma-separated target prefixes protected by place reservation.",
    )
    parser.add_argument(
        "--peer-reservation-timeout",
        type=float,
        default=2.0,
        help="Ignore peer reservation messages older than this many seconds.",
    )
    parser.add_argument(
        "--enable-grid-reservation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reserve the robot's current grid cell and next grid cell using peer reservation messages.",
    )
    parser.add_argument(
        "--grid-cell-size",
        type=float,
        default=1.0,
        help="World meters per reservation grid cell.",
    )
    parser.add_argument(
        "--grid-origin-x",
        type=float,
        default=0.0,
        help="World x coordinate for grid cell (0,0) origin.",
    )
    parser.add_argument(
        "--grid-origin-y",
        type=float,
        default=0.0,
        help="World y coordinate for grid cell (0,0) origin.",
    )
    parser.add_argument(
        "--enable-peer-safety-stop",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Pause this robot near a higher-priority peer until the peer is far enough away.",
    )
    parser.add_argument(
        "--peer-safety-stop-distance",
        type=float,
        default=3.0,
        help="Pause a lower-priority robot when the peer is at or below this distance.",
    )
    parser.add_argument(
        "--peer-safety-resume-distance",
        type=float,
        default=5.0,
        help="Resume a paused lower-priority robot after the peer is at or above this distance.",
    )
    parser.add_argument(
        "--avoidance-role",
        choices=["yield", "evade", "off"],
        default=default_avoidance_role,
        help="Local peer-pose avoidance behavior. Defaults to robot1 yielding and robot2 evading.",
    )
    parser.add_argument(
        "--peer-lane-tolerance",
        type=float,
        default=0.35,
        help="Lateral distance used to treat both robots as occupying the same lane.",
    )
    parser.add_argument(
        "--peer-avoidance-trigger-distance",
        type=float,
        default=2.5,
        help="Distance at which a head-on peer starts yielding or inserting a bypass route.",
    )
    parser.add_argument(
        "--peer-avoidance-path-margin",
        type=float,
        default=0.5,
        help="Only avoid a same-lane peer when it is on this robot's current segment plus this margin.",
    )
    parser.add_argument(
        "--peer-avoidance-lateral-offset",
        type=float,
        default=1.0,
        help="Side-step distance for the evading robot's temporary bypass lane.",
    )
    parser.add_argument(
        "--peer-avoidance-pass-distance",
        type=float,
        default=1.5,
        help="How far past the peer the evading robot rejoins the original lane.",
    )
    parser.add_argument(
        "--peer-avoidance-speed",
        type=float,
        default=4.0,
        help="Linear speed limit used on temporary grid/peer avoidance waypoints.",
    )
    parser.add_argument(
        "--peer-avoidance-turn-speed",
        type=float,
        default=3.0,
        help="Angular speed limit used on temporary grid/peer avoidance waypoints.",
    )
    parser.add_argument(
        "--peer-avoidance-reverse-distance",
        type=float,
        default=1.5,
        help="Minimum distance to keep reversing right after a head-on avoidance starts.",
    )
    parser.add_argument(
        "--peer-yaw-tolerance",
        type=float,
        default=0.75,
        help="Yaw tolerance for deciding that the peer is facing the opposite direction.",
    )
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--lift-joint-name", default="lift_joint")
    parser.add_argument("--lift-up-position", type=float, default=0.04)
    parser.add_argument("--lift-down-position", type=float, default=0.0)
    parser.add_argument("--wait-after-lift", type=float, default=1.0)
    parser.add_argument("--shelf-stop-duration", type=float, default=1.0)
    parser.add_argument("--unload-settle-duration", type=float, default=0.5)
    parser.add_argument("--lift-down-hold", type=float, default=2.0)
    parser.add_argument("--back-out-speed", type=float, default=1.0)
    parser.add_argument("--back-out-duration", type=float, default=4.0)
    parser.add_argument(
        "--wait-target",
        choices=["WAIT_1", "WAIT_2", "WAIT_3"],
        default=default_wait_target,
    )
    parser.add_argument(
        "--tracking-offset-x",
        type=float,
        default=-0.5,
        help="Body-frame x offset from odom/chassis to the point that should reach each target.",
    )
    parser.add_argument(
        "--tracking-offset-y",
        type=float,
        default=0.0,
        help="Body-frame y offset from odom/chassis to the point that should reach each target.",
    )
    parser.add_argument("--rate", type=float, default=10.0)
    return parser.parse_known_args(args)[0]


def main(
    args: Optional[list[str]] = None,
    *,
    default_robot_index: int = 1,
    default_stack_target: str = "STACK_1",
    default_shelf_storage_target: str = "SHELF_STORAGE_1",
    default_unload_target: str = "UNLOAD_1",
    default_wait_target: str = "WAIT_1",
    default_status_topic: str | None = None,
) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running robot1_stack_sequence.")

    parsed_args = _parse_args(
        args,
        default_robot_index=default_robot_index,
        default_stack_target=default_stack_target,
        default_shelf_storage_target=default_shelf_storage_target,
        default_unload_target=default_unload_target,
        default_wait_target=default_wait_target,
        default_status_topic=default_status_topic,
    )
    rclpy.init(args=args)
    node = Robot1StackSequence(parsed_args)
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
