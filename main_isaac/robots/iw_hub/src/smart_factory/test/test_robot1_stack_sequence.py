from smart_factory.models import Pose2D
from smart_factory.axis_nav_to_place import AxisRoute, compute_axis_nav_command
from smart_factory.robot1_stack_sequence import (
    PeerReservationState,
    SequencePhase,
    _build_grid_reverse_right_bypass_route,
    _build_left_bypass_route,
    _build_route_for_sequence_target,
    _format_reservation_status,
    _grid_reservation_conflict_reason,
    _is_grid_edge_swap,
    _is_peer_head_on_conflict,
    _is_reserved_place,
    _limit_command_acceleration,
    _next_grid_cell,
    _offset_pose,
    _parse_reservation_status,
    _parse_args,
    _reservation_has_priority,
    _should_pre_align_before_approach,
    _target_for_phase,
    _world_to_grid_cell,
)


def test_stack_sequence_defaults_match_robot1_topics():
    args = _parse_args([])

    assert args.odom_topic == "/iw_hub_01/odom"
    assert args.tf_topic == "/iw_hub_01/tf"
    assert args.peer_robot_id == "iw_hub_02"
    assert args.peer_odom_topic == "/iw_hub_02/odom"
    assert args.peer_tf_topic == "/iw_hub_02/tf"
    assert args.peer_base_frame == "iw_hub_02/base_link"
    assert args.peer_pose_source == "tf"
    assert args.pose_source == "tf"
    assert args.cmd_vel_topic == "/iw_hub_01/cmd_vel"
    assert args.lift_topic == "/iw_hub_01/lift_cmd"
    assert args.reservation_topic == "/smart_factory/robot1_stack_reservation"
    assert args.peer_reservation_topic == "/smart_factory/robot2_stack_reservation"
    assert args.lift_joint_name == "lift_joint"
    assert args.stack_target == "STACK_1"
    assert args.shelf_storage_target == "SHELF_STORAGE_1"
    assert args.unload_target == "UNLOAD_1"
    assert args.lift_up_position == 0.04
    assert args.lift_down_position == 0.0
    assert args.stack_axis_order == "yx"
    assert args.unload_axis_order == "yx"
    assert args.wait_axis_order == "yx"
    assert args.speed == 3.0
    assert args.stack_approach_speed == 0.5
    assert args.linear_accel_limit == 2.0
    assert args.angular_accel_limit == 3.0
    assert args.stack_y_align_x_offset == 0.0
    assert args.stack_pre_align
    assert args.stack_pre_align_kp == 1.2
    assert args.stack_pre_align_kd == 0.12
    assert args.stack_pre_align_turn_speed == 0.45
    assert args.stack_pre_align_yaw_tolerance == 0.035
    assert args.unload_pre_align
    assert args.stack_lateral_tolerance == 0.08
    assert args.turn_speed == 1.5
    assert args.enable_place_reservation
    assert args.reservation_priority == 1
    assert args.peer_reservation_priority == 2
    assert args.reserved_place_prefixes == "STACK,SHELF_STORAGE,UNLOAD"
    assert args.peer_reservation_timeout == 2.0
    assert args.enable_grid_reservation
    assert args.grid_cell_size == 1.0
    assert args.grid_origin_x == 0.0
    assert args.grid_origin_y == 0.0
    assert args.enable_peer_safety_stop
    assert args.peer_safety_stop_distance == 3.0
    assert args.peer_safety_resume_distance == 5.0
    assert args.avoidance_role == "yield"
    assert args.peer_lane_tolerance == 0.35
    assert args.peer_avoidance_trigger_distance == 2.5
    assert args.peer_avoidance_path_margin == 0.5
    assert args.peer_avoidance_lateral_offset == 1.0
    assert args.peer_avoidance_pass_distance == 1.5
    assert args.peer_avoidance_speed == 4.0
    assert args.peer_avoidance_turn_speed == 3.0
    assert args.peer_avoidance_reverse_distance == 1.2
    assert args.peer_yaw_tolerance == 0.75
    assert args.tracking_offset_x == -0.5
    assert args.tracking_offset_y == 0.0
    assert args.unload_settle_duration == 0.5
    assert args.lift_down_hold == 2.0
    assert args.back_out_speed == 1.0
    assert args.back_out_duration == 4.0
    assert args.wait_target == "WAIT_1"


def test_stack_sequence_can_default_to_robot2_targets():
    args = _parse_args(
        [],
        default_robot_index=2,
        default_stack_target="STACK_2",
        default_shelf_storage_target="SHELF_STORAGE_2",
        default_unload_target="UNLOAD_2",
        default_wait_target="WAIT_3",
        default_status_topic="/smart_factory/robot2_stack_sequence_status",
    )

    assert args.robot_id == "iw_hub_02"
    assert args.odom_topic == "/iw_hub_02/odom"
    assert args.tf_topic == "/iw_hub_02/tf"
    assert args.peer_robot_id == "iw_hub_01"
    assert args.peer_odom_topic == "/iw_hub_01/odom"
    assert args.peer_tf_topic == "/iw_hub_01/tf"
    assert args.peer_base_frame == "iw_hub_01/base_link"
    assert args.cmd_vel_topic == "/iw_hub_02/cmd_vel"
    assert args.lift_topic == "/iw_hub_02/lift_cmd"
    assert args.status_topic == "/smart_factory/robot2_stack_sequence_status"
    assert args.reservation_topic == "/smart_factory/robot2_stack_reservation"
    assert args.peer_reservation_topic == "/smart_factory/robot1_stack_reservation"
    assert args.stack_target == "STACK_2"
    assert args.shelf_storage_target == "SHELF_STORAGE_2"
    assert args.unload_target == "UNLOAD_2"
    assert args.wait_target == "WAIT_3"
    assert args.avoidance_role == "evade"
    assert args.reservation_priority == 2
    assert args.peer_reservation_priority == 1


def test_move_phases_target_requested_places():
    assert _target_for_phase(SequencePhase.MOVE_TO_STACK) == "STACK_1"
    assert _target_for_phase(SequencePhase.LIFT_UP) == "STACK_1"
    assert _target_for_phase(SequencePhase.WAIT_AFTER_LIFT) == "STACK_1"
    assert _target_for_phase(SequencePhase.MOVE_TO_SHELF_STORAGE) == "SHELF_STORAGE_1"
    assert _target_for_phase(SequencePhase.STOP_AT_SHELF_STORAGE) == "SHELF_STORAGE_1"
    assert _target_for_phase(SequencePhase.MOVE_TO_UNLOAD_1) == "UNLOAD_1"
    assert _target_for_phase(SequencePhase.SETTLE_AT_UNLOAD) == "UNLOAD_1"
    assert _target_for_phase(SequencePhase.LIFT_DOWN) == "UNLOAD_1"
    assert _target_for_phase(SequencePhase.BACK_OUT_FROM_UNLOAD) == "UNLOAD_1"
    assert _target_for_phase(SequencePhase.MOVE_TO_WAIT_1) == "WAIT_1"


def test_move_phases_use_configured_shelf_storage_target():
    args = _parse_args(["--shelf-storage-target", "SHELF_STORAGE_2"])

    assert _target_for_phase(SequencePhase.MOVE_TO_SHELF_STORAGE, args) == "SHELF_STORAGE_2"


def test_offset_pose_applies_body_frame_offset():
    pose = _offset_pose(Pose2D(x=1.0, y=2.0, yaw=0.0), offset_x=-0.2, offset_y=0.1)

    assert pose.x == 0.8
    assert pose.y == 2.1
    assert pose.yaw == 0.0


def test_stack_route_uses_y_first_before_final_x_approach():
    args = _parse_args([])
    route = _build_route_for_sequence_target((0.5, 4.0), "STACK_1", args)

    assert route.waypoints == [(0.5, 0.0), (-13.0, 0.0)]
    assert route.axes == ["y", "x"]


def test_stack_route_can_align_y_two_meters_before_final_x_approach():
    args = _parse_args(["--stack-y-align-x-offset", "2.0"])
    route = _build_route_for_sequence_target((0.5, 4.0), "STACK_1", args)

    assert route.waypoints == [(-11.0, 4.0), (-11.0, 0.0), (-13.0, 0.0)]
    assert route.axes == ["x", "y", "x"]


def test_unload_route_uses_y_first_to_avoid_unload_1_waypoint():
    args = _parse_args(
        [],
        default_robot_index=2,
        default_stack_target="STACK_2",
        default_unload_target="UNLOAD_2",
        default_wait_target="WAIT_3",
    )
    route = _build_route_for_sequence_target((0.0, -10.0), "UNLOAD_2", args)

    assert route.waypoints == [(0.0, 0.0), (13.0, 0.0)]
    assert route.axes == ["y", "x"]


def test_wait_route_uses_y_first_after_backing_out_from_unload():
    args = _parse_args([])
    route = _build_route_for_sequence_target((13.0, -3.0), "WAIT_1", args)

    assert route.waypoints == [(13.0, 15.0), (12.0, 15.0)]
    assert route.axes == ["y", "x"]


def test_pre_align_runs_on_each_stack_axis_and_final_unload_approach():
    args = _parse_args([])

    assert _should_pre_align_before_approach(
        args,
        "STACK_1",
        waypoint_index=0,
        waypoint_count=2,
        pre_aligned_waypoint_index=None,
    )
    assert _should_pre_align_before_approach(
        args,
        "STACK_1",
        waypoint_index=1,
        waypoint_count=2,
        pre_aligned_waypoint_index=0,
    )
    assert not _should_pre_align_before_approach(
        args,
        "STACK_1",
        waypoint_index=1,
        waypoint_count=2,
        pre_aligned_waypoint_index=1,
    )
    assert not _should_pre_align_before_approach(
        args,
        "UNLOAD_1",
        waypoint_index=0,
        waypoint_count=2,
        pre_aligned_waypoint_index=None,
    )
    assert _should_pre_align_before_approach(
        args,
        "UNLOAD_1",
        waypoint_index=1,
        waypoint_count=2,
        pre_aligned_waypoint_index=None,
    )
    assert not _should_pre_align_before_approach(
        args,
        "UNLOAD_1",
        waypoint_index=1,
        waypoint_count=2,
        pre_aligned_waypoint_index=1,
    )


def test_peer_head_on_conflict_uses_peer_pose_and_yaw():
    assert _is_peer_head_on_conflict(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(3.0, 0.0, 3.14159),
        (13.0, 0.0),
        "x",
        lane_tolerance=0.35,
        trigger_distance=5.0,
        path_margin=0.5,
        peer_yaw_tolerance=0.75,
    )


def test_peer_head_on_conflict_ignores_same_direction_peer():
    assert not _is_peer_head_on_conflict(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(3.0, 0.0, 0.0),
        (13.0, 0.0),
        "x",
        lane_tolerance=0.35,
        trigger_distance=5.0,
        path_margin=0.5,
        peer_yaw_tolerance=0.75,
    )


def test_peer_head_on_conflict_ignores_peer_beyond_current_segment():
    assert not _is_peer_head_on_conflict(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(3.0, 0.0, 3.14159),
        (1.0, 0.0),
        "x",
        lane_tolerance=0.35,
        trigger_distance=5.0,
        path_margin=0.5,
        peer_yaw_tolerance=0.75,
    )


def test_left_bypass_route_inserts_temporary_side_lane():
    args = _parse_args([])
    route = _build_route_for_sequence_target((0.0, 0.0), "UNLOAD_2", args)

    bypass = _build_left_bypass_route(
        (0.0, 0.0),
        Pose2D(3.0, 0.0, 3.14159),
        route,
        0,
        "x",
        lateral_offset=1.0,
        pass_distance=1.5,
    )

    assert bypass is not None
    assert bypass.waypoints[:4] == [(0.0, 1.0), (4.5, 1.0), (4.5, 0.0), (13.0, 0.0)]
    assert bypass.axes[:4] == ["y", "x", "y", "x"]


def test_command_acceleration_limit_smooths_abrupt_stop():
    linear_x, angular_z = _limit_command_acceleration(
        0.0,
        0.0,
        previous_linear_x=0.5,
        previous_angular_z=1.0,
        dt=0.1,
        linear_accel_limit=2.0,
        angular_accel_limit=3.0,
    )

    assert linear_x == 0.3
    assert angular_z == 0.7


def test_command_acceleration_limit_can_be_disabled():
    linear_x, angular_z = _limit_command_acceleration(
        0.0,
        0.0,
        previous_linear_x=0.5,
        previous_angular_z=1.0,
        dt=0.1,
        linear_accel_limit=0.0,
        angular_accel_limit=0.0,
    )

    assert linear_x == 0.0
    assert angular_z == 0.0


def test_reservation_status_round_trips():
    text = _format_reservation_status(
        robot_id="iw_hub_01",
        phase="move_to_stack",
        target_name="STACK_1",
        priority=1,
        active=True,
        cell=(0, 0),
        next_cell=(1, 0),
    )

    parsed = _parse_reservation_status(text, received_at=12.0)

    assert parsed == PeerReservationState(
        robot_id="iw_hub_01",
        phase="move_to_stack",
        target_name="STACK_1",
        priority=1,
        active=True,
        received_at=12.0,
        cell=(0, 0),
        next_cell=(1, 0),
    )


def test_place_reservation_uses_prefixes():
    assert _is_reserved_place("STACK_1", "STACK,SHELF_STORAGE,UNLOAD")
    assert _is_reserved_place("SHELF_STORAGE_2", "STACK,SHELF_STORAGE,UNLOAD")
    assert not _is_reserved_place("WAIT_1", "STACK,SHELF_STORAGE,UNLOAD")


def test_reservation_priority_uses_lower_number_then_robot_id():
    assert _reservation_has_priority("iw_hub_01", 1, "iw_hub_02", 2)
    assert not _reservation_has_priority("iw_hub_02", 2, "iw_hub_01", 1)
    assert _reservation_has_priority("iw_hub_01", 1, "iw_hub_02", 1)


def test_grid_reservation_yields_to_occupied_next_cell_even_with_higher_priority():
    peer = PeerReservationState(
        robot_id="iw_hub_02",
        phase="move_to_stack",
        target_name="STACK_2",
        priority=2,
        active=True,
        received_at=0.0,
        cell=(1, 0),
        next_cell=(1, 1),
    )

    assert _grid_reservation_conflict_reason(
        robot_id="iw_hub_01",
        priority=1,
        current_cell=(0, 0),
        next_cell=(1, 0),
        peer=peer,
    ) == "peer=iw_hub_02 occupies_next=(1, 0)"


def test_grid_reservation_uses_priority_for_same_next_cell():
    peer = PeerReservationState(
        robot_id="iw_hub_01",
        phase="move_to_stack",
        target_name="STACK_1",
        priority=1,
        active=True,
        received_at=0.0,
        cell=(0, 0),
        next_cell=(1, 0),
    )

    assert _grid_reservation_conflict_reason(
        robot_id="iw_hub_02",
        priority=2,
        current_cell=(2, 0),
        next_cell=(1, 0),
        peer=peer,
    ) == "peer=iw_hub_01 reserves_next=(1, 0)"


def test_grid_edge_swap_detects_head_on_cell_exchange():
    assert _is_grid_edge_swap(
        current_cell=(0, 0),
        next_cell=(1, 0),
        peer_cell=(1, 0),
        peer_next_cell=(0, 0),
    )


def test_grid_reverse_right_bypass_route_uses_right_side_first():
    route = AxisRoute(target_name="UNLOAD_2", waypoints=[(13.0, 0.0)], axes=["x"])

    bypass = _build_grid_reverse_right_bypass_route(
        (0.2, 0.2),
        route,
        0,
        current_cell=(0, 0),
        next_cell=(1, 0),
        peer_cell=(1, 0),
        peer_next_cell=(0, 0),
        cell_size=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    assert bypass is not None
    assert bypass.waypoints[:4] == [(0.5, -0.5), (2.5, -0.5), (2.5, 0.5), (13.0, 0.0)]
    assert bypass.axes[:4] == ["y", "x", "y", "x"]


def test_grid_reverse_right_bypass_route_tries_left_when_right_is_blocked():
    route = AxisRoute(target_name="UNLOAD_2", waypoints=[(13.0, 0.0)], axes=["x"])

    bypass = _build_grid_reverse_right_bypass_route(
        (0.2, 0.2),
        route,
        0,
        current_cell=(0, 0),
        next_cell=(1, 0),
        peer_cell=(0, -1),
        peer_next_cell=(1, 0),
        cell_size=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    assert bypass is not None
    assert bypass.waypoints[:3] == [(0.5, 1.5), (2.5, 1.5), (2.5, 0.5)]


def test_reverse_motion_axis_nav_turns_opposite_target_and_backs_up():
    command = compute_axis_nav_command(
        Pose2D(0.5, 0.2, 1.57079632679),
        (0.5, -0.5),
        segment_start=(0.5, 0.2),
        active_axis="y",
        max_linear_speed=4.0,
        max_angular_speed=2.0,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
        reverse_motion=True,
    )

    assert command.linear_x < 0.0
    assert abs(command.yaw_error) < 1e-6


def test_world_to_grid_cell_uses_floor_with_negative_coordinates():
    assert _world_to_grid_cell(0.2, 1.9, cell_size=1.0, origin_x=0.0, origin_y=0.0) == (0, 1)
    assert _world_to_grid_cell(-0.2, -1.1, cell_size=1.0, origin_x=0.0, origin_y=0.0) == (-1, -2)


def test_next_grid_cell_moves_one_cell_along_active_axis():
    args = _parse_args([])
    route = _build_route_for_sequence_target((0.2, 0.2), "UNLOAD_2", args)

    assert _next_grid_cell(
        (0, 0),
        Pose2D(0.2, 0.2, 0.0),
        route,
        0,
        cell_size=1.0,
        origin_x=0.0,
        origin_y=0.0,
    ) == (1, 0)
