from smart_factory.models import Pose2D
from smart_factory.reserved_axis_nav import (
    RobotAxisState,
    _frame_matches_robot,
    build_left_bypass_route,
    decide_reservations,
    detect_head_on_conflict,
    should_safety_stop,
)
from smart_factory.axis_nav_to_place import build_axis_route


def _robot(robot_id, target, route_target=None, index=0, completed=False):
    robot = RobotAxisState(robot_id=robot_id, target_name=target, pose=Pose2D(0.0, 0.0, 0.0))
    if route_target is not None:
        robot.route = build_axis_route((0.0, 0.0), route_target)
        robot.waypoint_index = index
    robot.completed = completed
    return robot


def test_decide_reservations_allows_different_free_segments():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=0),
        _robot("r2", "STACK", "STACK", index=0),
    )

    assert decision.robot_1_allowed
    assert decision.robot_2_allowed


def test_decide_reservations_reserves_same_target_for_first_robot():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=0),
        _robot("r2", "WAIT_1", "WAIT_1", index=0),
    )

    assert decision.robot_1_allowed
    assert not decision.robot_2_allowed
    assert "reserved" in decision.reason


def test_decide_reservations_allows_only_one_robot_on_reserved_lane():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=1),
        _robot("r2", "UNLOAD_1", "UNLOAD_1", index=1),
    )

    assert decision.robot_1_allowed
    assert not decision.robot_2_allowed
    assert "lane" in decision.reason


def test_decide_reservations_releases_lane_after_first_robot_completes():
    decision = decide_reservations(
        _robot("r1", "WAIT_1", "WAIT_1", index=1, completed=True),
        _robot("r2", "UNLOAD_1", "UNLOAD_1", index=1),
    )

    assert not decision.robot_1_allowed
    assert decision.robot_2_allowed


def test_should_safety_stop_when_robots_are_too_close():
    robot_1 = _robot("r1", "A1")
    robot_2 = _robot("r2", "A2")
    robot_2.pose = Pose2D(0.5, 0.0, 0.0)

    assert should_safety_stop(robot_1, robot_2, min_safe_distance=1.0)


def test_should_not_safety_stop_when_disabled():
    robot_1 = _robot("r1", "A1")
    robot_2 = _robot("r2", "A2")

    assert not should_safety_stop(robot_1, robot_2, min_safe_distance=0.0)


def test_detect_head_on_conflict_on_same_axis_lane():
    robot_1 = RobotAxisState("r1", "UNLOAD_2", pose=Pose2D(0.0, 0.0, 0.0))
    robot_1.route = build_axis_route((0.0, 0.0), "UNLOAD_2", axis_order="xy")
    robot_2 = RobotAxisState("r2", "STACK_1", pose=Pose2D(3.0, 0.0, 3.14))
    robot_2.route = build_axis_route((3.0, 0.0), "STACK_1", axis_order="xy")

    conflict = detect_head_on_conflict(
        robot_1,
        robot_2,
        lane_tolerance=0.2,
        trigger_distance=5.0,
    )

    assert conflict is not None
    assert conflict.axis == "x"
    assert conflict.robot_1_direction == 1.0
    assert conflict.robot_2_direction == -1.0


def test_detect_head_on_conflict_ignores_same_direction_traffic():
    robot_1 = RobotAxisState("r1", "UNLOAD_2", pose=Pose2D(0.0, 0.0, 0.0))
    robot_1.route = build_axis_route((0.0, 0.0), "UNLOAD_2", axis_order="xy")
    robot_2 = RobotAxisState("r2", "UNLOAD_3", pose=Pose2D(3.0, 0.0, 0.0))
    robot_2.route = build_axis_route((3.0, 0.0), "UNLOAD_3", axis_order="xy")

    conflict = detect_head_on_conflict(
        robot_1,
        robot_2,
        lane_tolerance=0.2,
        trigger_distance=5.0,
    )

    assert conflict is None


def test_build_left_bypass_route_inserts_left_side_dogleg():
    robot_1 = RobotAxisState("r1", "UNLOAD_2", pose=Pose2D(0.0, 0.0, 0.0))
    robot_1.route = build_axis_route((0.0, 0.0), "UNLOAD_2", axis_order="xy")
    robot_2 = RobotAxisState("r2", "STACK_1", pose=Pose2D(3.0, 0.0, 3.14))

    route = build_left_bypass_route(
        robot_1,
        robot_2,
        lateral_offset=1.0,
        pass_distance=1.5,
    )

    assert route is not None
    assert route.waypoints[:4] == [(0.0, 1.0), (4.5, 1.0), (4.5, 0.0), (13.0, 0.0)]
    assert route.axes[:4] == ["y", "x", "y", "x"]


def test_frame_matches_robot_accepts_isaac_transform_tree_frames():
    assert _frame_matches_robot("chassis", "iw_hub_02", "iw_hub_02/base_link")
    assert _frame_matches_robot(
        "/World/Robots/iw_hub_02/iw_hub_sensors",
        "iw_hub_02",
        "iw_hub_02/base_link",
    )
    assert not _frame_matches_robot(
        "/World/Robots/iw_hub_01/iw_hub_sensors/front_2d_lidar",
        "iw_hub_01",
        "iw_hub_01/base_link",
    )


def test_frame_matches_robot_rejects_unqualified_global_tf_frames():
    assert not _frame_matches_robot(
        "chassis",
        "iw_hub_02",
        "iw_hub_02/base_link",
        allow_unqualified_frame=False,
    )
    assert _frame_matches_robot(
        "iw_hub_02/base_link",
        "iw_hub_02",
        "iw_hub_02/base_link",
        allow_unqualified_frame=False,
    )
