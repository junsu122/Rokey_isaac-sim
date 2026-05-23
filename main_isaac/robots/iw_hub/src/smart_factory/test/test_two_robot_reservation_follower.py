import math

from smart_factory.models import Pose2D
from smart_factory.two_robot_reservation_follower import (
    build_segments,
    compute_drive_command,
    compute_straight_command,
    interpolate_corridor_points,
    is_segment_timed_out,
    should_safety_stop,
    should_move,
)


def test_interpolate_corridor_points_splits_start_and_end():
    points = interpolate_corridor_points((0.0, 0.0), (3.0, 0.0))

    assert points["A"] == (0.0, 0.0)
    assert points["B"] == (1.0, 0.0)
    assert points["C"] == (2.0, 0.0)
    assert points["D"] == (3.0, 0.0)


def test_build_segments_uses_next_path_targets():
    segments = build_segments(["A", "B", "C"], ["C", "C", "B", "A"])

    assert segments[0].robot_1_target == "B"
    assert segments[0].robot_2_target == "C"
    assert segments[1].robot_1_target == "C"
    assert segments[1].robot_2_target == "B"
    assert segments[2].robot_1_target == "C"
    assert segments[2].robot_2_target == "A"


def test_compute_drive_command_rotates_before_driving():
    linear_x, angular_z, done = compute_drive_command(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        (0.0, 1.0),
        max_linear_speed=0.2,
        max_angular_speed=0.5,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
    )

    assert linear_x == 0.0
    assert angular_z == 0.5
    assert not done


def test_compute_drive_command_drives_when_facing_target():
    linear_x, angular_z, done = compute_drive_command(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        (1.0, 0.0),
        max_linear_speed=0.2,
        max_angular_speed=0.5,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
    )

    assert math.isclose(linear_x, 0.2)
    assert math.isclose(angular_z, 0.0)
    assert not done


def test_compute_drive_command_stops_at_target():
    linear_x, angular_z, done = compute_drive_command(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        (0.05, 0.0),
        max_linear_speed=0.2,
        max_angular_speed=0.5,
        distance_tolerance=0.1,
        yaw_tolerance=0.2,
    )

    assert linear_x == 0.0
    assert angular_z == 0.0
    assert done


def test_should_move_detects_wait_segments():
    assert should_move(["D", "D", "C"], 0) is False
    assert should_move(["D", "D", "C"], 1) is True


def test_compute_straight_command_waits_without_motion():
    linear_x, done = compute_straight_command(
        Pose2D(x=0.0, y=0.0, yaw=1.0),
        Pose2D(x=0.0, y=0.0, yaw=1.0),
        should_robot_move=False,
        step_distance=0.6,
        speed=0.2,
        distance_tolerance=0.1,
    )

    assert linear_x == 0.0
    assert done


def test_compute_straight_command_drives_until_step_distance():
    linear_x, done = compute_straight_command(
        Pose2D(x=0.0, y=0.0, yaw=1.0),
        Pose2D(x=0.2, y=0.0, yaw=1.0),
        should_robot_move=True,
        step_distance=0.6,
        speed=0.2,
        distance_tolerance=0.1,
    )

    assert linear_x == 0.2
    assert not done


def test_compute_straight_command_stops_after_step_distance():
    linear_x, done = compute_straight_command(
        Pose2D(x=0.0, y=0.0, yaw=1.0),
        Pose2D(x=0.51, y=0.0, yaw=1.0),
        should_robot_move=True,
        step_distance=0.6,
        speed=0.2,
        distance_tolerance=0.1,
    )

    assert linear_x == 0.0
    assert done


def test_is_segment_timed_out_handles_disabled_timeout():
    assert is_segment_timed_out(0.0, 0.0) is False
    assert is_segment_timed_out(None, 1.0) is False


def test_should_safety_stop_checks_robot_distance():
    assert should_safety_stop(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        Pose2D(x=0.3, y=0.0, yaw=0.0),
        min_safe_distance=0.5,
    )
    assert not should_safety_stop(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        Pose2D(x=0.8, y=0.0, yaw=0.0),
        min_safe_distance=0.5,
    )
    assert not should_safety_stop(
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        min_safe_distance=0.0,
    )
