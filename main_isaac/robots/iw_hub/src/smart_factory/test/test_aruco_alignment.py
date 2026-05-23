import math

import pytest

from smart_factory.aruco_alignment import ArucoAlignmentController, compose_pose
from smart_factory.models import (
    MarkerObservation,
    MarkerOffset,
    MarkerType,
    Pose2D,
)


def test_pickup_alignment_commands_error_reduction():
    controller = ArucoAlignmentController()
    command = controller.align_under_shelf(
        MarkerObservation(
            marker_id=101,
            marker_type=MarkerType.SHELF_BOTTOM,
            pose=Pose2D(x=0.05, y=-0.02, yaw=0.01),
            target_id="shelf_wait_1",
        )
    )

    assert not command.aligned
    assert command.linear_x < 0.0
    assert command.linear_y > 0.0
    assert command.angular_z < 0.0


def test_pickup_alignment_allows_lift_when_inside_tolerance():
    controller = ArucoAlignmentController()
    command = controller.align_under_shelf(
        MarkerObservation(
            marker_id=101,
            marker_type=MarkerType.SHELF_BOTTOM,
            pose=Pose2D(x=0.01, y=-0.01, yaw=0.01),
            target_id="shelf_wait_1",
        )
    )

    assert command.aligned
    assert command.reason == "ready_to_lift_up"


def test_dropoff_alignment_uses_wall_marker_offset():
    controller = ArucoAlignmentController()
    command = controller.align_to_drop_slot(
        robot_pose=Pose2D(x=8.05, y=4.0, yaw=0.0),
        wall_marker=MarkerObservation(
            marker_id=502,
            marker_type=MarkerType.WALL_SLOT,
            pose=Pose2D(x=7.0, y=4.0, yaw=0.0),
            target_id="A-2",
        ),
        slot_offset=MarkerOffset(
            target_id="A-2",
            marker_id=502,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
    )

    assert not command.aligned
    assert command.linear_x < 0.0


def test_dropoff_alignment_allows_lift_down_when_inside_tolerance():
    controller = ArucoAlignmentController()
    command = controller.align_to_drop_slot(
        robot_pose=Pose2D(x=8.01, y=3.99, yaw=0.01),
        wall_marker=MarkerObservation(
            marker_id=502,
            marker_type=MarkerType.WALL_SLOT,
            pose=Pose2D(x=7.0, y=4.0, yaw=0.0),
            target_id="A-2",
        ),
        slot_offset=MarkerOffset(
            target_id="A-2",
            marker_id=502,
            marker_type=MarkerType.WALL_SLOT,
            offset=Pose2D(x=1.0, y=0.0, yaw=0.0),
        ),
    )

    assert command.aligned
    assert command.reason == "ready_to_lift_down"


def test_compose_pose_rotates_slot_offset():
    composed = compose_pose(
        Pose2D(x=1.0, y=2.0, yaw=math.pi / 2.0),
        Pose2D(x=1.0, y=0.0, yaw=0.0),
    )

    assert composed.x == pytest.approx(1.0)
    assert composed.y == pytest.approx(3.0)
