from __future__ import annotations

import math

from smart_factory.models import (
    AlignmentCommand,
    AlignmentTolerance,
    MarkerObservation,
    MarkerOffset,
    MarkerType,
    Pose2D,
)


class ArucoAlignmentController:
    def __init__(
        self,
        tolerance: AlignmentTolerance | None = None,
        linear_gain: float = 0.8,
        angular_gain: float = 1.2,
        max_linear_speed: float = 0.12,
        max_angular_speed: float = 0.35,
    ) -> None:
        self.tolerance = tolerance or AlignmentTolerance()
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def align_under_shelf(self, shelf_marker: MarkerObservation) -> AlignmentCommand:
        if shelf_marker.marker_type != MarkerType.SHELF_BOTTOM:
            raise ValueError("Pickup alignment requires a shelf bottom marker")

        error = shelf_marker.pose
        return self._command_from_error(error, aligned_reason="ready_to_lift_up")

    def align_to_drop_slot(
        self,
        robot_pose: Pose2D,
        wall_marker: MarkerObservation,
        slot_offset: MarkerOffset,
    ) -> AlignmentCommand:
        if wall_marker.marker_type != MarkerType.WALL_SLOT:
            raise ValueError("Dropoff alignment requires a wall slot marker")
        if slot_offset.marker_type != MarkerType.WALL_SLOT:
            raise ValueError("Slot offset must be based on a wall slot marker")
        if wall_marker.marker_id != slot_offset.marker_id:
            raise ValueError("Wall marker observation does not match slot offset")

        target_pose = compose_pose(wall_marker.pose, slot_offset.offset)
        error = robot_pose.relative_to(target_pose)
        return self._command_from_error(error, aligned_reason="ready_to_lift_down")

    def _command_from_error(self, error: Pose2D, aligned_reason: str) -> AlignmentCommand:
        yaw_error = normalize_angle(error.yaw)
        aligned = (
            abs(error.x) <= self.tolerance.max_x_error
            and abs(error.y) <= self.tolerance.max_y_error
            and abs(yaw_error) <= self.tolerance.max_yaw_error
        )
        if aligned:
            return AlignmentCommand(
                linear_x=0.0,
                linear_y=0.0,
                angular_z=0.0,
                aligned=True,
                reason=aligned_reason,
            )

        # Command signs are negative because the controller reduces measured pose error.
        return AlignmentCommand(
            linear_x=_clamp(-error.x * self.linear_gain, self.max_linear_speed),
            linear_y=_clamp(-error.y * self.linear_gain, self.max_linear_speed),
            angular_z=_clamp(-yaw_error * self.angular_gain, self.max_angular_speed),
            aligned=False,
            reason="aligning",
        )


def compose_pose(reference: Pose2D, offset: Pose2D) -> Pose2D:
    cos_yaw = math.cos(reference.yaw)
    sin_yaw = math.sin(reference.yaw)
    x = reference.x + offset.x * cos_yaw - offset.y * sin_yaw
    y = reference.y + offset.x * sin_yaw + offset.y * cos_yaw
    return Pose2D(x=x, y=y, yaw=normalize_angle(reference.yaw + offset.yaw))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
