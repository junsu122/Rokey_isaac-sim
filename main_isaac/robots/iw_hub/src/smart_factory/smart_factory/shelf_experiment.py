from __future__ import annotations

import argparse

from smart_factory.aruco_alignment import ArucoAlignmentController
from smart_factory.models import RobotState, ShelfTransportTask
from smart_factory.models import MarkerObservation, MarkerType, Pose2D
from smart_factory.sample_world import make_marker_offsets, make_shelf_transport_world
from smart_factory.robot_defaults import default_robot_id
from smart_factory.shelf_transport_planner import ShelfTransportPlanner


DEFAULT_ROBOT_ID = default_robot_id(1)
DEFAULT_ROBOT_CELL = (1, 2)


def main() -> None:
    args = _parse_args()
    width, height, shelves, slots = make_shelf_transport_world()

    target_slot = args.target or _ask_target_slot(slots)
    target_slot = target_slot.upper()
    source_zone = args.source_zone.upper()

    if target_slot not in slots:
        available = ", ".join(sorted(slots))
        raise SystemExit(f"Unknown slot '{target_slot}'. Available slots: {available}")
    if slots[target_slot].zone not in {"D", "E", "F"}:
        raise SystemExit("Target slot must be in destination zone D, E, or F")

    planner = ShelfTransportPlanner(width, height, shelves, slots)
    marker_offsets = make_marker_offsets()
    task = ShelfTransportTask(
        task_id=f"move_shelf_to_{target_slot.lower().replace('-', '_')}",
        robot_id=args.robot,
        target_slot=target_slot,
        source_zone=source_zone,
    )

    plan = planner.plan(
        robot=RobotState(robot_id=args.robot, waypoint=_cell_name(args.robot_x, args.robot_y)),
        robot_cell=(args.robot_x, args.robot_y),
        task=task,
    )

    print("\nShelf transport experiment")
    print(f"  robot       : {plan.robot_id}")
    print(f"  source zone : {source_zone}")
    print(f"  shelf       : {plan.shelf_id}")
    print(f"  target slot : {plan.target_slot}")
    print(f"  pickup time : {plan.pickup_time}")
    print(f"  finish time : {plan.finish_time}")
    print("\nApproach path")
    print("  " + _format_path(plan.approach_path))
    print("\nCarry path")
    print("  " + _format_path(plan.carry_path))

    alignment = ArucoAlignmentController()
    pickup_command = alignment.align_under_shelf(
        MarkerObservation(
            marker_id=101,
            marker_type=MarkerType.SHELF_BOTTOM,
            pose=Pose2D(x=args.pickup_dx, y=args.pickup_dy, yaw=args.pickup_dyaw),
            target_id=plan.shelf_id,
        )
    )
    print("\nPickup alignment from shelf-bottom ArUco")
    print("  " + _format_command(pickup_command))

    if target_slot in marker_offsets:
        wall_x, wall_y, wall_yaw = _resolve_wall_marker_pose(args, slots[target_slot], marker_offsets[target_slot])
        robot_pose_x, robot_pose_y, robot_pose_yaw = _resolve_robot_drop_pose(args, slots[target_slot])
        wall_marker = MarkerObservation(
            marker_id=marker_offsets[target_slot].marker_id,
            marker_type=MarkerType.WALL_SLOT,
            pose=Pose2D(x=wall_x, y=wall_y, yaw=wall_yaw),
            target_id=target_slot,
        )
        drop_command = alignment.align_to_drop_slot(
            robot_pose=Pose2D(x=robot_pose_x, y=robot_pose_y, yaw=robot_pose_yaw),
            wall_marker=wall_marker,
            slot_offset=marker_offsets[target_slot],
        )
        print("\nDropoff alignment from wall ArUco")
        print("  " + _format_command(drop_command))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move a shelf from source zone A/B/C to destination zone D/E/F."
    )
    parser.add_argument("--target", default="D-1", help="Destination shelf slot, for example D-1 or E-2.")
    parser.add_argument("--source-zone", default="A", help="Source shelf zone: A, B, or C.")
    parser.add_argument("--robot", default=DEFAULT_ROBOT_ID, help="Robot ID to assign.")
    parser.add_argument("--robot-x", type=int, default=DEFAULT_ROBOT_CELL[0], help="Robot start grid x.")
    parser.add_argument("--robot-y", type=int, default=DEFAULT_ROBOT_CELL[1], help="Robot start grid y.")
    parser.add_argument("--pickup-dx", type=float, default=0.04, help="Detected shelf marker x error in meters.")
    parser.add_argument("--pickup-dy", type=float, default=-0.02, help="Detected shelf marker y error in meters.")
    parser.add_argument("--pickup-dyaw", type=float, default=0.01, help="Detected shelf marker yaw error in radians.")
    parser.add_argument("--wall-x", type=float, help="Detected wall marker x pose.")
    parser.add_argument("--wall-y", type=float, help="Detected wall marker y pose.")
    parser.add_argument("--wall-yaw", type=float, default=0.0, help="Detected wall marker yaw.")
    parser.add_argument("--robot-pose-x", type=float, help="Robot pose x near drop slot.")
    parser.add_argument("--robot-pose-y", type=float, help="Robot pose y near drop slot.")
    parser.add_argument("--robot-pose-yaw", type=float, default=0.01, help="Robot pose yaw near drop slot.")
    return parser.parse_args()


def _ask_target_slot(slots: dict) -> str:
    empty_slots = [
        name
        for name, slot in sorted(slots.items())
        if slot.is_empty and slot.zone in {"D", "E", "F"}
    ]
    print("Empty target slots: " + ", ".join(empty_slots))
    return input("Move shelf to destination slot: ").strip()


def _cell_name(x: int, y: int) -> str:
    return f"N{x}_{y}"


def _format_path(path: list[tuple[int, int]]) -> str:
    return " -> ".join(f"({x},{y})" for x, y in path)


def _format_command(command) -> str:
    return (
        f"aligned={command.aligned}, reason={command.reason}, "
        f"vx={command.linear_x:.3f}, vy={command.linear_y:.3f}, wz={command.angular_z:.3f}"
    )


def _resolve_wall_marker_pose(args, target_slot, marker_offset) -> tuple[float, float, float]:
    wall_x = args.wall_x
    wall_y = args.wall_y
    if wall_x is None:
        wall_x = target_slot.center[0] - marker_offset.offset.x
    if wall_y is None:
        wall_y = target_slot.center[1] - marker_offset.offset.y
    return wall_x, wall_y, args.wall_yaw


def _resolve_robot_drop_pose(args, target_slot) -> tuple[float, float, float]:
    robot_x = args.robot_pose_x if args.robot_pose_x is not None else target_slot.center[0] + 0.04
    robot_y = args.robot_pose_y if args.robot_pose_y is not None else target_slot.center[1] + 0.01
    return robot_x, robot_y, args.robot_pose_yaw


if __name__ == "__main__":
    main()
