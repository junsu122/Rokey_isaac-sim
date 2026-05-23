from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

from smart_factory.reservation import ReservationTable
from smart_factory.robot_defaults import default_cmd_vel_topic

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Node = object
    String = object


@dataclass(frozen=True)
class TimedCommand:
    robot_1_linear_x: float
    robot_2_linear_x: float
    duration: float
    label: str


def build_corridor_reservation_plan() -> tuple[list[str], list[str]]:
    reservations = ReservationTable()
    robot_1_path = ["A", "B", "C", "D"]
    robot_2_raw_path = ["D", "C", "B", "A"]

    reservations.reserve_route(robot_1_path, start_time=0)
    robot_2_start_time = _find_first_safe_start_time(reservations, robot_2_raw_path)
    robot_2_path = [robot_2_raw_path[0]] * robot_2_start_time + robot_2_raw_path
    reservations.reserve_route(robot_2_path, start_time=0)
    return robot_1_path, robot_2_path


def _find_first_safe_start_time(reservations: ReservationTable, route: list[str]) -> int:
    for start_time in range(20):
        if _route_is_safe(reservations, route, start_time):
            return start_time
    raise RuntimeError("Could not find a safe start time for the second robot")


def _route_is_safe(reservations: ReservationTable, route: list[str], start_time: int) -> bool:
    for offset, waypoint in enumerate(route):
        if not reservations.is_free(waypoint, start_time + offset):
            return False

    for offset, (source, target) in enumerate(zip(route, route[1:])):
        if not reservations.can_move(source, target, start_time + offset):
            return False

    return True


def build_timed_commands(
    robot_1_path: list[str],
    robot_2_path: list[str],
    *,
    speed: float,
    step_duration: float,
) -> list[TimedCommand]:
    max_steps = max(len(robot_1_path), len(robot_2_path)) - 1
    commands = []

    for index in range(max_steps):
        robot_1_current = _path_at(robot_1_path, index)
        robot_1_next = _path_at(robot_1_path, index + 1)
        robot_2_current = _path_at(robot_2_path, index)
        robot_2_next = _path_at(robot_2_path, index + 1)

        robot_1_moving = robot_1_current != robot_1_next
        robot_2_moving = robot_2_current != robot_2_next
        commands.append(
            TimedCommand(
                robot_1_linear_x=speed if robot_1_moving else 0.0,
                robot_2_linear_x=speed if robot_2_moving else 0.0,
                duration=step_duration,
                label=(
                    f"t={index}: r1 {robot_1_current}->{robot_1_next}, "
                    f"r2 {robot_2_current}->{robot_2_next}"
                ),
            )
        )

    commands.append(
        TimedCommand(
            robot_1_linear_x=0.0,
            robot_2_linear_x=0.0,
            duration=0.5,
            label="stop",
        )
    )
    return commands


def _path_at(path: list[str], index: int) -> str:
    if index >= len(path):
        return path[-1]
    return path[index]


class TwoRobotReservationDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_two_robot_reservation_demo")
        self.robot_1_topic = args.robot_1_cmd_vel
        self.robot_2_topic = args.robot_2_cmd_vel
        self.robot_1_pub = self.create_publisher(Twist, args.robot_1_cmd_vel, 10)
        self.robot_2_pub = self.create_publisher(Twist, args.robot_2_cmd_vel, 10)
        self.plan_pub = self.create_publisher(String, "/smart_factory/reservation_demo", 10)
        self.rate_hz = args.rate
        self.wait_for_subscribers_sec = args.wait_for_subscribers

        self.robot_1_path, self.robot_2_path = build_corridor_reservation_plan()
        self.commands = build_timed_commands(
            self.robot_1_path,
            self.robot_2_path,
            speed=args.speed,
            step_duration=args.step_duration,
        )
        self.get_logger().info(
            "Reservation plan: "
            f"robot_1={'->'.join(self.robot_1_path)}; "
            f"robot_2={'->'.join(self.robot_2_path)}"
        )

    def run(self) -> None:
        if not self._wait_for_robot_subscribers():
            self.get_logger().warning(
                "Starting demo without both robot cmd_vel subscribers. "
                "Commands may not reach Isaac controllers."
            )
        self._publish_plan()
        for command in self.commands:
            self.get_logger().info(command.label)
            self._publish_for(command)
        self._publish_twists(0.0, 0.0)
        self.get_logger().info("Two-robot reservation demo complete")

    def _wait_for_robot_subscribers(self) -> bool:
        deadline = time.monotonic() + self.wait_for_subscribers_sec
        while time.monotonic() < deadline and rclpy.ok():
            robot_1_count = self.robot_1_pub.get_subscription_count()
            robot_2_count = self.robot_2_pub.get_subscription_count()
            if robot_1_count > 0 and robot_2_count > 0:
                self.get_logger().info(
                    f"Matched cmd_vel subscribers: "
                    f"{self.robot_1_topic}={robot_1_count}, {self.robot_2_topic}={robot_2_count}"
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warning(
            f"Subscriber counts: "
            f"{self.robot_1_topic}={self.robot_1_pub.get_subscription_count()}, "
            f"{self.robot_2_topic}={self.robot_2_pub.get_subscription_count()}"
        )
        return False

    def _publish_for(self, command: TimedCommand) -> None:
        end_time = time.monotonic() + command.duration
        period = 1.0 / self.rate_hz
        while time.monotonic() < end_time and rclpy.ok():
            self._publish_plan()
            self._publish_twists(command.robot_1_linear_x, command.robot_2_linear_x)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _publish_twists(self, robot_1_linear_x: float, robot_2_linear_x: float) -> None:
        robot_1_command = Twist()
        robot_1_command.linear.x = robot_1_linear_x
        self.robot_1_pub.publish(robot_1_command)

        robot_2_command = Twist()
        robot_2_command.linear.x = robot_2_linear_x
        self.robot_2_pub.publish(robot_2_command)

    def _publish_plan(self) -> None:
        msg = String()
        msg.data = (
            f"robot_1: {' -> '.join(self.robot_1_path)}\n"
            f"robot_2: {' -> '.join(self.robot_2_path)}"
        )
        self.plan_pub.publish(msg)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a two-robot open-loop Isaac demo using a reservation table. "
            "Place both robots facing each other along a corridor before running."
        )
    )
    parser.add_argument("--robot-1-cmd-vel", default=default_cmd_vel_topic(1))
    parser.add_argument("--robot-2-cmd-vel", default=default_cmd_vel_topic(2))
    parser.add_argument("--speed", type=float, default=0.2, help="Forward speed in m/s.")
    parser.add_argument(
        "--step-duration",
        type=float,
        default=2.5,
        help="Seconds used for one reserved corridor step.",
    )
    parser.add_argument("--rate", type=float, default=10.0, help="Command publish rate in Hz.")
    parser.add_argument(
        "--wait-for-subscribers",
        type=float,
        default=3.0,
        help="Seconds to wait for Isaac cmd_vel subscribers before starting.",
    )
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running this demo.")

    parsed_args = _parse_args()
    rclpy.init(args=args)
    node = TwoRobotReservationDemo(parsed_args)
    try:
        node.run()
    except KeyboardInterrupt:
        node._publish_twists(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
