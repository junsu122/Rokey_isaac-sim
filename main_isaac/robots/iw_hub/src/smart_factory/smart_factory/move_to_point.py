from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

from smart_factory.robot_defaults import default_cmd_vel_topic

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Twist = object
    Node = object


@dataclass(frozen=True)
class MotionStep:
    linear_x: float
    angular_z: float
    duration: float


def make_time_based_plan(
    target_x: float,
    target_y: float,
    linear_speed: float,
    angular_speed: float,
    start_yaw: float = 0.0,
) -> list[MotionStep]:
    if linear_speed <= 0.0:
        raise ValueError("linear_speed must be positive")
    if angular_speed <= 0.0:
        raise ValueError("angular_speed must be positive")

    distance = math.hypot(target_x, target_y)
    target_yaw = math.atan2(target_y, target_x) if distance > 0.0 else start_yaw
    yaw_error = normalize_angle(target_yaw - start_yaw)

    steps = []
    if not math.isclose(yaw_error, 0.0, abs_tol=1e-3):
        steps.append(
            MotionStep(
                linear_x=0.0,
                angular_z=math.copysign(angular_speed, yaw_error),
                duration=abs(yaw_error) / angular_speed,
            )
        )
    if not math.isclose(distance, 0.0, abs_tol=1e-3):
        steps.append(
            MotionStep(
                linear_x=linear_speed,
                angular_z=0.0,
                duration=distance / linear_speed,
            )
        )
    steps.append(MotionStep(linear_x=0.0, angular_z=0.0, duration=0.5))
    return steps


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MoveToPointNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smart_factory_move_to_point")
        self.publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.rate_hz = args.rate
        self.steps = make_time_based_plan(
            target_x=args.x,
            target_y=args.y,
            linear_speed=args.speed,
            angular_speed=args.turn_speed,
            start_yaw=args.start_yaw,
        )
        self.get_logger().info(
            f"Moving to ({args.x:.3f}, {args.y:.3f}) using {args.cmd_vel_topic}; "
            f"{len(self.steps)} motion steps"
        )

    def run(self) -> None:
        for step in self.steps:
            self._publish_step(step)
        self._publish_command(0.0, 0.0)
        self.get_logger().info("Move complete")

    def _publish_step(self, step: MotionStep) -> None:
        end_time = time.monotonic() + step.duration
        period = 1.0 / self.rate_hz
        while time.monotonic() < end_time and rclpy.ok():
            self._publish_command(step.linear_x, step.angular_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.publisher.publish(command)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move the Isaac robot toward a target point using open-loop cmd_vel timing."
    )
    parser.add_argument("--x", type=float, required=True, help="Target x in meters from the assumed start pose.")
    parser.add_argument("--y", type=float, required=True, help="Target y in meters from the assumed start pose.")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="Assumed starting yaw in radians.")
    parser.add_argument("--speed", type=float, default=0.2, help="Forward speed in m/s.")
    parser.add_argument("--turn-speed", type=float, default=0.5, help="Turn speed in rad/s.")
    parser.add_argument("--rate", type=float, default=10.0, help="Command publish rate in Hz.")
    parser.add_argument("--cmd-vel-topic", default=default_cmd_vel_topic(1), help="Twist command topic.")
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running move_to_point.")

    parsed_args = _parse_args()
    rclpy.init(args=args)
    node = MoveToPointNode(parsed_args)
    try:
        node.run()
    except KeyboardInterrupt:
        node._publish_command(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
