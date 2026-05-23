from __future__ import annotations

from typing import Optional

from smart_factory.dispatcher import TaskDispatcher
from smart_factory.robot_defaults import default_cmd_vel_topic
from smart_factory.sample_world import make_sample_factory_map, make_sample_robots, make_sample_tasks

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


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("smart_factory_task_manager")
        self.declare_parameter("cmd_vel_topic", default_cmd_vel_topic(1))
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.plan_pub = self.create_publisher(String, "/smart_factory/plan", 10)

        factory_map = make_sample_factory_map()
        self.robots = make_sample_robots()
        self.tasks = make_sample_tasks()
        self.dispatcher = TaskDispatcher(factory_map)
        self.plans = self.dispatcher.dispatch(self.robots, self.tasks)
        self.active_plan = self.plans[0] if self.plans else None
        self.route_index = 0

        self.timer = self.create_timer(1.0, self.on_timer)
        self.publish_plan_summary()

    def publish_plan_summary(self) -> None:
        if not self.plans:
            return
        msg = String()
        msg.data = "\n".join(
            f"{plan.robot_id}:{plan.task_id}:{'->'.join(plan.waypoints)}" for plan in self.plans
        )
        self.plan_pub.publish(msg)

    def on_timer(self) -> None:
        command = Twist()
        if self.active_plan is None:
            self.cmd_pub.publish(command)
            return

        if self.route_index < len(self.active_plan.waypoints) - 1:
            command.linear.x = 0.25
            command.angular.z = 0.0
            self.route_index += 1
        else:
            command.linear.x = 0.0
            command.angular.z = 0.0

        self.cmd_pub.publish(command)
        self.publish_plan_summary()


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running task_manager.")

    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
