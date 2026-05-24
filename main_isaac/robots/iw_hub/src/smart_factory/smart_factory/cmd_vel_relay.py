from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        self.declare_parameter("input_topic", "/hub_1/cmd_vel")
        self.declare_parameter("output_topic", "/iw_hub_01/cmd_vel")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.publisher.publish, 10)
        self.get_logger().info(f"relaying {input_topic} -> {output_topic}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
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
