from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("iw_hub_odom_tf_broadcaster")
        self.declare_parameter("robot_ids", ["iw_hub_01", "iw_hub_02"])
        robot_ids = list(self.get_parameter("robot_ids").value)
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(
                Odometry,
                f"/{robot_id}/odom",
                lambda msg, expected_robot_id=robot_id: self._on_odom(msg, expected_robot_id),
                qos,
            )
            for robot_id in robot_ids
        ]
        self.get_logger().info(
            "publishing odom TF for " + ", ".join(f"/{robot_id}/odom" for robot_id in robot_ids)
        )

    def _on_odom(self, msg: Odometry, robot_id: str) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = f"{robot_id}/odom"
        transform.child_frame_id = f"{robot_id}/base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
