from __future__ import annotations

from typing import Optional

from smart_factory.models import Pose2D
from smart_factory.pose_estimator import GridTransform, estimate_current_location, yaw_from_quaternion
from smart_factory.sample_world import make_sample_factory_map

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Odometry = object
    Node = object
    String = object


class CurrentPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("smart_factory_current_pose")
        self.declare_parameter("odom_topic", "/iw_hub_01/odom")
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("grid_resolution", 1.0)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        origin_x = self.get_parameter("origin_x").get_parameter_value().double_value
        origin_y = self.get_parameter("origin_y").get_parameter_value().double_value
        resolution = self.get_parameter("grid_resolution").get_parameter_value().double_value

        self.factory_map = make_sample_factory_map()
        self.transform = GridTransform(origin_x=origin_x, origin_y=origin_y, resolution=resolution)
        self.pose_pub = self.create_publisher(String, "/smart_factory/current_pose", 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(f"Estimating current pose from {odom_topic}")

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = Pose2D(
            x=position.x,
            y=position.y,
            yaw=yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        location = estimate_current_location(pose, self.factory_map, self.transform)

        out = String()
        out.data = (
            f"x={location.pose.x:.3f}, y={location.pose.y:.3f}, yaw={location.pose.yaw:.3f}, "
            f"grid=({location.grid_cell[0]},{location.grid_cell[1]}), "
            f"nearest={location.nearest_waypoint}, distance={location.nearest_distance:.3f}"
        )
        self.pose_pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running current_pose.")

    rclpy.init(args=args)
    node = CurrentPoseNode()
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
