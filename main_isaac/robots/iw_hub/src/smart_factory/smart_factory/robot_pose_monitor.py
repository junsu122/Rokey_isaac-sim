from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from smart_factory.models import Pose2D
from smart_factory.pose_estimator import yaw_from_quaternion
from smart_factory.robot_defaults import (
    default_base_frame,
    default_odom_topic,
    default_robot_id,
)

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:  # Allows algorithm tests without a sourced ROS2 environment.
    rclpy = None
    Odometry = object
    Node = object
    QoSProfile = object
    String = object
    TFMessage = object


@dataclass
class RobotPose:
    robot_id: str
    pose: Pose2D
    source: str


def pose_from_odom(robot_id: str, msg: Odometry, source: str) -> RobotPose:
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
    return RobotPose(robot_id=robot_id, pose=pose, source=source)


def pose_from_tf(robot_id: str, transform, source: str) -> RobotPose:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    pose = Pose2D(
        x=translation.x,
        y=translation.y,
        yaw=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    )
    return RobotPose(robot_id=robot_id, pose=pose, source=source)


def frame_matches_robot(child_frame_id: str, robot_id: str, base_frame: str) -> bool:
    if child_frame_id == base_frame or child_frame_id.endswith(f"/{base_frame}"):
        return True
    if child_frame_id in {"chassis", "base_link", "iw_hub_sensors"}:
        return True
    if robot_id not in child_frame_id:
        return False
    frame_tail = child_frame_id.rsplit("/", maxsplit=1)[-1]
    return frame_tail in {"base_link", "iw_hub_sensors"}


def format_robot_poses(poses: dict[str, RobotPose]) -> str:
    if not poses:
        return "no robot pose received yet"
    return "\n".join(
        (
            f"{robot_id}: x={robot_pose.pose.x:.3f}, y={robot_pose.pose.y:.3f}, "
            f"yaw={robot_pose.pose.yaw:.3f}, source={robot_pose.source}"
        )
        for robot_id, robot_pose in sorted(poses.items())
    )


class RobotPoseMonitor(Node):
    def __init__(self) -> None:
        super().__init__("smart_factory_robot_pose_monitor")
        self.declare_parameter("robot_1_id", default_robot_id(1))
        self.declare_parameter("robot_2_id", default_robot_id(2))
        self.declare_parameter("robot_1_odom_topic", default_odom_topic(1))
        self.declare_parameter("robot_2_odom_topic", default_odom_topic(2))
        self.declare_parameter("robot_1_tf_topic", f"/{default_robot_id(1)}/tf")
        self.declare_parameter("robot_2_tf_topic", f"/{default_robot_id(2)}/tf")
        self.declare_parameter("robot_1_base_frame", default_base_frame(1))
        self.declare_parameter("robot_2_base_frame", default_base_frame(2))

        self.robot_1_id = self._string_parameter("robot_1_id")
        self.robot_2_id = self._string_parameter("robot_2_id")
        self.robot_1_base_frame = self._string_parameter("robot_1_base_frame")
        self.robot_2_base_frame = self._string_parameter("robot_2_base_frame")
        robot_1_odom_topic = self._string_parameter("robot_1_odom_topic")
        robot_2_odom_topic = self._string_parameter("robot_2_odom_topic")
        robot_1_tf_topic = self._string_parameter("robot_1_tf_topic")
        robot_2_tf_topic = self._string_parameter("robot_2_tf_topic")

        self.poses: dict[str, RobotPose] = {}
        self.pose_pub = self.create_publisher(String, "/smart_factory/robot_poses", 10)
        isaac_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(
                Odometry,
                robot_1_odom_topic,
                lambda msg: self._store_pose(pose_from_odom(self.robot_1_id, msg, robot_1_odom_topic)),
                isaac_qos,
            ),
            self.create_subscription(
                Odometry,
                robot_2_odom_topic,
                lambda msg: self._store_pose(pose_from_odom(self.robot_2_id, msg, robot_2_odom_topic)),
                isaac_qos,
            ),
            self.create_subscription(TFMessage, robot_1_tf_topic, self._on_robot_1_tf, isaac_qos),
            self.create_subscription(TFMessage, robot_2_tf_topic, self._on_robot_2_tf, isaac_qos),
        ]
        self.create_timer(0.5, self._publish_poses)
        self.get_logger().info(
            "Monitoring robot poses from "
            f"{robot_1_odom_topic}, {robot_2_odom_topic}, {robot_1_tf_topic}, and {robot_2_tf_topic}"
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_robot_1_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(msg, self.robot_1_id, self.robot_1_base_frame)

    def _on_robot_2_tf(self, msg: TFMessage) -> None:
        self._on_robot_tf(msg, self.robot_2_id, self.robot_2_base_frame)

    def _on_robot_tf(self, msg: TFMessage, robot_id: str, base_frame: str) -> None:
        for transform in msg.transforms:
            child_frame_id = transform.child_frame_id
            if frame_matches_robot(child_frame_id, robot_id, base_frame):
                self._store_pose(pose_from_tf(robot_id, transform, "/tf"))
                return

    def _store_pose(self, robot_pose: RobotPose) -> None:
        existing = self.poses.get(robot_pose.robot_id)
        if existing is not None and existing.source == "/tf" and robot_pose.source != "/tf":
            return
        self.poses[robot_pose.robot_id] = robot_pose

    def _publish_poses(self) -> None:
        msg = String()
        msg.data = format_robot_poses(self.poses)
        self.pose_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available. Source ROS2 before running robot_pose_monitor.")

    rclpy.init(args=args)
    node = RobotPoseMonitor()
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
