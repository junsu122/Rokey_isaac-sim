from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name: str, x: float, y: float, yaw: float, child_frame: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", str(yaw),
            "--frame-id", "map",
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _static_tf("iw_hub_01_map_to_odom", -8.0, -14.0, 1.5708, "iw_hub_01/odom"),
        _static_tf("iw_hub_02_map_to_odom", -10.0, -14.0, 1.5708, "iw_hub_02/odom"),
        Node(
            package="smart_factory",
            executable="odom_tf_broadcaster",
            name="iw_hub_odom_tf_broadcaster",
            output="screen",
        ),
    ])
