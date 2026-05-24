from launch import LaunchDescription
from launch_ros.actions import Node


def _sequence_node(
    robot_index: int,
    executable: str,
    *,
    avoidance_role: str,
    stack_target: str | None = None,
) -> Node:
    robot_id = f"iw_hub_{robot_index:02d}"
    nav2_namespace = f"iw_nav_{robot_index}"
    peer_index = 2 if robot_index == 1 else 1
    peer_id = f"iw_hub_{peer_index:02d}"
    map_origins = {
        1: (0.0, 0.0, 0.0),
        2: (0.0, 0.0, 0.0),
    }
    origin = map_origins[robot_index]
    peer_origin = map_origins[peer_index]
    return Node(
        package="smart_factory",
        executable=executable,
        name=f"{robot_id}_nav2_reserved_sequence",
        output="screen",
        arguments=[
            "--motion-controller", "nav2",
            "--nav2-action-name", f"/{nav2_namespace}/navigate_to_pose",
            "--nav2-goal-frame", "map",
            "--no-nav2-expand-grid-steps",
            "--prefer-straight-nav2-route",
            "--avoidance-role", avoidance_role,
            "--enable-grid-reservation",
            "--enable-place-reservation",
            "--no-enable-peer-safety-stop",
            "--robot-id", robot_id,
            "--peer-robot-id", peer_id,
            "--tf-topic", "/tf",
            "--peer-tf-topic", "/tf",
            "--pose-source", "map_odom",
            "--peer-pose-source", "map_odom",
            "--map-origin-x", str(origin[0]),
            "--map-origin-y", str(origin[1]),
            "--map-origin-yaw", str(origin[2]),
            "--peer-map-origin-x", str(peer_origin[0]),
            "--peer-map-origin-y", str(peer_origin[1]),
            "--peer-map-origin-yaw", str(peer_origin[2]),
            "--reservation-topic", f"/smart_factory/robot{robot_index}_stack_reservation",
            "--peer-reservation-topic", f"/smart_factory/robot{peer_index}_stack_reservation",
        ]
        + (["--stack-target", stack_target] if stack_target is not None else []),
    )


def generate_launch_description():
    return LaunchDescription([
        _sequence_node(1, "robot1_stack_sequence", avoidance_role="off", stack_target="STACK_3"),
        _sequence_node(2, "robot2_stack_sequence", avoidance_role="off"),
    ])
