from launch import LaunchDescription
from launch_ros.actions import Node


def _sequence_node(robot_index: int, executable: str, *, avoidance_role: str) -> Node:
    robot_id = f"iw_hub_{robot_index:02d}"
    nav2_namespace = f"iw_nav_{robot_index}"
    peer_index = 2 if robot_index == 1 else 1
    peer_id = f"iw_hub_{peer_index:02d}"
    return Node(
        package="smart_factory",
        executable=executable,
        name=f"{robot_id}_nav2_reserved_sequence",
        output="screen",
        arguments=[
            "--motion-controller", "nav2",
            "--nav2-action-name", f"/{nav2_namespace}/navigate_to_pose",
            "--nav2-goal-frame", "map",
            "--nav2-expand-grid-steps",
            "--avoidance-role", avoidance_role,
            "--enable-grid-reservation",
            "--enable-place-reservation",
            "--no-enable-peer-safety-stop",
            "--robot-id", robot_id,
            "--peer-robot-id", peer_id,
            "--tf-topic", "/tf",
            "--peer-tf-topic", "/tf",
            "--reservation-topic", f"/smart_factory/robot{robot_index}_stack_reservation",
            "--peer-reservation-topic", f"/smart_factory/robot{peer_index}_stack_reservation",
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _sequence_node(1, "robot1_stack_sequence", avoidance_role="off"),
        _sequence_node(2, "robot2_stack_sequence", avoidance_role="off"),
    ])
