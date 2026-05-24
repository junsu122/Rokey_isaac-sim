import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


_NAV2_LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


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


def _cmd_vel_relay(name: str, input_topic: str, output_topic: str) -> Node:
    return Node(
        package="smart_factory",
        executable="cmd_vel_relay",
        name=name,
        output="screen",
        parameters=[{
            "input_topic": input_topic,
            "output_topic": output_topic,
        }],
    )


def _navigation_stack(namespace: str, params_file: str) -> list[Node]:
    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [params_file],
    }
    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
            **common,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            **common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            **common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            **common,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": _NAV2_LIFECYCLE_NODES,
            }],
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("smart_factory")
    config_dir = os.path.join(share_dir, "config")
    map_yaml = os.path.join(config_dir, "iw_hub_warehouse_map.yaml")
    hub_1_params = os.path.join(config_dir, "nav2_iw_hub_01.yaml")
    hub_2_params = os.path.join(config_dir, "nav2_iw_hub_02.yaml")

    actions = [
        _static_tf("iw_hub_01_map_to_odom", -8.0, -14.0, 1.5708, "iw_hub_01/odom"),
        _static_tf("iw_hub_02_map_to_odom", -10.0, -14.0, 1.5708, "iw_hub_02/odom"),
        Node(
            package="smart_factory",
            executable="odom_tf_broadcaster",
            name="iw_hub_odom_tf_broadcaster",
            output="screen",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "yaml_filename": map_yaml,
                "topic_name": "map",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
        *_navigation_stack("iw_nav_1", hub_1_params),
        *_navigation_stack("iw_nav_2", hub_2_params),
        _cmd_vel_relay("iw_nav_1_cmd_vel_relay", "/iw_nav_1/cmd_vel", "/iw_hub_01/cmd_vel"),
        _cmd_vel_relay("iw_nav_2_cmd_vel_relay", "/iw_nav_2/cmd_vel", "/iw_hub_02/cmd_vel"),
    ]
    return LaunchDescription(actions)
