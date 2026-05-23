from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cmd_vel_topic = "/iw_hub_01/cmd_vel"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value=default_cmd_vel_topic,
                description="Twist command topic subscribed by the Isaac Sim robot.",
            ),
            Node(
                package="smart_factory",
                executable="task_manager",
                name="smart_factory_task_manager",
                output="screen",
                parameters=[{"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")}],
            ),
        ]
    )
