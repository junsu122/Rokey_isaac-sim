from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── IW Hub #1 이동 노드 ─────────────────────────────────────────
        Node(
            package='iw_hub_movement',
            executable='axis_nav',
            name='axis_nav_01',
            parameters=[{
                'robot_name': 'iw_hub_01',
                'waypoint':   'WAIT_1',    # ★ 시작 웨이포인트
                'axis_order': 'xy',
            }],
            output='screen',
        ),

        # ── IW Hub #2 이동 노드 ─────────────────────────────────────────
        Node(
            package='iw_hub_movement',
            executable='axis_nav',
            name='axis_nav_02',
            parameters=[{
                'robot_name': 'iw_hub_02',
                'waypoint':   'WAIT_2',    # ★ 시작 웨이포인트
                'axis_order': 'xy',
            }],
            output='screen',
        ),

    ])
