from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── IW Hub #1 미션 시퀀스 (smart_factory 패키지) ────────────────
        # STACK → 리프트 → UNLOAD → WAIT 전체 시퀀스 + 충돌 방지 내장
        Node(
            package='smart_factory',
            executable='robot1_stack_sequence',
            name='robot1_stack_sequence',
            output='screen',
        ),

        # ── IW Hub #2 미션 시퀀스 (smart_factory 패키지) ────────────────
        Node(
            package='smart_factory',
            executable='robot2_stack_sequence',
            name='robot2_stack_sequence',
            output='screen',
        ),

    ])
