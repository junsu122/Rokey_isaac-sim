from smart_factory.two_robot_reservation_demo import (
    build_corridor_reservation_plan,
    build_timed_commands,
)


def test_corridor_reservation_makes_second_robot_wait():
    robot_1_path, robot_2_path = build_corridor_reservation_plan()

    assert robot_1_path == ["A", "B", "C", "D"]
    assert robot_2_path[:5] == ["D", "D", "D", "D", "D"]
    assert robot_2_path[-1] == "A"


def test_timed_commands_stop_waiting_robot():
    robot_1_path, robot_2_path = build_corridor_reservation_plan()

    commands = build_timed_commands(
        robot_1_path,
        robot_2_path,
        speed=0.2,
        step_duration=1.0,
    )

    assert commands[0].robot_1_linear_x == 0.2
    assert commands[0].robot_2_linear_x == 0.0
    assert commands[1].robot_1_linear_x == 0.2
    assert commands[1].robot_2_linear_x == 0.0
    assert commands[2].robot_1_linear_x == 0.2
    assert commands[2].robot_2_linear_x == 0.0
    assert commands[3].robot_1_linear_x == 0.0
    assert commands[3].robot_2_linear_x == 0.0
    assert commands[4].robot_1_linear_x == 0.0
    assert commands[4].robot_2_linear_x == 0.2
    assert commands[-1].robot_1_linear_x == 0.0
    assert commands[-1].robot_2_linear_x == 0.0
