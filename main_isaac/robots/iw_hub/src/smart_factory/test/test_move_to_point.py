import math

import pytest

from smart_factory.move_to_point import make_time_based_plan, normalize_angle


def test_make_time_based_plan_drives_straight_to_forward_target():
    steps = make_time_based_plan(
        target_x=1.0,
        target_y=0.0,
        linear_speed=0.2,
        angular_speed=0.5,
    )

    assert len(steps) == 2
    assert steps[0].linear_x == 0.2
    assert steps[0].angular_z == 0.0
    assert math.isclose(steps[0].duration, 5.0)
    assert steps[-1].linear_x == 0.0


def test_make_time_based_plan_turns_then_drives():
    steps = make_time_based_plan(
        target_x=0.0,
        target_y=1.0,
        linear_speed=0.25,
        angular_speed=0.5,
    )

    assert len(steps) == 3
    assert steps[0].linear_x == 0.0
    assert steps[0].angular_z == 0.5
    assert math.isclose(steps[0].duration, math.pi)
    assert steps[1].linear_x == 0.25
    assert math.isclose(steps[1].duration, 4.0)


def test_make_time_based_plan_rejects_invalid_speed():
    with pytest.raises(ValueError):
        make_time_based_plan(1.0, 0.0, linear_speed=0.0, angular_speed=0.5)


def test_normalize_angle_uses_shortest_turn():
    assert math.isclose(normalize_angle(3.5), -2.7831853071795867)
