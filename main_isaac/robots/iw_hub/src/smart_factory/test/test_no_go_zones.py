import pytest

from smart_factory.no_go_zones import NoGoZone, plan_axis_route_around_zones, route_crosses_no_go
from smart_factory.robot_axis_nav_to_xy import build_xy_axis_route


def test_plan_axis_route_around_zone_inserts_detour():
    zone = NoGoZone("pillar", min_x=1.0, max_x=2.0, min_y=-0.4, max_y=0.4, clearance=0.0)

    waypoints, axes = plan_axis_route_around_zones(
        (0.0, 0.0),
        (3.0, 0.0),
        axis_order="xy",
        zones=[zone],
        detour_margin=0.5,
    )

    assert waypoints[-1] == (3.0, 0.0)
    assert len(waypoints) > 1
    assert not route_crosses_no_go((0.0, 0.0), waypoints, [zone])


def test_plan_axis_route_returns_none_when_direct_route_is_clear():
    zone = NoGoZone("far_wall", min_x=1.0, max_x=2.0, min_y=2.0, max_y=3.0, clearance=0.0)

    route = plan_axis_route_around_zones(
        (0.0, 0.0),
        (3.0, 0.0),
        axis_order="xy",
        zones=[zone],
    )

    assert route is None


def test_plan_axis_route_rejects_target_inside_no_go_zone():
    zone = NoGoZone("pillar", min_x=1.0, max_x=2.0, min_y=-0.4, max_y=0.4, clearance=0.0)

    with pytest.raises(ValueError):
        plan_axis_route_around_zones(
            (0.0, 0.0),
            (1.5, 0.0),
            axis_order="xy",
            zones=[zone],
        )


def test_xy_route_uses_normal_shape_when_no_zones_are_configured():
    route = build_xy_axis_route((0.0, 1.0), (3.0, 4.0), axis_order="xy")

    assert route.waypoints == [(3.0, 1.0), (3.0, 4.0)]
    assert route.axes == ["x", "y"]
