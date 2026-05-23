from smart_factory.robot_axis_nav_to_xy import build_xy_axis_route


def test_build_xy_axis_route_uses_xy_segments():
    route = build_xy_axis_route((0.0, 1.0), (3.0, 4.0), axis_order="xy")

    assert route.waypoints == [(3.0, 1.0), (3.0, 4.0)]
    assert route.axes == ["x", "y"]


def test_build_xy_axis_route_uses_yx_segments():
    route = build_xy_axis_route((0.0, 1.0), (3.0, 4.0), axis_order="yx")

    assert route.waypoints == [(0.0, 4.0), (3.0, 4.0)]
    assert route.axes == ["y", "x"]


def test_build_xy_axis_route_removes_zero_length_segment():
    route = build_xy_axis_route((0.0, 1.0), (0.0, 4.0), axis_order="xy")

    assert route.waypoints == [(0.0, 4.0)]
    assert route.axes == ["y"]
