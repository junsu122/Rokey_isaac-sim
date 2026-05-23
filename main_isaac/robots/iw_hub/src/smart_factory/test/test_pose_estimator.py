import math

from smart_factory.models import Pose2D
from smart_factory.pose_estimator import GridTransform, estimate_current_location, yaw_from_quaternion
from smart_factory.sample_world import make_sample_factory_map


def test_yaw_from_quaternion_reads_planar_rotation():
    half_angle = math.pi / 4.0

    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

    assert math.isclose(yaw, math.pi / 2.0)


def test_estimate_current_location_finds_nearest_waypoint():
    transform = GridTransform(origin_x=10.0, origin_y=-2.0, resolution=0.5)
    pose = Pose2D(x=10.52, y=-0.98, yaw=0.1)

    location = estimate_current_location(pose, make_sample_factory_map(), transform)

    assert location.grid_cell == (1, 2)
    assert location.nearest_waypoint == "CHARGE"
    assert location.nearest_distance < 0.03
