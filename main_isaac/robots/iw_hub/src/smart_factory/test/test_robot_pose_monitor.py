from smart_factory.models import Pose2D
from smart_factory.robot_pose_monitor import RobotPose, format_robot_poses, frame_matches_robot


def test_format_robot_poses_reports_empty_state():
    assert format_robot_poses({}) == "no robot pose received yet"


def test_format_robot_poses_reports_each_robot():
    text = format_robot_poses(
        {
            "iw_hub_02": RobotPose(
                robot_id="iw_hub_02",
                pose=Pose2D(x=2.0, y=1.0, yaw=0.5),
                source="/tf",
            ),
            "iw_hub_01": RobotPose(
                robot_id="iw_hub_01",
                pose=Pose2D(x=0.0, y=-1.0, yaw=-0.25),
                source="/iw_hub_01/odom",
            ),
        }
    )

    assert "iw_hub_01: x=0.000, y=-1.000, yaw=-0.250, source=/iw_hub_01/odom" in text
    assert "iw_hub_02: x=2.000, y=1.000, yaw=0.500, source=/tf" in text


def test_frame_matches_robot_accepts_exact_and_isaac_prim_frames():
    assert frame_matches_robot("iw_hub_01/base_link", "iw_hub_01", "iw_hub_01/base_link")
    assert frame_matches_robot("chassis", "iw_hub_01", "iw_hub_01/base_link")
    assert frame_matches_robot(
        "/World/Robots/iw_hub_01/iw_hub_sensors",
        "iw_hub_01",
        "iw_hub_01/base_link",
    )
    assert not frame_matches_robot(
        "/World/Robots/iw_hub_02/iw_hub_sensors",
        "iw_hub_01",
        "iw_hub_01/base_link",
    )
