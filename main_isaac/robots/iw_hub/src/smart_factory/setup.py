from glob import glob
from setuptools import find_packages, setup

package_name = "smart_factory"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@example.com",
    description="Smart logistics sorting and stacking planner for Isaac Sim Nova Carter scenarios.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "factory_demo = smart_factory.demo:main",
            "axis_nav_to_place = smart_factory.axis_nav_to_place:main",
            "shelf_experiment = smart_factory.shelf_experiment:main",
            "task_manager = smart_factory.task_manager_node:main",
            "current_pose = smart_factory.current_pose_node:main",
            "move_to_point = smart_factory.move_to_point:main",
            "robot_axis_nav_to_xy = smart_factory.robot_axis_nav_to_xy:main",
            "reserved_axis_nav = smart_factory.reserved_axis_nav:main",
            "robot1_stack_sequence = smart_factory.robot1_stack_sequence:main",
            "robot2_stack_sequence = smart_factory.robot2_stack_sequence:main",
            "robot_pose_monitor = smart_factory.robot_pose_monitor:main",
            "two_robot_reservation_demo = smart_factory.two_robot_reservation_demo:main",
            "two_robot_reservation_follower = smart_factory.two_robot_reservation_follower:main",
        ],
    },
)
