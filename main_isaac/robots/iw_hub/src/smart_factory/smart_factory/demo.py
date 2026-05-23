from __future__ import annotations

from smart_factory.dispatcher import TaskDispatcher
from smart_factory.models import RobotState, ShelfTransportTask
from smart_factory.robot_defaults import default_robot_id
from smart_factory.sample_world import (
    make_sample_factory_map,
    make_sample_robots,
    make_sample_tasks,
    make_shelf_transport_world,
)
from smart_factory.shelf_transport_planner import ShelfTransportPlanner


def main() -> None:
    factory_map = make_sample_factory_map()
    robots = make_sample_robots()
    tasks = make_sample_tasks()
    dispatcher = TaskDispatcher(factory_map)

    plans = dispatcher.dispatch(robots, tasks)
    for plan in plans:
        route = " -> ".join(plan.waypoints)
        print(
            f"{plan.robot_id} handles {plan.task_id}: "
            f"start={plan.start_time}, pickup={plan.pickup_time}, "
            f"finish={plan.finish_time}, route={route}"
        )

    width, height, shelves, slots = make_shelf_transport_world()
    shelf_planner = ShelfTransportPlanner(width, height, shelves, slots)
    shelf_task = ShelfTransportTask(
        task_id="move_shelf_to_d1",
        robot_id=default_robot_id(1),
        target_slot="D-1",
        source_zone="A",
    )
    shelf_plan = shelf_planner.plan(
        robot=RobotState(robot_id=default_robot_id(1), waypoint="N1_2"),
        robot_cell=(1, 2),
        task=shelf_task,
    )
    approach = " -> ".join(str(cell) for cell in shelf_plan.approach_path)
    carry = " -> ".join(str(cell) for cell in shelf_plan.carry_path)
    print(
        f"{shelf_plan.robot_id} carries {shelf_plan.shelf_id} to {shelf_plan.target_slot}: "
        f"pickup={shelf_plan.pickup_time}, finish={shelf_plan.finish_time}"
    )
    print(f"  approach: {approach}")
    print(f"  carry: {carry}")


if __name__ == "__main__":
    main()
