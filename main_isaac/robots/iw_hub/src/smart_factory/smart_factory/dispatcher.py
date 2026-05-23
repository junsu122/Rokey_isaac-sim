from __future__ import annotations

from typing import Iterable, List

from smart_factory.graph import shortest_path
from smart_factory.models import CargoTask, FactoryMap, PlannedRoute, RobotState, RobotStatus, TaskStatus
from smart_factory.reservation import ReservationTable


class TaskDispatcher:
    def __init__(self, factory_map: FactoryMap, reservations: ReservationTable | None = None) -> None:
        self.factory_map = factory_map
        self.reservations = reservations or ReservationTable()

    def dispatch(self, robots: Iterable[RobotState], tasks: Iterable[CargoTask]) -> List[PlannedRoute]:
        available_tasks = sorted(
            [task for task in tasks if task.status == TaskStatus.WAITING],
            key=lambda task: (-task.priority, task.task_id),
        )
        idle_robots = sorted(
            [robot for robot in robots if robot.status == RobotStatus.IDLE],
            key=lambda robot: (robot.available_at, robot.robot_id),
        )

        planned_routes: List[PlannedRoute] = []
        for task in available_tasks:
            if not idle_robots:
                break

            robot = self._select_robot(idle_robots, task)
            idle_robots.remove(robot)

            to_pickup = shortest_path(self.factory_map, robot.waypoint, task.pickup)
            to_dropoff = shortest_path(self.factory_map, task.pickup, task.dropoff)
            raw_route = to_pickup + to_dropoff[1:]
            route = self.reservations.plan_with_waits(raw_route, robot.available_at)
            self.reservations.reserve_route(route, robot.available_at)

            pickup_time = robot.available_at + route.index(task.pickup)
            finish_time = robot.available_at + len(route) - 1
            planned = PlannedRoute(
                robot_id=robot.robot_id,
                task_id=task.task_id,
                waypoints=route,
                start_time=robot.available_at,
                pickup_time=pickup_time,
                finish_time=finish_time,
            )
            planned_routes.append(planned)

            task.status = TaskStatus.ASSIGNED
            task.assigned_robot = robot.robot_id
            robot.route = route
            robot.available_at = finish_time
            robot.status = RobotStatus.MOVING_TO_PICKUP

        return planned_routes

    def _select_robot(self, robots: List[RobotState], task: CargoTask) -> RobotState:
        return min(
            robots,
            key=lambda robot: (
                robot.available_at + self.factory_map.distance(robot.waypoint, task.pickup),
                robot.robot_id,
            ),
        )
