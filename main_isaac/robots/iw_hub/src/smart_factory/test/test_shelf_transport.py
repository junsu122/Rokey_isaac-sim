from smart_factory.footprint_reservation import FootprintReservationTable
from smart_factory.models import RobotState, ShelfGeometry, ShelfTransportTask
from smart_factory.occupancy_grid import build_shelf_occupancy_grid
from smart_factory.robot_defaults import default_robot_id
from smart_factory.sample_world import make_shelf_transport_world
from smart_factory.shelf_geometry import carried_footprint_cells, shelf_footprint_cells
from smart_factory.shelf_transport_planner import ShelfTransportPlanner


def test_shelf_transport_uses_orthogonal_grid_moves():
    width, height, shelves, slots = make_shelf_transport_world()
    planner = ShelfTransportPlanner(width, height, shelves, slots)

    plan = planner.plan(
        RobotState(robot_id=default_robot_id(1), waypoint="N1_2"),
        (1, 2),
        ShelfTransportTask(
            task_id="move_shelf_to_d1",
            robot_id=default_robot_id(1),
            target_slot="D-1",
            source_zone="A",
        ),
    )

    full_path = plan.approach_path + plan.carry_path
    for current, next_cell in zip(full_path, full_path[1:]):
        dx = abs(current[0] - next_cell[0])
        dy = abs(current[1] - next_cell[1])
        assert (dx, dy) in {(0, 0), (1, 0), (0, 1)}


def test_empty_robot_can_enter_only_target_shelf_center():
    width, height, shelves, _ = make_shelf_transport_world()
    target_shelf = next(shelf for shelf in shelves if shelf.shelf_id == "shelf_a_1")
    other_shelf = next(shelf for shelf in shelves if shelf.shelf_id == "shelf_b_1")

    grid = build_shelf_occupancy_grid(
        width,
        height,
        shelves,
        target_shelf_id=target_shelf.shelf_id,
    )

    assert grid.is_free(target_shelf.center)
    assert not grid.is_free(other_shelf.center)


def test_carried_shelf_footprint_does_not_overlap_other_shelves():
    width, height, shelves, slots = make_shelf_transport_world()
    planner = ShelfTransportPlanner(width, height, shelves, slots)
    plan = planner.plan(
        RobotState(robot_id=default_robot_id(1), waypoint="N1_2"),
        (1, 2),
        ShelfTransportTask(
            task_id="move_shelf_to_d1",
            robot_id=default_robot_id(1),
            target_slot="D-1",
            source_zone="A",
        ),
    )

    carried_shelf = next(shelf for shelf in shelves if shelf.shelf_id == plan.shelf_id)
    blocked_by_other_shelves = set()
    for shelf in shelves:
        if shelf.shelf_id != plan.shelf_id:
            blocked_by_other_shelves.update(shelf_footprint_cells(shelf))

    for center in plan.carry_path:
        assert carried_footprint_cells(center, carried_shelf.geometry).isdisjoint(
            blocked_by_other_shelves
        )


def test_carried_shelf_reserves_full_footprint():
    width, height, shelves, slots = make_shelf_transport_world()
    planner = ShelfTransportPlanner(width, height, shelves, slots)
    plan = planner.plan(
        RobotState(robot_id=default_robot_id(1), waypoint="N1_2"),
        (1, 2),
        ShelfTransportTask(
            task_id="move_shelf_to_d1",
            robot_id=default_robot_id(1),
            target_slot="D-1",
            source_zone="A",
        ),
    )

    carried_shelf = next(shelf for shelf in shelves if shelf.shelf_id == plan.shelf_id)
    first_carry_cells = carried_footprint_cells(plan.carry_path[0], carried_shelf.geometry)

    assert first_carry_cells.issubset(planner.reservations.cell_slots[plan.pickup_time])


def test_two_carried_shelves_wait_when_footprints_share_corridor():
    geometry = ShelfGeometry(width_cells=1, depth_cells=1)
    reservations = FootprintReservationTable()
    first_robot_path = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2)]
    second_robot_raw_path = [(5, 2), (4, 2), (3, 2), (2, 2), (1, 2)]

    reservations.reserve_path(first_robot_path, start_time=0, geometry=geometry)
    second_robot_planned_path = reservations.plan_with_waits(
        second_robot_raw_path,
        start_time=0,
        geometry=geometry,
    )
    reservations.reserve_path(second_robot_planned_path, start_time=0, geometry=geometry)

    assert _has_wait(second_robot_planned_path)

    first_timed = _timed_footprints(
        first_robot_path,
        0,
        geometry,
    )
    second_timed = _timed_footprints(
        second_robot_planned_path,
        0,
        geometry,
    )

    for time_step, first_cells in first_timed.items():
        assert first_cells.isdisjoint(second_timed.get(time_step, set()))


def _has_wait(path):
    return any(current == next_cell for current, next_cell in zip(path, path[1:]))


def _timed_footprints(path, start_time, geometry):
    return {
        start_time + offset: carried_footprint_cells(center, geometry)
        for offset, center in enumerate(path)
    }
