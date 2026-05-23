from smart_factory.dispatcher import TaskDispatcher
from smart_factory.sample_world import make_sample_factory_map, make_sample_robots, make_sample_tasks


def test_dispatch_assigns_waiting_tasks():
    dispatcher = TaskDispatcher(make_sample_factory_map())
    plans = dispatcher.dispatch(make_sample_robots(), make_sample_tasks())

    assert len(plans) == 2
    assert plans[0].waypoints[0] in {"CHARGE", "N1_4"}
    assert plans[0].finish_time >= plans[0].pickup_time


def test_reservation_adds_wait_when_needed():
    factory_map = make_sample_factory_map()
    robots = make_sample_robots()
    tasks = make_sample_tasks()[:2]
    tasks[0].pickup = "IN_A"
    tasks[0].dropoff = "SORT_RED"
    tasks[1].pickup = "IN_A"
    tasks[1].dropoff = "SORT_RED"

    dispatcher = TaskDispatcher(factory_map)
    plans = dispatcher.dispatch(robots, tasks)

    occupied = set()
    for plan in plans:
        for offset, waypoint in enumerate(plan.waypoints):
            slot = (plan.start_time + offset, waypoint)
            assert slot not in occupied
            occupied.add(slot)
