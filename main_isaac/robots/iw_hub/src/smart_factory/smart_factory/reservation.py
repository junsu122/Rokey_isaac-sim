from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Set, Tuple


TimedNode = Tuple[int, str]
TimedEdge = Tuple[int, str, str]


@dataclass
class ReservationTable:
    node_slots: Dict[int, Set[str]] = field(default_factory=dict)
    edge_slots: Dict[int, Set[Tuple[str, str]]] = field(default_factory=dict)

    def is_free(self, waypoint: str, time_step: int) -> bool:
        return waypoint not in self.node_slots.get(time_step, set())

    def can_move(self, source: str, target: str, time_step: int) -> bool:
        occupied_edges = self.edge_slots.get(time_step, set())
        reverse_conflict = (target, source) in occupied_edges
        target_busy = target in self.node_slots.get(time_step + 1, set())
        return not reverse_conflict and not target_busy

    def reserve_route(self, route: List[str], start_time: int) -> None:
        for offset, waypoint in enumerate(route):
            self.node_slots.setdefault(start_time + offset, set()).add(waypoint)

        for offset, (source, target) in enumerate(zip(route, route[1:])):
            self.edge_slots.setdefault(start_time + offset, set()).add((source, target))

    def plan_with_waits(self, route: List[str], start_time: int) -> List[str]:
        if not route:
            return []

        planned = [route[0]]
        time_step = start_time
        index = 0

        while index < len(route) - 1:
            current = route[index]
            target = route[index + 1]

            if self.is_free(current, time_step) and self.can_move(current, target, time_step):
                planned.append(target)
                index += 1
            else:
                planned.append(current)
            time_step += 1

        return planned
