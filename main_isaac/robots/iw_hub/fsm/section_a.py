"""
fsm/section_a.py
================
IW Hub Section A 스크립트 루트 FSM.

경로: spawn(-12.8, 13.0) yaw=0(east)
  GOTO_POD    → south y=9.0   (PodStack_01)
  LIFTING
  GOTO_CORRIDOR → north y=13.0
  GOTO_SLOT_X   → east  x=-3.5  (fast)
  GOTO_SLOT_Y   → south y=7.0   (slot A-01)
  LOWERING
  GOTO_RETURN_Y → north y=13.0  (fast)
  GOTO_HOME_X   → west  x=-12.7 (fast)
  → WAITING
"""
import math

ROUTE_TOL = 0.06
HOME_X    = -12.7   # spawn/home column x
HOME_Y    = 13.0    # spawn/corridor y
POD_Y     = 9.0     # PodStack_01 pickup y
DROP_X    = -3.5    # slot A-01 column x
DROP_Y    = 7.0     # slot A-01 row y


class SectionAFSM:
    def _run_section_a_fsm(self) -> None:
        cnt = self._get_signal_count()

        if self._sa_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] A_WAITING  signal={cnt}/{self._complete_needed} "
                      f"pos=({x:.2f},{y:.2f})")
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self._sa_state = "GOTO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] A_WAITING → GOTO_POD  y={POD_Y}")

        elif self._sa_state == "GOTO_POD":
            if self._drive_minimap_axis_with_heading(POD_Y, "y", -math.pi / 2, ROUTE_TOL):
                self._sa_state = "LIFTING"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_POD → LIFTING")

        elif self._sa_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_CORRIDOR"
                self._fsm_step = 0
                print(f"[{self.name}] LIFTING → GOTO_CORRIDOR  y={HOME_Y}")

        elif self._sa_state == "GOTO_CORRIDOR":
            if self._drive_minimap_axis_with_heading(HOME_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "GOTO_SLOT_X"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_CORRIDOR → GOTO_SLOT_X  x={DROP_X}")

        elif self._sa_state == "GOTO_SLOT_X":
            if self._drive_minimap_axis_with_heading(DROP_X, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_SLOT_Y"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_SLOT_X → GOTO_SLOT_Y  y={DROP_Y}")

        elif self._sa_state == "GOTO_SLOT_Y":
            if self._drive_minimap_axis_with_heading(DROP_Y, "y", -math.pi / 2, ROUTE_TOL):
                self._sa_state = "LOWERING"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_SLOT_Y → LOWERING")

        elif self._sa_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._drop_idx += 1
                self._sa_state = "GOTO_RETURN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] LOWERING → GOTO_RETURN_Y  y={HOME_Y}")

        elif self._sa_state == "GOTO_RETURN_Y":
            if self._drive_minimap_axis_with_heading(HOME_Y, "y", math.pi / 2, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_HOME_X"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_RETURN_Y → GOTO_HOME_X  x={HOME_X}")

        elif self._sa_state == "GOTO_HOME_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] GOTO_HOME_X → WAITING (cycle #{self._drop_idx})")
