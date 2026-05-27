"""
fsm/section_c.py
================
IW Hub Section C 스크립트 루트 FSM.

경로: spawn(-9.7, -9.6) yaw=0(east)
  GOTO_POD    → north y=-8.9  (PodStack_03)
  LIFTING
  GOTO_SLOT_X → east  x=-3.5  (fast)
  GOTO_SLOT_Y → south y=-13.0 (slot C-01)
  LOWERING
  GOTO_RETURN_Y → north y=-9.6 (fast)
  GOTO_HOME_X   → west  x=-9.7 (fast)
  → WAITING
"""
import math

ROUTE_TOL = 0.06
HOME_X    = -9.7    # spawn/home column x
HOME_Y    = -9.6    # spawn/home y
POD_Y     = -8.9    # PodStack_03 pickup y
DROP_X    = -3.5    # slot C-01 column x
SLOT_Y    = -13.0   # slot C-01 row y


class SectionCFSM:
    def _run_section_c_fsm(self) -> None:
        cnt = self._get_signal_count()

        if self._sa_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] C_WAITING  signal={cnt}/{self._complete_needed} "
                      f"pos=({x:.2f},{y:.2f})")
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self._sa_state = "GOTO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] C_WAITING → GOTO_POD  y={POD_Y}")

        elif self._sa_state == "GOTO_POD":
            if self._drive_minimap_axis_with_heading(POD_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "LIFTING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_POD → LIFTING")

        elif self._sa_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_SLOT_X"
                self._fsm_step = 0
                print(f"[{self.name}] C_LIFTING → GOTO_SLOT_X  x={DROP_X}")

        elif self._sa_state == "GOTO_SLOT_X":
            if self._drive_minimap_axis_with_heading(DROP_X, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_SLOT_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_SLOT_X → GOTO_SLOT_Y  y={SLOT_Y}")

        elif self._sa_state == "GOTO_SLOT_Y":
            if self._drive_minimap_axis_with_heading(SLOT_Y, "y", -math.pi / 2, ROUTE_TOL):
                self._sa_state = "LOWERING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_SLOT_Y → LOWERING")

        elif self._sa_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._drop_idx += 1
                self._sa_state = "GOTO_RETURN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_LOWERING → GOTO_RETURN_Y  y={HOME_Y}")

        elif self._sa_state == "GOTO_RETURN_Y":
            if self._drive_minimap_axis_with_heading(HOME_Y, "y", math.pi / 2, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_HOME_X"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_RETURN_Y → GOTO_HOME_X  x={HOME_X}")

        elif self._sa_state == "GOTO_HOME_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_HOME_X → WAITING (cycle #{self._drop_idx})")
