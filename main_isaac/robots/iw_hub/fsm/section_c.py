"""
fsm/section_c.py
================
IW Hub Section C FSM (iw_hub_03).

사이클 구조 (총 6사이클):
  [배달] Stack(-9.65,-8.9) 집고 → corridor y=-15 후진
         → east → Conveyor(14.5~2.0, -12.0) 내려놓고
         → corridor y=-15 후진

  [보충] (drop_idx 1~5인 경우만)
         → west → C-slot_x 이동 → north → C-slot_y 집고
         → corridor y=-15 후진
         → west → HOME_X=-9.65 → north → STACK_Y=-8.9 내려놓고
         → corridor y=-15 후진
         → WAITING

  drop_idx=6 (마지막): 보충 없이 GOTO_DONE_X → HOME → WAITING

배달 위치:
  drop_idx=0 → x=14.5, drop_idx=1 → x=12.0, ..., drop_idx=5 → x=2.0

보충 슬롯 (PICKUP_SLOTS[drop_idx]):
  idx=1 → C-02 (0.0, -13.0)
  idx=2 → C-03 (3.5, -13.0)
  idx=3 → C-04 (-3.5, -10.0)
  idx=4 → C-05 (0.0, -10.0)
  idx=5 → C-06 (3.5, -10.0)
"""
import math

ROUTE_TOL       = 0.06
POD_TOL         = 0.01
POD_SPEED       = 0.25
STACK_PLACE_TOL = 0.03
HOME_X          = -9.65
HOME_Y          = -15.0
STACK_Y         = -8.9
FIRST_DROP_X    = 14.5
DROP_STEP_X     = 1.5
DROP_Y          = -12.0

# C-01은 stack 위치이므로 보충 대상에서 제외. idx=1~5만 사용.
PICKUP_SLOTS = [
    (-3.5, -13.0),  # C-01 (미사용: stack이 여기서 시작)
    ( 0.0, -13.0),  # C-02
    ( 3.5, -13.0),  # C-03
    (-3.5, -10.0),  # C-04
    ( 0.0, -10.0),  # C-05
    ( 3.5, -10.0),  # C-06
]
MAX_DELIVERIES = len(PICKUP_SLOTS)  # 6


class SectionCFSM:
    def _run_section_c_fsm(self) -> None:
        cnt = self._get_signal_count()

        # ── WAITING ──────────────────────────────────────────────────────────
        if self._sa_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] C_WAITING  signal={cnt}/{self._complete_needed} "
                      f"pos=({x:.2f},{y:.2f}) del={self._drop_idx}/{MAX_DELIVERIES}")
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self._sa_state = "TURN_TO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] C_WAITING → TURN_TO_POD  yaw=90°")

        # ── 배달 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "TURN_TO_POD":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "GOTO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] C_TURN_TO_POD → GOTO_POD  y={STACK_Y}")

        elif self._sa_state == "GOTO_POD":
            if self._drive_minimap_axis_with_heading(
                    STACK_Y, "y", math.pi / 2, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_POD → LIFTING")

        elif self._sa_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_CORR_A"
                self._fsm_step = 0
                print(f"[{self.name}] C_LIFTING → GOTO_CORR_A  y={HOME_Y} (후진)")

        elif self._sa_state == "GOTO_CORR_A":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._sa_state = "TURN_TO_EAST"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_CORR_A → TURN_TO_EAST  yaw=0°")

        elif self._sa_state == "TURN_TO_EAST":
            if self._turn_to_heading(0.0):
                drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
                self._sa_state = "GOTO_DELIVERY_X"
                self._fsm_step = 0
                print(f"[{self.name}] C_TURN_TO_EAST → GOTO_DELIVERY_X  x={drop_x:.1f} (#{self._drop_idx})")

        elif self._sa_state == "GOTO_DELIVERY_X":
            drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
            if self._drive_minimap_axis_with_heading(drop_x, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_DELIVERY_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_DELIVERY_X → GOTO_DELIVERY_Y  y={DROP_Y}")

        elif self._sa_state == "GOTO_DELIVERY_Y":
            if self._drive_minimap_axis_with_heading(DROP_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "LOWERING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_DELIVERY_Y → LOWERING")

        elif self._sa_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._drop_idx += 1
                self._sa_state = "GOTO_RETURN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_LOWERING → GOTO_RETURN_Y  y={HOME_Y} 후진 (del#{self._drop_idx})")

        elif self._sa_state == "GOTO_RETURN_Y":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                if self._drop_idx < MAX_DELIVERIES:
                    sx, sy = PICKUP_SLOTS[self._drop_idx]
                    self._sa_state = "GOTO_REPL_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] C_GOTO_RETURN_Y → GOTO_REPL_X  x={sx:.1f} C-0{self._drop_idx+1}")
                else:
                    self._sa_state = "GOTO_DONE_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] C_GOTO_RETURN_Y → GOTO_DONE_X  모든 배달 완료")

        # ── 보충 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "GOTO_REPL_X":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            x, _, _ = self._get_xy_hdg()
            hdg = 0.0 if sx > x else math.pi
            if self._drive_minimap_axis_with_heading(sx, "x", hdg, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_REPL_X → TURN_TO_REPL  slot=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "TURN_TO_REPL":
            if self._turn_to_heading(math.pi / 2):
                sx, sy = PICKUP_SLOTS[self._drop_idx]
                self._sa_state = "GOTO_REPL_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_TURN_TO_REPL → GOTO_REPL_Y  y={sy:.1f}")

        elif self._sa_state == "GOTO_REPL_Y":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            if self._drive_minimap_axis_with_heading(
                    sy, "y", math.pi / 2, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_REPL_Y → LIFTING_REPL  at=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "LIFTING_REPL":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_REPL_CORR"
                self._fsm_step = 0
                print(f"[{self.name}] C_LIFTING_REPL → GOTO_REPL_CORR  y={HOME_Y} (후진)")

        elif self._sa_state == "GOTO_REPL_CORR":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._sa_state = "GOTO_HOME_X"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_REPL_CORR → GOTO_HOME_X  x={HOME_X}")

        elif self._sa_state == "GOTO_HOME_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_STACK"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_HOME_X → TURN_TO_STACK  yaw=90°")

        elif self._sa_state == "TURN_TO_STACK":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "GOTO_STACK_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_TURN_TO_STACK → GOTO_STACK_Y  y={STACK_Y}")

        elif self._sa_state == "GOTO_STACK_Y":
            if self._drive_minimap_axis_with_heading(
                    STACK_Y, "y", math.pi / 2, STACK_PLACE_TOL):
                self._sa_state = "LOWERING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_STACK_Y → LOWERING_REPL  deposit at ({HOME_X},{STACK_Y})")

        elif self._sa_state == "LOWERING_REPL":
            if self._run_lift_phase(up=False):
                self._sa_state = "GOTO_HOME_Y"
                self._fsm_step = 0
                print(f"[{self.name}] C_LOWERING_REPL → GOTO_HOME_Y  y={HOME_Y} (후진)")

        elif self._sa_state == "GOTO_HOME_Y":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._publish_start_signal()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_HOME_Y → WAITING  cycle del={self._drop_idx}/{MAX_DELIVERIES}")

        # ── 마지막 배달 후 귀환 ────────────────────────────────────────────────

        elif self._sa_state == "GOTO_DONE_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._publish_start_signal()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] C_GOTO_DONE_X → WAITING  모든 사이클({MAX_DELIVERIES}) 완료")
