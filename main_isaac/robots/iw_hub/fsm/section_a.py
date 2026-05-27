"""
fsm/section_a.py
================
IW Hub Section A FSM (iw_hub_01).

spawn↔corridor 이동 시 반드시 waypoint(-7.0) 경유:
  outbound: spawn(-12.8,13) → stack(9.0) 집기 → 후진 spawn(13)
            → east (-7,13) → south (-7,5) corridor
            → east → 배달(14.5~7.0, 9.0) → south corridor(5.0)
  inbound:  corridor(5.0) → west (-7,5) → north (-7,13)
            → west spawn(-12.8,13) → [보충 시: south stack 내려놓기]

사이클 구조 (총 6사이클):
  [배달]
    TURN_TO_POD      → yaw=-90° (south)
    GOTO_POD         → y=9.0  (stack)
    LIFTING
    BACK_TO_SPAWN_Y  → no-turn north y=13.0
    TURN_TO_WP       → yaw=0°  (east)
    GOTO_WP_X        → east x=-7.0  (along y=13)
    TURN_TO_CORR     → yaw=-90° (south)
    GOTO_CORR_A      → south y=5.0  (along x=-7)
    TURN_TO_EAST     → yaw=0°
    GOTO_DELIVERY_X  → east x=14.5-drop_idx×1.5
    GOTO_DELIVERY_Y  → north y=9.0
    LOWERING         (drop_idx++)
    GOTO_RETURN_Y    → no-turn south y=5.0

  [보충] (drop_idx 1~5)
    GOTO_REPL_X      → west x=slot_x
    TURN_TO_REPL     → yaw=90°
    GOTO_REPL_Y      → north y=slot_y
    LIFTING_REPL
    GOTO_REPL_CORR   → no-turn south y=5.0
    REPL_WP_X        → west x=-7.0
    REPL_WP_TURN     → yaw=90°
    REPL_WP_Y        → north y=13.0
    GOTO_HOME_X      → west x=-12.8
    TURN_TO_STACK    → yaw=-90°
    GOTO_STACK_Y     → south y=9.0
    LOWERING_REPL
    GOTO_HOME_Y      → no-turn north y=13.0
    → WAITING

  [마지막] (drop_idx=6)
    DONE_WP_X        → west x=-7.0
    DONE_WP_TURN     → yaw=90°
    DONE_WP_Y        → north y=13.0
    GOTO_DONE_X      → west x=-12.8
    → WAITING

보충 슬롯 PICKUP_SLOTS[drop_idx]:
  idx=1 A-02 (0.0,  7.0)
  idx=2 A-03 (3.5,  7.0)
  idx=3 A-04 (-3.5, 10.0)
  idx=4 A-05 (0.0,  10.0)
  idx=5 A-06 (3.5,  10.0)
"""
import math

ROUTE_TOL       = 0.06
POD_TOL         = 0.01
POD_SPEED       = 0.25
STACK_PLACE_TOL = 0.03
HOME_X          = -12.8
HOME_Y          = 13.0    # spawn / waiting y
WP_X            = -7.0    # waypoint column x
CORR_Y          = 5.0     # outbound corridor y
STACK_Y         = 9.0     # PodStack_01 y
FIRST_DROP_X    = 14.5
DROP_STEP_X     = 1.5
DROP_Y          = 9.0

PICKUP_SLOTS = [
    (-3.5,  7.0),  # A-01 (미사용)
    ( 0.0,  7.0),  # A-02
    ( 3.5,  7.0),  # A-03
    (-3.5, 10.0),  # A-04
    ( 0.0, 10.0),  # A-05
    ( 3.5, 10.0),  # A-06
]
MAX_DELIVERIES = len(PICKUP_SLOTS)  # 6


class SectionAFSM:
    def _run_section_a_fsm(self) -> None:
        cnt = self._get_signal_count()

        # ── WAITING ──────────────────────────────────────────────────────────
        if self._sa_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] A_WAITING  signal={cnt}/{self._complete_needed} "
                      f"pos=({x:.2f},{y:.2f}) del={self._drop_idx}/{MAX_DELIVERIES}")
            if not self._b_signal_ready and cnt >= self._complete_needed:
                self._reset_signal_count()
                self._b_signal_ready = True
            if self._b_signal_ready:
                self._sa_state = "TURN_TO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] A_WAITING → TURN_TO_POD  yaw=-90°")

        # ── 배달 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "TURN_TO_POD":
            if self._turn_to_heading(-math.pi / 2):
                self._sa_state = "GOTO_POD"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_POD → GOTO_POD  y={STACK_Y}")

        elif self._sa_state == "GOTO_POD":
            if self._drive_minimap_axis_with_heading(
                    STACK_Y, "y", -math.pi / 2, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_POD → LIFTING")

        elif self._sa_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._sa_state = "BACK_TO_SPAWN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_LIFTING → BACK_TO_SPAWN_Y  y={HOME_Y} (후진)")

        elif self._sa_state == "BACK_TO_SPAWN_Y":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._sa_state = "TURN_TO_WP"
                self._fsm_step = 0
                print(f"[{self.name}] A_BACK_TO_SPAWN_Y → TURN_TO_WP  yaw=0°")

        elif self._sa_state == "TURN_TO_WP":
            if self._turn_to_heading(0.0):
                self._sa_state = "GOTO_WP_X"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_WP → GOTO_WP_X  x={WP_X}")

        elif self._sa_state == "GOTO_WP_X":
            if self._drive_minimap_axis_with_heading(WP_X, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_CORR"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_WP_X → TURN_TO_CORR  yaw=-90°  at ({WP_X},{HOME_Y})")

        elif self._sa_state == "TURN_TO_CORR":
            if self._turn_to_heading(-math.pi / 2):
                self._sa_state = "GOTO_CORR_A"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_CORR → GOTO_CORR_A  y={CORR_Y}")

        elif self._sa_state == "GOTO_CORR_A":
            if self._drive_minimap_axis_no_turn(CORR_Y, "y", ROUTE_TOL):
                self._sa_state = "TURN_TO_EAST"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_CORR_A → TURN_TO_EAST  yaw=0°  at ({WP_X},{CORR_Y})")

        elif self._sa_state == "TURN_TO_EAST":
            if self._turn_to_heading(0.0):
                drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
                self._sa_state = "GOTO_DELIVERY_X"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_EAST → GOTO_DELIVERY_X  x={drop_x:.1f} (#{self._drop_idx})")

        elif self._sa_state == "GOTO_DELIVERY_X":
            drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
            if self._drive_minimap_axis_with_heading(drop_x, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_DELIVERY_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_DELIVERY_X → GOTO_DELIVERY_Y  y={DROP_Y}")

        elif self._sa_state == "GOTO_DELIVERY_Y":
            if self._drive_minimap_axis_with_heading(DROP_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "LOWERING"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_DELIVERY_Y → LOWERING")

        elif self._sa_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._b_signal_ready = False
                self._drop_idx += 1
                self._sa_state = "GOTO_RETURN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_LOWERING → GOTO_RETURN_Y  y={CORR_Y} 후진 (del#{self._drop_idx})")

        elif self._sa_state == "GOTO_RETURN_Y":
            if self._drive_minimap_axis_no_turn(CORR_Y, "y", ROUTE_TOL):
                if self._drop_idx < MAX_DELIVERIES:
                    sx, sy = PICKUP_SLOTS[self._drop_idx]
                    self._sa_state = "GOTO_REPL_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] A_GOTO_RETURN_Y → GOTO_REPL_X  x={sx:.1f} A-0{self._drop_idx+1}")
                else:
                    self._sa_state = "DONE_WP_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] A_GOTO_RETURN_Y → DONE_WP_X  모든 배달 완료")

        # ── 보충 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "GOTO_REPL_X":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            if self._drive_minimap_axis_with_heading(sx, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_REPL_X → TURN_TO_REPL  slot=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "TURN_TO_REPL":
            if self._turn_to_heading(math.pi / 2):
                sx, sy = PICKUP_SLOTS[self._drop_idx]
                self._sa_state = "GOTO_REPL_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_REPL → GOTO_REPL_Y  y={sy:.1f}")

        elif self._sa_state == "GOTO_REPL_Y":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            if self._drive_minimap_axis_with_heading(
                    sy, "y", math.pi / 2, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_REPL_Y → LIFTING_REPL  at=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "LIFTING_REPL":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_REPL_CORR"
                self._fsm_step = 0
                print(f"[{self.name}] A_LIFTING_REPL → GOTO_REPL_CORR  y={CORR_Y} (후진)")

        elif self._sa_state == "GOTO_REPL_CORR":
            if self._drive_minimap_axis_no_turn(CORR_Y, "y", ROUTE_TOL):
                self._sa_state = "REPL_WP_X"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_REPL_CORR → REPL_WP_X  x={WP_X}")

        # ── 보충 귀환: corridor → waypoint(-7) → spawn → stack 내려놓기 ────────

        elif self._sa_state == "REPL_WP_X":
            if self._drive_minimap_axis_with_heading(WP_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "REPL_WP_TURN"
                self._fsm_step = 0
                print(f"[{self.name}] A_REPL_WP_X → REPL_WP_TURN  yaw=90°  at ({WP_X},{CORR_Y})")

        elif self._sa_state == "REPL_WP_TURN":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "REPL_WP_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_REPL_WP_TURN → REPL_WP_Y  y={HOME_Y}")

        elif self._sa_state == "REPL_WP_Y":
            if self._drive_minimap_axis_with_heading(HOME_Y, "y", math.pi / 2, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_HOME_X"
                self._fsm_step = 0
                print(f"[{self.name}] A_REPL_WP_Y → GOTO_HOME_X  x={HOME_X}")

        elif self._sa_state == "GOTO_HOME_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_STACK"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_HOME_X → TURN_TO_STACK  yaw=-90°")

        elif self._sa_state == "TURN_TO_STACK":
            if self._turn_to_heading(-math.pi / 2):
                self._sa_state = "GOTO_STACK_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_TURN_TO_STACK → GOTO_STACK_Y  y={STACK_Y}")

        elif self._sa_state == "GOTO_STACK_Y":
            if self._drive_minimap_axis_with_heading(
                    STACK_Y, "y", -math.pi / 2, STACK_PLACE_TOL):
                self._sa_state = "LOWERING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_STACK_Y → LOWERING_REPL  deposit at ({HOME_X},{STACK_Y})")

        elif self._sa_state == "LOWERING_REPL":
            if self._run_lift_phase(up=False):
                self._b_signal_ready = False
                self._sa_state = "GOTO_HOME_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_LOWERING_REPL → GOTO_HOME_Y  y={HOME_Y} (후진)")

        elif self._sa_state == "GOTO_HOME_Y":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._publish_start_signal()
                self._reset_signal_count()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_HOME_Y → WAITING  cycle del={self._drop_idx}/{MAX_DELIVERIES}")

        # ── 마지막 배달 후 귀환: corridor → waypoint(-7) → spawn ────────────

        elif self._sa_state == "DONE_WP_X":
            if self._drive_minimap_axis_with_heading(WP_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "DONE_WP_TURN"
                self._fsm_step = 0
                print(f"[{self.name}] A_DONE_WP_X → DONE_WP_TURN  yaw=90°  at ({WP_X},{CORR_Y})")

        elif self._sa_state == "DONE_WP_TURN":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "DONE_WP_Y"
                self._fsm_step = 0
                print(f"[{self.name}] A_DONE_WP_TURN → DONE_WP_Y  y={HOME_Y}")

        elif self._sa_state == "DONE_WP_Y":
            if self._drive_minimap_axis_with_heading(HOME_Y, "y", math.pi / 2, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_DONE_X"
                self._fsm_step = 0
                print(f"[{self.name}] A_DONE_WP_Y → GOTO_DONE_X  x={HOME_X}")

        elif self._sa_state == "GOTO_DONE_X":
            if self._drive_minimap_axis_with_heading(HOME_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._drop_idx = 0
                self._publish_start_signal()
                self._reset_signal_count()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] A_GOTO_DONE_X → WAITING  모든 사이클({MAX_DELIVERIES}) 완료, drop_idx 리셋")
