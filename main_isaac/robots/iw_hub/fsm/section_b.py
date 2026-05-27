"""
fsm/section_b.py
================
IW Hub Section B FSM (iw_hub_02).

corridor↔stack 이동 시 반드시 waypoint(-6.0) 경유:
  outbound: stack(-8.2,1.55) 집기 → 후진 WP_X(-6,1.55)
            → south (-6,0) → south (-6,-5) corridor
            → east → 배달(14.5~7.0, -1.5) → south corridor(-5.0)
  inbound:  corridor(-5.0) → west (-6,-5) → north (-6,0)
            → north (-6,1.55) → west stack(-8.2,1.55) 내려놓기

사이클 구조 (총 6사이클):
  [배달]
    TURN_TO_STACK    → yaw=180° (west)
    GOTO_STACK       → x=-8.2 (stack)
    LIFTING
    BACK_WP_X        → no-turn east x=-6.0
    TURN_TO_WP_SOUTH → yaw=-90° (south)
    GOTO_WP_Y        → south y=0.0
    GOTO_CORR_B      → south y=-5.0
    TURN_TO_EAST     → yaw=0°
    GOTO_DELIVERY_X  → east x=14.5-drop_idx×1.5
    GOTO_DELIVERY_Y  → north y=-1.5
    LOWERING         (drop_idx++)
    GOTO_RETURN_Y    → no-turn south y=-5.0

  [보충] (drop_idx 1~5)
    GOTO_REPL_X      → west x=slot_x
    TURN_TO_REPL     → yaw=90°
    GOTO_REPL_Y      → north y=slot_y
    LIFTING_REPL
    GOTO_REPL_CORR   → no-turn south y=-5.0
    GOTO_HOME_X      → west x=-6.0
    TURN_TO_NORTH    → yaw=90°
    GOTO_WP_BACK_Y   → north y=0.0
    GOTO_STACK_Y     → north y=1.55
    TURN_TO_STACK_DEP → yaw=180°
    GOTO_STACK_DEP   → west x=-8.2
    LOWERING_REPL
    BACK_FROM_STACK  → no-turn east x=-6.0
    → WAITING

  [마지막] (drop_idx=6)
    GOTO_DONE_X      → west x=-6.0
    DONE_TURN_NORTH  → yaw=90°
    GOTO_DONE_WP_Y   → north y=0.0
    GOTO_DONE_Y      → north y=1.55
    → WAITING

보충 슬롯 PICKUP_SLOTS[drop_idx]:
  idx=1 B-01 (-3.5, -3.0)
  idx=2 B-02 (0.0,  -3.0)
  idx=3 B-03 (3.5,  -3.0)
  idx=4 B-04 (-3.5,  0.0)
  idx=5 B-05 (0.0,   0.0)
"""
import math

ROUTE_TOL       = 0.06
POD_TOL         = 0.01
POD_SPEED       = 0.25
STACK_PLACE_TOL = 0.03
STACK_X         = -8.3    # PodStack_02 x
STACK_Y         =  1.55   # PodStack_02 y = spawn y
WP_X            = -6.0    # waypoint column x
WP_Y            =  0.0    # upper waypoint y
HOME_Y          = -5.0    # corridor y (lower waypoint y)
FIRST_DROP_X    = 14.5
DROP_STEP_X     =  1.5
DROP_Y          = -1.5    # delivery y

PICKUP_SLOTS = [
    (-3.5, -3.0),  # idx=0 (미사용 placeholder)
    ( 0.0, -3.0),  # idx=1 → B-02
    ( 3.5, -3.0),  # idx=2 → B-03
    (-3.5,  0.0),  # idx=3 → B-04
    ( 0.0,  0.0),  # idx=4 → B-05
    ( 3.5,  0.0),  # idx=5 → B-06
]
MAX_DELIVERIES = len(PICKUP_SLOTS)  # 6


class SectionBFSM:
    def _run_section_b_fsm(self) -> None:
        cnt = self._get_signal_count()

        # ── WAITING ──────────────────────────────────────────────────────────
        if self._sa_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] B_WAITING  signal={cnt}/{self._complete_needed} "
                      f"ready={self._b_signal_ready} "
                      f"pos=({x:.2f},{y:.2f}) del={self._drop_idx}/{MAX_DELIVERIES}")
            if not self._b_signal_ready and cnt >= self._complete_needed:
                self._reset_signal_count()
                self._b_signal_ready = True
            if self._b_signal_ready:
                self._sa_state = "TURN_TO_STACK"
                self._fsm_step = 0
                print(f"[{self.name}] B_WAITING → TURN_TO_STACK  yaw=180°")

        # ── 배달 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "TURN_TO_STACK":
            if self._turn_to_heading(math.pi):
                self._sa_state = "GOTO_STACK"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_STACK → GOTO_STACK  x={STACK_X}")

        elif self._sa_state == "GOTO_STACK":
            if self._drive_minimap_axis_with_heading(
                    STACK_X, "x", math.pi, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_STACK → LIFTING")

        elif self._sa_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._sa_state = "BACK_WP_X"
                self._fsm_step = 0
                print(f"[{self.name}] B_LIFTING → BACK_WP_X  x={WP_X} (후진)")

        elif self._sa_state == "BACK_WP_X":
            if self._drive_minimap_axis_no_turn(WP_X, "x", ROUTE_TOL):
                self._sa_state = "TURN_TO_WP_SOUTH"
                self._fsm_step = 0
                print(f"[{self.name}] B_BACK_WP_X → TURN_TO_WP_SOUTH  yaw=-90°  at ({WP_X},{STACK_Y})")

        elif self._sa_state == "TURN_TO_WP_SOUTH":
            if self._turn_to_heading(-math.pi / 2):
                self._sa_state = "GOTO_WP_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_WP_SOUTH → GOTO_WP_Y  y={WP_Y}")

        elif self._sa_state == "GOTO_WP_Y":
            if self._drive_minimap_axis_no_turn(WP_Y, "y", ROUTE_TOL):
                self._sa_state = "GOTO_CORR_B"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_WP_Y → GOTO_CORR_B  y={HOME_Y}")

        elif self._sa_state == "GOTO_CORR_B":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._sa_state = "TURN_TO_EAST"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_CORR_B → TURN_TO_EAST  yaw=0°  at ({WP_X},{HOME_Y})")

        elif self._sa_state == "TURN_TO_EAST":
            if self._turn_to_heading(0.0):
                drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
                self._sa_state = "GOTO_DELIVERY_X"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_EAST → GOTO_DELIVERY_X  x={drop_x:.1f} (#{self._drop_idx})")

        elif self._sa_state == "GOTO_DELIVERY_X":
            drop_x = FIRST_DROP_X - self._drop_idx * DROP_STEP_X
            if self._drive_minimap_axis_with_heading(drop_x, "x", 0.0, ROUTE_TOL, fast=True):
                self._sa_state = "GOTO_DELIVERY_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_DELIVERY_X → GOTO_DELIVERY_Y  y={DROP_Y}")

        elif self._sa_state == "GOTO_DELIVERY_Y":
            if self._drive_minimap_axis_with_heading(DROP_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "LOWERING"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_DELIVERY_Y → LOWERING")

        elif self._sa_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._b_signal_ready = False
                self._drop_idx += 1
                self._sa_state = "GOTO_RETURN_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_LOWERING → GOTO_RETURN_Y  y={HOME_Y} 후진 (del#{self._drop_idx})")

        elif self._sa_state == "GOTO_RETURN_Y":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                if self._drop_idx < MAX_DELIVERIES:
                    sx, sy = PICKUP_SLOTS[self._drop_idx]
                    self._sa_state = "GOTO_REPL_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] B_GOTO_RETURN_Y → GOTO_REPL_X  x={sx:.1f} B-0{self._drop_idx}")
                else:
                    self._sa_state = "GOTO_DONE_X"
                    self._fsm_step = 0
                    print(f"[{self.name}] B_GOTO_RETURN_Y → GOTO_DONE_X  모든 배달 완료")

        # ── 보충 서브-페이즈 ──────────────────────────────────────────────────

        elif self._sa_state == "GOTO_REPL_X":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            if self._drive_minimap_axis_with_heading(sx, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_REPL_X → TURN_TO_REPL  slot=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "TURN_TO_REPL":
            if self._turn_to_heading(math.pi / 2):
                sx, sy = PICKUP_SLOTS[self._drop_idx]
                self._sa_state = "GOTO_REPL_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_REPL → GOTO_REPL_Y  y={sy:.1f}")

        elif self._sa_state == "GOTO_REPL_Y":
            sx, sy = PICKUP_SLOTS[self._drop_idx]
            if self._drive_minimap_axis_with_heading(
                    sy, "y", math.pi / 2, POD_TOL, max_speed=POD_SPEED):
                self._sa_state = "LIFTING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_REPL_Y → LIFTING_REPL  at=({sx:.1f},{sy:.1f})")

        elif self._sa_state == "LIFTING_REPL":
            if self._run_lift_phase(up=True):
                self._sa_state = "GOTO_REPL_CORR"
                self._fsm_step = 0
                print(f"[{self.name}] B_LIFTING_REPL → GOTO_REPL_CORR  y={HOME_Y} (후진)")

        elif self._sa_state == "GOTO_REPL_CORR":
            if self._drive_minimap_axis_no_turn(HOME_Y, "y", ROUTE_TOL):
                self._sa_state = "GOTO_HOME_X"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_REPL_CORR → GOTO_HOME_X  x={WP_X}")

        # ── 보충 귀환: corridor → waypoint(-6) → stack 내려놓기 ─────────────

        elif self._sa_state == "GOTO_HOME_X":
            if self._drive_minimap_axis_with_heading(WP_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "TURN_TO_NORTH"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_HOME_X → TURN_TO_NORTH  yaw=90°  at ({WP_X},{HOME_Y})")

        elif self._sa_state == "TURN_TO_NORTH":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "GOTO_WP_BACK_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_NORTH → GOTO_WP_BACK_Y  y={WP_Y}")

        elif self._sa_state == "GOTO_WP_BACK_Y":
            if self._drive_minimap_axis_no_turn(WP_Y, "y", ROUTE_TOL):
                self._sa_state = "GOTO_STACK_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_WP_BACK_Y → GOTO_STACK_Y  y={STACK_Y}")

        elif self._sa_state == "GOTO_STACK_Y":
            if self._drive_minimap_axis_with_heading(STACK_Y, "y", math.pi / 2, ROUTE_TOL):
                self._sa_state = "TURN_TO_STACK_DEP"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_STACK_Y → TURN_TO_STACK_DEP  yaw=180°")

        elif self._sa_state == "TURN_TO_STACK_DEP":
            if self._turn_to_heading(math.pi):
                self._sa_state = "GOTO_STACK_DEP"
                self._fsm_step = 0
                print(f"[{self.name}] B_TURN_TO_STACK_DEP → GOTO_STACK_DEP  x={STACK_X}")

        elif self._sa_state == "GOTO_STACK_DEP":
            if self._drive_minimap_axis_with_heading(
                    STACK_X, "x", math.pi, STACK_PLACE_TOL):
                self._sa_state = "LOWERING_REPL"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_STACK_DEP → LOWERING_REPL  deposit at ({STACK_X},{STACK_Y})")

        elif self._sa_state == "LOWERING_REPL":
            if self._run_lift_phase(up=False):
                self._b_signal_ready = False
                self._sa_state = "BACK_FROM_STACK"
                self._fsm_step = 0
                print(f"[{self.name}] B_LOWERING_REPL → BACK_FROM_STACK  x={WP_X} (후진)")

        elif self._sa_state == "BACK_FROM_STACK":
            if self._drive_minimap_axis_no_turn(WP_X, "x", ROUTE_TOL):
                self._publish_start_signal()
                self._reset_signal_count()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] B_BACK_FROM_STACK → WAITING  cycle del={self._drop_idx}/{MAX_DELIVERIES}")

        # ── 마지막 배달 후 귀환: corridor → waypoint(-6) → spawn ────────────

        elif self._sa_state == "GOTO_DONE_X":
            if self._drive_minimap_axis_with_heading(WP_X, "x", math.pi, ROUTE_TOL, fast=True):
                self._sa_state = "DONE_TURN_NORTH"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_DONE_X → DONE_TURN_NORTH  yaw=90°  at ({WP_X},{HOME_Y})")

        elif self._sa_state == "DONE_TURN_NORTH":
            if self._turn_to_heading(math.pi / 2):
                self._sa_state = "GOTO_DONE_WP_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_DONE_TURN_NORTH → GOTO_DONE_WP_Y  y={WP_Y}")

        elif self._sa_state == "GOTO_DONE_WP_Y":
            if self._drive_minimap_axis_no_turn(WP_Y, "y", ROUTE_TOL):
                self._sa_state = "GOTO_DONE_Y"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_DONE_WP_Y → GOTO_DONE_Y  y={STACK_Y}")

        elif self._sa_state == "GOTO_DONE_Y":
            if self._drive_minimap_axis_with_heading(STACK_Y, "y", math.pi / 2, ROUTE_TOL):
                self._drop_idx = 0
                self._publish_start_signal()
                self._reset_signal_count()
                self._sa_state = "WAITING"
                self._fsm_step = 0
                print(f"[{self.name}] B_GOTO_DONE_Y → WAITING  모든 사이클({MAX_DELIVERIES}) 완료, drop_idx 리셋")
