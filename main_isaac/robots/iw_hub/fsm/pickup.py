"""
fsm/pickup.py
=============
IW Hub 픽업 모드 FSM (iw_hub_02).

흐름: spawn(-6.45,1.5) → 트리거 → X이동→pickup(-7.95,1.5) → 리프트업
      → 통로(-6.0,1.5) → Y이동→X이동 → 슬롯01 → 리프트다운
      → 후진 이탈 → 다음 섹션 pod 픽업 → conveyor 옆 pickup 위치로 복귀
"""
import math

CORRIDOR_XY = (-6.0, 1.5)


class PickupFSM:
    def _run_pickup_fsm(self) -> None:
        cnt       = self._get_signal_count()
        px, py    = self._pickup_xy
        cx, cy    = CORRIDOR_XY
        sx, sy    = self._section_entry_xy()

        if self._pickup_state == "WAITING":
            self._fsm_step += 1
            if self._fsm_step % 100 == 0:
                x, y, _ = self._get_xy_hdg()
                print(f"[{self.name}] WAITING  신호={cnt}/{self._complete_needed}  "
                      f"pos=({x:.2f},{y:.2f})")
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self._reset_dock_pid()
                self._pickup_state = "GOTO_PICKUP"
                self._fsm_step     = 0
                print(f"[{self.name}] WAITING → GOTO_PICKUP  pickup={self._pickup_xy}")

        elif self._pickup_state == "GOTO_PICKUP":
            if self._drive_along_x(px):
                self._reset_dock_pid()
                self._pickup_state = "LIFTING"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_PICKUP → LIFTING")

        elif self._pickup_state == "LIFTING":
            if self._run_lift_phase(up=True):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_INTERM"
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING → GOTO_INTERM  interm=({sx:.2f},{sy:.2f})")

        elif self._pickup_state == "GOTO_INTERM":
            if self._drive_axis_to_tol(sx, sy, "x", self.PRECISE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "TURN_FIRST_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_INTERM → TURN_FIRST_Y  yaw=-90°")

        elif self._pickup_state == "TURN_FIRST_Y":
            if self._turn_to_heading(-math.pi / 2.0):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_FIRST_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_FIRST_Y → GOTO_FIRST_Y  target={self._first_route_y_target()}")

        elif self._pickup_state == "GOTO_FIRST_Y":
            tx, ty = self._first_route_y_target()
            if self._drive_minimap_axis_with_heading(ty, "y", -math.pi / 2.0, self.PRECISE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "TURN_FIRST_X"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_FIRST_Y → TURN_FIRST_X  yaw=-180°")

        elif self._pickup_state == "TURN_FIRST_X":
            if self._turn_to_heading(-math.pi):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SLOT"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_FIRST_X → GOTO_SLOT  target={self._first_place_target()}")

        elif self._pickup_state == "GOTO_SLOT":
            self._fsm_step += 1
            tx, ty = self._first_place_target()
            mx, my, _ = self._get_xy_hdg()
            reached_drop_area = mx >= self.FIRST_DROP_MIN_X
            if reached_drop_area:
                self._publish_cmd_vel(0.0, 0.0)
                self._backout_x    = self._section_exit_x()
                self._dock_target  = (mx, my)
                self._reset_dock_pid()
                self._pickup_state = "LOWERING"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SLOT → LOWERING  line reached "
                      f"pos=({mx:.2f},{my:.2f}) target=({tx:.2f},{ty:.2f})")
                return
            if self._drive_minimap_axis_with_heading(tx, "x", -math.pi, self.PLACE_TOL, fast=True):
                mx, my, _ = self._get_xy_hdg()
                if mx < self.FIRST_DROP_MIN_X:
                    print(f"[{self.name}] GOTO_SLOT minimap guard  "
                          f"pos=({mx:.2f},{my:.2f}) target=({tx:.2f},{ty:.2f})")
                    return
                self._backout_x    = self._section_exit_x()
                self._dock_target  = (mx, my)
                self._reset_dock_pid()
                self._pickup_state = "LOWERING"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SLOT → LOWERING  at=({mx:.2f},{my:.2f})")

        elif self._pickup_state == "SETTLE_PLACE":
            tx, ty = self._dock_target or self._first_place_target()
            if self._hold_exact_place_target(tx, ty):
                self._reset_dock_pid()
                self._pickup_state = "LOWERING"
                self._fsm_step     = 0
                print(f"[{self.name}] SETTLE_PLACE → LOWERING  exact=({tx:.2f},{ty:.2f})")

        elif self._pickup_state == "LOWERING":
            if self._run_lift_phase(up=False):
                self._drop_idx    += 1
                self._reset_dock_pid()
                self._pickup_state = "RETURN_FIRST_GATE_X"
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING → RETURN_FIRST_GATE_X  gate={self._first_route_y_target()}")

        elif self._pickup_state == "RETURN_FIRST_GATE_X":
            tx, _ = self._first_route_y_target()
            if self._drive_x_fast_to(tx, self.PRECISE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SECOND_TOP_X"
                self._fsm_step     = 0
                print(f"[{self.name}] RETURN_FIRST_GATE_X → GOTO_SECOND_TOP_X  top={self._second_route_top_target()}")

        elif self._pickup_state in ("RETURN_TOP_X", "GOTO_SECOND_TOP_X"):
            tx, _ = self._second_route_top_target()
            if self._drive_minimap_axis_with_heading(tx, "x", -math.pi, 0.03, fast=False):
                self._reset_dock_pid()
                self._pickup_state = "TURN_SECOND_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SECOND_TOP_X → TURN_SECOND_Y  yaw=-90°")

        elif self._pickup_state == "TURN_SECOND_Y":
            if self._turn_to_heading(-math.pi / 2.0):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SECOND_POD_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_SECOND_Y → GOTO_SECOND_POD_Y  target={self._second_pod_target()}")

        elif self._pickup_state == "GOTO_SECOND_POD_Y":
            _, ty = self._second_pod_target()
            if self._drive_minimap_axis_with_heading(ty, "y", -math.pi / 2.0, 0.03):
                self._reset_dock_pid()
                self._pickup_state = "LIFTING_OUT"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SECOND_POD_Y → LIFTING_OUT  at={self._second_pod_target()}")

        elif self._pickup_state == "LIFTING_OUT":
            if self._run_lift_phase(up=True):
                self._reset_dock_pid()
                self._pickup_state = "TURN_SECOND_TOP_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING_OUT → TURN_SECOND_TOP_Y  yaw=90°")

        elif self._pickup_state == "TURN_SECOND_TOP_Y":
            if self._turn_to_heading(math.pi / 2.0):
                self._reset_dock_pid()
                self._pickup_state = "RETURN_SECOND_TOP_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_SECOND_TOP_Y → RETURN_SECOND_TOP_Y")

        elif self._pickup_state == "RETURN_SECOND_TOP_Y":
            _, ty = self._second_route_top_target()
            if self._drive_minimap_axis_with_heading(ty, "y", math.pi / 2.0, self.PRECISE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "TURN_SECOND_GATE_X"
                self._fsm_step     = 0
                print(f"[{self.name}] RETURN_SECOND_TOP_Y → TURN_SECOND_GATE_X  yaw=-180°")

        elif self._pickup_state == "TURN_SECOND_GATE_X":
            if self._turn_to_heading(-math.pi):
                self._reset_dock_pid()
                self._pickup_state = "RETURN_SECOND_GATE_X"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_SECOND_GATE_X → RETURN_SECOND_GATE_X")

        elif self._pickup_state == "RETURN_SECOND_GATE_X":
            tx, _ = self._section_entry_xy()
            if self._drive_minimap_axis_with_heading(tx, "x", -math.pi, self.PRECISE_TOL, fast=True):
                self._reset_dock_pid()
                self._pickup_state = "TURN_SECOND_PICKUP_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] RETURN_SECOND_GATE_X → TURN_SECOND_PICKUP_Y  yaw=-90°")

        elif self._pickup_state == "TURN_SECOND_PICKUP_Y":
            if self._turn_to_heading(-math.pi / 2.0):
                self._reset_dock_pid()
                self._pickup_state = "RETURN_SECOND_PICKUP_Y"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_SECOND_PICKUP_Y → RETURN_SECOND_PICKUP_Y")

        elif self._pickup_state == "RETURN_SECOND_PICKUP_Y":
            _, ty = self._pickup_xy
            if self._drive_minimap_axis_with_heading(ty, "y", -math.pi / 2.0, self.PRECISE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "TURN_SECOND_PICKUP_X"
                self._fsm_step     = 0
                print(f"[{self.name}] RETURN_SECOND_PICKUP_Y → TURN_SECOND_PICKUP_X  yaw=-180°")

        elif self._pickup_state == "TURN_SECOND_PICKUP_X":
            if self._turn_to_heading(-math.pi):
                self._reset_dock_pid()
                self._pickup_state = "RETURN_SECOND_PICKUP_X"
                self._fsm_step     = 0
                print(f"[{self.name}] TURN_SECOND_PICKUP_X → RETURN_SECOND_PICKUP_X")

        elif self._pickup_state == "RETURN_SECOND_PICKUP_X":
            tx, _ = self._pickup_xy
            if self._drive_minimap_axis_with_heading(tx, "x", -math.pi, self.PLACE_TOL):
                self._reset_dock_pid()
                self._pickup_state = "LOWERING_OUT"
                self._fsm_step     = 0
                print(f"[{self.name}] RETURN_SECOND_PICKUP_X → LOWERING_OUT  supply={self._pickup_xy}")

        elif self._pickup_state == "GOTO_CORR_RETURN":
            if self._nav_axis_aligned(cx, cy, yfirst=True):
                self._reset_dock_pid()
                self._pickup_state = "GOTO_SUPPLY"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_CORR_RETURN → GOTO_SUPPLY  supply={self._pickup_xy}")

        elif self._pickup_state == "GOTO_SUPPLY":
            if self._nav_axis_aligned(px, py):
                self._reset_dock_pid()
                self._pickup_state = "LOWERING_OUT"
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SUPPLY → LOWERING_OUT")

        elif self._pickup_state == "LOWERING_OUT":
            if self._run_lift_phase(up=False):
                self._delivery_idx = (self._delivery_idx + 1) % max(1, len(self._delivery_slots))
                self._reset_dock_pid()
                self._pickup_state = "WAITING"
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING_OUT → WAITING  supply={self._pickup_xy}")

        elif self._pickup_state == "RETREAT":
            hx, hy = self._home_xy
            if self._nav_axis_aligned(hx, hy):
                self._reset_dock_pid()
                self._pickup_state = "WAITING"
                self._fsm_step     = 0
                print(f"[{self.name}] RETREAT → WAITING  (cycle #{self._drop_idx})")
