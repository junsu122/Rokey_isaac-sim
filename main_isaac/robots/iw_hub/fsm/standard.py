"""
fsm/standard.py
===============
IW Hub 표준 모드 FSM (iw_hub_02 등 section_a/c/pickup 이외 로봇).

상태:
  0 WAITING      완료 신호 대기
  1 LIFTING      리프트 업
  2 GOTO_SECTION 섹션 슬롯 01 로 이동
  3 LOWERING     리프트 다운
  4 MOVE_OUT     슬롯에서 홈 방향으로 1m 이탈
  5 GOTO_HOME    홈(PodStack) 으로 복귀
"""
import math


class StandardFSM:
    def _run_fsm(self) -> None:
        cnt = self._get_signal_count()

        if self.mission_state == 0:              # WAITING
            if cnt >= self._complete_needed:
                self._reset_signal_count()
                self.mission_state = 1
                self._fsm_step     = 0
                print(f"[{self.name}] WAITING → LIFTING  (신호 {cnt}회 수신)")

        elif self.mission_state == 1:            # LIFTING
            self._fsm_step += 1
            t = min(self._fsm_step / self.LIFT_STEPS, 1.0)
            self._publish_lift(t * self.LIFT_UP)
            if self._fsm_step >= self.LIFT_STEPS:
                self._nav_target   = self._get_drop_pos()
                self._plan_path_to(*self._nav_target)
                self.mission_state = 2
                self._fsm_step     = 0
                print(f"[{self.name}] LIFTING → GOTO_SECTION({self._section_name}) "
                      f"슬롯01={self._nav_target}")

        elif self.mission_state == 2:            # GOTO_SECTION
            if self._nav_along_path():
                self.mission_state = 3
                self._fsm_step     = 0
                print(f"[{self.name}] GOTO_SECTION → LOWERING")

        elif self.mission_state == 3:            # LOWERING
            self._fsm_step += 1
            t = min(self._fsm_step / self.LIFT_STEPS, 1.0)
            self._publish_lift((1.0 - t) * self.LIFT_UP)
            if self._fsm_step >= self.LIFT_STEPS:
                self._drop_idx += 1
                sx, sy = self._get_drop_pos()
                hx, hy = self._home_xy
                dist = math.hypot(hx - sx, hy - sy)
                if dist > 0.1:
                    ratio = min(1.0, 1.0 / dist)
                    mx = sx + (hx - sx) * ratio
                    my = sy + (hy - sy) * ratio
                else:
                    mx, my = hx, hy
                self._nav_target = (mx, my)
                self._plan_path_to(mx, my)
                self.mission_state = 4
                self._fsm_step     = 0
                print(f"[{self.name}] LOWERING → MOVE_OUT  중간목표=({mx:.2f},{my:.2f})")

        elif self.mission_state == 4:            # MOVE_OUT
            if self._nav_along_path():
                self._nav_target = self._home_xy
                self._plan_path_to(*self._home_xy)
                self.mission_state = 5
                self._fsm_step     = 0
                print(f"[{self.name}] MOVE_OUT → GOTO_HOME")

        elif self.mission_state == 5:            # GOTO_HOME
            if self._nav_along_path():
                self._publish_start_signal()
                self.mission_state = 0
                print(f"[{self.name}] GOTO_HOME → WAITING (배달 #{self._drop_idx})")
