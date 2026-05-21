"""
robots/ 컬렉션 관리.
로봇 타입별 (AMR / Drone / Arm) 상태를 Firestore에 기록합니다.

Firestore 구조:
  robots/
    amr_001/    ← 자율주행 로봇
    drone_001/  ← 드론
    m0609/      ← 두산 협동로봇
"""

from firebase_admin import firestore
from .firebase_manager import now_ts


# ── 공통 상태 상수 ────────────────────────────────────────────
class RobotState:
    IDLE    = "idle"
    MOVING  = "moving"
    ERROR   = "error"

class ArmState(RobotState):
    PICKING  = "picking"
    PLACING  = "placing"

class AMRState:
    # 충전 상태
    CHARGING   = "charging"    # 충전 중
    OPERATING  = "operating"   # 운행 중

    # 물품 적재 상태
    EMPTY      = "empty"       # 빈 카트
    LOADING    = "loading"     # 수납 중 (박스 올리는 중)
    TRANSPORTING = "transporting"  # 옮기는 중
    UNLOADING  = "unloading"   # 물품 내려놓는 중

class DroneState(RobotState):
    TAKING_OFF = "taking_off"
    FLYING     = "flying"
    HOVERING   = "hovering"
    LANDING    = "landing"


# ── 베이스 클래스 ─────────────────────────────────────────────
class BaseRobotManager:
    robot_type: str = "base"

    def __init__(self, db: firestore.Client, robot_id: str):
        self.robot_id = robot_id
        self._ref = db.collection("robots").document(robot_id)
        if not self._ref.get().exists:
            self._ref.set(self._initial_doc())

    def _initial_doc(self) -> dict:
        return {
            "robot_id":     self.robot_id,
            "type":         self.robot_type,
            "status":       RobotState.IDLE,
            "current_task": None,
            "battery":      100.0,
            "error_code":   None,
            "last_updated": now_ts(),
        }

    def set_status(self, status: str, task_id: str | None = None):
        update = {"status": status, "last_updated": now_ts()}
        if task_id is not None:
            update["current_task"] = task_id
        self._ref.update(update)
        print(f"[{self.robot_type}/{self.robot_id}] 상태: {status}" +
              (f"  작업: {task_id}" if task_id else ""))

    def update_battery(self, level: float):
        self._ref.update({"battery": round(level, 1), "last_updated": now_ts()})

    def set_error(self, error_code: str):
        self._ref.update({
            "status":       RobotState.ERROR,
            "error_code":   error_code,
            "last_updated": now_ts(),
        })
        print(f"[{self.robot_type}/{self.robot_id}] 에러: {error_code}")

    def clear_error(self):
        self._ref.update({
            "status":       RobotState.IDLE,
            "error_code":   None,
            "current_task": None,
            "last_updated": now_ts(),
        })

    def get(self) -> dict:
        return self._ref.get().to_dict()


# ── AMR (자율주행 로봇) ───────────────────────────────────────
class AMRManager(BaseRobotManager):
    """
    자율주행 로봇 상태 관리.

    Firestore 필드:
      battery           : 남은 배터리 (0.0 ~ 100.0 %)
      charge_status     : 충전 상태 — "charging" | "operating"
      cargo_status      : 물품 적재 상태 — "empty" | "loading" | "transporting" | "unloading"
      position          : {x, y, yaw}   ← 오도메트리 기반 현재 위치
      speed             : 현재 속도 (m/s)
      current_task      : 현재 작업 ID

      localization                       ← ArUco 기반 위치 보정 데이터
        ├─ marker_id    : 인식한 마커 ID
        ├─ label        : 마커 위치명 (예: "구역A", "충전소 앞")
        ├─ estimated_pos: 마커로 추정한 로봇 위치 {x, y, yaw}
        ├─ distance     : 마커까지의 거리 (m)
        └─ detected_at  : 인식 시각

      localization_history : 최근 위치 인식 이력 (최대 10건)
    """
    robot_type = "amr"

    def _initial_doc(self) -> dict:
        doc = super()._initial_doc()
        doc.pop("status", None)
        doc.pop("battery", None)
        doc.update({
            "battery":              100.0,
            "charge_status":        AMRState.OPERATING,
            "cargo_status":         AMRState.EMPTY,
            "position":             {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "speed":                0.0,
            "localization":         None,
            "localization_history": [],
        })
        return doc

    # ── 배터리 / 충전 ─────────────────────────────────────────

    def update_battery(self, level: float):
        """남은 배터리(%) 업데이트."""
        self._ref.update({
            "battery":      round(level, 1),
            "last_updated": now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 배터리: {level:.1f}%")

    def start_charging(self):
        """충전 시작."""
        self._ref.update({
            "charge_status": AMRState.CHARGING,
            "cargo_status":  AMRState.EMPTY,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 충전 중")

    def stop_charging(self):
        """충전 완료 → 운행 대기."""
        self._ref.update({
            "charge_status": AMRState.OPERATING,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 충전 완료 → 운행 중")

    # ── 물품 적재 상태 ────────────────────────────────────────

    def set_loading(self, task_id: str):
        """수납 중 — M0609가 박스를 AMR 카트에 올리는 중."""
        self._ref.update({
            "cargo_status":  AMRState.LOADING,
            "current_task":  task_id,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 수납 중  작업={task_id}")

    def set_transporting(self):
        """옮기는 중 — 목적지를 향해 이동."""
        self._ref.update({
            "cargo_status":  AMRState.TRANSPORTING,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 옮기는 중")

    def set_unloading(self):
        """물품 내려놓는 중 — 목적지 빈에 내려놓는 중."""
        self._ref.update({
            "cargo_status":  AMRState.UNLOADING,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 물품 내려놓는 중")

    def set_empty(self):
        """빈 카트 — 배달 완료 후 복귀."""
        self._ref.update({
            "cargo_status":  AMRState.EMPTY,
            "current_task":  None,
            "last_updated":  now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 빈 카트 (복귀 중)")

    # ── ArUco 위치 인식 ──────────────────────────────────────

    def set_localization(self, marker_id: int, label: str,
                         estimated_pos: dict,
                         distance: float | None = None):
        """
        ArUco 마커로 현재 위치를 인식했을 때 저장합니다.
        estimated_pos: {"x": 1.2, "y": 0.5, "yaw": 90.0}
        """
        loc = {
            "marker_id":     marker_id,
            "label":         label,
            "estimated_pos": estimated_pos,
            "distance":      round(distance, 3) if distance else None,
            "detected_at":   now_ts(),
        }

        current = self._ref.get().to_dict() or {}
        history = current.get("localization_history", [])
        history.append(loc)
        if len(history) > 10:
            history = history[-10:]

        self._ref.update({
            "localization":         loc,
            "localization_history": history,
            "position":             estimated_pos,   # 위치도 함께 보정
            "last_updated":         now_ts(),
        })
        print(f"[AMR/{self.robot_id}] 위치 인식: ID={marker_id} '{label}'"
              f"  추정위치=({estimated_pos['x']:.2f}, {estimated_pos['y']:.2f})"
              + (f"  거리={distance:.3f}m" if distance else ""))

    def get_localization_history(self) -> list[dict]:
        """최근 위치 인식 이력 반환."""
        doc = self._ref.get().to_dict() or {}
        return doc.get("localization_history", [])

    # ── 위치 ─────────────────────────────────────────────────

    def update_pose(self, x: float, y: float, yaw: float, speed: float = 0.0):
        self._ref.update({
            "position":     {"x": x, "y": y, "yaw": yaw},
            "speed":        round(speed, 3),
            "last_updated": now_ts(),
        })


# ── Drone ─────────────────────────────────────────────────────
class DroneManager(BaseRobotManager):
    """
    드론 상태 관리.

    Firestore 필드:
      battery           : 남은 배터리 (0.0 ~ 100.0 %)
      charge_status     : 충전 상태 — "charging" | "operating"
      cargo_status      : 물품 적재 상태 — "empty" | "loading" | "transporting" | "unloading"
      position          : {x, y, z}   ← GPS/IMU 기반 현재 위치
      altitude          : 현재 고도 (m)
      heading           : 방향 (degrees, 0=북)
      speed             : 현재 속도 (m/s)
      current_task      : 현재 작업 ID

      localization                       ← ArUco 기반 위치 보정 데이터
        ├─ marker_id    : 인식한 마커 ID
        ├─ label        : 마커 위치명 (예: "착륙존A", "웨이포인트3")
        ├─ estimated_pos: 마커로 추정한 드론 위치 {x, y, z}
        ├─ distance     : 마커까지의 거리 (m)
        └─ detected_at  : 인식 시각

      localization_history : 최근 위치 인식 이력 (최대 10건)
    """
    robot_type = "drone"

    def _initial_doc(self) -> dict:
        doc = super()._initial_doc()
        doc.pop("status",  None)
        doc.pop("battery", None)
        doc.update({
            "battery":              100.0,
            "charge_status":        AMRState.OPERATING,
            "cargo_status":         AMRState.EMPTY,
            "position":             {"x": 0.0, "y": 0.0, "z": 0.0},
            "altitude":             0.0,
            "heading":              0.0,
            "speed":                0.0,
            "localization":         None,
            "localization_history": [],
        })
        return doc

    # ── 배터리 / 충전 ─────────────────────────────────────────

    def update_battery(self, level: float):
        """남은 배터리(%) 업데이트."""
        self._ref.update({
            "battery":      round(level, 1),
            "last_updated": now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 배터리: {level:.1f}%")

    def start_charging(self):
        """착륙 후 충전 시작."""
        self._ref.update({
            "charge_status": AMRState.CHARGING,
            "cargo_status":  AMRState.EMPTY,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 충전 중")

    def stop_charging(self):
        """충전 완료 → 운행 대기."""
        self._ref.update({
            "charge_status": AMRState.OPERATING,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 충전 완료 → 운행 중")

    # ── 물품 적재 상태 ────────────────────────────────────────

    def set_loading(self, task_id: str):
        """수납 중 — 물품 픽업하는 중."""
        self._ref.update({
            "cargo_status":  AMRState.LOADING,
            "current_task":  task_id,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 수납 중  작업={task_id}")

    def set_transporting(self):
        """옮기는 중 — 목적지를 향해 비행."""
        self._ref.update({
            "cargo_status":  AMRState.TRANSPORTING,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 옮기는 중")

    def set_unloading(self):
        """물품 내려놓는 중 — 목적지에 내려놓는 중."""
        self._ref.update({
            "cargo_status":  AMRState.UNLOADING,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 물품 내려놓는 중")

    def set_empty(self):
        """빈 상태 — 배달 완료 후 복귀."""
        self._ref.update({
            "cargo_status":  AMRState.EMPTY,
            "current_task":  None,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 빈 상태 (복귀 중)")

    # ── ArUco 위치 인식 ──────────────────────────────────────

    def set_localization(self, marker_id: int, label: str,
                         estimated_pos: dict,
                         distance: float | None = None):
        """
        ArUco 마커로 현재 위치를 인식했을 때 저장합니다.
        estimated_pos: {"x": 1.2, "y": 0.5, "z": 2.0}
        """
        loc = {
            "marker_id":     marker_id,
            "label":         label,
            "estimated_pos": estimated_pos,
            "distance":      round(distance, 3) if distance else None,
            "detected_at":   now_ts(),
        }

        current = self._ref.get().to_dict() or {}
        history = current.get("localization_history", [])
        history.append(loc)
        if len(history) > 10:
            history = history[-10:]

        self._ref.update({
            "localization":         loc,
            "localization_history": history,
            "position":             estimated_pos,   # 위치도 함께 보정
            "altitude":             round(estimated_pos.get("z", 0.0), 3),
            "last_updated":         now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 위치 인식: ID={marker_id} '{label}'"
              f"  추정위치=({estimated_pos['x']:.2f}, {estimated_pos['y']:.2f}, {estimated_pos['z']:.2f})"
              + (f"  거리={distance:.3f}m" if distance else ""))

    def get_localization_history(self) -> list[dict]:
        """최근 위치 인식 이력 반환."""
        doc = self._ref.get().to_dict() or {}
        return doc.get("localization_history", [])

    # ── 위치 / 비행 ───────────────────────────────────────────

    def update_pose(self, x: float, y: float, z: float,
                    heading: float = 0.0, speed: float = 0.0):
        """3D 위치 + 방향 + 속도 업데이트."""
        self._ref.update({
            "position":     {"x": x, "y": y, "z": z},
            "altitude":     round(z, 3),
            "heading":      round(heading, 2),
            "speed":        round(speed, 3),
            "last_updated": now_ts(),
        })

    def take_off(self, target_altitude: float, task_id: str | None = None):
        update = {
            "charge_status":    DroneState.TAKING_OFF,
            "target_altitude":  target_altitude,
            "last_updated":     now_ts(),
        }
        if task_id:
            update["current_task"] = task_id
        self._ref.update(update)
        print(f"[Drone/{self.robot_id}] 이륙 → 목표 고도: {target_altitude}m")

    def land(self):
        self._ref.update({
            "charge_status": DroneState.LANDING,
            "cargo_status":  AMRState.EMPTY,
            "last_updated":  now_ts(),
        })
        print(f"[Drone/{self.robot_id}] 착륙 시작")


# ── 두산 M0609 협동로봇 ───────────────────────────────────────
class ArmManager(BaseRobotManager):
    """
    협동로봇 암 상태 관리.

    Firestore 필드:
      status           : 로봇 동작 상태 — "idle" | "picking" | "placing" | "moving" | "error"
      gripper          : 그리퍼 상태 — "open" | "closed"
      position         : 엔드이펙터 위치 {x, y, z}
      joints           : 6축 관절값 [j1~j6] (degrees)
      current_task     : 현재 작업 ID

      detected_item    : 마지막으로 인식한 물품 정보
        ├─ marker_id   : ArUco 마커 ID
        ├─ label       : 분류 레이블 (예: "강남")
        ├─ category    : 분류 유형 (예: "destination")
        ├─ position_xyz: 마커 검출 당시 3D 위치 [x, y, z]
        └─ detected_at : 인식 시각

      detection_history: 최근 인식 이력 (최대 10건 유지)
    """
    robot_type = "arm"

    def _initial_doc(self) -> dict:
        doc = super()._initial_doc()
        doc.update({
            "position":          {"x": 0.0, "y": 0.0, "z": 0.0},
            "joints":            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper":           "open",
            "detected_item":     None,
            "detection_history": [],
        })
        return doc

    # ── ArUco 인식 결과 저장 ──────────────────────────────────

    def set_detected_item(self, marker_id: int, label: str, category: str,
                          position_xyz: tuple[float, float, float] | None = None,
                          item_id: str | None = None):
        """
        ArUco 마커로 인식한 물품 정보를 저장합니다.
        동시에 detection_history에 이력을 추가합니다 (최대 10건).
        """
        detected = {
            "marker_id":    marker_id,
            "label":        label,
            "category":     category,
            "position_xyz": list(position_xyz) if position_xyz else None,
            "item_id":      item_id,
            "detected_at":  now_ts(),
        }

        # 현재 이력 조회 후 최대 10건 유지
        current = self._ref.get().to_dict() or {}
        history = current.get("detection_history", [])
        history.append(detected)
        if len(history) > 10:
            history = history[-10:]

        self._ref.update({
            "detected_item":     detected,
            "detection_history": history,
            "last_updated":      now_ts(),
        })
        print(f"[Arm/{self.robot_id}] 인식: ID={marker_id} '{label}'  위치={position_xyz}")

    def clear_detected_item(self):
        """인식 물품 초기화 (작업 완료 후 호출)."""
        self._ref.update({
            "detected_item": None,
            "last_updated":  now_ts(),
        })

    def get_detection_history(self) -> list[dict]:
        """최근 인식 이력 반환."""
        doc = self._ref.get().to_dict() or {}
        return doc.get("detection_history", [])

    # ── 동작 상태 ─────────────────────────────────────────────

    def set_picking(self, task_id: str):
        self._ref.update({
            "status":       ArmState.PICKING,
            "current_task": task_id,
            "gripper":      "open",
            "last_updated": now_ts(),
        })
        print(f"[Arm/{self.robot_id}] 픽업 시작  작업={task_id}")

    def set_placing(self):
        self._ref.update({
            "status":       ArmState.PLACING,
            "last_updated": now_ts(),
        })
        print(f"[Arm/{self.robot_id}] 내려놓는 중")

    def set_idle(self):
        self._ref.update({
            "status":        ArmState.IDLE,
            "current_task":  None,
            "detected_item": None,
            "gripper":       "open",
            "last_updated":  now_ts(),
        })
        print(f"[Arm/{self.robot_id}] 대기 중")

    # ── 위치 / 그리퍼 ─────────────────────────────────────────

    def update_pose(self, x: float, y: float, z: float,
                    joints: list[float] | None = None):
        update = {
            "position":     {"x": x, "y": y, "z": z},
            "last_updated": now_ts(),
        }
        if joints is not None:
            update["joints"] = [round(j, 3) for j in joints]
        self._ref.update(update)

    def set_gripper(self, state: str):
        """그리퍼 상태 변경. state: 'open' | 'closed'"""
        self._ref.update({"gripper": state, "last_updated": now_ts()})
        print(f"[Arm/{self.robot_id}] 그리퍼: {state}")


# ── 전체 로봇 팩토리 ──────────────────────────────────────────
class RobotFleet:
    """
    세 종류 로봇을 한 번에 초기화하고 관리합니다.

    사용 예:
        fleet = RobotFleet(db)
        fleet.arm.set_status(ArmState.PICKING, task_id="task_001")
        fleet.amr.update_pose(x=1.0, y=0.5, yaw=90)
        fleet.drone.take_off(target_altitude=2.0)
    """
    def __init__(self, db: firestore.Client,
                 amr_id:   str = "amr_001",
                 drone_id: str = "drone_001",
                 arm_id:   str = "m0609"):
        self.amr   = AMRManager(db,   amr_id)
        self.drone = DroneManager(db, drone_id)
        self.arm   = ArmManager(db,   arm_id)
        print(f"[RobotFleet] AMR={amr_id} / Drone={drone_id} / Arm={arm_id} 초기화 완료")

    def get_all_status(self) -> dict:
        return {
            "amr":   self.amr.get(),
            "drone": self.drone.get(),
            "arm":   self.arm.get(),
        }
