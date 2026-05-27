"""
iw_hub_movement/manual_console.py
=================================
Interactive manual console for an IW Hub.

Run:
    ros2 run iw_hub_movement manual_console --ros-args -p robot_name:=iw_hub_02

Useful commands:
    f [speed]        forward
    b [speed]        backward
    left [speed]     rotate left
    right [speed]    rotate right
    stop             stop wheels
    up               lift up
    down             lift down
    lift <height>    set lift height in meters
    pose             print odom and minimap pose
    cal <x> <y> [yaw_deg]
                     set current odom pose equal to this minimap pose
    rec [name]       record current pose
    list             show recorded positions
    save [path]      save recorded positions as JSON
    save_py [path]   save recorded minimap waypoints as Python list
    quit             stop and exit
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from iw_hub_movement.models import normalize_angle, yaw_from_quaternion

_ISAAC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class ManualConsoleNode(Node):
    def __init__(self, robot_name_override: str | None = None) -> None:
        super().__init__("iw_hub_manual_console")

        self.declare_parameter("robot_name", "iw_hub_02")
        self.declare_parameter("default_linear", 0.25)
        self.declare_parameter("default_angular", 0.5)
        self.declare_parameter("lift_up", 0.30)

        self.robot_name = robot_name_override or str(self.get_parameter("robot_name").value)
        self.default_linear = float(self.get_parameter("default_linear").value)
        self.default_angular = float(self.get_parameter("default_angular").value)
        self.lift_up = float(self.get_parameter("lift_up").value)

        self._cmd_pub = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", 10)
        self._lift_pub = self.create_publisher(JointState, f"/{self.robot_name}/lift_cmd", 10)
        self.create_subscription(Odometry, f"/{self.robot_name}/odom", self._on_odom, _ISAAC_QOS)

        self._odom = None
        self._map_tf = None
        self._records = []

        self.get_logger().info(f"[{self.robot_name}] manual console ready")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._odom = {
            "x": float(p.x),
            "y": float(p.y),
            "yaw": yaw_from_quaternion(o.x, o.y, o.z, o.w),
        }

    def publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def publish_lift(self, height: float) -> None:
        msg = JointState()
        msg.name = ["lift_joint"]
        msg.position = [float(height)]
        self._lift_pub.publish(msg)

    def calibrate_minimap(self, map_x: float, map_y: float, map_yaw_deg: float = 0.0) -> None:
        if self._odom is None:
            print("No odom yet. Wait a moment and run cal again.")
            return

        map_yaw = math.radians(map_yaw_deg)
        yaw_off = normalize_angle(map_yaw - self._odom["yaw"])
        c = math.cos(yaw_off)
        s = math.sin(yaw_off)
        ox_map = c * self._odom["x"] - s * self._odom["y"]
        oy_map = s * self._odom["x"] + c * self._odom["y"]
        self._map_tf = {
            "yaw": yaw_off,
            "tx": float(map_x) - ox_map,
            "ty": float(map_y) - oy_map,
        }
        print(f"calibrated: current odom -> minimap ({map_x:.3f}, {map_y:.3f}, {map_yaw_deg:.1f}deg)")

    def odom_pose(self):
        return self._odom

    def minimap_pose(self):
        if self._odom is None or self._map_tf is None:
            return None
        yaw_off = self._map_tf["yaw"]
        c = math.cos(yaw_off)
        s = math.sin(yaw_off)
        x = c * self._odom["x"] - s * self._odom["y"] + self._map_tf["tx"]
        y = s * self._odom["x"] + c * self._odom["y"] + self._map_tf["ty"]
        yaw = normalize_angle(self._odom["yaw"] + yaw_off)
        return {"x": x, "y": y, "yaw": yaw}

    def print_pose(self) -> None:
        if self._odom is None:
            print("odom: waiting...")
            return
        print(
            f"odom:    x={self._odom['x']:.3f} y={self._odom['y']:.3f} "
            f"yaw={math.degrees(self._odom['yaw']):.1f}deg"
        )
        mp = self.minimap_pose()
        if mp is None:
            print("minimap: not calibrated. Use: cal <minimap_x> <minimap_y> [yaw_deg]")
        else:
            print(
                f"minimap: x={mp['x']:.3f} y={mp['y']:.3f} "
                f"yaw={math.degrees(mp['yaw']):.1f}deg"
            )

    def record(self, name: str) -> None:
        if self._odom is None:
            print("No odom yet.")
            return
        entry = {
            "name": name,
            "time": time.time(),
            "odom": dict(self._odom),
            "minimap": self.minimap_pose(),
        }
        self._records.append(entry)
        mp = entry["minimap"]
        if mp is None:
            print(f"recorded {name}: odom=({entry['odom']['x']:.3f}, {entry['odom']['y']:.3f})")
        else:
            print(f"recorded {name}: minimap=({mp['x']:.3f}, {mp['y']:.3f})")

    def list_records(self) -> None:
        if not self._records:
            print("No recorded positions.")
            return
        for idx, rec in enumerate(self._records, start=1):
            mp = rec["minimap"]
            od = rec["odom"]
            if mp is None:
                print(f"{idx:02d}. {rec['name']}: odom=({od['x']:.3f}, {od['y']:.3f})")
            else:
                print(f"{idx:02d}. {rec['name']}: minimap=({mp['x']:.3f}, {mp['y']:.3f})")

    def save_json(self, path: str) -> None:
        out = Path(path).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "robot_name": self.robot_name,
            "odom_to_minimap": self._map_tf,
            "records": self._records,
        }
        out.write_text(json.dumps(data, indent=2), encoding="utf-8")
        print(f"saved {out}")

    def save_py(self, path: str) -> None:
        out = Path(path).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        waypoints = []
        for rec in self._records:
            pose = rec["minimap"] or rec["odom"]
            waypoints.append((rec["name"], round(pose["x"], 4), round(pose["y"], 4)))

        lines = [
            "# Generated by iw_hub_movement manual_console",
            f"ROBOT_NAME = {self.robot_name!r}",
            "WAYPOINTS = [",
        ]
        for name, x, y in waypoints:
            lines.append(f"    ({name!r}, {x:.4f}, {y:.4f}),")
        lines.append("]")
        lines.append("")
        out.write_text("\n".join(lines), encoding="utf-8")
        print(f"saved {out}")


def _spin_thread(node: ManualConsoleNode, stop_event: threading.Event) -> None:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.05)
            except IndexError as exc:
                # Isaac's embedded ROS can throw wait-set errors if another
                # executor rebuilds entities at the same time. Skip that tick.
                print(f"[manual_console] ROS wait-set skipped once: {exc}")
                time.sleep(0.05)
    finally:
        executor.remove_node(node)


def _parse_float(parts, idx: int, default: float) -> float:
    if len(parts) <= idx:
        return default
    return float(parts[idx])


def _handle_console_command(node: ManualConsoleNode, raw: str) -> bool:
    parts = raw.split()
    if not parts:
        return True
    cmd = parts[0].lower()

    try:
        if cmd in {"q", "quit", "exit"}:
            return False
        if cmd in {"h", "help"}:
            print(__doc__)
        elif cmd in {"f", "forward"}:
            node.publish_cmd(_parse_float(parts, 1, node.default_linear), 0.0)
        elif cmd in {"b", "back", "backward"}:
            node.publish_cmd(-_parse_float(parts, 1, node.default_linear), 0.0)
        elif cmd in {"left", "lturn"}:
            node.publish_cmd(0.0, _parse_float(parts, 1, node.default_angular))
        elif cmd in {"right", "rturn"}:
            node.publish_cmd(0.0, -_parse_float(parts, 1, node.default_angular))
        elif cmd in {"s", "stop"}:
            node.publish_cmd(0.0, 0.0)
        elif cmd == "up":
            node.publish_lift(node.lift_up)
        elif cmd == "down":
            node.publish_lift(0.0)
        elif cmd == "lift":
            node.publish_lift(float(parts[1]))
        elif cmd == "pose":
            node.print_pose()
        elif cmd == "cal":
            yaw = _parse_float(parts, 3, 0.0)
            node.calibrate_minimap(float(parts[1]), float(parts[2]), yaw)
        elif cmd in {"rec", "record"}:
            name = parts[1] if len(parts) > 1 else f"p{len(node._records) + 1:02d}"
            node.record(name)
        elif cmd == "list":
            node.list_records()
        elif cmd == "save":
            path = parts[1] if len(parts) > 1 else "iw_hub_positions.json"
            node.save_json(path)
        elif cmd == "save_py":
            path = parts[1] if len(parts) > 1 else "iw_hub_waypoints.py"
            node.save_py(path)
        else:
            print(f"Unknown command: {cmd}. Type 'help'.")
    except (IndexError, ValueError) as exc:
        print(f"Bad command arguments: {exc}")
    return True


class EmbeddedManualConsole:
    """Background stdin console used by main.py."""

    def __init__(self, robot_name: str = "iw_hub_02") -> None:
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = ManualConsoleNode(robot_name_override=robot_name)
        self.node.get_logger().info(f"[{robot_name}] embedded console attached")
        self._stop_event = threading.Event()
        self._spin_worker = threading.Thread(
            target=_spin_thread, args=(self.node, self._stop_event), daemon=True)
        self._input_worker = threading.Thread(target=self._input_loop, daemon=True)

    def start(self) -> None:
        self._spin_worker.start()
        self._input_worker.start()
        print("[main] IW Hub manual console started. Type 'help' in this terminal.")

    def _input_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw = input("iw-hub> ").strip()
            except (EOFError, KeyboardInterrupt):
                self._stop_event.set()
                break
            if not _handle_console_command(self.node, raw):
                self._stop_event.set()
                break

    def close(self) -> None:
        self.node.publish_cmd(0.0, 0.0)
        self._stop_event.set()
        self._spin_worker.join(timeout=1.0)
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def start_embedded_console(robot_name: str = "iw_hub_02") -> EmbeddedManualConsole:
    console = EmbeddedManualConsole(robot_name=robot_name)
    console.start()
    return console


def main(args=None):
    rclpy.init(args=args)
    node = ManualConsoleNode()
    stop_event = threading.Event()
    worker = threading.Thread(target=_spin_thread, args=(node, stop_event), daemon=True)
    worker.start()

    print("Manual IW Hub console. Type 'help' for commands.")
    try:
        while True:
            raw = input("iw-hub> ").strip()
            if not raw:
                continue
            if not _handle_console_command(node, raw):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        stop_event.set()
        worker.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
