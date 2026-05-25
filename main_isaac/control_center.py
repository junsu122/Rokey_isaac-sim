"""
External control center bridge.

Isaac Sim keeps the simulation-side APIs.  A separate system-Python process owns
the visible control window, so it can live outside the Isaac Sim application like
the minimap.
"""

from __future__ import annotations

import json
import os
import queue
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from auto_spawn_panel import AutoSpawnPanel


_SCRIPT = Path(__file__).parent / "external_control_center.py"
_CAM_W = 200
_CAM_H = 150
_DEFAULT_CAMERA_FPS = 4.0
_CONSOLE_TICK_HZ = 10.0


def _clean_env() -> dict:
    keep = {
        "HOME", "USER", "USERNAME", "LOGNAME",
        "DISPLAY", "XAUTHORITY", "WAYLAND_DISPLAY",
        "XDG_RUNTIME_DIR", "DBUS_SESSION_BUS_ADDRESS",
        "LANG", "LC_ALL", "LC_CTYPE", "TZ", "TERM",
    }
    env = {k: v for k, v in os.environ.items() if k in keep}
    env.setdefault("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin")
    return env


@dataclass
class _CameraSlot:
    agent: object
    key: str
    title: str
    enabled: bool = True


class ControlCenter:
    """Simulation-side bridge for the external control-center process."""

    def __init__(self, world, agents: list):
        self.world = world
        self.agents = agents
        self.spawn_panel = AutoSpawnPanel(world, build_window=False)

        self._camera_slots: list[_CameraSlot] = [
            _CameraSlot(
                agent=a,
                key=getattr(a, "name", f"robot_{i}"),
                title=f"{getattr(a, 'name', f'robot_{i}')} ({a.cfg.get('type', '?')})",
            )
            for i, a in enumerate(agents)
            if self._has_camera(a)
        ]
        self._camera_fps = _DEFAULT_CAMERA_FPS
        self._camera_index = 0
        self._last_camera_t = 0.0
        self._last_console_t = 0.0
        self._last_state_t = 0.0

        self._proc: Optional[subprocess.Popen] = None
        self._send_q: queue.Queue = queue.Queue(maxsize=4)
        self._cmd_q: queue.Queue = queue.Queue()
        self._send_thread: Optional[threading.Thread] = None
        self._stdout_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None
        self._ready = False

        self._ensure_ready()
        print("[ControlCenter] external control center bridge ready")

    # ── Process lifecycle ────────────────────────────────────────────────

    def _ensure_ready(self) -> bool:
        if self._ready and self._proc is not None and self._proc.poll() is None:
            return True
        self._ready = False
        try:
            self._proc = subprocess.Popen(
                ["/usr/bin/python3", str(_SCRIPT)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=_clean_env(),
            )
            self._send_thread = threading.Thread(target=self._sender, daemon=True)
            self._stdout_thread = threading.Thread(target=self._stdout_reader, daemon=True)
            self._stderr_thread = threading.Thread(target=self._stderr_reader, daemon=True)
            self._send_thread.start()
            self._stdout_thread.start()
            self._stderr_thread.start()
            self._ready = True
            self._send_state()
            return True
        except Exception as e:
            print(f"[ControlCenter] external process start failed: {e}")
            return False

    def _sender(self):
        while True:
            item = self._send_q.get()
            if item is None:
                break
            if self._proc is None or self._proc.poll() is not None:
                break
            try:
                self._proc.stdin.write(item)
                self._proc.stdin.flush()
            except Exception:
                break

    def _stdout_reader(self):
        try:
            for raw_line in self._proc.stdout:
                line = raw_line.decode(errors="ignore").strip()
                if line:
                    self._cmd_q.put_nowait(line)
        except Exception:
            pass

    def _stderr_reader(self):
        try:
            for raw_line in self._proc.stderr:
                line = raw_line.decode(errors="ignore").rstrip()
                if line:
                    print(f"[ControlCenter-proc] {line}")
        except Exception:
            pass

    def close(self):
        self._enqueue(0xFF, b"")
        try:
            self._send_q.put_nowait(None)
        except Exception:
            pass
        if self._proc is not None:
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._ready = False

    # ── Main tick ────────────────────────────────────────────────────────

    def tick(self):
        if not self._ensure_ready():
            return

        self._drain_commands()

        now = time.time()
        if now - self._last_console_t >= 1.0 / _CONSOLE_TICK_HZ:
            self._last_console_t = now
            self.spawn_panel.external_tick()

        if now - self._last_state_t >= 0.5:
            self._last_state_t = now
            self._send_state()

        active = [s for s in self._camera_slots if s.enabled]
        if not active:
            return
        stagger_period = 1.0 / max(self._camera_fps * len(active), 0.5)
        if now - self._last_camera_t >= stagger_period:
            self._last_camera_t = now
            slot = active[self._camera_index % len(active)]
            self._camera_index = (self._camera_index + 1) % len(active)
            self._send_camera(slot)

    # ── Commands from external process ───────────────────────────────────

    def _drain_commands(self):
        while True:
            try:
                line = self._cmd_q.get_nowait()
            except queue.Empty:
                break
            self._handle_command(line)

    def _handle_command(self, line: str):
        parts = line.split()
        if not parts:
            return
        cmd = parts[0]

        if cmd == "CAM" and len(parts) == 3:
            key, enabled = parts[1], parts[2] == "1"
            for slot in self._camera_slots:
                if slot.key == key:
                    slot.enabled = enabled
                    break
            self._send_state()
        elif cmd == "FPS" and len(parts) == 2:
            try:
                self._camera_fps = float(np.clip(float(parts[1]), 0.5, 15.0))
            except ValueError:
                pass
            self._send_state()
        elif cmd == "BOX_RANDOM_ONCE":
            self.spawn_panel.external_random_spawn_once()
        elif cmd == "BOX_RANDOM_TOGGLE":
            self.spawn_panel.external_toggle_random()
        elif cmd == "BOX_AUTO" and len(parts) == 2:
            self.spawn_panel.external_set_auto(parts[1] == "1")
        elif cmd == "BOX_CLEAR":
            self.spawn_panel.external_clear_all()
        elif cmd == "BOX_SPAWN_AT" and len(parts) >= 7:
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                w, d, h = float(parts[4]), float(parts[5]), float(parts[6])
                aruco_id = int(parts[7]) if len(parts) >= 8 else 0
            except ValueError:
                return
            self.spawn_panel.external_spawn_at(
                x, y, z, w, d, h, aruco_id=aruco_id
            )
        elif cmd == "DRONE_GO" and len(parts) == 5:
            self._drone_go(parts[1], parts[2], parts[3], parts[4])
        elif cmd == "DRONE_LAND" and len(parts) == 2:
            self._drone_land(parts[1])
        elif cmd == "DRONE_MISSION" and len(parts) >= 3:
            self._drone_mission(parts[1], parts[2:])

    def _drone_go(self, name: str, sx: str, sy: str, sz: str):
        agent = self._find_agent(name)
        ctrl = getattr(agent, "controller", None) if agent is not None else None
        if ctrl is None:
            return
        try:
            gx, gy, gz = float(sx), float(sy), max(0.1, float(sz))
        except ValueError:
            return
        ctrl.target_pos = np.array([gx, gy, gz])
        ctrl.is_airborne = True
        ctrl.integral = np.zeros(3)
        print(f"[ControlCenter] Drone Go {name}: ({gx:.2f}, {gy:.2f}, {gz:.2f})")

    def _drone_land(self, name: str):
        agent = self._find_agent(name)
        ctrl = getattr(agent, "controller", None) if agent is not None else None
        if ctrl is None:
            return
        ctrl.target_pos[2] = 0.07
        ctrl.is_airborne = False
        print(f"[ControlCenter] Drone Land {name}")

    def _drone_mission(self, name: str, args: list) -> None:
        """'A 2 B 3 C 1' or 'A2 B3 C1' → agent.set_mission({...})"""
        import re
        agent = self._find_agent(name)
        if agent is None or not hasattr(agent, "set_mission"):
            print(f"[ControlCenter] DRONE_MISSION: 에이전트 없음 또는 set_mission 미지원: {name}")
            return
        targets = {}
        flat = " ".join(args)
        for m in re.finditer(r'([A-Ca-c])\s*(\d+)', flat):
            targets[m.group(1).upper()] = int(m.group(2))
        if not targets:
            print(f"[ControlCenter] DRONE_MISSION 파싱 실패: {flat!r}")
            return
        agent.set_mission(targets)
        print(f"[ControlCenter] Drone {name} 미션 설정: {targets}")

    def _find_agent(self, name: str):
        return next((a for a in self.agents if getattr(a, "name", None) == name), None)

    # ── Messages to external process ─────────────────────────────────────

    def _send_state(self):
        payload = {
            "camera_fps": self._camera_fps,
            "cameras": [
                {"key": s.key, "title": s.title, "enabled": s.enabled}
                for s in self._camera_slots
            ],
            "drones": [
                {"name": getattr(a, "name", "Drone")}
                for a in self.agents
                if getattr(a, "cfg", {}).get("type") == "drone"
            ],
        }
        self._enqueue(0x02, json.dumps(payload).encode("utf-8"))

    def _send_camera(self, slot: _CameraSlot):
        frame = self._get_agent_rgb(slot.agent)
        label = self._agent_label(slot.agent)
        if frame is None:
            bgr = self._blank_bgr("no frame")
        else:
            bgr = self._rgb_to_panel_bgr(frame, label)
        key = slot.key.encode("utf-8")
        payload = (
            struct.pack("<BHH", len(key), _CAM_H, _CAM_W)
            + key
            + bgr.tobytes()
        )
        self._enqueue(0x01, payload)

    def _enqueue(self, msg_type: int, data: bytes):
        header = bytes([msg_type]) + struct.pack("<I", len(data))
        try:
            self._send_q.put_nowait(header + data)
        except queue.Full:
            pass

    @staticmethod
    def _has_camera(agent) -> bool:
        if getattr(agent, "cfg", {}).get("type") in ("m0609", "spot", "drone"):
            return True
        return hasattr(agent, "get_camera_rgb") or getattr(agent, "_wrist_cam", None) is not None

    @staticmethod
    def _get_agent_rgb(agent):
        if hasattr(agent, "get_camera_rgb"):
            return agent.get_camera_rgb()
        wrist_cam = getattr(agent, "_wrist_cam", None)
        if wrist_cam is not None:
            try:
                return wrist_cam.get_rgb()
            except Exception:
                return None
        return None

    @staticmethod
    def _agent_label(agent) -> str:
        if hasattr(agent, "_display_label"):
            try:
                return agent._display_label()
            except Exception:
                pass
        state = getattr(agent, "_state", None)
        if state:
            return f"state: {state}"
        return "ready"

    @staticmethod
    def _rgb_to_panel_bgr(rgb: np.ndarray, label: str) -> np.ndarray:
        if rgb.ndim == 3 and rgb.shape[2] > 3:
            rgb = rgb[:, :, :3]
        if rgb.dtype != np.uint8:
            max_v = float(np.nanmax(rgb)) if rgb.size else 0.0
            scale = 255.0 if max_v <= 1.0 else 1.0
            rgb = np.clip(rgb * scale, 0, 255).astype(np.uint8)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bgr = cv2.resize(bgr, (_CAM_W, _CAM_H), interpolation=cv2.INTER_AREA)
        cv2.line(bgr, (_CAM_W // 2, 0), (_CAM_W // 2, _CAM_H), (0, 255, 255), 1)
        cv2.line(bgr, (0, _CAM_H // 2), (_CAM_W, _CAM_H // 2), (0, 255, 255), 1)
        cv2.putText(bgr, label[:42], (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        return bgr

    @staticmethod
    def _blank_bgr(text: str) -> np.ndarray:
        bgr = np.zeros((_CAM_H, _CAM_W, 3), dtype=np.uint8)
        cv2.putText(bgr, text, (95, 120), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (120, 120, 120), 2)
        return bgr
