#!/usr/bin/env python3
"""
External Robot Control Center.

stdin  <- binary frames/state from Isaac process
stdout -> text commands back to Isaac process
"""

from __future__ import annotations

import json
import base64
import queue
import struct
import sys
import threading
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np


CAM_W = 200
CAM_H = 150


class ExternalControlCenter:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Control Center")
        self.root.geometry("920x620")

        self.msg_q: queue.Queue = queue.Queue()
        self.camera_widgets = {}
        self.camera_vars = {}
        self.drone_rows = {}
        self.fps_var = tk.DoubleVar(value=4.0)
        self.box_x = tk.DoubleVar(value=0.0)
        self.box_y = tk.DoubleVar(value=0.0)
        self.box_z = tk.DoubleVar(value=0.0)
        self.box_w = tk.DoubleVar(value=0.30)
        self.box_d = tk.DoubleVar(value=0.30)
        self.box_h = tk.DoubleVar(value=0.30)
        self.box_aruco = tk.IntVar(value=0)

        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        threading.Thread(target=self._stdin_reader, daemon=True).start()
        self.root.after(30, self._poll_messages)

    # ── UI ───────────────────────────────────────────────────────────────

    def _build(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = ttk.Frame(main, width=420)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)

        cam_bar = ttk.Frame(left)
        cam_bar.pack(fill=tk.X)
        ttk.Label(cam_bar, text="Robot Cameras", font=("Sans", 13, "bold")).pack(side=tk.LEFT)
        ttk.Label(cam_bar, text="FPS").pack(side=tk.LEFT, padx=(18, 4))
        fps = ttk.Spinbox(
            cam_bar, textvariable=self.fps_var, from_=0.5, to=15.0,
            increment=0.5, width=5, command=self._send_fps,
        )
        fps.pack(side=tk.LEFT)
        fps.bind("<Return>", lambda _e: self._send_fps())

        self.camera_canvas = tk.Canvas(left, highlightthickness=0)
        self.camera_scroll = ttk.Scrollbar(left, orient=tk.VERTICAL, command=self.camera_canvas.yview)
        self.camera_grid = ttk.Frame(self.camera_canvas)
        self.camera_grid.bind(
            "<Configure>",
            lambda _e: self.camera_canvas.configure(scrollregion=self.camera_canvas.bbox("all")),
        )
        self.camera_canvas.create_window((0, 0), window=self.camera_grid, anchor="nw")
        self.camera_canvas.configure(yscrollcommand=self.camera_scroll.set)
        self.camera_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=(8, 0))
        self.camera_scroll.pack(side=tk.RIGHT, fill=tk.Y, pady=(8, 0))

        tabs = ttk.Notebook(right)
        tabs.pack(fill=tk.BOTH, expand=True)
        self.box_tab = ttk.Frame(tabs, padding=8)
        self.drone_tab = ttk.Frame(tabs, padding=8)
        tabs.add(self.box_tab, text="Box Spawn")
        tabs.add(self.drone_tab, text="Drone")
        self._build_box_tab()
        self._build_drone_tab()

    def _build_box_tab(self):
        ttk.Label(self.box_tab, text="Box Generation", font=("Sans", 13, "bold")).pack(anchor="w")
        pos = ttk.LabelFrame(self.box_tab, text="Spawn At Position", padding=8)
        pos.pack(fill=tk.X, pady=(10, 8))

        for label, var in (
            ("X", self.box_x), ("Y", self.box_y), ("Z", self.box_z),
            ("W", self.box_w), ("D", self.box_d), ("H", self.box_h),
        ):
            row = ttk.Frame(pos)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=2).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=9).pack(side=tk.LEFT, fill=tk.X, expand=True)
        row = ttk.Frame(pos)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="ID", width=2).pack(side=tk.LEFT)
        ttk.Spinbox(row, textvariable=self.box_aruco, from_=0, to=9, width=9).pack(side=tk.LEFT)
        ttk.Button(pos, text="Spawn Box Here", command=self._send_spawn_at).pack(fill=tk.X, pady=(8, 0))

        ttk.Button(
            self.box_tab, text="Random Spawn Once",
            command=lambda: self._send("BOX_RANDOM_ONCE"),
        ).pack(fill=tk.X, pady=(12, 4))
        ttk.Button(
            self.box_tab, text="Random ON / OFF",
            command=lambda: self._send("BOX_RANDOM_TOGGLE"),
        ).pack(fill=tk.X, pady=4)
        ttk.Button(
            self.box_tab, text="Auto Queue ON",
            command=lambda: self._send("BOX_AUTO 1"),
        ).pack(fill=tk.X, pady=(16, 4))
        ttk.Button(
            self.box_tab, text="Auto Queue OFF",
            command=lambda: self._send("BOX_AUTO 0"),
        ).pack(fill=tk.X, pady=4)
        ttk.Button(
            self.box_tab, text="Clear All Boxes",
            command=lambda: self._send("BOX_CLEAR"),
        ).pack(fill=tk.X, pady=(16, 4))
        ttk.Label(
            self.box_tab,
            text="Uses the same saved zones and label sizes as the Isaac panel.",
            wraplength=360,
            foreground="#666666",
        ).pack(anchor="w", pady=(18, 0))

    def _build_drone_tab(self):
        ttk.Label(self.drone_tab, text="Drone Controls", font=("Sans", 13, "bold")).pack(anchor="w")
        self.drone_list = ttk.Frame(self.drone_tab)
        self.drone_list.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

    def _ensure_camera(self, key: str, title: str, enabled: bool):
        if key in self.camera_widgets:
            self.camera_vars[key].set(enabled)
            return
        idx = len(self.camera_widgets)
        tile = ttk.Frame(self.camera_grid, padding=4, relief=tk.GROOVE)
        tile.grid(row=idx // 2, column=idx % 2, padx=4, pady=4, sticky="nsew")
        self.camera_grid.columnconfigure(idx % 2, weight=1)

        var = tk.BooleanVar(value=enabled)
        cb = ttk.Checkbutton(
            tile, text=title, variable=var,
            command=lambda k=key, v=var: self._send(f"CAM {k} {1 if v.get() else 0}"),
        )
        cb.pack(anchor="w")
        blank = tk.PhotoImage(width=CAM_W, height=CAM_H)
        img = tk.Label(tile, image=blank, bg="black")
        img.image = blank
        img.pack()
        self.camera_vars[key] = var
        self.camera_widgets[key] = img

    def _ensure_drone(self, name: str):
        if name in self.drone_rows:
            return
        frame = ttk.LabelFrame(self.drone_list, text=name, padding=8)
        frame.pack(fill=tk.X, pady=6)
        x = tk.DoubleVar(value=0.0)
        y = tk.DoubleVar(value=0.0)
        z = tk.DoubleVar(value=1.5)
        for label, var in (("X", x), ("Y", y), ("Z", z)):
            row = ttk.Frame(frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=2).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=10).pack(side=tk.LEFT, fill=tk.X, expand=True)
        buttons = ttk.Frame(frame)
        buttons.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(
            buttons, text="Go",
            command=lambda n=name, vx=x, vy=y, vz=z:
                self._send(f"DRONE_GO {n} {vx.get():.3f} {vy.get():.3f} {vz.get():.3f}"),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        ttk.Button(
            buttons, text="Land",
            command=lambda n=name: self._send(f"DRONE_LAND {n}"),
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))
        self.drone_rows[name] = frame

    # ── Protocol ─────────────────────────────────────────────────────────

    def _stdin_reader(self):
        stdin = sys.stdin.buffer
        while True:
            header = self._read_exact(stdin, 5)
            if header is None:
                break
            msg_type = header[0]
            size = struct.unpack("<I", header[1:5])[0]
            data = self._read_exact(stdin, size) if size else b""
            if data is None:
                break
            if msg_type == 0xFF:
                break
            self.msg_q.put((msg_type, data))

    @staticmethod
    def _read_exact(stream, n: int):
        buf = bytearray()
        while len(buf) < n:
            chunk = stream.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _poll_messages(self):
        try:
            while True:
                msg_type, data = self.msg_q.get_nowait()
                if msg_type == 0x01:
                    self._handle_camera(data)
                elif msg_type == 0x02:
                    self._handle_state(data)
        except queue.Empty:
            pass
        self.root.after(30, self._poll_messages)

    def _handle_state(self, data: bytes):
        try:
            state = json.loads(data.decode("utf-8"))
        except Exception:
            return
        self.fps_var.set(float(state.get("camera_fps", 4.0)))
        for cam in state.get("cameras", []):
            self._ensure_camera(cam["key"], cam["title"], bool(cam.get("enabled", True)))
        for drone in state.get("drones", []):
            self._ensure_drone(drone["name"])

    def _handle_camera(self, data: bytes):
        if len(data) < 5:
            return
        name_len, h, w = struct.unpack("<BHH", data[:5])
        key_start = 5
        key_end = key_start + name_len
        key = data[key_start:key_end].decode("utf-8", errors="ignore")
        raw = data[key_end:]
        if len(raw) != h * w * 3 or key not in self.camera_widgets:
            return
        bgr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
        ok, png = cv2.imencode(".png", bgr)
        if not ok:
            return
        try:
            photo = tk.PhotoImage(
                data=base64.b64encode(png.tobytes()).decode("ascii"),
                format="png",
            )
        except tk.TclError:
            return
        label = self.camera_widgets[key]
        label.configure(image=photo)
        label.image = photo

    def _send_fps(self):
        self._send(f"FPS {self.fps_var.get():.3f}")

    def _send_spawn_at(self):
        self._send(
            "BOX_SPAWN_AT "
            f"{self.box_x.get():.3f} {self.box_y.get():.3f} {self.box_z.get():.3f} "
            f"{self.box_w.get():.3f} {self.box_d.get():.3f} {self.box_h.get():.3f} "
            f"{self.box_aruco.get()}"
        )

    @staticmethod
    def _send(line: str):
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def _on_close(self):
        self._send("SHUTDOWN")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ExternalControlCenter().run()
