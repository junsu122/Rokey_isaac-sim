"""create_codi_box_from2.py — Auto-spawn boxes with zone-based coordinates (Isaac Sim 5.x)

- Zone base coordinate manager (up to 6 zones, add/edit/delete, saved to zone_config.json)
- Zone Selector: checkboxes for Zone 0-5, live label ComboBox per zone
- Random Spawn (once): spawn Count boxes in each checked zone using that zone's label size
- Random ON/OFF toggle: repeat random spawn at Delay interval (manual mode only)
- Auto mode: spawn at external coordinates from spawn_queue.json

Run:
    isaac-python create_codi_box/create_codi_box_from2.py
    isaac-python create_codi_box/create_codi_box_from2.py --usd_path /path/to/scene.usd
"""

import argparse, math, random
from pathlib import Path
import json, time
from collections import deque

_DIR              = Path(__file__).resolve().parent
_QUEUE            = _DIR / "spawn_queue.json"
_LABEL_SIZES_FILE = _DIR / "label_sizes.json"
_ZONE_CONFIG_FILE = _DIR / "zone_config.json"
_PRESET_LABELS    = ["box1_위", "box2_위", "box3_위", "box4_위", "box5_위"]
_MAX_ZONES        = 6

# ── Parse --usd_path ──────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser()
_parser.add_argument("--usd_path", type=str, default=None,
                     help="USD scene file to open instead of default ground plane")
_args, _ = _parser.parse_known_args()

# ── SimulationApp ─────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
})

import carb
_s = carb.settings.get_settings()
_s.set("/app/runLoops/main/rateLimitEnabled",   True)
_s.set("/app/runLoops/main/rateLimitFrequency", 30)
_s.set("/rtx/ambientOcclusion/enabled",         False)
_s.set("/rtx/reflections/enabled",              False)
_s.set("/rtx/shadows/enabled",                  False)
_s.set("/rtx/post/aa/op",                       0)
del _s

import numpy as np
import omni.ui as ui
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


def _angle_to_quat_z(angle_deg: float) -> np.ndarray:
    half = np.radians(angle_deg) / 2.0
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


# ── Stage / World init ────────────────────────────────────────────────────────
if _args.usd_path:
    _usd = Path(_args.usd_path)
    if _usd.exists():
        print(f"[FROM2] Opening USD: {_usd}")
        omni.usd.get_context().open_stage(str(_usd))
        for _ in range(10):
            simulation_app.update()
        world = World()
        world.reset()
        print("[FROM2] USD stage loaded OK")
    else:
        print(f"[FROM2] WARNING: USD not found: {_usd} — using default ground plane")
        _args.usd_path = None

if not _args.usd_path:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()

# Global state
_box_count       = 0
_active_prim_ids: list = []
_spawn_hist      = []
_ui_q: deque     = deque()


# ── UI Panel ──────────────────────────────────────────────────────────────────
class AutoSpawnPanel:
    _BOX_COLOR = np.array([0.25, 0.60, 1.0])

    def __init__(self):
        self._auto       = False
        self._min_dist   = 0.0
        self._log_lines: deque = deque(maxlen=18)
        self._spawn_list: list = []
        self._label_sizes: dict = {}
        self._pending_clear  = False
        self._queue_mtime    = 0.0

        # Zone config  — zone dict: {"name":str, "x":f, "y":f, "z":f, "label_idx":int}
        self._zones: list         = []
        self._spawn_count: int    = 1      # boxes per selected zone per trigger
        self._spawn_radius: float = 0.5
        self._rand_on: bool       = False
        self._rand_delay: float   = 3.0
        self._rand_last_t: float  = 0.0

        # Zone selector UI widgets (pre-built for _MAX_ZONES rows)
        self._zone_checkboxes:   list = []  # ui.CheckBox  ×6
        self._zone_sel_labels:   list = []  # ui.Label     ×6  (zone info text)
        self._zone_label_combos: list = []  # ui.ComboBox  ×6  (live label per zone)
        self._updating_combos: bool = False

        self._load_label_sizes()
        self._load_zone_config()

        self._window = ui.Window("Auto Box Spawn 2 (Zone Mode)", width=520, height=900)
        self._build()
        self._update_label_list()
        self._update_zone_display()

        world.add_physics_callback("panel2_clear_cb", self._physics_step_cb)

    # ── Config I/O ────────────────────────────────────────────────────────────
    def _load_zone_config(self):
        try:
            if _ZONE_CONFIG_FILE.exists():
                d = json.loads(_ZONE_CONFIG_FILE.read_text())
                self._zones        = d.get("zones", [])[:_MAX_ZONES]
                self._spawn_count  = max(1, int(d.get("count", 1)))
                self._spawn_radius = max(0.0, float(d.get("radius", 0.5)))
                self._rand_delay   = max(0.1, float(d.get("rand_delay", 3.0)))
                print(f"[FROM2] Loaded zone config: {len(self._zones)} zones")
        except Exception as e:
            print(f"[FROM2] Zone config load error: {e}")

    def _save_zone_config(self):
        try:
            d = {
                "zones":      self._zones,
                "count":      self._spawn_count,
                "radius":     self._spawn_radius,
                "rand_delay": self._rand_delay,
            }
            tmp = _ZONE_CONFIG_FILE.with_suffix('.tmp')
            tmp.write_text(json.dumps(d, indent=2, ensure_ascii=False))
            tmp.replace(_ZONE_CONFIG_FILE)
        except Exception as e:
            self._log(f"Zone save error: {e}")

    def _load_label_sizes(self):
        try:
            if _LABEL_SIZES_FILE.exists():
                self._label_sizes = json.loads(_LABEL_SIZES_FILE.read_text())
        except Exception as e:
            print(f"[FROM2] Label sizes load error: {e}")

    def _save_label_sizes(self):
        try:
            tmp = _LABEL_SIZES_FILE.with_suffix('.tmp')
            tmp.write_text(json.dumps(self._label_sizes, indent=2, ensure_ascii=False))
            tmp.replace(_LABEL_SIZES_FILE)
        except Exception as e:
            self._log(f"Label save error: {e}")

    # ── Zone management ───────────────────────────────────────────────────────
    def _load_zone_to_fields(self):
        idx = self._if_zsel.model.get_value_as_int()
        if 0 <= idx < len(self._zones):
            z  = self._zones[idx]
            li = z.get("label_idx", 0)
            self._tf_zone_name.model.set_value(z["name"])
            self._ff_zx.model.set_value(z["x"])
            self._ff_zy.model.set_value(z["y"])
            self._ff_zz.model.set_value(z["z"])
            self._combo_zone_label.model.get_item_value_model().set_value(
                max(0, min(li, len(_PRESET_LABELS) - 1)))
            self._log(f"Zone [{idx}] loaded: {z['name']}")
        else:
            self._log(f"Zone [{idx}] not found (total {len(self._zones)})")

    def _add_zone(self):
        if len(self._zones) >= _MAX_ZONES:
            self._log(f"Max {_MAX_ZONES} zones reached")
            return
        name = self._tf_zone_name.model.get_value_as_string().strip()
        if not name:
            name = f"Zone{len(self._zones)}"
        li = self._combo_zone_label.model.get_item_value_model().get_value_as_int()
        z = {
            "name":      name,
            "x":         self._ff_zx.model.get_value_as_float(),
            "y":         self._ff_zy.model.get_value_as_float(),
            "z":         self._ff_zz.model.get_value_as_float(),
            "label_idx": li,
        }
        self._zones.append(z)
        self._update_zone_display()
        self._save_zone_config()
        self._log(f"Zone added [{len(self._zones)-1}]: {name}  "
                  f"X={z['x']:.3f} Y={z['y']:.3f} Z={z['z']:.3f}  "
                  f"label={_PRESET_LABELS[li]}")

    def _update_zone(self):
        idx = self._if_zsel.model.get_value_as_int()
        if 0 <= idx < len(self._zones):
            name = self._tf_zone_name.model.get_value_as_string().strip() or self._zones[idx]["name"]
            li   = self._combo_zone_label.model.get_item_value_model().get_value_as_int()
            self._zones[idx] = {
                "name":      name,
                "x":         self._ff_zx.model.get_value_as_float(),
                "y":         self._ff_zy.model.get_value_as_float(),
                "z":         self._ff_zz.model.get_value_as_float(),
                "label_idx": li,
            }
            self._update_zone_display()
            self._save_zone_config()
            self._log(f"Zone [{idx}] updated: {name}  label={_PRESET_LABELS[li]}")
        else:
            self._log(f"Zone [{idx}] not found")

    def _delete_zone(self):
        idx = self._if_zsel.model.get_value_as_int()
        if 0 <= idx < len(self._zones):
            name = self._zones[idx]["name"]
            self._zones.pop(idx)
            self._update_zone_display()
            self._save_zone_config()
            self._log(f"Zone [{idx}] '{name}' deleted")
        else:
            self._log(f"Zone [{idx}] not found")

    def _on_zone_label_changed(self, zone_idx: int, label_idx: int):
        if self._updating_combos:
            return
        if 0 <= zone_idx < len(self._zones):
            self._zones[zone_idx]["label_idx"] = label_idx
            self._save_zone_config()
            label = _PRESET_LABELS[label_idx] if 0 <= label_idx < len(_PRESET_LABELS) else "?"
            self._log(f"Zone[{zone_idx}] label -> {label}")

    def _update_zone_display(self):
        # Update compact text list
        if not self._zones:
            self._lbl_zones.text = "(no zones)"
        else:
            lines = []
            for i, z in enumerate(self._zones):
                li    = z.get("label_idx", 0)
                label = _PRESET_LABELS[li] if 0 <= li < len(_PRESET_LABELS) else "?"
                lines.append(
                    f"[{i}] {z['name']:<10}  X={z['x']:+.3f}  Y={z['y']:+.3f}  "
                    f"Z={z['z']:+.3f}  [{label}]"
                )
            self._lbl_zones.text = "\n".join(lines)

        # Update zone selector rows (6 pre-built rows)
        self._updating_combos = True
        for zi in range(_MAX_ZONES):
            sel_lbl = self._zone_sel_labels[zi]
            combo   = self._zone_label_combos[zi]
            if zi < len(self._zones):
                z  = self._zones[zi]
                li = z.get("label_idx", 0)
                sel_lbl.text  = f"[{zi}] {z['name']:<10}  ({z['x']:+.2f}, {z['y']:+.2f})"
                sel_lbl.style = {"font_size": 11, "color": 0xFFDDDDDD}
                combo.model.get_item_value_model().set_value(
                    max(0, min(li, len(_PRESET_LABELS) - 1)))
            else:
                sel_lbl.text  = f"Zone[{zi}]  (not added)"
                sel_lbl.style = {"font_size": 11, "color": 0xFF555555}
        self._updating_combos = False

    # ── Zone selector helper ──────────────────────────────────────────────────
    def _get_selected_zones(self) -> list:
        return [
            zi for zi, cb in enumerate(self._zone_checkboxes)
            if zi < len(self._zones) and cb.model.get_value_as_bool()
        ]

    # ── Random ON / OFF toggle ────────────────────────────────────────────────
    def _toggle_rand_on(self):
        if self._auto:
            self._log("Random ON is manual-only — turn off Auto first")
            return
        self._rand_on = not self._rand_on
        if self._rand_on:
            if not self._get_selected_zones():
                self._rand_on = False
                self._log("No zones checked — select at least one zone")
                return
            self._rand_last_t = time.time()
            self._rand_btn.text  = "Random: ON"
            self._rand_btn.style = {"font_size": 14, "background_color": 0xFF1A6B1A}
            self._log(f"Random ON — delay={self._rand_delay:.1f}s")
        else:
            self._rand_btn.text  = "Random: OFF"
            self._rand_btn.style = {"font_size": 14, "background_color": 0xFF444444}
            self._log("Random OFF")

    def _on_delay_changed(self, m):
        self._rand_delay = max(0.1, m.get_value_as_float())
        self._save_zone_config()

    # ── Random spawn ──────────────────────────────────────────────────────────
    def _random_spawn(self):
        selected = self._get_selected_zones()
        if not selected:
            self._log("No zones checked — check zone checkboxes first")
            return
        radius = max(self._ff_radius.model.get_value_as_float(), 0.0)
        self._spawn_radius = radius
        count  = max(1, self._if_count.model.get_value_as_int())

        spawned = 0
        for zi in selected:
            zone  = self._zones[zi]
            # Read label live from the zone's selector combo
            li    = self._zone_label_combos[zi].model.get_item_value_model().get_value_as_int()
            label = _PRESET_LABELS[li] if 0 <= li < len(_PRESET_LABELS) else "obj"
            if label in self._label_sizes:
                sz = self._label_sizes[label]
                bw, bd, bh = sz["w"], sz["d"], sz["h"]
            else:
                bw = max(self._fw.model.get_value_as_float(), 0.01)
                bd = max(self._fd.model.get_value_as_float(), 0.01)
                bh = max(self._fh.model.get_value_as_float(), 0.01)
            for _ in range(count):
                r     = random.uniform(0.0, radius)
                theta = random.uniform(0.0, 2 * math.pi)
                x_m   = zone["x"] + r * math.cos(theta)
                y_m   = zone["y"] + r * math.sin(theta)
                ok    = self._spawn_box_at(x_m, y_m, zone.get("z", 0.0),
                                           bw, bd, bh, label, 0.0)
                if ok:
                    spawned += 1

        self._lbl_n.text = f"{len(_active_prim_ids)} boxes"
        if spawned:
            self._log(f"Random spawn: {spawned} box(es) across zones {selected}")

    def _spawn_box_at(self, x_m: float, y_m: float, z_off: float,
                      bw: float, bd: float, bh: float,
                      label: str, angle_deg: float = 0.0) -> bool:
        global _box_count, _active_prim_ids
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        if self._too_close(x_mm, y_mm):
            return False
        z_m = z_off + bh / 2.0
        _box_count += 1
        _active_prim_ids.append(_box_count)
        _spawn_hist.append((x_mm, y_mm))
        world.scene.add(DynamicCuboid(
            prim_path=f"/World/AutoBox_{_box_count:04d}",
            name=f"autobox_{_box_count:04d}",
            position=np.array([x_m, y_m, z_m]),
            orientation=_angle_to_quat_z(angle_deg),
            scale=np.array([bw, bd, bh]),
            color=self._BOX_COLOR,
            mass=1.0,
        ))
        display_n = len(_active_prim_ids)
        self._spawn_list.append((display_n, label, x_mm, y_mm, angle_deg, bw, bd, bh))
        self._update_box_list()
        self._log(f"Spawn #{display_n}  {label}  ({x_mm:.0f},{y_mm:.0f}) mm  "
                  f"[{bw:.2f}x{bd:.2f}x{bh:.2f}]")
        return True

    # ── UI build ──────────────────────────────────────────────────────────────
    def _build(self):
        with self._window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=5, style={"margin": 8}):

                    ui.Label("Isaac Sim Auto Box Spawner (Zone Mode)",
                             style={"font_size": 16, "color": 0xFFFFDD88})

                    # ── Control buttons ───────────────────────────────────────
                    with ui.HStack(spacing=4, height=44):
                        self._auto_btn = ui.Button(
                            "Auto: OFF", width=130, height=44,
                            style={"font_size": 15, "background_color": 0xFF444444})
                        self._auto_btn.set_clicked_fn(self._toggle_auto)
                        clear_btn = ui.Button("Clear All Boxes", width=140, height=44)
                        clear_btn.set_clicked_fn(self._clear_all_boxes)
                        q_btn = ui.Button("Clear Queue", width=110, height=44)
                        q_btn.set_clicked_fn(self._clear_queue)

                    # ── Default size / distance ───────────────────────────────
                    ui.Label("─── Default Size / Distance ───",
                             style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.HStack(spacing=6):
                        ui.Label("Min Distance (mm):", width=150)
                        self._f_dist = ui.FloatField(width=80)
                        self._f_dist.model.set_value(0.0)
                        self._f_dist.model.add_value_changed_fn(
                            lambda m: setattr(self, "_min_dist", m.get_value_as_float()))
                    with ui.HStack(spacing=6):
                        ui.Label("Default W / D / H (m):", width=150)
                        self._fw = ui.FloatField(width=60); self._fw.model.set_value(0.30)
                        self._fd = ui.FloatField(width=60); self._fd.model.set_value(0.30)
                        self._fh = ui.FloatField(width=60); self._fh.model.set_value(0.30)

                    # ── Label size config ─────────────────────────────────────
                    ui.Label("─── Label Size Config ───",
                             style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.HStack(spacing=4, height=30):
                        ui.Label("Label:", width=45)
                        self._combo_labels = ui.ComboBox(0, *_PRESET_LABELS)
                        load_btn = ui.Button("Load", width=65, height=30)
                        load_btn.set_clicked_fn(self._load_selected_label)
                    with ui.HStack(spacing=4):
                        ui.Label("W:", width=20); self._sf_w = ui.FloatField(width=70); self._sf_w.model.set_value(0.30)
                        ui.Label("D:", width=20); self._sf_d = ui.FloatField(width=70); self._sf_d.model.set_value(0.30)
                        ui.Label("H:", width=20); self._sf_h = ui.FloatField(width=70); self._sf_h.model.set_value(0.30)
                    with ui.HStack(spacing=4, height=32):
                        set_btn = ui.Button("Set Size", width=100, height=32)
                        set_btn.set_clicked_fn(self._set_label_size)
                        clr_lbl_btn = ui.Button("Clear Label Configs", width=150, height=32)
                        clr_lbl_btn.set_clicked_fn(self._clear_label_configs)
                    with ui.ScrollingFrame(height=75):
                        self._lbl_label_list = ui.Label(
                            "(no label configs)",
                            style={"background_color": 0xFF0A0A14, "color": 0xFFFFDD88, "font_size": 11},
                            word_wrap=True)

                    # ── Zone Base Coordinate Manager ──────────────────────────
                    ui.Label("─── Zone Base Coordinate Manager ───",
                             style={"font_size": 13, "color": 0xFFFFAA44})
                    ui.Label("  Index (0-based) -> Load -> Edit  |  Max 6 zones",
                             style={"font_size": 11, "color": 0xFFAAAAAA})
                    with ui.HStack(spacing=4):
                        ui.Label("Zone Index:", width=80)
                        self._if_zsel = ui.IntField(width=55)
                        self._if_zsel.model.set_value(0)
                        load_z_btn = ui.Button("Load", width=75, height=28)
                        load_z_btn.set_clicked_fn(self._load_zone_to_fields)
                    with ui.HStack(spacing=4):
                        ui.Label("Name:", width=40)
                        self._tf_zone_name = ui.StringField(width=130)
                        self._tf_zone_name.model.set_value("Zone0")
                    with ui.HStack(spacing=4):
                        ui.Label("X(m):", width=42); self._ff_zx = ui.FloatField(width=72); self._ff_zx.model.set_value(0.0)
                        ui.Label("Y(m):", width=42); self._ff_zy = ui.FloatField(width=72); self._ff_zy.model.set_value(0.0)
                        ui.Label("Z(m):", width=42); self._ff_zz = ui.FloatField(width=72); self._ff_zz.model.set_value(0.0)
                    with ui.HStack(spacing=4):
                        ui.Label("Label:", width=42)
                        self._combo_zone_label = ui.ComboBox(0, *_PRESET_LABELS)
                    with ui.HStack(spacing=4, height=32):
                        add_z_btn = ui.Button("Add Zone", width=95, height=32)
                        add_z_btn.set_clicked_fn(self._add_zone)
                        upd_z_btn = ui.Button("Edit Zone", width=95, height=32)
                        upd_z_btn.set_clicked_fn(self._update_zone)
                        del_z_btn = ui.Button("Del Zone", width=95, height=32)
                        del_z_btn.set_clicked_fn(self._delete_zone)
                    with ui.ScrollingFrame(height=80):
                        self._lbl_zones = ui.Label(
                            "(no zones)",
                            style={"background_color": 0xFF081408, "color": 0xFF88FF88, "font_size": 11},
                            word_wrap=True)

                    # ── Zone Selector for Random Spawn ────────────────────────
                    ui.Label("─── Zone Selector for Random Spawn ───",
                             style={"font_size": 13, "color": 0xFFFFAA44})
                    ui.Label("  Check zones  |  Change label live  |  Uses label's W/D/H",
                             style={"font_size": 11, "color": 0xFFAAAAAA})

                    for zi in range(_MAX_ZONES):
                        with ui.HStack(spacing=4, height=28):
                            cb = ui.CheckBox(width=20, height=20)
                            cb.model.set_value(False)
                            self._zone_checkboxes.append(cb)
                            sel_lbl = ui.Label(
                                f"Zone[{zi}]  (not added)", width=190,
                                style={"font_size": 11, "color": 0xFF555555})
                            self._zone_sel_labels.append(sel_lbl)
                            ui.Label("Label:", width=40,
                                     style={"font_size": 11, "color": 0xFFAAAAAA})
                            combo = ui.ComboBox(0, *_PRESET_LABELS)
                            combo.model.get_item_value_model().add_value_changed_fn(
                                lambda m, z=zi: self._on_zone_label_changed(z, m.get_value_as_int()))
                            self._zone_label_combos.append(combo)

                    # ── Random Spawn controls ─────────────────────────────────
                    ui.Label("─── Random Spawn Settings ───",
                             style={"font_size": 13, "color": 0xFFFFAA44})
                    with ui.HStack(spacing=6):
                        ui.Label("Count/Zone:", width=80)
                        self._if_count = ui.IntField(width=50)
                        self._if_count.model.set_value(self._spawn_count)
                        self._if_count.model.add_value_changed_fn(
                            lambda m: setattr(self, "_spawn_count", max(1, m.get_value_as_int())))
                        ui.Label("Radius(m):", width=72)
                        self._ff_radius = ui.FloatField(width=65)
                        self._ff_radius.model.set_value(self._spawn_radius)
                        self._ff_radius.model.add_value_changed_fn(
                            lambda m: setattr(self, "_spawn_radius", m.get_value_as_float()))

                    with ui.HStack(spacing=6, height=40):
                        self._rand_btn = ui.Button(
                            "Random: OFF", width=145, height=40,
                            style={"font_size": 14, "background_color": 0xFF444444})
                        self._rand_btn.set_clicked_fn(self._toggle_rand_on)
                        ui.Label("Delay(s):", width=65)
                        self._ff_delay = ui.FloatField(width=65)
                        self._ff_delay.model.set_value(self._rand_delay)
                        self._ff_delay.model.add_value_changed_fn(self._on_delay_changed)

                    rand_once_btn = ui.Button(
                        "Random Spawn (once)", height=36,
                        style={"font_size": 13, "background_color": 0xFF1A3A1A})
                    rand_once_btn.set_clicked_fn(self._random_spawn)

                    # ── Status ────────────────────────────────────────────────
                    ui.Label("─── Status ───",
                             style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.HStack(spacing=10):
                        ui.Label("Queue:", width=70)
                        self._lbl_q = ui.Label("0 items", style={"color": 0xFFFFCC44})
                    with ui.HStack(spacing=10):
                        ui.Label("Spawned:", width=70)
                        self._lbl_n = ui.Label("0 boxes", style={"color": 0xFF44FF88})
                    self._lbl_last = ui.Label(
                        "Last action: waiting...",
                        style={"font_size": 11, "color": 0xFF999999})

                    # ── Log ───────────────────────────────────────────────────
                    ui.Label("─── Log ───", style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.ScrollingFrame(height=140):
                        self._lbl_log = ui.Label(
                            "",
                            style={"background_color": 0xFF0D0D1A, "color": 0xFF33FF99, "font_size": 11},
                            word_wrap=True)

                    # ── Spawned box list ──────────────────────────────────────
                    ui.Label("─── Spawned Box List ───",
                             style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.ScrollingFrame(height=200):
                        self._lbl_boxes = ui.Label(
                            "(no boxes yet)",
                            style={"background_color": 0xFF0A0A14, "color": 0xFFCCEEFF, "font_size": 11},
                            word_wrap=True)

    # ── Core methods ──────────────────────────────────────────────────────────
    def _toggle_auto(self):
        self._auto = not self._auto
        if self._auto:
            if self._rand_on:
                self._rand_on = False
                self._rand_btn.text  = "Random: OFF"
                self._rand_btn.style = {"font_size": 14, "background_color": 0xFF444444}
            try:
                if _QUEUE.exists():
                    _QUEUE.write_text("[]")
            except Exception:
                pass
            self._auto_btn.text  = "Auto: ON"
            self._auto_btn.style = {"font_size": 15, "background_color": 0xFF1A6B1A}
            self._log("Auto mode ON — queue flushed")
        else:
            self._auto_btn.text  = "Auto: OFF"
            self._auto_btn.style = {"font_size": 15, "background_color": 0xFF444444}
            self._log("Auto mode OFF")

    def _clear_all_boxes(self):
        self._pending_clear = True
        self._log("Clear requested — removing on next step...")

    def _physics_step_cb(self, _step_size: float):
        if not self._pending_clear:
            return
        global _spawn_hist, _active_prim_ids
        self._pending_clear = False
        removed = 0
        for pid in list(_active_prim_ids):
            name = f"autobox_{pid:04d}"
            try:
                if world.scene.object_exists(name):
                    world.scene.remove_object(name)
                    removed += 1
            except Exception as e:
                print(f"[FROM2] Remove err #{pid}: {e}")
        _active_prim_ids.clear()
        _spawn_hist.clear()
        self._spawn_list.clear()
        n = removed
        _ui_q.append(lambda: setattr(self._lbl_n,    "text", "0 boxes"))
        _ui_q.append(lambda: setattr(self._lbl_boxes, "text", "(no boxes yet)"))
        _ui_q.append(lambda: self._log(f"Cleared {n} boxes"))

    def _clear_queue(self):
        try:
            if _QUEUE.exists():
                _QUEUE.unlink()
            _ui_q.append(lambda: setattr(self._lbl_q, "text", "0 items"))
            self._log("Queue file cleared")
        except Exception as e:
            self._log(f"Error clearing queue: {e}")

    def _load_selected_label(self):
        idx = self._combo_labels.model.get_item_value_model().get_value_as_int()
        label = _PRESET_LABELS[idx]
        if label in self._label_sizes:
            sz = self._label_sizes[label]
            self._sf_w.model.set_value(sz["w"])
            self._sf_d.model.set_value(sz["d"])
            self._sf_h.model.set_value(sz["h"])
            self._log(f"Loaded '{label}': W={sz['w']:.3f} D={sz['d']:.3f} H={sz['h']:.3f} m")
        else:
            self._log(f"No saved config for '{label}'")

    def _set_label_size(self):
        idx = self._combo_labels.model.get_item_value_model().get_value_as_int()
        label = _PRESET_LABELS[idx]
        w = max(self._sf_w.model.get_value_as_float(), 0.01)
        d = max(self._sf_d.model.get_value_as_float(), 0.01)
        h = max(self._sf_h.model.get_value_as_float(), 0.01)
        self._label_sizes[label] = {"w": w, "d": d, "h": h}
        self._update_label_list()
        self._save_label_sizes()
        self._log(f"Saved '{label}': W={w:.3f} D={d:.3f} H={h:.3f} m")

    def _update_label_list(self):
        if not self._label_sizes:
            self._lbl_label_list.text = "(no label configs)"
            return
        lines = []
        for lbl in _PRESET_LABELS:
            if lbl in self._label_sizes:
                sz = self._label_sizes[lbl]
                lines.append(f"{lbl:<14} W={sz['w']:.3f}  D={sz['d']:.3f}  H={sz['h']:.3f} m")
            else:
                lines.append(f"{lbl:<14} (not set)")
        self._lbl_label_list.text = "\n".join(lines)

    def _clear_label_configs(self):
        self._label_sizes.clear()
        self._update_label_list()
        self._save_label_sizes()
        self._log("Label size configs cleared")

    def _too_close(self, x_mm: float, y_mm: float) -> bool:
        for hx, hy in _spawn_hist:
            if np.hypot(x_mm - hx, y_mm - hy) < self._min_dist:
                return True
        return False

    def _spawn_box(self, x_mm: float, y_mm: float,
                   bw: float, bd: float, bh: float,
                   label: str, angle_deg: float = 0.0) -> bool:
        global _box_count, _active_prim_ids
        if self._too_close(x_mm, y_mm):
            self._log(f"Skip {label} ({x_mm:.0f},{y_mm:.0f}) mm — too close")
            return False
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        z_m = bh / 2.0
        _box_count += 1
        _active_prim_ids.append(_box_count)
        _spawn_hist.append((x_mm, y_mm))
        world.scene.add(DynamicCuboid(
            prim_path=f"/World/AutoBox_{_box_count:04d}",
            name=f"autobox_{_box_count:04d}",
            position=np.array([x_m, y_m, z_m]),
            orientation=_angle_to_quat_z(angle_deg),
            scale=np.array([bw, bd, bh]),
            color=self._BOX_COLOR,
            mass=1.0,
        ))
        display_n = len(_active_prim_ids)
        self._spawn_list.append((display_n, label, x_mm, y_mm, angle_deg, bw, bd, bh))
        self._update_box_list()
        self._log(f"Spawn #{display_n}  {label}  ({x_mm:.0f},{y_mm:.0f}) mm  "
                  f"a={angle_deg:.1f}  [{bw:.2f}x{bd:.2f}x{bh:.2f}]")
        return True

    def _process_queue(self):
        if not _QUEUE.exists():
            self._lbl_q.text = "0 items"
            return
        try:
            mtime = _QUEUE.stat().st_mtime
            if mtime == self._queue_mtime:
                return
            self._queue_mtime = mtime
        except Exception:
            pass
        try:
            raw  = _QUEUE.read_text().strip()
            data = json.loads(raw) if raw else []
        except Exception as e:
            self._log(f"Queue read error: {e}")
            return
        if not data:
            self._lbl_q.text = "0 items"
            return

        self._lbl_q.text = f"{len(data)} items"
        def_w = max(self._fw.model.get_value_as_float(), 0.01)
        def_d = max(self._fd.model.get_value_as_float(), 0.01)
        def_h = max(self._fh.model.get_value_as_float(), 0.01)

        spawned = 0
        for entry in data:
            label = entry.get("label", "obj")
            if label not in self._label_sizes:
                self._label_sizes[label] = {"w": def_w, "d": def_d, "h": def_h}
                self._update_label_list()
                self._save_label_sizes()
            sz = self._label_sizes[label]
            bw, bd, bh = sz["w"], sz["d"], sz["h"]
            x_mm  = entry.get("x_mm", 0.0)
            y_mm  = entry.get("y_mm", 0.0)
            angle = entry.get("angle_deg", 0.0)
            ok = self._spawn_box(x_mm, y_mm, bw, bd, bh, label, angle)
            if ok:
                spawned += 1

        try:
            _QUEUE.write_text("[]")
        except Exception:
            pass

        self._lbl_q.text = "0 items"
        self._lbl_n.text = f"{len(_active_prim_ids)} boxes"
        if spawned:
            self._lbl_last.text = f"Last: +{spawned} box(es) spawned"

    def _update_box_list(self):
        if not self._spawn_list:
            self._lbl_boxes.text = "(no boxes yet)"
            return
        lines = [
            f"#{idx:04d}  {label:<12}  ({x_mm:+8.1f},{y_mm:+8.1f}) mm  "
            f"a={a:+6.1f}  [{bw:.2f}x{bd:.2f}x{bh:.2f}]"
            for idx, label, x_mm, y_mm, a, bw, bd, bh in self._spawn_list
        ]
        self._lbl_boxes.text = "\n".join(lines)

    def tick(self):
        while _ui_q:
            _ui_q.popleft()()
        if self._auto:
            self._process_queue()
        elif self._rand_on:
            now = time.time()
            if now - self._rand_last_t >= self._rand_delay:
                self._rand_last_t = now
                self._random_spawn()

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self._log_lines.append(f"[{ts}] {msg}")
        self._lbl_last.text = f"Last: {msg}"
        self._lbl_log.text  = "\n".join(self._log_lines)
        print(f"[FROM2] {msg}")


# ── Main ──────────────────────────────────────────────────────────────────────
try:
    panel  = AutoSpawnPanel()
    _frame = 0
    _RENDER_EVERY = 2

    while simulation_app.is_running():
        _frame += 1
        world.step(render=(_frame % _RENDER_EVERY == 0))
        if _frame % 60 == 0:
            panel.tick()

except Exception as e:
    print(f"[ERROR] Simulation loop error: {e}")

finally:
    simulation_app.close()
