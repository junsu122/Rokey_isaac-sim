"""
main_isaac/auto_spawn_panel.py
================================
create_codi_box_from2.py 에서 AutoSpawnPanel 만 추출한 모듈.
SimulationApp / World 생성 없이 main.py 의 world 를 주입받아 동작한다.

사용:
    from auto_spawn_panel import AutoSpawnPanel
    panel = AutoSpawnPanel(my_world)          # main.py 에서 호출
    panel.tick()                              # 메인 루프에서 주기적 호출
"""

import math, random, json, time
from pathlib import Path
from collections import deque

import numpy as np
import omni.ui as ui
import omni.usd
from pxr import UsdGeom, UsdPhysics, UsdShade, Gf, Sdf

# ── 설정 파일 경로 (원본 위치 그대로 사용) ────────────────────────────────
_CONFIG_DIR       = Path(__file__).parent
_QUEUE            = _CONFIG_DIR / "spawn_queue.json"
_LABEL_SIZES_FILE = _CONFIG_DIR / "label_sizes.json"
_ZONE_CONFIG_FILE = _CONFIG_DIR / "zone_config.json"

# ── ArUco 텍스처 경로 ──────────────────────────────────────────────────────
_ARUCO_TEX_DIR = (
    Path(__file__).parent
    / "robots" / "m0609" / "m0609_aruco_detect" / "aruco_marker_6x6"
)
_ARUCO_ID_MAX  = 9      # aruco_id0.png ~ aruco_id9.png

_PRESET_LABELS = ["box1_위", "box2_위", "box3_위", "box4_위", "box5_위"]
_MAX_ZONES     = 6

# ── 모듈 레벨 상태 ─────────────────────────────────────────────────────────
_box_count       = 0
_active_prim_ids: list = []
_spawn_hist      = []
_ui_q: deque     = deque()


def _angle_to_quat_z(angle_deg: float) -> np.ndarray:
    half = np.radians(angle_deg) / 2.0
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])



def _create_box_with_aruco(prim_path: str,
                            x_m: float, y_m: float, z_m: float,
                            bw: float, bd: float, bh: float,
                            color_rgb: tuple, mass: float,
                            orientation_wxyz: tuple,
                            aruco_id: int) -> None:
    """
    Physics 박스를 USD 직접 생성 + 윗면에 ArUco 텍스처 plane 부착.
    prim_path 하위 구조:
        prim_path/           ← Xform + RigidBodyAPI  (root)
        prim_path/box        ← Cube  + CollisionAPI  (물리 박스)
        prim_path/aruco_top  ← Mesh  (텍스처 plane, 물리 없음)
        prim_path/aruco_mat  ← Material
    """
    stage = omni.usd.get_context().get_stage()
    w, x, y, z = orientation_wxyz

    # ── Root Xform (RigidBody) ───────────────────────────────────
    root = stage.DefinePrim(prim_path, "Xform")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).GetMassAttr().Set(float(mass))

    xf = UsdGeom.Xformable(root)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x_m, y_m, z_m))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(w, x, y, z))

    # ── Box body (scale 적용된 Cube) ─────────────────────────────
    box_path = f"{prim_path}/box"
    cube = UsdGeom.Cube.Define(stage, box_path)
    cube.CreateSizeAttr(1.0)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    box_xf = UsdGeom.Xformable(cube.GetPrim())
    box_xf.ClearXformOpOrder()
    box_xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(bw, bd, bh))
    cube.GetPrim().CreateAttribute(
        "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
    ).Set([Gf.Vec3f(*color_rgb)])

    # ── ArUco 텍스처 plane (윗면, root 기준 로컬 좌표) ─────────────
    # root에 scale 없으므로 로컬 좌표 = 월드 오프셋
    # 박스 top face: z = bh/2  → plane을 2 mm 위에 배치
    hw = bw * 0.45   # 90% of face width
    hd = bd * 0.45
    zt = bh / 2.0 + 0.002

    plane_path = f"{prim_path}/aruco_top"
    mesh = UsdGeom.Mesh.Define(stage, plane_path)
    mesh.CreatePointsAttr([
        Gf.Vec3f(-hw, -hd, zt),
        Gf.Vec3f( hw, -hd, zt),
        Gf.Vec3f( hw,  hd, zt),
        Gf.Vec3f(-hw,  hd, zt),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)

    pv = UsdGeom.PrimvarsAPI(mesh)
    st = pv.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                          UsdGeom.Tokens.faceVarying)
    st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    # ── Material with ArUco texture ──────────────────────────────
    tex_path = str(_ARUCO_TEX_DIR / f"aruco_id{aruco_id}.png")
    mat_path = f"{prim_path}/aruco_mat"
    mat = UsdShade.Material.Define(stage, mat_path)

    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(0.0)

    uv_reader = UsdShade.Shader.Define(stage, f"{mat_path}/UVReader")
    uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
    uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_out = uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex_sh = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex_sh.CreateIdAttr("UsdUVTexture")
    tex_sh.CreateInput("file",  Sdf.ValueTypeNames.Asset).Set(tex_path)
    tex_sh.CreateInput("st",    Sdf.ValueTypeNames.Float2).ConnectToSource(uv_out)
    tex_sh.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex_sh.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex_out = tex_sh.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_out)
    surf_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(surf_out)

    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)


class AutoSpawnPanel:
    _BOX_COLOR = np.array([0.25, 0.60, 1.0])

    def __init__(self, world, build_window: bool = True):
        """
        world : main.py 의 World 인스턴스를 그대로 전달
        """
        self._world = world

        self._auto       = False
        self._min_dist   = 0.0
        self._log_lines: deque = deque(maxlen=18)
        self._spawn_list: list = []
        self._label_sizes: dict = {}
        self._pending_clear  = False
        self._queue_mtime    = 0.0

        self._zones: list         = []
        self._spawn_count: int    = 1
        self._spawn_radius: float = 0.5
        self._rand_on: bool       = False
        self._rand_delay: float   = 3.0
        self._rand_last_t: float  = 0.0

        self._zone_checkboxes:   list = []
        self._zone_sel_labels:   list = []
        self._zone_label_combos: list = []
        self._updating_combos: bool = False
        self._ui_ready: bool = False

        self._load_label_sizes()
        self._load_zone_config()

        self._window = (
            ui.Window("Auto Box Spawn 2 (Zone Mode)", width=520, height=900)
            if build_window else None
        )
        if self._window is not None:
            with self._window.frame:
                self.build_ui()

        # physics callback 은 main.py 의 world 에 등록
        self._world.add_physics_callback("panel2_clear_cb", self._physics_step_cb)
        print("[AutoSpawnPanel] 초기화 완료")

    # ── Config I/O ────────────────────────────────────────────────────────────
    def _load_zone_config(self):
        try:
            if _ZONE_CONFIG_FILE.exists():
                d = json.loads(_ZONE_CONFIG_FILE.read_text())
                self._zones        = d.get("zones", [])[:_MAX_ZONES]
                self._spawn_count  = max(1, int(d.get("count", 1)))
                self._spawn_radius = max(0.0, float(d.get("radius", 0.5)))
                self._rand_delay   = max(0.1, float(d.get("rand_delay", 3.0)))
        except Exception as e:
            print(f"[AutoSpawnPanel] Zone config load error: {e}")

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
            print(f"[AutoSpawnPanel] Label sizes load error: {e}")

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
            li    = self._zone_label_combos[zi].model.get_item_value_model().get_value_as_int()
            label = _PRESET_LABELS[li] if 0 <= li < len(_PRESET_LABELS) else "obj"
            if label in self._label_sizes:
                sz = self._label_sizes[label]
                bw, bd, bh = sz["w"], sz["d"], sz["h"]
            else:
                bw = max(self._fw.model.get_value_as_float(), 0.01)
                bd = max(self._fd.model.get_value_as_float(), 0.01)
                bh = max(self._fh.model.get_value_as_float(), 0.01)
            # zone 인덱스 = ArUco ID (zone0→id0, zone1→id1, zone2→id2, …)
            aruco_id = zi % (_ARUCO_ID_MAX + 1)
            for _ in range(count):
                r     = random.uniform(0.0, radius)
                theta = random.uniform(0.0, 2 * math.pi)
                x_m   = zone["x"] + r * math.cos(theta)
                y_m   = zone["y"] + r * math.sin(theta)
                ok    = self._spawn_box_at(x_m, y_m, zone.get("z", 0.0),
                                           bw, bd, bh, label, 0.0,
                                           aruco_id=aruco_id)
                if ok:
                    spawned += 1

        self._lbl_n.text = f"{len(_active_prim_ids)} boxes"
        if spawned:
            self._log(f"Random spawn: {spawned} box(es) across zones {selected}")

    def _spawn_box_at(self, x_m, y_m, z_off, bw, bd, bh, label,
                      angle_deg=0.0, aruco_id=0):
        global _box_count, _active_prim_ids
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        if self._too_close(x_mm, y_mm):
            return False
        z_m = z_off + bh / 2.0
        _box_count += 1
        _active_prim_ids.append(_box_count)
        _spawn_hist.append((x_mm, y_mm))

        q = _angle_to_quat_z(angle_deg)   # [w, x, y, z]
        aid = int(aruco_id) % (_ARUCO_ID_MAX + 1)
        _create_box_with_aruco(
            prim_path=f"/World/AutoBox_{_box_count:04d}",
            x_m=x_m, y_m=y_m, z_m=z_m,
            bw=bw, bd=bd, bh=bh,
            color_rgb=tuple(self._BOX_COLOR),
            mass=3.0,
            orientation_wxyz=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            aruco_id=aid,
        )
        display_n = len(_active_prim_ids)
        self._spawn_list.append((display_n, label, x_mm, y_mm, angle_deg, bw, bd, bh))
        self._update_box_list()
        self._log(f"Spawn #{display_n}  {label}  ({x_mm:.0f},{y_mm:.0f}) mm  "
                  f"[{bw:.2f}x{bd:.2f}x{bh:.2f}]  ArUco ID={aid}")
        return True

    # ── UI build ──────────────────────────────────────────────────────────────
    def build_ui(self):
        self._ui_ready = True
        with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
            with ui.VStack(spacing=5, style={"margin": 8}):
                    ui.Label("Isaac Sim Auto Box Spawner (Zone Mode)",
                             style={"font_size": 16, "color": 0xFFFFDD88})

                    with ui.HStack(spacing=4, height=44):
                        self._auto_btn = ui.Button(
                            "Auto: OFF", width=130, height=44,
                            style={"font_size": 15, "background_color": 0xFF444444})
                        self._auto_btn.set_clicked_fn(self._toggle_auto)
                        clear_btn = ui.Button("Clear All Boxes", width=140, height=44)
                        clear_btn.set_clicked_fn(self._clear_all_boxes)
                        q_btn = ui.Button("Clear Queue", width=110, height=44)
                        q_btn.set_clicked_fn(self._clear_queue)

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

                    ui.Label("─── Log ───", style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.ScrollingFrame(height=140):
                        self._lbl_log = ui.Label(
                            "",
                            style={"background_color": 0xFF0D0D1A, "color": 0xFF33FF99, "font_size": 11},
                            word_wrap=True)

                    ui.Label("─── Spawned Box List ───",
                             style={"font_size": 13, "color": 0xFF88CCFF})
                    with ui.ScrollingFrame(height=200):
                        self._lbl_boxes = ui.Label(
                            "(no boxes yet)",
                            style={"background_color": 0xFF0A0A14, "color": 0xFFCCEEFF, "font_size": 11},
                            word_wrap=True)
        self._update_label_list()
        self._update_zone_display()

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
        stage = omni.usd.get_context().get_stage()
        for pid in list(_active_prim_ids):
            prim_path = f"/World/AutoBox_{pid:04d}"
            try:
                if stage.GetPrimAtPath(prim_path).IsValid():
                    stage.RemovePrim(prim_path)
                    removed += 1
            except Exception as e:
                print(f"[AutoSpawnPanel] Remove err #{pid}: {e}")
        _active_prim_ids.clear()
        _spawn_hist.clear()
        self._spawn_list.clear()
        n = removed
        if self._ui_ready:
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

    def _too_close(self, x_mm, y_mm):
        for hx, hy in _spawn_hist:
            if np.hypot(x_mm - hx, y_mm - hy) < self._min_dist:
                return True
        return False

    def _spawn_box(self, x_mm, y_mm, bw, bd, bh, label,
                   angle_deg=0.0, aruco_id=0):
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

        q = _angle_to_quat_z(angle_deg)
        aid = int(aruco_id) % (_ARUCO_ID_MAX + 1)
        _create_box_with_aruco(
            prim_path=f"/World/AutoBox_{_box_count:04d}",
            x_m=x_m, y_m=y_m, z_m=z_m,
            bw=bw, bd=bd, bh=bh,
            color_rgb=tuple(self._BOX_COLOR),
            mass=3.0,
            orientation_wxyz=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            aruco_id=aid,
        )
        display_n = len(_active_prim_ids)
        self._spawn_list.append((display_n, label, x_mm, y_mm, angle_deg, bw, bd, bh))
        self._update_box_list()
        self._log(f"Spawn #{display_n}  {label}  ({x_mm:.0f},{y_mm:.0f}) mm  "
                  f"a={angle_deg:.1f}  [{bw:.2f}x{bd:.2f}x{bh:.2f}]  ArUco ID={aid}")
        return True

    def _process_queue(self):
        if not _QUEUE.exists():
            if self._ui_ready:
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
            if self._ui_ready:
                self._lbl_q.text = "0 items"
            return

        if self._ui_ready:
            self._lbl_q.text = f"{len(data)} items"
        def_w = max(self._fw.model.get_value_as_float(), 0.01) if self._ui_ready else 0.30
        def_d = max(self._fd.model.get_value_as_float(), 0.01) if self._ui_ready else 0.30
        def_h = max(self._fh.model.get_value_as_float(), 0.01) if self._ui_ready else 0.30

        spawned = 0
        for entry in data:
            label = entry.get("label", "obj")
            if label not in self._label_sizes:
                self._label_sizes[label] = {"w": def_w, "d": def_d, "h": def_h}
                self._update_label_list()
                self._save_label_sizes()
            sz = self._label_sizes[label]
            bw, bd, bh = sz["w"], sz["d"], sz["h"]
            x_mm     = entry.get("x_mm", 0.0)
            y_mm     = entry.get("y_mm", 0.0)
            angle    = entry.get("angle_deg", 0.0)
            aruco_id = entry.get("aruco_id", entry.get("zone_id", 0))
            ok = self._spawn_box(x_mm, y_mm, bw, bd, bh, label, angle,
                                 aruco_id=aruco_id)
            if ok:
                spawned += 1

        try:
            _QUEUE.write_text("[]")
        except Exception:
            pass

        if self._ui_ready:
            self._lbl_q.text = "0 items"
            self._lbl_n.text = f"{len(_active_prim_ids)} boxes"
        if spawned and self._ui_ready:
            self._lbl_last.text = f"Last: +{spawned} box(es) spawned"
        elif spawned:
            self._log(f"Queue spawned {spawned} box(es)")

    def _update_box_list(self):
        if not self._ui_ready:
            return
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
        """메인 루프에서 주기적으로 호출 (약 60 frame 마다 권장)."""
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
        if self._ui_ready:
            self._lbl_last.text = f"Last: {msg}"
            self._lbl_log.text  = "\n".join(self._log_lines)
        print(f"[AutoSpawnPanel] {msg}")

    # ── External control center API ─────────────────────────────────────────
    def external_tick(self):
        while _ui_q:
            _ui_q.popleft()()
        if self._auto:
            self._process_queue()
        elif self._rand_on:
            now = time.time()
            if now - self._rand_last_t >= self._rand_delay:
                self._rand_last_t = now
                self.external_random_spawn_once()

    def external_set_auto(self, enabled: bool):
        self._auto = bool(enabled)
        if self._auto:
            self._rand_on = False
        self._log(f"External auto {'ON' if self._auto else 'OFF'}")

    def external_toggle_random(self):
        self._rand_on = not self._rand_on
        if self._rand_on:
            self._auto = False
            self._rand_last_t = time.time()
        self._log(f"External random {'ON' if self._rand_on else 'OFF'}")

    def external_random_spawn_once(self):
        selected = list(range(len(self._zones))) if self._zones else [0]
        if not self._zones:
            self._zones = [{
                "name": "Zone0",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "label_idx": 0,
            }]

        spawned = 0
        for zi in selected:
            zone = self._zones[zi]
            li = zone.get("label_idx", 0)
            label = _PRESET_LABELS[li] if 0 <= li < len(_PRESET_LABELS) else "obj"
            sz = self._label_sizes.get(label, {"w": 0.30, "d": 0.30, "h": 0.30})
            count = max(1, int(self._spawn_count))
            radius = max(0.0, float(self._spawn_radius))
            for _ in range(count):
                r = random.uniform(0.0, radius)
                theta = random.uniform(0.0, 2 * math.pi)
                x_m = zone["x"] + r * math.cos(theta)
                y_m = zone["y"] + r * math.sin(theta)
                if self._spawn_box_at(
                    x_m, y_m, zone.get("z", 0.0),
                    sz["w"], sz["d"], sz["h"], label, 0.0,
                    aruco_id=zi % (_ARUCO_ID_MAX + 1),
                ):
                    spawned += 1
        if self._ui_ready:
            self._lbl_n.text = f"{len(_active_prim_ids)} boxes"
        self._log(f"External random spawn: {spawned} box(es)")

    def external_clear_all(self):
        self._pending_clear = True
        self._log("External clear requested")

    def external_spawn_at(self, x_m: float, y_m: float, z_m: float,
                          w_m: float = 0.30, d_m: float = 0.30, h_m: float = 0.30,
                          label: str = "box1_위", aruco_id: int = 0):
        ok = self._spawn_box_at(
            float(x_m), float(y_m), float(z_m),
            max(float(w_m), 0.01),
            max(float(d_m), 0.01),
            max(float(h_m), 0.01),
            label,
            0.0,
            aruco_id=int(aruco_id),
        )
        self._log(
            f"External spawn-at {'OK' if ok else 'SKIP'}: "
            f"({x_m:.2f}, {y_m:.2f}, {z_m:.2f})"
        )
        return ok
