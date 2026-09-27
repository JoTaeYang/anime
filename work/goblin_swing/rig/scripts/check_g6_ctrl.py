"""check_g6_ctrl - Gate G6 CTRL checker (design doc d-23 section 5 + section 7, G6.1 .. G6.9, G6.11, G6.12).

G6.12 (2026-09-26 HR2, rig/data/hr2_contract.md): PROPS mouth_open [0, 1] default 0 (props contract), the
driver GOB_mesh key_blocks["mouth_open"].value <- PROPS["mouth_open"] (SINGLE_PROP, AVERAGE), the evaluated
shape-key value follows the property 0 -> 1, and goblin.reset_rig / goblin.ready_pose set it to 0.

G6.10 is a user item (GUI handling) and is not measured here.

Inputs (independent of the production scripts s06a..s06e; only blend data + data json + addon):
  rig/gob_r04_ctrl.blend             GOB_rig (DEF 24 + CTRL/MCH/PROPS), GOB_mesh, GOB_club,
                                     blend text goblin_rig_ui.py (operators goblin.*)
  rig/data/ctrl_manifest.json        controls{side, sweep}, spaces{control, values, ...},
                                     ikfk{prop, fk, ik, pole, def}, clamps{prop: [min, max]}
  rig/data/canonical_skeleton.json   canonical rest matrices (armature space, row-major) - G6.2
  rig/data/parts.json                part_id values (shoe_l / shoe_r) - G6.7
  rig/scripts/addon/goblin_rig_ui.py addon copy of the operators - G6.9 (import only)

Conventions (plan "공간 전환·IK/FK 규약" + manifest _doc):
  PROPS pose bone custom properties; ik_fk 0 = FK, 1 = IK (checked empirically, see G6.3 note);
  space props are int indices into spaces.<prop>.values; sweep rot = Euler degrees in the
  control's rotation_mode, loc = metres; foot_* props in degrees (clamps in prop units).
  "All CTRL identity" = every manifest control, every bone in the CTRL bone-collection subtree
  and every bone named CTRL_* (and PROPS) at loc 0 / rot identity / scale 1; PROPS custom
  properties at the contract defaults (arm_ik_fk 0, leg_ik_fk 1, others 0; UI defaults for keys
  outside the contract).  MCH bones are never touched.
  Operators come from the blend text goblin_rig_ui.py: Text.as_module().register() (factory
  startup does not auto-run blend scripts).  The addon file is only imported (not registered).
  Evaluation after every change: GOB_rig.update_tag() + view_layer.update().  Auto keying is off
  and any active action on GOB_rig is detached while measuring.
All poses / property values are changed in memory only and restored at the end; the blend and
data files are never saved or edited.  No renders (G6.10 is the visual item).

Run:    bl.ps1 -Script check_g6_ctrl.py -Blend gob_r04_ctrl.blend
Debug:  bl.ps1 -Script check_g6_ctrl.py -Blend work/r04a.blend -- --blend work/r04a.blend
        (writes rig/inspect/G6/check_g6_ctrl_debug.json instead)
Output: rig/inspect/G6/check_g6_ctrl.json
Exit:   0 when the evidence was written; 2 on a script error (run_main or a per-criterion error,
        evidence still written) or when an input file is missing.
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.  Lengths in m, reported in mm.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import colorsys  # noqa: E402
import ctypes  # noqa: E402
import importlib.util  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import random  # noqa: E402
import re  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Euler, Matrix, Vector  # noqa: E402

GATE = "G6"
CHECKER = "check_g6_ctrl"
BLEND = goblib.RIG / "gob_r04_ctrl.blend"
MANIFEST_JSON = goblib.DATA / "ctrl_manifest.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
PARTS_JSON = goblib.DATA / "parts.json"
ADDON = goblib.RIG / "scripts" / "addon" / "goblin_rig_ui.py"
TEXT_UI = "goblin_rig_ui.py"
TEXT_MANIFEST = "goblin_manifest.json"
ARM = "GOB_rig"
MESH = "GOB_mesh"
PROPS = "PROPS"
OPS = ("snap_ikfk", "switch_space", "reset_rig", "ready_pose")
COLL_NAMES = ("CTRL", "MCH", "DEF")

SEED = 1234
N_RANDOM = 20                # G6.3 random poses per chain
REST_TOL = 1e-4              # G6.2 max abs matrix element
SNAP_POS = 0.001             # G6.3 / G6.4 (m)
SNAP_ROT = 1.0               # G6.3 / G6.4 (deg)
ROT_STEP = 1.0               # G6.5 deg
LOC_STEP = 0.01              # G6.5 m (1 cm)
MAX_SAMPLES = 721            # G6.5 per channel; more -> subsample (recorded)
JUMP_K, JUMP_C = 3.0, 1.0    # G6.5 threshold = step * K + C (deg); loc step counted in cm
SCALE_TOL = 1e-4             # G6.6
GROUND_TOL = -0.001          # G6.7 (m)
PROP_STEP = 1.0              # G6.7 prop units (deg)
SPACE_POSE_FRAC = 0.35       # G6.4 target pose = frac * sweep end
CONTEXT_POSE_FRAC = 0.3      # G6.4 other controls random within frac * sweep range
FOOT_PROPS = ("foot_roll", "foot_bank", "heel_twist", "toe_twist")
SIDES = ("l", "r")
CONTRACT_DEFAULTS = {
    "arm_ik_fk_l": 0.0, "arm_ik_fk_r": 0.0, "leg_ik_fk_l": 1.0, "leg_ik_fk_r": 1.0,
    "head_space": 0, "hand_ik_space_l": 0, "hand_ik_space_r": 0, "weapon_space": 0,
    "knee_pole_space_l": 0, "knee_pole_space_r": 0,
    "foot_roll_l": 0.0, "foot_roll_r": 0.0, "foot_bank_l": 0.0, "foot_bank_r": 0.0,
    "heel_twist_l": 0.0, "heel_twist_r": 0.0, "toe_twist_l": 0.0, "toe_twist_r": 0.0,
    "mouth_open": 0.0,   # 2026-09-26 HR2 (hr2_contract.md)
}
MOUTH_PROP = "mouth_open"     # G6.12 PROPS key
MOUTH_KEY = "mouth_open"      # G6.12 GOB_mesh shape key
MOUTH_RANGE = (0.0, 1.0)      # G6.12 UI min / max
MOUTH_TOL = 1e-6              # G6.12 evaluated key value vs property value; reset / ready value vs 0
MOUTH_STEPS = (0.0, 0.25, 0.5, 0.75, 1.0, 0.0)   # G6.12 property values set in turn
MOUTH_PROBE = 0.7             # G6.12 value set before calling reset_rig / ready_pose
DOC_FK_VALUE = 0.0           # plan contract / manifest _doc: ik_fk 0 = FK, 1 = IK
COLOR_METHOD = ("effective color = pose_bone.color if its palette != DEFAULT else bone.color; "
                "THEMEnn -> preferences.themes[0].bone_color_sets[nn-1].normal (factory theme), "
                "CUSTOM -> color.custom.normal; class by HSV of that rgb: red h<20 or h>=330, "
                "yellow 40<=h<75, blue 190<=h<260 (s>=0.25, v>=0.2), else other")
SIDE_CLASS = {"L": "blue", "R": "red", "C": "yellow"}

_STATE = {"missing": [], "errors": []}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    """A criterion cannot be measured (missing bone / operator / manifest section)."""


def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=4):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def rel(p):
    return goblib._rel(p)


def mat_rot_deg(m1, m2):
    q1 = m1.to_3x3().normalized().to_quaternion()
    q2 = m2.to_3x3().normalized().to_quaternion()
    return goblib.quat_angle_deg(q1, q2)


def mat_finite(m):
    return all(math.isfinite(m[i][j]) for i in range(4) for j in range(4))


def mat_max_abs(m1, m2):
    return max(abs(m1[i][j] - m2[i][j]) for i in range(4) for j in range(4))


def _load_json(path, key, ctx):
    if not path.exists():
        _STATE["missing"].append(rel(path))
        ctx.miss[key] = f"missing input: {rel(path)}"
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _cap(lst, n=20):
    lst = list(lst)
    return lst if len(lst) <= n else lst[:n] + [f"... +{len(lst) - n} more"]


class Ctx:
    def __init__(self):
        self.miss = {}
        self.err = []
        self.rig = self.mesh = None
        self.man = self.canon = self.parts = None
        self.saved = None
        self.ctrl_set = []
        self.prop_defaults = {}
        self.fk_value = {}
        self.fk_detect = {}
        self.ops = {}
        self.ops_info = {}
        self.text_mod = None
        self.cycle_capture = None
        self.scale = {}          # source tag -> {"max_dev", "bone", "n"}
        self.nan = {}            # source tag -> count
        self.results = {}

    @property
    def pbs(self):
        return self.rig.pose.bones

    def pb(self, name):
        pb = self.rig.pose.bones.get(name)
        if pb is None:
            raise Blocked(f"bone {name} missing in {ARM}")
        return pb

    def section(self, key):
        v = (self.man or {}).get(key)
        if not v:
            raise Blocked(f"manifest section '{key}' missing or empty")
        return v

    def def_names(self):
        return [b["name"] for b in self.canon["bones"]]


# ---------------------------------------------------------------- evaluation / state
def evaluate(ctx):
    ctx.rig.update_tag()
    bpy.context.view_layer.update()


def track_def(ctx, tag, mats):
    """Record DEF scale deviation and NaN count for G6.5 / G6.6 (mats: name -> armature matrix)."""
    s = ctx.scale.setdefault(tag, {"max_dev": 0.0, "bone": None, "n": 0})
    s["n"] += 1
    for n, m in mats.items():
        if not mat_finite(m):
            ctx.nan[tag] = ctx.nan.get(tag, 0) + 1
            continue
        m3 = m.to_3x3()
        for i in range(3):
            d = abs(m3.col[i].length - 1.0)
            if d > s["max_dev"]:
                s["max_dev"], s["bone"] = d, n


def def_arm(ctx, tag=None):
    """Armature-space pose matrices of the 24 DEF bones (copies); tracked for scale/NaN if tag."""
    out = {n: ctx.pbs[n].matrix.copy() for n in ctx.def_names() if n in ctx.pbs}
    if tag:
        track_def(ctx, tag, out)
    return out


def def_world(ctx, names, tag=None):
    mw = ctx.rig.matrix_world
    if tag:
        track_def(ctx, tag, {n: ctx.pbs[n].matrix for n in ctx.def_names() if n in ctx.pbs})
    return {n: mw @ ctx.pbs[n].matrix for n in names if n in ctx.pbs}


def ctrl_world(ctx, name):
    return ctx.rig.matrix_world @ ctx.pb(name).matrix


def props_holder(ctx, name):
    pb = ctx.rig.pose.bones.get(PROPS)
    if pb is not None and name in pb.keys():
        return pb
    b = ctx.rig.data.bones.get(PROPS)
    if b is not None and name in b.keys():
        return b
    return None


def get_prop(ctx, name):
    h = props_holder(ctx, name)
    if h is None:
        raise Blocked(f"property {name} missing on {PROPS}")
    return h[name]


def set_prop(ctx, name, v):
    h = props_holder(ctx, name)
    if h is None:
        raise Blocked(f"property {name} missing on {PROPS}")
    cur = h[name]
    if isinstance(cur, int) and not isinstance(cur, bool):
        h[name] = int(round(v))
    else:
        h[name] = float(v)
    ctx.rig.update_tag()


def ui_default(h, key):
    try:
        d = h.id_properties_ui(key).as_dict().get("default")
    except Exception:
        return None
    return d if isinstance(d, (int, float)) and not isinstance(d, bool) else None


def ui_range(h, key):
    try:
        d = h.id_properties_ui(key).as_dict()
    except Exception:
        return None
    return {k: d.get(k) for k in ("min", "max", "soft_min", "soft_max", "description") if k in d}


def reset_ctrls(ctx):
    for n in ctx.ctrl_set:
        pb = ctx.pbs.get(n)
        if pb is None:
            continue
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)


def reset_props(ctx):
    for k, v in ctx.prop_defaults.items():
        set_prop(ctx, k, v)


def reset_all(ctx):
    reset_ctrls(ctx)
    reset_props(ctx)


def set_channels(pb, chans, loc_unit=1.0):
    """Identity, then channels [(channel, axis, value)]: rot = Euler deg in rotation_mode, loc = m."""
    rot = [0.0, 0.0, 0.0]
    loc = [0.0, 0.0, 0.0]
    for ch, ax, v in chans:
        i = "XYZ".index(ax.upper())
        if ch == "rot":
            rot[i] = float(v)
        elif ch == "loc":
            loc[i] = float(v) * loc_unit
    pb.location = loc
    pb.scale = (1.0, 1.0, 1.0)
    e = [math.radians(a) for a in rot]
    mode = pb.rotation_mode
    if mode == "QUATERNION":
        pb.rotation_quaternion = Euler(e, "XYZ").to_quaternion()
    elif mode == "AXIS_ANGLE":
        ax, ang = Euler(e, "XYZ").to_quaternion().to_axis_angle()
        pb.rotation_axis_angle = (ang, ax.x, ax.y, ax.z)
    else:
        pb.rotation_euler = e


def save_state(ctx):
    rig = ctx.rig
    pose = [(pb.name, pb.rotation_mode, tuple(pb.location), tuple(pb.rotation_quaternion),
             tuple(pb.rotation_euler), tuple(pb.rotation_axis_angle), tuple(pb.scale))
            for pb in rig.pose.bones]
    props = {}
    for holder_kind, h in (("pose", rig.pose.bones.get(PROPS)), ("data", rig.data.bones.get(PROPS))):
        if h is None:
            continue
        for k in h.keys():
            v = h[k]
            props[(holder_kind, k)] = v.to_list() if hasattr(v, "to_list") else (
                v.to_dict() if hasattr(v, "to_dict") else v)
    ts = bpy.context.scene.tool_settings
    ad = rig.animation_data
    ctx.saved = {"pose": pose, "props": props, "autokey": ts.use_keyframe_insert_auto,
                 "action": ad.action if ad is not None else None,
                 "pose_position": rig.data.pose_position}


def restore_state(ctx):
    s = ctx.saved
    if not s:
        return
    rig = ctx.rig
    for name, mode, loc, q, e, aa, sc in s["pose"]:
        pb = rig.pose.bones.get(name)
        if pb is None:
            continue
        pb.rotation_mode = mode
        pb.location = loc
        pb.rotation_quaternion = q
        pb.rotation_euler = e
        pb.rotation_axis_angle = aa
        pb.scale = sc
    for (kind, k), v in s["props"].items():
        h = rig.pose.bones.get(PROPS) if kind == "pose" else rig.data.bones.get(PROPS)
        if h is not None:
            try:
                h[k] = v
            except Exception:
                pass
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = s["autokey"]
    if rig.animation_data is not None and s["action"] is not None:
        rig.animation_data.action = s["action"]
    rig.data.pose_position = s["pose_position"]
    evaluate(ctx)


def prepare(ctx):
    rig = ctx.rig
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = False
    if rig.animation_data is not None and rig.animation_data.action is not None:
        rig.animation_data.action = None
    rig.data.pose_position = "POSE"
    vl = bpy.context.view_layer
    try:
        vl.objects.active = rig
        rig.select_set(True)
        if rig.mode != "POSE":
            bpy.ops.object.mode_set(mode="POSE")
        ctx.ops_info["context_mode (report)"] = rig.mode
    except Exception as e:  # operators may still work from object mode
        ctx.ops_info["context_mode (report)"] = f"mode_set error: {e}"
    # control set: manifest controls + CTRL collection subtree + CTRL_* names + PROPS
    names = set((ctx.man or {}).get("controls", {}).keys())
    for b in rig.data.bones:
        if b.name.startswith("CTRL_") or b.name == PROPS or any(in_ctrl_tree(c) for c in b.collections):
            names.add(b.name)
    ctx.ctrl_set = sorted(n for n in names if n in rig.pose.bones)
    # property defaults
    ctx.prop_defaults, ctx.prop_default_info = {}, {}
    h = rig.pose.bones.get(PROPS)
    keys = [k for k in (h.keys() if h is not None else []) if not k.startswith("_")]
    hd = rig.data.bones.get(PROPS)
    keys += [k for k in (hd.keys() if hd is not None else []) if not k.startswith("_") and k not in keys]
    for k in keys:
        hh = props_holder(ctx, k)
        v = hh[k]
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            continue
        ud = ui_default(hh, k)
        d = CONTRACT_DEFAULTS.get(k, ud if ud is not None else v)
        ctx.prop_defaults[k] = d
        ctx.prop_default_info[k] = {"used": d, "ui_default": ud,
                                    "contract": CONTRACT_DEFAULTS.get(k)}


# ---------------------------------------------------------------- bone collections / colours
def in_ctrl_tree(bc):
    while bc is not None:
        if bc.name == "CTRL":
            return True
        bc = bc.parent
    return False


def coll_visible(bc):
    v = getattr(bc, "is_visible_effectively", None)
    if v is not None:
        return bool(v)
    while bc is not None:
        if not bc.is_visible:
            return False
        bc = bc.parent
    return True


def _theme_sets():
    try:
        return list(bpy.context.preferences.themes[0].bone_color_sets)
    except Exception:
        return None


def color_info(rig, name, theme):
    pb = rig.pose.bones[name]
    b = rig.data.bones[name]
    src, col = "bone", b.color
    if pb.color.palette != "DEFAULT":
        src, col = "pose_bone", pb.color
    pal = col.palette
    rgb = None
    if pal == "CUSTOM":
        rgb = tuple(col.custom.normal)
    elif pal.startswith("THEME") and theme is not None:
        i = int(pal[5:]) - 1
        if 0 <= i < len(theme):
            rgb = tuple(theme[i].normal)
    if rgb is None:
        return {"src": src, "palette": pal, "rgb": None, "class": "default" if pal == "DEFAULT"
                else "unknown"}
    h, s, v = colorsys.rgb_to_hsv(*rgb[:3])
    h *= 360.0
    cls = "other"
    if s >= 0.25 and v >= 0.2:
        if h < 20.0 or h >= 330.0:
            cls = "red"
        elif 40.0 <= h < 75.0:
            cls = "yellow"
        elif 190.0 <= h < 260.0:
            cls = "blue"
    return {"src": src, "palette": pal, "rgb": [rnd(c, 3) for c in rgb[:3]],
            "hsv": [rnd(h, 1), rnd(s, 2), rnd(v, 2)], "class": cls}


# ---------------------------------------------------------------- G6.1
def c_structure(ctx):
    rig = ctx.rig
    arm = rig.data
    controls = ctx.section("controls")
    missing = [c for c in controls if c not in arm.bones]
    colls = {bc.name: bc for bc in arm.collections_all}
    coll_info = {n: {"visible_eff": coll_visible(bc), "is_visible": bc.is_visible,
                     "parent": bc.parent.name if bc.parent else None, "n_bones": len(bc.bones)}
                 for n, bc in colls.items()}
    req_missing = [n for n in COLL_NAMES if n not in colls]
    vis_ok = ("CTRL" in colls and coll_visible(colls["CTRL"])
              and all(n in colls and not coll_visible(colls[n]) for n in ("MCH", "DEF")))
    visible_outside = []
    for b in arm.bones:
        if b.hide:
            continue
        cs = list(b.collections)
        if any(in_ctrl_tree(c) for c in cs):
            continue
        if not cs or any(coll_visible(c) for c in cs):
            visible_outside.append(b.name)
    not_in_ctrl = [c for c in controls if c in arm.bones
                   and not any(in_ctrl_tree(bc) for bc in arm.bones[c].collections)]
    defs = set(ctx.def_names())
    def_mch_in_ctrl = [b.name for b in arm.bones if (b.name in defs or b.name.startswith("MCH"))
                       and any(in_ctrl_tree(bc) for bc in b.collections)]
    theme = _theme_sets()
    colors, color_fail, shape_missing, side_bad = {}, [], [], []
    for c, spec in controls.items():
        if c not in arm.bones:
            continue
        side = str(spec.get("side", "")).upper()
        want = SIDE_CLASS.get(side)
        if want is None:
            side_bad.append(c)
        ci = color_info(rig, c, theme)
        ci["side"] = side
        ci["ok"] = want is not None and ci["class"] == want
        colors[c] = ci
        if not ci["ok"]:
            color_fail.append(c)
        if rig.pose.bones[c].custom_shape is None:
            shape_missing.append(c)
    extra_ctrl = sorted(b.name for b in arm.bones if b.name.startswith("CTRL_") and b.name not in controls)
    ok = (not missing and not req_missing and vis_ok and not visible_outside and not not_in_ctrl
          and not def_mch_in_ctrl and not color_fail and not shape_missing and not side_bad)
    return {"n_controls": len(controls), "missing_controls": missing,
            "collections": coll_info, "required_collections_missing": req_missing,
            "ctrl_only_visible": vis_ok, "visible_bones_outside_ctrl": _cap(visible_outside),
            "n_visible_bones_outside_ctrl": len(visible_outside),
            "controls_not_in_ctrl_collection": not_in_ctrl, "def_or_mch_in_ctrl": _cap(def_mch_in_ctrl),
            "color_fail": color_fail, "side_invalid": side_bad, "custom_shape_missing": shape_missing,
            "theme_available": theme is not None, "colors": colors,
            "ctrl_bones_not_in_manifest (report)": extra_ctrl}, ok


# ---------------------------------------------------------------- G6.2
def driver_status(ctx):
    ad = ctx.rig.animation_data
    if ad is None:
        return {"n": 0}
    bad, nonsimple = [], []
    for fc in ad.drivers:
        d = fc.driver
        if not d.is_valid or not fc.is_valid:
            bad.append(fc.data_path)
        if d.type == "SCRIPTED" and not d.is_simple_expression:
            nonsimple.append(fc.data_path)
    return {"n": len(ad.drivers), "invalid": _cap(bad, 10), "non_simple_expression": _cap(nonsimple, 10)}


def c_rest(ctx):
    reset_all(ctx)
    evaluate(ctx)
    per, missing = {}, []
    for b in ctx.canon["bones"]:
        n = b["name"]
        if n not in ctx.pbs:
            missing.append(n)
            continue
        m = ctx.pbs[n].matrix
        per[n] = mat_max_abs(m, Matrix(b["rest_matrix"]))
    track_def(ctx, "G6.2", {n: ctx.pbs[n].matrix for n in per})
    worst = max(per.values()) if per else float("inf")
    top = sorted(per.items(), key=lambda kv: -kv[1])[:5]
    mism = {k: v for k, v in ctx.prop_default_info.items()
            if v["ui_default"] is not None and v["contract"] is not None
            and abs(float(v["ui_default"]) - float(v["contract"])) > 1e-9}
    return {"max_abs": rnd(worst, 7), "worst5": {k: rnd(v, 7) for k, v in top},
            "missing_def": missing, "n_ctrl_reset": len(ctx.ctrl_set),
            "props_used": {k: v["used"] for k, v in ctx.prop_default_info.items()},
            "props_ui_default_vs_contract_mismatch (report)": mism,
            "drivers (report)": driver_status(ctx)}, (not missing and worst <= REST_TOL)


# ---------------------------------------------------------------- operators
def op_fn(name):
    return getattr(bpy.ops.goblin, name)


def op_registered(name):
    try:
        op_fn(name).get_rna_type()
        return True
    except Exception:
        return False


def op_params(ctx, name):
    if not ctx.ops.get(name):
        raise Blocked(f"operator goblin.{name} not registered")
    rna = op_fn(name).get_rna_type()
    return {p.identifier: p for p in rna.properties if p.identifier != "rna_type"}


def adapt(p, name, index):
    """Value for operator property p that means value `name` (string) / `index` (int)."""
    if p.type == "ENUM":
        ids = [i.identifier for i in p.enum_items]
        for i in ids:
            if name is not None and i.lower() == str(name).lower():
                return i
        if ids and index is not None and 0 <= index < len(ids):
            return ids[index]
        return str(name) if name is not None else str(index)
    if p.type == "INT":
        return int(index)
    if p.type == "FLOAT":
        return float(index)
    return str(name) if name is not None else str(index)


def pick_param(params, *keys):
    for k in keys:
        for pid in params:
            if k in pid.lower():
                return pid
    return None


def run_op(ctx, name, **kw):
    """Call bpy.ops.goblin.<name>(**kw); returns (result string, error or None)."""
    if not ctx.ops.get(name):
        raise Blocked(f"operator goblin.{name} not registered")
    try:
        r = op_fn(name)(**kw)
        return ",".join(sorted(r)), None
    except Exception as e:
        return None, f"{type(e).__name__}: {str(e).strip()[:200]}"


def register_ops(ctx):
    """G6.9 data: register the blend text module, check the three operators, import the addon."""
    info = {"text_exists": False}
    txt = bpy.data.texts.get(TEXT_UI)
    info["registered_before"] = {o: op_registered(o) for o in OPS}
    if txt is not None:
        info["text_exists"] = True
        info["use_module"] = bool(txt.use_module)
        try:
            mod = txt.as_module()
            ctx.text_mod = mod
            reg = getattr(mod, "register", None)
            if callable(reg):
                reg()
                info["register"] = "called"
            else:
                info["register"] = "no register()"
        except Exception as e:
            info["register"] = f"error: {type(e).__name__}: {e}"
    info["registered_after"] = {o: op_registered(o) for o in OPS}
    ctx.ops = dict(info["registered_after"])
    # addon file (import only)
    if ADDON.exists():
        try:
            spec = importlib.util.spec_from_file_location("goblin_rig_ui_addon_g6check", str(ADDON))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            info["addon_import"] = "ok"
            info["addon_has_register"] = callable(getattr(mod, "register", None))
            info["addon_has_unregister"] = callable(getattr(mod, "unregister", None))
        except Exception as e:
            info["addon_import"] = f"error: {type(e).__name__}: {e}"
        if txt is not None:
            def norm(s):
                return "\n".join(line.rstrip() for line in s.replace("\r\n", "\n").split("\n")).strip()
            info["same_code"] = norm(txt.as_string()) == norm(ADDON.read_text(encoding="utf-8"))
    else:
        info["addon_import"] = f"missing: {rel(ADDON)}"
    mt = bpy.data.texts.get(TEXT_MANIFEST)
    if mt is not None and ctx.man is not None:
        try:
            info["embedded_manifest_equals_file (report)"] = json.loads(mt.as_string()) == ctx.man
        except Exception as e:
            info["embedded_manifest_equals_file (report)"] = f"error: {e}"
    else:
        info["embedded_manifest_equals_file (report)"] = f"text {TEXT_MANIFEST} missing"
    return info


# ---------------------------------------------------------------- pose generation
def sweeps(ctx, ctrl):
    return ((ctx.man.get("controls") or {}).get(ctrl) or {}).get("sweep") or []


def loc_unit(ctx):
    u = str(((ctx.man or {}).get("units") or {}).get("loc", "m")).lower()
    return 0.01 if u == "cm" else 1.0


def random_pose(ctx, rng, ctrls, frac=1.0):
    pose = {}
    for c in ctrls:
        ch = []
        for sw in sweeps(ctx, c):
            lo, hi = float(sw["min"]), float(sw["max"])
            if frac < 1.0:
                mid = 0.5 * (lo + hi)
                lo, hi = mid + (lo - mid) * frac, mid + (hi - mid) * frac
            ch.append((sw["channel"], sw["axis"], rng.uniform(lo, hi)))
        pose[c] = ch
    return pose


def apply_pose(ctx, pose):
    lu = loc_unit(ctx)
    for c, ch in pose.items():
        if c in ctx.pbs:
            set_channels(ctx.pbs[c], ch, lu)


def chain_ctrls(ch):
    out = list(ch.get("fk") or [])
    for k in ("ik", "pole"):
        v = ch.get(k)
        if isinstance(v, str):
            out.append(v)
        elif isinstance(v, list):
            out.extend(v)
    return out


def detect_fk_value(ctx, key, ch):
    """Which ik_fk value makes the FK controls drive DEF (moves measured on the last def bone)."""
    if key in ctx.fk_value:
        return ctx.fk_value[key]
    fk = ch.get("fk") or []
    defs = ch.get("def") or []
    moves = {}
    if fk and defs and fk[0] in ctx.pbs and defs[-1] in ctx.pbs:
        for v in (0.0, 1.0):
            reset_all(ctx)
            set_prop(ctx, ch["prop"], v)
            evaluate(ctx)
            m0 = def_world(ctx, [defs[-1]])[defs[-1]]
            set_channels(ctx.pbs[fk[0]], [("rot", "X", 20.0), ("rot", "Z", 20.0)], loc_unit(ctx))
            evaluate(ctx)
            m1 = def_world(ctx, [defs[-1]])[defs[-1]]
            moves[v] = (m1.translation - m0.translation).length * 1000.0 + mat_rot_deg(m0, m1)
        reset_all(ctx)
    emp = max(moves, key=moves.get) if moves and max(moves.values()) > 1e-3 else None
    man = ch.get("fk_value")
    if isinstance(man, (int, float)) and not isinstance(man, bool):
        fkv, src = float(man), "manifest fk_value"
    elif emp is not None:
        fkv, src = emp, "empirical"
    else:
        fkv, src = DOC_FK_VALUE, "contract doc"
    ctx.fk_value[key] = fkv
    ctx.fk_detect[key] = {"moves_mm+deg": {str(k): rnd(v, 3) for k, v in moves.items()},
                          "empirical_fk_value": emp, "manifest_fk_value": man,
                          "used": fkv, "source": src}
    return fkv


def basis_snapshot(ctx, names):
    return {n: ctx.pbs[n].matrix_basis.copy() for n in names if n in ctx.pbs}


def moved(a, b):
    return any(mat_max_abs(a[n], b[n]) > 1e-6 for n in a if n in b)


# ---------------------------------------------------------------- G6.3
def snap_params(ctx):
    params = op_params(ctx, "snap_ikfk")
    pc = pick_param(params, "chain", "limb")
    pd = pick_param(params, "dir", "mode", "to")
    if pc is None or pd is None:
        raise Blocked(f"snap_ikfk parameters not recognised: {list(params)}")
    return params, pc, pd


def classify_dirs(ctx, key, ch, params, pc, pd):
    """Map direction values -> 'fk_to_ik' (IK controls matched, source FK) / 'ik_to_fk'."""
    p = params[pd]
    if p.type == "ENUM" and len(p.enum_items):
        cands = [i.identifier for i in p.enum_items]
    else:
        cands = ["FK_TO_IK", "IK_TO_FK"]
    fkv = ctx.fk_value[key]
    fk = list(ch.get("fk") or [])
    ik = [c for c in chain_ctrls(ch) if c not in fk]
    out, info = {}, {}
    rng = random.Random(SEED + 7)
    pose = random_pose(ctx, rng, fk + ik)
    for d in cands:
        cls = None
        for src in (fkv, 1.0 - fkv):
            reset_all(ctx)
            apply_pose(ctx, pose)
            set_prop(ctx, ch["prop"], src)
            evaluate(ctx)
            b_fk, b_ik = basis_snapshot(ctx, fk), basis_snapshot(ctx, ik)
            r, err = run_op(ctx, "snap_ikfk", **{pc: adapt(params[pc], key, None), pd: d})
            evaluate(ctx)
            mf, mi = moved(b_fk, basis_snapshot(ctx, fk)), moved(b_ik, basis_snapshot(ctx, ik))
            info.setdefault(d, []).append({"src": src, "result": r, "err": err, "moved_fk": mf,
                                           "moved_ik": mi})
            if mi and not mf:
                cls = "fk_to_ik"
            elif mf and not mi:
                cls = "ik_to_fk"
            if cls:
                break
        if cls is None:
            n = re.sub(r"[^a-z]", "", d.lower())
            if n.endswith("toik") or n == "ik":
                cls = "fk_to_ik"
            elif n.endswith("tofk") or n == "fk":
                cls = "ik_to_fk"
            info.setdefault(d, []).append({"classified_by_name": cls})
        if cls and cls not in out:
            out[cls] = d
    reset_all(ctx)
    return out, info


def c_snap(ctx):
    ikfk = ctx.section("ikfk")
    params, pc, pd = snap_params(ctx)
    meas, ok = {}, True
    t0 = time.time()
    for key, ch in ikfk.items():
        fkv = detect_fk_value(ctx, key, ch)
        dirs, dinfo = classify_dirs(ctx, key, ch, params, pc, pd)
        ctrls = chain_ctrls(ch)
        defs = [d for d in (ch.get("def") or []) if d in ctx.pbs]
        missing = [c for c in ctrls if c not in ctx.pbs] + [d for d in (ch.get("def") or [])
                                                            if d not in ctx.pbs]
        rng = random.Random(SEED)
        poses = [random_pose(ctx, rng, ctrls) for _ in range(N_RANDOM)]
        cm = {"prop": ch.get("prop"), "fk_value": fkv, "fk_detect": ctx.fk_detect.get(key),
              "direction_values": dirs, "direction_probe": dinfo, "missing": missing}
        if missing or not defs:
            ok = False
            meas[key] = cm
            continue
        for label in ("fk_to_ik", "ik_to_fk"):
            d = dirs.get(label)
            if d is None:
                cm[label] = "blocked: direction value not identified"
                ok = False
                continue
            src = fkv if label == "fk_to_ik" else 1.0 - fkv
            tgt = 1.0 - src
            worst_p, worst_r, worst_at, errs, not_set = 0.0, 0.0, None, [], 0
            for i, pose in enumerate(poses):
                reset_all(ctx)
                apply_pose(ctx, pose)
                set_prop(ctx, ch["prop"], src)
                evaluate(ctx)
                w0 = def_world(ctx, defs, tag="G6.3")
                r, err = run_op(ctx, "snap_ikfk", **{pc: adapt(params[pc], key, None), pd: d})
                if err:
                    errs.append(f"pose {i}: {err}")
                evaluate(ctx)
                if abs(float(get_prop(ctx, ch["prop"])) - tgt) > 1e-6:
                    not_set += 1
                    set_prop(ctx, ch["prop"], tgt)
                    evaluate(ctx)
                w1 = def_world(ctx, defs, tag="G6.3")
                for n in defs:
                    dp = (w1[n].translation - w0[n].translation).length
                    dr = mat_rot_deg(w0[n], w1[n])
                    if dp > worst_p or dr > worst_r:
                        if dp > worst_p:
                            worst_p = dp
                        if dr > worst_r:
                            worst_r = dr
                        worst_at = {"pose": i, "bone": n}
            lok = not errs and worst_p <= SNAP_POS and worst_r <= SNAP_ROT
            ok = ok and lok
            cm[label] = {"direction_value": d, "src_value": src, "tgt_value": tgt,
                         "max_pos_mm": mm(worst_p), "max_rot_deg": rnd(worst_r),
                         "worst_at": worst_at, "n_poses": len(poses),
                         "prop_not_set_by_op (forced after op)": not_set,
                         "op_errors": _cap(errs, 5), "ok": lok}
        meas[key] = cm
    reset_all(ctx)
    return {"chains": meas, "op_params": {"chain": pc, "direction": pd},
            "seconds": rnd(time.time() - t0, 1)}, ok


# ---------------------------------------------------------------- G6.4
_DRV_RE = re.compile(r'pose\.bones\["(.+?)"\]\.constraints\["(.+?)"\]\.influence')


def driven_constraints(ctx, prop):
    """(bone, constraint) whose influence has a driver that references PROPS[prop]."""
    ad = ctx.rig.animation_data
    out = []
    if ad is None:
        return out
    key = f'["{prop}"]'
    for fc in ad.drivers:
        m = _DRV_RE.fullmatch(fc.data_path)
        if not m:
            continue
        refs = any(key in (t.data_path or "") for v in fc.driver.variables for t in v.targets)
        if refs:
            out.append((m.group(1), m.group(2), fc.driver.expression))
    return out


def has_influence_driver(ctx, bone, con):
    ad = ctx.rig.animation_data
    if ad is None:
        return False
    dp = f'pose.bones["{bone}"].constraints["{con}"].influence'
    return any(fc.data_path == dp for fc in ad.drivers)


def read_infl(ctx, pairs):
    dg = bpy.context.evaluated_depsgraph_get()
    re_ = ctx.rig.evaluated_get(dg)
    out = {}
    for b, c in pairs:
        pb = re_.pose.bones.get(b)
        con = pb.constraints.get(c) if pb is not None else None
        out[f"{b}/{c}"] = None if con is None else float(con.influence)
    return out


def space_constraints(ctx, prop, sp):
    """Driven constraints of a space prop: manifest influence field ('driver ...') on sp['mch'],
    else driver scan.  Returns ([(bone, con)], {bone/con: expected-k or None}, source)."""
    mch = sp.get("mch")
    infl = sp.get("influence")
    if mch and isinstance(infl, dict):
        pairs, exp = [], {}
        for cn, how in infl.items():
            if "driver" in str(how).lower():
                pairs.append((mch, cn))
                m = re.search(r"==\s*(-?\d+)", str(how))
                exp[f"{mch}/{cn}"] = int(m.group(1)) if m else None
        if pairs:
            return pairs, exp, "manifest influence"
    scan = driven_constraints(ctx, prop)
    pairs = [(b, c) for b, c, _e in scan]
    exp = {}
    for b, c, e in scan:
        m = re.fullmatch(r"\s*\w+\s*==\s*(-?\d+)\s*", e or "")
        exp[f"{b}/{c}"] = int(m.group(1)) if m else None
    return pairs, exp, "driver scan"


def c_space(ctx):
    spaces = ctx.section("spaces")
    params = op_params(ctx, "switch_space")
    pp = pick_param(params, "prop")
    pv = pick_param(params, "value", "space", "target")
    if pp is None or pv is None:
        raise Blocked(f"switch_space parameters not recognised: {list(params)}")
    controls = list((ctx.man.get("controls") or {}).keys())
    meas, ok = {}, True
    t0 = time.time()
    for prop_key, sp in spaces.items():
        prop = sp.get("prop", prop_key)
        ctrl = sp.get("control")
        vals = list(sp.get("values") or [])
        pm = {"control": ctrl, "values": vals}
        if ctrl not in ctx.pbs or props_holder(ctx, prop) is None or len(vals) < 2:
            pm["blocked"] = (f"control {ctrl} missing" if ctrl not in ctx.pbs else
                             f"prop {prop} missing" if props_holder(ctx, prop) is None else "< 2 values")
            meas[prop_key], ok = pm, False
            continue
        pairs, exp_k, src = space_constraints(ctx, prop, sp)
        pm["driven_constraints"] = [f"{b}/{c}" for b, c in pairs]
        pm["driven_source"] = src
        pm["influence_has_driver"] = {f"{b}/{c}": has_influence_driver(ctx, b, c) for b, c in pairs}
        rng = random.Random(SEED)
        ctx_pose = random_pose(ctx, rng, [c for c in controls if c != ctrl], CONTEXT_POSE_FRAC)
        tgt_ch = []
        for sw in sweeps(ctx, ctrl):
            lo, hi = float(sw["min"]), float(sw["max"])
            v = SPACE_POSE_FRAC * (hi if abs(hi) > 1e-9 else lo)
            tgt_ch.append((sw["channel"], sw["axis"], v))
        if not tgt_ch:
            tgt_ch = [("rot", "X", 10.0), ("rot", "Y", -15.0), ("rot", "Z", 20.0)]
        pm["target_pose"] = [[c, a, rnd(v, 4)] for c, a, v in tgt_ch]
        pair_res = {}
        # driver-defined influence per value (prop set directly, no operator): reference
        ref = {}
        for ib in range(len(vals)):
            reset_all(ctx)
            apply_pose(ctx, ctx_pose)
            set_prop(ctx, prop, ib)
            evaluate(ctx)
            ref[ib] = read_infl(ctx, pairs)
        for ia, a in enumerate(vals):
            for ib, b in enumerate(vals):
                if ia == ib:
                    continue
                reset_all(ctx)
                apply_pose(ctx, ctx_pose)
                set_prop(ctx, prop, ia)
                evaluate(ctx)
                set_channels(ctx.pbs[ctrl], tgt_ch, loc_unit(ctx))
                evaluate(ctx)
                w0 = ctrl_world(ctx, ctrl)
                l0 = ctx.pbs[ctrl].matrix_basis.copy()
                r, err = run_op(ctx, "switch_space", **{pp: adapt(params[pp], prop, None),
                                                        pv: adapt(params[pv], b, ib)})
                evaluate(ctx)
                w1 = ctrl_world(ctx, ctrl)
                l1 = ctx.pbs[ctrl].matrix_basis.copy()
                pval = get_prop(ctx, prop)
                infl = read_infl(ctx, pairs)
                dp = (w1.translation - w0.translation).length
                dr = mat_rot_deg(w0, w1)
                prop_ok = abs(float(pval) - ib) < 1e-6
                # expected influence: parsed '== k' when available, else driver reference for b
                infl_ok = bool(pairs)
                infl_exp = {}
                for k_, v_ in infl.items():
                    kk = exp_k.get(k_)
                    e_ = (1.0 if kk == ib else 0.0) if kk is not None else ref[ib].get(k_)
                    infl_exp[k_] = e_
                    if v_ is None or e_ is None or abs(v_ - e_) > 1e-4:
                        infl_ok = False
                differs = any(ref[ia].get(k_) is not None and ref[ib].get(k_) is not None
                              and abs(ref[ia][k_] - ref[ib][k_]) > 0.5 for k_ in infl)
                lok = (not err and dp <= SNAP_POS and dr <= SNAP_ROT and prop_ok and infl_ok
                       and differs)
                ok = ok and lok
                pair_res[f"{a}->{b}"] = {"pos_mm": mm(dp), "rot_deg": rnd(dr), "op": r, "op_error": err,
                                         "prop_after": pval, "prop_ok": prop_ok,
                                         "influence_after": {k_: rnd(v_, 4) if v_ is not None else None
                                                             for k_, v_ in infl.items()},
                                         "influence_expected": infl_exp, "influence_ok": infl_ok,
                                         "driven_influence_differs_a_b": differs,
                                         "ctrl_local_changed": mat_max_abs(l0, l1) > 1e-6, "ok": lok}
        pm["pairs"] = pair_res
        meas[prop_key] = pm
    reset_all(ctx)
    return {"spaces": meas, "op_params": {"prop": pp, "value": pv},
            "seconds": rnd(time.time() - t0, 1)}, ok


# ---------------------------------------------------------------- G6.5
def ctrl_modes(ctx):
    """control -> [(prop, value)] so that the control drives DEF (FK controls FK, IK/pole IK)."""
    out = {}
    for key, ch in ((ctx.man or {}).get("ikfk") or {}).items():
        if not ch.get("prop") or props_holder(ctx, ch["prop"]) is None:
            continue
        fkv = detect_fk_value(ctx, key, ch)
        for c in ch.get("fk") or []:
            out[c] = [(ch["prop"], fkv)]
        for c in chain_ctrls(ch):
            if c not in (ch.get("fk") or []):
                out[c] = [(ch["prop"], 1.0 - fkv)]
    # manifest controls.<CTRL>.mode {prop: value} takes precedence
    for c, spec in ((ctx.man or {}).get("controls") or {}).items():
        md = spec.get("mode")
        if isinstance(md, dict) and md:
            out[c] = [(p, float(v)) for p, v in md.items() if props_holder(ctx, p) is not None]
    return out


def sweep_values(lo, hi, step):
    span = hi - lo
    n = int(math.floor(span / step + 1e-9)) + 1
    sub = False
    if n > MAX_SAMPLES:
        n, sub = MAX_SAMPLES, True
    if n < 2:
        return [lo, hi], (hi - lo), sub
    st = span / (n - 1) if sub else step
    vals = [lo + i * st for i in range(n)]
    if hi - vals[-1] > 1e-9:
        vals.append(hi)
    return vals, st, sub


REFINE_LEVELS = 4             # G6.5 bisection levels (h/2 .. h/16)
POP_RATIO = 0.5               # G6.5 pop if jump(h/16) > POP_RATIO * jump(h)


def _def_quats(ctx):
    mats = def_arm(ctx, tag="G6.5")
    return {n: m.to_3x3().normalized().to_quaternion() for n, m in mats.items() if mat_finite(m)}


def _max_jump(qa, qb):
    """Largest quaternion angle change over the DEF bones present in both samples -> (deg, bone)."""
    mx, bone = 0.0, None
    for n in qa:
        if n in qb:
            d = goblib.quat_angle_deg(qa[n], qb[n])
            if d > mx:
                mx, bone = d, n
    return mx, bone


FLIP_MIN_DEG = 2.0            # G6.5 (b) branch flip: sign change with |theta| > 2 deg on both sides


def bend_theta(ctx, upper, lower):
    """theta = atan2(dot(cross(u, l), X_upper), dot(u, l)) in deg; u, l = world DEF segment
    directions (tail - head) of upper / lower, X_upper = world local X of the upper pose bone."""
    mw = ctx.rig.matrix_world
    pu, pl = ctx.pbs[upper], ctx.pbs[lower]
    u = ((mw @ pu.tail) - (mw @ pu.head)).normalized()
    ll = ((mw @ pl.tail) - (mw @ pl.head)).normalized()
    x = (mw.to_3x3() @ pu.matrix.to_3x3()).col[0].normalized()
    return math.degrees(math.atan2(u.cross(ll).dot(x), u.dot(ll)))


def bend_chains(ctx):
    """chain key -> (upper DEF, lower DEF) from manifest ikfk def lists."""
    out = {}
    for key, ch in ((ctx.man or {}).get("ikfk") or {}).items():
        d = ch.get("def") or []
        if len(d) >= 2 and d[0] in ctx.pbs and d[1] in ctx.pbs:
            out[key] = (d[0], d[1])
    return out


def c_sweep(ctx):
    controls = ctx.section("controls")
    modes = ctrl_modes(ctx)
    lu = loc_unit(ctx)
    chains = bend_chains(ctx)
    flips, th_range = [], {k: [float("inf"), -float("inf")] for k in chains}
    per, pops, cands, subs, missing = {}, [], [], [], []
    n_samples = n_refine = 0
    nan_before = ctx.nan.get("G6.5", 0)
    t0 = time.time()
    for c, spec in controls.items():
        if c not in ctx.pbs:
            missing.append(c)
            continue
        cres = {}
        pb = ctx.pbs[c]
        for sw in spec.get("sweep") or []:
            chn, ax = sw["channel"], sw["axis"]
            lo, hi = float(sw["min"]), float(sw["max"])
            step = ROT_STEP if chn == "rot" else LOC_STEP / lu
            vals, st, sub = sweep_values(lo, hi, step)
            step_units = st if chn == "rot" else st * lu * 100.0   # deg or cm
            thr = step_units * JUMP_K + JUMP_C
            if sub:
                subs.append(f"{c}.{chn}{ax} step {rnd(step_units, 3)}")
            reset_all(ctx)
            for p, v in modes.get(c, []):
                set_prop(ctx, p, v)

            def sample(v):
                set_channels(pb, [(chn, ax, v)], lu)
                evaluate(ctx)
                return _def_quats(ctx)

            qs, ths = [], []
            for v in vals:
                qs.append(sample(v))
                n_samples += 1
                th = {k: bend_theta(ctx, u, l_) for k, (u, l_) in chains.items()}
                for k, t in th.items():
                    th_range[k] = [min(th_range[k][0], t), max(th_range[k][1], t)]
                if ths:
                    for k, t in th.items():
                        t0_ = ths[-1][k]
                        if (t0_ > 0) != (t > 0) and abs(t0_) > FLIP_MIN_DEG and abs(t) > FLIP_MIN_DEG:
                            flips.append({"channel": f"{c}.{chn}{ax}", "chain": k,
                                          "values": [rnd(vals[len(ths) - 1], 4), rnd(v, 4)],
                                          "theta_deg": [rnd(t0_, 3), rnd(t, 3)]})
                ths.append(th)
            mx, at, n_cand, n_pop = 0.0, None, 0, 0
            for i in range(1, len(vals)):
                j, bone = _max_jump(qs[i - 1], qs[i])
                if j > mx:
                    mx, at = j, {"bone": bone, "value": rnd(vals[i], 4)}
                if j <= thr:
                    continue
                # pop candidate: bisect toward the larger half REFINE_LEVELS times
                a, b, qa, qb = vals[i - 1], vals[i], qs[i - 1], qs[i]
                seq, bones = [j], [bone]
                for _lvl in range(REFINE_LEVELS):
                    m = 0.5 * (a + b)
                    qm = sample(m)
                    n_refine += 1
                    jl, bl = _max_jump(qa, qm)
                    jr, br = _max_jump(qm, qb)
                    if jl >= jr:
                        b, qb = m, qm
                        seq.append(jl)
                        bones.append(bl)
                    else:
                        a, qa = m, qm
                        seq.append(jr)
                        bones.append(br)
                pop = seq[-1] > POP_RATIO * seq[0]
                n_cand += 1
                cid = f"{c}.{chn}{ax}"
                cands.append({"channel": cid, "interval": [rnd(vals[i - 1], 4), rnd(vals[i], 4)],
                              "final_interval": [rnd(a, 6), rnd(b, 6)],
                              "jump_deg_h_to_h16": [rnd(x, 4) for x in seq], "bones": bones,
                              "thr_deg": rnd(thr, 3),
                              "class": "pop" if pop else "continuous"})
                if pop:
                    n_pop += 1
                    if cid not in pops:
                        pops.append(cid)
            cres[f"{chn}{ax}"] = {"n": len(vals), "step": rnd(step_units, 4), "max_jump_deg": rnd(mx, 3),
                                  "thr_deg": rnd(thr, 3), "at": at, "candidates": n_cand, "pops": n_pop}
        per[c] = cres
    reset_all(ctx)
    nan = ctx.nan.get("G6.5", 0) - nan_before
    ok_a = not pops and not missing and nan == 0
    ok_b = bool(chains) and not flips
    ok = ok_a and ok_b
    return {"ok_a_continuity": ok_a, "ok_b_branch_flip": ok_b,
            "b_n_flips": len(flips), "b_flips": _cap(flips, 30),
            "b_chains": {k: list(v) for k, v in chains.items()},
            "b_theta_deg_min_max": {k: [rnd(v[0], 3), rnd(v[1], 3)] for k, v in th_range.items()},
            "n_controls": len(controls), "n_samples": n_samples, "n_refine_samples": n_refine,
            "nan_bone_samples": nan, "pop_channels": pops, "n_candidates": len(cands),
            "n_pops": sum(1 for x in cands if x["class"] == "pop"), "candidates": cands,
            "missing": missing, "subsampled": subs,
            "mode_props_set": {k: [list(x) for x in v] for k, v in modes.items()},
            "per_control": per, "seconds": rnd(time.time() - t0, 1)}, ok


# ---------------------------------------------------------------- G6.7
def shoe_verts(ctx, part):
    me = ctx.mesh.data
    att = me.attributes.get("part_id")
    if att is None or att.domain != "FACE":
        raise Blocked("GOB_mesh face attribute part_id missing")
    pid = ctx.parts.get(part)
    if pid is None:
        raise Blocked(f"parts.json has no {part}")
    n = len(me.polygons)
    vals = np.empty(n, dtype=np.int32)
    att.data.foreach_get("value", vals)
    idx = set()
    for i in np.nonzero(vals == int(pid))[0]:
        idx.update(me.polygons[int(i)].vertices)
    if not idx:
        raise Blocked(f"no faces with part_id {pid} ({part})")
    return np.array(sorted(idx), dtype=np.int64)


PLANT_TORSO_DZ = -0.015       # G6.7 CTRL_torso world Z offset (m) of the ready pose
PLANT_REACH = 0.001           # G6.7 (1) |MCH_calf_ik tail - MCH_roll_foot head| (m)
PLANT_Z = (-0.001, 0.003)     # G6.7 (2) lowest shoe vertex z range (m)
PLANT_SLIP = 0.005            # G6.7 (3) roll / bank slip (m)
SLIP_JUDGED = ("foot_roll", "foot_bank")
CONTACT_TOL = 0.001           # G6.7 (3) contact set: z <= lowest z of the step + 1 mm


def path_values(end, step):
    """Values from 0 (excluded) toward end (included) in `step` increments."""
    if abs(end) < 1e-12:
        return []
    sg = 1.0 if end > 0 else -1.0
    out, k = [], 1
    while k * step < abs(end) - 1e-9:
        out.append(sg * k * step)
        k += 1
    out.append(end)
    return out


def shoe_world(ctx, idx):
    """World coordinates (n, 3) of the evaluated GOB_mesh vertices idx."""
    dg = bpy.context.evaluated_depsgraph_get()
    oe = ctx.mesh.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        n = len(me.vertices)
        co = np.empty(n * 3, dtype=np.float64)
        me.vertices.foreach_get("co", co)
    finally:
        oe.to_mesh_clear()
    co = co.reshape(-1, 3)
    if idx.max() >= len(co):
        raise Blocked("evaluated GOB_mesh vertex count differs from the original")
    mw = np.array(oe.matrix_world, dtype=np.float64)
    return co[idx] @ mw[:3, :3].T + mw[:3, 3]


def torso_down_channel(ctx):
    """CTRL_torso loc channel whose world direction is +Z -> (axis letter, value for PLANT_TORSO_DZ)."""
    b = ctx.rig.data.bones.get("CTRL_torso")
    if b is None:
        raise Blocked("bone CTRL_torso missing")
    m = ctx.rig.matrix_world.to_3x3() @ b.matrix_local.to_3x3()
    dots = [m.col[i].normalized().z for i in range(3)]
    i = max(range(3), key=lambda k: abs(dots[k]))
    return "XYZ"[i], PLANT_TORSO_DZ / (dots[i] * m.col[i].length), dots[i]


def ready_pose(ctx, leg_prop, torso_ch):
    reset_all(ctx)
    if leg_prop is not None:
        set_prop(ctx, leg_prop, ctx.prop_defaults.get(leg_prop, 1.0))
    set_channels(ctx.pbs["CTRL_torso"], [("loc", torso_ch[0], torso_ch[1])], 1.0)
    evaluate(ctx)


def c_foot(ctx):
    if ctx.mesh is None:
        raise Blocked(f"{MESH} missing")
    clamps = ctx.section("clamps")
    ikfk = ctx.section("ikfk")
    torso_ch = torso_down_channel(ctx)
    mw = ctx.rig.matrix_world
    meas, ok = {"ready_pose": {"CTRL_torso_loc_axis": torso_ch[0], "value_m": rnd(torso_ch[1], 5),
                               "axis_world_z_dot": rnd(torso_ch[2], 5)}}, True
    t0 = time.time()
    for s in SIDES:
        sm = {}
        leg = ikfk.get(f"leg_{s}")
        if leg is None:
            meas[s], ok = f"blocked: manifest ikfk.leg_{s} missing", False
            continue
        ikc = leg.get("ik_constraint") or {}
        mch = leg.get("mch_ik") or []
        calf = ikc.get("bone") or (mch[1] if len(mch) > 1 else None)
        tgt = ikc.get("target")
        if calf not in ctx.pbs or tgt not in ctx.pbs:
            meas[s], ok = (f"blocked: IK reach bones missing (calf {calf!r}, target {tgt!r}; "
                           f"manifest ikfk.leg_{s} mch_ik / ik_constraint.target)"), False
            continue
        idx = shoe_verts(ctx, f"shoe_{s}")
        leg_prop = leg.get("prop", f"leg_ik_fk_{s}")
        if props_holder(ctx, leg_prop) is None:
            leg_prop = None
        sm["leg_prop"] = {"name": leg.get("prop"), "value_used": ctx.prop_defaults.get(leg.get("prop")),
                          "manifest_ik_value": leg.get("ik_value")}
        sm["reach_bones"] = [f"{calf} tail", f"{tgt} head"]
        for fp in FOOT_PROPS:
            name = f"{fp}_{s}"
            cl = clamps.get(name)
            if cl is None or props_holder(ctx, name) is None:
                sm[name] = "blocked: " + ("clamp missing in manifest" if cl is None else "prop missing")
                ok = False
                continue
            lo, hi = float(cl[0]), float(cl[1])
            judged_slip = fp in SLIP_JUDGED
            ready_pose(ctx, leg_prop, torso_ch)
            r = {"reach": [float("inf"), -float("inf")], "z": [float("inf"), -float("inf")],
                 "slip": 0.0, "slip_at": 0.0, "n": 0}
            first_fail = [None]

            def measure(v):
                set_prop(ctx, name, v)
                evaluate(ctx)
                track_def(ctx, "G6.7", {n: ctx.pbs[n].matrix for n in ctx.def_names() if n in ctx.pbs})
                reach = ((mw @ ctx.pbs[calf].tail) - (mw @ ctx.pbs[tgt].head)).length
                w = shoe_world(ctx, idx)
                return reach, w, float(w[:, 2].min())

            def check(v, reach, z, S):
                r["n"] += 1
                r["reach"] = [min(r["reach"][0], reach), max(r["reach"][1], reach)]
                r["z"] = [min(r["z"][0], z), max(r["z"][1], z)]
                if S > r["slip"]:
                    r["slip"], r["slip_at"] = S, v
                fails = []
                if reach > PLANT_REACH:
                    fails.append("reach")
                if not (PLANT_Z[0] <= z <= PLANT_Z[1]):
                    fails.append("z")
                if judged_slip and S > PLANT_SLIP:
                    fails.append("slip")
                if fails and first_fail[0] is None:
                    first_fail[0] = {"value": rnd(v, 3), "fails": fails, "reach_mm": mm(reach),
                                     "z_mm": mm(z), "slip_cum_mm": mm(S)}

            reach0, w0, z0 = measure(0.0)
            check(0.0, reach0, z0, 0.0)
            ends = {}
            for end in (hi, lo):
                S, wp, zp = 0.0, w0, z0
                for v in path_values(end, PROP_STEP):
                    reach, w, z = measure(v)
                    c_k = wp[:, 2] <= zp + CONTACT_TOL          # contact set of step k
                    S += float(np.mean(np.hypot(w[c_k, 0] - wp[c_k, 0], w[c_k, 1] - wp[c_k, 1])))
                    check(v, reach, z, S)
                    wp, zp = w, z
                ends[f"S_at_{rnd(end, 3)}_mm"] = mm(S)
            lok = first_fail[0] is None
            ok = ok and lok
            hh = props_holder(ctx, name)
            sm[name] = {"clamp": [lo, hi], "n": r["n"],
                        "reach_mm_min_max": [mm(x) for x in r["reach"]],
                        "z_mm_min_max": [mm(x) for x in r["z"]],
                        ("slip_cum_mm_max" if judged_slip else "slip_cum_mm_max (report)"): mm(r["slip"]),
                        "slip_cum_max_at": rnd(r["slip_at"], 3), "slip_cum_at_ends": ends,
                        "first_fail": first_fail[0], "ok": lok,
                        "prop_ui_range (report)": ui_range(hh, name)}
        meas[s] = sm
    reset_all(ctx)
    meas["seconds"] = rnd(time.time() - t0, 1)
    return meas, ok


# ---------------------------------------------------------------- G6.6
def ik_constraints(ctx):
    out = []
    for pb in ctx.pbs:
        for con in pb.constraints:
            if con.type != "IK":
                continue
            chain, b = [], pb
            cnt = int(getattr(con, "chain_count", 0))
            while b is not None and (cnt == 0 or len(chain) < cnt):
                chain.append(b.name)
                b = b.parent
            out.append((pb.name, con, chain))
    return out


def c_scale(ctx):
    iks = ik_constraints(ctx)
    stretch_on = [f"{b}/{c.name}" for b, c, _ch in iks if getattr(c, "use_stretch", False)]
    ik_stretch = {}
    for _b, _c, chain in iks:
        for n in chain:
            v = float(ctx.pbs[n].ik_stretch)
            if v != 0.0:
                ik_stretch[n] = rnd(v, 5)
    all_ik_stretch = {pb.name: rnd(pb.ik_stretch, 5) for pb in ctx.pbs if pb.ik_stretch != 0.0}
    per = {k: {"max_dev": rnd(v["max_dev"], 7), "bone": v["bone"], "n_samples": v["n"]}
           for k, v in ctx.scale.items()}
    worst = max((v["max_dev"] for v in ctx.scale.values()), default=float("nan"))
    nan = sum(ctx.nan.values())
    ran = [k for k in ("G6.3", "G6.5", "G6.7") if k in ctx.scale]
    ok = (bool(ran) and math.isfinite(worst) and worst <= SCALE_TOL and nan == 0
          and not stretch_on and not ik_stretch)
    return {"max_scale_dev": rnd(worst, 7), "per_source": per, "nan_bone_samples": nan,
            "sources_measured": ran, "n_ik_constraints": len(iks),
            "ik_constraints": [f"{b}/{c.name} chain={len(ch)}" for b, c, ch in iks],
            "ik_use_stretch_true": stretch_on, "ik_chain_ik_stretch_nonzero": ik_stretch,
            "any_bone_ik_stretch_nonzero (report)": all_ik_stretch}, ok


# ---------------------------------------------------------------- G6.8
def _crt():
    for n in ("ucrtbase", "msvcrt"):
        try:
            return ctypes.CDLL(n)
        except Exception:
            continue
    return None


def capture_fd(action):
    """Run action() with C-level fd 1/2 redirected to a temp file; returns (text, error)."""
    crt = _crt()

    def flush():
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except Exception:
            pass
        if crt is not None:
            try:
                crt.fflush(None)
            except Exception:
                pass

    flush()
    tmp = tempfile.TemporaryFile()
    saved = (os.dup(1), os.dup(2))
    err = None
    try:
        os.dup2(tmp.fileno(), 1)
        os.dup2(tmp.fileno(), 2)
        try:
            if crt is not None:
                crt.puts(b"__G6_CAPTURE_PROBE__")
            action()
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
        flush()
    finally:
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        os.close(saved[0])
        os.close(saved[1])
    tmp.seek(0)
    text = tmp.read().decode("utf-8", "replace")
    tmp.close()
    return text, err


def capture_cycles(ctx):
    """Force a depsgraph relations rebuild (link/unlink a temporary empty) while capturing output."""
    def action():
        e = bpy.data.objects.new("_g6_relations_probe", None)
        bpy.context.scene.collection.objects.link(e)
        bpy.context.view_layer.update()
        bpy.data.objects.remove(e, do_unlink=True)
        bpy.context.view_layer.update()

    text, err = capture_fd(action)
    lines = [ln for ln in text.splitlines() if ln.strip() and "__G6_CAPTURE_PROBE__" not in ln]
    for ln in lines:
        print(f"[{GATE}/{CHECKER}] captured: {ln}")
    cyc = [ln for ln in lines if "cycle" in ln.lower()]
    m = [re.search(r"Detected\s+(\d+)\s+dependency cycles", ln) for ln in lines]
    n_det = sum(int(x.group(1)) for x in m if x)
    return {"capture_works": "__G6_CAPTURE_PROBE__" in text, "error": err,
            "dependency_cycle_lines": sum(1 for ln in lines if "Dependency cycle" in ln),
            "detected_count": n_det, "cycle_lines": _cap(cyc, 30), "n_captured_lines": len(lines)}


def _con_targets(con):
    out = []
    for at, asub in (("target", "subtarget"), ("pole_target", "pole_subtarget")):
        t = getattr(con, at, None)
        if t is not None and isinstance(t, bpy.types.ID):
            out.append((t, getattr(con, asub, "") or ""))
    for tt in getattr(con, "targets", None) or []:
        t = getattr(tt, "target", None)
        if t is not None:
            out.append((t, getattr(tt, "subtarget", "") or ""))
    return out


def dep_edges(ctx):
    """Static bone/object dependency edges: node -> set(nodes it depends on)."""
    rig = ctx.rig
    edges = {}

    def add(a, b):
        edges.setdefault(a, set()).add(b)
        edges.setdefault(b, set())

    def node(t, s):
        if t.name == rig.name and s:
            return "B:" + s
        return "OB:" + t.name

    muted = 0
    for pb in rig.pose.bones:
        edges.setdefault("B:" + pb.name, set())
        if pb.parent is not None:
            add("B:" + pb.name, "B:" + pb.parent.name)
        for con in pb.constraints:
            if con.mute:
                muted += 1
            owners = [pb.name]
            if con.type == "IK":
                owners, b = [], pb
                cnt = int(getattr(con, "chain_count", 0))
                while b is not None and (cnt == 0 or len(owners) < cnt):
                    owners.append(b.name)
                    b = b.parent
            for t, s in _con_targets(con):
                for o in owners:
                    add("B:" + o, node(t, s))
    ad = rig.animation_data
    for fc in (ad.drivers if ad is not None else []):
        m = re.match(r'pose\.bones\["(.+?)"\]', fc.data_path)
        owner = "B:" + m.group(1) if m else "OB:" + rig.name
        for v in fc.driver.variables:
            for t in v.targets:
                if t.id is None:
                    continue
                if t.id.name == rig.name and t.id_type == "OBJECT":
                    mm_ = re.match(r'pose\.bones\["(.+?)"\]', t.data_path or "")
                    if getattr(t, "bone_target", ""):
                        add(owner, "B:" + t.bone_target)
                    elif mm_:
                        add(owner, "B:" + mm_.group(1))
                    else:
                        add(owner, "OB:" + rig.name)
                else:
                    add(owner, "ID:" + t.id.name)
    for ob in bpy.context.scene.objects:
        if ob.name == rig.name:
            continue
        on = "OB:" + ob.name
        edges.setdefault(on, set())
        if ob.parent is not None:
            if ob.parent.name == rig.name and ob.parent_type == "BONE" and ob.parent_bone:
                add(on, "B:" + ob.parent_bone)
            else:
                add(on, "OB:" + ob.parent.name)
        for mod in getattr(ob, "modifiers", []):
            if mod.type == "ARMATURE" and getattr(mod, "object", None) is not None \
                    and mod.object.name == rig.name:
                for pb in rig.pose.bones:
                    add(on, "B:" + pb.name)
        for con in ob.constraints:
            for t, s in _con_targets(con):
                add(on, node(t, s))
    for con in rig.constraints:
        for t, s in _con_targets(con):
            add("OB:" + rig.name, node(t, s))
    return edges, muted


def sccs(edges):
    """Tarjan (iterative); returns strongly connected components with > 1 node or a self loop."""
    index, low, on, stack, out = {}, {}, set(), [], []
    counter = [0]
    for root in edges:
        if root in index:
            continue
        work = [(root, iter(sorted(edges[root])))]
        index[root] = low[root] = counter[0]
        counter[0] += 1
        stack.append(root)
        on.add(root)
        while work:
            v, it = work[-1]
            adv = False
            for w in it:
                if w not in index:
                    index[w] = low[w] = counter[0]
                    counter[0] += 1
                    stack.append(w)
                    on.add(w)
                    work.append((w, iter(sorted(edges.get(w, ())))))
                    adv = True
                    break
                if w in on:
                    low[v] = min(low[v], index[w])
            if adv:
                continue
            work.pop()
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[v])
            if low[v] == index[v]:
                comp = []
                while True:
                    w = stack.pop()
                    on.discard(w)
                    comp.append(w)
                    if w == v:
                        break
                if len(comp) > 1 or v in edges.get(v, ()):
                    out.append(sorted(comp))
    return out


def c_cycle(ctx):
    spaces = (ctx.man or {}).get("spaces") or {}
    sp = spaces.get("hand_ik_space_r")
    m_vals = list((sp or {}).get("values") or [])
    man_weapon = [v for v in m_vals if "weapon" in str(v).lower()]
    h = props_holder(ctx, "hand_ik_space_r")
    ui = ui_range(h, "hand_ik_space_r") if h is not None else None
    ui_max = None if not ui else ui.get("max")
    # int prop: a weapon slot would be index >= len(values) (contract: weapon = 3); description
    # text is reported only (it may mention "no weapon")
    ui_weapon = bool(ui) and ui_max is not None and bool(m_vals) and ui_max > len(m_vals) - 1
    emb = None
    mt = bpy.data.texts.get(TEXT_MANIFEST)
    if mt is not None:
        try:
            ev = (json.loads(mt.as_string()).get("spaces") or {}).get("hand_ik_space_r") or {}
            emb = [v for v in ev.get("values") or [] if "weapon" in str(v).lower()]
        except Exception as e:
            emb = f"error: {e}"
    pairs, _exp, _src = space_constraints(ctx, "hand_ik_space_r", sp or {})
    r_targets = {}
    for b, c in pairs:
        pb = ctx.pbs.get(b)
        con = pb.constraints.get(c) if pb is not None else None
        if con is not None:
            r_targets[f"{b}/{c}"] = [f"{t.name}:{s}" for t, s in _con_targets(con)]
    mch = (sp or {}).get("mch")
    if mch and mch in ctx.pbs:
        for con in ctx.pbs[mch].constraints:
            r_targets.setdefault(f"{mch}/{con.name}", [f"{t.name}:{s}" for t, s in _con_targets(con)])
    weapon_targets = [k for k, v in r_targets.items() if any("weapon" in x.lower() for x in v)]
    edges, muted = dep_edges(ctx)
    cyc = sccs(edges)
    cap = ctx.cycle_capture or {}
    cap_ok = (not cap.get("capture_works")) or (cap.get("dependency_cycle_lines", 0) == 0
                                                 and cap.get("detected_count", 0) == 0)
    no_weapon = (sp is not None and not man_weapon and not ui_weapon and not weapon_targets
                 and not (isinstance(emb, list) and emb))
    ok = no_weapon and not cyc and cap_ok and h is not None
    return {"manifest_hand_ik_space_r_values": m_vals if sp is not None else "missing",
            "manifest_weapon_values": man_weapon, "props_ui_hand_ik_space_r": ui,
            "props_ui_weapon": ui_weapon, "embedded_manifest_weapon_values": emb,
            "hand_ik_space_r_constraint_targets": r_targets, "weapon_targets": weapon_targets,
            "blender_capture": cap, "graph_nodes": len(edges),
            "graph_edges": sum(len(v) for v in edges.values()), "graph_cycles": _cap(cyc, 10),
            "muted_constraints_counted": muted}, ok


# ---------------------------------------------------------------- G6.9
def c_ops(ctx):
    info = dict(ctx.ops_info or {})
    ok = (info.get("text_exists") and info.get("use_module")
          and all((info.get("registered_after") or {}).get(o) for o in OPS)
          and info.get("addon_import") == "ok" and info.get("same_code") is True)
    return info, bool(ok)


# ---------------------------------------------------------------- G6.11
READY_KNEE = (40.0, 60.0)     # G6.11 |knee theta| range (deg)
READY_ELBOW = (10.0, 30.0)    # G6.11 |elbow theta| range (deg)
READY_FOOT_XY = 0.001         # G6.11 FK leg: DEF foot world head XY shift by the operator (m)


def c_ready(ctx):
    if not ctx.ops.get("ready_pose"):
        raise Blocked("operator goblin.ready_pose not registered (missing)")
    if ctx.mesh is None:
        raise Blocked(f"{MESH} missing")
    ikfk = ctx.section("ikfk")
    chains = bend_chains(ctx)
    arms = sorted(k for k in chains if k.startswith("arm"))
    legs = sorted(k for k in chains if k.startswith("leg"))
    if not arms or not legs:
        raise Blocked(f"ikfk arm/leg chains missing (have {sorted(chains)})")
    mw = ctx.rig.matrix_world
    meas, ok = {}, True
    combos = [(f"arm_{a[:2]}_leg_{b[:2]}", a, b) for a in ("fk_value", "ik_value")
              for b in ("fk_value", "ik_value")]
    for case, key, leg_key in combos:
        cm = {}
        reset_all(ctx)
        evaluate(ctx)
        if ctx.ops.get("reset_rig"):
            rr, re_ = run_op(ctx, "reset_rig")
        else:
            rr, re_ = None, "reset_rig not registered"
        evaluate(ctx)
        cm["reset_rig"] = {"result": rr, "error": re_}
        arm_set = {}
        for k in arms:
            ch = ikfk[k]
            v = ch.get(key, 0.0 if key == "fk_value" else 1.0)
            set_prop(ctx, ch["prop"], v)
            arm_set[ch["prop"]] = v
        leg_set, foot_def = {}, {}
        for s in SIDES:
            ch = ikfk.get(f"leg_{s}") or {}
            if ch.get("prop") and props_holder(ctx, ch["prop"]) is not None:
                v = ch.get(leg_key, 0.0 if leg_key == "fk_value" else 1.0)
                set_prop(ctx, ch["prop"], v)
                leg_set[ch["prop"]] = v
            d = ch.get("def") or []
            if d and d[-1] in ctx.pbs:
                foot_def[s] = d[-1]
        evaluate(ctx)
        foot0 = {s: mw @ ctx.pbs[n].head for s, n in foot_def.items()}
        r, err = run_op(ctx, "ready_pose")
        evaluate(ctx)
        cm["ready_pose"] = {"result": r, "error": err}
        cm["arm_prop_set"] = arm_set
        cm["arm_prop_after"] = {p: get_prop(ctx, p) for p in arm_set}
        cm["leg_prop_set"] = leg_set
        cm["leg_prop_after"] = {p: get_prop(ctx, p) for p in leg_set}
        leg_fk = leg_key == "fk_value"
        knee = {k: bend_theta(ctx, *chains[k]) for k in legs}
        elbow = {k: bend_theta(ctx, *chains[k]) for k in arms}
        cm["knee_theta_deg"] = {k: rnd(v, 3) for k, v in knee.items()}
        cm["elbow_theta_deg"] = {k: rnd(v, 3) for k, v in elbow.items()}
        knee_ok = all(READY_KNEE[0] <= abs(v) <= READY_KNEE[1] for v in knee.values())
        elbow_ok = all(READY_ELBOW[0] <= abs(v) <= READY_ELBOW[1] for v in elbow.values())
        feet, feet_ok = {}, True
        for s in SIDES:
            leg = ikfk.get(f"leg_{s}") or {}
            ikc = leg.get("ik_constraint") or {}
            mch = leg.get("mch_ik") or []
            calf = ikc.get("bone") or (mch[1] if len(mch) > 1 else None)
            tgt = ikc.get("target")
            if calf not in ctx.pbs or tgt not in ctx.pbs:
                feet[s], feet_ok = f"blocked: IK reach bones missing ({calf!r}, {tgt!r})", False
                continue
            reach = ((mw @ ctx.pbs[calf].tail) - (mw @ ctx.pbs[tgt].head)).length
            z = float(shoe_world(ctx, shoe_verts(ctx, f"shoe_{s}"))[:, 2].min())
            fn = foot_def.get(s)
            if fn is not None:
                f1 = mw @ ctx.pbs[fn].head
                dxy = math.hypot(f1.x - foot0[s].x, f1.y - foot0[s].y)
            else:
                dxy = float("inf")
            z_ok = PLANT_Z[0] <= z <= PLANT_Z[1]
            if leg_fk:
                fok = z_ok and dxy <= READY_FOOT_XY
            else:
                fok = z_ok and reach <= PLANT_REACH
            feet_ok = feet_ok and fok
            feet[s] = {"leg_mode": "FK" if leg_fk else "IK", "reach_mm": mm(reach), "min_z_mm": mm(z), "def_foot": fn,
                       "def_foot_xy_shift_mm": mm(dxy), "judged": ("z, foot xy shift" if leg_fk
                                                                   else "z, reach"), "ok": fok}
        cm["feet"] = feet
        sdev, sbone, nan = 0.0, None, 0
        for n in ctx.def_names():
            m = ctx.pbs[n].matrix
            if not mat_finite(m):
                nan += 1
                continue
            for i in range(3):
                d = abs(m.to_3x3().col[i].length - 1.0)
                if d > sdev:
                    sdev, sbone = d, n
        cm["def_scale_max_dev"] = rnd(sdev, 7)
        cm["def_scale_bone"] = sbone
        cm["nan_bones"] = nan
        cok = (not err and knee_ok and elbow_ok and feet_ok and sdev <= SCALE_TOL and nan == 0)
        cm["checks"] = {"knee": knee_ok, "elbow": elbow_ok, "feet": feet_ok,
                        "scale": sdev <= SCALE_TOL and nan == 0, "op": not err}
        cm["ok"] = cok
        ok = ok and cok
        meas[case] = cm
    reset_all(ctx)
    evaluate(ctx)
    return meas, ok


# ---------------------------------------------------------------- G6.12
def mouth_values(ctx):
    """(original key block value, evaluated key block value or None) of GOB_mesh mouth_open."""
    key = ctx.mesh.data.shape_keys
    orig = float(key.key_blocks[MOUTH_KEY].value)
    try:
        ke = key.evaluated_get(bpy.context.evaluated_depsgraph_get())
        evv = float(ke.key_blocks[MOUTH_KEY].value)
    except Exception:
        evv = None
    return orig, evv


def _norm_path(p):
    return (p or "").replace("'", '"').replace(" ", "")


def c_mouth(ctx):
    if ctx.mesh is None:
        raise Blocked(f"{MESH} missing")
    h = props_holder(ctx, MOUTH_PROP)
    if h is None:
        raise Blocked(f"property {MOUTH_PROP} missing on {PROPS}")
    key = ctx.mesh.data.shape_keys
    if key is None or key.key_blocks.get(MOUTH_KEY) is None:
        raise Blocked(f"{MESH} has no shape key {MOUTH_KEY}")
    meas = {}
    v0 = h[MOUTH_PROP]
    ui = ui_range(h, MOUTH_PROP) or {}
    ud = ui_default(h, MOUTH_PROP)
    prop_ok = (isinstance(v0, float) and ui.get("min") == MOUTH_RANGE[0] and ui.get("max") == MOUTH_RANGE[1]
               and ud is not None and abs(ud - 0.0) <= MOUTH_TOL
               and CONTRACT_DEFAULTS.get(MOUTH_PROP) == 0.0)
    meas["prop"] = {"holder": "pose bone" if h == ctx.rig.pose.bones.get(PROPS) else "data bone",
                    "type": type(v0).__name__, "value_in_file": v0, "ui": ui, "ui_default": ud,
                    "contract_default": CONTRACT_DEFAULTS.get(MOUTH_PROP), "ok": bool(prop_ok)}
    # driver
    ad = key.animation_data
    want = f'key_blocks["{MOUTH_KEY}"].value'
    fcs = [fc for fc in (ad.drivers if ad is not None else []) if _norm_path(fc.data_path) == want]
    want_t = _norm_path(f'pose.bones["{PROPS}"]["{MOUTH_PROP}"]')
    drv = {"n_driver_fcurves": len(fcs), "data_path": want}
    drv_ok = False
    if len(fcs) == 1:
        fc = fcs[0]
        d = fc.driver
        vs = []
        for var in d.variables:
            tg = []
            for t in var.targets:
                tg.append({"id_type": t.id_type, "id": t.id.name if t.id is not None else None,
                           "data_path": t.data_path})
            vs.append({"name": var.name, "type": var.type, "targets": tg})
        drv.update({"type": d.type, "expression": d.expression, "is_valid": bool(d.is_valid),
                    "use_self": bool(d.use_self), "mute": bool(fc.mute), "variables": vs,
                    "keyframe_points_report": len(fc.keyframe_points),
                    "modifiers_report": [m.type for m in fc.modifiers]})
        v1 = d.variables[0] if len(d.variables) == 1 else None
        t1 = v1.targets[0] if v1 is not None else None
        drv_ok = (d.type == "AVERAGE" and bool(d.is_valid) and not fc.mute and v1 is not None
                  and v1.type == "SINGLE_PROP" and t1.id_type == "OBJECT" and t1.id == ctx.rig
                  and _norm_path(t1.data_path) == want_t)
    drv["ok"] = bool(drv_ok)
    meas["driver"] = drv
    # response 0 -> 1
    reset_all(ctx)
    evaluate(ctx)
    rows, resp_ok = [], True
    for v in MOUTH_STEPS:
        set_prop(ctx, MOUTH_PROP, v)
        evaluate(ctx)
        o, e = mouth_values(ctx)
        good = e is not None and abs(e - v) <= MOUTH_TOL
        resp_ok = resp_ok and good
        rows.append({"prop": v, "prop_read_back": get_prop(ctx, MOUTH_PROP), "key_value_evaluated": e,
                     "key_value_original_report": o, "ok": good})
    meas["response"] = rows
    # reset_rig / ready_pose -> 0
    ops, ops_ok = {}, True
    for op in ("reset_rig", "ready_pose"):
        reset_all(ctx)
        set_prop(ctx, MOUTH_PROP, MOUTH_PROBE)
        evaluate(ctx)
        if not ctx.ops.get(op):
            ops[op] = f"operator goblin.{op} not registered"
            ops_ok = False
            continue
        r, err = run_op(ctx, op)
        evaluate(ctx)
        after = get_prop(ctx, MOUTH_PROP)
        o, e = mouth_values(ctx)
        good = (err is None and abs(float(after)) <= MOUTH_TOL and e is not None and abs(e) <= MOUTH_TOL)
        ops_ok = ops_ok and good
        ops[op] = {"set_before": MOUTH_PROBE, "result": r, "error": err, "prop_after": after,
                   "key_value_evaluated_after": e, "ok": good}
    meas["reset_ready"] = ops
    reset_all(ctx)
    evaluate(ctx)
    return meas, bool(prop_ok and drv_ok and resp_ok and ops_ok)


# ---------------------------------------------------------------- criterion table
def specs():
    """[(id, threshold, note)] in evidence order + execution order + functions."""
    table = [
        ("G6.1", "all manifest controls exist; bone collections CTRL/MCH/DEF exist; only the CTRL "
                 "subtree visible (MCH, DEF hidden; no visible bone outside CTRL; controls in CTRL; "
                 "no DEF/MCH bone in CTRL); colour class L blue / R red / C yellow; custom shape set",
         "bone-collection effective visibility (is_visible_effectively), bone.hide; colour: "
         + COLOR_METHOD, c_structure),
        ("G6.2", f"max abs(DEF pose matrix - canonical rest_matrix) <= {REST_TOL:g} (armature space)",
         "all CTRL identity (see module doc), PROPS contract defaults, update; pose_bone.matrix vs "
         "canonical_skeleton.json rest_matrix, 24 bones; driver validity reported only", c_rest),
        ("G6.3", f"per chain and direction: DEF world diff before/after snap <= {SNAP_POS * 1000:g} mm "
                 f"and <= {SNAP_ROT:g} deg; no operator error",
         f"per manifest ikfk chain: {N_RANDOM} random poses (random.Random({SEED}), uniform in each "
         "sweep range of fk/ik/pole controls); fk_to_ik: prop = FK value, snap op, ik_to_fk: prop = IK "
         "value, snap op; if the op leaves the prop unchanged it is forced to the target value "
         "(counted); DEF = chain 'def' list; direction values identified by which controls the op "
         "moves (fallback: name); FK value detected by moving the first FK control", c_snap),
        ("G6.4", f"per space value pair a->b: target world diff <= {SNAP_POS * 1000:g} mm and "
                 f"<= {SNAP_ROT:g} deg; prop == b; driven constraint influences == expected for b; "
                 "driven influences differ between a and b; no operator error",
         f"other controls random at {CONTEXT_POSE_FRAC:g} x sweep range (seed {SEED}) so spaces differ; "
         f"prop = a set directly; target pose = {SPACE_POSE_FRAC:g} x sweep end per channel; "
         "goblin.switch_space(prop, b); driven constraints = manifest spaces.<p>.influence entries "
         "marked 'driver' (expected '== k' parsed) else drivers referencing PROPS[prop] (expected = "
         "influence with prop set to b directly); influences read from the evaluated rig", c_space),
        ("G6.5", f"0 pops over all control channels and no NaN; an adjacent-sample interval whose DEF "
                 f"rotation change exceeds step x {JUMP_K:g} + {JUMP_C:g} deg (loc step counted in cm) is a "
                 f"pop candidate, bisected toward the larger change {REFINE_LEVELS} times (h/16); pop "
                 f"if change(h/16) > {POP_RATIO:g} x change(h); (b) 0 branch flips: adjacent samples "
                 f"with opposite sign of theta and |theta| > {FLIP_MIN_DEG:g} deg on both",
         f"manifest sweep, one channel at a time from identity, {ROT_STEP:g} deg / "
         f"{LOC_STEP * 100:g} cm steps (> {MAX_SAMPLES} samples -> subsampled, listed); ik_fk prop "
         "set so the control drives DEF (FK controls FK value, IK/pole IK value); change = max over "
         "24 DEF world rotations of the quaternion angle between the interval ends; every candidate "
         "listed with jumps h, h/2, h/4, h/8, h/16 and class; (b) per manifest ikfk chain theta = "
         "atan2(dot(cross(u,l), X_upper), dot(u,l)), u/l = world direction of DEF def[0]/def[1] "
         "(tail - head), X_upper = world local X of def[0], on every main sweep sample; ok = a and b "
         "(ok_a_continuity / ok_b_branch_flip in measured)", c_sweep),
        ("G6.6", f"DEF scale (column norms of armature pose matrix) = 1 +- {SCALE_TOL:g} across "
                 "G6.3/G6.5/G6.7 samples; IK constraints use_stretch False; ik_stretch 0 on IK chain "
                 "bones",
         "scale tracked on every evaluated sample of G6.2/G6.3/G6.5/G6.7; IK chain = owner + "
         "chain_count parents (0 = to root)", c_scale),
        ("G6.7", f"planted at every value of all 8 props: (1) IK reach <= {PLANT_REACH * 1000:g} mm, "
                 f"(2) lowest shoe_x vertex z in [{PLANT_Z[0] * 1000:g}, {PLANT_Z[1] * 1000:g}] mm, "
                 f"(3) foot_roll / foot_bank only: cumulative contact slip <= {PLANT_SLIP * 1000:g} mm",
         f"ready pose: all CTRL identity, CTRL_torso loc on its local axis whose world direction is +Z "
         f"= {PLANT_TORSO_DZ * 1000:g} mm world Z, leg_ik_fk contract default (IK), foot IK at rest; "
         f"each of foot_roll/foot_bank/heel_twist/toe_twist per side swept alone over the full "
         f"manifest clamps in {PROP_STEP:g} deg steps; reach = |world tail of ikfk.leg_x "
         f"ik_constraint.bone (MCH_calf_ik_x) - world head of ik_constraint.target (MCH_roll_foot_x)|; "
         f"z = min world z of evaluated GOB_mesh verts of faces with part_id shoe_x; cumulative slip "
         f"S(v): from 0 toward each end one step at a time, contact set C_k = shoe verts with z <= "
         f"lowest z of step k + {CONTACT_TOL * 1000:g} mm, S += mean XY displacement of C_k from step k "
         f"to k+1 (twist props: S reported only)", c_foot),
        ("G6.8", "hand_ik_space_r: no 'weapon' value in manifest / embedded manifest / PROPS UI range "
                 "and no weapon target on its driven constraints; 0 cycles in the static constraint "
                 "graph; 0 Blender 'Dependency cycle' lines",
         "Blender output captured at C fd level during a forced relations rebuild (link/unlink a temp "
         "empty) - if capture_works is False only the graph is judged; graph = bone parent, "
         "constraint targets (IK: whole chain), driver variable targets, object parents, "
         "armature modifiers; Tarjan SCC", c_cycle),
        ("G6.9", f"blend text {TEXT_UI} exists with use_module; after its register() goblin."
                 + ", goblin.".join(OPS) + " registered; addon file imports; same code (whitespace-"
                 "normalised)",
         "Text.as_module().register(); bpy.ops.goblin.<op>.get_rna_type(); addon exec via importlib "
         "(not registered); embedded manifest equality reported only", c_ops),
        ("G6.11", f"goblin.ready_pose in all 4 cases arm FK/IK x leg FK/IK: |knee theta| in "
                  f"[{READY_KNEE[0]:g}, {READY_KNEE[1]:g}] deg, |elbow theta| in [{READY_ELBOW[0]:g}, "
                  f"{READY_ELBOW[1]:g}] deg, both feet planted (IK leg: reach <= {PLANT_REACH * 1000:g} mm "
                  f"and lowest shoe z in [{PLANT_Z[0] * 1000:g}, {PLANT_Z[1] * 1000:g}] mm; FK leg: lowest "
                  f"shoe z in the same range and DEF foot world head XY shift by the operator <= "
                  f"{READY_FOOT_XY * 1000:g} mm), DEF scale 1 +- {SCALE_TOL:g}, no operator error",
         "per case: all CTRL identity + PROPS defaults, goblin.reset_rig, arm_ik_fk_l/r and "
         "leg_ik_fk_l/r = manifest fk_value / ik_value, goblin.ready_pose, update; theta as G6.5 (b) "
         "on ikfk leg_x / arm_x def chains; reach / shoe z as G6.7 (reach reported for FK legs too); "
         "foot shift = XY of world head of ikfk leg_x def[-1] after vs before the call; scale = column "
         "norms of the 24 DEF armature pose matrices; restored afterwards (in memory only)", c_ready),
        ("G6.12", f"PROPS {MOUTH_PROP}: float, UI min {MOUTH_RANGE[0]:g} max {MOUTH_RANGE[1]:g}, UI default 0, "
                  f"contract default 0; exactly one driver on {MESH} shape_keys key_blocks[\"{MOUTH_KEY}\"].value: "
                  f"type AVERAGE, valid, one SINGLE_PROP variable targeting OBJECT {ARM} "
                  f"pose.bones[\"{PROPS}\"][\"{MOUTH_PROP}\"]; evaluated key value = property value "
                  f"(|diff| <= {MOUTH_TOL:g}) at {list(MOUTH_STEPS)}; goblin.reset_rig and goblin.ready_pose "
                  f"(called with {MOUTH_PROP} = {MOUTH_PROBE:g}) leave the property and the evaluated key value "
                  f"at 0 (<= {MOUTH_TOL:g}), no operator error",
         "2026-09-26 HR2 (rig/data/hr2_contract.md). All CTRL identity + PROPS contract defaults first; the "
         "property is set directly on the PROPS holder, then GOB_rig.update_tag + view_layer.update; evaluated "
         "value = GOB_mesh.data.shape_keys.evaluated_get(depsgraph).key_blocks value (original datablock value "
         "reported); driver F-curve keyframes / modifiers reported only; restored afterwards (in memory only)",
         c_mouth),
    ]
    order = ["G6.1", "G6.2", "G6.3", "G6.4", "G6.5", "G6.7", "G6.6", "G6.8", "G6.9", "G6.11", "G6.12"]
    return table, order


# ---------------------------------------------------------------- main
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    blend = None
    if "--blend" in argv:
        i = argv.index("--blend")
        blend = goblib.RIG / argv[i + 1]
    return blend


def main():
    global BLEND, CHECKER
    dbg = parse_args()
    if dbg is not None and dbg.resolve() != BLEND.resolve():
        BLEND, CHECKER = dbg, CHECKER + "_debug"
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ctx = Ctx()

    if not BLEND.exists():
        _STATE["missing"].append(rel(BLEND))
        ctx.miss["blend"] = f"missing input: {rel(BLEND)}"
    else:
        loaded = bpy.data.filepath
        if not loaded or Path(loaded).resolve() != BLEND.resolve():
            print(f"[{GATE}/{CHECKER}] opening {rel(BLEND)} (loaded: {loaded!r})")
            bpy.ops.wm.open_mainfile(filepath=str(BLEND))
        ev.add_input(BLEND)
    ev.add_stage_inputs("s06")
    if ADDON.exists():
        ev.add_input(ADDON)
    else:
        _STATE["missing"].append(rel(ADDON))
    ctx.man = _load_json(MANIFEST_JSON, "manifest", ctx)
    ctx.canon = _load_json(CANON_JSON, "canon", ctx)
    ctx.parts = _load_json(PARTS_JSON, "parts", ctx)

    table, order = specs()
    by_id = {t[0]: t for t in table}
    results = {}
    try:
        blocking = [ctx.miss[k] for k in ("blend", "manifest", "canon") if ctx.miss.get(k)]
        if not blocking:
            rig = bpy.data.objects.get(ARM)
            if rig is None or rig.type != "ARMATURE":
                blocking = [f"armature object {ARM} missing in {rel(BLEND)}"]
            else:
                ctx.rig = rig
                mo = bpy.data.objects.get(MESH)
                ctx.mesh = mo if mo is not None and mo.type == "MESH" else None
        if blocking:
            r = "blocked: " + "; ".join(blocking)
            for cid, th, note, _fn in table:
                ev.criterion(cid, r, th, False, note)
            return
        ctx.cycle_capture = capture_cycles(ctx)     # before anything else touches the depsgraph
        ctx.ops_info = register_ops(ctx)
        save_state(ctx)
        try:
            prepare(ctx)
            for cid in order:
                _c, _th, _note, fn = by_id[cid]
                t0 = time.time()
                try:
                    measured, ok = fn(ctx)
                except Blocked as e:
                    measured, ok = f"blocked: {e}", False
                except Exception as e:
                    tb = traceback.format_exc()
                    print(tb)
                    _STATE["errors"].append(cid)
                    measured, ok = f"error: {type(e).__name__}: {e}", False
                print(f"[{GATE}/{CHECKER}] {cid} done in {time.time() - t0:.1f}s")
                sys.stdout.flush()
                results[cid] = (measured, ok)
        finally:
            restore_state(ctx)
        for cid, th, note, _fn in table:
            measured, ok = results.get(cid, ("not run", False))
            ev.criterion(cid, measured, th, ok, note)
    finally:
        ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"] or _STATE["errors"]:
        if _STATE["missing"]:
            print("missing input: " + ", ".join(_STATE["missing"]))
        if _STATE["errors"]:
            print("criterion errors: " + ", ".join(_STATE["errors"]))
        sys.stdout.flush()
        sys.exit(2)
