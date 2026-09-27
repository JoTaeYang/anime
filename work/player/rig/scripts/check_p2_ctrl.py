"""check_p2_ctrl - Player P2.5 control-rig checker (task T241; spec work/player/d-02-player-rig.md section 2 gate P2.5
= the goblin G6 set; checker side).

Port of work/goblin_swing/rig/scripts/check_g6_ctrl.py (goblin G6.1 .. G6.9, G6.11; copied and adapted, the goblin
file is not imported or modified; goblib helpers replaced by the local shim `goblib` below).  Player changes:
  - rig PL_rig, mesh PL_mesh, operators bpy.ops.player.* (blend text + addon player_rig_ui.py)
  - DEF = the 41 bones of work/player/rig/data/canonical_skeleton.json (skeleton v1, incl. Root, sockets, Skirt_*)
  - contract defaults by rule: *arm_ik_fk* 0 (FK), *leg_ik_fk* 1 (IK), *_space* 0, foot_roll / foot_bank /
    heel_twist / toe_twist 0; other PROPS keys: their UI default
  - G6.7 stance: manifest clamps_reference.chosen_stance_mm (mm) or chosen_stance_m (m) when given, else -15 mm (goblin)
  - G6.8 weapon rule per side: hand_ik_space_<x> must not target the weapon of side <x> (goblin: right hand only)
  - G6.11 knee 40-60 / elbow 10-30 deg ranges are goblin numbers -> calibration (report) on this first player run;
    planted feet, scale and operator errors stay judged
  - G6.12 (goblin mouth) dropped: no mouth on the player
  - new rows: P2.5c DEF skeleton unchanged vs canonical + DEF not keyed / driven; P2.5d PROPS contract (defaults,
    ranges vs manifest); P2.5l reset_rig restores defaults; P2.5n Skirt_* follow Pelvis only, sockets follow their
    parents, weapon controls drive WeaponSocket_L / R; P2.5o control overview sheet
Inputs (defaults; override after "--"):
  --blend    work/player/rig/pl_r04_ctrl.blend        PL_rig (DEF 41 + CTRL / MCH / PROPS), PL_mesh, blend text
  --manifest work/player/rig/data/ctrl_manifest.json  goblin schema: controls, spaces, ikfk, clamps(_reference)
  --canon    work/player/rig/data/canonical_skeleton.json
  --parts    work/player/rig/data/parts.json          shoe_l / shoe_r part_id ({"part_id": {...}} accepted)
  --addon    work/player/rig/scripts/addon/player_rig_ui.py
  --out      work/player/inspect/P2/check_p2_ctrl.json (sheet: <out dir>/P2c_ctrl_sheet.png)
All poses / property values are changed in memory only and restored at the end; the blend and data files are never
saved or edited.
Run:    blender --background --factory-startup --python check_p2_ctrl.py [-- <overrides>]
Exit:   0 evidence written; 2 script error, per-criterion error or missing input (evidence still written).
Coordinates: front -Y, up +Z, character left (_L) +X, ground z = 0.  Lengths in m, reported in mm.
"""
import os
import sys

HERE_ = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE_)
sys.dont_write_bytecode = True   # no __pycache__ next to the scripts
import check_p2_deform as p2d  # noqa: E402  (generic helpers: sha256, rel, jsonable)

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

HERE = Path(HERE_)
PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE = "P2.5"
CHECKER = "check_p2_ctrl"
TASK = "T241"
CALIB = p2d.CALIB
P = {"blend": PLAYER_RIG / "pl_r04_ctrl.blend",
     "manifest": PLAYER_RIG / "data" / "ctrl_manifest.json",
     "canon": PLAYER_RIG / "data" / "canonical_skeleton.json",
     "parts": PLAYER_RIG / "data" / "parts.json",
     "addon": HERE / "addon" / "player_rig_ui.py",
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_ctrl.json"}
SHEET_NAME = "P2c_ctrl_sheet.png"
TEXT_UI = "player_rig_ui.py"
TEXT_MANIFEST = "player_manifest.json"
ARM = "PL_rig"
MESH = "PL_mesh"
PROPS = "PROPS"
OP_NS = "player"
OPS = ("snap_ikfk", "switch_space", "reset_rig", "ready_pose")


class goblib:  # noqa: N801  local shim for the goblib helpers the copied code calls
    @staticmethod
    def quat_angle_deg(q1, q2):
        """goblib.quat_angle_deg: degrees(2*acos(min(1,|q1.q2|)))."""
        a = [float(x) for x in q1]
        b = [float(x) for x in q2]
        return math.degrees(2.0 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a, b))))))

    @staticmethod
    def _rel(path):
        return p2d.rel(path)
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


def contract_default(key):
    """player contract default by rule (goblin CONTRACT_DEFAULTS generalised); None = use the UI default."""
    k = str(key).lower()
    if "arm_ik_fk" in k:
        return 0.0
    if "leg_ik_fk" in k:
        return 1.0
    if "_space" in k:
        return 0
    if k.startswith(FOOT_PROPS):
        return 0.0
    return None


DEF_REST_TOL = 1e-6          # P2.5c data rest matrix vs canonical (m / matrix element)
FOLLOW_TOL = 1e-5            # P2.5n parent-relative matrix max abs element (Skirt_*, sockets)
WEAPON_REL_POS = 1e-4        # P2.5n weapon control -> socket relative transform change (m)
WEAPON_REL_ROT = 0.01        # P2.5n (deg)
WEAPON_MOVE_MIN = 1.0        # P2.5n socket must move by >= this (deg) when the weapon control turns 20 deg
N_FOLLOW = 10                # P2.5n random poses
RESET_TOL = 1e-6             # P2.5l
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
        cd = contract_default(k)
        d = cd if cd is not None else (ud if ud is not None else v)
        ctx.prop_defaults[k] = d
        ctx.prop_default_info[k] = {"used": d, "ui_default": ud, "contract": cd}
    ctx.torso = next((n for n in ("CTRL_torso", "CTRL_cog", "CTRL_COG") if n in rig.pose.bones),
                     next((n for n in ctx.ctrl_set if "torso" in n.lower() or "cog" in n.lower()), None))


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
    return getattr(getattr(bpy.ops, OP_NS), name)


def op_registered(name):
    try:
        op_fn(name).get_rna_type()
        return True
    except Exception:
        return False


def op_params(ctx, name):
    if not ctx.ops.get(name):
        raise Blocked(f"operator {OP_NS}.{name} not registered")
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
    """Call bpy.ops.player.<name>(**kw); returns (result string, error or None)."""
    if not ctx.ops.get(name):
        raise Blocked(f"operator {OP_NS}.{name} not registered")
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
            spec = importlib.util.spec_from_file_location("player_rig_ui_addon_p2check", str(ADDON))
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
        raise Blocked("PL_mesh face attribute part_id missing")
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
    """World coordinates (n, 3) of the evaluated PL_mesh vertices idx."""
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
        raise Blocked("evaluated PL_mesh vertex count differs from the original")
    mw = np.array(oe.matrix_world, dtype=np.float64)
    return co[idx] @ mw[:3, :3].T + mw[:3, 3]


def stance_dz(ctx):
    """G6.7 stance (m): manifest clamps_reference.chosen_stance_mm (mm) or chosen_stance_m (m), else PLANT_TORSO_DZ
    (goblin -15 mm); the key and value used go to measured.ready_pose.stance_source."""
    cr = (ctx.man or {}).get("clamps_reference") or {}
    v = cr.get("chosen_stance_mm")
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v) / 1000.0, f"manifest clamps_reference.chosen_stance_mm = {v}"
    v = cr.get("chosen_stance_m")
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v), f"manifest clamps_reference.chosen_stance_m = {v}"
    return PLANT_TORSO_DZ, "goblin default -15 mm (no chosen_stance_mm / chosen_stance_m)"


def torso_down_channel(ctx):
    """torso control loc channel whose world direction is +Z -> (axis letter, value for the stance dz)."""
    b = ctx.rig.data.bones.get(ctx.torso or "")
    if b is None:
        raise Blocked("torso control (CTRL_torso / *torso* / *cog*) missing")
    m = ctx.rig.matrix_world.to_3x3() @ b.matrix_local.to_3x3()
    dots = [m.col[i].normalized().z for i in range(3)]
    i = max(range(3), key=lambda k: abs(dots[k]))
    dz, _src = stance_dz(ctx)
    return "XYZ"[i], dz / (dots[i] * m.col[i].length), dots[i]


def ready_pose(ctx, leg_prop, torso_ch):
    reset_all(ctx)
    if leg_prop is not None:
        set_prop(ctx, leg_prop, ctx.prop_defaults.get(leg_prop, 1.0))
    set_channels(ctx.pbs[ctx.torso], [("loc", torso_ch[0], torso_ch[1])], 1.0)
    evaluate(ctx)


def c_foot(ctx):
    if ctx.mesh is None:
        raise Blocked(f"{MESH} missing")
    clamps = ctx.section("clamps")
    ikfk = ctx.section("ikfk")
    torso_ch = torso_down_channel(ctx)
    mw = ctx.rig.matrix_world
    meas, ok = {"ready_pose": {"torso_control": ctx.torso, "loc_axis": torso_ch[0], "value_m": rnd(torso_ch[1], 5),
                               "axis_world_z_dot": rnd(torso_ch[2], 5), "stance_dz_m": stance_dz(ctx)[0],
                               "stance_source": stance_dz(ctx)[1]}}, True
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
    """G6.8 adapted: per side, hand_ik_space_<x> has no value / UI slot / driven-constraint target referring to the
    weapon of side <x> (a same-side weapon space would be a cycle: the weapon follows that hand); cycles 0."""
    spaces = (ctx.man or {}).get("spaces") or {}
    per, no_weapon_all = {}, True
    for x in SIDES:
        key = f"hand_ik_space_{x}"
        sp = spaces.get(key)
        m_vals = list((sp or {}).get("values") or [])

        ctrls = (ctx.man or {}).get("controls") or {}

        def side_of(name):
            """manifest side of a control (L / R / C), else its _l / _r suffix, else None."""
            sd = str((ctrls.get(name) or {}).get("side", "")).upper()
            if sd in ("L", "R", "C"):
                return sd
            nm = str(name).lower()
            return "L" if nm.endswith("_l") else "R" if nm.endswith("_r") else None

        def same_side(v):
            """True when a weapon value / target belongs to side x (value resolved through spaces
            value_constraint -> targets -> control side); None when a weapon entry cannot be resolved."""
            v = str(v)
            if "weapon" not in v.lower():
                return False
            tgt = ((sp or {}).get("targets") or {}).get(((sp or {}).get("value_constraint") or {}).get(v), v)
            sd = side_of(tgt)
            return None if sd is None else sd == x.upper()
        man_weapon = [v for v in m_vals if same_side(v)]
        ambiguous = [v for v in m_vals if same_side(v) is None]
        h = props_holder(ctx, key)
        ui = ui_range(h, key) if h is not None else None
        pairs, _exp, _src = space_constraints(ctx, key, sp or {})
        r_targets = {}
        for b, c in pairs:
            pb = ctx.pbs.get(b)
            con = pb.constraints.get(c) if pb is not None else None
            if con is not None:
                r_targets[f"{b}/{c}"] = [f"{t.name}:{s_}" for t, s_ in _con_targets(con)]
        mch = (sp or {}).get("mch")
        if mch and mch in ctx.pbs:
            for con in ctx.pbs[mch].constraints:
                r_targets.setdefault(f"{mch}/{con.name}", [f"{t.name}:{s_}" for t, s_ in _con_targets(con)])
        weapon_targets = [k for k, v in r_targets.items() if any(same_side(t.split(":")[-1]) is True for t in v)]
        nw = sp is not None and h is not None and not man_weapon and not weapon_targets
        no_weapon_all = no_weapon_all and nw
        per[key] = {"manifest_values": m_vals if sp is not None else "missing", "same_side_weapon_values": man_weapon,
                    "unresolved_weapon_values (report)": ambiguous,
                    "props_ui": ui, "constraint_targets": r_targets, "same_side_weapon_targets": weapon_targets,
                    "ok": nw}
    edges, muted = dep_edges(ctx)
    cyc = sccs(edges)
    cap = ctx.cycle_capture or {}
    cap_ok = (not cap.get("capture_works")) or (cap.get("dependency_cycle_lines", 0) == 0
                                                 and cap.get("detected_count", 0) == 0)
    ok = no_weapon_all and not cyc and cap_ok
    return {"hand_ik_spaces": per, "blender_capture": cap, "graph_nodes": len(edges),
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
        raise Blocked(f"operator {OP_NS}.ready_pose not registered (missing)")
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
        cok = (not err and feet_ok and sdev <= SCALE_TOL and nan == 0)
        cm["checks"] = {"knee_in_goblin_range (calibration)": knee_ok, "elbow_in_goblin_range (calibration)": elbow_ok,
                        "feet": feet_ok, "scale": sdev <= SCALE_TOL and nan == 0, "op": not err}
        cm["ok"] = cok
        ok = ok and cok
        meas[case] = cm
    reset_all(ctx)
    evaluate(ctx)
    return meas, ok


# ---------------------------------------------------------------- P2.5c DEF contract (player, new)
def fcurves_of(action):
    """F-curves of a layered (Blender 4.4+) or legacy action (check_p1.fcurves_of)."""
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", ()):
                out.extend(cb.fcurves)
    if not out and hasattr(action, "fcurves"):
        out.extend(action.fcurves)
    return out


_BONE_PATH = re.compile(r'pose\.bones\["(.+?)"\]')


def def_channels(ctx, names, constraint_paths=False):
    """F-curves / drivers of the rig whose data path addresses a pose bone in `names`.  Paths into the bone's
    constraints (pose.bones["X"].constraints[...], e.g. the FK/IK influence drivers = the constraint chain itself)
    are returned only with constraint_paths=True."""
    out = []
    ad = ctx.rig.animation_data
    for kind, fcs in (("driver", list(ad.drivers) if ad is not None else []),
                      ("action", fcurves_of((ctx.saved or {}).get("action")) if (ctx.saved or {}).get("action")
                       else [])):
        for fc in fcs:
            dp = fc.data_path or ""
            m = _BONE_PATH.match(dp)
            if not m or m.group(1) not in names:
                continue
            is_con = dp[m.end():].startswith(".constraints[")
            if is_con == constraint_paths:
                out.append(f"{kind}: {dp}[{fc.array_index}]")
    return out


def c_def_contract(ctx):
    arm = ctx.rig.data
    mw = ctx.rig.matrix_world
    canon = {b["name"]: b for b in ctx.canon["bones"]}
    missing, over, par_bad, dfm_bad = [], {}, {}, {}
    worst, worst_n = 0.0, None
    for n, b in canon.items():
        bb = arm.bones.get(n)
        if bb is None:
            missing.append(n)
            continue
        d = mat_max_abs(mw @ bb.matrix_local, Matrix(b["rest_matrix"]))
        if d > worst:
            worst, worst_n = d, n
        if d > DEF_REST_TOL:
            over[n] = float(f"{d:.3e}")
        p = bb.parent.name if bb.parent else None
        if p != (b.get("parent") or None):
            par_bad[n] = {"canonical": b.get("parent"), "rig": p}
        want = b.get("deform", b.get("use_deform"))
        if want is not None and bool(want) != bool(bb.use_deform):
            dfm_bad[n] = {"canonical": bool(want), "rig": bool(bb.use_deform)}
    extra_deform = sorted(b.name for b in arm.bones if b.use_deform and b.name not in canon)
    keyed = def_channels(ctx, set(canon))
    cons = {}
    for n in canon:
        pb = ctx.pbs.get(n)
        if pb is not None and len(pb.constraints):
            cons[n] = [f"{c.type}:{c.name}->" + ",".join(f"{t.name}:{s_}" for t, s_ in _con_targets(c))
                       for c in pb.constraints]
    no_con = sorted(n for n in canon if n in ctx.pbs and not len(ctx.pbs[n].constraints))
    ok = not missing and not over and not par_bad and not dfm_bad and not extra_deform and not keyed
    return {"n_canonical": len(canon), "missing": missing, "rest_max_abs": float(f"{worst:.3e}"),
            "rest_worst_bone": worst_n, "rest_over_tol": over, "parent_mismatch": par_bad,
            "deform_mismatch": dfm_bad, "deform_bones_not_in_canonical": extra_deform,
            "def_fcurves_or_drivers": _cap(keyed, 30), "n_def_fcurves_or_drivers": len(keyed),
            "def_constraint_drivers (report, the constraint chain)": _cap(def_channels(ctx, set(canon), True), 30),
            "def_constraints (report)": cons, "def_bones_without_constraints (report)": no_con,
            "action_detached_in_memory": (ctx.saved or {}).get("action").name
            if (ctx.saved or {}).get("action") is not None else None}, ok


# ---------------------------------------------------------------- P2.5d PROPS contract (player, new)
def _num_eq(a, b, tol=1e-6):
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol


def c_props(ctx):
    man = ctx.man or {}
    need = {}
    for _k, ch in (man.get("ikfk") or {}).items():
        if ch.get("prop"):
            lo = min(float(ch.get("fk_value", 0)), float(ch.get("ik_value", 1)))
            hi = max(float(ch.get("fk_value", 0)), float(ch.get("ik_value", 1)))
            need[ch["prop"]] = ("ikfk", [lo, hi])
    for k, sp in (man.get("spaces") or {}).items():
        vals = list(sp.get("values") or [])
        need[sp.get("prop", k)] = ("space", [0, max(len(vals) - 1, 0)])
    for k, cl in (man.get("clamps") or {}).items():
        need[k] = ("clamp", [float(cl[0]), float(cl[1])])
    rows, ok = {}, bool(need)
    for prop, (kind, rng) in sorted(need.items()):
        h = props_holder(ctx, prop)
        if h is None:
            rows[prop] = "missing"
            ok = False
            continue
        v = h[prop]
        ui = ui_range(h, prop) or {}
        ud = ui_default(h, prop)
        cd = contract_default(prop)
        type_ok = (isinstance(v, int) and not isinstance(v, bool)) if kind == "space" else isinstance(v, float)
        range_ok = _num_eq(ui.get("min"), rng[0]) and _num_eq(ui.get("max"), rng[1])
        default_ok = cd is not None and _num_eq(ud, cd)
        rows[prop] = {"kind": kind, "type": type(v).__name__, "value_in_file": v, "ui": ui, "ui_default": ud,
                      "contract_default": cd, "expected_range": rng, "type_ok": type_ok, "range_ok": range_ok,
                      "default_ok": default_ok, "file_value_equals_default (report)": _num_eq(v, cd)}
        ok = ok and type_ok and range_ok and default_ok
    extra = sorted(k for k in ctx.prop_defaults if k not in need)
    return {"props": rows, "n_contract_props": len(need), "props_not_in_manifest (report)": extra,
            "range_rule": "ikfk [min, max](fk_value, ik_value); space int [0, len(values) - 1]; clamp = manifest "
                          "clamps [lo, hi]; compared with the PROPS id_properties_ui min / max"}, ok


# ---------------------------------------------------------------- P2.5l reset_rig (player, new)
def c_reset(ctx):
    if not ctx.ops.get("reset_rig"):
        raise Blocked(f"operator {OP_NS}.reset_rig not registered (missing)")
    controls = [c for c in (ctx.man.get("controls") or {}) if c in ctx.pbs]
    rng = random.Random(SEED + 11)
    runs, ok = [], True
    I4 = Matrix.Identity(4)
    for i in range(3):
        reset_all(ctx)
        apply_pose(ctx, random_pose(ctx, rng, controls))
        for k in ctx.prop_defaults:
            h = props_holder(ctx, k)
            ui = ui_range(h, k) or {}
            lo, hi = ui.get("min"), ui.get("max")
            if lo is None or hi is None:
                continue
            set_prop(ctx, k, rng.uniform(float(lo), float(hi)))
        evaluate(ctx)
        r, err = run_op(ctx, "reset_rig")
        evaluate(ctx)
        bad_ctrl = {n: rnd(mat_max_abs(ctx.pbs[n].matrix_basis, I4), 7) for n in ctx.ctrl_set
                    if mat_max_abs(ctx.pbs[n].matrix_basis, I4) > RESET_TOL}
        bad_props = {k: {"after": get_prop(ctx, k), "default": d} for k, d in ctx.prop_defaults.items()
                     if not _num_eq(get_prop(ctx, k), d, RESET_TOL)}
        dev = max((mat_max_abs(ctx.pbs[b["name"]].matrix, Matrix(b["rest_matrix"])) for b in ctx.canon["bones"]
                   if b["name"] in ctx.pbs), default=None)
        rok = err is None and not bad_ctrl and not bad_props
        ok = ok and rok
        runs.append({"result": r, "error": err, "controls_not_identity": bad_ctrl, "props_not_default": bad_props,
                     "def_vs_rest_max_abs (report)": rnd(dev, 7) if dev is not None else None, "ok": rok})
    reset_all(ctx)
    evaluate(ctx)
    return {"runs": runs, "n_controls": len(ctx.ctrl_set),
            "method": "3 x (random pose of every manifest control in its sweep range + every PROPS key random in its "
                      f"UI range, seed {SEED + 11}) -> {OP_NS}.reset_rig -> every control matrix_basis = identity "
                      f"(<= {RESET_TOL:g}) and every PROPS key = contract default"}, ok


# ---------------------------------------------------------------- P2.5n Skirt / sockets / weapon (player, new)
FOLLOW_SOCKETS = ("HeadEquipmentSocket", "BackSocket", "BackWeaponSocket")


def _rel_pose(ctx, child, parent):
    return ctx.pbs[parent].matrix.inverted() @ ctx.pbs[child].matrix


def _rel_rest(ctx, child, parent):
    b = ctx.rig.data.bones
    return b[parent].matrix_local.inverted() @ b[child].matrix_local


def weapon_controls(ctx):
    out = {}
    for c, spec in (ctx.man.get("controls") or {}).items():
        if "weapon" not in c.lower() and str(spec.get("part", "")).lower() != "weapon":
            continue
        side = str(spec.get("side", "")).upper()
        if side not in ("L", "R"):
            side = "L" if c.lower().endswith("_l") else "R" if c.lower().endswith("_r") else side
        if side in ("L", "R"):
            out.setdefault(side, c)
    return out


def c_follow(ctx):
    canon = {b["name"]: b for b in ctx.canon["bones"]}
    skirt = sorted(n for n in canon if n.startswith("Skirt_"))
    follow = {n: canon[n].get("parent") for n in skirt + [s for s in FOLLOW_SOCKETS if s in canon]}
    # T255: d-02 section 8 follow design supersedes "no constraints" for Skirt_*_01 (follow_copy <- MCH_skirt_hinge_*):
    # their constraints and the MCH hinge / probe bones are reported; constraints on Skirt_*_02 and skirt controls fail
    skirt_cons_all = {n: [f"{c.type}:{c.name}" for c in ctx.pbs[n].constraints] for n in skirt
                      if n in ctx.pbs and len(ctx.pbs[n].constraints)}
    skirt_cons = {n: v for n, v in skirt_cons_all.items() if not n.endswith("_01")}
    skirt_keyed = def_channels(ctx, set(skirt))
    skirt_named = sorted(c for c in list((ctx.man.get("controls") or {}).keys()) + [b.name for b in ctx.rig.data.bones]
                         if "skirt" in c.lower() and c not in canon)
    skirt_ctrls = [c for c in skirt_named if c in (ctx.man.get("controls") or {}) or c.startswith("CTRL_")]
    fprop = "skirt_follow"
    fh = props_holder(ctx, fprop)
    controls = [c for c in (ctx.man.get("controls") or {}) if c in ctx.pbs]
    missing = [n for n, p in follow.items() if n not in ctx.pbs or p not in ctx.pbs]

    def follow_pass(follow_value):
        """N_FOLLOW random poses (same seed each pass); skirt_follow forced to follow_value (None = PROPS default)."""
        rng = random.Random(SEED + 23)
        worst = {n: 0.0 for n in follow}
        for _i in range(N_FOLLOW):
            reset_all(ctx)
            apply_pose(ctx, random_pose(ctx, rng, controls))
            for key, ch in (ctx.man.get("ikfk") or {}).items():
                if ch.get("prop") and props_holder(ctx, ch["prop"]) is not None:
                    set_prop(ctx, ch["prop"], rng.choice([ch.get("fk_value", 0), ch.get("ik_value", 1)]))
            if follow_value is not None and fh is not None:
                set_prop(ctx, fprop, follow_value)
            evaluate(ctx)
            for n, p in follow.items():
                if n in missing:
                    continue
                worst[n] = max(worst[n], mat_max_abs(_rel_pose(ctx, n, p), _rel_rest(ctx, n, p)))
        return worst

    worst = follow_pass(0.0)                                   # judged (T255)
    worst_default = follow_pass(None)                          # info: skirt_follow at its PROPS default
    over = {n: float(f"{v:.3e}") for n, v in worst.items() if v > FOLLOW_TOL}
    # weapon controls drive WeaponSocket_L / R
    wc = weapon_controls(ctx)
    weap, w_ok = {}, True
    for S in ("L", "R"):
        sock, hand = f"WeaponSocket_{S}", f"Hand_{S}"
        c = wc.get(S)
        if c is None or sock not in ctx.pbs or hand not in ctx.pbs:
            weap[S] = f"blocked: weapon control {c!r}, socket present {sock in ctx.pbs}"
            w_ok = False
            continue
        reset_all(ctx)
        evaluate(ctx)
        d_def = mat_max_abs(_rel_pose(ctx, sock, hand), _rel_rest(ctx, sock, hand))
        mw = ctx.rig.matrix_world
        s0, c0 = mw @ ctx.pbs[sock].matrix, mw @ ctx.pbs[c].matrix
        set_channels(ctx.pbs[c], [("rot", "X", 20.0), ("rot", "Z", 10.0), ("loc", "Y", 0.02 / loc_unit(ctx))],
                     loc_unit(ctx))
        evaluate(ctx)
        s1, c1 = mw @ ctx.pbs[sock].matrix, mw @ ctx.pbs[c].matrix
        moved = mat_rot_deg(s0, s1)
        r0, r1 = c0.inverted() @ s0, c1.inverted() @ s1
        dp, dr = (r1.translation - r0.translation).length, mat_rot_deg(r0, r1)
        ok_s = d_def <= FOLLOW_TOL and moved >= WEAPON_MOVE_MIN and dp <= WEAPON_REL_POS and dr <= WEAPON_REL_ROT
        w_ok = w_ok and ok_s
        weap[S] = {"control": c, "socket": sock, "default_rel_to_hand_max_abs": float(f"{d_def:.3e}"),
                   "socket_turn_deg": rnd(moved, 4), "ctrl_to_socket_rel_change_mm": mm(dp),
                   "ctrl_to_socket_rel_change_deg": rnd(dr, 5), "ok": ok_s}
    reset_all(ctx)
    evaluate(ctx)
    ok = not missing and not over and not skirt_cons and not skirt_keyed and not skirt_ctrls and w_ok
    return {"followers": {n: {"parent": p, "max_abs_rel_dev": float(f"{worst[n]:.3e}")} for n, p in follow.items()},
            "skirt_follow_judged": {"prop": fprop, "found": fh is not None, "value": 0.0 if fh is not None else "absent",
                                    "fcurves": "rig action detached in memory by prepare(): no F-curve can override it"},
            "info_skirt_follow_default": {"value": ctx.prop_defaults.get(fprop),
                                          "max_abs_rel_dev": {n: float(f"{v:.3e}") for n, v in worst_default.items()
                                                              if n.startswith("Skirt_")}},
            "skirt_01_constraints (report, section 8 follow)": {n: v for n, v in skirt_cons_all.items() if n.endswith("_01")},
            "skirt_named_non_controls (report)": [c for c in skirt_named if c not in skirt_ctrls],
            "over_tol": over, "missing": missing, "skirt_constraints": skirt_cons, "skirt_fcurves_or_drivers":
                skirt_keyed, "skirt_controls": skirt_ctrls, "weapon": weap, "n_random_poses": N_FOLLOW,
            "method": f"{N_FOLLOW} random poses (all manifest controls in their sweep ranges, ik/fk props random, "
                      f"seed {SEED + 23}); follower rel = parent pose matrix^-1 @ child pose matrix vs the same at "
                      "rest (canonical parent); weapon: default socket rel to Hand_x vs rest, then the weapon control "
                      "turned rot X 20 / Z 10 deg + loc Y 20 mm: socket turn and control->socket relative change"}, ok


# ---------------------------------------------------------------- P2.5o control overview sheet (player, new)
SIDE_RGBA = {"L": (0.15, 0.4, 1.0, 1.0), "R": (1.0, 0.15, 0.15, 1.0), "C": (1.0, 0.8, 0.05, 1.0)}
SHEET_PX = 640


def _shape_curve(ctx, c, side):
    pb = ctx.pbs[c]
    sh = pb.custom_shape
    if sh is None or sh.type != "MESH":
        return None, f"custom shape {'missing' if sh is None else sh.type}"
    me = sh.data
    src = ctx.pbs.get(pb.custom_shape_transform.name) if pb.custom_shape_transform is not None else pb
    M = ctx.rig.matrix_world @ src.matrix
    sc = Vector(pb.custom_shape_scale_xyz)
    if pb.use_custom_shape_bone_size:
        sc = sc * pb.bone.length
    off = (Matrix.Translation(pb.custom_shape_translation) @ pb.custom_shape_rotation_euler.to_matrix().to_4x4()
           @ Matrix.Diagonal((sc.x, sc.y, sc.z, 1.0)))
    W = M @ off
    cu = bpy.data.curves.new(f"_p25_{c}", "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = 0.003
    for e in me.edges:
        a, b = (W @ me.vertices[e.vertices[0]].co), (W @ me.vertices[e.vertices[1]].co)
        sp = cu.splines.new("POLY")
        sp.points.add(1)
        sp.points[0].co = (a.x, a.y, a.z, 1.0)
        sp.points[1].co = (b.x, b.y, b.z, 1.0)
    ob = bpy.data.objects.new(f"_p25_{c}", cu)
    ob.color = SIDE_RGBA.get(side, (0.3, 0.3, 0.3, 1.0))
    return ob, None


def _render(objs, view, out_png):
    if view == "front":
        d = Vector((0, 1, 0))
    else:
        az, el = math.radians(45.0), math.radians(15.0)
        d = -Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
    d.normalize()
    right = d.cross(Vector((0, 0, 1))).normalized()
    up = right.cross(d).normalized()
    center, half, dist = Vector((0.0, 0.0, 0.56)), 0.64, 3.0
    sc = bpy.data.scenes.new("_p25_render")
    cam_data = bpy.data.cameras.new("_p25_cam")
    cam = bpy.data.objects.new("_p25_cam", cam_data)
    try:
        for o in objs:
            sc.collection.objects.link(o)
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = 2 * half
        cam_data.clip_end = 10.0
        cam.matrix_world = Matrix.Translation(center - d * dist) @ Matrix((right, up, -d)).transposed().to_4x4()
        sc.collection.objects.link(cam)
        sc.camera = cam
        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = r.resolution_y = SHEET_PX
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "OBJECT"
        sc.display.shading.show_xray = True
        sc.display.shading.xray_alpha = 0.6
        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
        bpy.data.objects.remove(cam, do_unlink=True)
        bpy.data.cameras.remove(cam_data)
        bpy.data.scenes.remove(sc)
    return out_png


def _cell_state(ctx, state):
    reset_all(ctx)
    evaluate(ctx)
    note = "all controls identity, PROPS contract defaults"
    if state == "ready":
        if ctx.ops.get("reset_rig"):
            run_op(ctx, "reset_rig")
        if ctx.ops.get("ready_pose"):
            _r, err = run_op(ctx, "ready_pose")
            note = f"{OP_NS}.reset_rig + {OP_NS}.ready_pose" + (f" (error {err})" if err else "")
        else:
            note = f"{OP_NS}.ready_pose not registered: defaults shown"
        evaluate(ctx)
    return note


def c_sheet(ctx):
    import tempfile as _tf
    tmp = Path(_tf.mkdtemp(prefix="p25_sheet_"))
    cells, notes, skipped = [], {}, {}
    try:
        for state in ("default", "ready"):
            notes[state] = _cell_state(ctx, state)
            objs, extra = [], []
            if ctx.mesh is not None:
                dg = bpy.context.evaluated_depsgraph_get()
                me = bpy.data.meshes.new_from_object(ctx.mesh.evaluated_get(dg), depsgraph=dg)
                mo = bpy.data.objects.new("_p25_mesh", me)
                mo.matrix_world = ctx.mesh.matrix_world.copy()
                mo.color = (0.82, 0.82, 0.82, 1.0)
                objs.append(mo)
                extra.append(("mesh", me))
            for c, spec in (ctx.man.get("controls") or {}).items():
                if c not in ctx.pbs:
                    continue
                ob, why = _shape_curve(ctx, c, str(spec.get("side", "")).upper())
                if ob is None:
                    skipped[c] = why
                    continue
                objs.append(ob)
                extra.append(("curve", ob.data))
            row = []
            try:
                for view in ("front", "three_quarter"):
                    row.append(_render(objs, view, tmp / f"{state}_{view}.png"))
            finally:
                for o in objs:
                    bpy.data.objects.remove(o, do_unlink=True)
                for kind, dblock in extra:
                    (bpy.data.meshes if kind == "mesh" else bpy.data.curves).remove(dblock)
            cells.append(row)
        # compose (p2d.compose_sheet layout, larger cells)
        imgs = [[p2d.load_rgba(pth) for pth in row] for row in cells]
        g = 8
        H = len(imgs) * SHEET_PX + (len(imgs) + 1) * g
        W = 2 * SHEET_PX + 3 * g
        sheet = np.full((H, W, 4), 1.0, dtype=np.float32)
        sheet[:, :, :3] = 0.55
        for i, row in enumerate(imgs):
            for j, im in enumerate(row):
                y0, x0 = g + i * (SHEET_PX + g), g + j * (SHEET_PX + g)
                a = im[:SHEET_PX, :SHEET_PX, 3:4]
                sheet[y0:y0 + a.shape[0], x0:x0 + a.shape[1], :3] = im[:SHEET_PX, :SHEET_PX, :3] * a + (1.0 - a)
        out = Path(P["out"]).parent / SHEET_NAME
        out.parent.mkdir(parents=True, exist_ok=True)
        img = bpy.data.images.new("_p25_sheet", W, H, alpha=True)
        try:
            img.pixels.foreach_set(np.ascontiguousarray(sheet[::-1]).ravel())
            img.filepath_raw = str(out)
            img.file_format = "PNG"
            img.save()
        finally:
            bpy.data.images.remove(img)
        ctx.sheet = out
    finally:
        import shutil as _sh
        _sh.rmtree(tmp, ignore_errors=True)
        reset_all(ctx)
        evaluate(ctx)
    return {"sheet": rel(out), "rows": ["default", "ready"], "cols": ["front", "three_quarter"], "states": notes,
            "controls_drawn": len([c for c in (ctx.man.get("controls") or {}) if c in ctx.pbs]) - len(skipped),
            "controls_skipped": skipped,
            "draw": "mesh light grey (x-ray 0.6), control custom-shape edges as 3 mm tubes coloured by manifest side "
                    "(L blue, R red, C yellow), placed at world bone matrix @ custom-shape translation / rotation / "
                    "scale (x bone length when use_custom_shape_bone_size)"}, out.exists()


# ---------------------------------------------------------------- criterion table
def specs():
    """[(id, threshold, note, fn)] in evidence order + execution order."""
    table = [
        ("P2.5a", "(G6.1) all manifest controls exist; bone collections CTRL/MCH/DEF exist; only the CTRL subtree "
                  "visible (MCH, DEF hidden; no visible bone outside CTRL; controls in CTRL; no DEF/MCH bone in CTRL); "
                  "colour class L blue / R red / C yellow; custom shape set",
         "bone-collection effective visibility (is_visible_effectively), bone.hide; colour: " + COLOR_METHOD,
         c_structure),
        ("P2.5b", f"(G6.2) max abs(DEF pose matrix - canonical rest_matrix) <= {REST_TOL:g} with every control "
                  "identity and PROPS contract defaults",
         "41 canonical bones; canonical rest_matrix is world, PL_rig transform identity (P2.3b); drivers reported",
         c_rest),
        ("P2.5c", f"DEF skeleton unchanged: every canonical bone present, data rest (world) <= {DEF_REST_TOL:g} vs "
                  "rest_matrix, parent and deform flag equal, no deform bone outside canonical; 0 F-curves / drivers "
                  "on DEF pose channels (DEF moves only through its constraints)",
         "DEF constraints and DEF bones without constraints reported", c_def_contract),
        ("P2.5d", "PROPS contract: every ikfk / space / clamp prop of the manifest exists; ik_fk float, space int, "
                  "clamp float; UI min / max = expected range; UI default = contract default",
         "contract defaults by rule (module doc); extra PROPS keys reported", c_props),
        ("P2.5e", f"(G6.3) per chain and direction: DEF world diff before/after snap <= {SNAP_POS * 1000:g} mm and "
                  f"<= {SNAP_ROT:g} deg; no operator error",
         f"goblin G6.3 method: {N_RANDOM} random poses per ikfk chain (seed {SEED}); {OP_NS}.snap_ikfk", c_snap),
        ("P2.5f", f"(G6.4) per space value pair a->b: target world diff <= {SNAP_POS * 1000:g} mm and <= "
                  f"{SNAP_ROT:g} deg; prop == b; driven influences == expected; they differ between a and b; no "
                  "operator error", f"goblin G6.4 method; {OP_NS}.switch_space", c_space),
        ("P2.5g", f"(G6.5) 0 pops and no NaN over every control channel sweep (step x {JUMP_K:g} + {JUMP_C:g} deg "
                  f"candidate, {REFINE_LEVELS}-level bisection, pop if change(h/16) > {POP_RATIO:g} x change(h)); "
                  f"(b) 0 branch flips (theta sign change with |theta| > {FLIP_MIN_DEG:g} deg)",
         "goblin G6.5 method (a) + (b), manifest sweep ranges", c_sweep),
        ("P2.5h", f"(G6.6) DEF scale 1 +- {SCALE_TOL:g} over the P2.5e/g/i samples; IK use_stretch False; ik_stretch "
                  "0 on IK chain bones", "goblin G6.6", c_scale),
        ("P2.5i", f"(G6.7) planted at every value of the 8 foot props over the manifest clamps: reach <= "
                  f"{PLANT_REACH * 1000:g} mm, lowest shoe z in [{PLANT_Z[0] * 1000:g}, {PLANT_Z[1] * 1000:g}] mm, "
                  f"roll / bank cumulative slip <= {PLANT_SLIP * 1000:g} mm",
         "goblin G6.7 method; stance = manifest clamps_reference.chosen_stance_mm else -15 mm (measured.ready_pose)",
         c_foot),
        ("P2.5j", "(G6.8 per side) hand_ik_space_l / _r have no same-side weapon value / target; 0 cycles in the "
                  "static constraint graph; 0 Blender 'Dependency cycle' lines",
         "goblin G6.8 graph + C-level output capture", c_cycle),
        ("P2.5k", f"(G6.9) blend text {TEXT_UI} with use_module; after its register() {OP_NS}."
                  + f", {OP_NS}.".join(OPS) + " registered; addon file imports; same code",
         "goblin G6.9; the task names reset_rig / ready_pose, snap_ikfk / switch_space are the goblin set", c_ops),
        ("P2.5l", f"{OP_NS}.reset_rig restores every control to identity and every PROPS key to its contract "
                  f"default (<= {RESET_TOL:g}), no operator error", "3 random states (measured.method)", c_reset),
        ("P2.5m", f"(G6.11) {OP_NS}.ready_pose in the 4 arm FK/IK x leg FK/IK cases: both feet planted, DEF scale 1 "
                  f"+- {SCALE_TOL:g}, no operator error; knee [{READY_KNEE[0]:g}, {READY_KNEE[1]:g}] / elbow "
                  f"[{READY_ELBOW[0]:g}, {READY_ELBOW[1]:g}] deg reported (goblin ranges, calibration)",
         CALIB + " for the knee / elbow ranges (goblin numbers, first player run); feet / scale / op judged", c_ready),
        ("P2.5n", f"with PROPS skirt_follow = 0 (T255): Skirt_* follow their parents rigidly (parent-relative pose = rest <= "
                  f"{FOLLOW_TOL:g}); no constraints on Skirt_*_02 (Skirt_*_01 follow constraints reported), no F-curves / "
                  "drivers on skirt DEF channels, no skirt controls; skirt_follow default deviations reported; HeadEquipmentSocket / BackSocket / BackWeaponSocket follow their "
                  f"parents (<= {FOLLOW_TOL:g}); weapon control L / R drive WeaponSocket_L / R (socket at rest "
                  f"relative to Hand_x at defaults <= {FOLLOW_TOL:g}; turns >= {WEAPON_MOVE_MIN:g} deg with the control; "
                  f"control->socket relative change <= {WEAPON_REL_POS * 1000:g} mm / {WEAPON_REL_ROT:g} deg)",
         "measured.method", c_follow),
        ("P2.5o", "[U] control overview sheet written (rows default / ready pose, cols front / three_quarter)",
         "the user judges the sheet", c_sheet),
    ]
    order = ["P2.5a", "P2.5b", "P2.5c", "P2.5d", "P2.5e", "P2.5f", "P2.5g", "P2.5i", "P2.5h", "P2.5j", "P2.5k",
             "P2.5l", "P2.5m", "P2.5n", "P2.5o"]
    return table, order


# ---------------------------------------------------------------- main
ADDON = P["addon"]


class Evidence(p2d.Evidence):
    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {t[0]: k for k, t in enumerate(specs()[0])}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               **p2d.jsonable(self.extra), "inputs": self.inputs, "renders": self.renders,
               "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={json.dumps(c['measured'])[:300]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


def parse_args():
    global ADDON
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    keys = {"--blend": "blend", "--manifest": "manifest", "--canon": "canon", "--parts": "parts",
            "--addon": "addon", "--out": "out"}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <path>' for k in keys)}")
        P[keys[argv[i]]] = Path(argv[i + 1])
        i += 2
    ADDON = Path(P["addon"])


def main():
    parse_args()
    ev = Evidence()
    ev.add_input(__file__)
    ev.add_input(p2d.__file__)
    ctx = Ctx()
    ctx.sheet = None
    blend = Path(P["blend"])
    if not blend.exists():
        _STATE["missing"].append(rel(blend))
        ctx.miss["blend"] = f"missing input: {rel(blend)}"
    else:
        bpy.ops.wm.open_mainfile(filepath=str(blend), load_ui=False)
        ev.add_input(blend)
    if ADDON.exists():
        ev.add_input(ADDON)
    else:
        _STATE["missing"].append(rel(ADDON))
    for q in sorted(HERE.glob("p2*.py")):
        ev.add_input(q)
    ctx.man = _load_json(Path(P["manifest"]), "manifest", ctx)
    ctx.canon = _load_json(Path(P["canon"]), "canon", ctx)
    raw = _load_json(Path(P["parts"]), "parts", ctx)
    ctx.parts = raw["part_id"] if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict) else raw
    for k in ("manifest", "canon", "parts"):
        if Path(P[k]).exists():
            ev.add_input(P[k])
    table, order = specs()
    by_id = {t[0]: t for t in table}
    results = {}
    try:
        blocking = [ctx.miss[k] for k in ("blend", "manifest", "canon") if ctx.miss.get(k)]
        if not blocking:
            rig = bpy.data.objects.get(ARM)
            if rig is None:
                arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
                rig = arms[0] if len(arms) == 1 else None
            if rig is None or rig.type != "ARMATURE":
                blocking = [f"armature object {ARM} missing in {rel(blend)}"]
            else:
                ctx.rig = rig
                mo = bpy.data.objects.get(MESH)
                ctx.mesh = mo if mo is not None and mo.type == "MESH" else None
                ev.extra["objects"] = {"armature": rig.name, "mesh": ctx.mesh.name if ctx.mesh else None}
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
                    print(traceback.format_exc())
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
        if ctx.sheet is not None and Path(ctx.sheet).exists():
            ev.render(ctx.sheet)
    finally:
        ev.write()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(2)
    if _STATE["missing"] or _STATE["errors"]:
        if _STATE["missing"]:
            print("missing input: " + ", ".join(_STATE["missing"]))
        if _STATE["errors"]:
            print("criterion errors: " + ", ".join(_STATE["errors"]))
        sys.stdout.flush()
        sys.exit(2)
