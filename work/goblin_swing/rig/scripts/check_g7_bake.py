"""check_g7_bake - Gate G7 BAKE checker (design doc d-23 section 7, G7.1 .. G7.5; plan T30).

Inputs:
  rig/gob_r05_rigtest.blend         work rig GOB_rig (DEF 24 + CTRL/MCH/PROPS) + action `rigtest`
  rig/data/rigtest_manifest.json    {"segments":[{"name","start","end","covers":[...],"foot_lock":[...]}]}
  rig/data/canonical_skeleton.json  canonical rest skeleton (name, parent, rest_matrix row-major)
  rig/scripts/s07b_export_rig.py    production module, imported only:
                                    build_export(src_blend: Path, action_name: str | None,
                                                 out_blend: Path) -> Path

Procedure:
  A. work blend (read only): assign `rigtest` to GOB_rig in memory, frame_set every integer frame
     of the action frame range, cache the evaluated world matrices of the 24 DEF bones (+ GOB_club,
     + PROPS values) in memory.  G7.1 is measured here (manifest covers, action keys, last frame).
  B. stage blends: the source blend is reopened before each call, then
     build_export(BLEND, "rigtest", tmp/stage_rigtest.blend) and
     build_export(BLEND, None, tmp/stage_rest.blend).  tmp = rig/export/tmp_g7/, deleted at the end.
  C. each stage blend is opened: G7.2 structure (both stages), G7.3 rest equality, then the rigtest
     stage GOB_export (baked action, constraints removed) is evaluated at the same frames and
     compared with the cache (G7.4 / G7.5).
  G7.6 (2026-09-26 HR2, rig/data/hr2_contract.md): the work GOB_mesh evaluated shape-key value mouth_open is
     cached per frame in A; the rigtest stage goblin_mesh evaluated mouth_open value at the same frames must be
     equal (<= 1e-6); both stages keep the key (relative); stage key animation / drivers / modifiers reported.
  Nothing is saved: the input blend and data files are never written; stage blends are temporary.

Run:    bl.ps1 -Script check_g7_bake.py -Blend gob_r05_rigtest.blend
Output: rig/inspect/G7/check_g7_bake.json
Exit:   0 when the evidence was written; 2 on a checker script error (run_main) or when an input
        (blend, manifest, canonical json, s07b_export_rig.py) is missing (evidence still written with
        blocked criteria, then "missing input" is printed).  An exception inside the production
        build_export is recorded as "blocked: build_export ..." (exit 0).
Units: distance measured in m, reported in mm; rotation error = degrees(2 acos |q1.q2|).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import fnmatch  # noqa: E402
import importlib  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
import shutil  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

GATE = "G7"
CHECKER = "check_g7_bake"
BLEND = goblib.RIG / "gob_r05_rigtest.blend"
MANIFEST_JSON = goblib.DATA / "rigtest_manifest.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
S07B_NAME = "s07b_export_rig"
S07B = goblib.RIG / "scripts" / f"{S07B_NAME}.py"
TMP = goblib.EXPORT / "tmp_g7"
STAGE_RIGTEST = TMP / "stage_rigtest.blend"
STAGE_REST = TMP / "stage_rest.blend"

WORK_ARM = "GOB_rig"
WORK_CLUB = "GOB_club"
WORK_MESH = "GOB_mesh"
MOUTH_KEY = "mouth_open"     # G7.6 (hr2_contract.md)
MOUTH_TOL = 1e-6             # G7.6 stage vs work evaluated shape-key value
PROPS = "PROPS"
ACTION = "rigtest"
EXP_ARM = "GOB_export"
EXP_MESH = "goblin_mesh"
EXP_CLUB = "goblin_club"
SOCKET = "weapon_socket_r"

N_BONES = 24
REST_TOL = 1e-5          # G7.2 max abs element, matrix_local vs canonical rest_matrix
SAME_REST_TOL = 1e-6     # G7.3 max abs element, rigtest stage vs rest stage
POS_TOL = 0.001          # G7.4 / G7.5 (m)
ROT_TOL_DEG = 1.0        # G7.4 / G7.5
FOOT_LOCK_TOL = 0.001    # G7.5 foot lock (m)
LAST_REST_TOL = 1e-4     # G7.1 last frame DEF pose vs canonical (G6.2 tolerance)
SWITCH_WIN = 2           # G7.5 frames before/after a switch frame
LOCK_CONST_TOL = 1e-6    # G7.5 foot_lock manifest: max variation of CTRL_foot_ik / roll props
FOOT_IK_CTRL = "CTRL_foot_ik_{s}"
FOOT_LOCK_PROPS = ("foot_roll_{s}", "foot_bank_{s}", "heel_twist_{s}", "toe_twist_{s}")
LEG_IKFK_PROP = "leg_ik_fk_{s}"
LEG_IK_VALUE = 1.0       # ctrl_manifest ikfk leg_x ik_value

# G7.1 required covers: (label, alternatives as fnmatch patterns; any one match satisfies the label)
REQUIRED_COVERS = (
    ("root", ("root",)),
    ("cog", ("cog",)),
    ("pelvis", ("pelvis",)),
    ("arm_fk_l", ("arm_fk_l",)),
    ("arm_fk_r", ("arm_fk_r",)),
    ("ikfk_blend_l|ikfk_blend_r", ("ikfk_blend_l", "ikfk_blend_r")),
    ("snap_arm_*", ("snap_arm_*",)),
    ("head_space", ("head_space",)),
    ("hand_ik_space_l_weapon", ("hand_ik_space_l_weapon",)),
    ("weapon_world", ("weapon_world",)),
    ("weapon_return", ("weapon_return",)),
    ("foot_roll_l", ("foot_roll_l",)),
    ("foot_roll_r", ("foot_roll_r",)),
    ("leg_fk_*", ("leg_fk_*",)),
    ("rest", ("rest",)),
)
TWIST = ("upperarm_twist_l", "lowerarm_twist_l", "upperarm_twist_r", "lowerarm_twist_r")
IK_BONES = tuple(f"{b}_{s}" for s in ("l", "r")
                 for b in ("upperarm", "lowerarm", "hand", "thigh", "calf", "foot"))
# space prop -> DEF bone that the switched control drives (reported next to the all-bone max)
SPACE_TARGET = {"head_space": "head", "hand_ik_space_l": "hand_l", "hand_ik_space_r": "hand_r",
                "weapon_space": SOCKET, "knee_pole_space_l": "calf_l", "knee_pole_space_r": "calf_r"}
IKFK_RE = re.compile(r"_ik_fk_[lr]$")
FORBIDDEN_RE = re.compile(r"^(CTRL|MCH|WGT)_|^PROPS$")
BONE_PATH_RE = re.compile(r'^pose\.bones\["((?:[^"\\]|\\.)*)"\]')

_STATE = {"missing": []}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    """A criterion cannot be measured (missing object / action / stage)."""


def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=5):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def sci(x):
    x = float(x)
    return float(f"{x:.3e}") if math.isfinite(x) else str(x)


def rel(p):
    return goblib._rel(p)


def max_abs(m1, m2):
    return max(abs(m1[i][j] - m2[i][j]) for i in range(4) for j in range(4))


def tb_tail(n=6):
    lines = traceback.format_exc().strip().splitlines()
    return " | ".join(lines[-n:])


def open_blend(path):
    bpy.ops.wm.open_mainfile(filepath=str(path))


def fcurves_of(action):
    """All F-curves of a (layered, Blender 4.4+) action; legacy action.fcurves as fallback."""
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", ()):
                out.extend(cb.fcurves)
    if not out and hasattr(action, "fcurves"):
        out.extend(action.fcurves)
    return out


def assign_action(obj, act):
    """In memory only.  Returns the slot identifier (or None)."""
    ad = obj.animation_data or obj.animation_data_create()
    if ad.action != act:
        ad.action = act
    if hasattr(ad, "action_slot") and ad.action_slot is None:
        from bpy_extras import anim_utils
        slot = anim_utils.action_get_first_suitable_slot(act, "OBJECT")
        if slot is not None:
            ad.action_slot = slot
    slot = getattr(ad, "action_slot", None)
    return getattr(slot, "identifier", None) if slot is not None else None


def action_range(act):
    f0, f1 = act.frame_range
    return int(round(f0)), int(round(f1))


def load_canon(canon):
    bones = canon.get("bones") if isinstance(canon, dict) else None
    if not isinstance(bones, list) or not bones:
        raise Blocked("canonical_skeleton.json: no 'bones' list")
    names, parent, mat = [], {}, {}
    for b in bones:
        n = b.get("name")
        if n is None or "rest_matrix" not in b:
            raise Blocked(f"canonical_skeleton.json bone record lacks name/rest_matrix: {n!r}")
        names.append(n)
        parent[n] = b.get("parent") or None
        mat[n] = Matrix(b["rest_matrix"])
    return names, parent, mat


def decompose_np(M):
    loc, q, sc = M.decompose()
    return np.array(M, dtype=np.float64), (q.w, q.x, q.y, q.z), tuple(sc)


def prop_value(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


class Ctx:
    def __init__(self):
        self.manifest = self.canon = None
        self.names = self.parent = self.canon_mat = None
        self.frames = []
        self.W = None           # work cache {"M","q","s"} arrays (F,B,...)
        self.W_club = None      # (F,4,4) or None
        self.props = {}         # prop -> list of values per frame
        self.work_info = {}
        self.g71 = None
        self.s07b = None
        self.segments = []
        self.foot_ctrl = {}     # side -> {"mode", "loc" (F,3), "rot" (F,4), "scale" (F,3)} work rig
        self.lock_valid = set()  # (segment name, side) passing the foot_lock manifest check
        self.work_mouth = None   # G7.6 work GOB_mesh evaluated mouth_open per frame (None: no key)
        self.work_mouth_info = {}
        self.stage_mouth = None  # G7.6 rigtest stage goblin_mesh evaluated mouth_open per frame
        self.mouth_struct = {}   # G7.6 per stage: key structure report


def key_value(obj, dg, name=MOUTH_KEY):
    """Evaluated shape-key value `name` of mesh obj (None when the mesh has no such key)."""
    key = obj.data.shape_keys if obj is not None and obj.type == "MESH" else None
    if key is None or key.key_blocks.get(name) is None:
        return None
    try:
        return float(key.evaluated_get(dg).key_blocks[name].value)
    except Exception:
        return float(key.key_blocks[name].value)


def key_struct(obj):
    """G7.6 report: shape keys, relative flags, Key animation (action / drivers) and modifiers of a mesh."""
    if obj is None or obj.type != "MESH":
        return {"mesh": "missing"}
    key = obj.data.shape_keys
    out = {"mesh": obj.name, "modifiers": [m.type for m in obj.modifiers]}
    if key is None:
        out["shape_keys"] = None
        return out
    ad = key.animation_data
    act = ad.action if ad is not None else None
    kb = key.key_blocks.get(MOUTH_KEY)
    out.update({"shape_keys": [k.name for k in key.key_blocks], "use_relative": bool(key.use_relative),
                "mouth_relative_key": (kb.relative_key.name if kb is not None and kb.relative_key else None),
                "key_action": act.name if act is not None else None,
                "key_action_fcurves": [fc.data_path for fc in fcurves_of(act)] if act is not None else [],
                "key_drivers": [fc.data_path for fc in ad.drivers] if ad is not None else []})
    return out


# ---------------------------------------------------------------- A: work blend
def work_phase(ctx):
    rig = bpy.data.objects.get(WORK_ARM)
    if rig is None or rig.type != "ARMATURE":
        raise Blocked(f"armature {WORK_ARM} missing in {rel(BLEND)}")
    act = bpy.data.actions.get(ACTION)
    if act is None:
        raise Blocked(f"action {ACTION} missing in {rel(BLEND)}")
    pre = rig.animation_data.action.name if rig.animation_data and rig.animation_data.action else None
    slot = assign_action(rig, act)
    missing_b = [n for n in ctx.names if n not in rig.pose.bones]
    if missing_b:
        raise Blocked(f"DEF bones missing in {WORK_ARM}: {missing_b}")
    f0, f1 = action_range(act)
    ctx.frames = list(range(f0, f1 + 1))
    sc = bpy.context.scene
    club = bpy.data.objects.get(WORK_CLUB)
    wmesh = bpy.data.objects.get(WORK_MESH)
    ctx.work_mouth_info = key_struct(wmesh)
    F, B = len(ctx.frames), len(ctx.names)
    W = {"M": np.zeros((F, B, 4, 4)), "q": np.zeros((F, B, 4)), "s": np.zeros((F, B, 3))}
    W_club = np.zeros((F, 4, 4)) if club is not None else None
    mouth = []
    props = {}
    last = {}
    for i, f in enumerate(ctx.frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = rig.evaluated_get(dg)
        mw = oe.matrix_world
        for j, n in enumerate(ctx.names):
            W["M"][i, j], W["q"][i, j], W["s"][i, j] = decompose_np(mw @ oe.pose.bones[n].matrix)
        if club is not None:
            W_club[i] = np.array(club.evaluated_get(dg).matrix_world, dtype=np.float64)
        mouth.append(key_value(wmesh, dg))
        ppb = oe.pose.bones.get(PROPS)
        if ppb is not None:
            for k in ppb.keys():
                v = prop_value(ppb[k])
                if v is not None:
                    props.setdefault(k, [None] * F)[i] = v
        for side in ("l", "r"):
            fpb = oe.pose.bones.get(FOOT_IK_CTRL.format(s=side))
            if fpb is None:
                continue
            if fpb.rotation_mode == "QUATERNION":
                rot = tuple(fpb.rotation_quaternion)
            elif fpb.rotation_mode == "AXIS_ANGLE":
                rot = tuple(fpb.rotation_axis_angle)
            else:
                rot = tuple(fpb.rotation_euler) + (0.0,)
            ch = ctx.foot_ctrl.setdefault(side, {"mode": fpb.rotation_mode, "loc": np.zeros((F, 3)),
                                                  "rot": np.zeros((F, 4)), "scale": np.zeros((F, 3))})
            ch["loc"][i], ch["rot"][i], ch["scale"][i] = tuple(fpb.location), rot, tuple(fpb.scale)
        if f == f1:
            # last frame: DEF armature-space pose vs canonical rest, CTRL basis vs identity
            dev = {n: max_abs(oe.pose.bones[n].matrix, ctx.canon_mat[n]) for n in ctx.names}
            wn = max(dev, key=dev.get)
            ctrl = {}
            for pb in oe.pose.bones:
                if pb.name.startswith("CTRL_"):
                    d = max_abs(pb.matrix_basis, Matrix.Identity(4))
                    if d > 1e-6:
                        ctrl[pb.name] = sci(d)
            pdef = {}
            opb = rig.pose.bones.get(PROPS)
            if opb is not None and ppb is not None:
                for k in ppb.keys():
                    try:
                        dflt = opb.id_properties_ui(k).as_dict().get("default")
                    except Exception:
                        continue
                    v = prop_value(ppb[k])
                    if v is not None and prop_value(dflt) is not None and abs(v - float(dflt)) > 1e-6:
                        pdef[k] = {"value": rnd(v, 4), "default": rnd(float(dflt), 4)}
            last = {"def_vs_canon_max_abs": sci(dev[wn]), "worst_bone": wn,
                    "ctrl_basis_non_identity": ctrl, "props_non_default": pdef}
    ctx.W, ctx.W_club, ctx.props = W, W_club, props
    ctx.work_mouth = None if any(m is None for m in mouth) else np.array(mouth, dtype=np.float64)
    ad = rig.animation_data
    ctx.work_info = {"action_before_assign": pre, "slot": slot, "frame_range": [f0, f1],
                     "n_frames": F, "fps": rnd(sc.render.fps / (sc.render.fps_base or 1.0), 4),
                     "nla_tracks": len(ad.nla_tracks) if ad else 0,
                     "rig_matrix_world_dev_from_identity": sci(max_abs(rig.matrix_world, Matrix.Identity(4))),
                     "work_club": WORK_CLUB if club is not None else None, "last": last}
    ctx.g71_keys = classify_keys(act, set(ctx.names))


def classify_keys(act, def_names):
    bones, props, bad = set(), set(), {"def": set(), "mch": set(), "other_bone": set(), "non_pose": set()}
    fcs = fcurves_of(act)
    for fc in fcs:
        dp = fc.data_path
        m = BONE_PATH_RE.match(dp)
        if not m:
            bad["non_pose"].add(dp)
            continue
        b = m.group(1)
        if b.startswith("CTRL_"):
            bones.add(b)
        elif b == PROPS:
            pm = re.match(r'^pose\.bones\["PROPS"\]\["([^"]+)"\]$', dp)
            props.add(pm.group(1) if pm else dp)
        elif b in def_names:
            bad["def"].add(b)
        elif b.startswith("MCH_"):
            bad["mch"].add(b)
        else:
            bad["other_bone"].add(dp)
    return {"n_fcurves": len(fcs), "keyed_ctrl": sorted(bones), "keyed_props": sorted(props),
            "bad": {k: sorted(v) for k, v in bad.items()}}


def parse_segments(ctx):
    segs = ctx.manifest.get("segments") if isinstance(ctx.manifest, dict) else None
    issues, out = [], []
    if not isinstance(segs, list) or not segs:
        return [], ["manifest has no 'segments' list"]
    f0, f1 = ctx.frames[0], ctx.frames[-1]
    for k, s in enumerate(segs):
        if not isinstance(s, dict):
            issues.append(f"segment #{k} not an object")
            continue
        need = [x for x in ("name", "start", "end", "covers") if x not in s]
        if need:
            issues.append(f"segment #{k} {s.get('name')!r} lacks {need}")
            continue
        try:
            a, b = int(s["start"]), int(s["end"])
        except (TypeError, ValueError):
            issues.append(f"segment {s['name']}: start/end not int")
            continue
        if a > b:
            issues.append(f"segment {s['name']}: start {a} > end {b}")
        if a < f0 or b > f1:
            issues.append(f"segment {s['name']}: [{a},{b}] outside action range [{f0},{f1}]")
        fl = s.get("foot_lock") or []
        out.append({"name": s["name"], "start": a, "end": b, "covers": list(s.get("covers") or []),
                    "foot_lock": [str(x) for x in fl]})
    return out, issues


# ---------------------------------------------------------------- G7.1
def c_g71(ctx):
    segs, issues = parse_segments(ctx)
    ctx.segments = segs
    found = sorted({c for s in segs for c in s["covers"]})
    req, miss = {}, []
    for label, pats in REQUIRED_COVERS:
        hit = sorted({c for c in found for p in pats if fnmatch.fnmatchcase(c, p)})
        req[label] = hit
        if not hit:
            miss.append(label)
    keys = ctx.g71_keys
    n_bad = sum(len(v) for v in keys["bad"].values())
    last = ctx.work_info["last"]
    f1 = ctx.frames[-1]
    last_ok = bool(last) and float(last["def_vs_canon_max_abs"]) <= LAST_REST_TOL
    man_end = max((s["end"] for s in segs), default=None)
    ok = not miss and not issues and n_bad == 0 and keys["n_fcurves"] > 0 and last_ok
    return {"action": ACTION, "work": {k: v for k, v in ctx.work_info.items() if k != "last"},
            "n_segments": len(segs),
            "segments": [[s["name"], s["start"], s["end"], s["covers"], s["foot_lock"]] for s in segs],
            "segment_issues": issues, "covers_found": found, "required_matches": req,
            "missing_required": miss, "n_fcurves": keys["n_fcurves"], "keyed_ctrl": keys["keyed_ctrl"],
            "keyed_props": keys["keyed_props"], "bad_keys": keys["bad"], "n_bad_keys": n_bad,
            "last_frame": f1, "manifest_max_end": man_end, "last_frame_rest": last}, ok


# ---------------------------------------------------------------- B: stage build
def build_stage(ctx, action_name, out):
    """Reopen the pristine source blend, call build_export, return (path, error)."""
    open_blend(BLEND)
    try:
        res = ctx.s07b.build_export(BLEND, action_name, out)
    except Exception:
        return None, f"build_export({action_name!r}) raised: {tb_tail()}"
    p = Path(res) if res else Path(out)
    if not p.exists():
        return None, f"build_export({action_name!r}) returned {res!r}; file not found"
    return p, None


# ---------------------------------------------------------------- C: stage structure
def stage_struct(ctx):
    """G7.2 + stage object checks on the currently open stage blend."""
    exp = bpy.data.objects.get(EXP_ARM)
    if exp is None or exp.type != "ARMATURE":
        raise Blocked(f"{EXP_ARM} armature missing in stage (objects: {sorted(o.name for o in bpy.data.objects)})")
    bones = exp.data.bones
    names = [b.name for b in bones]
    only_c = sorted(set(ctx.names) - set(names))
    only_e = sorted(set(names) - set(ctx.names))
    par_mis = {}
    rest_dev, worst, worst_n = {}, 0.0, None
    for n in ctx.names:
        b = bones.get(n)
        if b is None:
            continue
        p = b.parent.name if b.parent else None
        if p != ctx.parent[n]:
            par_mis[n] = {"canon": ctx.parent[n], "export": p}
        d = max_abs(b.matrix_local, ctx.canon_mat[n])
        rest_dev[n] = d
        if d > worst:
            worst, worst_n = d, n
    connected = sorted(b.name for b in bones if b.use_connect)
    n_pcon = sum(len(pb.constraints) for pb in exp.pose.bones)
    n_ocon = len(exp.constraints)
    other_arm = sorted(o.name for o in bpy.data.objects if o.type == "ARMATURE" and o != exp)
    other_arm_data = sorted(a.name for a in bpy.data.armatures if a != exp.data and a.users > 0)
    forb_obj = sorted(o.name for o in bpy.data.objects
                      if FORBIDDEN_RE.search(o.name) or o.name == WORK_ARM)
    forb_bone = sorted(f"{a.name}:{b.name}" for a in bpy.data.armatures if a.users > 0
                       for b in a.bones if FORBIDDEN_RE.search(b.name))
    forb_coll = sorted(c.name for c in bpy.data.collections if c.name in ("CTRL", "MCH", "WGT"))
    bcoll = sorted(bc.name for bc in getattr(exp.data, "collections_all", ()))
    other_con = {o.name: len(o.constraints) for o in bpy.data.objects if o != exp and len(o.constraints)}
    ok = (len(names) == N_BONES and not only_c and not only_e and not par_mis
          and worst <= REST_TOL and len(rest_dev) == N_BONES and n_pcon == 0 and n_ocon == 0
          and not other_arm and not forb_obj and not forb_bone and not connected)
    g72 = {"n_bones": len(names), "only_canon": only_c, "only_export": only_e,
           "parent_mismatch": par_mis, "rest_max_abs_diff": sci(worst), "rest_worst_bone": worst_n,
           "rest_over_tol": {n: sci(d) for n, d in rest_dev.items() if d > REST_TOL},
           "connected_bones": connected,
           "pose_bone_constraints": n_pcon, "object_constraints": n_ocon,
           "other_armature_objects": other_arm, "other_armature_data_with_users": other_arm_data,
           "ctrl_mch_wgt_objects": forb_obj, "ctrl_mch_wgt_bones": forb_bone,
           "ctrl_mch_wgt_collections_report": forb_coll, "export_bone_collections_report": bcoll,
           "other_object_constraints_report": other_con,
           "export_matrix_world_dev_from_identity_report": sci(max_abs(exp.matrix_world, Matrix.Identity(4))),
           "use_deform_report": {b.name: bool(b.use_deform) for b in bones if not b.use_deform}}

    mesh = bpy.data.objects.get(EXP_MESH)
    club = bpy.data.objects.get(EXP_CLUB)
    st = {"objects": sorted(f"{o.name}({o.type})" for o in bpy.data.objects)}
    ok_s = True
    if mesh is None or mesh.type != "MESH":
        st["goblin_mesh"] = "missing"
        ok_s = False
    else:
        mods = [(m.name, m.type, getattr(getattr(m, "object", None), "name", None)) for m in mesh.modifiers]
        arm_mods = [m for m in mods if m[1] == "ARMATURE"]
        vg = [g.name for g in mesh.vertex_groups]
        st["goblin_mesh"] = {"modifiers": mods, "parent": mesh.parent.name if mesh.parent else None,
                             "vgroups_without_bone_report": sorted(set(vg) - set(names)),
                             "deform_bones_without_vgroup_report": sorted(
                                 b.name for b in bones if b.use_deform and b.name not in vg)}
        ok_s = ok_s and len(arm_mods) == 1 and arm_mods[0][2] == EXP_ARM
    if club is None:
        st["goblin_club"] = "missing"
        ok_s = False
    else:
        st["goblin_club"] = {"parent": club.parent.name if club.parent else None,
                             "parent_type": club.parent_type, "parent_bone": club.parent_bone}
        ok_s = ok_s and club.parent == exp and club.parent_type == "BONE" and club.parent_bone == SOCKET
    rest = {n: bones[n].matrix_local.copy() for n in ctx.names if n in bones}
    return exp, (g72, ok), (st, ok_s), rest, exp.matrix_world.copy()


def action_info(obj):
    ad = obj.animation_data
    act = ad.action if ad else None
    if act is None:
        return None, {"action": None}
    fcs = fcurves_of(act)
    f0, f1 = action_range(act)
    need = set(range(f0, f1 + 1))
    short = 0
    keyed_b = set()
    for fc in fcs:
        ks = {int(round(k.co.x)) for k in fc.keyframe_points}
        if not need <= ks:
            short += 1
        m = BONE_PATH_RE.match(fc.data_path)
        if m:
            keyed_b.add(m.group(1))
    return act, {"action": act.name, "slot": getattr(getattr(ad, "action_slot", None), "identifier", None),
                 "frame_range": [f0, f1], "n_fcurves": len(fcs),
                 "fcurves_missing_some_integer_frame": short, "keyed_bones": len(keyed_b),
                 "unkeyed_bones": sorted(set(b.name for b in obj.data.bones) - keyed_b)}


def eval_stage(ctx, exp):
    act, info = action_info(exp)
    if act is None:
        raise Blocked(f"{EXP_ARM} has no action in the rigtest stage")
    sc = bpy.context.scene
    club = bpy.data.objects.get(EXP_CLUB)
    smesh = bpy.data.objects.get(EXP_MESH)
    F, B = len(ctx.frames), len(ctx.names)
    E = {"M": np.zeros((F, B, 4, 4)), "q": np.zeros((F, B, 4)), "s": np.zeros((F, B, 3))}
    E_club = np.zeros((F, 4, 4)) if club is not None else None
    mouth = []
    for i, f in enumerate(ctx.frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = exp.evaluated_get(dg)
        mw = oe.matrix_world
        for j, n in enumerate(ctx.names):
            E["M"][i, j], E["q"][i, j], E["s"][i, j] = decompose_np(mw @ oe.pose.bones[n].matrix)
        if club is not None:
            E_club[i] = np.array(club.evaluated_get(dg).matrix_world, dtype=np.float64)
        mouth.append(key_value(smesh, dg))
    ctx.stage_mouth = None if any(m is None for m in mouth) else np.array(mouth, dtype=np.float64)
    return E, E_club, info


def c_g76(ctx):
    """G7.6: rigtest stage goblin_mesh evaluated mouth_open vs the work GOB_mesh evaluated value, every frame."""
    m = {"work": ctx.work_mouth_info, "stages": ctx.mouth_struct, "n_frames": len(ctx.frames)}
    W, S = ctx.work_mouth, ctx.stage_mouth
    if W is None or S is None:
        m["values"] = ("work GOB_mesh has no mouth_open key" if W is None else "") + \
                      ("; " if W is None and S is None else "") + \
                      ("rigtest stage goblin_mesh has no mouth_open key" if S is None else "")
        return m, False
    d = np.abs(S - W)
    i = int(d.argmax()) if len(d) else 0
    m.update({"max_abs_diff": sci(d.max()) if len(d) else None, "worst_frame": ctx.frames[i] if len(d) else None,
              "n_frames_over_tol": int((d > MOUTH_TOL).sum()),
              "work_min_max": [rnd(W.min(), 6), rnd(W.max(), 6)], "stage_min_max": [rnd(S.min(), 6), rnd(S.max(), 6)],
              "frames_work_nonzero": [f for f, v in zip(ctx.frames, W.tolist()) if v != 0.0][:60],
              "per_frame_[work,stage]_report": {f: [rnd(a, 6), rnd(b, 6)] for f, a, b in
                                                zip(ctx.frames, W.tolist(), S.tolist())}})
    st_ok = all(isinstance(s.get("shape_keys"), list) and MOUTH_KEY in s["shape_keys"] and s.get("use_relative")
                for s in ctx.mouth_struct.values()) and len(ctx.mouth_struct) == 2
    return m, bool(len(d) == len(ctx.frames) and d.max() <= MOUTH_TOL and st_ok)


def rest_stage_pose(exp):
    """Rest stage: pose at its first action frame (or scene frame) vs rest, report only."""
    ad = exp.animation_data
    act = ad.action if ad else None
    sc = bpy.context.scene
    f = action_range(act)[0] if act is not None else sc.frame_current
    sc.frame_set(f)
    dg = bpy.context.evaluated_depsgraph_get()
    oe = exp.evaluated_get(dg)
    d = {pb.name: max_abs(pb.matrix, exp.data.bones[pb.name].matrix_local) for pb in oe.pose.bones}
    wn = max(d, key=d.get) if d else None
    return {"frame": f, "pose_vs_rest_max_abs": sci(d[wn]) if wn else None, "worst_bone": wn}


# ---------------------------------------------------------------- compare
def diff(W, E):
    pos = np.linalg.norm(W["M"][..., :3, 3] - E["M"][..., :3, 3], axis=-1)
    dot = np.clip(np.abs((W["q"] * E["q"]).sum(-1)), 0.0, 1.0)
    rot = np.degrees(2.0 * np.arccos(dot))
    sc = np.abs(W["s"] - E["s"]).max(-1)
    sc1 = np.abs(E["s"] - 1.0).max(-1)
    return {"pos": pos, "rot": rot, "sc": sc, "sc1": sc1}


def mat_err(A, Bm):
    """(F,4,4) world matrices -> per-frame position (m) and rotation (deg) error."""
    pos = np.linalg.norm(A[:, :3, 3] - Bm[:, :3, 3], axis=-1)
    rot = np.zeros(len(A))
    for i in range(len(A)):
        qa = Matrix(A[i].tolist()).decompose()[1]
        qb = Matrix(Bm[i].tolist()).decompose()[1]
        rot[i] = goblib.quat_angle_deg(qa, qb)
    return pos, rot


def stats(ctx, D, fidx, bidx):
    if not fidx or not bidx:
        return None, True
    p = D["pos"][np.ix_(fidx, bidx)]
    r = D["rot"][np.ix_(fidx, bidx)]
    ip = np.unravel_index(int(p.argmax()), p.shape)
    ir = np.unravel_index(int(r.argmax()), r.shape)
    out = {"frames": [ctx.frames[fidx[0]], ctx.frames[fidx[-1]]], "n_frames": len(fidx),
           "pos_mm_max": mm(p.max()), "pos_worst": [ctx.names[bidx[ip[1]]], ctx.frames[fidx[ip[0]]]],
           "rot_deg_max": rnd(r.max(), 4), "rot_worst": [ctx.names[bidx[ir[1]]], ctx.frames[fidx[ir[0]]]]}
    return out, bool(p.max() <= POS_TOL and r.max() <= ROT_TOL_DEG)


def seg_fidx(ctx, s):
    f0 = ctx.frames[0]
    return [f - f0 for f in range(s["start"], s["end"] + 1) if ctx.frames[0] <= f <= ctx.frames[-1]]


def c_g74(ctx, D, info):
    F, B = D["pos"].shape
    ip = np.unravel_index(int(D["pos"].argmax()), D["pos"].shape)
    ir = np.unravel_index(int(D["rot"].argmax()), D["rot"].shape)
    isc = np.unravel_index(int(D["sc"].argmax()), D["sc"].shape)
    per_bone = {n: [mm(D["pos"][:, j].max()), rnd(D["rot"][:, j].max(), 4)] for j, n in enumerate(ctx.names)}
    over = int(((D["pos"] > POS_TOL) | (D["rot"] > ROT_TOL_DEG)).sum())
    ok = B == N_BONES and F > 0 and D["pos"].max() <= POS_TOL and D["rot"].max() <= ROT_TOL_DEG
    return {"n_frames": F, "frames": [ctx.frames[0], ctx.frames[-1]], "n_bones": B,
            "pos_mm_max": mm(D["pos"].max()), "pos_worst": [ctx.names[ip[1]], ctx.frames[ip[0]]],
            "rot_deg_max": rnd(D["rot"].max(), 4), "rot_worst": [ctx.names[ir[1]], ctx.frames[ir[0]]],
            "n_samples_over_tol": over,
            "scale_dev_work_vs_export_max": sci(D["sc"].max()),
            "scale_dev_worst": [ctx.names[isc[1]], ctx.frames[isc[0]]],
            "export_scale_dev_from_1_max": sci(D["sc1"].max()),
            "per_bone_max_[pos_mm,rot_deg]": per_bone, "export_action": info}, ok


def c_g75_segments(ctx, D, bones):
    bidx = [ctx.names.index(b) for b in bones if b in ctx.names]
    out, ok = {}, True
    for s in ctx.segments:
        st, o = stats(ctx, D, seg_fidx(ctx, s), bidx)
        out[s["name"]] = st if st is not None else "no frames in compared range"
        ok = ok and o and st is not None
    return {"bones": list(bones) if len(bones) < N_BONES else "all 24", "per_segment": out}, ok and bool(out)


def switch_events(ctx, pred, runs):
    """Frames where a PROPS value selected by pred changes from the previous frame."""
    ev = []
    for k, vals in sorted(ctx.props.items()):
        if not pred(k):
            continue
        ch = [i for i in range(1, len(vals))
              if vals[i] is not None and vals[i - 1] is not None and abs(vals[i] - vals[i - 1]) > 1e-6]
        if not ch:
            continue
        if runs:
            grp, cur = [], [ch[0], ch[0]]
            for i in ch[1:]:
                if i == cur[1] + 1:
                    cur[1] = i
                else:
                    grp.append(cur)
                    cur = [i, i]
            grp.append(cur)
        else:
            grp = [[i, i] for i in ch]
        for a, b in grp:
            ev.append((k, a, b, vals[a - 1], vals[b]))
    return ev


def c_g75_switch(ctx, D, pred, runs, target_map):
    evs = switch_events(ctx, pred, runs)
    allb = list(range(len(ctx.names)))
    out, ok = [], True
    for k, a, b, v0, v1 in evs:
        lo, hi = max(0, a - SWITCH_WIN), min(len(ctx.frames) - 1, b + SWITCH_WIN)
        fidx = list(range(lo, hi + 1))
        st, o = stats(ctx, D, fidx, allb)
        rec = {"prop": k, "change_frames": [ctx.frames[a], ctx.frames[b]], "from": rnd(v0, 4),
               "to": rnd(v1, 4), "window": [ctx.frames[lo], ctx.frames[hi]], "all_bones": st}
        tb = target_map.get(k) if target_map else None
        if tb in ctx.names:
            rec["target_bone"] = tb
            rec["target"] = stats(ctx, D, fidx, [ctx.names.index(tb)])[0]
        out.append(rec)
        ok = ok and o
    return {"n_events": len(out), "window_frames": f"+-{SWITCH_WIN}", "events": out,
            "props_seen": sorted(k for k in ctx.props if pred(k))}, ok and bool(out)


def c_g75_weapon(ctx, D, E_club):
    j = ctx.names.index(SOCKET)
    seg = {}
    ok = True
    for s in ctx.segments:
        st, o = stats(ctx, D, seg_fidx(ctx, s), [j])
        seg[s["name"]] = st
        ok = ok and o
    allst, o = stats(ctx, D, list(range(len(ctx.frames))), [j])
    ok = ok and o
    club = None
    if ctx.W_club is None or E_club is None:
        club = f"not compared ({WORK_CLUB} in work: {ctx.W_club is not None}, {EXP_CLUB} in stage: {E_club is not None})"
        ok = False
    else:
        cp, cr = mat_err(ctx.W_club, E_club)
        club = {"pos_mm_max": mm(cp.max()), "pos_worst_frame": ctx.frames[int(cp.argmax())],
                "rot_deg_max": rnd(cr.max(), 4), "rot_worst_frame": ctx.frames[int(cr.argmax())]}
        ok = ok and cp.max() <= POS_TOL and cr.max() <= ROT_TOL_DEG
    return {"socket_all_frames": allst, "socket_per_segment": seg,
            "club_object_world_work_vs_stage": club}, ok


def c_g75_foot_manifest(ctx):
    """foot_lock definition: CTRL_foot_ik_x loc/rot/scale and the 4 roll props constant over the whole
    segment (max |v(f) - v(start)| <= LOCK_CONST_TOL) and leg_ik_fk_x == IK at every frame."""
    out, ok = [], True
    ctx.lock_valid = set()
    for s in ctx.segments:
        fidx = seg_fidx(ctx, s)
        for side in s["foot_lock"]:
            rec = {"segment": s["name"], "side": side}
            errs = []
            if not fidx:
                errs.append("no frames in compared range")
            ch = ctx.foot_ctrl.get(side)
            var = {}
            if ch is None:
                errs.append(f"{FOOT_IK_CTRL.format(s=side)} missing in {WORK_ARM}")
            elif fidx:
                rec["rotation_mode"] = ch["mode"]
                for key in ("loc", "rot", "scale"):
                    a = ch[key][fidx]
                    var[f"ctrl_{key}"] = float(np.abs(a - a[0]).max())
            for pf in FOOT_LOCK_PROPS:
                pn = pf.format(s=side)
                vals = ctx.props.get(pn)
                if vals is None:
                    errs.append(f"PROPS[{pn}] missing")
                    continue
                v = [vals[i] for i in fidx]
                if any(x is None for x in v):
                    errs.append(f"PROPS[{pn}] not evaluated at some frame")
                    continue
                if v:
                    var[pn] = float(max(abs(x - v[0]) for x in v))
            ikn = LEG_IKFK_PROP.format(s=side)
            ikv = ctx.props.get(ikn)
            if ikv is None:
                errs.append(f"PROPS[{ikn}] missing")
            elif fidx:
                v = [ikv[i] for i in fidx]
                if any(x is None for x in v):
                    errs.append(f"PROPS[{ikn}] not evaluated at some frame")
                else:
                    dev = max(abs(x - LEG_IK_VALUE) for x in v)
                    rec[f"{ikn}_max_dev_from_ik"] = sci(dev)
                    if dev > LOCK_CONST_TOL:
                        errs.append(f"leg not IK ({ikn} max |v-{LEG_IK_VALUE:g}| = {dev:.3e})")
            over = {k: sci(d) for k, d in var.items() if d > LOCK_CONST_TOL}
            if over:
                errs.append(f"varies > {LOCK_CONST_TOL:g}: {sorted(over)}")
            rec["max_variation"] = {k: sci(d) for k, d in var.items()}
            rec["errors"] = errs
            rec["valid"] = not errs
            if rec["valid"]:
                ctx.lock_valid.add((s["name"], side))
            ok = ok and rec["valid"]
            out.append(rec)
    return {"n_locked": len(out), "n_valid": len(ctx.lock_valid), "per_segment_side": out}, ok and bool(out)


def c_g75_foot(ctx, E):
    out, ok = [], True
    n_judged = 0
    for s in ctx.segments:
        for side in s["foot_lock"]:
            bn = f"foot_{side}"
            fidx = seg_fidx(ctx, s)
            if bn not in ctx.names or not fidx:
                out.append({"segment": s["name"], "side": side, "measured": f"blocked: {bn} / frames"})
                ok = False
                continue
            j = ctx.names.index(bn)
            valid = (s["name"], side) in ctx.lock_valid
            rec = {"segment": s["name"], "side": side, "frames": [ctx.frames[fidx[0]], ctx.frames[fidx[-1]]],
                   "manifest_valid": valid}
            n_judged += int(valid)
            for tag, A in (("baked", E), ("work_ref", ctx.W)):
                M = A["M"][fidx, j]
                head = M[:, :3, 3]
                ln = ctx.bone_len.get(bn, 0.0)
                tail = head + M[:, :3, 1] * ln   # column 1 = bone Y (scaled; scale ~1)
                dh = np.linalg.norm(head - head[0], axis=-1)
                dt = np.linalg.norm(tail - tail[0], axis=-1)
                q = A["q"][fidx, j]
                ang = np.degrees(2.0 * np.arccos(np.clip(np.abs((q * q[0]).sum(-1)), 0.0, 1.0)))
                rec[tag] = {"head_move_mm_max": mm(dh.max()), "head_worst_frame": ctx.frames[fidx[int(dh.argmax())]],
                            "tail_move_mm_max": mm(dt.max()), "rot_change_deg_max": rnd(ang.max(), 4)}
                if tag == "baked" and valid:
                    ok = ok and dh.max() <= FOOT_LOCK_TOL
            rec["segment_covers"] = s["covers"]
            out.append(rec)
    return {"n_locked": len(out), "n_judged_manifest_valid": n_judged,
            "per_segment_side": out}, ok and n_judged > 0


# ---------------------------------------------------------------- main
IDS = (
    ("G7.1", "all required covers present (incl. pelvis = CTRL_pelvis alone; ikfk_blend either side; "
             "snap_arm_*/leg_fk_* any); segments "
             "well-formed inside the action range; action keys only CTRL_* / PROPS; last frame DEF pose "
             f"(armature space) vs canonical rest_matrix max abs <= {LAST_REST_TOL:g}",
     "manifest covers matched with fnmatch; F-curve data_path bone names from action layers/channelbags; "
     "work rig evaluated at the last action frame (DEF pb.matrix vs canonical rest_matrix); CTRL basis / "
     "PROPS defaults at the last frame reported"),
    ("G7.2", f"both stages: {N_BONES} bones, names+parents = canonical, matrix_local vs canonical "
             f"rest_matrix max abs <= {REST_TOL:g}, constraints 0, every bone use_connect False, no other "
             "armature, no CTRL/MCH/WGT/PROPS objects or bones",
     "GOB_export of stage_rigtest and stage_rest built by s07b.build_export into rig/export/tmp_g7 "
     "(temporary), bone data read after opening each stage blend; connected_bones = bones with "
     "use_connect True"),
    ("G7.2_stage", f"both stages: {EXP_MESH} has exactly one Armature modifier -> {EXP_ARM}; {EXP_CLUB} "
                   f"parent {EXP_ARM}, parent_type BONE, parent_bone {SOCKET}",
     "object data of each stage blend; vertex group / deform bone mismatches and source-blend sha256 "
     "before/after build are reported only"),
    ("G7.3", f"same bone set; per bone matrix_local max abs diff <= {SAME_REST_TOL:g} (rigtest stage vs "
             "rest stage)",
     "GOB_export.data.bones[*].matrix_local of both stages; GOB_export matrix_world difference and the "
     "rest-stage pose vs rest are reported"),
    ("G7.4", f"all frames x {N_BONES} bones: position <= {POS_TOL * 1000:g} mm, rotation <= {ROT_TOL_DEG:g} deg",
     "world = matrix_world @ pose_bone.matrix (evaluated) per integer frame of the work action range; work "
     "GOB_rig DEF cached from the source blend, rigtest stage GOB_export (baked action) evaluated at the same "
     "frames; position = |t1-t2|, rotation = 2 acos|q1.q2| of decompose(); scale deviation reported"),
    ("G7.5_segments", f"every segment max over 24 bones: pos <= {POS_TOL * 1000:g} mm, rot <= {ROT_TOL_DEG:g} deg",
     "per manifest segment [start,end] max of the G7.4 error arrays"),
    ("G7.5_twist", f"per segment max over {', '.join(TWIST)}: same tolerances",
     "subset of the G7.4 error arrays"),
    ("G7.5_ik", "per segment max over IK chain DEF bones (upperarm/lowerarm/hand/thigh/calf/foot l/r): "
                "same tolerances", "subset of the G7.4 error arrays"),
    ("G7.5_space_switch", f"every space switch: max over 24 bones in the +-{SWITCH_WIN} frame window within "
                          "tolerances; >= 1 switch found",
     "switch frame = frame where an evaluated PROPS property whose name contains 'space' differs from the "
     "previous frame (work rig); target DEF bone of the switched control reported separately"),
    ("G7.5_ikfk_switch", f"every ik_fk change run: max over 24 bones in the +-{SWITCH_WIN} frame window within "
                         "tolerances; >= 1 run found",
     "runs of consecutive frames where an evaluated PROPS *_ik_fk_l/r value changes (blend and snap)"),
    ("G7.5_weapon", f"{SOCKET} all frames and per segment within tolerances; {EXP_CLUB} world vs work "
                    f"{WORK_CLUB} world within tolerances",
     "weapon_socket_r subset of the G7.4 arrays; club object evaluated matrix_world compared per frame"),
    ("G7.5_foot_lock_manifest", f"each foot_lock segment/side: CTRL_foot_ik_x loc/rot/scale and PROPS "
                                f"foot_roll/foot_bank/heel_twist/toe_twist_x max variation <= {LOCK_CONST_TOL:g}; "
                                f"leg_ik_fk_x = IK ({LEG_IK_VALUE:g}) +- {LOCK_CONST_TOL:g} at every frame",
     "work rig evaluated per frame (pose bone channels of the active rotation_mode, evaluated PROPS values); "
     "variation = max |v(f) - v(segment start)| over the segment; failing entries are manifest errors"),
    ("G7.5_foot_lock", f"each manifest-valid foot_lock segment/side: baked foot_x head world movement <= "
                       f"{FOOT_LOCK_TOL * 1000:g} mm; >= 1 manifest-valid locked segment",
     "max |head(f) - head(segment start)| of baked GOB_export foot_x; entries failing G7.5_foot_lock_manifest "
     "are reported (manifest_valid false) but not judged; tail movement, rotation change and the same values "
     "of the work rig DEF reported"),
    ("G7.6", f"all frames: rigtest stage {EXP_MESH} evaluated shape-key value {MOUTH_KEY} = work {WORK_MESH} "
             f"evaluated value (|diff| <= {MOUTH_TOL:g}); both stages' {EXP_MESH} keep the key {MOUTH_KEY} "
             "(use_relative True)",
     "2026-09-26 HR2 (rig/data/hr2_contract.md). value = mesh.data.shape_keys.evaluated_get(depsgraph)"
     ".key_blocks value after scene.frame_set(f) (work: driven by PROPS; stage: keyed on the Key); stage Key "
     "action F-curves, drivers, modifiers and the rest-stage value reported only"),
)


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    note = {i: n for i, _t, n in IDS}
    done = set()

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, note[cid])
        done.add(cid)

    def block_rest(reason):
        for cid, _t, _n in IDS:
            if cid not in done:
                put(cid, f"blocked: {reason}", False)

    miss = []
    if not BLEND.exists():
        miss.append(rel(BLEND))
    else:
        loaded = bpy.data.filepath
        if not loaded or Path(loaded).resolve() != BLEND.resolve():
            print(f"[{GATE}/{CHECKER}] opening {rel(BLEND)} (loaded: {loaded!r})")
            open_blend(BLEND)
        ev.add_input(BLEND)
    ev.add_stage_inputs("s07")
    for p in (MANIFEST_JSON, CANON_JSON):
        if p.exists():
            ev.add_input(p)
        else:
            miss.append(rel(p))
    s07b_missing = not S07B.exists()
    _STATE["missing"] = miss + ([rel(S07B)] if s07b_missing else [])

    try:
        if miss:
            block_rest("missing input: " + ", ".join(miss))
            return
        with open(MANIFEST_JSON, "r", encoding="utf-8") as f:
            ctx.manifest = json.load(f)
        with open(CANON_JSON, "r", encoding="utf-8") as f:
            ctx.canon = json.load(f)
        try:
            ctx.names, ctx.parent, ctx.canon_mat = load_canon(ctx.canon)
            ctx.bone_len = {b["name"]: (Vector(b["tail"]) - Vector(b["head"])).length
                            for b in ctx.canon["bones"] if "head" in b and "tail" in b}
            work_phase(ctx)
        except Blocked as e:
            block_rest(str(e))
            return
        put("G7.1", *c_g71(ctx))

        if s07b_missing:
            block_rest(f"missing input: {rel(S07B)}")
            return
        try:
            ctx.s07b = importlib.import_module(S07B_NAME)
            if not callable(getattr(ctx.s07b, "build_export", None)):
                raise AttributeError(f"{S07B_NAME}.build_export not found")
        except Exception:
            block_rest(f"import {S07B_NAME} failed: {tb_tail()}")
            return

        src_sha = goblib.sha256(BLEND)
        if TMP.exists():
            shutil.rmtree(TMP, ignore_errors=True)
        TMP.mkdir(parents=True, exist_ok=True)
        p_rig, e_rig = build_stage(ctx, ACTION, STAGE_RIGTEST)
        p_rest, e_rest = build_stage(ctx, None, STAGE_REST)
        src_unchanged = goblib.sha256(BLEND) == src_sha
        if e_rig or e_rest:
            block_rest("; ".join(x for x in (e_rig, e_rest) if x))
            return

        # rest stage
        open_blend(p_rest)
        try:
            _exp_r, g72_rest, st_rest, rest_r, mw_r = stage_struct(ctx)
            rest_pose = rest_stage_pose(_exp_r)
            ctx.mouth_struct["rest"] = key_struct(bpy.data.objects.get(EXP_MESH))
            ctx.mouth_struct["rest"]["value_evaluated"] = key_value(bpy.data.objects.get(EXP_MESH),
                                                                    bpy.context.evaluated_depsgraph_get())
        except Blocked as e:
            block_rest(f"rest stage: {e}")
            return
        # rigtest stage
        open_blend(p_rig)
        try:
            exp, g72_rig, st_rig, rest_g, mw_g = stage_struct(ctx)
            ctx.mouth_struct["rigtest"] = key_struct(bpy.data.objects.get(EXP_MESH))
        except Blocked as e:
            block_rest(f"rigtest stage: {e}")
            return

        put("G7.2", {"rigtest": g72_rig[0], "rest": g72_rest[0]}, g72_rig[1] and g72_rest[1])
        put("G7.2_stage", {"rigtest": st_rig[0], "rest": st_rest[0],
                           "source_blend_sha256_unchanged_report": src_unchanged},
            st_rig[1] and st_rest[1])

        common = [n for n in ctx.names if n in rest_g and n in rest_r]
        dev = {n: max_abs(rest_g[n], rest_r[n]) for n in common}
        wn = max(dev, key=dev.get) if dev else None
        g73_ok = len(common) == N_BONES and set(rest_g) == set(rest_r) and dev and dev[wn] <= SAME_REST_TOL
        put("G7.3", {"n_compared": len(common), "max_abs_diff": sci(dev[wn]) if wn else None,
                     "worst_bone": wn, "over_tol": {n: sci(d) for n, d in dev.items() if d > SAME_REST_TOL},
                     "only_rigtest": sorted(set(rest_g) - set(rest_r)),
                     "only_rest": sorted(set(rest_r) - set(rest_g)),
                     "export_matrix_world_max_abs_diff_report": sci(max_abs(mw_g, mw_r)),
                     "rest_stage_pose_report": rest_pose}, bool(g73_ok))

        try:
            E, E_club, info = eval_stage(ctx, exp)
        except Blocked as e:
            block_rest(str(e))
            return
        D = diff(ctx.W, E)
        put("G7.4", *c_g74(ctx, D, info))
        put("G7.5_segments", *c_g75_segments(ctx, D, ctx.names))
        put("G7.5_twist", *c_g75_segments(ctx, D, TWIST))
        put("G7.5_ik", *c_g75_segments(ctx, D, IK_BONES))
        put("G7.5_space_switch", *c_g75_switch(ctx, D, lambda k: "space" in k, False, SPACE_TARGET))
        put("G7.5_ikfk_switch", *c_g75_switch(ctx, D, lambda k: bool(IKFK_RE.search(k)), True, None))
        put("G7.5_weapon", *c_g75_weapon(ctx, D, E_club))
        put("G7.5_foot_lock_manifest", *c_g75_foot_manifest(ctx))
        put("G7.5_foot_lock", *c_g75_foot(ctx, E))
        put("G7.6", *c_g76(ctx))
    finally:
        try:
            block_rest("checker aborted before this criterion")
        finally:
            ev.write()
            if TMP.exists():
                shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
