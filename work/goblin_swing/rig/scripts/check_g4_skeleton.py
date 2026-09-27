"""check_g4_skeleton - Gate G4 SKEL checker (design doc d-23 section 7, G4.1 .. G4.9).

Inputs (independent of the production script s04_skeleton.py; only blend data + data json):
  rig/gob_r02_skeleton.blend        GOB_rig (armature, 24 DEF bones of spec section 2), GOB_mesh
  rig/data/retopo_loops.json        ring vertex indices into GOB_mesh
  rig/data/pivots.json              measured pivots (co, axis)
  rig/data/canonical_skeleton.json  canonical rest skeleton (G4.7)

Reference points (plan Phase 4, main measurement 2026-09-25):
  ring center = mean of the GOB_mesh world coordinates of the ring's verts (retopo_loops.json).
  elbow center = elbow_x_1, wrist = wrist_x_1, knee center = knee_x_1, ankle = ankle_x_1.
  shoulder_root_x, shoulder_x (upperarm head) and hip_x come from pivots.json.
  tube axis = pivots elbow_x / knee_x `axis`; perp(v, a) = unit(v - (v.a) a).
All bone data is read in world space: GOB_rig.matrix_world @ bone.head_local / tail_local /
matrix_local.  Poses (G4.4) are applied in memory only and restored; the temporary armature of
G4.7 is deleted after the comparison.  The blend and data files are never saved or edited.
No renders (G4 has no visual items).

Run:    bl.ps1 -Script check_g4_skeleton.py -Blend gob_r02_skeleton.blend
Output: rig/inspect/G4/check_g4_skeleton.json
Exit:   0 when the evidence was written; 2 on a script error (run_main) or when an input file is
        missing (the evidence is still written, then "missing input" is printed).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.  Lengths in m, reported in mm.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import functools  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Quaternion, Vector  # noqa: E402

GATE = "G4"
CHECKER = "check_g4_skeleton"
BLEND = goblib.RIG / "gob_r02_skeleton.blend"
ARM = "GOB_rig"
MESH = "GOB_mesh"
LOOPS_JSON = goblib.DATA / "retopo_loops.json"
PIVOTS_JSON = goblib.DATA / "pivots.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"

SIDES = ("l", "r")

# Spec section 2 tree, fixed literal: name -> (parent, use_connect).
# use_connect is True exactly where spec section 2 (pivot -> DEF table) and plan T18 place the
# child head on the parent tail: pelvis->spine_01->spine_02->head, shoulder->upperarm->lowerarm
# ->hand, thigh->calf->foot.  Twist bones, weapon_socket_r, shoulder, thigh, pelvis: not connected.
TREE = {
    "root": (None, False),
    "pelvis": ("root", False),
    "spine_01": ("pelvis", True),
    "spine_02": ("spine_01", True),
    "head": ("spine_02", True),
    "shoulder_l": ("spine_02", False),
    "upperarm_l": ("shoulder_l", True),
    "upperarm_twist_l": ("upperarm_l", False),
    "lowerarm_l": ("upperarm_l", True),
    "lowerarm_twist_l": ("lowerarm_l", False),
    "hand_l": ("lowerarm_l", True),
    "shoulder_r": ("spine_02", False),
    "upperarm_r": ("shoulder_r", True),
    "upperarm_twist_r": ("upperarm_r", False),
    "lowerarm_r": ("upperarm_r", True),
    "lowerarm_twist_r": ("lowerarm_r", False),
    "hand_r": ("lowerarm_r", True),
    "weapon_socket_r": ("hand_r", False),
    "thigh_l": ("pelvis", False),
    "calf_l": ("thigh_l", True),
    "foot_l": ("calf_l", True),
    "thigh_r": ("pelvis", False),
    "calf_r": ("thigh_r", True),
    "foot_r": ("calf_r", True),
}
NON_DEFORM = ("root", "weapon_socket_r")

HEAD_TOL = 0.002            # G4.3, G4.6 head position (m)
AXIS_TOL_DEG = 1.0          # G4.4a, G4.6, G4.9d
POSE_DEG = 30.0             # G4.4b/c
TWIST_POS_TOL = 0.0001      # G4.5 positions / length (m)
TWIST_ROT_TOL_DEG = 0.1     # G4.5 axis + roll
REBUILD_TOL = 1e-5          # G4.7 max abs element of matrix_local
ROOT_TOL = 1e-6             # G4.8
BEND_ANG_DEG = (0.5, 3.0)   # G4.9a
ELBOW_BEND = 0.004          # +Y
KNEE_BEND = 0.0015          # -Y
BEND_OFF_TOL = 0.0005       # G4.9b

ARM_CHAIN = ("upperarm", "lowerarm", "hand")
LEG_CHAIN = ("thigh", "calf")

# G4.3: bone -> (reference kind, reference name, preferred bend (sign of Y, offset m, axis pivot))
HEAD_REFS = (
    ("shoulder", "pivot", "shoulder_root_{s}", None),
    ("upperarm", "pivot", "shoulder_{s}", None),
    ("lowerarm", "ring", "elbow_{s}_1", (+1.0, ELBOW_BEND, "elbow_{s}")),
    ("hand", "ring", "wrist_{s}_1", None),
    ("thigh", "pivot", "hip_{s}", None),
    ("calf", "ring", "knee_{s}_1", (-1.0, KNEE_BEND, "knee_{s}")),
    ("foot", "ring", "ankle_{s}_1", None),
)

# G4.9 chains: key -> (upper bone, lower bone, center ring, axis pivot, target offset, Y sign, chain)
BEND_CHAINS = {
    "arm": ("upperarm", "lowerarm", "elbow_{s}_1", "elbow_{s}", ELBOW_BEND, +1.0, ARM_CHAIN),
    "leg": ("thigh", "calf", "knee_{s}_1", "knee_{s}", KNEE_BEND, -1.0, LEG_CHAIN),
}

_STATE = {"missing": []}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    """A criterion cannot be measured (missing bone / ring / pivot / object)."""


def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def mmv(v):
    return [mm(c) for c in v]


def rnd(x, n=5):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def rel(p):
    return goblib._rel(p)


def unit(v):
    v = Vector(v)
    n = v.length
    if n < 1e-12:
        raise Blocked("zero-length vector")
    return v / n


def ang_deg(a, b):
    """Unsigned angle between two directions (deg), 0..180."""
    a, b = unit(a), unit(b)
    return math.degrees(math.acos(max(-1.0, min(1.0, a.dot(b)))))


def perp(v, axis):
    """Component of v perpendicular to unit axis (not normalized)."""
    a = unit(axis)
    v = Vector(v)
    return v - a * v.dot(a)


def max_abs(m1, m2):
    return max(abs(m1[i][j] - m2[i][j]) for i in range(4) for j in range(4))


class Ctx:
    def __init__(self):
        self.miss = {}
        self.err = []
        self.loops = self.pivots = self.canon = None
        self.rig = self.mesh = None
        self.mw = Matrix.Identity(4)
        self.mesh_co = None

    def m(self):
        return [v for v in self.miss.values() if v] + self.err

    # ---- bones (world space)
    def bone(self, name):
        b = self.rig.data.bones.get(name)
        if b is None:
            raise Blocked(f"bone {name} missing in {ARM}")
        return b

    def head(self, name):
        return self.mw @ self.bone(name).head_local

    def tail(self, name):
        return self.mw @ self.bone(name).tail_local

    def rot(self, name):
        """World rest rotation (3x3, orthonormalized) of bone `name`."""
        return (self.mw.to_3x3() @ self.bone(name).matrix_local.to_3x3()).normalized()

    def axis(self, name, i):
        return unit(self.rot(name).col[i])

    # ---- references
    def pivot(self, name):
        p = self.pivots.get(name)
        if p is None or "co" not in p:
            raise Blocked(f"pivot {name} missing in pivots.json")
        return Vector(p["co"])

    def pivot_axis(self, name):
        p = self.pivots.get(name)
        if p is None or "axis" not in p:
            raise Blocked(f"pivot {name}.axis missing in pivots.json")
        return unit(p["axis"])

    def ring_center(self, name):
        if self.mesh_co is None:
            raise Blocked(f"{MESH} missing in blend")
        r = self.loops.get(name)
        if r is None or not r.get("verts"):
            raise Blocked(f"ring {name} missing in retopo_loops.json")
        idx = np.asarray(r["verts"], dtype=np.int64)
        if idx.min() < 0 or idx.max() >= len(self.mesh_co):
            raise Blocked(f"ring {name} vertex index out of range of {MESH}")
        return Vector(self.mesh_co[idx].mean(axis=0).tolist())


def _load_json(path, ctx, key):
    if not path.exists():
        _STATE["missing"].append(rel(path))
        ctx.miss[key] = f"missing input: {rel(path)}"
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def check_setup(ctx):
    rig = bpy.data.objects.get(ARM)
    if rig is None or rig.type != "ARMATURE":
        ctx.err.append(f"armature object {ARM} missing in {rel(BLEND)}")
        return
    ctx.rig = rig
    ctx.mw = rig.matrix_world.copy()
    mo = bpy.data.objects.get(MESH)
    if mo is not None and mo.type == "MESH":
        ctx.mesh = mo
        n = len(mo.data.vertices)
        co = np.empty(n * 3, dtype=np.float64)
        mo.data.vertices.foreach_get("co", co)
        co = co.reshape(n, 3)
        mw = np.array(mo.matrix_world, dtype=np.float64)
        ctx.mesh_co = co @ mw[:3, :3].T + mw[:3, 3]


# ---------------------------------------------------------------- G4.1 / G4.2
def c_tree(ctx):
    bones = ctx.rig.data.bones
    names = [b.name for b in bones]
    missing = sorted(n for n in TREE if n not in bones)
    extra = sorted(n for n in names if n not in TREE)
    parent_mis, connect_mis = {}, {}
    for n, (par, con) in TREE.items():
        b = bones.get(n)
        if b is None:
            continue
        got = b.parent.name if b.parent else None
        if got != par:
            parent_mis[n] = {"got": got, "expected": par}
        if bool(b.use_connect) != con:
            connect_mis[n] = {"got": bool(b.use_connect), "expected": con}
    ok = len(names) == len(TREE) and not (missing or extra or parent_mis or connect_mis)
    return {"n_bones": len(names), "missing": missing, "extra": extra,
            "parent_mismatch": parent_mis, "connect_mismatch": connect_mis}, ok


def c_deform(ctx):
    mis = {}
    n_true = 0
    for n in TREE:
        b = ctx.rig.data.bones.get(n)
        if b is None:
            mis[n] = "missing"
            continue
        exp = n not in NON_DEFORM
        n_true += int(bool(b.use_deform))
        if bool(b.use_deform) != exp:
            mis[n] = {"got": bool(b.use_deform), "expected": exp}
    extra_deform = sorted(b.name for b in ctx.rig.data.bones if b.name not in TREE and b.use_deform)
    ok = not mis and not extra_deform
    return {"deform_true_count": n_true, "mismatch": mis, "extra_deform_bones": extra_deform}, ok


# ---------------------------------------------------------------- G4.3
def c_head(ctx, s, bone, kind, ref, bend):
    name = f"{bone}_{s}"
    got = ctx.head(name)
    refn = ref.format(s=s)
    base = ctx.pivot(refn) if kind == "pivot" else ctx.ring_center(refn)
    exp = base.copy()
    meas = {"head": mmv(got), "reference_point": mmv(base), "reference": f"{kind} {refn}"}
    if bend is not None:
        sign, off, axn = bend
        a = ctx.pivot_axis(axn.format(s=s))
        d = unit(perp(Vector((0.0, sign, 0.0)), a))
        exp = base + d * off
        meas["bend_dir"] = [rnd(c, 6) for c in d]
        meas["expected"] = mmv(exp)
    err = (got - exp).length
    meas["err_mm"] = mm(err)
    meas["delta_mm"] = mmv(got - exp)
    return meas, err <= HEAD_TOL


# ---------------------------------------------------------------- G4.4
def c_chain_x(ctx, s, chain):
    names = [f"{b}_{s}" for b in chain]
    xs = {n: ctx.axis(n, 0) for n in names}
    pair = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            pair[f"{names[i]}-{names[j]}"] = rnd(ang_deg(xs[names[i]], xs[names[j]]), 5)
    worst = max(pair.values())
    return {"max_deg": worst, "pair_deg": pair,
            "local_x": {n: [rnd(c, 6) for c in xs[n]] for n in names}}, worst <= AXIS_TOL_DEG


def pose_probe(ctx, bone, probe, deg):
    """Rotate pose bone `bone` by deg about its local +X (all other pose bones at rest); return the
    world tail of `probe` at rest and posed.  Pose values are restored afterwards."""
    ctx.bone(bone)
    ctx.bone(probe)
    rig = ctx.rig
    pbs = rig.pose.bones
    vl = bpy.context.view_layer
    saved = [(pb.name, pb.rotation_mode, tuple(pb.rotation_quaternion), tuple(pb.rotation_euler),
              tuple(pb.rotation_axis_angle), tuple(pb.location), tuple(pb.scale)) for pb in pbs]
    saved_pp = rig.data.pose_position
    try:
        rig.data.pose_position = "POSE"
        for pb in pbs:
            pb.rotation_mode = "QUATERNION"
            pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
            pb.location = (0.0, 0.0, 0.0)
            pb.scale = (1.0, 1.0, 1.0)
        vl.update()
        rest = ctx.mw @ pbs[probe].tail
        pbs[bone].rotation_quaternion = Quaternion((1.0, 0.0, 0.0), math.radians(deg))
        vl.update()
        posed = ctx.mw @ pbs[probe].tail
    finally:
        for name, mode, q, e, aa, loc, sc in saved:
            pb = pbs[name]
            pb.rotation_mode = mode
            pb.rotation_quaternion = q
            pb.rotation_euler = e
            pb.rotation_axis_angle = aa
            pb.location = loc
            pb.scale = sc
        rig.data.pose_position = saved_pp
        vl.update()
    return rest, posed


def c_bend_dir(ctx, s, bone, probe, want_sign):
    bn, pn = f"{bone}_{s}", f"{probe}_{s}"
    rest, posed = pose_probe(ctx, bn, pn, POSE_DEG)
    d = posed - rest
    rest_dev = (rest - ctx.tail(pn)).length
    ok = (d.y < 0.0) if want_sign < 0 else (d.y > 0.0)
    return {"rotated": bn, "probe": f"{pn} tail", "angle_deg": POSE_DEG,
            "displacement_mm": mmv(d), "dy_mm": mm(d.y), "disp_len_mm": mm(d.length),
            "rest_probe_vs_bone_tail_mm": mm(rest_dev)}, ok


# ---------------------------------------------------------------- G4.5
def c_twist(ctx):
    meas, ok = {}, True
    for s in SIDES:
        ua, uat = f"upperarm_{s}", f"upperarm_twist_{s}"
        la, lat = f"lowerarm_{s}", f"lowerarm_twist_{s}"
        try:
            uh, ut = ctx.head(ua), ctx.tail(ua)
            th, tt = ctx.head(uat), ctx.tail(uat)
            head_err = (th - uh).length
            len_err = abs((tt - th).length - 0.5 * (ut - uh).length)
            rot_deg = goblib.quat_angle_deg(ctx.rot(uat).to_quaternion(), ctx.rot(ua).to_quaternion())
            y_deg = ang_deg(ctx.axis(uat, 1), ctx.axis(ua, 1))
            x_deg = ang_deg(ctx.axis(uat, 0), ctx.axis(ua, 0))
            o = (head_err <= TWIST_POS_TOL and len_err <= TWIST_POS_TOL
                 and rot_deg <= TWIST_ROT_TOL_DEG)
            meas[uat] = {"head_err_mm": mm(head_err), "length_mm": mm((tt - th).length),
                         "parent_length_mm": mm((ut - uh).length), "length_err_mm": mm(len_err),
                         "rot_diff_deg": rnd(rot_deg, 6), "y_axis_deg": rnd(y_deg, 6),
                         "x_axis_deg": rnd(x_deg, 6), "ok": o}
        except Blocked as e:
            meas[uat], o = f"blocked: {e}", False
        ok = ok and o
        try:
            lh, lt = ctx.head(la), ctx.tail(la)
            th, tt = ctx.head(lat), ctx.tail(lat)
            mid = lh + (lt - lh) * 0.5
            head_err = (th - mid).length
            tail_err = (tt - lt).length
            rot_deg = goblib.quat_angle_deg(ctx.rot(lat).to_quaternion(), ctx.rot(la).to_quaternion())
            y_deg = ang_deg(ctx.axis(lat, 1), ctx.axis(la, 1))
            x_deg = ang_deg(ctx.axis(lat, 0), ctx.axis(la, 0))
            o = (head_err <= TWIST_POS_TOL and tail_err <= TWIST_POS_TOL
                 and rot_deg <= TWIST_ROT_TOL_DEG)
            meas[lat] = {"head_err_mm": mm(head_err), "tail_err_mm": mm(tail_err),
                         "rot_diff_deg": rnd(rot_deg, 6), "y_axis_deg": rnd(y_deg, 6),
                         "x_axis_deg": rnd(x_deg, 6), "ok": o}
        except Blocked as e:
            meas[lat], o = f"blocked: {e}", False
        ok = ok and o
    return meas, ok


# ---------------------------------------------------------------- G4.6
def c_socket(ctx):
    n = "weapon_socket_r"
    b = ctx.bone(n)
    h = ctx.head(n)
    g = ctx.pivot("grip_r")
    ga = ctx.pivot_axis("grip_r")
    y = ctx.axis(n, 1)
    err = (h - g).length
    yd = ang_deg(y, ga)
    par = b.parent.name if b.parent else None
    ok = err <= HEAD_TOL and yd <= AXIS_TOL_DEG and par == "hand_r"
    return {"head_mm": mmv(h), "grip_r_mm": mmv(g), "head_err_mm": mm(err),
            "local_y": [rnd(c, 6) for c in y], "grip_axis": [rnd(c, 6) for c in ga],
            "y_axis_deg": rnd(yd, 5), "parent": par}, ok


# ---------------------------------------------------------------- G4.7
def _canon_bones(canon):
    src = canon.get("bones", canon) if isinstance(canon, dict) else canon
    items = []
    if isinstance(src, dict):
        for name, b in src.items():
            if isinstance(b, dict):
                d = dict(b)
                d.setdefault("name", name)
                items.append(d)
    elif isinstance(src, list):
        items = [dict(b) for b in src if isinstance(b, dict)]
    if not items:
        raise Blocked("canonical_skeleton.json: no bone records found (expected 'bones' list/dict)")
    out, field_missing = [], {}
    for b in items:
        name = b.get("name")
        need = [k for k in ("name", "head", "tail", "roll", "use_deform") if k not in b]
        if need:
            raise Blocked(f"canonical_skeleton.json bone {name!r} lacks {need}")
        con = b.get("use_connect", b.get("connect"))
        if con is None:
            field_missing.setdefault(name, []).append("use_connect/connect")
        if "parent" not in b:
            field_missing.setdefault(name, []).append("parent")
        mat = None
        for k in ("matrix_local", "rest_matrix", "matrix", "rest"):
            if b.get(k) is not None:
                mat = b[k]
                break
        out.append({"name": name, "parent": b.get("parent") or None,
                    "head": Vector(b["head"]), "tail": Vector(b["tail"]), "roll": float(b["roll"]),
                    "use_deform": bool(b["use_deform"]), "connect": bool(con) if con is not None else False,
                    "matrix": mat})
    return out, field_missing


def _rebuild(bones):
    """Temporary armature built from the canonical records; returns {name: matrix_local}."""
    vl = bpy.context.view_layer
    prev_active = vl.objects.active
    prev_sel = [o for o in vl.objects if o.select_get()]
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    ad = bpy.data.armatures.new("_g4_rebuild")
    ob = bpy.data.objects.new("_g4_rebuild", ad)
    bpy.context.scene.collection.objects.link(ob)
    issues = []
    try:
        for o in prev_sel:
            o.select_set(False)
        ob.select_set(True)
        vl.objects.active = ob
        bpy.ops.object.mode_set(mode="EDIT")
        ebs = {}
        for b in bones:
            eb = ad.edit_bones.new(b["name"])
            eb.head = b["head"]
            eb.tail = b["tail"]
            eb.roll = b["roll"]
            eb.use_deform = b["use_deform"]
            ebs[b["name"]] = eb
        for b in bones:
            if b["parent"] is not None:
                p = ebs.get(b["parent"])
                if p is None:
                    issues.append(f"{b['name']}: parent {b['parent']} not in json")
                    continue
                ebs[b["name"]].parent = p
                ebs[b["name"]].use_connect = b["connect"]
        bpy.ops.object.mode_set(mode="OBJECT")
        mats = {b.name: b.matrix_local.copy() for b in ad.bones}
    finally:
        if bpy.context.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.armatures.remove(ad)
        for o in prev_sel:
            o.select_set(True)
        vl.objects.active = prev_active
    return mats, issues


def c_rebuild(ctx):
    if ctx.canon is None:
        raise Blocked(ctx.miss.get("canon", "canonical_skeleton.json not loaded"))
    bones, field_missing = _canon_bones(ctx.canon)
    mats, issues = _rebuild(bones)
    rb = ctx.rig.data.bones
    jn = [b["name"] for b in bones]
    only_json = sorted(set(jn) - set(rb.keys()))
    only_rig = sorted(set(rb.keys()) - set(jn))
    dup = sorted({n for n in jn if jn.count(n) > 1})
    per, worst, worst_n = {}, 0.0, None
    field_mis, json_mat = {}, {}
    for b in bones:
        n = b["name"]
        rbn = rb.get(n)
        if rbn is None:
            continue
        if n in mats:
            d = max_abs(mats[n], rbn.matrix_local)
            per[n] = d
            if d > worst:
                worst, worst_n = d, n
        else:
            issues.append(f"{n}: dropped from rebuilt armature")
        got = {"parent": rbn.parent.name if rbn.parent else None,
               "connect": bool(rbn.use_connect), "use_deform": bool(rbn.use_deform)}
        want = {"parent": b["parent"], "connect": b["connect"], "use_deform": b["use_deform"]}
        diff = {k: {"json": want[k], "rig": got[k]} for k in got if got[k] != want[k]}
        if diff:
            field_mis[n] = diff
        if b["matrix"] is not None:
            try:
                m = Matrix(b["matrix"])
                json_mat[n] = (max_abs(m, rbn.matrix_local) if len(m.row) == 4 and len(m.col) == 4
                               else "not 4x4")
            except Exception as e:  # report only
                json_mat[n] = f"unreadable: {e}"
    over = {n: float(f"{d:.3e}") for n, d in per.items() if d > REBUILD_TOL}
    num = [v for v in json_mat.values() if isinstance(v, float)]
    mw_dev = max_abs(ctx.mw, Matrix.Identity(4))
    ok = (not only_json and not only_rig and not dup and not issues and not field_mis
          and not field_missing and len(per) == len(rb) and worst <= REBUILD_TOL)
    return {"n_json_bones": len(bones), "n_compared": len(per),
            "max_abs_diff": float(f"{worst:.3e}"), "worst_bone": worst_n, "over_tol": over,
            "only_in_json": only_json, "only_in_rig": only_rig, "duplicate_names": dup,
            "field_mismatch": field_mis, "field_missing": field_missing, "build_issues": issues,
            "json_rest_matrix_max_abs_diff_report": float(f"{max(num):.3e}") if num else None,
            "json_rest_matrix_n": len(json_mat),
            "rig_matrix_world_dev_from_identity": float(f"{mw_dev:.3e}")}, ok


# ---------------------------------------------------------------- G4.8
def c_root(ctx):
    b = ctx.bone("root")
    h, t = ctx.head("root"), ctx.tail("root")
    herr = max(abs(c) for c in h)
    terr = max(abs(a - e) for a, e in zip(t, (0.0, 0.0, 0.2)))
    _axis, roll = bpy.types.Bone.AxisRollFromMatrix(b.matrix_local.to_3x3())
    ok = herr <= ROOT_TOL and terr <= ROOT_TOL and abs(roll) <= ROOT_TOL
    return {"head": [float(f"{c:.9f}") for c in h], "tail": [float(f"{c:.9f}") for c in t],
            "head_max_abs_err": float(f"{herr:.3e}"), "tail_max_abs_err": float(f"{terr:.3e}"),
            "roll_rad": float(f"{roll:.3e}")}, ok


# ---------------------------------------------------------------- G4.9
def _bend_geom(ctx, s, key):
    up, lo, ring, axp, target, ysign, chain = BEND_CHAINS[key]
    upn, lon = f"{up}_{s}", f"{lo}_{s}"
    du = ctx.tail(upn) - ctx.head(upn)
    dl = ctx.tail(lon) - ctx.head(lon)
    rc = ctx.ring_center(ring.format(s=s))
    a = ctx.pivot_axis(axp.format(s=s))
    off = ctx.head(lon) - rc
    op = perp(off, a)
    return {"upn": upn, "lon": lon, "du": du, "dl": dl, "off": off, "op": op, "axis": a,
            "target": target, "ysign": ysign, "chain": [f"{c}_{s}" for c in chain]}


def c_bend_a(ctx, s, key):
    g = _bend_geom(ctx, s, key)
    d = ang_deg(g["du"], g["dl"])
    return {"angle_deg": rnd(d, 5), "segments": [g["upn"], g["lon"]]}, \
        BEND_ANG_DEG[0] <= d <= BEND_ANG_DEG[1]


def c_bend_b(ctx, s, key):
    g = _bend_geom(ctx, s, key)
    mag = g["op"].length
    return {"perp_offset_mm": mm(mag), "target_mm": mm(g["target"]),
            "axial_offset_mm": mm(g["off"].dot(g["axis"])), "perp_vec_mm": mmv(g["op"])}, \
        abs(mag - g["target"]) <= BEND_OFF_TOL


def c_bend_c(ctx, s, key):
    g = _bend_geom(ctx, s, key)
    y = g["op"].y
    ok = (y > 0.0) if g["ysign"] > 0 else (y < 0.0)
    return {"perp_offset_y_mm": mm(y), "expected_sign": "+" if g["ysign"] > 0 else "-"}, ok


def c_bend_d(ctx, s, key):
    g = _bend_geom(ctx, s, key)
    n = unit(g["du"].cross(g["dl"]))
    per = {}
    lim = math.cos(math.radians(AXIS_TOL_DEG))
    ok = True
    for b in g["chain"]:
        c = abs(n.dot(ctx.axis(b, 0)))
        per[b] = {"abs_cos": rnd(c, 8), "angle_deg": rnd(math.degrees(math.acos(min(1.0, c))), 5)}
        ok = ok and c >= lim
    return {"bend_normal": [rnd(c, 6) for c in n], "per_bone": per,
            "min_abs_cos": min(v["abs_cos"] for v in per.values())}, ok


# ---------------------------------------------------------------- criterion table
def specs():
    """[(id, threshold, note, fn(ctx) -> (measured, ok))] in evidence order."""
    P = functools.partial
    out = [
        ("G4.1", "24 bones; names, parents, use_connect == spec section 2 literal; no other bones",
         "GOB_rig.data.bones vs fixed TREE literal (connect = child head on parent tail per spec "
         "section 2 pivot table / plan T18)", c_tree),
        ("G4.2", "use_deform False: root, weapon_socket_r; True: other 22",
         "bone.use_deform of every TREE bone; extra deform bones listed", c_deform),
    ]
    for s in SIDES:
        for bone, kind, ref, bend in HEAD_REFS:
            if bend is None:
                how = f"|head - {kind} {ref.format(s=s)}|"
            else:
                how = (f"|head - (ring center {ref.format(s=s)} + perp({'+' if bend[0] > 0 else '-'}Y, "
                       f"pivots {bend[2].format(s=s)}.axis) * {bend[1] * 1000:g} mm)|")
            out.append((f"G4.3_{bone}_{s}", f"<= {HEAD_TOL * 1000:g} mm",
                        how + "; ring center = mean GOB_mesh world co of ring verts; head = world head",
                        P(c_head, s=s, bone=bone, kind=kind, ref=ref, bend=bend)))
    for key, chain in (("arm", ARM_CHAIN), ("leg", LEG_CHAIN)):
        for s in SIDES:
            out.append((f"G4.4a_{key}_{s}", f"max pairwise angle of local X <= {AXIS_TOL_DEG:g} deg",
                        "signed angle between world rest local X (matrix_local col 0) of "
                        + ", ".join(f"{c}_{s}" for c in chain), P(c_chain_x, s=s, chain=chain)))
    for s in SIDES:
        out.append((f"G4.4b_{s}", "hand tail dy < 0 (moves -Y)",
                    f"pose lowerarm_{s} quaternion +{POSE_DEG:g} deg about local +X, all other pose bones "
                    f"identity; displacement of world hand_{s} tail; in memory, restored, not saved",
                    P(c_bend_dir, s=s, bone="lowerarm", probe="hand", want_sign=-1)))
    for s in SIDES:
        out.append((f"G4.4c_{s}", "foot tail dy > 0 (moves +Y)",
                    f"pose calf_{s} quaternion +{POSE_DEG:g} deg about local +X, all other pose bones "
                    f"identity; displacement of world foot_{s} tail; in memory, restored, not saved",
                    P(c_bend_dir, s=s, bone="calf", probe="foot", want_sign=+1)))
    out.append(("G4.5", f"positions/length <= {TWIST_POS_TOL * 1000:g} mm; rotation (axis+roll) <= "
                        f"{TWIST_ROT_TOL_DEG:g} deg",
                "upperarm_twist: |head - upperarm head|, |len - 0.5 upperarm len|; lowerarm_twist: "
                "|head - lowerarm midpoint|, |tail - lowerarm tail|; rotation = quat angle between world "
                "rest rotations (x/y axis angles reported)", c_twist))
    out.append(("G4.6", f"head err <= {HEAD_TOL * 1000:g} mm; local Y vs grip axis <= {AXIS_TOL_DEG:g} deg; "
                        "parent hand_r",
                "|weapon_socket_r head - pivots grip_r.co|; signed angle world local Y (matrix_local "
                "col 1) vs pivots grip_r.axis", c_socket))
    out.append(("G4.7", f"max abs element diff of matrix_local <= {REBUILD_TOL:g}; same bone set; "
                        "parent/connect/use_deform equal",
                "temporary armature built in edit mode from canonical_skeleton.json (head, tail, roll, "
                "parent, connect, use_deform), matrix_local compared with GOB_rig bone by bone, then "
                "deleted (not saved); json rest matrix diff reported only", c_rebuild))
    out.append(("G4.8", f"head (0,0,0), tail (0,0,0.2), roll 0; each <= {ROOT_TOL:g}",
                "world head/tail max abs error (m); roll = Bone.AxisRollFromMatrix(matrix_local) (rad)",
                c_root))
    for key in BEND_CHAINS:
        for s in SIDES:
            joint = "elbow" if key == "arm" else "knee"
            out.append((f"G4.9a_{key}_{s}", f"{BEND_ANG_DEG[0]:g} <= angle <= {BEND_ANG_DEG[1]:g} deg",
                        "angle between the two segment directions (tail - head) of "
                        + ("upperarm/lowerarm" if key == "arm" else "thigh/calf"),
                        P(c_bend_a, s=s, key=key)))
            tgt = ELBOW_BEND if key == "arm" else KNEE_BEND
            out.append((f"G4.9b_{key}_{s}", f"{tgt * 1000:g} +- {BEND_OFF_TOL * 1000:g} mm",
                        f"|perp component (to pivots {joint}_{s}.axis) of "
                        f"({'lowerarm' if key == 'arm' else 'calf'}_{s} head - ring center {joint}_{s}_1)|",
                        P(c_bend_b, s=s, key=key)))
            out.append((f"G4.9c_{key}_{s}", "perp offset Y > 0" if key == "arm" else "perp offset Y < 0",
                        "sign of the Y component of the G4.9b perpendicular offset",
                        P(c_bend_c, s=s, key=key)))
            out.append((f"G4.9d_{key}_{s}", f"|cos(normal, local X)| >= cos {AXIS_TOL_DEG:g} deg for "
                                            "every chain bone",
                        "bend-plane normal = unit(upper dir x lower dir); |cos| with world rest local X "
                        "of " + ", ".join(f"{c}_{s}" for c in (ARM_CHAIN if key == "arm" else LEG_CHAIN)),
                        P(c_bend_d, s=s, key=key)))
    return out


# ---------------------------------------------------------------- main
def main():
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
    ev.add_stage_inputs("s04")

    loops = _load_json(LOOPS_JSON, ctx, "loops")
    piv = _load_json(PIVOTS_JSON, ctx, "pivots")
    ctx.loops = loops.get("rings", {}) if loops else None
    ctx.pivots = piv.get("pivots", {}) if piv else None
    ctx.canon = _load_json(CANON_JSON, ctx, "canon")
    if CANON_JSON.exists():
        ev.add_input(CANON_JSON)

    table = specs()
    try:
        blocking = [ctx.miss[k] for k in ("blend", "loops", "pivots") if ctx.miss.get(k)]
        if not blocking:
            check_setup(ctx)
            blocking = ctx.err
        if blocking:
            r = "blocked: " + "; ".join(blocking)
            for cid, th, note, _fn in table:
                ev.criterion(cid, r, th, False, note)
            return
        for cid, th, note, fn in table:
            try:
                measured, ok = fn(ctx)
            except Blocked as e:
                measured, ok = f"blocked: {e}", False
            ev.criterion(cid, measured, th, ok, note)
    finally:
        ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
