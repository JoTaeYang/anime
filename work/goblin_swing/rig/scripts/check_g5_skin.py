"""check_g5_skin - Gate G5 SKIN checker (design doc d-23 section 4 and section 7, G5.1 .. G5.7).

Inputs (independent of the production script s05_skin.py; only blend content + data json):
  rig/gob_r03_skinned.blend   GOB_rig (24 bones = canonical_skeleton), GOB_mesh (Armature modifier ->
                              GOB_rig, vertex groups = deform bone names, face INT part_id),
                              GOB_club (bone parent weapon_socket_r)
  rig/data/skin_decisions.json  written by s05 (belt_bone, twist {...}); read flexibly, missing keys noted
  rig/data/rom_poses.json       owned by this checker (T20), fixed hand-written pose table
  rig/data/parts.json, retopo_loops.json, pivots.json (canonical_skeleton.json optional, info only)

Poses: rom_poses.json poses[].rot {bone: [X, Y, Z] local Euler deg, rotation_mode XYZ}; every other pose
bone at rest; twist bones are driven by their constraints.  Applied in memory only, restored at the end;
the blend is never saved.

Criteria
  G5.1  per vertex: nonzero weights (> 1e-4) <= 4, weight sum = 1 +-1e-4, root / weapon_socket_r weight
        0 (<= 1e-4); vertex group names subset of the deform bone names; Armature modifier -> GOB_rig.
  G5.2  rigid parts (part_id + parts.json): every vertex 100 % on one bone: head->head, hand_x->hand_x,
        shoe_x->foot_x, belt->skin_decisions belt bone (Option A one bone; Option B: two bones recorded,
        nonzero weights must stay inside them).  HR2 (2026-09-26, hr2_contract.md): the mouth vertices
        (faces of material slots GOB_mouth_inner / GOB_teeth / GOB_tongue + vertices the mouth_open shape
        key moves) must be head-part vertices and 100 % DEF head; reported under head.mouth (no mouth = n 0).
  G5.3  twist bones upperarm_twist_x / lowerarm_twist_x: constraint of the skin_decisions type (TRANSFORM
        SWING_TWIST_Y; COPY_ROTATION old contract accepted) on GOB_rig/source, mode / spaces / Y mapping slope
        = sign*ratio match skin_decisions; behaviour: source local Y rotated +90 deg (upperarm for upperarm_twist, hand for
        lowerarm_twist, or the recorded source) -> twist bone axial (Y) rotation relative to its parent
        = sign * ratio * 90 deg +-2 deg (swing-twist decomposition of parent-relative delta).
  G5.3b twist-free swing of the source about local (X+Z)/sqrt2 (upperarm 90 deg, hand 60 deg): twist bone
        axial rotation relative to its parent <= 2 deg (per twist bone).
  G5.4  every ROM pose: rigid part vertices vs the rigid transform of their bone (posed world @ rest
        world^-1 applied to the rest vertex) max residual < 1 mm; GOB_club vertices vs weapon_socket_r.
  G5.5  per pose, per measured joint side (G3 metric definitions, copied from check_g3_deform):
          section ratio = min over slices of (min in-plane ray radius posed / rest), slices every 2.5 mm
            over the retopo ring span +-10 mm, 72 rays, plane moved with the owner bone;
          joint-region self intersection = BVH overlap of region body faces (same polygon / shared vertex
            pairs dropped; body faces only so designed body-rigid overlaps never enter);
          collapsed faces = region faces with posed area < 10 % of rest;
          boundary exposure = signed depth of wrist/ankle `_end` and `_1` ring verts inside the posed
            hand_x / shoe_x shell (+ inside, BVH nearest distance, ray parity majority of 3 directions).
        ok uses tier=judged poses only: ratio >= 0.8 (30 deg, wrist twist 90, ankle 30) / >= 0.75 (60 deg),
        collapse 0, min face set covering every intersecting pair <= 1 per joint region, `_end` depth >= 0, `_1` depth
        >= rest depth - 0.1 mm; report poses are recorded in measured only (2026-09-25 실측 후 확정).
  G5.6  body shell volume (divergence theorem over part_id=body triangles), |V/V0 - 1| <= 5 % all poses.
  G5.7  [U] sheets rig/inspect/G5/rom_<group>.png for shoulder, elbow, wrist, hip, knee, ankle,
        spine_head (G3 style: wire cut-out close-ups, rows = pose kind x side, cols = angle / variant)
        and full (rows = pose, cols = front / three_quarter, whole GOB_mesh + GOB_club, solid).

Run:    bl.ps1 -Script check_g5_skin.py -Blend gob_r03_skinned.blend
Output: rig/inspect/G5/check_g5_skin.json + rig/inspect/G5/rom_<group>.png
Exit:   0 when the evidence was written; 2 on a script error (run_main) or when an input file is missing
        (the evidence is still written, then "missing input" is printed).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.  Lengths in m, reported in mm.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import json  # noqa: E402
import math  # noqa: E402
import shutil  # noqa: E402
import tempfile  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
import bpy  # noqa: E402
from mathutils import Euler, Quaternion, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

GATE = "G5"
CHECKER = "check_g5_skin"
BLEND = goblib.RIG / "gob_r03_skinned.blend"
PARTS_JSON = goblib.DATA / "parts.json"
LOOPS_JSON = goblib.DATA / "retopo_loops.json"
PIVOTS_JSON = goblib.DATA / "pivots.json"
ROM_JSON = goblib.DATA / "rom_poses.json"
SKIN_JSON = goblib.DATA / "skin_decisions.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"

ARM = "GOB_rig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
SOCKET = "weapon_socket_r"
SIDES = ("l", "r")

NONZERO = 1e-4           # G5.1
MAX_INF = 4
SUM_TOL = 1e-4
RIGID_TOL = 1e-4         # G5.2: expected bone weight >= 1 - tol, other weights <= tol
MOUTH_KEY = "mouth_open"                                  # G5.2 HR2 (hr2_contract.md)
MOUTH_SLOTS = ("GOB_mouth_inner", "GOB_teeth", "GOB_tongue")
MOUTH_MOVE_EPS = 1e-6    # G5.2 HR2: vertex moved by mouth_open when |key - reference key| > this (m)
TWIST_TEST_DEG = 90.0    # G5.3
TWIST_TOL_DEG = 2.0
RATIO_TOL = 1e-3
RESID_MAX = 0.001        # G5.4 (m)
RATIO_MIN = 0.8          # G5.5: 30 deg poses, wrist twist 90, ankle 30
RATIO_MIN_60 = 0.75      # G5.5: 60 deg poses (2026-09-25 user decision)
ISECT_COVER_MAX = 1      # G5.5: minimum face set covering every intersecting pair, per joint region
END1_TOL = 0.0001        # G5.5: `_1` ring depth >= rest depth - 0.1 mm (m)
SWING_TEST_DEG = {"upperarm": 90.0, "hand": 60.0}   # G5.3b twist-free swing about (X+Z)/sqrt2
SLOPE_TOL = 1e-3         # G5.3 Transformation Y mapping slope vs sign*ratio
DEPTH_MIN = 0.0          # G5.5 exposure (m)
COLLAPSE = 0.10          # G5.5
VOL_MAX = 0.05           # G5.6
SLICE_STEP = 0.0025      # m
SLICE_MARGIN = 0.010     # m
N_RAYS = 72
RAY_MAX = 0.30           # m
LIST_CAP = 10
CELL_PX = 400
FULL_PX = 720
CELL_GAP = 8
DRAFT = "2026-09-25 실측 후 확정"

RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237),
                                              (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6

# joint -> rings (k suffixes), (proximal owner, proximal axis bone), (distal owner, distal axis bone)  (= G3)
JOINTS = {
    "shoulder": {"rings": ("0", "1", "2"), "prox": ("shoulder", "upperarm"), "dist": ("upperarm", "upperarm")},
    "elbow": {"rings": ("0", "1", "2"), "prox": ("upperarm", "upperarm"), "dist": ("lowerarm", "lowerarm")},
    "wrist": {"rings": ("0", "1"), "prox": ("lowerarm", "lowerarm"), "dist": ("hand", "lowerarm")},
    "hip": {"rings": ("0", "1", "2"), "prox": ("pelvis", "thigh"), "dist": ("thigh", "thigh")},
    "knee": {"rings": ("0", "1", "2"), "prox": ("thigh", "thigh"), "dist": ("calf", "calf")},
    "ankle": {"rings": ("0", "1"), "prox": ("calf", "calf"), "dist": ("foot", "calf")},
}
LIMB = {"shoulder": "arm", "elbow": "arm", "wrist": "arm", "hip": "leg", "knee": "leg", "ankle": "leg"}
# boundary exposure: end joint -> (rim rings, rigid part, limb)
END_RING = {"wrist": (("wrist_{s}_end", "wrist_{s}_1"), "hand_{s}", "arm"),
            "ankle": (("ankle_{s}_end", "ankle_{s}_1"), "shoe_{s}", "leg")}
LIMB_END = {"arm": "wrist", "leg": "ankle"}
TWIST_BONES = ("upperarm_twist_l", "upperarm_twist_r", "lowerarm_twist_l", "lowerarm_twist_r")
TWIST_DEFAULT_SOURCE = {"upperarm_twist": "upperarm", "lowerarm_twist": "hand"}
RIGID_FIXED = {"head": "head", "hand_l": "hand_l", "hand_r": "hand_r", "shoe_l": "foot_l", "shoe_r": "foot_r"}

GROUPS = ("shoulder", "elbow", "wrist", "hip", "knee", "ankle", "spine_head", "full")
CELL_HALF = {"shoulder": 0.20, "elbow": 0.15, "wrist": 0.12, "hip": 0.15, "knee": 0.13, "ankle": 0.20,
             "spine_head": 0.50}
CELL_DEPTH = {"shoulder": 0.08, "elbow": 0.08, "wrist": 0.08, "hip": 0.08, "knee": 0.08, "ankle": 0.13,
              "spine_head": 0.60}
FULL_VIEWS = ("front", "three_quarter")
FULL_BBOX = ((-1.0, -1.0, -0.05), (1.0, 1.0, 1.8))
BODY_COLOR = (0.80, 0.80, 0.80, 1.0)
RIGID_COLOR = (0.55, 0.72, 0.95, 1.0)
HIDDEN_PARTS = ("head", "belt")      # joint close-ups (G3 style); spine_head shows every part

_STATE = {"missing": []}


# ---------------------------------------------------------------- helpers
def mm(x):
    if x is None:
        return None
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=4):
    return None if x is None else round(float(x), n)


def rel(p):
    return goblib._rel(p)


def unit(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def side_name(bone, x):
    return bone if bone == "pelvis" else f"{bone}_{x}"


def num_key(s):
    try:
        return (0, float(s), "")
    except (TypeError, ValueError):
        return (1, 0.0, str(s))


class Topo:
    """Topology of GOB_mesh.data (indices shared with the evaluated mesh)."""

    def __init__(self, obj, part_names):
        me = obj.data
        self.nv, self.npoly = len(me.vertices), len(me.polygons)
        ls = np.empty(self.npoly, dtype=np.int32)
        lt = np.empty(self.npoly, dtype=np.int32)
        me.polygons.foreach_get("loop_start", ls)
        me.polygons.foreach_get("loop_total", lt)
        lv = np.empty(len(me.loops), dtype=np.int32)
        me.loops.foreach_get("vertex_index", lv)
        self.loop_vert = lv.astype(np.int64)
        self.loop_poly = np.repeat(np.arange(self.npoly), lt.astype(np.int64))
        self.poly_nv = lt.astype(np.int64)
        self.poly_verts = [self.loop_vert[s:s + n].tolist() for s, n in zip(ls.tolist(), lt.tolist())]
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        tv = np.empty(nt * 3, dtype=np.int32)
        tp = np.empty(nt, dtype=np.int32)
        me.loop_triangles.foreach_get("vertices", tv)
        me.loop_triangles.foreach_get("polygon_index", tp)
        self.tri_v = tv.reshape(nt, 3).astype(np.int64)
        self.tri_poly = tp.astype(np.int64)
        self.part = None
        att = me.attributes.get("part_id")
        if att is not None and att.domain == "FACE" and att.data_type == "INT":
            pv = np.empty(self.npoly, dtype=np.int32)
            att.data.foreach_get("value", pv)
            self.part = pv.astype(np.int64)
        self.part_names = part_names

    def polys_of(self, name):
        pid = self.part_names.get(name)
        if pid is None or self.part is None:
            return np.zeros(0, dtype=np.int64)
        return np.nonzero(self.part == pid)[0]

    def verts_of(self, name):
        polys = self.polys_of(name)
        if not len(polys):
            return np.zeros(0, dtype=np.int64)
        return np.unique(np.concatenate([np.array(self.poly_verts[int(p)], dtype=np.int64)
                                         for p in polys.tolist()]))

    def tris_of_polys(self, polys):
        m = np.zeros(self.npoly, dtype=bool)
        m[polys] = True
        return np.nonzero(m[self.tri_poly])[0]

    def poly_area(self, V):
        T = V[self.tri_v]
        a = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
        return np.bincount(self.tri_poly, weights=a, minlength=self.npoly)

    def poly_centroid(self, V):
        c = np.zeros((self.npoly, 3))
        np.add.at(c, self.loop_poly, V[self.loop_vert])
        return c / self.poly_nv[:, None]


def eval_verts(obj):
    """World coords (nv,3) of the depsgraph-evaluated mesh of obj."""
    dg = bpy.context.evaluated_depsgraph_get()
    oe = obj.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        n = len(me.vertices)
        co = np.empty(n * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
    finally:
        oe.to_mesh_clear()
    mw = np.array(oe.matrix_world, dtype=np.float64)
    return co.reshape(n, 3).astype(np.float64) @ mw[:3, :3].T + mw[:3, 3]


def bvh_of(V, tri_v):
    return BVHTree.FromPolygons([tuple(p) for p in V.tolist()], tri_v.tolist(), all_triangles=True)


def _inside(bvh, p):
    votes = 0
    for d in RAY_DIRS:
        o, n = p.copy(), 0
        for _ in range(512):
            hit = bvh.ray_cast(o, d)
            if hit[0] is None:
                break
            n += 1
            o = hit[0] + d * RAY_EPS
        votes += n % 2
    return votes >= 2


def signed_depth(bvh, pts):
    """+ distance to the surface when inside (ray parity), - distance when outside."""
    out = np.empty(len(pts))
    for i, p in enumerate(pts.tolist()):
        pv = Vector(p)
        loc, _, _, dist = bvh.find_nearest(pv)
        if loc is None:
            out[i] = -np.inf
            continue
        out[i] = dist if _inside(bvh, pv) else -dist
    return out


def mat_np(M):
    return np.array(M, dtype=np.float64)


def shell_volume(V, tri_v):
    """Signed volume of a closed triangle shell (divergence theorem)."""
    T = V[tri_v]
    return float(np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6.0)


def twist_angle_deg(M3):
    """Swing-twist decomposition about local Y of a 3x3 rotation: (twist deg, swing deg)."""
    q = M3.to_quaternion().normalized()
    tw = math.degrees(2.0 * math.atan2(q.y, q.w))
    tw = (tw + 180.0) % 360.0 - 180.0
    y = M3 @ Vector((0.0, 1.0, 0.0))
    swing = math.degrees(math.atan2(y.cross(Vector((0.0, 1.0, 0.0))).length, y.y))
    return tw, swing


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.miss = {}
        self.err = []
        self.parts = self.loops = self.pivots = self.rom = self.dec = self.canon = None
        self.arm = self.mesh = self.club = None
        self.topo = None
        self.notes = []
        self.dec_notes = []
        self.poses = []
        self.rot_mode = "XYZ"
        self.saved_modes = {}

    def m(self):
        return [v for v in self.miss.values() if v] + self.err


def check_setup(ctx):
    arm = bpy.data.objects.get(ARM)
    mo = bpy.data.objects.get(MESH)
    club = bpy.data.objects.get(CLUB)
    if arm is None or arm.type != "ARMATURE":
        ctx.err.append(f"missing object: {ARM}" if arm is None else f"{ARM} is {arm.type}, not ARMATURE")
    if mo is None or mo.type != "MESH":
        ctx.err.append(f"missing object: {MESH}" if mo is None else f"{MESH} is {mo.type}, not MESH")
    if club is None or club.type != "MESH":
        ctx.err.append(f"missing object: {CLUB}" if club is None else f"{CLUB} is {club.type}, not MESH")
    if ctx.err:
        return
    ctx.arm, ctx.mesh, ctx.club = arm, mo, club
    need = {"pelvis", "spine_01", "spine_02", "head", SOCKET}
    for x in SIDES:
        for b in ("shoulder", "upperarm", "upperarm_twist", "lowerarm", "lowerarm_twist", "hand",
                  "thigh", "calf", "foot"):
            need.add(f"{b}_{x}")
    missing = sorted(n for n in need if n not in arm.pose.bones)
    if missing:
        ctx.err.append("missing bones: " + ", ".join(missing))
    if not isinstance(ctx.parts, dict):
        ctx.err.append("parts.json is not an object")
        return
    ctx.topo = Topo(mo, {k: v for k, v in ctx.parts.items() if isinstance(v, int)})
    if ctx.topo.part is None:
        ctx.err.append("GOB_mesh part_id FACE INT attribute missing/invalid")
    for p in ("body", "head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt"):
        if p not in ctx.parts:
            ctx.err.append(f"parts.json has no '{p}'")
    rings = (ctx.loops or {}).get("rings") if isinstance(ctx.loops, dict) else None
    if not isinstance(rings, dict):
        ctx.err.append("retopo_loops.json has no 'rings' object")
    piv = (ctx.pivots or {}).get("pivots") if isinstance(ctx.pivots, dict) else None
    if not isinstance(piv, dict):
        ctx.err.append("pivots.json has no 'pivots' object")
    if ctx.err:
        return
    ctx.rings, ctx.piv = rings, piv
    bad = []
    for j, spec in JOINTS.items():
        for x in SIDES:
            if f"{j}_{x}" not in piv:
                bad.append(f"pivot {j}_{x}")
            for k in spec["rings"]:
                if f"{j}_{x}_{k}" not in rings:
                    bad.append(f"ring {j}_{x}_{k}")
    for j, (rts, _, _) in END_RING.items():
        for x in SIDES:
            for rt in rts:
                if rt.format(s=x) not in rings:
                    bad.append(f"ring {rt.format(s=x)}")
    if bad:
        ctx.err.append("missing data: " + ", ".join(bad))
    parse_poses(ctx)


def parse_poses(ctx):
    rom = ctx.rom
    if not isinstance(rom, dict) or not isinstance(rom.get("poses"), list):
        ctx.err.append("rom_poses.json has no 'poses' list")
        return
    ctx.rot_mode = str(rom.get("rotation_mode", "XYZ"))
    bones = ctx.arm.pose.bones
    seen, bad = set(), []
    for p in rom["poses"]:
        pid = p.get("id")
        if not pid or pid in seen:
            bad.append(f"pose id {pid!r} empty/duplicate")
            continue
        seen.add(pid)
        for k in ("group", "side", "tier", "joint", "rot"):
            if k not in p:
                bad.append(f"{pid}: no '{k}'")
        if p.get("group") not in GROUPS:
            bad.append(f"{pid}: group {p.get('group')!r}")
        if p.get("tier") not in ("judged", "report"):
            bad.append(f"{pid}: tier {p.get('tier')!r}")
        for b, e in (p.get("rot") or {}).items():
            if b not in bones:
                bad.append(f"{pid}: bone {b} not in {ARM}")
            if b.startswith(("upperarm_twist", "lowerarm_twist")):
                bad.append(f"{pid}: twist bone {b} in rot (driven by constraints)")
            if not (isinstance(e, (list, tuple)) and len(e) == 3):
                bad.append(f"{pid}: rot {b} not [x,y,z]")
        j = p.get("joint")
        js = list(JOINTS) if j == "all" else ([j] if isinstance(j, str) else list(j or []))
        for jj in js:
            if jj not in JOINTS:
                bad.append(f"{pid}: joint {jj!r}")
        sides = list(SIDES) if p.get("side") not in SIDES else [p["side"]]
        q = dict(p)
        q["joint_sides"] = [(jj, x) for jj in js for x in sides if jj in JOINTS]
        limbs = sorted({LIMB[jj] for jj, _ in q["joint_sides"]})
        q["end_sides"] = [(LIMB_END[lb], x) for lb in limbs for x in sides]
        ctx.poses.append(q)
    if bad:
        ctx.err.append("rom_poses.json: " + "; ".join(bad[:20]))


# ---------------------------------------------------------------- skin decisions (flexible read)
def _sign_of(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return 1 if v > 0 else (-1 if v < 0 else None)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("+", "+1", "1", "pos", "positive", "same", "follow", "forward"):
            return 1
        if s in ("-", "-1", "neg", "negative", "invert", "inverted", "opposite", "counter", "reverse"):
            return -1
        if s.startswith("-") or "counter" in s or "invert" in s or "opposite" in s:
            return -1
        if s.startswith("+") or "same" in s:
            return 1
    return None


def parse_decisions(ctx):
    d = ctx.dec if isinstance(ctx.dec, dict) else {}
    notes = ctx.dec_notes
    # belt
    belt, bkey, option = None, None, None
    for key in ("belt_bone", "belt_bones", "belt"):
        v = d.get(key)
        if isinstance(v, str):
            belt, bkey = [v], key
        elif isinstance(v, (list, tuple)) and v and all(isinstance(s, str) for s in v):
            belt, bkey = list(v), key
        elif isinstance(v, dict):
            option = v.get("option")
            for k2 in ("bone", "bones", "belt_bone", "belt_bones"):
                w = v.get(k2)
                if isinstance(w, str):
                    belt, bkey = [w], f"{key}.{k2}"
                elif isinstance(w, (list, tuple)) and w and all(isinstance(s, str) for s in w):
                    belt, bkey = list(w), f"{key}.{k2}"
                if belt:
                    break
        if belt:
            break
    if option is None:
        option = d.get("belt_option")
    if not belt:
        belt = ["pelvis"]
        notes.append("skin_decisions: belt_bone key missing; belt expected bone falls back to 'pelvis' "
                     "(plan Phase 5 main decision)")
    ctx.belt = {"bones": belt, "key": bkey, "option": option if option is not None else
                ("A" if len(belt) == 1 else "B")}
    # twist
    tw, tkey = None, None
    for key in ("twist", "twist_constraints", "twists"):
        if isinstance(d.get(key), dict):
            tw, tkey = d[key], key
            break
    if tw is None:
        notes.append("skin_decisions: 'twist' object missing")
        tw = {}
    ctx.twist_dec = {}
    for bone in TWIST_BONES:
        base, x = bone[:-2], bone[-1]
        e, ekey = None, None
        for k in (bone, base + "_x", base, base + "_{x}"):
            if isinstance(tw.get(k), dict):
                e, ekey = tw[k], k
                break
        if isinstance(e, dict) and isinstance(e.get(x), dict):
            e, ekey = e[x], f"{ekey}.{x}"
        rec = {"key": f"{tkey}.{ekey}" if ekey else None, "raw": e}
        if not isinstance(e, dict):
            notes.append(f"skin_decisions: twist entry for {bone} missing")
            e = {}
        src = None
        for k in ("source", "source_bone", "subtarget", "target_bone", "target", "driver"):
            if isinstance(e.get(k), str):
                src = e[k]
                break
        if src is None:
            notes.append(f"skin_decisions: {bone} source missing; default {TWIST_DEFAULT_SOURCE[base]}_{x}")
            src = TWIST_DEFAULT_SOURCE[base] + "_" + x
        else:
            if src.endswith("_x"):
                src = src[:-2] + "_" + x
            elif "{x}" in src or "{s}" in src:
                src = src.replace("{x}", x).replace("{s}", x)
            elif ctx.arm is not None and src not in ctx.arm.pose.bones and f"{src}_{x}" in ctx.arm.pose.bones:
                src = f"{src}_{x}"
        axis, sign = None, None
        a = e.get("axis")
        if isinstance(a, str) and a.strip():
            s = a.strip().upper()
            if s[0] in "+-":
                sign = -1 if s[0] == "-" else 1
                s = s[1:]
            axis = s.replace("_", "").replace("LOCAL", "")[:1] or None
        else:
            notes.append(f"skin_decisions: {bone} axis missing")
        ratio = None
        for k in ("ratio", "influence", "factor", "weight"):
            if isinstance(e.get(k), (int, float)) and not isinstance(e.get(k), bool):
                ratio = float(e[k])
                break
        if ratio is None:
            notes.append(f"skin_decisions: {bone} ratio missing")
        sk = None
        for k in ("sign", "direction", "dir"):
            if k in e:
                sk = _sign_of(e[k])
                if sk is not None:
                    break
        if sk is None:
            for k in ("invert", "inverted", "invert_y", "invert_axis"):
                if isinstance(e.get(k), bool):
                    sk = -1 if e[k] else 1
                    break
        if sk is not None:
            sign = sk
        if sign is None:
            notes.append(f"skin_decisions: {bone} sign/direction missing (behaviour test compares magnitude)")
        space = None
        for k in ("space", "owner_space", "target_space"):
            if isinstance(e.get(k), str):
                space = e[k]
                break
        extra = {}
        for k in ("constraint", "from_rotation_mode", "target_space", "owner_space", "mix_mode_rot", "effect"):
            extra[k] = e.get(k) if isinstance(e.get(k), str) else None
        if extra["constraint"] is None:
            notes.append(f"skin_decisions: {bone} 'constraint' missing (COPY_ROTATION or TRANSFORM accepted)")
        rec.update({"source": src, "axis": axis, "ratio": ratio, "sign": sign, "space": space, **extra})
        ctx.twist_dec[bone] = rec


# ---------------------------------------------------------------- pose control
def reset_pose(arm):
    for pb in arm.pose.bones:
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)


def update():
    bpy.context.view_layer.update()


def rest_world(arm, name):
    return arm.matrix_world @ arm.data.bones[name].matrix_local


def pose_world(arm, name):
    return arm.matrix_world @ arm.pose.bones[name].matrix


def set_rot(ctx, bone, deg, mode=None):
    pb = ctx.arm.pose.bones[bone]
    if bone not in ctx.saved_modes:
        ctx.saved_modes[bone] = pb.rotation_mode
    pb.rotation_mode = mode or ctx.rot_mode
    pb.rotation_euler = Euler([math.radians(float(a)) for a in deg], pb.rotation_mode)


def apply_pose(ctx, p):
    reset_pose(ctx.arm)
    if p is not None:
        for b, e in p["rot"].items():
            set_rot(ctx, b, e)
    update()


def deltas(ctx, names):
    """bone -> 4x4 numpy delta (posed world @ rest world^-1)."""
    out = {}
    for n in names:
        out[n] = mat_np(pose_world(ctx.arm, n) @ rest_world(ctx.arm, n).inverted())
    return out


# ---------------------------------------------------------------- per joint geometry (= G3)
class JointGeo:
    """Slices (section ratio) and face region (self intersection, collapse) of one joint side."""

    def __init__(self, ctx, joint, x, V0):
        spec = JOINTS[joint]
        arm, topo = ctx.arm, ctx.topo
        self.joint, self.side = joint, x
        pv = ctx.piv[f"{joint}_{x}"]
        self.pivot = np.array(pv["co"], dtype=np.float64)
        rad = pv.get("radius")
        ring_names = [f"{joint}_{x}_{k}" for k in spec["rings"]]
        ring_idx = np.concatenate([np.array(ctx.rings[r]["verts"], dtype=np.int64) for r in ring_names])
        self.ring_names = ring_names
        (po, pa), (do, da) = [(side_name(o, x), side_name(a, x)) for o, a in (spec["prox"], spec["dist"])]
        self.owners = sorted({po, do})

        def bone_dir(n):
            b = arm.data.bones[n]
            return unit(np.array(arm.matrix_world @ b.tail_local) - np.array(arm.matrix_world @ b.head_local))

        def bone_x(n):
            return np.array(rest_world(arm, n).to_3x3().normalized().col[0])

        dp, dd = bone_dir(pa), bone_dir(da)
        dm = unit(dp + dd)
        centers = [V0[np.array(ctx.rings[r]["verts"], dtype=np.int64)].mean(axis=0) for r in ring_names]
        s_r = [float(np.dot(c - self.pivot, dm)) for c in centers]
        self.ring_s_mm = {r: mm(s) for r, s in zip(ring_names, s_r)}
        s0, s1 = min(s_r) - SLICE_MARGIN, max(s_r) + SLICE_MARGIN
        n = int(round((s1 - s0) / SLICE_STEP)) + 1
        self.slices = []
        for s in np.linspace(s0, s1, n).tolist():
            owner, abone, d = (po, pa, dp) if s < 0 else (do, da, dd)
            u = bone_x(abone)
            u = unit(u - np.dot(u, d) * d)
            v = np.cross(d, u)
            self.slices.append({"s": s, "owner": owner, "c": self.pivot + s * d, "n": d, "u": u, "v": v})
        self.phi = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False)
        tube_r = float(rad) if isinstance(rad, (int, float)) else 0.05
        self.R = float(np.linalg.norm(V0[ring_idx] - self.pivot, axis=1).max()) + tube_r
        body = topo.polys_of("body")
        cen = topo.poly_centroid(V0)
        self.region = body[np.linalg.norm(cen[body] - self.pivot, axis=1) <= self.R]
        self.region_tris = topo.tris_of_polys(self.region)
        self.region_sets = {int(p): set(topo.poly_verts[int(p)]) for p in self.region.tolist()}
        self.area0 = topo.poly_area(V0)[self.region]
        self.rest_r = None

    def radii(self, bvh, D=None):
        """Per slice: min in-plane ray hit distance (None if no ray hits), center inside flag."""
        out = []
        for sl in self.slices:
            c, u, v = sl["c"], sl["u"], sl["v"]
            if D is not None:
                M = D[sl["owner"]]
                c = M[:3, :3] @ c + M[:3, 3]
                u, v = M[:3, :3] @ u, M[:3, :3] @ v
            cv = Vector(c)
            best = None
            for ph in self.phi.tolist():
                dirv = Vector(math.cos(ph) * u + math.sin(ph) * v)
                hit = bvh.ray_cast(cv, dirv, RAY_MAX)
                if hit[0] is not None and (best is None or hit[3] < best):
                    best = hit[3]
            out.append((best, _inside(bvh, cv)))
        return out


class Region:
    """Whole-body face region (all body faces) for the report-only body metrics."""

    def __init__(self, topo, polys, V0):
        self.region = polys
        self.region_tris = topo.tris_of_polys(polys)
        self.region_sets = {int(p): set(topo.poly_verts[int(p)]) for p in polys.tolist()}
        self.area0 = topo.poly_area(V0)[polys]


def self_intersections(topo, geo, V):
    """(pair count, first pairs, sorted unique faces in intersecting pairs, all pairs)."""
    if not len(geo.region_tris):
        return 0, [], [], []
    tv = topo.tri_v[geo.region_tris]
    tp = topo.tri_poly[geo.region_tris]
    bvh = bvh_of(V, tv)
    pairs = set()
    for i, j in bvh.overlap(bvh):
        a, b = int(tp[i]), int(tp[j])
        if a == b:
            continue
        if geo.region_sets[a] & geo.region_sets[b]:
            continue
        pairs.add((min(a, b), max(a, b)))
    return len(pairs), sorted(pairs)[:LIST_CAP], sorted({f for pr in pairs for f in pr}), sorted(pairs)


def isect_cover(pairs):
    """Minimum face set covering every pair: (0, None) no pairs; (1, face) one face in every pair; ('>=2', None)."""
    if not pairs:
        return 0, None
    common = set(pairs[0])
    for pr in pairs[1:]:
        common &= set(pr)
        if not common:
            return ">=2", None
    return 1, min(common)


def collapsed(topo, geo, V):
    a1 = topo.poly_area(V)[geo.region]
    ratio = np.where(geo.area0 > 0, a1 / np.where(geo.area0 > 0, geo.area0, 1.0), np.inf)
    worst = int(np.argmin(ratio)) if len(ratio) else None
    return (int((ratio < COLLAPSE).sum()), rnd(ratio[worst], 4) if worst is not None else None,
            int(geo.region[worst]) if worst is not None else None)


# ---------------------------------------------------------------- G5.1 / G5.2 weights
def read_weights(mo):
    """list per vertex of {group name: weight}."""
    names = {vg.index: vg.name for vg in mo.vertex_groups}
    out = []
    for v in mo.data.vertices:
        out.append({names.get(g.group, f"#{g.group}"): float(g.weight) for g in v.groups})
    return out


def check_g51(ev, ctx, W):
    arm, mo = ctx.arm, ctx.mesh
    deform = sorted(b.name for b in arm.data.bones if b.use_deform)
    counts = np.array([sum(1 for w in d.values() if w > NONZERO) for d in W], dtype=np.int64)
    sums = np.array([sum(d.values()) for d in W], dtype=np.float64)
    over = np.nonzero(counts > MAX_INF)[0]
    badsum = np.nonzero(np.abs(sums - 1.0) > SUM_TOL)[0]
    forb = {}
    for nm in ("root", SOCKET):
        ws = [d[nm] for d in W if nm in d]
        forb[nm] = {"group_present": nm in mo.vertex_groups, "max_weight": max(ws) if ws else 0.0,
                    "n_verts_nonzero": int(sum(1 for w in ws if w > NONZERO))}
    vg_names = [vg.name for vg in mo.vertex_groups]
    not_deform = sorted(n for n in vg_names if n not in deform)
    mods = []
    for mod in mo.modifiers:
        if mod.type == "ARMATURE":
            mods.append({"name": mod.name, "object": mod.object.name if mod.object else None,
                         "use_vertex_groups": bool(mod.use_vertex_groups),
                         "use_bone_envelopes": bool(mod.use_bone_envelopes),
                         "use_deform_preserve_volume": bool(mod.use_deform_preserve_volume)})
    arm_ok = bool(mods) and all(m["object"] == ARM and m["use_vertex_groups"] for m in mods)
    canon = None
    if isinstance(ctx.canon, dict) and isinstance(ctx.canon.get("bones"), list):
        cb = {b["name"]: b for b in ctx.canon["bones"]}
        names = {b.name for b in arm.data.bones}
        mdiff = 0.0
        par_bad = []
        for n, b in cb.items():
            if n not in arm.data.bones:
                continue
            M = np.array(b["rest_matrix"], dtype=np.float64)
            mdiff = max(mdiff, float(np.abs(M - mat_np(arm.data.bones[n].matrix_local)).max()))
            p = arm.data.bones[n].parent
            if (p.name if p else None) != b.get("parent"):
                par_bad.append(n)
        canon = {"n_bones": len(names), "missing": sorted(set(cb) - names), "extra": sorted(names - set(cb)),
                 "parent_mismatch": par_bad, "max_rest_matrix_abs_diff": rnd(mdiff, 8)}
    unused = sorted(n for n in deform if n not in vg_names)
    measured = {
        "n_verts": len(W), "max_nonzero_per_vert": int(counts.max()) if len(counts) else None,
        "n_verts_over_4": int(len(over)), "over_4_examples": over[:LIST_CAP].tolist(),
        "sum_min": rnd(sums.min(), 7) if len(sums) else None, "sum_max": rnd(sums.max(), 7) if len(sums) else None,
        "n_verts_sum_off": int(len(badsum)), "sum_off_examples": badsum[:LIST_CAP].tolist(),
        "n_verts_no_weight": int((counts == 0).sum()),
        "forbidden_bones": forb,
        "vertex_groups": len(vg_names), "groups_not_deform_bone": not_deform,
        "deform_bones_without_group": unused,
        "armature_modifiers": mods,
        "skeleton_vs_canonical_info": canon,
    }
    ok = (len(over) == 0 and len(badsum) == 0 and all(f["max_weight"] <= NONZERO for f in forb.values())
          and not not_deform and arm_ok)
    ev.criterion("G5.1", measured,
                 f"per vertex: weights > {NONZERO:g} count <= {MAX_INF}; sum = 1 +-{SUM_TOL:g}; root and "
                 f"{SOCKET} weight 0 (<= {NONZERO:g}); vertex group names subset of deform bone names; "
                 f"every ARMATURE modifier object = {ARM} with vertex groups on (>= 1 modifier)",
                 ok, "weights from GOB_mesh.data vertex groups (all groups summed); skeleton_vs_canonical_info "
                 "is informational (canonical_skeleton.json) and not part of ok")


def check_g52(ev, ctx, W):
    topo = ctx.topo
    parts = dict(RIGID_FIXED)
    parts["belt"] = None
    res, ok_all = {}, True
    for part in parts:
        exp = ctx.belt["bones"] if part == "belt" else [parts[part]]
        vs = topo.verts_of(part)
        n_bad, min_exp, max_other, others, ex = 0, 1.0, 0.0, {}, []
        for i in vs.tolist():
            d = W[i]
            e = sum(d.get(b, 0.0) for b in exp)
            oth = {k: w for k, w in d.items() if k not in exp and w > 0.0}
            mo_ = max(oth.values()) if oth else 0.0
            if len(exp) == 1:
                bad = e < 1.0 - RIGID_TOL or mo_ > RIGID_TOL
            else:
                bad = mo_ > RIGID_TOL or abs(e - 1.0) > RIGID_TOL
            min_exp = min(min_exp, e)
            max_other = max(max_other, mo_)
            for k, w in oth.items():
                if w > RIGID_TOL:
                    others[k] = others.get(k, 0) + 1
            if bad:
                n_bad += 1
                if len(ex) < LIST_CAP:
                    ex.append(i)
        ok = len(vs) > 0 and n_bad == 0
        ok_all &= ok
        res[part] = {"expected_bones": exp, "n_verts": int(len(vs)), "n_bad": n_bad,
                     "min_expected_weight": rnd(min_exp, 7), "max_other_weight": rnd(max_other, 7),
                     "other_bones_nverts": others, "bad_examples": ex}
    mouth, m_ok = mouth_check(ctx, W)
    if "head" in res:
        res["head"]["mouth"] = mouth
    ok_all &= m_ok
    res["belt_decision"] = ctx.belt
    note = ("verts = vertices of faces with part_id of the part; Option A = one expected bone with weight >= "
            f"1-{RIGID_TOL:g} and every other weight <= {RIGID_TOL:g}; Option B (2 bones recorded) = weights "
            "outside the recorded bones <= tol and their sum = 1; head.mouth (HR2 2026-09-26) = vertices of "
            f"faces of material slots {list(MOUTH_SLOTS)} + vertices moved by shape key {MOUTH_KEY} (> "
            f"{MOUTH_MOVE_EPS * 1000:g} mm vs the reference key): all must be head-part vertices (faces of those "
            "slots part head) and 100 % DEF head (same rule as head); no mouth -> n 0, nothing judged")
    if ctx.dec_notes:
        note += "; " + "; ".join(n for n in ctx.dec_notes if "belt" in n)
    ev.criterion("G5.2", res,
                 "rigid parts 100 % on one bone: head->head, hand_x->hand_x, shoe_x->foot_x, belt->skin_decisions "
                 "belt bone (Option A) / recorded 2 bones (Option B)", ok_all, note)


def mouth_check(ctx, W):
    """G5.2 HR2: mouth vertices (mouth material slots + mouth_open key displacement) -> (report, ok)."""
    mo, topo = ctx.mesh, ctx.topo
    me = mo.data
    slots = [s.material.name if s.material else None for s in mo.material_slots]
    mi = np.empty(topo.npoly, dtype=np.int64)
    me.polygons.foreach_get("material_index", mi)
    sid = [k for k, n in enumerate(slots) if n in MOUTH_SLOTS]
    fm = np.isin(mi, sid)
    slot_v = set()
    for p in np.nonzero(fm)[0].tolist():
        slot_v.update(topo.poly_verts[p])
    key_v, key_info = set(), None
    key = me.shape_keys
    kb = key.key_blocks.get(MOUTH_KEY) if key is not None else None
    if kb is not None:
        ref = key.reference_key
        a = np.empty(topo.nv * 3, dtype=np.float32)
        b = np.empty(topo.nv * 3, dtype=np.float32)
        kb.data.foreach_get("co", a)
        (kb.relative_key or ref).data.foreach_get("co", b)
        d = np.linalg.norm((a - b).reshape(-1, 3).astype(np.float64), axis=1)
        key_v = set(np.nonzero(d > MOUTH_MOVE_EPS)[0].tolist())
        key_info = {"relative_key": (kb.relative_key or ref).name, "max_disp_mm": mm(d.max()) if len(d) else None}
    head_v = set(topo.verts_of("head").tolist())
    hid = (ctx.parts or {}).get("head")
    faces_not_head = int((fm & (topo.part != hid)).sum()) if topo.part is not None else int(fm.sum())
    mv = sorted(slot_v | key_v)
    not_head = [i for i in mv if i not in head_v]
    n_bad, min_e, max_o, ex = 0, 1.0, 0.0, []
    for i in mv:
        d = W[i]
        e = d.get("head", 0.0)
        mo_ = max([w for k, w in d.items() if k != "head" and w > 0.0], default=0.0)
        min_e, max_o = min(min_e, e), max(max_o, mo_)
        if e < 1.0 - RIGID_TOL or mo_ > RIGID_TOL:
            n_bad += 1
            if len(ex) < LIST_CAP:
                ex.append(i)
    rep = {"n_verts": len(mv), "n_slot_verts": len(slot_v), "n_key_moved_verts": len(key_v),
           "mouth_slots_found": [slots[k] for k in sid], "n_mouth_slot_faces": int(fm.sum()),
           "n_mouth_slot_faces_not_head_part": faces_not_head, "n_verts_not_head_part": len(not_head),
           "not_head_part_examples": not_head[:LIST_CAP], "shape_key": key_info,
           "n_bad": n_bad, "min_head_weight": rnd(min_e, 7) if mv else None,
           "max_other_weight": rnd(max_o, 7) if mv else None, "bad_examples": ex}
    return rep, bool(n_bad == 0 and not not_head and faces_not_head == 0)


# ---------------------------------------------------------------- G5.3 twist
def constraint_info(c):
    r = {"name": c.name, "type": c.type, "mute": bool(getattr(c, "mute", False)),
         "enabled": bool(getattr(c, "enabled", True)), "influence": rnd(c.influence, 5)}
    t = getattr(c, "target", None)
    r["target"] = t.name if t is not None else None
    r["subtarget"] = getattr(c, "subtarget", None)
    for k in ("owner_space", "target_space", "mix_mode", "euler_order"):
        if hasattr(c, k):
            r[k] = getattr(c, k)
    for k in ("use_x", "use_y", "use_z", "invert_x", "invert_y", "invert_z"):
        if hasattr(c, k):
            r[k] = bool(getattr(c, k))
    if c.type == "TRANSFORM":
        for k in ("map_from", "from_rotation_mode", "map_to", "map_to_x_from", "map_to_y_from", "map_to_z_from",
                  "mix_mode_rot", "to_euler_order", "use_motion_extrapolate"):
            if hasattr(c, k):
                v = getattr(c, k)
                r[k] = bool(v) if isinstance(v, bool) else v
        for k in ("from_min_y_rot", "from_max_y_rot", "to_min_x_rot", "to_max_x_rot", "to_min_y_rot",
                  "to_max_y_rot", "to_min_z_rot", "to_max_z_rot"):
            if hasattr(c, k):
                r[k] = rnd(math.degrees(getattr(c, k)), 4)
    return r


def _norm_ctype(t):
    if not isinstance(t, str):
        return None
    t = t.strip().upper().replace(" ", "_")
    return {"TRANSFORMATION": "TRANSFORM", "COPYROTATION": "COPY_ROTATION"}.get(t, t)


def _parent_rel_twist(arm, bone):
    """(twist deg, swing deg) of bone relative to its parent vs rest (swing-twist about local Y)."""
    par = arm.pose.bones[bone].parent.name
    rest_rel = rest_world(arm, par).inverted() @ rest_world(arm, bone)
    post_rel = pose_world(arm, par).inverted() @ pose_world(arm, bone)
    return twist_angle_deg((rest_rel.inverted() @ post_rel).to_3x3().normalized())


def static_twist(dec, cons):
    """Static constraint check for one twist bone (TRANSFORM contract, COPY_ROTATION old contract)."""
    src, axis, ratio, sign = dec["source"], dec["axis"], dec["ratio"], dec["sign"]
    want = _norm_ctype(dec.get("constraint"))
    active = [c for c in cons if not c["mute"] and c["enabled"] and c["influence"] > 0]
    on_src = [c for c in active if c["target"] == ARM and c["subtarget"] == src]
    typed = [c for c in on_src if (c["type"] == want if want else c["type"] in ("TRANSFORM", "COPY_ROTATION"))]
    st = {"expected_type": want or "COPY_ROTATION|TRANSFORM (constraint key missing)",
          "has_active_constraint": bool(active), "source_match": bool(on_src), "type_match": bool(typed)}
    checks = {}
    if typed:
        c = typed[0]
        st["checked_constraint"] = c["name"]
        ax = (axis or "Y").lower()
        if c["type"] == "COPY_ROTATION":
            checks["axis_match"] = (bool(c.get(f"use_{ax}")) and all(not c.get(f"use_{o}") for o in "xyz"
                                                                      if o != ax)) if axis else None
            checks["ratio_match"] = abs(c["influence"] - ratio) <= RATIO_TOL if ratio is not None else None
            checks["sign_match"] = ((c.get(f"invert_{ax}") is True) == (sign < 0)) if sign is not None else None
        else:
            fr, tr = c.get(f"from_min_{ax}_rot"), c.get(f"from_max_{ax}_rot")
            t0, t1 = c.get(f"to_min_{ax}_rot"), c.get(f"to_max_{ax}_rot")
            slope = (t1 - t0) / (tr - fr) if None not in (fr, tr, t0, t1) and abs(tr - fr) > 1e-9 else None
            icpt = (t0 - slope * fr) if slope is not None else None
            st["y_mapping"] = {"from_deg": [fr, tr], "to_deg": [t0, t1], "slope": rnd(slope, 6),
                               "intercept_deg": rnd(icpt, 4),
                               "expected_slope": rnd(sign * ratio, 6) if sign is not None and ratio is not None
                               else None,
                               "extrapolate": c.get("use_motion_extrapolate")}
            want_mode = dec.get("from_rotation_mode") or "SWING_TWIST_Y"
            checks["from_rotation_mode_match"] = c.get("from_rotation_mode") == want_mode
            checks["map_rotation_to_rotation"] = c.get("map_from") == "ROTATION" and c.get("map_to") == "ROTATION"
            checks["axis_match"] = c.get(f"map_to_{ax}_from") == ax.upper() if axis else None
            others = [c.get(f"to_{m}_{o}_rot") for o in "xyz" if o != ax for m in ("min", "max")]
            checks["other_axes_zero"] = all(v is not None and abs(v) <= 1e-4 for v in others)
            if slope is not None and sign is not None and ratio is not None:
                checks["ratio_sign_match"] = abs(slope - sign * ratio) <= SLOPE_TOL and abs(icpt) <= 1e-3
            else:
                checks["ratio_sign_match"] = False if slope is None else None
            for k in ("target_space", "owner_space"):
                checks[f"{k}_match"] = (c.get(k) == dec.get(k)) if dec.get(k) else None
            st["mix_mode_rot"] = {"constraint": c.get("mix_mode_rot"), "decision": dec.get("mix_mode_rot"),
                                  "match_info": (c.get("mix_mode_rot") == dec.get("mix_mode_rot"))
                                  if dec.get("mix_mode_rot") else None}
    st["checks"] = checks
    ok = bool(typed) and all(v is not False for v in checks.values())
    return st, ok


def check_g53(ev, ctx):
    arm = ctx.arm
    drivers = []
    if arm.animation_data is not None:
        drivers = [fc.data_path for fc in arm.animation_data.drivers]
    res, ok_all = {}, True
    for bone in TWIST_BONES:
        dec = ctx.twist_dec[bone]
        pb = arm.pose.bones[bone]
        cons = [constraint_info(c) for c in pb.constraints]
        src, ratio, sign = dec["source"], dec["ratio"], dec["sign"]
        static, static_ok = static_twist(dec, cons)
        drv = [p for p in drivers if f'"{bone}"' in p]
        # (a) behaviour: source local Y +90, twist bone axial rotation relative to its parent
        beh, beh_ok = {"source": src}, False
        # (b) twist-free swing of the source about local (X+Z)/sqrt2
        swg, swg_ok = {"source": src}, False
        if src in arm.pose.bones and arm.pose.bones[bone].parent is not None:
            reset_pose(arm)
            update()
            par = arm.pose.bones[bone].parent.name
            set_rot(ctx, src, [0.0, TWIST_TEST_DEG, 0.0], "XYZ")
            update()
            tw_rel, sw_rel = _parent_rel_twist(arm, bone)
            d_w = (rest_world(arm, bone).inverted() @ pose_world(arm, bone)).to_3x3().normalized()
            tw_w, _ = twist_angle_deg(d_w)
            d_s = (rest_world(arm, src).inverted() @ pose_world(arm, src)).to_3x3().normalized()
            tw_s, _ = twist_angle_deg(d_s)
            reset_pose(arm)
            update()
            if ratio is not None:
                if sign is not None:
                    exp = sign * ratio * TWIST_TEST_DEG
                    err = abs(tw_rel - exp)
                else:
                    exp = None
                    err = abs(abs(tw_rel) - ratio * TWIST_TEST_DEG)
                beh_ok = err <= TWIST_TOL_DEG
            else:
                exp, err = None, None
            beh.update({"parent": par, "source_rot_local_Y_deg": TWIST_TEST_DEG,
                        "source_world_twist_deg": rnd(tw_s, 3),
                        "twist_rel_parent_deg": rnd(tw_rel, 3), "swing_rel_parent_deg": rnd(sw_rel, 3),
                        "twist_world_deg": rnd(tw_w, 3),
                        "world_fraction_of_source": rnd(tw_w / tw_s, 4) if abs(tw_s) > 1e-6 else None,
                        "expected_rel_deg": rnd(exp, 3) if exp is not None else None,
                        "abs_err_deg": rnd(err, 3) if err is not None else None,
                        "magnitude_only": sign is None})
            # (b)
            ang = SWING_TEST_DEG.get(src[:-2] if src[-2:] in ("_l", "_r") else src, 90.0)
            spb = arm.pose.bones[src]
            if src not in ctx.saved_modes:
                ctx.saved_modes[src] = spb.rotation_mode
            spb.rotation_mode = "QUATERNION"
            spb.rotation_quaternion = Quaternion(Vector((1.0, 0.0, 1.0)).normalized(), math.radians(ang))
            update()
            tw_b, sw_b = _parent_rel_twist(arm, bone)
            d_s = (rest_world(arm, src).inverted() @ pose_world(arm, src)).to_3x3().normalized()
            tw_sb, sw_sb = twist_angle_deg(d_s)
            reset_pose(arm)
            update()
            swg_ok = abs(tw_b) <= TWIST_TOL_DEG
            swg.update({"parent": par, "swing_axis_local": "(X+Z)/sqrt2", "swing_deg": ang,
                        "source_twist_deg": rnd(tw_sb, 3), "source_swing_deg": rnd(sw_sb, 3),
                        "twist_rel_parent_deg": rnd(tw_b, 3), "swing_rel_parent_deg": rnd(sw_b, 3)})
        else:
            beh["error"] = swg["error"] = f"source {src!r} not a pose bone or {bone} has no parent"
        ok = static_ok and beh_ok
        ok_all &= ok
        res[bone] = {"decision": {k: dec.get(k) for k in ("key", "source", "constraint", "from_rotation_mode",
                                                            "axis", "ratio", "sign", "target_space",
                                                            "owner_space", "mix_mode_rot", "space", "effect")},
                     "constraints": cons, "drivers": drv, "static": static, "static_ok": static_ok,
                     "behaviour": beh, "behaviour_ok": beh_ok, "ok": ok}
        ev.criterion(f"G5.3b_{bone}", swg,
                     f"twist-free swing of {src} about local (X+Z)/sqrt2 ({swg.get('swing_deg')} deg): {bone} "
                     f"axial rotation relative to its parent <= {TWIST_TOL_DEG:g} deg",
                     swg_ok, "source rotation_mode QUATERNION (in memory); twist = swing-twist angle about local Y "
                             "of (rest parent->bone)^-1 (posed parent->bone); source_twist_deg = the source's own "
                             "swing-twist twist (0 by construction)")
    note = ("static: an active (unmuted, influence > 0) constraint of the skin_decisions 'constraint' type "
            "targets GOB_rig/<source>. TRANSFORM: from_rotation_mode = decision (default SWING_TWIST_Y), map "
            "ROTATION->ROTATION, map_to_<axis>_from = <axis>, <axis> mapping slope (to range / from range) = "
            f"sign*ratio +-{SLOPE_TOL:g} with intercept 0, other to-axes 0, target/owner space = decision; "
            "mix_mode_rot recorded (info). COPY_ROTATION (old contract): use_<axis> alone, influence = ratio, "
            "invert_<axis> = (sign < 0). behaviour: "
            f"source rotation_mode XYZ, Euler Y = {TWIST_TEST_DEG:g}, twist = swing-twist angle about local Y of "
            "(rest parent->bone)^-1 (posed parent->bone); sign = direction of the twist bone's parent-relative "
            "Y rotation relative to the source rotation; world_fraction = twist bone world axial rotation / "
            "source world axial rotation (info); swing leak test in G5.3b_<bone>")
    tw_notes = [n for n in ctx.dec_notes if "belt" not in n]
    if tw_notes:
        note += "; " + "; ".join(tw_notes)
    ev.criterion("G5.3", res,
                 "each twist bone: constraint of the recorded type with skin_decisions source / from_rotation_mode "
                 "/ spaces / axis mapping = sign*ratio; source Y +90 deg -> twist bone axial rotation relative to "
                 f"parent = sign*ratio*90 deg +-{TWIST_TOL_DEG:g} deg",
                 ok_all, note)


# ---------------------------------------------------------------- renders
def cutout_render(ctx, V, center, half, depth, view, hidden_parts, out_png):
    topo = ctx.topo
    d, right, up = goblib._view_basis(view)
    d, right, up = np.array(d), np.array(right), np.array(up)
    cen = topo.poly_centroid(V) - center
    inbox = ((np.abs(cen @ right) <= half) & (np.abs(cen @ up) <= half) & (np.abs(cen @ d) <= depth))
    hidden = np.zeros(topo.npoly, dtype=bool)
    for p in hidden_parts:
        hidden[topo.polys_of(p)] = True
    body = np.zeros(topo.npoly, dtype=bool)
    body[topo.polys_of("body")] = True
    groups = {"_g5_cell_body": np.nonzero(inbox & body)[0],
              "_g5_cell_rigid": np.nonzero(inbox & ~body & ~hidden)[0]}
    objs, meshes, colors = [], [], {}
    try:
        for name, polys in groups.items():
            if not len(polys):
                continue
            faces = [topo.poly_verts[int(p)] for p in polys.tolist()]
            used = np.unique(np.concatenate([np.array(f) for f in faces]))
            remap = {int(o): i for i, o in enumerate(used.tolist())}
            me = bpy.data.meshes.new(name)
            me.from_pydata(V[used].tolist(), [], [[remap[i] for i in f] for f in faces])
            me.update()
            meshes.append(me)
            ob = bpy.data.objects.new(name, me)
            bpy.context.scene.collection.objects.link(ob)
            objs.append(ob)
            colors[ob.name] = BODY_COLOR if name.endswith("body") else RIGID_COLOR
        if not objs:
            return None
        corners = [center + a * half * right + b * half * up + c * depth * d
                   for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
        C = np.array(corners)
        bbox = (tuple(C.min(axis=0)), tuple(C.max(axis=0)))
        goblib.ortho_render(objs, view, out_png, res_h=CELL_PX, frame_bbox=bbox, wire=True, colors=colors)
    finally:
        for ob in objs:
            bpy.data.objects.remove(ob, do_unlink=True)
        for me in meshes:
            bpy.data.meshes.remove(me)
    return out_png


def render_pose_cells(ctx, p, V, cell_dir):
    g = p["group"]
    out = {}
    if g == "full":
        for view in FULL_VIEWS:
            png = cell_dir / f"{p['id']}_{view}.png"
            goblib.ortho_render([ctx.mesh, ctx.club], view, png, res_h=FULL_PX, frame_bbox=FULL_BBOX,
                                wire=False, colors={ctx.mesh.name: BODY_COLOR, ctx.club.name: RIGID_COLOR})
            out[view] = png
        return out
    view = p.get("view") or "front"
    if view not in goblib._VIEWS:
        view = "front"
    if g == "spine_head":
        key = next((b for b in p["rot"] if b in ctx.piv), "spine_01")
        center = np.array(ctx.piv[key]["co"], dtype=np.float64)
        hidden = ()
    else:
        center = np.array(ctx.piv[f"{g}_{p['side']}"]["co"], dtype=np.float64)
        hidden = HIDDEN_PARTS
    png = cell_dir / f"{p['id']}.png"
    r = cutout_render(ctx, V, center, CELL_HALF[g], CELL_DEPTH[g], view, hidden, png)
    out["cell"] = r
    return out


def load_rgba(png):
    img = bpy.data.images.load(str(Path(png).resolve()), check_existing=False)
    try:
        w, h = img.size
        buf = np.empty(w * h * img.channels, dtype=np.float32)
        img.pixels.foreach_get(buf)
        px = buf.reshape(h, w, img.channels)
    finally:
        bpy.data.images.remove(img)
    if px.shape[2] == 3:
        px = np.concatenate([px, np.ones((h, w, 1), np.float32)], axis=2)
    return px[::-1]  # row 0 = top


def compose_sheet(cells, out_png):
    """cells: list of rows of png paths (None = empty).  Alpha over white, CELL_GAP px gaps."""
    imgs = [[load_rgba(p) if p is not None and Path(p).exists() else None for p in row] for row in cells]
    ch = max((im.shape[0] for row in imgs for im in row if im is not None), default=CELL_PX)
    cw = max((im.shape[1] for row in imgs for im in row if im is not None), default=CELL_PX)
    nr, nc = len(imgs), max(len(r) for r in imgs)
    H = nr * ch + (nr + 1) * CELL_GAP
    W = nc * cw + (nc + 1) * CELL_GAP
    sheet = np.full((H, W, 4), 1.0, dtype=np.float32)
    sheet[:, :, :3] = 0.55
    for i, row in enumerate(imgs):
        for j, im in enumerate(row):
            y0, x0 = CELL_GAP + i * (ch + CELL_GAP), CELL_GAP + j * (cw + CELL_GAP)
            sheet[y0:y0 + ch, x0:x0 + cw, :3] = 1.0
            if im is None:
                continue
            a = im[:, :, 3:4]
            hh, ww = im.shape[:2]
            sheet[y0:y0 + hh, x0:x0 + ww, :3] = im[:, :, :3] * a + (1.0 - a)
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img = bpy.data.images.new("_g5_sheet", W, H, alpha=True)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(sheet[::-1]).ravel())
        img.filepath_raw = str(out_png)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)
    return out_png


def sheet_layout(ctx, group, cells):
    ps = [p for p in ctx.poses if p["group"] == group]
    if group == "full":
        rows = [p["id"] for p in ps]
        cols = list(FULL_VIEWS)
        grid = [[cells.get(p["id"], {}).get(v) for v in cols] for p in ps]
        return grid, rows, cols
    rows, cols = [], []
    for p in ps:
        rk = f"{p.get('kind', p['id'])}_{p['side']}"
        if rk not in rows:
            rows.append(rk)
        c = str(p.get("col", p["id"]))
        if c not in cols:
            cols.append(c)
    if all(num_key(c)[0] == 0 for c in cols):
        cols = sorted(cols, key=num_key)
    at = {(f"{p.get('kind', p['id'])}_{p['side']}", str(p.get("col", p["id"]))): p["id"] for p in ps}
    grid = [[cells.get(at.get((r, c)), {}).get("cell") for c in cols] for r in rows]
    return grid, rows, cols


# ---------------------------------------------------------------- measurement (G5.4 .. G5.6)
def measure(ctx, cell_dir):
    arm, mo, topo = ctx.arm, ctx.mesh, ctx.topo
    apply_pose(ctx, None)
    V0 = eval_verts(mo)
    if len(V0) != topo.nv:
        raise RuntimeError(f"evaluated GOB_mesh has {len(V0)} verts, obj.data has {topo.nv}")
    C0 = eval_verts(ctx.club)
    body_polys = topo.polys_of("body")
    body_tris = topo.tris_of_polys(body_polys)
    bvh0 = bvh_of(V0, topo.tri_v[body_tris])
    vol0 = shell_volume(V0, topo.tri_v[body_tris])

    geos = {(j, x): JointGeo(ctx, j, x, V0) for j in JOINTS for x in SIDES}
    for g in geos.values():
        g.rest_r = g.radii(bvh0)
    whole = Region(topo, body_polys, V0)

    rigid = {p: [b] for p, b in RIGID_FIXED.items()}
    rigid["belt"] = list(ctx.belt["bones"])
    rigid_v = {p: topo.verts_of(p) for p in rigid}
    end_info = {}
    for j, (rts, part_t, _) in END_RING.items():
        for x in SIDES:
            end_info[(j, x)] = ({rt.format(s=x): np.array(ctx.rings[rt.format(s=x)]["verts"], dtype=np.int64)
                                 for rt in rts},
                                topo.tris_of_polys(topo.polys_of(part_t.format(s=x))), part_t.format(s=x))

    def exposure(V, key):
        rings, pt, _ = end_info[key]
        if not len(pt):
            return {r: {"min_depth_mm": None, "n_below_0": None} for r in rings}
        bvh = bvh_of(V, topo.tri_v[pt])
        out = {}
        for r, rv in rings.items():
            dep = signed_depth(bvh, V[rv])
            out[r] = {"min_depth_mm": mm(dep.min()), "n_below_0": int((dep < DEPTH_MIN).sum())}
        return out

    res = {"rest": {}, "poses": {}}
    res["rest"]["volume_m3"] = vol0
    res["rest"]["exposure"] = {f"{j}_{x}": exposure(V0, (j, x)) for (j, x) in end_info}
    n, ex, fl, _ = self_intersections(topo, whole, V0)
    res["rest"]["body_self_isect"] = {"pairs": n, "examples": ex, "n_faces": len(fl)}
    res["rest"]["joint_self_isect"] = {}
    for (j, x), g in geos.items():
        n, _, fl, _ = self_intersections(topo, g, V0)
        res["rest"]["joint_self_isect"][f"{j}_{x}"] = {"pairs": n, "n_faces": len(fl), "faces": fl}
    res["rest"]["club_origin_to_socket_head_mm"] = mm(np.linalg.norm(
        np.array(ctx.club.matrix_world.translation) - np.array(rest_world(arm, SOCKET).translation)))
    ya = np.array(ctx.club.matrix_world.to_3x3().normalized().col[1])
    yb = np.array(rest_world(arm, SOCKET).to_3x3().normalized().col[1])
    res["rest"]["club_Y_vs_socket_Y_deg"] = rnd(math.degrees(math.atan2(np.linalg.norm(np.cross(ya, yb)),
                                                                        float(np.dot(ya, yb)))), 4)
    res["club_parent"] = {"parent": ctx.club.parent.name if ctx.club.parent else None,
                          "parent_type": ctx.club.parent_type, "parent_bone": ctx.club.parent_bone,
                          "constraints": [c.type for c in ctx.club.constraints]}

    owners_all = sorted({o for g in geos.values() for o in g.owners})
    rig_bones = sorted({b for bs in rigid.values() for b in bs} | {SOCKET})
    cells = {}
    for p in ctx.poses:
        apply_pose(ctx, p)
        V = eval_verts(mo)
        C = eval_verts(ctx.club)
        D = deltas(ctx, set(owners_all) | set(rig_bones))
        r = {"tier": p["tier"], "group": p["group"], "side": p["side"]}
        # G5.4
        resid = {}
        for part, bones in rigid.items():
            vs = rigid_v[part]
            if len(bones) != 1 or not len(vs):
                resid[part] = None
                continue
            M = D[bones[0]]
            e = np.linalg.norm(V[vs] - (V0[vs] @ M[:3, :3].T + M[:3, 3]), axis=1)
            resid[part] = mm(e.max())
        M = D[SOCKET]
        resid["club"] = mm(np.linalg.norm(C - (C0 @ M[:3, :3].T + M[:3, 3]), axis=1).max())
        r["rigid_resid_mm"] = resid
        # G5.6
        vol = shell_volume(V, topo.tri_v[body_tris])
        r["volume_change_pct"] = rnd((vol / vol0 - 1.0) * 100.0, 4) if vol0 else None
        # G5.5
        bvh = bvh_of(V, topo.tri_v[body_tris])
        joints = {}
        for key in p["joint_sides"]:
            g = geos[key]
            rp = g.radii(bvh, D)
            best, n_out, n_miss = None, 0, 0
            for sl, (r0, _), (r1, ins) in zip(g.slices, g.rest_r, rp):
                n_out += 0 if ins else 1
                if r0 is None or r1 is None:
                    n_miss += 1
                    continue
                ratio = r1 / r0
                if best is None or ratio < best[0]:
                    best = (ratio, sl["s"], r1, r0, sl["owner"])
            ni, exi, fli, api = self_intersections(topo, g, V)
            cov, cf = isect_cover(api)
            nc, mar, maf = collapsed(topo, g, V)
            joints[f"{key[0]}_{key[1]}"] = {
                "min_ratio": rnd(best[0], 4) if best else None,
                "worst_slice_s_mm": mm(best[1]) if best else None,
                "posed_min_r_mm": mm(best[2]) if best else None, "rest_min_r_mm": mm(best[3]) if best else None,
                "worst_slice_owner": best[4] if best else None, "n_slices_no_hit": n_miss,
                "n_centers_outside_body": n_out,
                "self_isect_pairs": ni, "self_isect_examples": exi, "self_isect_n_faces": len(fli),
                "self_isect_faces": fli, "isect_cover": cov, "isect_common_face": cf,
                "n_collapsed": nc, "min_area_ratio": mar, "min_area_face": maf}
        r["joints"] = joints
        r["exposure"] = {f"{j}_{x}": exposure(V, (j, x)) for (j, x) in p["end_sides"]}
        nb, exb, flb, _ = self_intersections(topo, whole, V)
        ncb, marb, mafb = collapsed(topo, whole, V)
        r["body"] = {"self_isect_pairs": nb, "self_isect_examples": exb, "self_isect_n_faces": len(flb),
                     "n_collapsed": ncb,
                     "min_area_ratio": marb, "min_area_face": mafb}
        res["poses"][p["id"]] = r
        # G5.7 cells
        cells[p["id"]] = render_pose_cells(ctx, p, V, cell_dir)
        jr = [v["min_ratio"] for v in joints.values() if v["min_ratio"] is not None]
        print(f"[{GATE}/{CHECKER}] pose {p['id']} ({p['tier']}): min_ratio={min(jr) if jr else None} "
              f"isect={sum(v['self_isect_pairs'] for v in joints.values())} "
              f"collapsed={sum(v['n_collapsed'] for v in joints.values())} "
              f"vol={r['volume_change_pct']}% resid_max_mm="
              f"{max((v for v in resid.values() if isinstance(v, (int, float))), default=None)}")
        sys.stdout.flush()
    apply_pose(ctx, None)
    return res, geos, cells


def _min(vals):
    v = [x for x in vals if isinstance(x, (int, float))]
    return min(v) if v else None


def _max(vals):
    v = [x for x in vals if isinstance(x, (int, float))]
    return max(v) if v else None


def write_criteria(ev, ctx, res, geos):
    P = res["poses"]
    # G5.4
    parts = list(RIGID_FIXED) + ["belt", "club"]
    m4 = {}
    ok4 = True
    for part in parts:
        vals = {pid: r["rigid_resid_mm"][part] for pid, r in P.items()}
        num = {k: v for k, v in vals.items() if isinstance(v, (int, float))}
        if not num:
            m4[part] = {"max_mm": None, "note": "not rigid (Option B) or no vertices"}
            if part != "belt" or len(ctx.belt["bones"]) == 1:
                ok4 = False
            continue
        worst = max(num, key=num.get)
        m4[part] = {"max_mm": num[worst], "worst_pose": worst}
        ok4 &= num[worst] < RESID_MAX * 1000.0
    m4["per_pose"] = {pid: r["rigid_resid_mm"] for pid, r in P.items()}
    m4["club_parent"] = res["club_parent"]
    m4["club_rest"] = {"origin_to_socket_head_mm": res["rest"]["club_origin_to_socket_head_mm"],
                       "club_Y_vs_socket_Y_deg": res["rest"]["club_Y_vs_socket_Y_deg"]}
    ev.criterion("G5.4", m4, f"all {len(P)} ROM poses: rigid part vertex residual vs the rigid transform of its "
                             f"bone < {RESID_MAX * 1000:g} mm (head, hand_x, shoe_x->foot_x, belt->belt bone); "
                             f"GOB_club vs {SOCKET} < {RESID_MAX * 1000:g} mm",
                 ok4, "residual = |posed evaluated vertex - (posed world @ rest world^-1 of the bone) rest vertex|, "
                      "max over the part's vertices; club: evaluated GOB_club world vertices; club_parent / "
                      "club_rest are informational; belt Option B is not rigid -> not measured")
    # G5.5 per joint side
    fails = []
    pose_by_id = {p["id"]: p for p in ctx.poses}

    def ratio_min(pid):
        p = pose_by_id.get(pid, {})
        if p.get("kind") in ("wrist_twist", "ankle"):
            return RATIO_MIN
        try:
            return RATIO_MIN_60 if abs(float(p.get("col"))) == 60.0 else RATIO_MIN
        except (TypeError, ValueError):
            return RATIO_MIN

    th_j = (f"tier=judged poses only: section ratio >= {RATIO_MIN} (30 deg poses, wrist twist 90, ankle 30) / "
            f">= {RATIO_MIN_60} (60 deg poses); collapsed faces (< {COLLAPSE:.0%} rest area) = 0; joint-region "
            f"self intersection: 모든 교차 쌍을 덮는 최소 면 집합 <= {ISECT_COVER_MAX} (+ [U] visual check); "
            f"report poses recorded only ({DRAFT})")
    for j in JOINTS:
        for x in SIDES:
            key = f"{j}_{x}"
            g = geos[(j, x)]
            poses = {pid: dict(r["joints"][key], tier=r["tier"]) for pid, r in P.items() if key in r["joints"]}
            jd = {k: v for k, v in poses.items() if v["tier"] == "judged"}
            ok = bool(jd)
            for pid, v in jd.items():
                rmin = ratio_min(pid)
                v["ratio_threshold"] = rmin
                bad = []
                if v["min_ratio"] is None or v["min_ratio"] < rmin:
                    bad.append(f"ratio {v['min_ratio']} < {rmin}")
                if not isinstance(v["isect_cover"], int) or v["isect_cover"] > ISECT_COVER_MAX:
                    bad.append(f"isect cover {v['isect_cover']} (pairs {v['self_isect_pairs']}, "
                               f"faces {v['self_isect_n_faces']})")
                if v["n_collapsed"] != 0:
                    bad.append(f"collapsed {v['n_collapsed']}")
                v["judged_ok"] = not bad
                if bad:
                    ok = False
                    fails.append(f"{pid}/{key}: " + ", ".join(bad))
            rp = {k: v for k, v in poses.items() if v["tier"] == "report"}
            meas = {"judged_min_ratio": _min(v["min_ratio"] for v in jd.values()),
                    "judged_max_self_isect_pairs": _max(v["self_isect_pairs"] for v in jd.values()),
                    "judged_max_self_isect_faces": _max(v["self_isect_n_faces"] for v in jd.values()),
                    "judged_isect_cover": {k: v["isect_cover"] for k, v in jd.items()},
                    "judged_max_collapsed": _max(v["n_collapsed"] for v in jd.values()),
                    "report_min_ratio": _min(v["min_ratio"] for v in rp.values()),
                    "report_max_self_isect_pairs": _max(v["self_isect_pairs"] for v in rp.values()),
                    "report_max_self_isect_faces": _max(v["self_isect_n_faces"] for v in rp.values()),
                    "report_max_collapsed": _max(v["n_collapsed"] for v in rp.values()),
                    "judged_poses": sorted(jd), "rest_self_isect": res["rest"]["joint_self_isect"][key],
                    "rings_s_mm": g.ring_s_mm, "slice_s_mm": [mm(g.slices[0]["s"]), mm(g.slices[-1]["s"])],
                    "rest_min_r_mm": [mm(r) if r is not None else None for r, _ in g.rest_r],
                    "region": {"radius_mm": mm(g.R), "n_faces": int(len(g.region))},
                    "poses": poses}
            ev.criterion(f"G5.5_{key}", meas, th_j, ok,
                         f"G3 definitions: slices every {SLICE_STEP * 1000:.1f} mm over the ring span "
                         f"+-{SLICE_MARGIN * 1000:.0f} mm, {N_RAYS} in-plane rays from the axis point (body faces, "
                         f"max {RAY_MAX} m), plane moved with the owner bone; self intersection = BVHTree.overlap "
                         "of region body triangles, same polygon / shared vertex pairs dropped, distinct polygon "
                         "pairs; self_isect_n_faces / self_isect_faces = unique polygons in those pairs; isect_cover "
                         "= minimum face set covering every pair (0 = no pair, 1 = isect_common_face is in every "
                         "pair, '>=2' = no common face; exact size above 1 not computed); region = "
                         "body faces with rest centroid within max ring radius + tube radius of the pivot; ratio "
                         "threshold per pose = |pose angle| 60 -> 0.75, else 0.8 (ratio_threshold); poses measuring "
                         "this joint side = rom_poses joint field ('all' = every joint side)")
    th_e = (f"tier=judged poses of the same limb side: `_end` ring min signed depth inside the posed rigid shell "
            f">= {DEPTH_MIN * 1000:g} mm; `_1` ring min depth >= its rest min depth - {END1_TOL * 1000:g} mm (no worse "
            f"than rest); report poses recorded only ({DRAFT})")
    for j, (rts, part_t, limb) in END_RING.items():
        for x in SIDES:
            key = f"{j}_{x}"
            r_end, r_1 = rts[0].format(s=x), rts[1].format(s=x)
            rest = res["rest"]["exposure"][key]
            rest_1 = rest[r_1]["min_depth_mm"]
            thr = {r_end: DEPTH_MIN * 1000,
                   r_1: (rest_1 - END1_TOL * 1000) if isinstance(rest_1, (int, float)) else None}
            poses = {pid: dict(r["exposure"][key], tier=r["tier"]) for pid, r in P.items() if key in r["exposure"]}
            jd = {k: v for k, v in poses.items() if v["tier"] == "judged"}
            rp = {k: v for k, v in poses.items() if v["tier"] == "report"}
            ok = bool(jd)
            for pid, v in jd.items():
                bad = []
                for rn in (r_end, r_1):
                    d = v[rn]["min_depth_mm"]
                    if not isinstance(d, (int, float)) or thr[rn] is None or d < thr[rn]:
                        bad.append(f"{rn} {d} < {rnd(thr[rn], 4) if thr[rn] is not None else None}")
                v["judged_ok"] = not bad
                if bad:
                    ok = False
                    fails.append(f"{pid}/{key} exposure: " + ", ".join(bad))

            def mn(dd):
                return {rt.format(s=x): _min(v[rt.format(s=x)]["min_depth_mm"] for v in dd.values()) for rt in rts}
            ev.criterion(f"G5.5_exposure_{key}",
                         {"judged_min_depth_mm": mn(jd), "report_min_depth_mm": mn(rp),
                          "rest": rest, "threshold_mm": {k: rnd(v, 4) for k, v in thr.items()},
                          "part": part_t.format(s=x), "judged_poses": sorted(jd), "poses": poses},
                         th_e, ok,
                         "depth = BVH nearest distance to the posed part faces, + inside by ray parity (majority "
                         f"of 3 fixed directions); measured for every pose with a {limb} joint of side {x} and for "
                         "'all' poses; 'rest' = rest-pose baseline (per ring min depth and count below 0)")
    body = {pid: dict(r["body"], tier=r["tier"]) for pid, r in P.items()}
    ev.criterion("G5.5_body", {"max_self_isect": _max(v["self_isect_pairs"] for v in body.values()),
                               "max_self_isect_faces": _max(v["self_isect_n_faces"] for v in body.values()),
                               "max_collapsed": _max(v["n_collapsed"] for v in body.values()),
                               "rest_self_isect": res["rest"]["body_self_isect"], "poses": body},
                 "report only: whole-body (all body faces) self-intersection pairs and collapsed faces per pose",
                 True, "same overlap / collapse rules as the joint regions over every body face; covers the "
                       "spine_head and full poses outside the limb joint regions; not part of any ok")
    ev.criterion("G5.5", {"n_poses": len(P), "n_judged": sum(1 for r in P.values() if r["tier"] == "judged"),
                          "judged_failures": fails},
                 f"summary of G5.5_<joint>_<side> and G5.5_exposure_<joint>_<side> over tier=judged poses "
                 f"(ratio >= {RATIO_MIN} at 30 deg / wrist twist 90 / ankle 30, >= {RATIO_MIN_60} at 60 deg; collapse "
                 f"0; self intersection 모든 교차 쌍을 덮는 최소 면 집합 <= {ISECT_COVER_MAX}; `_end` depth >= 0, `_1` depth >= rest - "
                 f"{END1_TOL * 1000:g} mm; {DRAFT})",
                 not fails and any(r["tier"] == "judged" for r in P.values()),
                 "judged = rom_poses.json tier (G3 carry-over: shoulder raise/forward, elbow, knee, hip flexion "
                 "30/60, wrist twist +90, ankle dorsiflexion 30)")
    # G5.6
    vols = {pid: r["volume_change_pct"] for pid, r in P.items()}
    worst = max(vols, key=lambda k: abs(vols[k]) if vols[k] is not None else float("inf")) if vols else None
    ok6 = bool(vols) and all(v is not None and abs(v) <= VOL_MAX * 100.0 for v in vols.values())
    ev.criterion("G5.6", {"max_abs_change_pct": abs(vols[worst]) if worst and vols[worst] is not None else None,
                          "worst_pose": worst, "rest_volume_cm3": rnd(res["rest"]["volume_m3"] * 1e6, 3),
                          "poses_pct": vols},
                 f"every ROM pose: |body shell volume / rest - 1| <= {VOL_MAX:.0%}", ok6,
                 "volume = sum over part_id=body triangles of a.(b x c)/6 (divergence theorem, evaluated world "
                 "coordinates); all poses, both tiers")


def blocked_all(ev, reasons):
    r = "; ".join(x for x in reasons if x)
    ev.criterion("G5.1", r, "weights: <= 4 nonzero, sum 1, root/socket 0, groups = deform bones, modifier", False)
    ev.criterion("G5.2", r, "rigid parts 100 % one bone", False)
    ev.criterion("G5.3", r, "twist constraints + behaviour", False)
    for tb in TWIST_BONES:
        ev.criterion(f"G5.3b_{tb}", r, f"twist-free swing: axial rotation <= {TWIST_TOL_DEG:g} deg", False)
    ev.criterion("G5.4", r, "rigid residual < 1 mm (all ROM poses), club vs socket", False)
    for j in JOINTS:
        for x in SIDES:
            ev.criterion(f"G5.5_{j}_{x}", r, f"judged: ratio >= {RATIO_MIN} (30) / >= {RATIO_MIN_60} (60), 모든 교차 쌍을 덮는 최소 면 집합 <= {ISECT_COVER_MAX}, collapse 0 ({DRAFT})", False)
    for j in END_RING:
        for x in SIDES:
            ev.criterion(f"G5.5_exposure_{j}_{x}", r, f"judged: _end depth >= 0, _1 depth >= rest - {END1_TOL * 1000:g} mm ({DRAFT})", False)
    ev.criterion("G5.5_body", r, "report only", False)
    ev.criterion("G5.5", r, f"judged summary ({DRAFT})", False)
    ev.criterion("G5.6", r, f"|volume change| <= {VOL_MAX:.0%}", False)
    ev.criterion("G5.7", r, "[U] 8 sheets written", False)


# ---------------------------------------------------------------- main
def _load_json(path, ctx, key, required=True):
    if not path.exists():
        if required:
            _STATE["missing"].append(rel(path))
            ctx.miss[key] = f"missing input: {rel(path)}"
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def restore(ctx):
    if ctx.arm is None:
        return
    reset_pose(ctx.arm)
    for b, mode in ctx.saved_modes.items():
        ctx.arm.pose.bones[b].rotation_mode = mode
    update()


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ctx = Ctx()
    outdir = goblib.INSPECT / GATE

    if not BLEND.exists():
        _STATE["missing"].append(rel(BLEND))
        ctx.miss["blend"] = f"missing input: {rel(BLEND)}"
    else:
        loaded = bpy.data.filepath
        if not loaded or Path(loaded).resolve() != BLEND.resolve():
            print(f"[{GATE}/{CHECKER}] opening {rel(BLEND)} (loaded: {loaded!r})")
            bpy.ops.wm.open_mainfile(filepath=str(BLEND))
        ev.add_input(BLEND)
    ev.add_stage_inputs("s05")
    if ROM_JSON.exists():
        ev.add_input(ROM_JSON)

    ctx.parts = _load_json(PARTS_JSON, ctx, "parts")
    ctx.loops = _load_json(LOOPS_JSON, ctx, "loops")
    ctx.pivots = _load_json(PIVOTS_JSON, ctx, "pivots")
    ctx.rom = _load_json(ROM_JSON, ctx, "rom")
    ctx.dec = _load_json(SKIN_JSON, ctx, "skin_decisions")
    ctx.canon = _load_json(CANON_JSON, ctx, "canon", required=False)

    try:
        if ctx.m():
            blocked_all(ev, ctx.m())
            return
        check_setup(ctx)
        if ctx.err:
            blocked_all(ev, ctx.m())
            return
        arm = ctx.arm
        if arm.animation_data is not None and arm.animation_data.action is not None:
            ctx.notes.append(f"armature action {arm.animation_data.action.name} unassigned in memory")
            arm.animation_data.action = None
        if arm.data.pose_position != "POSE":
            ctx.notes.append(f"armature pose_position {arm.data.pose_position} -> POSE in memory")
            arm.data.pose_position = "POSE"
        parse_decisions(ctx)
        W = read_weights(ctx.mesh)
        check_g51(ev, ctx, W)
        check_g52(ev, ctx, W)
        check_g53(ev, ctx)
        tmp = Path(tempfile.mkdtemp(prefix="g5_cells_"))
        try:
            res, geos, cells = measure(ctx, tmp)
            write_criteria(ev, ctx, res, geos)
            written, layout = [], {}
            for group in GROUPS:
                grid, rows, cols = sheet_layout(ctx, group, cells)
                if not grid:
                    continue
                out = compose_sheet(grid, outdir / f"rom_{group}.png")
                ev.render(out)
                written.append(out.name)
                n_empty = sum(1 for row in grid for c in row if c is None)
                layout[out.name] = {"rows": rows, "cols": cols, "n_empty_cells": n_empty,
                                    "crop_half_mm": mm(CELL_HALF.get(group)) if group != "full" else None,
                                    "crop_depth_mm": mm(CELL_DEPTH.get(group)) if group != "full" else None}
            ev.criterion("G5.7", {"written": written, "layout": layout},
                         f"[U] user visual approval of the ROM sheets (ok = all {len(GROUPS)} sheets written)",
                         len(written) == len(GROUPS),
                         "joint groups: goblib.ortho_render wire=True of cut-out copies of the posed GOB_mesh "
                         "(faces with centroid in the crop box around the joint pivot; body grey, rigid parts blue; "
                         "head/belt hidden in limb close-ups, shown in spine_head; GOB_club not drawn), rows = "
                         "kind_side, cols = pose angle / variant, view = rom_poses view; full: GOB_mesh + GOB_club "
                         f"solid, fixed frame {FULL_BBOX}, cols front / three_quarter; no text labels; "
                         "poses in memory, blend not saved"
                         + ("; " + "; ".join(ctx.notes) if ctx.notes else ""))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    finally:
        restore(ctx)
        ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
