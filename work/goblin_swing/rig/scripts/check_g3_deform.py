"""check_g3_deform - Gate G3 RETOPO-DEFORM checker (design doc d-23 section 7, G3.2 .. G3.6).

Inputs (independent of the production script s03_temprig.py; only blend geometry + data json):
  rig/gob_r01t_temprig.blend  GOB_temprig (armature, 24 bones of spec section 2, roll rule G4.4),
                              GOB_mesh (Armature modifier, face INT part_id), GOB_club (bone parent)
  rig/data/parts.json, rig/data/retopo_loops.json, rig/data/pivots.json

Pose set (G3.2), per side, one bone rotated per pose, every other pose bone at rest.  Poses are
applied in memory only (quaternion on the pose bone); the blend is never saved.
  shoulder_raise_30/60/90  upperarm, world axis = unit(d x +Z), d = upperarm head->tail
  shoulder_fwd_30/60/90    upperarm, world axis = unit(d x -Y)
  elbow_30/60/90           lowerarm, local +X (G4.4: the hand moves -Y)
  hip_flex_30/60/90        thigh,    world axis = unit(d x -Y), d = thigh head->tail
  knee_30/60/90            calf,     local +X (G4.4: the foot moves +Y)
  wrist_twist_90           hand,     local +Y
  ankle_30                 foot,     world axis = unit(d x +Z), d = foot head -> pivots foot_tip
World axes are converted to the bone's local rest frame (R_rest^T axis); since the parents are at
rest this is the same rotation about the bone head.  Local axis, sign and a displacement check of
the moved point are written to criterion G3.2.

Metrics on the depsgraph-evaluated GOB_mesh (world space, same vertex/face indices as obj.data):
  G3.3  slices perpendicular to the chain bone axis through the joint ring region (the joint's
        retopo rings projected on the axis, +-SLICE_MARGIN, step SLICE_STEP).  Each slice plane is
        fixed to an owner bone (proximal owner for s < 0, distal owner for s >= 0, s measured from
        the joint pivot) and moved with that bone's pose delta.  In-plane radius = first hit of
        N_RAYS rays cast from the slice center (on the axis line through the pivot) against the
        body faces.  ratio = min radius posed / min radius rest of the same slice; per pose the
        minimum over slices.
  G3.4  wrist_x_end / ankle_x_end ring verts: signed inside depth w.r.t. the posed hand_x / shoe_x
        faces (BVH nearest distance, + inside by ray parity majority of 3 fixed directions).
        Evaluated for every pose of the same limb side and for rest.
  G3.5  joint region = body faces whose rest centroid is within R of the joint pivot,
        R = max rest distance of the joint's ring verts to the pivot + tube radius (pivots.json).
        a: BVHTree.overlap of the region triangles with themselves; triangle pairs of the same
           polygon or of polygons sharing a vertex are dropped; distinct polygon pairs counted.
           Only body faces are used, so the designed overlap pairs (body - rigid part) never enter.
        b: region faces with posed area < COLLAPSE * rest area.
Renders (G3.6): rig/inspect/G3/sheet_<joint>.png, one cell per pose (rows = side, for the shoulder
  raise l / raise r / fwd l / fwd r; columns = 30/60/90 deg, single column for wrist and ankle).
  Cell = goblib.ortho_render(wire=True) of temporary cut-out meshes of the posed GOB_mesh (body grey,
  hand/shoe parts blue, head/belt and GOB_club not drawn): faces whose centroid is inside a box
  around the joint pivot (image-plane half size CELL_HALF, view-depth half size CELL_DEPTH).
  Cells are composited with numpy on white, no text labels.

Run:    bl.ps1 -Script check_g3_deform.py -Blend gob_r01t_temprig.blend
Output: rig/inspect/G3/check_g3_deform.json + rig/inspect/G3/sheet_<joint>.png
Exit:   0 when the evidence was written; 2 on a script error (run_main) or when an input file is
        missing (the evidence is still written, then "missing input" is printed).
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
from mathutils import Matrix, Quaternion, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

GATE = "G3"
CHECKER = "check_g3_deform"
BLEND = goblib.RIG / "gob_r01t_temprig.blend"
PARTS_JSON = goblib.DATA / "parts.json"
LOOPS_JSON = goblib.DATA / "retopo_loops.json"
PIVOTS_JSON = goblib.DATA / "pivots.json"

ARM = "GOB_temprig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
SIDES = ("l", "r")
ANGLES = (30, 60, 90)

RATIO_MIN = 0.8          # G3.3
DEPTH_MIN = 0.0          # G3.4 (m)
COLLAPSE = 0.10          # G3.5b
SLICE_STEP = 0.0025      # m
SLICE_MARGIN = 0.010     # m
N_RAYS = 72
RAY_MAX = 0.30           # m
LIST_CAP = 10
CELL_PX = 480
CELL_GAP = 8

RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237),
                                              (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6

# joint -> rings (k suffixes), (proximal owner, proximal axis bone), (distal owner, distal axis bone)
JOINTS = {
    "shoulder": {"rings": ("0", "1", "2"), "prox": ("shoulder", "upperarm"), "dist": ("upperarm", "upperarm")},
    "elbow": {"rings": ("0", "1", "2"), "prox": ("upperarm", "upperarm"), "dist": ("lowerarm", "lowerarm")},
    "wrist": {"rings": ("0", "1"), "prox": ("lowerarm", "lowerarm"), "dist": ("hand", "lowerarm")},
    "hip": {"rings": ("0", "1", "2"), "prox": ("pelvis", "thigh"), "dist": ("thigh", "thigh")},
    "knee": {"rings": ("0", "1", "2"), "prox": ("thigh", "thigh"), "dist": ("calf", "calf")},
    "ankle": {"rings": ("0", "1"), "prox": ("calf", "calf"), "dist": ("foot", "calf")},
}
LIMB = {"shoulder": "arm", "elbow": "arm", "wrist": "arm", "hip": "leg", "knee": "leg", "ankle": "leg"}
END_RING = {"wrist": ("wrist_{s}_end", "hand_{s}", "arm"), "ankle": ("ankle_{s}_end", "shoe_{s}", "leg")}

# pose kind -> joint, bone, mode, axis/target, moved point, check target, angles
POSE_KINDS = {
    "shoulder_raise": {"joint": "shoulder", "bone": "upperarm", "mode": "world", "target": (0, 0, 1),
                       "point": "tail", "angles": ANGLES},
    "shoulder_fwd": {"joint": "shoulder", "bone": "upperarm", "mode": "world", "target": (0, -1, 0),
                     "point": "tail", "angles": ANGLES},
    "elbow": {"joint": "elbow", "bone": "lowerarm", "mode": "local", "axis": (1, 0, 0),
              "target": (0, -1, 0), "point": "tail", "angles": ANGLES},
    "hip_flex": {"joint": "hip", "bone": "thigh", "mode": "world", "target": (0, -1, 0),
                 "point": "tail", "angles": ANGLES},
    "knee": {"joint": "knee", "bone": "calf", "mode": "local", "axis": (1, 0, 0),
             "target": (0, 1, 0), "point": "tail", "angles": ANGLES},
    "wrist_twist": {"joint": "wrist", "bone": "hand", "mode": "local", "axis": (0, 1, 0),
                    "target": None, "point": None, "angles": (90,)},
    "ankle": {"joint": "ankle", "bone": "foot", "mode": "world", "target": (0, 0, 1),
              "point": "foot_tip", "angles": (30,)},
}

# renders: view per pose kind, image-plane half size and view-depth half size per joint (m)
CELL_VIEW = {"shoulder_raise": "front", "shoulder_fwd": "top", "elbow": "top", "wrist_twist": "front",
             "hip_flex": "side", "knee": "side", "ankle": "side"}
CELL_HALF = {"shoulder": 0.20, "elbow": 0.15, "wrist": 0.12, "hip": 0.15, "knee": 0.13, "ankle": 0.20}
CELL_DEPTH = {"shoulder": 0.08, "elbow": 0.08, "wrist": 0.08, "hip": 0.08, "knee": 0.08, "ankle": 0.13}
SHEETS = {
    "shoulder": {"rows": [("shoulder_raise", "l"), ("shoulder_raise", "r"),
                          ("shoulder_fwd", "l"), ("shoulder_fwd", "r")], "cols": ANGLES},
    "elbow": {"rows": [("elbow", "l"), ("elbow", "r")], "cols": ANGLES},
    "wrist": {"rows": [("wrist_twist", "l"), ("wrist_twist", "r")], "cols": (90,)},
    "hip": {"rows": [("hip_flex", "l"), ("hip_flex", "r")], "cols": ANGLES},
    "knee": {"rows": [("knee", "l"), ("knee", "r")], "cols": ANGLES},
    "ankle": {"rows": [("ankle", "l"), ("ankle", "r")], "cols": (30,)},
}
BODY_COLOR = (0.80, 0.80, 0.80, 1.0)
RIGID_COLOR = (0.55, 0.72, 0.95, 1.0)
HIDDEN_PARTS = ("head", "belt")

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


def axis_label(v):
    v = np.asarray(v, dtype=np.float64)
    i = int(np.argmax(np.abs(v)))
    return f"{'+' if v[i] >= 0 else '-'}{'XYZ'[i]} ({abs(v[i]):.4f})"


def vec_angle_deg(a, b):
    """Angle between vectors via atan2(|a x b|, a.b) (stable near 0, unlike acos of float32 dots)."""
    a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    return math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.dot(a, b))))


def side_name(bone, x):
    return bone if bone == "pelvis" else f"{bone}_{x}"


def pose_id(kind, a):
    return f"{kind}_{a}"


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


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.miss = {}
        self.err = []
        self.parts = self.loops = self.pivots = None
        self.arm = self.mesh = None
        self.topo = None
        self.notes = []

    def m(self):
        return [v for v in self.miss.values() if v] + self.err


def check_setup(ctx):
    arm = bpy.data.objects.get(ARM)
    mo = bpy.data.objects.get(MESH)
    if arm is None or arm.type != "ARMATURE":
        ctx.err.append(f"missing object: {ARM}" if arm is None else f"{ARM} is {arm.type}, not ARMATURE")
    if mo is None or mo.type != "MESH":
        ctx.err.append(f"missing object: {MESH}" if mo is None else f"{MESH} is {mo.type}, not MESH")
    if ctx.err:
        return
    ctx.arm, ctx.mesh = arm, mo
    need = set()
    for x in SIDES:
        for b in ("shoulder", "upperarm", "lowerarm", "hand", "thigh", "calf", "foot"):
            need.add(f"{b}_{x}")
    need.add("pelvis")
    missing = sorted(n for n in need if n not in arm.pose.bones)
    if missing:
        ctx.err.append("missing bones: " + ", ".join(missing))
    if not isinstance(ctx.parts, dict):
        ctx.err.append("parts.json is not an object")
        return
    ctx.topo = Topo(mo, {k: v for k, v in ctx.parts.items() if isinstance(v, int)})
    if ctx.topo.part is None:
        ctx.err.append("GOB_mesh part_id FACE INT attribute missing/invalid")
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
    for x in SIDES:
        for r in (f"wrist_{x}_end", f"ankle_{x}_end"):
            if r not in rings:
                bad.append(f"ring {r}")
        if f"foot_tip_{x}" not in piv:
            bad.append(f"pivot foot_tip_{x}")
    if bad:
        ctx.err.append("missing data: " + ", ".join(bad))
    for mod in mo.modifiers:
        if mod.type == "ARMATURE" and mod.object is not arm:
            ctx.notes.append(f"armature modifier {mod.name} targets "
                             f"{mod.object.name if mod.object else None}")


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


# ---------------------------------------------------------------- pose table
def build_poses(ctx):
    arm = ctx.arm
    poses = []
    for x in SIDES:
        for kind, spec in POSE_KINDS.items():
            bone = f"{spec['bone']}_{x}"
            M = rest_world(arm, bone)
            R = M.to_3x3().normalized()
            head = np.array(M.translation)
            b = arm.data.bones[bone]
            tail = np.array(arm.matrix_world @ b.tail_local)
            if spec["point"] == "tail":
                point = tail
            elif spec["point"] == "foot_tip":
                point = np.array(ctx.piv[f"foot_tip_{x}"]["co"], dtype=np.float64)
            else:
                point = None
            if spec["mode"] == "world":
                d = point - head
                waxis = unit(np.cross(d, np.array(spec["target"], dtype=np.float64)))
                laxis = np.array(R.transposed() @ Vector(waxis))
            else:
                laxis = np.array(spec["axis"], dtype=np.float64)
                waxis = np.array(R @ Vector(laxis))
            for a in spec["angles"]:
                poses.append({"id": pose_id(kind, a), "kind": kind, "side": x, "joint": spec["joint"],
                              "bone": bone, "mode": spec["mode"], "angle": a, "local_axis": laxis,
                              "world_axis": waxis, "point": point, "head": head,
                              "target": spec["target"]})
    return poses


def apply_pose(ctx, p):
    reset_pose(ctx.arm)
    if p is not None:
        pb = ctx.arm.pose.bones[p["bone"]]
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion(Vector(p["local_axis"]), math.radians(p["angle"]))
    update()


def deltas(ctx, names):
    """bone -> 4x4 numpy delta (posed world @ rest world^-1)."""
    out = {}
    for n in names:
        post = ctx.arm.matrix_world @ ctx.arm.pose.bones[n].matrix
        out[n] = mat_np(post @ rest_world(ctx.arm, n).inverted())
    return out


# ---------------------------------------------------------------- per joint geometry
class JointGeo:
    """Slices (G3.3) and face region (G3.5) of one joint side, built from the rest mesh."""

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
        # G3.5 region
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


def self_intersections(topo, geo, V):
    if not len(geo.region_tris):
        return 0, []
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
    return len(pairs), sorted(pairs)[:LIST_CAP]


# ---------------------------------------------------------------- renders
def render_cell(ctx, V, joint, kind, side, out_png):
    topo = ctx.topo
    d, right, up = goblib._view_basis(CELL_VIEW[kind])
    d, right, up = np.array(d), np.array(right), np.array(up)
    piv = np.array(ctx.piv[f"{joint}_{side}"]["co"], dtype=np.float64)
    h, dep = CELL_HALF[joint], CELL_DEPTH[joint]
    cen = topo.poly_centroid(V) - piv
    inbox = ((np.abs(cen @ right) <= h) & (np.abs(cen @ up) <= h) & (np.abs(cen @ d) <= dep))
    hidden = np.zeros(topo.npoly, dtype=bool)
    for p in HIDDEN_PARTS:
        hidden[topo.polys_of(p)] = True
    body = np.zeros(topo.npoly, dtype=bool)
    body[topo.polys_of("body")] = True
    groups = {"_g3_cell_body": np.nonzero(inbox & body)[0],
              "_g3_cell_rigid": np.nonzero(inbox & ~body & ~hidden)[0]}
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
        corners = [piv + a * h * right + b * h * up + c * dep * d
                   for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
        C = np.array(corners)
        bbox = (tuple(C.min(axis=0)), tuple(C.max(axis=0)))
        goblib.ortho_render(objs, CELL_VIEW[kind], out_png, res_h=CELL_PX, frame_bbox=bbox, wire=True,
                            colors=colors)
    finally:
        for ob in objs:
            bpy.data.objects.remove(ob, do_unlink=True)
        for me in meshes:
            bpy.data.meshes.remove(me)
    return out_png


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
    img = bpy.data.images.new("_g3_sheet", W, H, alpha=True)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(sheet[::-1]).ravel())
        img.filepath_raw = str(out_png)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)
    return out_png


# ---------------------------------------------------------------- measurement
def measure(ev, ctx, cell_dir):
    arm, mo, topo = ctx.arm, ctx.mesh, ctx.topo
    if arm.animation_data is not None and arm.animation_data.action is not None:
        ctx.notes.append(f"armature action {arm.animation_data.action.name} unassigned in memory")
        arm.animation_data.action = None
    if arm.data.pose_position != "POSE":
        ctx.notes.append(f"armature pose_position {arm.data.pose_position} -> POSE in memory")
        arm.data.pose_position = "POSE"

    apply_pose(ctx, None)
    V0 = eval_verts(mo)
    if len(V0) != topo.nv:
        raise RuntimeError(f"evaluated GOB_mesh has {len(V0)} verts, obj.data has {topo.nv}")
    body_tris = topo.tris_of_polys(topo.polys_of("body"))
    bvh0 = bvh_of(V0, topo.tri_v[body_tris])

    geos = {(j, x): JointGeo(ctx, j, x, V0) for j in JOINTS for x in SIDES}
    for g in geos.values():
        g.rest_r = g.radii(bvh0)

    end_info = {}
    for j, (ring_t, part_t, _) in END_RING.items():
        for x in SIDES:
            end_info[(j, x)] = (np.array(ctx.rings[ring_t.format(s=x)]["verts"], dtype=np.int64),
                                topo.tris_of_polys(topo.polys_of(part_t.format(s=x))),
                                part_t.format(s=x))

    res = {"G3.3": {}, "G3.4": {}, "G3.5a": {}, "G3.5b": {}, "G3.2": {}}
    # rest baselines
    for (j, x), (rv, pt, _) in end_info.items():
        dep = signed_depth(bvh_of(V0, topo.tri_v[pt]), V0[rv]) if len(pt) else np.full(len(rv), -np.inf)
        res["G3.4"].setdefault((j, x), {})["rest"] = {"min_depth_mm": mm(dep.min()),
                                                      "n_below_0": int((dep < DEPTH_MIN).sum())}
    for (j, x), g in geos.items():
        n, ex = self_intersections(topo, g, V0)
        res["G3.5a"].setdefault((j, x), {})["rest"] = {"pairs": n, "examples": ex}
    # G3.5a_rest: whole body (all body faces), same overlap / exclusion rule as the joint regions
    body_polys = topo.polys_of("body")
    whole = type("WholeBody", (), {})()
    whole.region_tris = body_tris
    whole.region_sets = {int(p): set(topo.poly_verts[int(p)]) for p in body_polys.tolist()}
    n, ex = self_intersections(topo, whole, V0)
    res["G3.5a_rest"] = {"pairs": n, "examples": ex, "n_body_faces": int(len(body_polys))}

    poses = build_poses(ctx)
    cells = {}
    for p in poses:
        apply_pose(ctx, p)
        V = eval_verts(mo)
        x, j = p["side"], p["joint"]
        g = geos[(j, x)]
        D = deltas(ctx, set(g.owners) | {p["bone"]})
        # G3.2 record
        Db = D[p["bone"]]
        rec = {"bone": p["bone"], "mode": p["mode"], "angle_deg": p["angle"],
               "local_axis": [rnd(c, 5) for c in p["local_axis"]], "local_axis_label": axis_label(p["local_axis"]),
               "sign": "+" if p["angle"] > 0 else "-",
               "world_axis": [rnd(c, 5) for c in p["world_axis"]]}
        if p["point"] is not None:
            q = Db[:3, :3] @ p["point"] + Db[:3, 3]
            disp = q - p["point"]
            tgt = np.array(p["target"], dtype=np.float64)
            rec["check"] = {"moved_point": "bone tail" if POSE_KINDS[p["kind"]]["point"] == "tail" else "foot_tip pivot",
                            "target_dir": list(p["target"]), "disp_mm": [mm(c) for c in disp],
                            "disp_dir_dot_target": rnd(float(np.dot(unit(disp), tgt)), 5)}
            rec["ok"] = rec["check"]["disp_dir_dot_target"] > 0
        else:
            Rr = rest_world(ctx.arm, p["bone"]).to_3x3().normalized()
            Rp = (ctx.arm.matrix_world @ ctx.arm.pose.bones[p["bone"]].matrix).to_3x3().normalized()
            yr, yp = np.array(Rr.col[1]), np.array(Rp.col[1])
            xr, xp = np.array(Rr.col[0]), np.array(Rp.col[0])
            ang_y = vec_angle_deg(yr, yp)
            ang_x = vec_angle_deg(xr, xp)
            rec["check"] = {"bone_Y_axis_change_deg": rnd(ang_y, 5), "bone_X_axis_turn_deg": rnd(ang_x, 4)}
            rec["ok"] = ang_y <= 0.01 and abs(ang_x - p["angle"]) <= 0.01
        res["G3.2"][p["id"] + f"_{x}"] = rec

        # G3.3
        bvh = bvh_of(V, topo.tri_v[body_tris])
        rp = g.radii(bvh, D)
        best = None
        n_out = n_miss = 0
        for sl, (r0, _), (r1, ins) in zip(g.slices, g.rest_r, rp):
            n_out += 0 if ins else 1
            if r0 is None or r1 is None:
                n_miss += 1
                continue
            ratio = r1 / r0
            if best is None or ratio < best[0]:
                best = (ratio, sl["s"], r1, r0, sl["owner"])
        res["G3.3"].setdefault((j, x), {})[p["id"]] = {
            "min_ratio": rnd(best[0], 4) if best else None,
            "worst_slice_s_mm": mm(best[1]) if best else None,
            "posed_min_r_mm": mm(best[2]) if best else None, "rest_min_r_mm": mm(best[3]) if best else None,
            "worst_slice_owner": best[4] if best else None,
            "n_slices": len(g.slices), "n_slices_no_hit": n_miss, "n_centers_outside_body": n_out}

        # G3.4 (every pose of the same limb side)
        for (ej, ex), (rv, pt, part) in end_info.items():
            if ex != x or END_RING[ej][2] != LIMB[j]:
                continue
            dep = signed_depth(bvh_of(V, topo.tri_v[pt]), V[rv]) if len(pt) else np.full(len(rv), -np.inf)
            res["G3.4"][(ej, ex)][p["id"]] = {"min_depth_mm": mm(dep.min()),
                                              "n_below_0": int((dep < DEPTH_MIN).sum())}

        # G3.5
        n, ex_pairs = self_intersections(topo, g, V)
        res["G3.5a"][(j, x)][p["id"]] = {"pairs": n, "examples": ex_pairs}
        a1 = topo.poly_area(V)[g.region]
        ratio = np.where(g.area0 > 0, a1 / np.where(g.area0 > 0, g.area0, 1.0), np.inf)
        worst = int(np.argmin(ratio)) if len(ratio) else None
        res["G3.5b"].setdefault((j, x), {})[p["id"]] = {
            "n_collapsed": int((ratio < COLLAPSE).sum()),
            "min_area_ratio": rnd(ratio[worst], 4) if worst is not None else None,
            "min_area_face": int(g.region[worst]) if worst is not None else None}

        # G3.6 cell
        out = cell_dir / f"{p['kind']}_{x}_{p['angle']}.png"
        render_cell(ctx, V, j, p["kind"], x, out)
        cells[(p["kind"], x, p["angle"])] = out
        print(f"[{GATE}/{CHECKER}] pose {p['id']}_{x}: ratio={res['G3.3'][(j, x)][p['id']]['min_ratio']} "
              f"isect={n} collapsed={res['G3.5b'][(j, x)][p['id']]['n_collapsed']}")
        sys.stdout.flush()

    apply_pose(ctx, None)
    return res, geos, cells


def write_criteria(ev, ctx, res, geos):
    th2 = ("each pose: only the listed bone rotated, all other pose bones at rest; moved point "
           "displacement . target direction > 0 (wrist twist: hand Y axis unchanged <= 0.01 deg, "
           "hand X axis turned by the pose angle +-0.01 deg)")
    ev.criterion("G3.2", res["G3.2"], th2, all(r["ok"] for r in res["G3.2"].values()) and len(res["G3.2"]) == 34,
                 "world-defined poses: world axis = unit(d x target), local axis = R_rest^T world axis "
                 "(parents at rest); elbow/knee/twist: local axis as given (G4.4)"
                 + ("; " + "; ".join(ctx.notes) if ctx.notes else ""))
    th3 = (f"judged poses (30 and 60 deg of the 30/60/90 sets; single-pose joints wrist twist 90, ankle 30: "
           f"that pose): min over slices of (min ray radius posed / min ray radius rest) >= {RATIO_MIN}; "
           "90 deg of the 30/60/90 sets reported only (visual judgement in G3.6)")
    th4 = "report only (spec G3.4 revised 2026-09-25): judged in G5.5"
    th5a_rest = "rest pose: intersecting body face pairs (all body faces) = 0"
    th5a = ("judged poses (30 and 60 deg): intersecting body face pairs in the joint region = 0; "
            "90 deg reported only")
    th5b = f"all poses: body faces in the joint region with posed area < {COLLAPSE:.0%} of rest area = 0"
    isect_note = ("BVHTree.overlap(tris, same tris); tri pairs of the same polygon or of polygons sharing any "
                  "vertex are excluded; counted as distinct polygon pairs; body faces only (designed body-rigid "
                  "overlap never compared)")

    def judged(key):
        kind, a = key.rsplit("_", 1)
        return len(POSE_KINDS[kind]["angles"]) == 1 or int(a) in (30, 60)

    r5 = res["G3.5a_rest"]
    ev.criterion("G3.5a_rest", r5, th5a_rest, r5["pairs"] == 0,
                 isect_note + "; rest = all pose bones at identity, depsgraph-evaluated GOB_mesh")
    for j in JOINTS:
        for x in SIDES:
            g = geos[(j, x)]
            m3 = dict(res["G3.3"][(j, x)])
            jv = [v["min_ratio"] for k, v in m3.items() if judged(k)]
            rv = [v["min_ratio"] for k, v in m3.items() if not judged(k)]
            allv = [v["min_ratio"] for v in m3.values()]
            m3 = {"min_judged": min((v for v in jv if v is not None), default=None),
                  "min_report_only_90": min((v for v in rv if v is not None), default=None),
                  "min": min((v for v in allv if v is not None), default=None),
                  "judged_poses": [k for k in m3 if judged(k)], "poses": m3, "rings_s_mm": g.ring_s_mm,
                  "slice_s_mm": [mm(g.slices[0]["s"]), mm(g.slices[-1]["s"])],
                  "rest_min_r_mm": [mm(r) if r is not None else None for r, _ in g.rest_r]}
            ok3 = bool(jv) and all(v is not None and v >= RATIO_MIN for v in jv)
            ev.criterion(f"G3.3_{j}_{x}", m3, th3, ok3,
                         f"slices every {SLICE_STEP * 1000:.1f} mm over ring span +-{SLICE_MARGIN * 1000:.0f} mm, "
                         f"{N_RAYS} in-plane rays from the axis point (body faces only, max {RAY_MAX} m); "
                         "plane normal = chain bone axis, plane moved with the owner bone; ok uses judged poses only")
            m5a = res["G3.5a"][(j, x)]
            jp = [v["pairs"] for k, v in m5a.items() if k != "rest" and judged(k)]
            rp = [v["pairs"] for k, v in m5a.items() if k != "rest" and not judged(k)]
            ev.criterion(f"G3.5a_{j}_{x}", {"max_judged": max(jp) if jp else None,
                                            "max_report_only_90": max(rp) if rp else None,
                                            "judged_poses": [k for k in m5a if k != "rest" and judged(k)],
                                            "poses": m5a,
                                            "region": {"radius_mm": mm(g.R), "n_faces": int(len(g.region))}},
                         th5a, bool(jp) and all(v == 0 for v in jp),
                         isect_note + "; no rest-pair subtraction; 'rest' = region value at rest, reference only "
                         "(rest is judged by G3.5a_rest)")
            m5b = res["G3.5b"][(j, x)]
            ev.criterion(f"G3.5b_{j}_{x}", {"max": max(v["n_collapsed"] for v in m5b.values()), "poses": m5b,
                                            "region": {"radius_mm": mm(g.R), "n_faces": int(len(g.region))}},
                         th5b, all(v["n_collapsed"] == 0 for v in m5b.values()),
                         "posed / rest polygon area (sum of loop triangles), region as G3.5a")
            if j in END_RING:
                m4 = res["G3.4"][(j, x)]
                posed4 = [v["min_depth_mm"] for k, v in m4.items() if k != "rest"]
                num = [v for v in posed4 if isinstance(v, (int, float))]
                ev.criterion(f"G3.4_{j}_{x}", {"min_mm": min(num) if num else None, "poses": m4,
                                               "ring": END_RING[j][0].format(s=x), "part": END_RING[j][1].format(s=x)},
                             th4, True,
                             "판정은 G5.5로 이관(스펙 G3.4); "
                             f"all {LIMB[j]} poses of side {x}; depth = BVH nearest distance, sign by ray parity "
                             "(majority of 3 directions) against all faces of the part; 'rest' = baseline")


def blocked_all(ev, reasons):
    r = "; ".join(x for x in reasons if x)
    ev.criterion("G3.2", r, "pose set applied", False)
    ev.criterion("G3.5a_rest", r, "rest body self-intersection pairs = 0", False)
    for j in JOINTS:
        for x in SIDES:
            ev.criterion(f"G3.3_{j}_{x}", r, f"30/60 deg (single-pose joints: that pose) >= {RATIO_MIN}", False)
            if j in END_RING:
                ev.criterion(f"G3.4_{j}_{x}", r, "report only (judged in G5.5)", False)
            ev.criterion(f"G3.5a_{j}_{x}", r, "30/60 deg: 0", False)
            ev.criterion(f"G3.5b_{j}_{x}", r, "0", False)
    ev.criterion("G3.6", r, "[U] sheets written", False)


# ---------------------------------------------------------------- main
def _load_json(path, ctx, key):
    if not path.exists():
        _STATE["missing"].append(rel(path))
        ctx.miss[key] = f"missing input: {rel(path)}"
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


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
    ev.add_stage_inputs("s03")

    ctx.parts = _load_json(PARTS_JSON, ctx, "parts")
    ctx.loops = _load_json(LOOPS_JSON, ctx, "loops")
    ctx.pivots = _load_json(PIVOTS_JSON, ctx, "pivots")

    try:
        if ctx.m():
            blocked_all(ev, ctx.m())
            return
        check_setup(ctx)
        if ctx.err:
            blocked_all(ev, ctx.m())
            return
        tmp = Path(tempfile.mkdtemp(prefix="g3_cells_"))
        try:
            res, geos, cells = measure(ev, ctx, tmp)
            write_criteria(ev, ctx, res, geos)
            written, layout = [], {}
            for joint, sh in SHEETS.items():
                grid = [[cells.get((kind, x, a)) for a in sh["cols"]] for kind, x in sh["rows"]]
                out = compose_sheet(grid, outdir / f"sheet_{joint}.png")
                ev.render(out)
                written.append(out.name)
                layout[out.name] = {"rows": [f"{kind}_{x}" for kind, x in sh["rows"]],
                                    "cols_deg": list(sh["cols"]),
                                    "view": sorted({CELL_VIEW[kind] for kind, _ in sh["rows"]}),
                                    "crop_half_mm": mm(CELL_HALF[joint]), "crop_depth_mm": mm(CELL_DEPTH[joint])}
            ev.criterion("G3.6", {"written": written, "layout": layout},
                         "[U] user visual approval of the joint sheets (ok = all 6 sheets written)",
                         len(written) == len(SHEETS),
                         "cell = goblib.ortho_render wire=True of cut-out copies of the posed GOB_mesh "
                         "(faces with centroid in the crop box around the joint pivot; body grey, hand/shoe "
                         "blue, head/belt/GOB_club hidden); no text labels; poses in memory, blend not saved")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    finally:
        ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
