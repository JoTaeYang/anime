"""check_p2_deform - Player P2.2 deform-test checker (task T221; spec work/player/d-02-player-rig.md section 2
gate P2.2, checker side).

Inputs (defaults; override after "--"):
  --blend  work/player/rig/pl_r01t_temprig.blend     temporary rig: an armature with the skeleton v0 bone names
                                                      (d-01 section 2) deforming one MESH with FACE INT part_id
  --parts  work/player/rig/data/parts.json            {part name: part_id} or {"part_id": {part name: part_id}, ...}
  --loops  work/player/rig/data/retopo_loops.json     rings (slice span of the tube ratio; optional per joint)
  --p0b    work/player/inspect/P0b/p0b_measure.json   hip contact angles (skirt.hip_rotation_contact)
  --mesh / --armature <object name>                   default: the only MESH with part_id / its Armature modifier
  --out    work/player/inspect/P2/check_p2_deform.json   (sheet: <out dir>/P2_joint_sheet.png)
  --in-memory                                         self-test seam: use the data already loaded (no --blend open)

Pose set (one bone rotated per pose, every other pose bone at rest; quaternion on the pose bone in memory; the
blend is never saved).  World axis = unit(d x target), d = bone head -> tail at rest; local axis = R_rest^T world
axis (parents at rest -> rotation about the bone head).  Copied from goblin check_g3_deform.build_poses; the player
temp rig has no roll contract yet, so every bend is world-defined.
  elbow 30/60/90           Forearm_X   target -Y (fist forward)
  knee 30/60/90            Calf_X      target +Y (foot back)
  shoulder_raise 30/60/90  Arm_X       target +Z
  shoulder_fwd 30/60/90    Arm_X       target -Y
  hip_flex / hip_abd / hip_ext  Thigh_X  target -Y / outward (+-X) / +Y; angles = (c/2, c, c+15) with c = the P0b
                           contact angle of that side (skirt.hip_rotation_contact <side>.hip_landmark.flex_front /
                           abduct_side / extend_back; fallback 40.5 / 24.8 / 35.8) -> up to and past the contact
  wrist_twist 60           Hand_X      about its own rest axis d
Metrics on the depsgraph-evaluated mesh (world space, obj.data indices):
  tube ratio (elbow, knee; wrist twist report): goblin G3.3 slices (step 2.5 mm over the joint's rings of
    retopo_loops.json +- 10 mm, fallback +-30 mm about the pivot), plane fixed to the owner bone (proximal s < 0,
    distal s >= 0) and moved with its pose delta; 72 in-plane rays from the axis point against the tube part faces
    (arm_X / leg_X); ratio = posed / rest min ray radius of the same slice; per pose the min over slices.
  visible self-intersection: region = deforming-part faces (arm, leg, tunic, skirt) whose rest centroid is within R
    of the pivot (R = max ring-vertex distance + 20 mm, fallback per joint); BVHTree.overlap of the posed region
    triangles with themselves, pairs of the same polygon or of polygons sharing a vertex dropped, distinct polygon
    pairs counted (goblin G3.5a); a pair is hidden when every intersection point of its triangle pairs (segment/
    triangle crossings, mean) lies inside a closed rigid shell of the posed mesh (ray parity majority of 3) or,
    T223 (a), inside the tunic virtually capped (open tunic / skirt rims fan-capped to the rim vertex mean, in
    memory, inside tests only); the pair's own parts are never used as the container -
    goblin used body faces only, so body - rigid overlaps never counted; here the tube ends inside sleeves, fists,
    cuffs are dropped the same way.  visible = not hidden.  self = both faces in one part, cross = two deforming parts.
    T223 (b): leg face x skirt-ceiling face (tunic faces with every rest vertex within 2 mm of retopo_loops.json
    tunic.ceiling_z) = designed crossing, counted apart and never visible.
  sleeve vs tunic / skirt / scarf / scarf_tail (shoulder poses): polygon pairs whose triangles intersect, visible =
    intersection point not inside a closed / virtually capped shell of any third part, split visible / hidden with
    the pair penetration depth (T223 c); plus vertices of one inside the other's shells; rest values beside.
  leg vs skirt (hip poses): leg_X faces vs skirt faces (tunic / skirt faces with rest centroid z < belt bottom);
    same pair / visible / designed rule; leg vertices inside the virtually capped tunic split above (designed) /
    below (skirt wall) the ceiling; leg vertices poking out through the skirt wall (T223 d, poke_out); rest beside.
Renders: one cut-out cell per pose (faces with centroid inside a box around the pivot; camera looks along the pose
  rotation axis from the front / outside; deforming parts grey, rigid parts blue, head and eyes hidden; hip cells also arm / fist / sleeve), wire overlay,
  composited into <out dir>/P2_joint_sheet.png: rows = pose kinds, columns = side l 3 angles | side r 3 angles.
Draft thresholds (tube ratio >= 0.8, visible self-intersection 0 at 30/60) are calibration rows: ok = True, note
"calibration run: threshold set after measurement".  A row that cannot be measured is "blocked: ..." ok False.
Methods copied / adapted (not imported): goblin check_g3_deform.py (poses, slices, overlap, cells, sheet),
  goblib.ortho_render (arbitrary view basis here), goblin check_g2_static (inside test), check_p1.py (evidence).
Run:    blender --background --factory-startup --python check_p2_deform.py [-- <overrides>]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
Coordinates: front -Y, up +Z, character left (_l, bones _L) +X.  Lengths in m, reported in mm.
"""
import hashlib
import json
import math
import re
import shutil
import sys
import tempfile
import traceback
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

HERE = Path(__file__).resolve().parent
PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.2", "check_p2_deform", "T221"
CALIB = "calibration run: threshold set after measurement"

P = {"blend": PLAYER_RIG / "pl_r01t_temprig.blend",
     "parts": PLAYER_RIG / "data" / "parts.json",
     "loops": PLAYER_RIG / "data" / "retopo_loops.json",
     "p0b": REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json",
     "mesh": None, "armature": None,
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_deform.json",
     "in_memory": False}
SHEET_NAME = "P2_joint_sheet.png"

RIGID_FAM = ("head", "eye", "fist", "shoe", "cuff", "belt", "pouch", "scarf", "scarf_tail", "sleeve")
DEFORM_FAM = ("arm", "leg", "tunic", "skirt")
FAM_ALIAS = {"eyes": "eye", "fists": "fist", "hand": "fist", "shoes": "shoe", "cuffs": "cuff", "sleeves": "sleeve"}
SIDES = ("l", "r")
BSIDE = {"l": "L", "r": "R"}

RATIO_MIN = 0.8           # draft (spec P2.2 italic)
ISECT_MAX = 0             # draft
ANGLES = (30, 60, 90)
JUDGED_ANGLES = (30, 60)
HIP_FALLBACK = {"flex_front": 40.5, "abduct_side": 24.8, "extend_back": 35.8}
HIP_PAST = 15.0           # deg beyond the contact angle
SLICE_STEP = 0.0025
SLICE_MARGIN = 0.010
SLICE_FALLBACK = 0.030
N_RAYS = 72
RAY_MAX = 0.30
RING_R_PAD = 0.020
VIRTUAL_CAP_FAM = ("tunic", "skirt")   # T223 (a) open rims capped virtually for the inside tests
CEIL_TOL = 0.002          # T223 (b) skirt ceiling faces: every rest vertex within this of loops tunic.ceiling_z
POKE_RAY_MAX = 0.60       # T223 (d) radial ray length from the skirt axis
REGION_R = {"elbow": 0.08, "knee": 0.08, "shoulder": 0.12, "hip": 0.15, "wrist": 0.08}
LIST_CAP = 10
CELL_PX = 320
CELL_GAP = 8
CELL_HALF = {"elbow": 0.15, "knee": 0.15, "shoulder": 0.20, "hip": 0.24, "wrist": 0.13}
CELL_DEPTH = {"elbow": 0.10, "knee": 0.10, "shoulder": 0.14, "hip": 0.26, "wrist": 0.10}
CELL_DEPTH_KIND = {"shoulder_raise": 0.40}   # T223 (e): whole body depth, the tunic front no longer cut away
CELL_HIDDEN_FAM = {"hip": ("head", "eye", "arm", "fist", "sleeve")}   # default ("head", "eye")
DEFORM_COLOR = (0.80, 0.80, 0.80, 1.0)
RIGID_COLOR = (0.55, 0.72, 0.95, 1.0)
RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237), (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6

# joint -> (proximal owner, proximal axis bone), (distal owner, distal axis bone), tube family, pivot bone
JOINTS = {
    "elbow": {"prox": ("Arm", "Arm"), "dist": ("Forearm", "Forearm"), "tube": "arm", "pivot": "Forearm"},
    "knee": {"prox": ("Thigh", "Thigh"), "dist": ("Calf", "Calf"), "tube": "leg", "pivot": "Calf"},
    "wrist": {"prox": ("Forearm", "Forearm"), "dist": ("Hand", "Forearm"), "tube": "arm", "pivot": "Hand"},
    "shoulder": {"pivot": "Arm"},
    "hip": {"pivot": "Thigh"},
}
POSE_KINDS = {
    "elbow": {"joint": "elbow", "bone": "Forearm", "target": (0, -1, 0), "angles": ANGLES},
    "knee": {"joint": "knee", "bone": "Calf", "target": (0, 1, 0), "angles": ANGLES},
    "shoulder_raise": {"joint": "shoulder", "bone": "Arm", "target": (0, 0, 1), "angles": ANGLES},
    "shoulder_fwd": {"joint": "shoulder", "bone": "Arm", "target": (0, -1, 0), "angles": ANGLES},
    "hip_flex": {"joint": "hip", "bone": "Thigh", "target": (0, -1, 0), "angles": "flex_front"},
    "hip_abd": {"joint": "hip", "bone": "Thigh", "target": "out", "angles": "abduct_side"},
    "hip_ext": {"joint": "hip", "bone": "Thigh", "target": (0, 1, 0), "angles": "extend_back"},
    "wrist_twist": {"joint": "wrist", "bone": "Hand", "target": None, "angles": (60,)},
}
TUBE_KINDS = {"elbow": "elbow", "knee": "knee", "wrist_twist": "wrist"}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    pass


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rel(p):
    p = Path(p).resolve()
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return p.as_posix()


def jsonable(v):
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else str(f)
    if v is None or isinstance(v, str):
        return v
    if isinstance(v, Path):
        return rel(v)
    if isinstance(v, np.ndarray):
        return jsonable(v.tolist())
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set)) or isinstance(v, (Vector, Quaternion)):
        return [jsonable(x) for x in v]
    return str(v)


def mm(x):
    if x is None:
        return None
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=4):
    if x is None:
        return None
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def tb_tail(n=4):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def unit(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def vec_angle_deg(a, b):
    a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    return math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.dot(a, b))))


def family(name):
    m = re.match(r"^(.*?)(?:_([lr]))?$", str(name).lower())
    fam, side = m.group(1), m.group(2)
    return FAM_ALIAS.get(fam, fam), side


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


def bvh_of(V, tri_v):
    return BVHTree.FromPolygons([tuple(p) for p in V.tolist()], tri_v.tolist(), all_triangles=True)


def mat_np(M):
    return np.array(M, dtype=np.float64)


def _components(n, a, b):
    parent = list(range(n))

    def find(i):
        r = i
        while parent[r] != r:
            r = parent[r]
        while parent[i] != r:
            parent[i], i = r, parent[i]
        return r

    for x, y in zip(np.asarray(a).tolist(), np.asarray(b).tolist()):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[rx] = ry
    return np.array([find(i) for i in range(n)], dtype=np.int64)


# ---------------------------------------------------------------- topology
class Topo:
    """Topology of the mesh data (indices shared with the evaluated mesh) + part shells."""

    def __init__(self, obj, parts):
        me = obj.data
        self.nv, self.npoly = len(me.vertices), len(me.polygons)
        ls = np.empty(self.npoly, dtype=np.int32)
        lt = np.empty(self.npoly, dtype=np.int32)
        me.polygons.foreach_get("loop_start", ls)
        me.polygons.foreach_get("loop_total", lt)
        nl = len(me.loops)
        lv = np.empty(nl, dtype=np.int32)
        le = np.empty(nl, dtype=np.int32)
        me.loops.foreach_get("vertex_index", lv)
        me.loops.foreach_get("edge_index", le)
        self.loop_vert, self.loop_edge = lv.astype(np.int64), le.astype(np.int64)
        self.loop_poly = np.repeat(np.arange(self.npoly), lt.astype(np.int64))
        self.poly_nv = lt.astype(np.int64)
        st = ls.astype(np.int64)[self.loop_poly]
        self.loop_next = st + (np.arange(nl) - st + 1) % self.poly_nv[self.loop_poly]
        self.poly_verts = [self.loop_vert[s:s + n].tolist() for s, n in zip(ls.tolist(), lt.tolist())]
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        tv = np.empty(nt * 3, dtype=np.int32)
        tp = np.empty(nt, dtype=np.int32)
        me.loop_triangles.foreach_get("vertices", tv)
        me.loop_triangles.foreach_get("polygon_index", tp)
        self.tri_v = tv.reshape(nt, 3).astype(np.int64)
        self.tri_poly = tp.astype(np.int64)
        ne = len(me.edges)
        e = np.empty(ne * 2, dtype=np.int32)
        me.edges.foreach_get("vertices", e)
        self.e = e.reshape(ne, 2).astype(np.int64)
        self.part = None
        att = me.attributes.get("part_id")
        if att is not None and att.domain == "FACE" and att.data_type == "INT":
            pv = np.empty(self.npoly, dtype=np.int32)
            att.data.foreach_get("value", pv)
            self.part = pv.astype(np.int64)
        self.parts = parts
        self.part_names = {v: k for k, v in parts.items()}
        self.shells = []
        if self.part is not None:
            self._shells()

    def _shells(self):
        key = self.part[self.loop_poly] * self.nv + self.loop_vert
        order = np.argsort(key, kind="stable")
        k, f = key[order], self.loop_poly[order]
        same = k[1:] == k[:-1]
        roots = _components(self.npoly, f[:-1][same], f[1:][same])
        _, sof = np.unique(roots, return_inverse=True)
        sof = sof.reshape(-1)
        for sid in range(int(sof.max()) + 1 if self.npoly else 0):
            faces = np.nonzero(sof == sid)[0]
            corners = np.nonzero(sof[self.loop_poly] == sid)[0]
            _u, inv, c = np.unique(self.loop_edge[corners], return_inverse=True, return_counts=True)
            name = self.part_names.get(int(self.part[faces[0]]), f"part_{int(self.part[faces[0]])}")
            fam = family(name)[0]
            closed = bool(np.all(c == 2))
            rims = self._rims(corners[(c == 1)[inv.reshape(-1)]]) if not closed else []
            # T223 (a): open tunic / skirt shells without non-manifold edges are capped virtually (fan to each
            # rim's vertex mean, in memory, inside tests only) and used as hiding volumes
            virtual = (not closed) and fam in VIRTUAL_CAP_FAM and not np.any(c > 2) and bool(rims)
            self.shells.append({"sid": sid, "part": name, "fam": fam, "faces": faces,
                                "tris": np.nonzero(sof[self.tri_poly] == sid)[0],
                                "closed": closed, "virtual": virtual, "rims": rims, "label": f"{name}#{sid}"})

    def _rims(self, bc):
        """Boundary loops of a shell: [(vertex index array, edge array (k, 2))]."""
        if not len(bc):
            return []
        a = self.loop_vert[bc]
        b = self.loop_vert[self.loop_next[bc]]
        vs = np.unique(np.concatenate([a, b]))
        loc = {int(v): i for i, v in enumerate(vs.tolist())}
        la = np.array([loc[int(x)] for x in a.tolist()])
        lb = np.array([loc[int(x)] for x in b.tolist()])
        root = _components(len(vs), la, lb)
        out = []
        for r in np.unique(root).tolist():
            mv = root == r
            em = mv[la]
            out.append((vs[mv], np.stack([a[em], b[em]], axis=1)))
        return out

    def polys_of_fams(self, fams, side=None):
        if self.part is None:
            return np.zeros(0, dtype=np.int64)
        vals = [v for k, v in self.parts.items() if family(k)[0] in fams and (side is None or family(k)[1] == side)]
        return np.nonzero(np.isin(self.part, vals))[0]

    def tris_of_polys(self, polys):
        m = np.zeros(self.npoly, dtype=bool)
        m[polys] = True
        return np.nonzero(m[self.tri_poly])[0]

    def poly_centroid(self, V):
        c = np.zeros((self.npoly, 3))
        np.add.at(c, self.loop_poly, V[self.loop_vert])
        return c / self.poly_nv[:, None]

    def verts_of_polys(self, polys):
        return np.unique(self.loop_vert[np.isin(self.loop_poly, polys)])


def eval_verts(obj):
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


class ShellSet:
    """Posed BVHs of closed shells and virtually capped shells (lazy) for inside tests."""

    def __init__(self, topo, V, shells):
        self.topo, self.V = topo, V
        self.shells = [s for s in shells if s["closed"] or s.get("virtual")]
        self._b = {}

    def bvh(self, s):
        if s["sid"] not in self._b:
            tv = self.topo.tri_v[s["tris"]]
            vi = np.unique(tv)
            P_ = self.V[vi]
            if s.get("virtual"):
                cen = np.array([self.V[rv].mean(axis=0) for rv, _e in s["rims"]])
                caps = np.concatenate([np.column_stack([e[:, 1], e[:, 0], np.full(len(e), self.topo.nv + k)])
                                       for k, (_rv, e) in enumerate(s["rims"])])
                Vx = np.concatenate([self.V, cen])
                b = bvh_of(Vx, np.concatenate([tv, caps]))
                P_ = np.concatenate([P_, cen])
            else:
                b = bvh_of(self.V, tv)
            self._b[s["sid"]] = (b, P_.min(axis=0), P_.max(axis=0))
        return self._b[s["sid"]]

    def inside_any(self, p, exclude_parts=()):
        """label of the first closed shell (not of exclude_parts) containing p, else None."""
        for s in self.shells:
            if s["part"] in exclude_parts:
                continue
            b, mn, mx = self.bvh(s)
            if np.any(p < mn) or np.any(p > mx):
                continue
            if _inside(b, Vector(p)):
                return s["label"]
        return None

    def depth(self, s, pts):
        b, mn, mx = self.bvh(s)
        out = np.full(len(pts), -np.inf)
        inb = np.all((pts >= mn) & (pts <= mx), axis=1)
        for i in np.nonzero(inb)[0].tolist():
            pv = Vector(pts[i])
            loc, _n, _i, d = b.find_nearest(pv)
            if loc is not None and _inside(b, pv):
                out[i] = d
        return out


# ---------------------------------------------------------------- triangle intersection point
def _seg_tri(p0, p1, a, b, c):
    e1, e2, d = b - a, c - a, p1 - p0
    h = np.cross(d, e2)
    det = float(e1 @ h)
    if abs(det) < 1e-18:
        return None
    f = 1.0 / det
    s = p0 - a
    u = f * float(s @ h)
    if u < 0.0 or u > 1.0:
        return None
    q = np.cross(s, e1)
    v = f * float(d @ q)
    if v < 0.0 or u + v > 1.0:
        return None
    t = f * float(e2 @ q)
    return p0 + t * d if 0.0 <= t <= 1.0 else None


def tri_tri_point(T1, T2):
    pts = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        for A, B in ((T1, T2), (T2, T1)):
            p = _seg_tri(A[i], A[j], *B)
            if p is not None:
                pts.append(p)
    if pts:
        return np.mean(pts, axis=0)
    return 0.5 * (T1.mean(axis=0) + T2.mean(axis=0))


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.parts = self.loops = self.p0b = None
        self.arm = self.mesh = self.topo = None
        self.err = []
        self.notes = []
        self.valid_rings = {}
        self.ring_note = ""
        self.ceiling_polys = set()     # T223 (b)
        self.skirt_axis = (0.0, 0.0)   # T223 (d)
        self.V0 = None


def find_objects(ctx):
    if P["mesh"]:
        mo = bpy.data.objects.get(P["mesh"])
        if mo is None or mo.type != "MESH":
            raise Blocked(f"--mesh {P['mesh']!r} not found / not a MESH")
    else:
        c = [o for o in bpy.data.objects if o.type == "MESH" and o.data.attributes.get("part_id") is not None
             and o.data.attributes["part_id"].domain == "FACE" and o.data.attributes["part_id"].data_type == "INT"]
        if len(c) != 1:
            raise Blocked(f"expected exactly one MESH with a FACE INT part_id, found {[o.name for o in c]}")
        mo = c[0]
    if P["armature"]:
        arm = bpy.data.objects.get(P["armature"])
        if arm is None or arm.type != "ARMATURE":
            raise Blocked(f"--armature {P['armature']!r} not found / not an ARMATURE")
    else:
        mods = [m for m in mo.modifiers if m.type == "ARMATURE" and m.object is not None]
        if len(mods) != 1:
            raise Blocked(f"{mo.name}: expected one Armature modifier with an object, found "
                          f"{[(m.name, m.object.name if m.object else None) for m in mo.modifiers if m.type == 'ARMATURE']}")
        arm = mods[0].object
    ctx.mesh, ctx.arm = mo, arm


def check_setup(ctx):
    find_objects(ctx)
    need = []
    for x in SIDES:
        s = BSIDE[x]
        need += [f"{b}_{s}" for b in ("Arm", "Forearm", "Hand", "Thigh", "Calf")]
    miss = [n for n in need if n not in ctx.arm.pose.bones]
    if miss:
        raise Blocked(f"{ctx.arm.name} lacks bones {miss}")
    if not isinstance(ctx.parts, dict) or not ctx.parts:
        raise Blocked("parts.json is not a non-empty object")
    ctx.topo = Topo(ctx.mesh, {k: v for k, v in ctx.parts.items() if isinstance(v, int)})
    if ctx.topo.part is None:
        raise Blocked(f"{ctx.mesh.name} part_id FACE INT attribute missing/invalid")
    for x in SIDES:
        for fam in ("arm", "leg"):
            if not len(ctx.topo.polys_of_fams((fam,), x)):
                raise Blocked(f"no faces of part family {fam}_{x}")
    # rings (optional): valid closed loops of this mesh
    rings = (ctx.loops or {}).get("rings") if isinstance(ctx.loops, dict) else None
    if not isinstance(rings, dict):
        ctx.ring_note = "no retopo_loops.json rings -> fallback slice spans"
        return
    t = ctx.topo
    ek = set((t.e.min(axis=1) * t.nv + t.e.max(axis=1)).tolist())
    bad = []
    for name, r in rings.items():
        vs = r.get("verts") if isinstance(r, dict) else None
        try:
            arr = np.array(vs, dtype=np.int64)
        except (TypeError, ValueError):
            bad.append(name)
            continue
        if arr.ndim != 1 or len(arr) < 3 or np.any(arr < 0) or np.any(arr >= t.nv):
            bad.append(name)
            continue
        nx = np.roll(arr, -1)
        if all(k in ek for k in (np.minimum(arr, nx) * t.nv + np.maximum(arr, nx)).tolist()):
            ctx.valid_rings[name] = arr
        else:
            bad.append(name)
    ctx.ring_note = (f"{len(ctx.valid_rings)} rings valid on {ctx.mesh.name}; invalid (not a closed loop of this "
                     f"mesh, ignored): {bad[:20]}")


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


def hip_angles(ctx, x, key):
    hc = ((ctx.p0b or {}).get("skirt") or {}).get("hip_rotation_contact") or {}
    r = hc.get(f"{x}.hip_landmark.{key}")
    c = float(r["angle_deg"]) if isinstance(r, dict) and isinstance(r.get("angle_deg"), (int, float)) else None
    src = f"P0b skirt.hip_rotation_contact {x}.hip_landmark.{key}" if c is not None else "fallback"
    if c is None:
        c = HIP_FALLBACK[key]
    return (round(c / 2.0, 2), round(c, 2), round(c + HIP_PAST, 2)), c, src


def build_poses(ctx):
    arm = ctx.arm
    poses, hip_src = [], {}
    for x in SIDES:
        s = BSIDE[x]
        for kind, spec in POSE_KINDS.items():
            bone = f"{spec['bone']}_{s}"
            M = rest_world(arm, bone)
            R = M.to_3x3().normalized()
            head = np.array(M.translation)
            tail = np.array(arm.matrix_world @ arm.data.bones[bone].tail_local)
            d = unit(tail - head)
            if isinstance(spec["angles"], str):
                angs, c, src = hip_angles(ctx, x, spec["angles"])
                hip_src[f"{kind}_{x}"] = {"contact_deg": c, "source": src, "angles": list(angs)}
            else:
                angs = spec["angles"]
            if spec["target"] is None:
                waxis, tgt = d, None
            else:
                tgt = np.array((1.0 if x == "l" else -1.0, 0.0, 0.0)) if spec["target"] == "out" \
                    else np.array(spec["target"], dtype=np.float64)
                waxis = unit(np.cross(d, tgt))
            laxis = np.array(R.transposed() @ Vector(waxis))
            for a in angs:
                poses.append({"id": f"{kind}_{x}_{a:g}", "kind": kind, "side": x, "joint": spec["joint"],
                              "bone": bone, "angle": float(a), "local_axis": laxis, "world_axis": waxis,
                              "head": head, "tail": tail, "target": tgt})
    return poses, hip_src


def apply_pose(ctx, p):
    reset_pose(ctx.arm)
    if p is not None:
        pb = ctx.arm.pose.bones[p["bone"]]
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion(Vector(p["local_axis"]), math.radians(p["angle"]))
    update()


def deltas(ctx, names):
    out = {}
    for n in names:
        post = ctx.arm.matrix_world @ ctx.arm.pose.bones[n].matrix
        out[n] = mat_np(post @ rest_world(ctx.arm, n).inverted())
    return out


def pivot_of(ctx, joint, x):
    return np.array(rest_world(ctx.arm, f"{JOINTS[joint]['pivot']}_{BSIDE[x]}").translation)


def joint_rings(ctx, joint, x):
    return [n for n in sorted(ctx.valid_rings) if re.match(rf"^{joint}_{x}_(\d+)$", n)]


# ---------------------------------------------------------------- tube slices (goblin G3.3)
class TubeGeo:
    def __init__(self, ctx, joint, x, V0):
        spec = JOINTS[joint]
        arm = ctx.arm
        s = BSIDE[x]
        self.pivot = pivot_of(ctx, joint, x)
        (po, pa), (do, da) = [(f"{o}_{s}", f"{a}_{s}") for o, a in (spec["prox"], spec["dist"])]
        self.owners = sorted({po, do})

        def bone_dir(n):
            b = arm.data.bones[n]
            return unit(np.array(arm.matrix_world @ b.tail_local) - np.array(arm.matrix_world @ b.head_local))

        def bone_x(n):
            return np.array(rest_world(arm, n).to_3x3().normalized().col[0])

        dp, dd = bone_dir(pa), bone_dir(da)
        dm = unit(dp + dd)
        rn = joint_rings(ctx, joint, x)
        self.rings = rn
        if rn:
            s_r = [float(np.dot(V0[ctx.valid_rings[r]].mean(axis=0) - self.pivot, dm)) for r in rn]
            s0, s1 = min(s_r) - SLICE_MARGIN, max(s_r) + SLICE_MARGIN
            self.span_src = f"rings {rn} +- {SLICE_MARGIN * 1000:g} mm"
        else:
            s0, s1 = -SLICE_FALLBACK, SLICE_FALLBACK
            self.span_src = f"fallback +-{SLICE_FALLBACK * 1000:g} mm (no valid {joint}_{x}_* ring)"
        n = int(round((s1 - s0) / SLICE_STEP)) + 1
        self.slices = []
        for sv in np.linspace(s0, s1, n).tolist():
            owner, abone, d = (po, pa, dp) if sv < 0 else (do, da, dd)
            u = bone_x(abone)
            u = unit(u - np.dot(u, d) * d)
            if np.linalg.norm(u) == 0:
                u = unit(np.cross(d, [0.0, 0.0, 1.0]))
            v = np.cross(d, u)
            self.slices.append({"s": sv, "owner": owner, "c": self.pivot + sv * d, "u": u, "v": v})
        self.phi = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False)
        self.tube_polys = ctx.topo.polys_of_fams((spec["tube"],), x)
        self.tube_tris = ctx.topo.tris_of_polys(self.tube_polys)
        self.rest_r = None

    def radii(self, bvh, D=None):
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
                hit = bvh.ray_cast(cv, Vector(math.cos(ph) * u + math.sin(ph) * v), RAY_MAX)
                if hit[0] is not None and (best is None or hit[3] < best):
                    best = hit[3]
            out.append(best)
        return out


def tube_ratio(geo, V, D):
    rp = geo.radii(bvh_of(V, geo.tube_tris_tv), D)
    best, n_miss = None, 0
    for sl, r0, r1 in zip(geo.slices, geo.rest_r, rp):
        if r0 is None or r1 is None:
            n_miss += 1
            continue
        ratio = r1 / r0
        if best is None or ratio < best[0]:
            best = (ratio, sl["s"], r1, r0, sl["owner"])
    return {"min_ratio": rnd(best[0], 4) if best else None, "worst_slice_s_mm": mm(best[1]) if best else None,
            "posed_min_r_mm": mm(best[2]) if best else None, "rest_min_r_mm": mm(best[3]) if best else None,
            "worst_slice_owner": best[4] if best else None, "n_slices": len(geo.slices),
            "n_slices_no_hit": n_miss}


# ---------------------------------------------------------------- intersections
def overlap_pairs(topo, V, tris_a, tris_b=None):
    """{(poly_a, poly_b): [(tri_a, tri_b), ...]} of intersecting triangles; same polygon / shared-vertex polygon
    pairs dropped.  tris_b None = self overlap of tris_a (unordered pairs)."""
    ta = topo.tri_v[tris_a]
    bva = bvh_of(V, ta)
    if tris_b is None:
        tb, bvb, same = ta, bva, True
        tris_b = tris_a
    else:
        tb = topo.tri_v[tris_b]
        bvb, same = bvh_of(V, tb), False
    out = {}
    for i, j in bva.overlap(bvb):
        ga, gb = int(tris_a[i]), int(tris_b[j])
        pa, pb = int(topo.tri_poly[ga]), int(topo.tri_poly[gb])
        if pa == pb:
            continue
        if set(topo.poly_verts[pa]) & set(topo.poly_verts[pb]):
            continue
        if same:
            key = (min(pa, pb), max(pa, pb))
            tt = (ga, gb) if pa < pb else (gb, ga)
        else:
            key, tt = (pa, pb), (ga, gb)
        out.setdefault(key, []).append(tt)
    return out


def classify_pairs(topo, V, pairs, shellset, exclude_parts_fn, designed_fn=None):
    """-> (visible [(key, point)], hidden [(key, container)], designed [(key, point)]).
    designed: designed_fn(key) true (T223 b, skirt ceiling piercing), never visible.  hidden: every intersection
    point of the pair's triangle pairs is inside a closed or virtually capped shell of a part other than the pair's
    own parts (exclude_parts_fn)."""
    vis, hid, des = [], [], []
    for key, tts in pairs.items():
        if designed_fn is not None and designed_fn(key):
            ga, gb = tts[0]
            des.append((key, tri_tri_point(V[topo.tri_v[ga]], V[topo.tri_v[gb]])))
            continue
        excl = exclude_parts_fn(key)
        where = []
        for ga, gb in tts:
            q = tri_tri_point(V[topo.tri_v[ga]], V[topo.tri_v[gb]])
            where.append((q, shellset.inside_any(q, excl)))
        if all(w is not None for _q, w in where):
            hid.append((key, where[0][1]))
        else:
            vis.append((key, next(q for q, w in where if w is None)))
    return vis, hid, des


def pname(topo, poly):
    return topo.part_names.get(int(topo.part[poly]), "?")


def region_polys(ctx, joint, x, V0):
    t = ctx.topo
    piv = pivot_of(ctx, joint, x)
    rn = joint_rings(ctx, joint, x)
    if rn:
        R = float(max(np.linalg.norm(V0[ctx.valid_rings[r]] - piv, axis=1).max() for r in rn)) + RING_R_PAD
        src = f"rings {rn}: max ring vertex distance + {RING_R_PAD * 1000:g} mm"
    else:
        R, src = REGION_R[joint], f"fallback {REGION_R[joint] * 1000:g} mm"
    dp = t.polys_of_fams(DEFORM_FAM)
    cen = t.poly_centroid(V0)
    reg = dp[np.linalg.norm(cen[dp] - piv, axis=1) <= R]
    return reg, R, src


def own_parts(t):
    return lambda k: (pname(t, k[0]), pname(t, k[1]))


def ceiling_fn(ctx):
    """T223 (b): leg face x skirt ceiling face = designed crossing."""
    t = ctx.topo
    ceil = ctx.ceiling_polys

    def f(k):
        a, b = k
        return (a in ceil and family(pname(t, b))[0] == "leg") or (b in ceil and family(pname(t, a))[0] == "leg")
    return f


def pt(q):
    return [round(float(c), 5) for c in q]


def self_isect(ctx, V, reg, shellset):
    t = ctx.topo
    if not len(reg):
        return {"pairs": 0, "visible": 0}
    pairs = overlap_pairs(t, V, t.tris_of_polys(reg))
    vis, hid, des = classify_pairs(t, V, pairs, shellset, own_parts(t), ceiling_fn(ctx))
    n_self = sum(1 for (a, b), _q in vis if t.part[a] == t.part[b])
    by_parts = {}
    for (a, b), _q in vis:
        k = "~".join(sorted((pname(t, a), pname(t, b))))
        by_parts[k] = by_parts.get(k, 0) + 1
    cont = {}
    for _k, c in hid:
        cont[c] = cont.get(c, 0) + 1
    return {"pairs": len(pairs), "visible": len(vis), "visible_self": n_self, "visible_cross": len(vis) - n_self,
            "hidden": len(hid), "hidden_by_container": cont, "designed_ceiling_crossing": len(des),
            "visible_by_parts": by_parts,
            "visible_examples": [{"polys": list(k), "parts": [pname(t, k[0]), pname(t, k[1])], "point_m": pt(q)}
                                 for k, q in vis[:LIST_CAP]]}


class DepthCache:
    """Signed inside depth of a vertex w.r.t. the closed / virtually capped shells of one part (max over shells)."""

    def __init__(self, ctx, V, shellset):
        self.V, self.ss = V, shellset
        self.c = {}

    def __call__(self, v, part):
        k = (v, part)
        if k not in self.c:
            d = -np.inf
            for s in self.ss.shells:
                if s["part"] == part:
                    d = max(d, float(self.ss.depth(s, self.V[[v]])[0]))
            self.c[k] = d
        return self.c[k]


def pair_depth(t, key, dc):
    """penetration depth of a polygon pair: max over the vertices of each polygon of their inside depth in the other
    polygon's part (closed / virtually capped shells); 0 when no vertex is inside."""
    a, b = key
    d = max([dc(v, pname(t, b)) for v in t.poly_verts[a]] + [dc(v, pname(t, a)) for v in t.poly_verts[b]])
    return max(d, 0.0)


def cross_isect(ctx, V, polys_a, polys_b, shellset, with_depth=False):
    t = ctx.topo
    if not len(polys_a) or not len(polys_b):
        return {"pairs": 0, "visible": 0, "note": "a side has no faces"}
    pairs = overlap_pairs(t, V, t.tris_of_polys(polys_a), t.tris_of_polys(polys_b))
    vis, hid, des = classify_pairs(t, V, pairs, shellset, own_parts(t), ceiling_fn(ctx))
    dc = DepthCache(ctx, V, shellset) if with_depth else None

    def per_other(rows, container=False):
        by = {}
        for k, x in rows:
            r = by.setdefault(pname(t, k[1]), {"n": 0})
            r["n"] += 1
            if dc is not None:
                r["max_depth_mm"] = max(r.get("max_depth_mm") or 0.0, mm(pair_depth(t, k, dc)))
            if container:
                r.setdefault("containers", {})
                r["containers"][x] = r["containers"].get(x, 0) + 1
        return by

    out = {"pairs": len(pairs), "visible": len(vis), "hidden": len(hid), "designed_ceiling_crossing": len(des),
           "visible_by_other_part": per_other(vis), "hidden_by_other_part": per_other(hid, True),
           "visible_examples": [{"polys": list(k), "parts": [pname(t, k[0]), pname(t, k[1])], "point_m": pt(q)}
                                for k, q in vis[:LIST_CAP]]}
    if dc is not None:
        vd = [mm(pair_depth(t, k, dc)) for k, _q in vis]
        hd = [mm(pair_depth(t, k, dc)) for k, _c in hid]
        out["visible_max_depth_mm"] = max(vd) if vd else None
        out["hidden_max_depth_mm"] = max(hd) if hd else None
    return out


def verts_inside(ctx, V, verts, shells, shellset):
    """count / max depth of verts inside any of the given closed or virtually capped shells; also the depth array."""
    shells = [s for s in shells if s["closed"] or s.get("virtual")]
    best = np.full(len(verts), -np.inf)
    if not len(verts) or not shells:
        return {"n_verts": int(len(verts)), "inside": 0, "max_depth_mm": None,
                "containers": [s["label"] for s in shells]}, best
    for s in shells:
        best = np.maximum(best, shellset.depth(s, V[verts]))
    ins = best > 0
    return {"n_verts": int(len(verts)), "inside": int(ins.sum()),
            "max_depth_mm": mm(best[ins].max()) if ins.any() else None,
            "containers": [s["label"] for s in shells]}, best


def poke_out(ctx, V, verts, wall_polys):
    """T223 (d): leg vertices radially outside the skirt wall where the skirt exists at that height and azimuth.
    Horizontal ray from the skirt axis point (axis_xy, vertex z) through the vertex against the posed skirt wall
    faces (skirt region without the ceiling), every hit collected (re-cast past each hit, <= POKE_RAY_MAX).
    footprint = the ray hits the wall; poke = footprint and the vertex is beyond the last hit (distance = vertex
    radius - last hit radius); in_wall = odd number of hits beyond the vertex."""
    t = ctx.topo
    ax = ctx.skirt_axis
    if not len(verts) or not len(wall_polys):
        return {"n_tested": int(len(verts))}
    bvh = bvh_of(V, t.tri_v[t.tris_of_polys(wall_polys)])
    n_fp = n_poke = n_wall = 0
    worst, worst_p = 0.0, None
    for v in verts.tolist():
        q = V[v]
        rv = q[:2] - np.asarray(ax, dtype=np.float64)
        r = float(np.linalg.norm(rv))
        if r < 1e-6:
            continue
        d = Vector((rv[0] / r, rv[1] / r, 0.0))
        o, hits, travelled = Vector((ax[0], ax[1], q[2])), [], 0.0
        for _ in range(64):
            h = bvh.ray_cast(o, d, POKE_RAY_MAX - travelled)
            if h[0] is None:
                break
            travelled += h[3]
            hits.append(travelled)
            o = h[0] + d * RAY_EPS
            travelled += RAY_EPS
        if not hits:
            continue
        n_fp += 1
        beyond = [x for x in hits if x > r]
        if not beyond:
            n_poke += 1
            if r - hits[-1] > worst:
                worst, worst_p = r - hits[-1], q
        elif len(beyond) % 2 == 1:
            n_wall += 1
    return {"n_tested": int(len(verts)), "n_in_footprint": n_fp, "n_poke_out": n_poke,
            "max_poke_out_mm": mm(worst) if n_poke else None,
            "worst_point_m": pt(worst_p) if worst_p is not None else None, "n_in_wall": n_wall}


def belt_bottom(ctx, V0):
    bp = ctx.topo.polys_of_fams(("belt",))
    if len(bp):
        return float(V0[ctx.topo.verts_of_polys(bp), 2].min()), "belt part min z (rest)"
    bz = ((ctx.p0b or {}).get("skirt") or {}).get("belt_z")
    if bz:
        return float(bz[0]), "P0b skirt.belt_z[0]"
    return 0.408, "d-01 section 2 z .408"


# ---------------------------------------------------------------- renders (goblib.ortho_render, any view basis)
def view_basis(axis, pivot):
    a = unit(axis)
    if abs(a[1]) >= 0.3:
        d = a if a[1] > 0 else -a                 # camera in front (-Y) looking +Y
    else:
        sx = -1.0 if pivot[0] >= 0 else 1.0       # camera outside the pivot side
        d = a if a[0] * sx > 0 else -a
        if abs(a[0]) < 1e-6:
            d = a
    wup = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.95 else np.array([0.0, 1.0, 0.0])
    right = unit(np.cross(d, wup))
    up = unit(np.cross(right, d))
    return d, right, up


def ortho_cell(objs, d, right, up, center, half, depth, out_png, colors):
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    src_scene = bpy.context.scene
    sc = bpy.data.scenes.new("_p2_render")
    cam_data = cam_obj = wire_mat = None
    temp_objs, temp_meshes, saved = [], [], {}
    try:
        sc.frame_current = src_scene.frame_current
        for o in objs:
            sc.collection.objects.link(o)
        cam_data = bpy.data.cameras.new("_p2_cam")
        cam_data.type = "ORTHO"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.ortho_scale = 2 * half
        dist = 2 * depth + 1.0
        cam_data.clip_start = 0.001
        cam_data.clip_end = dist + 2 * depth + 1.0
        cam_obj = bpy.data.objects.new("_p2_cam", cam_data)
        rot = Matrix((Vector(right), Vector(up), Vector(-d))).transposed()
        cam_obj.matrix_world = Matrix.Translation(Vector(center - d * dist)) @ rot.to_4x4()
        sc.collection.objects.link(cam_obj)
        sc.camera = cam_obj
        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = r.resolution_y = CELL_PX
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "OBJECT"
        sh.show_xray = False
        for name, col in colors.items():
            o = bpy.data.objects[name]
            saved[name] = tuple(o.color)
            o.color = tuple(col)
        px = 2 * half / CELL_PX
        wire_mat = bpy.data.materials.new("_p2_wire")
        wire_mat.diffuse_color = (0.0, 0.0, 0.0, 1.0)
        for o in objs:
            me = o.data.copy()
            me.materials.clear()
            me.materials.append(wire_mat)
            temp_meshes.append(me)
            wo = bpy.data.objects.new("_p2_wire_" + o.name, me)
            wo.color = (0.0, 0.0, 0.0, 1.0)
            sc.collection.objects.link(wo)
            temp_objs.append(wo)
            mod = wo.modifiers.new("wire", "WIREFRAME")
            mod.thickness = 1.2 * px
            mod.use_boundary = True
            mod.use_replace = True
            mod.use_even_offset = True
        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
        for name, col in saved.items():
            bpy.data.objects[name].color = col
        for wo in temp_objs:
            bpy.data.objects.remove(wo, do_unlink=True)
        for me in temp_meshes:
            bpy.data.meshes.remove(me)
        if wire_mat is not None:
            bpy.data.materials.remove(wire_mat)
        if cam_obj is not None:
            bpy.data.objects.remove(cam_obj, do_unlink=True)
        if cam_data is not None:
            bpy.data.cameras.remove(cam_data)
        bpy.data.scenes.remove(sc)
    return out_png


def render_cell(ctx, V, p, out_png):
    t = ctx.topo
    joint = p["joint"]
    piv = pivot_of(ctx, joint, p["side"])
    d, right, up = view_basis(p["world_axis"] if p["kind"] != "wrist_twist" else np.array([0.0, 1.0, 0.0]), piv)
    h, dep = CELL_HALF[joint], CELL_DEPTH_KIND.get(p["kind"], CELL_DEPTH[joint])
    cen = t.poly_centroid(V) - piv
    inbox = (np.abs(cen @ right) <= h) & (np.abs(cen @ up) <= h) & (np.abs(cen @ d) <= dep)
    hidden = np.zeros(t.npoly, bool)
    hidden[t.polys_of_fams(CELL_HIDDEN_FAM.get(joint, ("head", "eye")))] = True
    rigid = np.zeros(t.npoly, bool)
    rigid[t.polys_of_fams(RIGID_FAM)] = True
    groups = {"_p2_cell_deform": np.nonzero(inbox & ~rigid & ~hidden)[0],
              "_p2_cell_rigid": np.nonzero(inbox & rigid & ~hidden)[0]}
    objs, meshes, colors = [], [], {}
    try:
        for name, polys in groups.items():
            if not len(polys):
                continue
            faces = [t.poly_verts[int(q)] for q in polys.tolist()]
            used = np.unique(np.concatenate([np.array(f) for f in faces]))
            remap = {int(o): i for i, o in enumerate(used.tolist())}
            me = bpy.data.meshes.new(name)
            me.from_pydata(V[used].tolist(), [], [[remap[i] for i in f] for f in faces])
            me.update()
            meshes.append(me)
            ob = bpy.data.objects.new(name, me)
            bpy.context.scene.collection.objects.link(ob)
            objs.append(ob)
            colors[ob.name] = DEFORM_COLOR if name.endswith("deform") else RIGID_COLOR
        if objs:
            ortho_cell(objs, d, right, up, piv, h, dep, out_png, colors)
    finally:
        for ob in objs:
            bpy.data.objects.remove(ob, do_unlink=True)
        for me in meshes:
            bpy.data.meshes.remove(me)
    return out_png if objs else None


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
    return px[::-1]


def compose_sheet(cells, out_png):
    """cells: rows of png paths (None = empty); alpha over white, CELL_GAP px grey gaps (goblin)."""
    imgs = [[load_rgba(p) if p is not None and Path(p).exists() else None for p in row] for row in cells]
    ch = cw = CELL_PX
    nr, nc = len(imgs), max(len(r) for r in imgs)
    H = nr * ch + (nr + 1) * CELL_GAP
    W = nc * cw + (nc + 1) * CELL_GAP
    sheet = np.full((H, W, 4), 1.0, dtype=np.float32)
    sheet[:, :, :3] = 0.55
    for i, row in enumerate(imgs):
        for j, im in enumerate(row):
            y0, x0 = CELL_GAP + i * (ch + CELL_GAP), CELL_GAP + j * (cw + CELL_GAP)
            sheet[y0:y0 + ch, x0:x0 + cw, :3] = 1.0 if im is not None else 0.85
            if im is None:
                continue
            a = im[:ch, :cw, 3:4]
            sheet[y0:y0 + a.shape[0], x0:x0 + a.shape[1], :3] = im[:ch, :cw, :3] * a + (1.0 - a)
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img = bpy.data.images.new("_p2_sheet", W, H, alpha=True)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(sheet[::-1]).ravel())
        img.filepath_raw = str(out_png)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)
    return out_png


# ---------------------------------------------------------------- measurement
def measure(ctx, cell_dir):
    arm, mo, t = ctx.arm, ctx.mesh, ctx.topo
    if arm.animation_data is not None and arm.animation_data.action is not None:
        ctx.notes.append(f"armature action {arm.animation_data.action.name} unassigned in memory")
        arm.animation_data.action = None
    if arm.data.pose_position != "POSE":
        ctx.notes.append(f"armature pose_position {arm.data.pose_position} -> POSE in memory")
        arm.data.pose_position = "POSE"
    ncon = sum(len(pb.constraints) for pb in arm.pose.bones)
    if ncon:
        ctx.notes.append(f"{ncon} pose-bone constraints present (left as is)")
    apply_pose(ctx, None)
    V0 = eval_verts(mo)
    if len(V0) != t.nv:
        raise Blocked(f"evaluated {mo.name} has {len(V0)} verts, obj.data has {t.nv}")
    ctx.V0 = V0
    rigid_shells = [s for s in t.shells if s["fam"] in RIGID_FAM]
    hide_shells = [s for s in t.shells if s["fam"] in RIGID_FAM or s.get("virtual")]   # T223 (a)
    ss0_rigid = ShellSet(t, V0, hide_shells)
    ss0_all = ShellSet(t, V0, t.shells)
    tun = (ctx.loops or {}).get("tunic") if isinstance(ctx.loops, dict) else None
    tun = tun if isinstance(tun, dict) else {}
    cz = tun.get("ceiling_z")
    ceil_src = "none (retopo_loops.json tunic.ceiling_z missing)"
    if isinstance(cz, (int, float)):
        tp = t.polys_of_fams(("tunic", "skirt"))
        ctx.ceiling_polys = {int(q) for q in tp.tolist()
                             if np.all(np.abs(V0[t.poly_verts[int(q)], 2] - cz) <= CEIL_TOL)}
        ceil_src = f"tunic / skirt faces with every rest vertex within {CEIL_TOL * 1000:g} mm of tunic.ceiling_z {cz}"
    axy = tun.get("axis_xy")
    if isinstance(axy, list) and len(axy) == 2:
        ctx.skirt_axis, ax_src = (float(axy[0]), float(axy[1])), "retopo_loops.json tunic.axis_xy"
    else:
        c = (((ctx.p0b or {}).get("skirt") or {}).get("hem_ellipse") or {}).get("center_xy") or [0.0, 0.0]
        ctx.skirt_axis, ax_src = (float(c[0]), float(c[1])), "P0b skirt.hem_ellipse.center_xy"
    tubes = {}
    for j in ("elbow", "knee", "wrist"):
        for x in SIDES:
            g = TubeGeo(ctx, j, x, V0)
            g.tube_tris_tv = t.tri_v[g.tube_tris]
            g.rest_r = g.radii(bvh_of(V0, g.tube_tris_tv))
            tubes[(j, x)] = g
    regions = {(j, x): region_polys(ctx, j, x, V0) for j in ("elbow", "knee", "shoulder", "hip", "wrist")
               for x in SIDES}
    zb, zb_src = belt_bottom(ctx, V0)
    skirt_polys = t.polys_of_fams(("tunic", "skirt"))
    skirt_polys = skirt_polys[t.poly_centroid(V0)[skirt_polys, 2] < zb]
    skirt_shells = [s for s in t.shells if s["fam"] in ("tunic", "skirt") and (s["closed"] or s.get("virtual"))]
    others_sh = ("tunic", "skirt", "scarf", "scarf_tail")
    res = {"rest": {}, "poses": {}, "shoulder": {}, "hip": {}}
    # rest baselines
    res["rest"]["self_isect_all_deforming"] = self_isect(ctx, V0, t.polys_of_fams(DEFORM_FAM), ss0_rigid)
    res["rest"]["regions"] = {f"{j}_{x}": {"radius_mm": mm(R), "radius_source": src, "n_faces": int(len(reg)),
                                          "self_isect": self_isect(ctx, V0, reg, ss0_rigid)}
                              for (j, x), (reg, R, src) in regions.items()}
    for x in SIDES:
        sl = t.polys_of_fams(("sleeve",), x)
        oth = t.polys_of_fams(others_sh)
        res["rest"][f"sleeve_{x}"] = shoulder_metrics(ctx, V0, sl, oth, x, ss0_all)
        leg = t.polys_of_fams(("leg",), x)
        res["rest"][f"leg_{x}"] = hip_metrics(ctx, V0, leg, skirt_polys, skirt_shells, ss0_all)
    poses, hip_src = build_poses(ctx)
    cells = {}
    for p in poses:
        apply_pose(ctx, p)
        V = eval_verts(mo)
        x, j = p["side"], p["joint"]
        ss_rigid = ShellSet(t, V, hide_shells)
        ss_all = ShellSet(t, V, t.shells)
        rec = {"bone": p["bone"], "angle_deg": p["angle"], "world_axis": [rnd(c, 5) for c in p["world_axis"]],
               "local_axis": [rnd(c, 5) for c in p["local_axis"]]}
        owners = {p["bone"]}
        if p["kind"] in TUBE_KINDS:
            owners |= set(tubes[(TUBE_KINDS[p["kind"]], x)].owners)
        D = deltas(ctx, owners)
        Db = D[p["bone"]]
        if p["target"] is not None:
            q = Db[:3, :3] @ p["tail"] + Db[:3, 3]
            disp = q - p["tail"]
            rec["check"] = {"moved_point": "bone tail", "target_dir": [rnd(c, 4) for c in p["target"]],
                            "disp_mm": [mm(c) for c in disp],
                            "disp_dir_dot_target": rnd(float(np.dot(unit(disp), p["target"])), 5)}
            rec["ok"] = rec["check"]["disp_dir_dot_target"] > 0
        else:
            Rr = rest_world(arm, p["bone"]).to_3x3().normalized()
            Rp = (arm.matrix_world @ arm.pose.bones[p["bone"]].matrix).to_3x3().normalized()
            ang_y = vec_angle_deg(np.array(Rr.col[1]), np.array(Rp.col[1]))
            ang_x = vec_angle_deg(np.array(Rr.col[0]), np.array(Rp.col[0]))
            rec["check"] = {"bone_Y_axis_change_deg": rnd(ang_y, 5), "bone_X_axis_turn_deg": rnd(ang_x, 4)}
            rec["ok"] = ang_y <= 0.01 and abs(ang_x - p["angle"]) <= 0.01
        others_moved = [n for n, pb in arm.pose.bones.items() if n != p["bone"] and (
            pb.rotation_quaternion.angle > 1e-9 if pb.rotation_mode == "QUATERNION" else False)]
        rec["other_bones_rotated"] = others_moved
        rec["ok"] = rec["ok"] and not others_moved
        if p["kind"] in TUBE_KINDS:
            rec["tube"] = tube_ratio(tubes[(TUBE_KINDS[p["kind"]], x)], V, D)
        reg, _R, _src = regions[(j, x)]
        rec["self_isect"] = self_isect(ctx, V, reg, ss_rigid)
        if j == "shoulder":
            rec["sleeve_vs_tunic_scarf"] = shoulder_metrics(ctx, V, t.polys_of_fams(("sleeve",), x),
                                                            t.polys_of_fams(others_sh), x, ss_all)
        if j == "hip":
            rec["leg_vs_skirt"] = hip_metrics(ctx, V, t.polys_of_fams(("leg",), x), skirt_polys, skirt_shells, ss_all)
        res["poses"][p["id"]] = rec
        out = cell_dir / f"{p['id']}.png"
        cells[(p["kind"], x, p["angle"])] = render_cell(ctx, V, p, out)
        print(f"[{GATE}/{CHECKER}] pose {p['id']}: ratio={rec.get('tube', {}).get('min_ratio')} "
              f"visible_isect={rec['self_isect'].get('visible')}")
        sys.stdout.flush()
    apply_pose(ctx, None)
    res["meta"] = {"belt_bottom_z_mm": mm(zb), "belt_bottom_source": zb_src, "skirt_faces": int(len(skirt_polys)),
                   "closed_skirt_shells": [s["label"] for s in skirt_shells if s["closed"]],
                   "virtually_capped_shells": [{"shell": s["label"], "rims": [int(len(rv)) for rv, _e in s["rims"]]}
                                               for s in t.shells if s.get("virtual")],
                   "hiding_shells_P2.2c": [s["label"] for s in hide_shells if s["closed"] or s.get("virtual")],
                   "ceiling": {"faces": len(ctx.ceiling_polys), "z": cz, "source": ceil_src},
                   "skirt_axis_xy": list(ctx.skirt_axis), "skirt_axis_source": ax_src, "hip_angles": hip_src,
                   "tubes": {f"{j}_{x}": {"span": g.span_src, "n_slices": len(g.slices), "tube_faces":
                                          int(len(g.tube_polys)),
                                          "rest_min_r_mm": [mm(r) for r in g.rest_r]} for (j, x), g in tubes.items()},
                   "rigid_closed_shells": [s["label"] for s in rigid_shells if s["closed"]],
                   "rigid_open_shells": [s["label"] for s in rigid_shells if not s["closed"]],
                   "rings": ctx.ring_note}
    return res, poses, cells


def shoulder_metrics(ctx, V, sleeve, others, x, ss):
    """T223 (c): sleeve x tunic / skirt / scarf / scarf_tail pairs split visible / hidden (crossing point inside a
    closed or virtually capped shell of a third part) with the pair penetration depth (pair_depth)."""
    t = ctx.topo
    sl_sh = [s for s in t.shells if s["fam"] == "sleeve" and family(s["part"])[1] == x]
    ot_sh = [s for s in t.shells if s["fam"] in ("tunic", "skirt", "scarf", "scarf_tail")]
    return {"pairs": cross_isect(ctx, V, sleeve, others, ss, with_depth=True),
            "sleeve_verts_inside_tunic_scarf": verts_inside(ctx, V, t.verts_of_polys(sleeve), ot_sh, ss)[0],
            "tunic_scarf_verts_inside_sleeve": verts_inside(ctx, V, t.verts_of_polys(others), sl_sh, ss)[0]}


def hip_metrics(ctx, V, leg, skirt_polys, skirt_shells, ss):
    """T223 (b, d): leg x skirt pairs (ceiling crossing = designed, separate); leg vertices inside the virtually
    capped tunic split by rest z at the ceiling (above = leg top inside the torso, designed; below = in the skirt
    wall); leg vertices poking out through the skirt wall (poke_out, vertices with rest z below the ceiling)."""
    t = ctx.topo
    lv = t.verts_of_polys(leg)
    rec, depth = verts_inside(ctx, V, lv, skirt_shells, ss)
    cz = (ctx.loops or {}).get("tunic", {}).get("ceiling_z") if isinstance(ctx.loops, dict) else None
    zc = float(cz) if isinstance(cz, (int, float)) else np.inf
    below = ctx.V0[lv, 2] < zc
    ins = depth > 0
    split = {"above_ceiling_designed": int((ins & ~below).sum()),
             "below_ceiling_in_skirt_wall": int((ins & below).sum()),
             "below_ceiling_max_depth_mm": mm(depth[ins & below].max()) if (ins & below).any() else None,
             "split_rule": "leg vertex rest z vs retopo_loops.json tunic.ceiling_z"}
    walls = np.array(sorted(set(skirt_polys.tolist()) - ctx.ceiling_polys), dtype=np.int64)
    return {"pairs": cross_isect(ctx, V, leg, skirt_polys, ss),
            "leg_verts_inside_tunic_skirt": {**rec, **split},
            "leg_poke_out_of_skirt_wall": poke_out(ctx, V, lv[below], walls)}


# ---------------------------------------------------------------- rows
def judged(p):
    return p["angle"] in JUDGED_ANGLES and p["kind"] in ("elbow", "knee", "shoulder_raise", "shoulder_fwd")


def write_rows(put, ctx, res, poses):
    pr = res["poses"]
    put("P2.2a", {"n_poses": len(pr), "poses": {k: {kk: v[kk] for kk in ("bone", "angle_deg", "world_axis",
                                                                           "local_axis", "check",
                                                                           "other_bones_rotated", "ok")}
                                               for k, v in pr.items()},
                  "hip_angles": res["meta"]["hip_angles"], "notes": ctx.notes},
        all(v["ok"] for v in pr.values()) and len(pr) == sum(
            (3 if not isinstance(s["angles"], tuple) else len(s["angles"])) for s in POSE_KINDS.values()) * 2,
        threshold="each pose: only the listed bone rotated; moved tail displacement . target > 0 (twist: bone Y "
                  "axis unchanged <= 0.01 deg, X axis turned by the angle +-0.01 deg); 44 poses",
        note="world axis = unit(d x target), local = R_rest^T world; hip angles (c/2, c, c+15) from the P0b contact")
    for j in ("elbow", "knee", "wrist"):
        for x in SIDES:
            kind = "wrist_twist" if j == "wrist" else j
            rows = {p["id"]: pr[p["id"]]["tube"] for p in poses if p["kind"] == kind and p["side"] == x}
            jv = [v["min_ratio"] for k, v in rows.items() if j != "wrist" and float(k.rsplit("_", 1)[1]) in JUDGED_ANGLES]
            rv = [v["min_ratio"] for k, v in rows.items() if j == "wrist" or float(k.rsplit("_", 1)[1]) not in
                  JUDGED_ANGLES]
            m = {"min_30_60": min((v for v in jv if v is not None), default=None),
                 "min_report": min((v for v in rv if v is not None), default=None),
                 "draft_threshold": RATIO_MIN, "meets_draft_30_60": bool(jv) and all(
                     v is not None and v >= RATIO_MIN for v in jv) if j != "wrist" else None,
                 "poses": rows, "slices": res["meta"]["tubes"][f"{j}_{x}"]}
            put(f"P2.2b_{j}_{x}", m, True)
    for j in ("elbow", "knee", "shoulder", "hip", "wrist"):
        for x in SIDES:
            sel = [p for p in poses if p["joint"] == j and p["side"] == x]
            rows = {p["id"]: pr[p["id"]]["self_isect"] for p in sel}
            jv = [pr[p["id"]]["self_isect"]["visible"] for p in sel if judged(p)]
            m = {"max_visible_30_60": max(jv) if jv else None,
                 "max_visible_all": max((v["visible"] for v in rows.values()), default=None),
                 "draft_threshold": ISECT_MAX if jv else "report",
                 "meets_draft_30_60": (all(v <= ISECT_MAX for v in jv) if jv else None),
                 "rest": res["rest"]["regions"][f"{j}_{x}"], "poses": rows}
            put(f"P2.2c_{j}_{x}", m, True)
    put("P2.2c_rest", res["rest"]["self_isect_all_deforming"], True)
    for x in SIDES:
        sel = [p for p in poses if p["joint"] == "shoulder" and p["side"] == x]
        rows = {p["id"]: pr[p["id"]]["sleeve_vs_tunic_scarf"] for p in sel}
        put(f"P2.2d_shoulder_{x}", {"max_visible_pairs": max((v["pairs"]["visible"] for v in rows.values()),
                                                             default=None),
                                    "max_visible_depth_mm": max((v["pairs"].get("visible_max_depth_mm") or 0.0
                                                                 for v in rows.values()), default=None),
                                    "max_hidden_pairs": max((v["pairs"]["hidden"] for v in rows.values()),
                                                            default=None),
                                    "max_hidden_depth_mm": max((v["pairs"].get("hidden_max_depth_mm") or 0.0
                                                                for v in rows.values()), default=None),
                                    "rest": res["rest"][f"sleeve_{x}"], "poses": rows}, True)
        sel = [p for p in poses if p["joint"] == "hip" and p["side"] == x]
        rows = {p["id"]: pr[p["id"]]["leg_vs_skirt"] for p in sel}
        put(f"P2.2e_hip_{x}", {"max_visible_pairs": max((v["pairs"]["visible"] for v in rows.values()), default=None),
                               "max_designed_ceiling_crossing": max((v["pairs"].get("designed_ceiling_crossing", 0)
                                                                     for v in rows.values()), default=None),
                               "max_leg_verts_in_skirt_wall": max((v["leg_verts_inside_tunic_skirt"]
                                                                   ["below_ceiling_in_skirt_wall"]
                                                                   for v in rows.values()), default=None),
                               "max_leg_verts_poke_out": max((v["leg_poke_out_of_skirt_wall"].get("n_poke_out", 0)
                                                              for v in rows.values()), default=None),
                               "max_poke_out_mm": max((v["leg_poke_out_of_skirt_wall"].get("max_poke_out_mm") or 0.0
                                                       for v in rows.values()), default=None),
                               "rest": res["rest"][f"leg_{x}"], "belt_bottom_z_mm": res["meta"]["belt_bottom_z_mm"],
                               "skirt_faces": res["meta"]["skirt_faces"],
                               "closed_skirt_shells": res["meta"]["closed_skirt_shells"],
                               "virtually_capped_shells": res["meta"]["virtually_capped_shells"],
                               "ceiling": res["meta"]["ceiling"], "skirt_axis_xy": res["meta"]["skirt_axis_xy"],
                               "poses": rows}, True)


ROW_DOC = {
    "P2.2b": ("(draft) tube cross-section ratio (posed / rest min slice radius) >= 0.8 at 30 / 60 deg (elbow, knee); "
              "90 deg and wrist twist 60 report", CALIB + "; slices / rays as goblin G3.3 (module doc)"),
    "P2.2c": ("(draft) visible self-intersecting deforming-face pairs in the joint region = 0 at 30 / 60 deg (elbow, "
              "knee, shoulder raise / fwd); 90 deg, hip and wrist poses report",
              CALIB + "; T223: hidden = every crossing point inside a closed rigid shell or the virtually capped "
                      "tunic (open rims fan-capped in memory), the pair's own parts excluded; leg x skirt-ceiling "
                      "pairs = designed_ceiling_crossing, not visible; self = one part, cross = two deforming parts, "
                      "both in 'visible'"),
    "P2.2d": ("(report) shoulder raise / fwd: sleeve vs tunic / skirt / scarf / scarf_tail intersecting pairs, "
              "visible (crossing point outside every closed / virtually capped shell of a third part) vs hidden, "
              "each with max penetration depth; vertices inside the other's shells",
              CALIB + "; T223 (c): pair depth = max inside depth of either polygon's vertices in the other "
                      "polygon's part (closed / virtually capped shells), 0 when none is inside"),
    "P2.2e": ("(report) hip flex / abd / ext to and past the P0b contact angle: leg vs skirt faces (below the belt "
              "bottom) intersecting pairs (visible / hidden / designed ceiling crossing), leg vertices inside the "
              "virtually capped tunic (above ceiling = designed, below = in the skirt wall) and leg vertices poking "
              "out through the skirt wall, raw temp-rig weights",
              CALIB + "; T223 (b, d), poke rule in poke_out docstring"),
    "P2.2f": ("[U] joint close-up sheet written (ok = sheet file written; the user judges it)",
              "rows = elbow, knee, shoulder_raise, shoulder_fwd, hip_flex, hip_abd, hip_ext, wrist_twist; "
              "columns = side l angles 1-3 | side r angles 1-3; grey = deforming, blue = rigid, head/eyes hidden "
              "(hip rows also arm / fist / sleeve, which hang in front of the hip in the side view); "
              "camera along the pose rotation axis (front / outside)"),
}
ROW_ORDER = ["P2.2a", "P2.2b", "P2.2c", "P2.2d", "P2.2e", "P2.2f"]
SHEET_ROWS = ["elbow", "knee", "shoulder_raise", "shoulder_fwd", "hip_flex", "hip_abd", "hip_ext", "wrist_twist"]


def doc_of(cid):
    base = cid.split("_")[0]
    return ROW_DOC.get(base, ("", ""))


class Evidence:
    def __init__(self):
        self.inputs, self.criteria, self.renders = [], [], []
        self.extra = {}

    def add_input(self, path):
        r = rel(path)
        if Path(path).exists() and not any(i["path"] == r for i in self.inputs):
            self.inputs.append({"path": r, "sha256": sha256(path)})

    def render(self, path):
        self.renders.append({"path": rel(path), "sha256": sha256(path)})

    def criterion(self, cid, measured, threshold, ok, note):
        self.criteria.append({"id": cid, "ok": bool(ok), "measured": jsonable(measured),
                              "threshold": threshold, "note": note})

    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        self.criteria.sort(key=lambda c: (ROW_ORDER.index(c["id"].split("_")[0])
                                          if c["id"].split("_")[0] in ROW_ORDER else 99))
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               **jsonable(self.extra), "inputs": self.inputs, "renders": self.renders, "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            s = json.dumps(c["measured"], ensure_ascii=False)
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={s[:400]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--blend": "blend", "--parts": "parts", "--loops": "loops", "--p0b": "p0b", "--mesh": "mesh",
            "--armature": "armature", "--out": "out"}
    i = 0
    while i < len(argv):
        if argv[i] == "--in-memory":
            P["in_memory"] = True
            i += 1
            continue
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <value>' for k in keys)} [--in-memory]")
        k = keys[argv[i]]
        P[k] = argv[i + 1] if k in ("mesh", "armature") else Path(argv[i + 1])
        i += 2


def all_row_ids():
    ids = ["P2.2a"]
    ids += [f"P2.2b_{j}_{x}" for j in ("elbow", "knee", "wrist") for x in SIDES]
    ids += [f"P2.2c_{j}_{x}" for j in ("elbow", "knee", "shoulder", "hip", "wrist") for x in SIDES] + ["P2.2c_rest"]
    ids += [f"P2.2d_shoulder_{x}" for x in SIDES] + [f"P2.2e_hip_{x}" for x in SIDES] + ["P2.2f"]
    return ids


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ctx = Ctx()
    done = set()
    ids = all_row_ids()

    def put(cid, measured, ok, threshold=None, note=None):
        t, n = doc_of(cid)
        ev.criterion(cid, measured, threshold if threshold is not None else t, ok, note if note is not None else n)
        done.add(cid)

    def block(cids, reason):
        for c in cids:
            if c not in done:
                put(c, f"blocked: {reason}", False, None if c != "P2.2a" else "pose set applied",
                    None if c != "P2.2a" else "")

    miss = []
    tmp = None
    try:
        if P["in_memory"]:
            ev.extra["in_memory_self_test"] = {"loaded_blend": bpy.data.filepath or "(unsaved session)"}
        elif Path(P["blend"]).exists():
            bpy.ops.wm.open_mainfile(filepath=str(P["blend"]), load_ui=False)
            ev.add_input(P["blend"])
        else:
            miss.append(rel(P["blend"]))
        for key in ("parts", "loops", "p0b"):
            if Path(P[key]).exists():
                ev.add_input(P[key])
            elif key != "loops":
                miss.append(rel(P[key]))
        for p in sorted(HERE.glob("p2*.py")):
            ev.add_input(p)
        _STATE["missing"] = miss
        if miss:
            block(ids, "missing input " + ", ".join(miss))
            return
        raw = load_json(P["parts"])
        ctx.parts = raw["part_id"] if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict) else raw
        ctx.loops = load_json(P["loops"]) if Path(P["loops"]).exists() else None
        ctx.p0b = load_json(P["p0b"])
        try:
            check_setup(ctx)
        except Blocked as e:
            block(ids, str(e))
            return
        ev.extra["objects"] = {"mesh": ctx.mesh.name, "armature": ctx.arm.name, "rings": ctx.ring_note}
        tmp = Path(tempfile.mkdtemp(prefix="p2_cells_"))
        try:
            res, poses, cells = measure(ctx, tmp)
        except Blocked as e:
            block(ids, str(e))
            return
        write_rows(put, ctx, res, poses)
        grid = []
        for kind in SHEET_ROWS:
            row = []
            for x in SIDES:
                angs = sorted({a for (k, s, a) in cells if k == kind and s == x})
                row += [cells.get((kind, x, a)) for a in angs] + [None] * (3 - len(angs))
            grid.append(row)
        sheet = compose_sheet(grid, Path(P["out"]).parent / SHEET_NAME)
        ev.render(sheet)
        layout = {"rows": SHEET_ROWS,
                  "cols": {kind: {x: sorted(a for (k, s, a) in cells if k == kind and s == x) for x in SIDES}
                           for kind in SHEET_ROWS},
                  "empty_cells": [f"{k}_{s}_{a:g}" for (k, s, a), v in cells.items() if v is None]}
        put("P2.2f", {"sheet": rel(sheet), "layout": layout, "cell_px": CELL_PX,
                      "crop_half_mm": {k: mm(v) for k, v in CELL_HALF.items()},
                      "crop_depth_mm": {k: mm(v) for k, v in CELL_DEPTH.items()},
                      "crop_depth_by_pose_kind_mm": {k: mm(v) for k, v in CELL_DEPTH_KIND.items()}}, sheet.exists())
    finally:
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)
        try:
            block(ids, "checker aborted before this criterion: " + tb_tail(2) if sys.exc_info()[0] else
                  "checker aborted before this criterion")
        finally:
            ev.write()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(2)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
