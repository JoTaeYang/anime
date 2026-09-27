"""check_p2_static - Player P2.1 retopo-static checker (task T221; spec work/player/d-02-player-rig.md section 1
defaults, section 2 gate P2.1, checker side).

Inputs (defaults; override after "--"):
  --blend  work/player/rig/pl_r01_retopo.blend      one MESH object with a FACE INT attribute part_id
  --parts  work/player/rig/data/parts.json           {part name: part_id} or {"part_id": {part name: part_id}, ...}
  --loops  work/player/rig/data/retopo_loops.json    {"mesh": name, "rings": {name: {"verts": [...], ...}},
                                                     "tunic": {"outer_rows" / "inner_rows": [{name, verts}]}}
  --p0b    work/player/inspect/P0b/p0b_measure.json  source part table (centroid, tris), landmarks, skirt
  --src    C:/Users/whxod/Downloads/Meshy_AI_Clay_Explorer_0926122001_generate.fbx  (read-only; imported in memory)
  --mesh   <object name>   default: the only MESH object with a FACE INT part_id
  --zone   work/player/rig/data/thigh_thin_zone.json  (T258, optional): its vertices are a design-change region excluded
           from the P2.1h/i deviation both ways and reported in P2.1i; absent -> behaviour unchanged
  --out    work/player/inspect/P2/check_p2_static.json
  --in-memory              self-test seam: use the data already loaded in this Blender session (no --blend open)

Methods copied / adapted from the goblin checkers (not imported; goblin files untouched):
  check_g2_static.py  MeshArrays, shells (vertex-connected faces of one part_id), ray-parity inside depth,
                      winding pairs + signed volume (G2.3), buried faces (G2.4), ring validation (G2.5),
                      UV islands / 2048 raster overlap / texel density (G2.9), custom normals (G2.10)
  check_g2_shape.py   area-weighted samples, hidden (> 1 mm inside another closed shell of the same side) and
                      interface (< 0.1 mm from another closed shell) exclusion, two-way nearest-surface
                      deviation, per-object judgement (T121f) -> here per part: every retopo part is compared
                      only with its own source part
  check_p1.py         evidence format (inputs + sha256, criteria rows id / ok / measured / threshold / note)
Part classes (spec section 1): rigid = head, eye, fist, shoe, cuff, belt, pouch, scarf, scarf_tail, sleeve (source
  unchanged -> deviation judged); deforming = arm, leg, tunic, skirt (rebuilt -> deviation report).  Part name ->
  family = name without a trailing _l / _r (side).  Source parts = FBX loose components matched to the P0b part
  table (vertex centroid <= 10 mm, same triangle count), placed with the P0b transform.
Rows marked (report) or with a draft number hold a first-run measured distribution: ok = True, note
"calibration run: threshold set after measurement".  A row that cannot be measured is "blocked: ..." ok False.
Nothing is saved: blends are opened read-only in memory; the FBX objects are removed again; only the evidence JSON
is written.
Run:    blender --background --factory-startup --python check_p2_static.py [-- <overrides>]
Output: work/player/inspect/P2/check_p2_static.json
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z = 0.  Lengths in m, reported in mm.
"""
import hashlib
import json
import math
import re
import sys
import traceback
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

HERE = Path(__file__).resolve().parent
PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.1", "check_p2_static", "T221"
CALIB = "calibration run: threshold set after measurement"

P = {"blend": PLAYER_RIG / "pl_r01_retopo.blend",
     "parts": PLAYER_RIG / "data" / "parts.json",
     "loops": PLAYER_RIG / "data" / "retopo_loops.json",
     "p0b": REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json",
     "src": Path(r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Explorer_0926122001_generate.fbx"),
     "zone": PLAYER_RIG / "data" / "thigh_thin_zone.json",
     "mesh": None,
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_static.json",
     "in_memory": False}

# ---------------------------------------------------------------- part classes (spec section 1)
RIGID_FAM = ("head", "eye", "fist", "shoe", "cuff", "belt", "pouch", "scarf", "scarf_tail", "sleeve")
DEFORM_FAM = ("arm", "leg", "tunic", "skirt")
FAM_ALIAS = {"eyes": "eye", "fists": "fist", "hand": "fist", "shoes": "shoe", "cuffs": "cuff", "sleeves": "sleeve"}
SRC_OF_FAM = {"skirt": "tunic"}       # retopo family -> source family when renamed (skirt was part of the tunic)
# designed overlaps (not buried-face defects), from P0b proportions.rest_overlaps_tri_pairs (+ skirt = tunic);
# when both parts are sided, only the same side
DESIGNED = (("leg", "tunic"), ("leg", "skirt"), ("leg", "belt"), ("leg", "cuff"), ("leg", "shoe"),
            ("cuff", "shoe"), ("arm", "sleeve"), ("arm", "fist"), ("arm", "tunic"), ("sleeve", "tunic"),
            ("sleeve", "scarf"), ("head", "tunic"), ("head", "scarf"), ("scarf", "tunic"), ("scarf_tail", "tunic"),
            ("scarf_tail", "scarf"), ("belt", "tunic"), ("belt", "skirt"), ("skirt", "tunic"), ("pouch", "belt"),
            ("pouch", "tunic"), ("pouch", "skirt"), ("eye", "head"))

TRI_MAX = 7000              # P2.1e draft (spec section 1 / 2 italic) -> calibration
INTERIOR_DEPTH = 0.002      # P2.1d (goblin G2.4)
UV_RES = 2048               # P2.1f (goblin G2.9)
UV_EPS = 1e-6
UV_MATCH = 1e-6
UV_AREA_EPS = 1e-12         # P2.1f coverage: face UV area > this
HEAD_FRONT_DOT = 0.7
DENSITY_RATIO = 1.5
TH_DEV = {"mean_mm": 1.0, "p95_mm": 2.0, "max_mm": 3.0}   # P2.1h (goblin G2.11)
N_SAMPLES = 80_000          # per direction, split over the parts by area
MIN_PART_SAMPLES = 1500
SEED = 0
HIDE_DEPTH = 0.001
IFACE_TOL = 0.0001
TOP_N = 10
SRC_MATCH_TOL = 0.010       # m: FBX component vertex centroid vs P0b part centroid
SKIRT_AZ = {"right(-X)": 180.0, "front(-Y)": -90.0, "left(+X)": 0.0, "back(+Y)": 90.0, "front-left": -45.0,
            "front-right": -135.0, "back-left": 45.0, "back-right": 135.0}
SKIRT_AZ_HALF = 10.0        # deg window for the hem radius by azimuth (report)
CAP_BAND = 0.010            # m: P0b bottom-cap definition (normal z < -0.9 within 10 mm of the bottom)
LIST_CAP = 40
SIDES = ("l", "r")
RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237), (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6


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
    if isinstance(v, (list, tuple, set)) or isinstance(v, Vector):
        return [jsonable(x) for x in v]
    return str(v)


def mm(x):
    if x is None:
        return None
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=5):
    if x is None:
        return None
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def tb_tail(n=4):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def cap(lst):
    return list(lst)[:LIST_CAP]


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def parts_map(raw):
    """parts.json -> ({name: part_id}, other fields).  Accepts {name: int} or {"part_id": {name: int}, ...}."""
    if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict):
        return raw["part_id"], {k: v for k, v in raw.items() if k != "part_id"}
    return raw, {}


def family(name):
    """part name -> (family, side or None)."""
    m = re.match(r"^(.*?)(?:_([lr]))?$", str(name).lower())
    fam, side = m.group(1), m.group(2)
    return FAM_ALIAS.get(fam, fam), side


def part_class(name):
    fam, _s = family(name)
    return "rigid" if fam in RIGID_FAM else "deforming" if fam in DEFORM_FAM else "unclassified"


def designed_pair(a, b):
    (fa, sa), (fb, sb) = family(a), family(b)
    if sa and sb and sa != sb:
        return False
    return (fa, fb) in DESIGNED or (fb, fa) in DESIGNED


def stats(d):
    d = np.asarray(d, dtype=np.float64)
    if not len(d):
        return {"n": 0, "mean_mm": None, "p95_mm": None, "max_mm": None}
    return {"n": int(len(d)), "mean_mm": mm(d.mean()), "p95_mm": mm(np.percentile(d, 95)), "max_mm": mm(d.max())}


def dev_ok(st):
    return (st["n"] > 0 and st["mean_mm"] <= TH_DEV["mean_mm"] and st["p95_mm"] <= TH_DEV["p95_mm"]
            and st["max_mm"] <= TH_DEV["max_mm"])


def _components(n, a, b):
    """Union-find over n nodes with edges (a[i], b[i]); root per node (goblin check_g2_static)."""
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


# ---------------------------------------------------------------- mesh arrays (goblin check_g2_static)
class MeshArrays:
    """World-space arrays of obj.data (no modifier evaluation, original indices)."""

    def __init__(self, obj):
        me = obj.data
        self.obj, self.me, self.name = obj, me, obj.name
        mw = np.array(obj.matrix_world, dtype=np.float64)
        nv, ne, npoly, nl = len(me.vertices), len(me.edges), len(me.polygons), len(me.loops)
        self.nv, self.ne, self.npoly, self.nl = nv, ne, npoly, nl
        co = np.empty(nv * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
        self.v = co.reshape(nv, 3).astype(np.float64) @ mw[:3, :3].T + mw[:3, 3]
        e = np.empty(ne * 2, dtype=np.int32)
        me.edges.foreach_get("vertices", e)
        self.e = e.reshape(ne, 2).astype(np.int64)
        ls = np.empty(npoly, dtype=np.int32)
        lt = np.empty(npoly, dtype=np.int32)
        me.polygons.foreach_get("loop_start", ls)
        me.polygons.foreach_get("loop_total", lt)
        ls, lt = ls.astype(np.int64), lt.astype(np.int64)
        lv = np.empty(nl, dtype=np.int32)
        le = np.empty(nl, dtype=np.int32)
        me.loops.foreach_get("vertex_index", lv)
        me.loops.foreach_get("edge_index", le)
        self.loop_vert, self.loop_edge = lv.astype(np.int64), le.astype(np.int64)
        offs = np.concatenate(([0], np.cumsum(lt)[:-1])) if npoly else np.zeros(0, np.int64)
        if not np.array_equal(ls, offs):
            raise Blocked(f"{obj.name}: polygon loops are not contiguous")
        self.loop_start, self.loop_total = ls, lt
        self.loop_poly = np.repeat(np.arange(npoly), lt)
        idx = np.arange(nl)
        st = ls[self.loop_poly]
        self.loop_next = st + (idx - st + 1) % lt[self.loop_poly]
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        tv = np.empty(nt * 3, dtype=np.int32)
        tl = np.empty(nt * 3, dtype=np.int32)
        tp = np.empty(nt, dtype=np.int32)
        me.loop_triangles.foreach_get("vertices", tv)
        me.loop_triangles.foreach_get("loops", tl)
        me.loop_triangles.foreach_get("polygon_index", tp)
        self.tri_v = tv.reshape(nt, 3).astype(np.int64)
        self.tri_l = tl.reshape(nt, 3).astype(np.int64)
        self.tri_poly = tp.astype(np.int64)
        self.ntri = nt
        T = self.v[self.tri_v]
        cr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        self.tri_area = 0.5 * np.linalg.norm(cr, axis=1)
        pn = np.zeros((npoly, 3))
        np.add.at(pn, self.tri_poly, cr)
        ln = np.linalg.norm(pn, axis=1)
        ln[ln == 0] = 1.0
        self.poly_normal = pn / ln[:, None]
        self.poly_area = np.bincount(self.tri_poly, weights=self.tri_area, minlength=npoly)
        cen = np.zeros((npoly, 3))
        np.add.at(cen, self.loop_poly, self.v[self.loop_vert])
        self.poly_center = cen / np.maximum(lt, 1)[:, None]
        self.v_list = [tuple(p) for p in self.v.tolist()]
        self.part = None
        att = me.attributes.get("part_id")
        self.part_attr = None if att is None else {"domain": att.domain, "data_type": att.data_type}
        if att is not None and att.domain == "FACE" and att.data_type == "INT":
            pv = np.empty(npoly, dtype=np.int32)
            att.data.foreach_get("value", pv)
            self.part = pv.astype(np.int64)
        self.uv, self.uv_name, self.n_uv_layers = None, None, len(me.uv_layers)
        uvl = me.uv_layers.active
        if uvl is not None:
            buf = np.empty(nl * 2, dtype=np.float32)
            try:
                uvl.uv.foreach_get("vector", buf)
            except (AttributeError, TypeError, RuntimeError):
                uvl.data.foreach_get("uv", buf)
            self.uv = buf.reshape(nl, 2).astype(np.float64)
            self.uv_name = uvl.name


class Shell:
    def __init__(self, ma, sid, part, faces, shell_of_face):
        self.ma, self.sid, self.part, self.faces = ma, sid, part, faces
        self.idx = 0
        self.corners = np.nonzero(shell_of_face[ma.loop_poly] == sid)[0]
        self.verts = np.unique(ma.loop_vert[self.corners])
        ue, inv, c = np.unique(ma.loop_edge[self.corners], return_inverse=True, return_counts=True)
        self.n_boundary = int((c == 1).sum())
        self.n_nonmanifold = int((c > 2).sum())
        self.closed = self.n_boundary == 0 and self.n_nonmanifold == 0
        self.boundary_corners = self.corners[(c == 1)[inv.reshape(-1)]]
        self.tris = np.nonzero(shell_of_face[ma.tri_poly] == sid)[0]
        Pv = ma.v[self.verts]
        self.mn, self.mx = Pv.min(axis=0), Pv.max(axis=0)
        self._bvh = None

    @property
    def label(self):
        return f"{self.part}#{self.idx}"

    def bvh(self):
        if self._bvh is None:
            self._bvh = BVHTree.FromPolygons(self.ma.v_list, self.ma.tri_v[self.tris].tolist(), all_triangles=True)
        return self._bvh

    def rims(self):
        """Boundary loops: list of (vertex index array, [(a, b) boundary edges in face traversal order])."""
        ma = self.ma
        bc = self.boundary_corners
        if not len(bc):
            return []
        a = ma.loop_vert[bc]
        b = ma.loop_vert[ma.loop_next[bc]]
        vs = np.unique(np.concatenate([a, b]))
        loc = {int(v): i for i, v in enumerate(vs.tolist())}
        la = np.array([loc[int(x)] for x in a.tolist()])
        lb = np.array([loc[int(x)] for x in b.tolist()])
        root = _components(len(vs), la, lb)
        out = []
        for r in np.unique(root).tolist():
            mv = root == r
            em = mv[la]
            out.append((vs[mv], list(zip(a[em].tolist(), b[em].tolist()))))
        return out

    def signed_volume(self, capped=False):
        """Divergence-theorem volume, origin at the shell vertex mean; capped: every rim closed by a fan to its
        vertex mean (cap winding follows the rim edges reversed, so a consistently wound open tube gets the sign
        of the closed tube)."""
        T = self.ma.v[self.ma.tri_v[self.tris]]
        if not len(T):
            return 0.0
        o = self.ma.v[self.verts].mean(axis=0)
        T = T - o
        vol = float(np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6.0)
        if capped:
            for vs, edges in self.rims():
                c = self.ma.v[vs].mean(axis=0) - o
                for a, b in edges:
                    pa, pb = self.ma.v[a] - o, self.ma.v[b] - o
                    vol += float(np.dot(pb, np.cross(pa, c)) / 6.0)
        return vol

    def inconsistent_pairs(self):
        ma, c = self.ma, self.corners
        e = ma.loop_edge[c]
        sg = ma.loop_vert[c] < ma.loop_vert[ma.loop_next[c]]
        order = np.argsort(e, kind="stable")
        es, sgs = e[order], sg[order]
        _, start, cnt = np.unique(es, return_index=True, return_counts=True)
        two = cnt == 2
        return int((sgs[start[two]] == sgs[start[two] + 1]).sum())


def build_shells(ma, face_group, group_names):
    """Shells = vertex-connected face components inside each group value (goblin check_g2_static)."""
    key = face_group[ma.loop_poly] * ma.nv + ma.loop_vert
    order = np.argsort(key, kind="stable")
    k, f = key[order], ma.loop_poly[order]
    same = k[1:] == k[:-1]
    roots = _components(ma.npoly, f[:-1][same], f[1:][same])
    _, shell_of_face = np.unique(roots, return_inverse=True)
    shell_of_face = shell_of_face.reshape(-1)
    shells = []
    for sid in range(int(shell_of_face.max()) + 1 if ma.npoly else 0):
        faces = np.nonzero(shell_of_face == sid)[0]
        g = int(face_group[faces[0]])
        shells.append(Shell(ma, sid, group_names.get(g, f"part_{g}"), faces, shell_of_face))
    by_part = {}
    for s in shells:
        by_part.setdefault(s.part, []).append(s)
    for lst in by_part.values():
        lst.sort(key=lambda s: -len(s.faces))
        for i, s in enumerate(lst):
            s.idx = i
    return shells, shell_of_face


def shell_depth(shell, pts):
    """Signed inside depth (m) of pts (n,3) w.r.t. shell: + inside, - outside (goblin)."""
    bvh = shell.bvh()
    out = np.empty(len(pts))
    inb = np.all((pts >= shell.mn) & (pts <= shell.mx), axis=1)
    for i, p in enumerate(np.asarray(pts).tolist()):
        pv = Vector(p)
        loc, _, _, dist = bvh.find_nearest(pv)
        if loc is None:
            out[i] = -np.inf
            continue
        out[i] = dist if (inb[i] and _inside(bvh, pv)) else -dist
    return out


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.mesh_obj = self.ma = None
        self.mesh_err = ""
        self.parts = None
        self.parts_meta = {}
        self.part_names = {}
        self.part_err = ""
        self.shells, self.shell_of_face = [], None
        self.loops = self.p0b = None
        self.src = None           # dict from load_source
        self.src_err = ""
        self.dev = None           # deviation results (P2.1h/i/j)
        self.zone_ids = None      # T258 thigh-thin zone vertex ids (None = no manifest)
        self.zone_info = None
        self.blend_note = None


def find_mesh():
    if P["mesh"]:
        o = bpy.data.objects.get(P["mesh"])
        if o is None or o.type != "MESH":
            raise Blocked(f"--mesh {P['mesh']!r} not found / not a MESH")
        return o, [P["mesh"]]
    cands = []
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        a = o.data.attributes.get("part_id")
        if a is not None and a.domain == "FACE" and a.data_type == "INT":
            cands.append(o)
    names = [o.name for o in cands]
    if len(cands) != 1:
        raise Blocked(f"expected exactly one MESH object with a FACE INT part_id, found {names} "
                      f"(all meshes: {[o.name for o in bpy.data.objects if o.type == 'MESH']})")
    return cands[0], names


def setup(ctx):
    try:
        mo, _c = find_mesh()
    except Blocked as e:
        ctx.mesh_err = str(e)
        return
    ctx.mesh_obj = mo
    ctx.ma = MeshArrays(mo)
    if ctx.ma.part is None:
        ctx.part_err = f"{mo.name} part_id FACE INT attribute missing/invalid ({ctx.ma.part_attr})"
    pj = ctx.parts
    if not isinstance(pj, dict):
        ctx.part_err = ctx.part_err or "parts.json is not an object"
        return
    bad = {k: v for k, v in pj.items() if not isinstance(v, int) or isinstance(v, bool)}
    if bad:
        ctx.part_err = ctx.part_err or f"parts.json non-int values {bad}"
        return
    ctx.part_names = {v: k for k, v in pj.items()}
    if ctx.ma.part is not None and ctx.ma.npoly:
        ctx.shells, ctx.shell_of_face = build_shells(ctx.ma, ctx.ma.part, ctx.part_names)


def shells_of_fam(ctx, fams):
    return [s for s in ctx.shells if family(s.part)[0] in fams]


def part_faces(ctx, name):
    v = (ctx.parts or {}).get(name)
    if v is None or ctx.ma is None or ctx.ma.part is None:
        return np.zeros(0, np.int64)
    return np.nonzero(ctx.ma.part == v)[0]


def fam_faces(ctx, fams):
    if ctx.ma is None or ctx.ma.part is None:
        return np.zeros(0, np.int64)
    vals = [v for k, v in (ctx.parts or {}).items() if family(k)[0] in fams]
    return np.nonzero(np.isin(ctx.ma.part, vals))[0]


def base_block(ctx):
    r = [x for x in (ctx.mesh_err, ctx.part_err) if x]
    return "; ".join(r)


# ---------------------------------------------------------------- P2.1a
def p21a(ctx):
    pj = ctx.parts
    ma = ctx.ma
    vals, cnt = np.unique(ma.part, return_counts=True)
    exp = set(pj.values())
    present = set(int(v) for v in vals)
    p0b_names = sorted((ctx.p0b or {}).get("parts", {}) or [])
    cls = {n: part_class(n) for n in pj}
    tp = ma.part[ma.tri_poly]
    rec = {"mesh_object": ma.name, "part_id_attribute": ma.part_attr, "parts_json": pj,
           "faces_per_value": {str(int(v)): {"name": ctx.part_names.get(int(v)), "faces": int(c),
                                             "tris": int((tp == v).sum())} for v, c in zip(vals, cnt)},
           "missing_values": sorted(exp - present), "unknown_values": sorted(present - exp),
           "duplicate_values": sorted({v for v in pj.values() if list(pj.values()).count(v) > 1}),
           "class_by_part": cls, "unclassified_parts": sorted(n for n, c in cls.items() if c == "unclassified"),
           "names_vs_p0b_report": {"p0b_parts_not_in_parts_json": sorted(set(p0b_names) - set(pj)),
                                   "parts_json_not_in_p0b": sorted(set(pj) - set(p0b_names))},
           "other_objects_report": sorted(f"{o.name}({o.type})" for o in bpy.data.objects if o != ctx.mesh_obj),
           "modifiers_report": [m.type for m in ctx.mesh_obj.modifiers]}
    meta = ctx.parts_meta or {}
    if meta:
        mine_r = sorted(n for n, c in cls.items() if c == "rigid")
        mine_d = sorted(n for n, c in cls.items() if c == "deforming")
        rec["parts_json_other_fields_report"] = {
            "mesh": meta.get("mesh"), "mesh_matches_checked_mesh": meta.get("mesh") in (None, ma.name),
            "rigid_list_vs_checker": {"only_parts_json": sorted(set(meta.get("rigid") or []) - set(mine_r)),
                                      "only_checker": sorted(set(mine_r) - set(meta.get("rigid") or []))},
            "rebuilt_list_vs_checker_deforming": {"only_parts_json": sorted(set(meta.get("rebuilt") or []) - set(mine_d)),
                                                  "only_checker": sorted(set(mine_d) - set(meta.get("rebuilt") or []))}}
    ok = not rec["missing_values"] and not rec["unknown_values"] and not rec["duplicate_values"] and len(pj) > 0
    return rec, ok


# ---------------------------------------------------------------- P2.1b / c
def closed_shells_except(ctx, shell):
    return [s for s in ctx.shells if s.closed and s is not shell]


def p21b(ctx):
    rows, ok = [], True
    n_open = n_nm = 0
    for s in ctx.shells:
        row = {"shell": s.label, "class": part_class(s.part), "faces": int(len(s.faces)),
               "boundary_edges": s.n_boundary, "nonmanifold_edges": s.n_nonmanifold, "closed": s.closed}
        if s.n_nonmanifold:
            n_nm += 1
            ok = False
        if s.n_boundary:
            n_open += 1
            rims = []
            others = closed_shells_except(ctx, s)
            for vs, edges in s.rims():
                P_ = ctx.ma.v[vs]
                best = np.full(len(vs), -np.inf)
                where = [None] * len(vs)
                for o in others:
                    if np.any(o.mx < P_.min(axis=0)) or np.any(o.mn > P_.max(axis=0)):
                        continue
                    d = shell_depth(o, P_)
                    for i in np.nonzero(d > best)[0].tolist():
                        where[i] = o.label
                    best = np.maximum(best, d)
                hidden = bool(len(vs) and np.all(best > 0))
                cont = sorted({w for w, b in zip(where, best.tolist()) if w is not None and b > 0})
                rims.append({"n_verts": int(len(vs)), "n_edges": len(edges), "hidden": hidden,
                             "min_depth_mm": mm(best.min()) if len(vs) else None,
                             "median_depth_mm": mm(np.median(best)) if len(vs) else None,
                             "verts_not_inside": int((best <= 0).sum()), "containers": cont,
                             "z_range_mm": [mm(P_[:, 2].min()), mm(P_[:, 2].max())]})
                ok = ok and hidden
            row["rims"] = rims
        rows.append(row)
    used = np.zeros(ctx.ma.ne, bool)
    used[ctx.ma.loop_edge] = True
    return {"n_shells": len(rows), "n_open_shells": n_open, "n_shells_with_nonmanifold": n_nm,
            "wire_edges_report": int((~used).sum()), "shells": rows}, ok


def p21c(ctx):
    rows, neg, inc_total = [], [], 0
    for s in ctx.shells:
        vol = s.signed_volume(capped=not s.closed)
        inc = s.inconsistent_pairs()
        rows.append({"shell": s.label, "closed": s.closed, "volume_cm3" + ("" if s.closed else "_rim_capped"):
                     round(vol * 1e6, 4), "inconsistent_pairs": inc})
        if not vol > 0:
            neg.append(s.label)
        inc_total += inc
    return {"n_shells": len(rows), "nonpositive_volume": neg, "inconsistent_pairs_total": inc_total,
            "shells": rows}, not neg and inc_total == 0


# ---------------------------------------------------------------- P2.1d
def p21d(ctx):
    ma = ctx.ma
    closed = [s for s in ctx.shells if s.closed]
    pairs, total, excluded, tested = [], 0, {}, 0
    for A in ctx.shells:
        for B in closed:
            if A is B:
                continue
            lo, hi = B.mn + INTERIOR_DEPTH, B.mx - INTERIOR_DEPTH
            if np.any(A.mx < lo) or np.any(A.mn > hi):
                continue
            Pv = ma.v[A.verts]
            cand = np.all((Pv >= lo) & (Pv <= hi), axis=1)
            if not cand.any():
                continue
            tested += 1
            depth = np.full(ma.nv, -np.inf)
            depth[A.verts[cand]] = shell_depth(B, Pv[cand])
            deep = depth[ma.loop_vert[A.corners]] > INTERIOR_DEPTH
            n_deep = np.bincount(ma.loop_poly[A.corners][deep], minlength=ma.npoly)
            faces = A.faces[n_deep[A.faces] == ma.loop_total[A.faces]]
            if not len(faces):
                continue
            des = A.part != B.part and designed_pair(A.part, B.part)
            total += len(faces)
            if des:
                key = f"{A.part} in {B.part}"
                excluded[key] = excluded.get(key, 0) + len(faces)
            pairs.append({"face_shell": A.label, "inside_shell": B.label, "faces": int(len(faces)),
                          "designed_overlap": des, "sample_faces": cap(faces.tolist())})
    remaining = total - sum(excluded.values())
    return {"buried_faces": total, "excluded_designed_overlap": excluded, "remaining_buried_faces": remaining,
            "pairs": pairs, "shell_pairs_tested": tested,
            "designed_overlap_families": [f"{a}~{b}" for a, b in DESIGNED],
            "closed_shells_used_as_container": [s.label for s in closed],
            "open_shells_not_containers": [s.label for s in ctx.shells if not s.closed]}, remaining == 0


# ---------------------------------------------------------------- P2.1e
def p21e(ctx):
    ma = ctx.ma
    tp = ma.part[ma.tri_poly]
    per = {n: int((tp == v).sum()) for n, v in ctx.parts.items()}
    by_cls = {}
    for n, c in per.items():
        k = part_class(n)
        by_cls[k] = by_cls.get(k, 0) + c
    return {"tris": ma.ntri, "draft_max": TRI_MAX, "within_draft": ma.ntri <= TRI_MAX, "per_part": per,
            "per_class": by_cls, "source_tris_p0b": ((ctx.p0b or {}).get("totals") or {}).get("tris"),
            "faces": ma.npoly, "verts": ma.nv}, True


# ---------------------------------------------------------------- P2.1f
def uv_islands(ma):
    """Island id per face: faces joined across edges whose two endpoint UVs match in both faces (goblin)."""
    order = np.argsort(ma.loop_edge, kind="stable")
    es = ma.loop_edge[order]
    _, start, cnt = np.unique(es, return_index=True, return_counts=True)
    two = cnt == 2
    c1, c2 = order[start[two]], order[start[two] + 1]
    n1, n2 = ma.loop_next[c1], ma.loop_next[c2]
    uv = ma.uv
    a1, a2 = ma.loop_vert[c1], ma.loop_vert[c2]
    rev = a1 != a2
    u2_a1 = np.where(rev[:, None], uv[n2], uv[c2])
    u2_b1 = np.where(rev[:, None], uv[c2], uv[n2])
    match = (np.abs(uv[c1] - u2_a1).max(axis=1) <= UV_MATCH) & (np.abs(uv[n1] - u2_b1).max(axis=1) <= UV_MATCH)
    f1, f2 = ma.loop_poly[c1][match], ma.loop_poly[c2][match]
    roots = _components(ma.npoly, f1, f2)
    _, isl = np.unique(roots, return_inverse=True)
    return isl.reshape(-1)


def uv_overlap_pixels(ma, island):
    """Pixels (UV_RES^2, pixel-centre sampling) covered by triangles of >= 2 different islands (goblin)."""
    owner = np.full((UV_RES, UV_RES), -1, dtype=np.int32)
    conflict = np.zeros((UV_RES, UV_RES), dtype=bool)
    T = ma.uv[ma.tri_l] * UV_RES
    tri_isl = island[ma.tri_poly]
    degenerate = 0
    conflict_pairs = {}
    for t in range(ma.ntri):
        p = T[t]
        if not np.all(np.isfinite(p)):
            degenerate += 1
            continue
        area2 = (p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[1, 1] - p[0, 1]) * (p[2, 0] - p[0, 0])
        if abs(area2) < 1e-12:
            degenerate += 1
            continue
        x0 = max(int(math.floor(p[:, 0].min() - 0.5)), 0)
        x1 = min(int(math.ceil(p[:, 0].max() - 0.5)), UV_RES - 1)
        y0 = max(int(math.floor(p[:, 1].min() - 0.5)), 0)
        y1 = min(int(math.ceil(p[:, 1].max() - 0.5)), UV_RES - 1)
        if x1 < x0 or y1 < y0:
            continue
        xs = np.arange(x0, x1 + 1) + 0.5
        ys = np.arange(y0, y1 + 1) + 0.5
        X, Y = np.meshgrid(xs, ys)
        s = 1.0 if area2 > 0 else -1.0
        inside = np.ones(X.shape, bool)
        for i in range(3):
            q0, q1 = p[i], p[(i + 1) % 3]
            w = (q1[0] - q0[0]) * (Y - q0[1]) - (q1[1] - q0[1]) * (X - q0[0])
            inside &= (s * w) >= 0
        if not inside.any():
            continue
        isl = int(tri_isl[t])
        sub = owner[y0:y1 + 1, x0:x1 + 1]
        csub = conflict[y0:y1 + 1, x0:x1 + 1]
        hit = inside & (sub >= 0) & (sub != isl)
        if hit.any():
            for o in np.unique(sub[hit]).tolist():
                k = (min(o, isl), max(o, isl))
                conflict_pairs[k] = conflict_pairs.get(k, 0) + int((sub[hit] == o).sum())
        csub[hit] = True
        sub[inside & (sub < 0)] = isl
    return int(conflict.sum()), degenerate, int((owner >= 0).sum()), conflict_pairs


def p21f(ctx):
    ma = ctx.ma
    if ma.uv is None:
        return {"uv": "no active UV layer", "n_uv_layers": ma.n_uv_layers}, False
    uv = ma.uv
    finite = np.all(np.isfinite(uv), axis=1)
    out = ~finite | np.any(uv < -UV_EPS, axis=1) | np.any(uv > 1.0 + UV_EPS, axis=1)
    isl = uv_islands(ma)
    ov, degen, covered, cpairs = uv_overlap_pixels(ma, isl)
    tri_uv = uv[ma.tri_l]
    a2 = ((tri_uv[:, 1, 0] - tri_uv[:, 0, 0]) * (tri_uv[:, 2, 1] - tri_uv[:, 0, 1])
          - (tri_uv[:, 1, 1] - tri_uv[:, 0, 1]) * (tri_uv[:, 2, 0] - tri_uv[:, 0, 0]))
    poly_uv_area = np.bincount(ma.tri_poly, weights=0.5 * np.abs(np.nan_to_num(a2)), minlength=ma.npoly)
    uncovered = np.nonzero(~(poly_uv_area > UV_AREA_EPS))[0]
    head_vals = [v for k, v in ctx.parts.items() if family(k)[0] == "head"]
    face_vals = [v for k, v in ctx.parts.items() if family(k)[0] in ("head", "eye")]
    head = np.isin(ma.part, head_vals) & (ma.poly_normal @ np.array([0.0, -1.0, 0.0]) > HEAD_FRONT_DOT)
    body = ~np.isin(ma.part, face_vals)
    dens = {}
    for key, m in (("head_front", head), ("body", body)):
        A3, Auv = float(ma.poly_area[m].sum()), float(poly_uv_area[m].sum())
        dens[key] = {"faces": int(m.sum()), "area3d_m2": round(A3, 6), "uv_area": round(Auv, 6),
                     "density_uv_per_m2": (Auv / A3) if A3 > 0 else None}
    dh, db = dens["head_front"]["density_uv_per_m2"], dens["body"]["density_uv_per_m2"]
    ratio = (dh / db) if (dh is not None and db) else None
    per_part = {}
    for n, v in ctx.parts.items():
        m = ma.part == v
        A3, Auv = float(ma.poly_area[m].sum()), float(poly_uv_area[m].sum())
        per_part[n] = {"density_uv_per_m2": round(Auv / A3, 4) if A3 > 0 else None,
                       "uv_islands": int(len(np.unique(isl[m]))) if m.any() else 0}
    # left/right (report): conflict pixel hits summed over island pairs where one island holds a *_l part and the
    # other a *_r part (a pixel covered by 3+ islands counts once per pair, so this can exceed overlap_pixels)
    isl_side = {}
    for n, v in ctx.parts.items():
        s = family(n)[1]
        for i in np.unique(isl[ma.part == v]).tolist():
            isl_side.setdefault(i, set()).add(s)
    lr = sum(c for (i, j), c in cpairs.items()
             if ("l" in isl_side.get(i, ()) and "r" in isl_side.get(j, ()))
             or ("r" in isl_side.get(i, ()) and "l" in isl_side.get(j, ())))
    top_pairs = sorted(cpairs.items(), key=lambda kv: -kv[1])[:20]
    isl_part = {}
    for n, v in ctx.parts.items():
        for i in np.unique(isl[ma.part == v]).tolist():
            isl_part.setdefault(i, []).append(n)
    rec = {"uv_layer": ma.uv_name, "n_uv_layers": ma.n_uv_layers,
           "faces": ma.npoly, "faces_without_uv_area": int(len(uncovered)), "sample_faces_without_uv_area":
               cap(uncovered.tolist()),
           "loops_out_of_0_1": int(out.sum()), "nonfinite_loops": int((~finite).sum()),
           "uv_min": uv[finite].min(axis=0).tolist() if finite.any() else None,
           "uv_max": uv[finite].max(axis=0).tolist() if finite.any() else None,
           "islands": int(isl.max()) + 1 if len(isl) else 0, "overlap_pixels": ov,
           "overlap_pixel_hits_left_vs_right_island_pairs": lr,
           "overlap_island_pairs_top": [{"islands": list(k), "parts": [isl_part.get(k[0]), isl_part.get(k[1])],
                                         "pixels": c} for k, c in top_pairs],
           "degenerate_uv_tris": degen, "covered_pixels": covered, "raster": UV_RES,
           "texel_density": {**dens, "ratio_area": ratio,
                             "ratio_linear_sqrt": math.sqrt(ratio) if ratio is not None and ratio >= 0 else None},
           "per_part_report": per_part}
    ok = (len(uncovered) == 0 and rec["loops_out_of_0_1"] == 0 and ov == 0 and ratio is not None
          and ratio >= DENSITY_RATIO)
    return rec, ok


# ---------------------------------------------------------------- P2.1g
def p21g(ctx):
    me = ctx.mesh_obj.data
    nl = len(me.loops)
    has = getattr(me, "has_custom_normals", None)
    attr = me.attributes.get("custom_normal")
    buf = np.empty(nl * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", buf)
    cn = buf.reshape(nl, 3)
    nonfinite = int((~np.all(np.isfinite(cn), axis=1)).sum())
    ln = np.linalg.norm(np.nan_to_num(cn), axis=1)
    rec = {"has_custom_normals": has,
           "custom_normal_attribute": None if attr is None else {"domain": attr.domain, "data_type": attr.data_type},
           "corners": nl, "nan_or_inf_corner_normals": nonfinite, "zero_length_corner_normals_report":
               int((ln < 1e-6).sum())}
    return rec, has is True and nonfinite == 0


# ---------------------------------------------------------------- source (FBX, read-only, in memory)
def components_tri(nv, tris):
    lab = np.arange(nv, dtype=np.int64)
    if not len(tris):
        return lab
    e = tris[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2)
    return _components(nv, e[:, 0], e[:, 1])


def load_source(ctx):
    """Import the FBX into the current session, read world triangles, match loose components to the P0b part
    table, remove every imported datablock again."""
    src = Path(P["src"])
    if not src.exists():
        raise Blocked(f"missing input {src.as_posix()}")
    p0b = ctx.p0b or {}
    parts = p0b.get("parts") or {}
    if not parts:
        raise Blocked("P0b part table missing")
    sha = sha256(src)
    kinds = ("objects", "meshes", "materials", "images", "textures", "armatures", "actions", "cameras", "lights",
             "collections")
    before = {k: set(getattr(bpy.data, k)) for k in kinds}
    try:
        res = bpy.ops.import_scene.fbx(filepath=str(src))
    except Exception:
        raise Blocked(f"FBX import failed: {tb_tail(2)}")
    new_objs = [o for o in bpy.data.objects if o not in before["objects"]]
    vs, ts = [], []
    nv = 0
    try:
        if "FINISHED" not in res:
            raise Blocked(f"FBX import returned {sorted(res)}")
        for o in new_objs:
            if o.type != "MESH":
                continue
            me = o.data
            me.calc_loop_triangles()
            co = np.empty(len(me.vertices) * 3, dtype=np.float64)
            me.vertices.foreach_get("co", co)
            mw = np.array(o.matrix_world, dtype=np.float64)
            v = co.reshape(-1, 3) @ mw[:3, :3].T + mw[:3, 3]
            t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
            me.loop_triangles.foreach_get("vertices", t)
            vs.append(v)
            ts.append(t.reshape(-1, 3) + nv)
            nv += len(v)
        mesh_names = [o.name for o in new_objs if o.type == "MESH"]
    finally:
        for k in kinds:
            coll = getattr(bpy.data, k)
            for idb in [x for x in coll if x not in before[k]]:
                try:
                    coll.remove(idb)
                except (ReferenceError, RuntimeError):
                    pass
    if not vs:
        raise Blocked("FBX has no mesh")
    V = np.concatenate(vs)
    T = np.concatenate(ts)
    sc = float(p0b.get("scale_factor", 1.0) or 1.0)
    off = np.array(p0b.get("offset") or [0.0, 0.0, 0.0], dtype=np.float64)
    V = sc * V + off
    lab = components_tri(len(V), T)
    tlab = lab[T[:, 0]]
    comps = np.unique(tlab)
    names, match, used = {}, [], set()
    for c in comps.tolist():
        tm = tlab == c
        vi = np.unique(T[tm])
        cen = V[vi].mean(axis=0)
        best = min(parts, key=lambda n: float(np.linalg.norm(np.array(parts[n]["centroid"]) - cen)))
        d = float(np.linalg.norm(np.array(parts[best]["centroid"]) - cen))
        ok = d <= SRC_MATCH_TOL and int(tm.sum()) == int(parts[best].get("tris", -1)) and best not in used
        match.append({"component_tris": int(tm.sum()), "p0b_part": best, "centroid_dist_mm": mm(d), "matched": ok})
        if ok:
            used.add(best)
            names[c] = best
    unmatched = [m for m in match if not m["matched"]]
    labels = sorted(set(names.values()))
    tri_label = np.array([labels.index(names[c]) if c in names else -1 for c in tlab.tolist()], dtype=np.int64)
    # shells = components; closed = every edge used by exactly 2 triangles
    _, tri_shell = np.unique(tlab, return_inverse=True)
    tri_shell = tri_shell.reshape(-1)
    e = np.sort(T[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    key = e[:, 0] * max(1, len(V)) + e[:, 1]
    _uk, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    bad_tri = (cnt[inv.reshape(-1)] != 2).reshape(-1, 3).any(axis=1)
    n_sh = int(tri_shell.max()) + 1
    closed = np.ones(n_sh, dtype=bool)
    closed[np.unique(tri_shell[bad_tri])] = False
    info = {"path": src.as_posix(), "sha256": sha, "p0b_sha256": p0b.get("source_sha256"),
            "sha256_matches_p0b": sha == p0b.get("source_sha256"), "imported_meshes": mesh_names,
            "verts": int(len(V)), "tris": int(len(T)), "components": int(len(comps)),
            "matched_parts": len(used), "unmatched_components": unmatched,
            "p0b_parts_without_component": sorted(set(parts) - used),
            "transform": f"p = {sc:g} * p_fbx_world + {off.tolist()} (P0b)", "closed_components": int(closed.sum())}
    ctx.src = {"v": V, "t": T, "tri_label": tri_label, "labels": labels, "tri_shell": tri_shell,
               "shell_closed": closed, "info": info}


# ---------------------------------------------------------------- deviation (goblin check_g2_shape, per part)
class Soup:
    def __init__(self, v, t, tri_shell, shell_closed):
        self.v, self.t = v, t
        self.tri_shell, self.shell_closed = tri_shell, shell_closed
        a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
        self.tri_area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        self._sb = {}

    def shell_bvh(self, s):
        if s not in self._sb:
            tt = self.t[self.tri_shell == s]
            used, loc = np.unique(tt.ravel(), return_inverse=True)
            vv = self.v[used]
            bvh = BVHTree.FromPolygons(vv.tolist(), loc.reshape(-1, 3).tolist(), epsilon=0.0)
            self._sb[s] = (bvh, vv.min(axis=0), vv.max(axis=0))
        return self._sb[s]

    def bvh_of(self, tri_idx):
        tt = self.t[tri_idx]
        used, loc = np.unique(tt.ravel(), return_inverse=True)
        return BVHTree.FromPolygons(self.v[used].tolist(), loc.reshape(-1, 3).tolist(), epsilon=0.0)

    def sample(self, tri_idx, n, rng):
        a = self.tri_area[tri_idx]
        if not len(tri_idx) or a.sum() <= 0:
            return np.zeros((0, 3)), np.zeros(0, np.int64)
        ti = tri_idx[rng.choice(len(tri_idx), size=n, p=a / a.sum())]
        r1 = np.sqrt(rng.random(n))
        r2 = rng.random(n)
        A, B, C = self.v[self.t[ti, 0]], self.v[self.t[ti, 1]], self.v[self.t[ti, 2]]
        return (1 - r1)[:, None] * A + (r1 * (1 - r2))[:, None] * B + (r1 * r2)[:, None] * C, ti


def occlusion(u, pts, pt_shell):
    """hidden: > HIDE_DEPTH inside another closed shell of the same soup; interface: not hidden and within
    IFACE_TOL of another closed shell surface (goblin check_g2_shape.occlusion)."""
    n = len(pts)
    hidden = np.zeros(n, dtype=bool)
    iface = np.zeros(n, dtype=bool)
    for s in range(len(u.shell_closed)):
        if not u.shell_closed[s]:
            continue
        bvh, mn, mx = u.shell_bvh(s)
        inb = np.all((pts >= mn - IFACE_TOL) & (pts <= mx + IFACE_TOL), axis=1)
        for i in np.nonzero(inb & (pt_shell != s) & ~hidden)[0].tolist():
            p = Vector(pts[i])
            loc, _n, _i, d = bvh.find_nearest(p)
            if loc is None:
                continue
            if d <= IFACE_TOL:
                iface[i] = True
            elif d > HIDE_DEPTH and _inside(bvh, p):
                hidden[i] = True
    iface &= ~hidden
    return hidden, iface


def nearest_dist(bvh, pts):
    out = np.full(len(pts), np.inf)
    for i, p in enumerate(pts.tolist()):
        loc, _n, _i, d = bvh.find_nearest(Vector(p))
        if loc is not None:
            out[i] = d
    return out


# ---------------------------------------------------------------- T258 thigh-thin zone (design change, P2.9c)
def zone_manifest(path):
    """thigh_thin_zone.json -> (set of vertex ids, info) or (None, reason).  Accepts the ids under vertex_ids / verts /
    vertices / ids (a list, a {side: list} dict, or a list of {id | index | vertex} records) at the top level or per
    side; displacement under displacement(_m / _mm) / per_vertex_displacement (list aligned with the ids, {id: value},
    or 3-vectors); cut z under cut_z (per side allowed)."""
    path = Path(path)
    if not path.exists():
        return None, f"no zone manifest ({rel(path)}): behaviour unchanged"
    zj = load_json(path)
    ids, disp, cut = set(), [], {}

    def take_ids(v):
        out = []
        if isinstance(v, list):
            for x in v:
                if isinstance(x, int) and not isinstance(x, bool):
                    out.append(x)
                elif isinstance(x, dict):
                    for k in ("id", "index", "vertex", "v"):
                        if isinstance(x.get(k), int):
                            out.append(x[k])
                            break
        elif isinstance(v, dict):
            for x in v.values():
                out += take_ids(x)
        return out

    def take_disp(v, scale):
        vals = v.values() if isinstance(v, dict) else v if isinstance(v, list) else []
        for x in vals:
            if isinstance(x, (int, float)) and not isinstance(x, bool):
                disp.append(abs(float(x)) * scale)
            elif isinstance(x, list) and len(x) == 3 and all(isinstance(c, (int, float)) for c in x):
                disp.append(math.sqrt(sum(float(c) ** 2 for c in x)) * scale)
            elif isinstance(x, dict):
                for k in ("displacement_m", "displacement", "d", "dist_m"):
                    if isinstance(x.get(k), (int, float, list)):
                        take_disp([x[k]], scale)
                        break

    def scan(d, tag):
        if not isinstance(d, dict):
            return
        for k in ("vertex_ids", "verts", "vertices", "ids"):
            if k in d:
                ids.update(take_ids(d[k]))
                break
        for k, sc in (("displacement_m", 1.0), ("displacement", 1.0), ("per_vertex_displacement", 1.0),
                      ("displacement_mm", 0.001)):
            if k in d:
                take_disp(d[k], sc)
                break
        if isinstance(d.get("cut_z"), (int, float)):
            cut[tag] = float(d["cut_z"])
    scan(zj, "all")
    for side in ("l", "r", "L", "R", "left", "right", "leg_l", "leg_r"):
        if isinstance(zj, dict) and isinstance(zj.get(side), dict):
            scan(zj[side], side)
    for sect in ("sides", "legs", "zones"):
        v = zj.get(sect) if isinstance(zj, dict) else None
        if isinstance(v, dict):
            for side, d in v.items():
                scan(d, side)
        elif isinstance(v, list):
            for i, d in enumerate(v):
                scan(d, str(i))
    if not ids:
        return None, f"{rel(path)}: no vertex ids found (keys {sorted(zj) if isinstance(zj, dict) else type(zj).__name__})"
    return ids, {"manifest": rel(path), "n_verts": len(ids), "cut_z_m": cut,
                 "max_displacement_mm (manifest)": mm(max(disp)) if disp else None,
                 "n_displacement_values": len(disp),
                 "top_level_keys": sorted(zj) if isinstance(zj, dict) else type(zj).__name__}


def nearest_tri(bvh, pts):
    out = np.full(len(pts), -1, dtype=np.int64)
    for i, q in enumerate(pts.tolist()):
        loc, _n, idx, _d = bvh.find_nearest(Vector(q))
        if loc is not None:
            out[i] = idx
    return out


def deviation(ctx):
    """Per source part g: A = retopo parts mapped to g (samples -> g surface), B = g samples -> A surface."""
    ma, src = ctx.ma, ctx.src
    rsoup = Soup(ma.v, ma.tri_v, ctx.shell_of_face[ma.tri_poly],
                 np.array([s.closed for s in ctx.shells], dtype=bool))
    ssoup = Soup(src["v"], src["t"], src["tri_shell"], src["shell_closed"])
    rpart = ma.part[ma.tri_poly]
    groups, unmapped = {}, []
    for n, v in ctx.parts.items():
        fam, side = family(n)
        sfam = SRC_OF_FAM.get(fam, fam)
        cand = [sname for sname in src["labels"] if family(sname) == (sfam, side)]
        if not cand:
            unmapped.append(n)
            continue
        groups.setdefault(cand[0], []).append(n)
    rng = np.random.default_rng(SEED)
    r_area = {g: float(rsoup.tri_area[np.isin(rpart, [ctx.parts[n] for n in ns])].sum()) for g, ns in groups.items()}
    s_area = {g: float(ssoup.tri_area[src["tri_label"] == src["labels"].index(g)].sum()) for g in groups}
    ra_tot, sa_tot = sum(r_area.values()) or 1.0, sum(s_area.values()) or 1.0
    A_pts, A_tri, A_grp, B_pts, B_tri, B_grp = [], [], [], [], [], []
    for gi, (g, ns) in enumerate(sorted(groups.items())):
        rt = np.nonzero(np.isin(rpart, [ctx.parts[n] for n in ns]))[0]
        st = np.nonzero(src["tri_label"] == src["labels"].index(g))[0]
        na = max(MIN_PART_SAMPLES, int(round(N_SAMPLES * r_area[g] / ra_tot)))
        nb = max(MIN_PART_SAMPLES, int(round(N_SAMPLES * s_area[g] / sa_tot)))
        p, t = rsoup.sample(rt, na, rng)
        A_pts.append(p), A_tri.append(t), A_grp.append(np.full(len(p), gi))
        p, t = ssoup.sample(st, nb, rng)
        B_pts.append(p), B_tri.append(t), B_grp.append(np.full(len(p), gi))
    gnames = sorted(groups)
    A_pts, A_tri, A_grp = np.concatenate(A_pts), np.concatenate(A_tri), np.concatenate(A_grp)
    B_pts, B_tri, B_grp = np.concatenate(B_pts), np.concatenate(B_tri), np.concatenate(B_grp)
    print(f"[{GATE}/{CHECKER}] deviation: {len(gnames)} part groups, {len(A_pts)} retopo + {len(B_pts)} source samples")
    sys.stdout.flush()
    A_hid, A_if = occlusion(rsoup, A_pts, rsoup.tri_shell[A_tri])
    B_hid, B_if = occlusion(ssoup, B_pts, ssoup.tri_shell[B_tri])
    A_d = np.full(len(A_pts), np.inf)
    B_d = np.full(len(B_pts), np.inf)
    for gi, g in enumerate(gnames):
        ns = groups[g]
        rt = np.nonzero(np.isin(rpart, [ctx.parts[n] for n in ns]))[0]
        st = np.nonzero(src["tri_label"] == src["labels"].index(g))[0]
        ma_ = A_grp == gi
        mb_ = B_grp == gi
        A_d[ma_] = nearest_dist(ssoup.bvh_of(st), A_pts[ma_])
        B_d[mb_] = nearest_dist(rsoup.bvh_of(rt), B_pts[mb_])
    # T258: thigh-thin zone = design-change region, excluded both ways (retopo samples on triangles touching a zone
    # vertex; source samples whose nearest retopo triangle of their group touches a zone vertex)
    A_zone = B_zone = None
    if ctx.zone_ids:
        ztri = np.any(np.isin(ma.tri_v, np.array(sorted(ctx.zone_ids), dtype=np.int64)), axis=1)
        A_zone = ztri[A_tri]
        B_zone = np.zeros(len(B_pts), dtype=bool)
        for gi, g in enumerate(gnames):
            rt = np.nonzero(np.isin(rpart, [ctx.parts[n] for n in groups[g]]))[0]
            mb_ = np.nonzero(B_grp == gi)[0]
            if not len(rt) or not len(mb_) or not ztri[rt].any():
                continue
            loc = nearest_tri(rsoup.bvh_of(rt), B_pts[mb_])
            B_zone[mb_] = (loc >= 0) & ztri[rt[np.clip(loc, 0, len(rt) - 1)]]
    ctx.dev = {"groups": groups, "gnames": gnames, "unmapped_retopo_parts": unmapped,
               "missing_source_parts": sorted(set(src["labels"]) - set(groups)),
               "A": (A_pts, A_grp, A_hid, A_if, A_d), "B": (B_pts, B_grp, B_hid, B_if, B_d),
               "A_zone": A_zone, "B_zone": B_zone}


def dir_stats(pts, grp, hid, iface, d, gi, extra=None, zone=None):
    m = grp == gi
    if extra is not None:
        m = m & extra
    j = m & ~hid & ~iface
    if zone is not None:            # T258 design-change region (thigh-thin zone), excluded and counted
        j = j & ~zone
    st = stats(d[j])
    ji = np.nonzero(j)[0]
    order = ji[np.argsort(-d[ji], kind="stable")][:TOP_N]
    st.update({"n_samples": int(m.sum()), "n_hidden": int((m & hid).sum()), "n_interface": int((m & iface).sum()),
               "incl_interface": stats(d[m & ~hid]),
               "top": [{"co_m": [round(float(x), 5) for x in pts[i]], "dev_mm": mm(d[i])} for i in order]})
    if zone is not None:
        st["n_zone_excluded"] = int((m & ~hid & ~iface & zone).sum())
        st["zone_region_stats_report"] = stats(d[m & ~hid & ~iface & zone])
    return st


def dev_rows(ctx, cls):
    D = ctx.dev
    rows, ok = {}, True
    for gi, g in enumerate(D["gnames"]):
        if part_class(g) != cls:
            continue
        a = dir_stats(*D["A"], gi, zone=D.get("A_zone"))
        b = dir_stats(*D["B"], gi, zone=D.get("B_zone"))
        r = {"retopo_parts": D["groups"][g], "retopo_to_source": a, "source_to_retopo": b}
        if cls == "rigid":
            r["ok_retopo_to_source"] = dev_ok(a)
            r["ok_source_to_retopo"] = dev_ok(b)
            ok = ok and r["ok_retopo_to_source"] and r["ok_source_to_retopo"]
        rows[g] = r
    return rows, ok


def p21h(ctx):
    rows, ok = dev_rows(ctx, "rigid")
    exp = sorted(n for n in (ctx.p0b or {}).get("parts", {}) if part_class(n) == "rigid")
    missing = sorted(set(exp) - set(rows))
    worst = {}
    for k in ("mean_mm", "p95_mm", "max_mm"):
        vals = [(r[dname][k], g, dname) for g, r in rows.items() for dname in ("retopo_to_source", "source_to_retopo")
                if r[dname][k] is not None]
        if vals:
            v = max(vals)
            worst[k] = {"value": v[0], "part": v[1], "direction": v[2]}
    return {"source": ctx.src["info"], "per_part": rows, "worst": worst,
            "rigid_source_parts_without_retopo_part": missing,
            "retopo_parts_without_source_part": ctx.dev["unmapped_retopo_parts"]}, ok and not missing and bool(rows)


def p21i(ctx):
    rows, _ok = dev_rows(ctx, "deforming")
    D = ctx.dev
    zone = {"info": ctx.zone_info,
            "n_excluded_retopo_to_source": int(D["A_zone"].sum()) if D.get("A_zone") is not None else 0,
            "n_excluded_source_to_retopo": int(D["B_zone"].sum()) if D.get("B_zone") is not None else 0}
    return {"per_part": rows, "note": "rebuilt parts (arm / leg tubes, tunic incl. skirt): report only",
            "design_change_region_thigh_thin (T258)": zone}, True


def belt_bottom(ctx):
    bf = fam_faces(ctx, ("belt",))
    if len(bf):
        vi = np.unique(ctx.ma.loop_vert[np.isin(ctx.ma.loop_poly, bf)])
        return float(ctx.ma.v[vi, 2].min()), "retopo belt part min z"
    bz = ((ctx.p0b or {}).get("skirt") or {}).get("belt_z")
    if bz:
        return float(bz[0]), "P0b skirt.belt_z[0] (no belt part in retopo)"
    return None, "no belt part and no P0b belt_z"


def p21j(ctx):
    ma = ctx.ma
    zb, zsrc = belt_bottom(ctx)
    if zb is None:
        raise Blocked(zsrc)
    sk = (ctx.p0b or {}).get("skirt") or {}
    ctr = np.array((sk.get("hem_ellipse") or {}).get("center_xy") or [0.0, 0.0], dtype=np.float64)
    tf = fam_faces(ctx, ("tunic", "skirt"))
    reg = tf[ma.poly_center[tf, 2] < zb]
    if not len(reg):
        return {"belt_bottom_z_mm": mm(zb), "belt_bottom_source": zsrc, "region_faces": 0}, True
    c = ma.poly_center[reg]
    n = ma.poly_normal[reg]
    rad = c[:, :2] - ctr
    rl = np.linalg.norm(rad, axis=1)
    rl[rl == 0] = 1.0
    nr = (n[:, :2] * (rad / rl[:, None])).sum(axis=1)
    cls = np.where(n[:, 2] < -0.7, "down", np.where(n[:, 2] > 0.7, "up",
                   np.where(nr > 0.3, "outward", np.where(nr < -0.3, "inward", "other"))))
    counts = {k: int((cls == k).sum()) for k in ("outward", "inward", "down", "up", "other")}
    vi = np.unique(ma.loop_vert[np.isin(ma.loop_poly, reg)])
    V = ma.v[vi]
    hem = float(V[:, 2].min())
    cap_faces = int(((n[:, 2] < -0.9) & (c[:, 2] <= hem + CAP_BAND)).sum())
    az = np.degrees(np.arctan2(V[:, 1] - ctr[1], V[:, 0] - ctr[0]))
    rv = np.linalg.norm(V[:, :2] - ctr, axis=1)
    near_hem = V[:, 2] <= hem + CAP_BAND
    hr = {}
    for k, a in SKIRT_AZ.items():
        d = np.abs((az - a + 180.0) % 360.0 - 180.0)
        m = (d <= SKIRT_AZ_HALF) & near_hem
        hr[k] = {"retopo_mm": mm(rv[m].max()) if m.any() else None,
                 "p0b_mm": mm((sk.get("hem_radius_by_azimuth") or {}).get(k))}
    # boundary edges of the region faces (open hem / open inner wall)
    corners = np.nonzero(np.isin(ma.loop_poly, reg))[0]
    _u, cc = np.unique(ma.loop_edge[corners], return_counts=True)
    # deviation inside the region (report): tunic group samples below the belt bottom
    dev = None
    D = ctx.dev
    if D is not None and "tunic" in D["gnames"]:
        gi = D["gnames"].index("tunic")
        zsrc_b = float((sk.get("belt_z") or [zb])[0])
        dev = {"retopo_to_source_z_below_belt_bottom": dir_stats(*D["A"], gi, extra=D["A"][0][:, 2] < zb),
               "source_to_retopo_z_below_p0b_belt_bottom": dir_stats(*D["B"], gi, extra=D["B"][0][:, 2] < zsrc_b)}
    return {"belt_bottom_z_mm": mm(zb), "belt_bottom_source": zsrc,
            "region": "tunic/skirt faces with centroid z < belt bottom",
            "region_faces": int(len(reg)), "region_tris": int(np.isin(ma.tri_poly, reg).sum()),
            "faces_by_normal_class": counts, "class_rule": ("down n.z < -0.7, up n.z > 0.7, else outward / inward = "
                                                            "radial normal component > 0.3 / < -0.3 about the P0b "
                                                            "hem ellipse centre"),
            "hem_z_mm": mm(hem), "p0b_hem_z_mm": mm(sk.get("hem_z_min")),
            "cap_faces_p0b_rule": cap_faces, "p0b_cap_faces": (sk.get("bottom_cap") or {}).get("tris"),
            "region_boundary_edges": int((cc == 1).sum()), "region_nonmanifold_edges": int((cc > 2).sum()),
            "hem_radius_by_azimuth": hr, "deviation_in_region": dev}, True


# ---------------------------------------------------------------- P2.1k / l rings
RING_RE = re.compile(r"^(shoulder|elbow|wrist|hip|knee|ankle)_([lr])_(\d+|end)$")
SKIRT_RE = re.compile(r"^skirt(?:_ring)?_(\d+)(?:_inner)?$")


def loop_problems(ma, vs, ekeys):
    """-> (vertex array or None, [problems]) for a ring given as a list of vertex indices (closed edge loop)."""
    if not (isinstance(vs, list) and all(isinstance(x, int) and not isinstance(x, bool) for x in vs)):
        return None, ["verts is not a list of int"]
    arr = np.array(vs, dtype=np.int64)
    if len(arr) < 3:
        return None, [f"only {len(arr)} verts"]
    if np.any(arr < 0) or np.any(arr >= ma.nv):
        return None, ["vertex index out of range"]
    pr = []
    if len(np.unique(arr)) != len(arr):
        pr.append("duplicate vertices")
    nxt = np.roll(arr, -1)
    keys = np.minimum(arr, nxt) * ma.nv + np.maximum(arr, nxt)
    bad = [int(i) for i, k in enumerate(keys.tolist()) if k not in ekeys]
    if bad:
        pr.append({"missing_edges_at_positions": cap(bad), "n_missing_edges": len(bad)})
    return (None if pr else arr), pr


def edge_keys(ma):
    a, b = ma.e.min(axis=1), ma.e.max(axis=1)
    return set((a * ma.nv + b).tolist())


def validate_rings(ctx):
    ma, loops = ctx.ma, ctx.loops
    rings = loops.get("rings") if isinstance(loops, dict) else None
    if not isinstance(rings, dict):
        raise Blocked("retopo_loops.json has no 'rings' object")
    ekeys = edge_keys(ma)
    valid, probs = {}, {}
    for name, r in rings.items():
        arr, pr = loop_problems(ma, r.get("verts") if isinstance(r, dict) else None, ekeys)
        if pr:
            probs[name] = pr
        else:
            valid[name] = arr
    return rings, valid, probs


def ring_parts(ctx, arr):
    ma = ctx.ma
    m = np.isin(ma.loop_vert, arr)
    vals, cnt = np.unique(ma.part[ma.loop_poly[m]], return_counts=True) if ma.part is not None else ([], [])
    return {ctx.part_names.get(int(v), f"part_{int(v)}"): int(c) for v, c in zip(vals, cnt)}


def p21k(ctx, rings, valid, probs):
    ma = ctx.ma
    lm = (ctx.p0b or {}).get("landmarks") or {}
    groups, errs, unexpected = {}, [], []
    for n in rings:
        m = RING_RE.match(n)
        if m:
            groups.setdefault(f"{m.group(1)}_{m.group(2)}", []).append(n)
        elif not (SKIRT_RE.match(n) or (isinstance(rings[n], dict) and rings[n].get("joint") == "skirt")):
            unexpected.append(n)
    need = {}
    for x in SIDES:
        for j in ("elbow", "knee"):
            need[f"{j}_{x}"] = [f"{j}_{x}_{k}" for k in range(3)]
        for j in ("shoulder", "hip"):
            need[f"{j}_{x}"] = None     # >= 1 ring
    per = {}
    for key, req in need.items():
        got = sorted(groups.get(key, []))
        bad = [g for g in got if g in probs]
        row = {"rings": got, "invalid": {g: probs[g] for g in bad},
               "sizes": {g: int(len(valid[g])) for g in got if g in valid},
               "center_flags_report": {g: rings[g].get("center") for g in got if isinstance(rings[g], dict)}}
        if req is not None:
            miss = [g for g in req if g not in rings]
            row["missing"] = miss
            if miss:
                errs.append(f"{key}: missing {miss}")
        elif not got:
            errs.append(f"{key}: no ring")
        if bad:
            errs.append(f"{key}: invalid {bad}")
        c1 = f"{key}_1"
        if req is not None and c1 in valid:
            cen = ma.v[valid[c1]].mean(axis=0)
            row["ring_1_center_mm"] = [mm(x) for x in cen]
            if key in lm:
                row["ring_1_center_to_p0b_landmark_mm_report"] = mm(np.linalg.norm(cen - np.array(lm[key]["pos"])))
        row["parts_touching_report"] = {g: ring_parts(ctx, valid[g]) for g in got if g in valid}
        per[key] = row
    mesh_field = ctx.loops.get("mesh") if isinstance(ctx.loops, dict) else None
    mf_ok = mesh_field is None or mesh_field == ma.name
    if not mf_ok:
        errs.append(f"retopo_loops.json mesh {mesh_field!r} != checked mesh {ma.name!r}")
    return {"mesh_field": mesh_field, "checked_mesh": ma.name, "n_rings": len(rings), "joints": per,
            "errors": errs, "other_ring_names_report": sorted(n for g in groups for n in groups[g]
                                                              if g.split("_")[0] not in ("elbow", "knee", "shoulder",
                                                                                         "hip")),
            "unrecognised_names_report": {"n": len(unexpected), "first": cap(sorted(unexpected))}}, not errs


def skirt_ring_entries(ctx, rings):
    """Skirt rings: rings{} entries named skirt_<k> / skirt_ring_<k> (optional _inner) or joint == 'skirt', plus
    retopo_loops.json tunic.outer_rows / tunic.inner_rows entries {name, verts} with such a name."""
    out = {}
    for n, r in rings.items():
        if SKIRT_RE.match(n) or (isinstance(r, dict) and r.get("joint") == "skirt"):
            out[n] = (r.get("verts") if isinstance(r, dict) else None, "rings")
    tun = ctx.loops.get("tunic") if isinstance(ctx.loops, dict) else None
    if isinstance(tun, dict):
        for key in ("outer_rows", "inner_rows"):
            for r in tun.get(key) or []:
                if isinstance(r, dict) and SKIRT_RE.match(str(r.get("name"))):
                    out[f"tunic.{key}:{r['name']}"] = (r.get("verts"), f"tunic.{key}")
    return out


def p21l(ctx, rings, valid, probs):
    ma = ctx.ma
    zb, zsrc = belt_bottom(ctx)
    tf = fam_faces(ctx, ("tunic", "skirt"))
    if zb is None or not len(tf):
        raise Blocked(f"belt bottom: {zsrc}; tunic/skirt faces {len(tf)}")
    vi = np.unique(ma.loop_vert[np.isin(ma.loop_poly, tf)])
    Vt = ma.v[vi]
    hem = float(Vt[:, 2].min())
    mid = 0.5 * (zb + hem)
    ekeys = edge_keys(ma)
    ents = skirt_ring_entries(ctx, rings)
    rows, n_between, bad = {}, 0, []
    span = {"upper_Skirt_01": 0, "lower_Skirt_02": 0}
    for n, (vs, where) in sorted(ents.items()):
        arr, pr = loop_problems(ma, vs, ekeys)
        if pr:
            rows[n] = {"invalid": pr, "source": where}
            bad.append(n)
            continue
        z = ma.v[arr, 2]
        zm = float(z.mean())
        between = hem < zm < zb
        n_between += int(between)
        if between:
            span["upper_Skirt_01" if zm >= mid else "lower_Skirt_02"] += 1
        rows[n] = {"source": where, "n_verts": int(len(arr)), "mean_z_mm": mm(zm),
                   "z_range_mm": [mm(z.min()), mm(z.max())], "between_belt_and_hem": between,
                   "parts_touching": ring_parts(ctx, arr)}
    return {"belt_bottom_z_mm": mm(zb), "belt_bottom_source": zsrc, "hem_z_mm": mm(hem),
            "skirt_bone_split_z_mm_report": mm(mid), "n_skirt_rings": len(ents), "n_between": n_between,
            "rings_per_skirt_bone_span_report": span, "invalid": bad, "rings": rows,
            "ring_name_rule": ("rings{} skirt_<k> / skirt_ring_<k> [_inner] or joint == 'skirt'; tunic.outer_rows / "
                               "tunic.inner_rows entries with such a name")}, n_between >= 1 and not bad


# ---------------------------------------------------------------- rows
IDS = (
    ("P2.1a", "one MESH object with FACE INT part_id; parts.json {name: int} unique values; every face value in "
              "parts.json and every parts.json value used", "names classified rigid / deforming by family (spec "
              "section 1); unclassified names and names vs P0b parts reported"),
    ("P2.1b", "every shell (vertex-connected faces of one part): 0 non-manifold edges, and closed OR every open-rim "
              "vertex inside another closed shell (signed depth > 0)", "rim depth = max over the other closed shells "
              "(ray parity majority of 3 + BVH nearest); min depth and containers reported"),
    ("P2.1c", "outward normals: every shell signed volume > 0 (open shells: rims fan-capped) and 0 adjacent face "
              "pairs traversing their shared edge in the same direction", ""),
    ("P2.1d", f"buried faces (every vertex > {INTERIOR_DEPTH * 1000:g} mm inside another closed shell) = 0 except "
              "designed overlaps (listed in measured.designed_overlap_families)", "same-part shells included; "
              "excluded counts per pair reported"),
    ("P2.1e", f"(draft) triangles <= {TRI_MAX}", CALIB),
    ("P2.1f", f"UV: every face UV area > {UV_AREA_EPS:g}; every loop UV in [0,1]; island overlap pixels = 0 "
              f"({UV_RES}^2 raster, left and right parts included); head front texel density >= body x "
              f"{DENSITY_RATIO}", "head front = head faces with normal . (0,-1,0) > 0.7; body = all faces except "
              "head / eye parts; density = UV area / 3D area; per-part density and L/R overlap reported"),
    ("P2.1g", "has_custom_normals true and 0 NaN / inf corner normals", ""),
    ("P2.1h", "unchanged rigid parts vs the source FBX, both directions, per part: mean <= 1, p95 <= 2, "
              "max <= 3 mm", "hidden (> 1 mm inside another closed shell of the same side) and interface "
              "(< 0.1 mm) points excluded and counted; every retopo part compared only with its own source part"),
    ("P2.1i", "(report) rebuilt deforming parts (arm, leg, tunic incl. skirt) vs the source FBX, both directions",
     CALIB),
    ("P2.1j", "(report) skirt redesign region: tunic/skirt faces below the belt bottom", CALIB),
    ("P2.1k", "retopo_loops.json rings: elbow_x_0/1/2 and knee_x_0/1/2 present, shoulder_x_* and hip_x_* >= 1 "
              "each, every listed ring a closed edge loop of the checked mesh; mesh field = checked mesh when "
              "given", "ring centre vs P0b landmark and touching parts reported"),
    ("P2.1l", "skirt rings (skirt_<k>): >= 1 valid closed ring with mean z strictly between the hem (tunic/skirt "
              "min z) and the belt bottom (belt min z); no invalid skirt ring", "rings per Skirt_01 / Skirt_02 "
              "span reported"),
)


class Evidence:
    def __init__(self):
        self.inputs, self.criteria = [], []
        self.extra = {}

    def add_input(self, path):
        r = rel(path)
        if Path(path).exists() and not any(i["path"] == r for i in self.inputs):
            self.inputs.append({"path": r, "sha256": sha256(path)})

    def criterion(self, cid, measured, threshold, ok, note):
        self.criteria.append({"id": cid, "ok": bool(ok), "measured": jsonable(measured),
                              "threshold": threshold, "note": note})

    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {i: k for k, (i, _t, _n) in enumerate(IDS)}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               **jsonable(self.extra), "inputs": self.inputs, "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            s = json.dumps(c["measured"], ensure_ascii=False)
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={s[:500]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--blend": "blend", "--parts": "parts", "--loops": "loops", "--p0b": "p0b", "--src": "src",
            "--mesh": "mesh", "--out": "out", "--zone": "zone"}
    i = 0
    while i < len(argv):
        if argv[i] == "--in-memory":
            P["in_memory"] = True
            i += 1
            continue
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <value>' for k in keys)} [--in-memory]")
        P[keys[argv[i]]] = argv[i + 1] if keys[argv[i]] == "mesh" else Path(argv[i + 1])
        i += 2


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    nt = {i: n for i, _t, n in IDS}
    done = set()
    all_ids = [i for i, _t, _n in IDS]

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, nt[cid])
        done.add(cid)

    def block(cids, reason):
        for c in cids:
            if c not in done:
                put(c, f"blocked: {reason}", False)

    def run(cid, fn, *a):
        if cid in done:
            return
        try:
            put(cid, *fn(*a))
        except Blocked as e:
            block([cid], str(e))

    miss = []
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
            else:
                miss.append(rel(P[key]))
        if Path(P["src"]).exists():
            ev.add_input(P["src"])
        else:
            miss.append(Path(P["src"]).as_posix())
        _STATE["missing"] = miss
        for p in sorted(HERE.glob("p2*.py")):
            ev.add_input(p)
        ctx.parts, ctx.parts_meta = parts_map(load_json(P["parts"])) if Path(P["parts"]).exists() else (None, {})
        ctx.loops = load_json(P["loops"]) if Path(P["loops"]).exists() else None
        ctx.p0b = load_json(P["p0b"]) if Path(P["p0b"]).exists() else None
        if not P["in_memory"] and not Path(P["blend"]).exists():
            block(all_ids, f"missing input {rel(P['blend'])}")
            return
        if ctx.parts is None:
            block(all_ids, f"missing input {rel(P['parts'])}")
            return
        setup(ctx)
        bb = base_block(ctx)
        if bb:
            block(all_ids, bb)
            return
        print(f"[{GATE}/{CHECKER}] mesh {ctx.ma.name}: {ctx.ma.nv} verts, {ctx.ma.npoly} faces, {ctx.ma.ntri} tris, "
              f"{len(ctx.shells)} shells")
        sys.stdout.flush()
        run("P2.1a", p21a, ctx)
        run("P2.1b", p21b, ctx)
        run("P2.1c", p21c, ctx)
        run("P2.1d", p21d, ctx)
        run("P2.1e", p21e, ctx)
        run("P2.1f", p21f, ctx)
        run("P2.1g", p21g, ctx)
        # rings
        if ctx.loops is None:
            block(["P2.1k", "P2.1l"], f"missing input {rel(P['loops'])}")
        else:
            try:
                rings, valid, probs = validate_rings(ctx)
            except Blocked as e:
                block(["P2.1k", "P2.1l"], str(e))
            else:
                run("P2.1k", p21k, ctx, rings, valid, probs)
                run("P2.1l", p21l, ctx, rings, valid, probs)
        # deviation (source FBX); T258 thigh-thin zone excluded when its manifest exists
        if Path(P["zone"]).exists():
            ev.add_input(P["zone"])
        ctx.zone_ids, ctx.zone_info = zone_manifest(P["zone"])
        try:
            if ctx.p0b is None:
                raise Blocked(f"missing input {rel(P['p0b'])}")
            load_source(ctx)
            deviation(ctx)
        except Blocked as e:
            block(["P2.1h", "P2.1i"], f"source: {e}")
        else:
            run("P2.1h", p21h, ctx)
            run("P2.1i", p21i, ctx)
        run("P2.1j", p21j, ctx)
    finally:
        try:
            block(all_ids, "checker aborted before this criterion: " + tb_tail(2) if sys.exc_info()[0] else
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
