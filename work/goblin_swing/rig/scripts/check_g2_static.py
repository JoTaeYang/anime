"""check_g2_static - Gate G2 RETOPO-STATIC checker (design doc d-23 section 7).

Numeric criteria: G2.1 .. G2.10, G2.14 (a/b/c), G2.16, G2.17 (2026-09-26 HR2 mesh contract: material slots +
shape keys, rig/data/hr2_contract.md).  Renders: G2.15 (joint loops, poles) and G2.16 (hand_r close-ups).
G2.11 .. G2.13 belong to check_g2_shape.  The UV / manifold / normal rules (G2.2, G2.3, G2.9, G2.10) cover every
face, the HR2 mouth faces (cavity, teeth, tongue, lip patch) included; G2.9 also reports them per material slot.
Geometry = obj.data vertex co (the Basis shape = closed mouth; G2.17 reports co vs Basis).

Inputs (plan d-23 "retopo data contract"):
  rig/gob_r01_retopo.blend   GOB_mesh (single object, face INT attribute part_id), GOB_club
  rig/data/parts.json        {"body":0,"head":1,"hand_l":2,"hand_r":3,"shoe_l":4,"shoe_r":5,"belt":6}
  rig/data/retopo_loops.json {"mesh":"GOB_mesh","rings":{name:{verts,joint,k,center}}}
  rig/data/pivots.json       {"pivots":{name:{co,axis?,...}}}
  rig/gob_r00_source.blend   SRC_hi, SRC_club_hi (G2.16 only; appended read-only via
                             bpy.data.libraries.load, the file is never written)
It does not import or reuse the production scripts (s02*); only geometry + data json.

Definitions:
  shell      = connected component (faces sharing a vertex) of the faces of one part_id value.
  designed overlap part pairs (not defects): (body, hand_l|hand_r|shoe_l|shoe_r|head|belt).
  inside depth of point p w.r.t. a closed shell = distance to the shell surface (BVH nearest)
             when p is inside, negative distance when outside.  Inside = ray parity, majority of
             3 fixed skew directions (each ray re-cast past every hit); points outside the
             shell bbox are outside.  For a part, depth = max over the part's shells.
Meshes are read from obj.data (modifiers not evaluated, indices = retopo_loops.json indices) and
transformed by matrix_world; all lengths in m, reported in mm.

Run:    bl.ps1 -Script check_g2_static.py -Blend gob_r01_retopo.blend
Output: rig/inspect/G2/check_g2_static.json + renders in rig/inspect/G2/.
Exit:   0 when the evidence was written; 2 on a script error (run_main) or when an input
        file is missing (the evidence is still written, then "missing input" is printed).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
import bpy  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402
from mathutils.kdtree import KDTree  # noqa: E402

GATE = "G2"
CHECKER = "check_g2_static"
BLEND = goblib.RIG / "gob_r01_retopo.blend"
SRC_BLEND = goblib.RIG / "gob_r00_source.blend"
PARTS_JSON = goblib.DATA / "parts.json"
LOOPS_JSON = goblib.DATA / "retopo_loops.json"
PIVOTS_JSON = goblib.DATA / "pivots.json"
MOUTH_JSON = goblib.DATA / "mouth.json"     # G2.4 designed mouth-interior overlap (T121b)
MOUTH_OVERLAP_GROUPS = ("cavity", "teeth", "tongue")

MESH = "GOB_mesh"
CLUB = "GOB_club"
SRC_BODY = "SRC_hi"
SRC_CLUB = "SRC_club_hi"
PART_NAMES = ["body", "head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt"]
OVERLAP_PAIRS = {frozenset(("body", p)) for p in ("hand_l", "hand_r", "shoe_l", "shoe_r", "head", "belt")}
SIDES = ("l", "r")

TRI_RANGE = (6000, 9500)    # G2.8 (2026-09-26 user decision, d-29 HR: head retopo + mouth)
CLUB_TRI_MAX = 1000
INTERIOR_DEPTH = 0.002      # G2.4
CENTER_TOL = 0.003          # G2.6
POLE_RADIUS = 0.030         # G2.7 region radius around ring vertices
BAND_DEG = 30.0             # G2.7 half width of the bend sectors
UV_RES = 2048               # G2.9
UV_EPS = 1e-6
UV_MATCH = 1e-6
HEAD_FRONT_DOT = 0.7
DENSITY_RATIO = 1.5
END_DEPTH = 0.003           # G2.14a/b
HEAD_BOTTOM_BAND = 0.002    # G2.14b
BELT_DEPTH = 0.0005         # G2.14c
HANDLE_HALF = 0.020         # G2.16 handle section: GOB_club verts with |s| <= this along grip_r axis
HANDLE_DEPTH = 0.001        # G2.16 handle surface samples inside hand_r shell by >= this
HANDLE_STEP = 0.001         # G2.16 max barycentric sample spacing on GOB_club faces (m)
HANDLE_MIN_N = 200          # G2.16 min surface samples (spacing halved until reached)
CHORD_RATIO = 0.8           # G2.16 grip-axis chord through hand_r >= this * ball diameter
RANSAC = {"rmin": 0.04, "rmax": 0.2, "tol": 0.001, "iters": 4000, "seed": 0, "refits": 5}
LIST_CAP = 40               # max list entries written per measured list
MAT_SLOTS = ("GOB_skin", "GOB_mouth_inner", "GOB_teeth", "GOB_tongue")   # G2.17 hr2_contract.md, in this order
SHAPE_KEYS = ("Basis", "mouth_open")                                     # G2.17 hr2_contract.md
KEY_MOVE_EPS = 1e-6         # G2.17 report: vertex counted as moved by a key when |key - Basis| > this (m)

RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237),
                                              (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6

RING_COLORS = {
    "shoulder": (1.0, 0.50, 0.0), "elbow": (0.95, 0.0, 0.85), "wrist": (0.0, 0.80, 0.95),
    "hip": (0.10, 0.80, 0.10), "knee": (0.15, 0.35, 1.0), "ankle": (0.55, 0.20, 0.95),
    "spine": (0.0, 0.60, 0.50), "belt": (0.60, 0.35, 0.10),
}
MESH_COLOR = (0.78, 0.78, 0.78, 1.0)
POLE_IN_COLOR = (1.0, 0.0, 0.0, 1.0)
POLE_OUT_COLOR = (1.0, 0.9, 0.0, 1.0)


# ---------------------------------------------------------------- helpers
def mm(x):
    return None if x is None else round(float(x) * 1000.0, 4)


def rel(p):
    return goblib._rel(p)


def vec3(v):
    try:
        a = np.array([float(x) for x in v], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        return None
    return a


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def cap(lst):
    return lst[:LIST_CAP]


def rnd4(x):
    return round(float(x), 4)


def _components(n, a, b):
    """Union-find over n nodes with edges (a[i], b[i]); returns root per node."""
    parent = list(range(n))

    def find(i):
        r = i
        while parent[r] != r:
            r = parent[r]
        while parent[i] != r:
            parent[i], i = r, parent[i]
        return r

    for x, y in zip(a.tolist(), b.tolist()):
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[rx] = ry
    return np.array([find(i) for i in range(n)], dtype=np.int64)


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
            raise RuntimeError(f"{obj.name}: polygon loops are not contiguous")
        self.loop_start, self.loop_total = ls, lt
        self.loop_poly = np.repeat(np.arange(npoly), lt)
        idx = np.arange(nl)
        st = ls[self.loop_poly]
        self.loop_next = st + (idx - st + 1) % lt[self.loop_poly]

        if hasattr(me, "calc_loop_triangles"):
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
        self.v_list = [tuple(p) for p in self.v.tolist()]

        # part_id
        self.part = None
        att = me.attributes.get("part_id")
        self.part_attr = None if att is None else {"domain": att.domain, "data_type": att.data_type}
        if att is not None and att.domain == "FACE" and att.data_type == "INT":
            pv = np.empty(npoly, dtype=np.int32)
            att.data.foreach_get("value", pv)
            self.part = pv.astype(np.int64)

        # active UV layer
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
        _, c = np.unique(ma.loop_edge[self.corners], return_counts=True)
        self.n_boundary = int((c == 1).sum())
        self.n_nonmanifold = int((c > 2).sum())
        self.closed = self.n_boundary == 0 and self.n_nonmanifold == 0
        self.tris = np.nonzero(shell_of_face[ma.tri_poly] == sid)[0]
        P = ma.v[self.verts]
        self.mn, self.mx = P.min(axis=0), P.max(axis=0)
        self._bvh = None

    @property
    def label(self):
        return f"{self.part}#{self.idx}"

    def bvh(self):
        if self._bvh is None:
            self._bvh = BVHTree.FromPolygons(self.ma.v_list, self.ma.tri_v[self.tris].tolist(),
                                             all_triangles=True)
        return self._bvh

    def signed_volume(self):
        T = self.ma.v[self.ma.tri_v[self.tris]]
        if not len(T):
            return 0.0
        T = T - T.reshape(-1, 3).mean(axis=0)
        return float(np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6.0)

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
    """Shells = vertex-connected face components inside each group value."""
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


def shell_depth(shell, pts):
    """Signed inside depth (m) of pts (n,3) w.r.t. shell: + inside, - outside."""
    bvh = shell.bvh()
    out = np.empty(len(pts))
    inb = np.all((pts >= shell.mn) & (pts <= shell.mx), axis=1)
    for i, p in enumerate(pts.tolist()):
        pv = Vector(p)
        loc, _, _, dist = bvh.find_nearest(pv)
        if loc is None:
            out[i] = -np.inf
            continue
        out[i] = dist if (inb[i] and _inside(bvh, pv)) else -dist
    return out


def part_depth(shells, pts):
    if not shells or not len(pts):
        return np.full(len(pts), -np.inf)
    return np.max(np.stack([shell_depth(s, pts) for s in shells]), axis=0)


def blocked(ev, ids, th, reasons):
    r = "; ".join(x for x in reasons if x)
    if r:
        for cid in ids:
            ev.criterion(cid, r, th, False)
        return True
    return False


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.miss = {}
        self.mesh_obj = self.club_obj = None
        self.ma = self.mc = None
        self.mesh_err = ""       # GOB_mesh object problem
        self.part_err = ""       # part_id / parts.json problem
        self.club_err = ""
        self.parts = None        # name -> value (parts.json)
        self.part_names = {}     # value -> name
        self.shells, self.shell_of_face = [], None
        self.club_shells = []
        self.loops = self.pivots = None
        self.ring_verts = {}     # valid rings: name -> vertex index array
        self.ring_problems = {}
        self.poles_in, self.poles_other = [], []
        self.g216_flagged = []   # world points (G2.16 club handle samples < HANDLE_DEPTH inside hand_r)
        self.src = {}

    def m(self, *keys):
        return [self.miss.get(k, "") for k in keys]

    def shells_of(self, part):
        return [s for s in self.shells if s.part == part]

    def verts_of_part(self, part):
        val = (self.parts or {}).get(part)
        if val is None or self.ma is None or self.ma.part is None:
            return np.zeros(0, np.int64)
        fm = self.ma.part == val
        return np.unique(self.ma.loop_vert[fm[self.ma.loop_poly]])

    def piv(self, name):
        p = ((self.pivots or {}).get("pivots") or {}).get(name)
        return p if isinstance(p, dict) else None


# ---------------------------------------------------------------- G2.1
def crit_g2_1(ev, ctx):
    th = ("GOB_mesh MESH with FACE INT attribute part_id; parts.json = the 7 contract names; every "
          "parts.json value present, no other value; GOB_club MESH, separate object")
    if blocked(ev, ["G2.1"], th, ctx.m("blend", "parts")):
        return
    measured, ok = {}, True
    pj = ctx.parts
    names_ok = isinstance(pj, dict) and sorted(pj) == sorted(PART_NAMES) and \
        all(isinstance(v, int) for v in pj.values()) and len(set(pj.values())) == len(pj)
    measured["parts_json"] = {"content": pj, "matches_contract_names": names_ok}
    ok = ok and names_ok
    mo = ctx.mesh_obj
    if mo is None:
        measured[MESH] = "missing object"
        ok = False
    else:
        info = {"type": mo.type, "modifiers": [m.type for m in mo.modifiers]}
        if mo.type == "MESH":
            ma = ctx.ma
            info["part_id"] = ma.part_attr
            if ma.part is None:
                ok = False
            else:
                vals, cnt = np.unique(ma.part, return_counts=True)
                inv = {v: k for k, v in pj.items()} if isinstance(pj, dict) else {}
                info["faces_per_value"] = {str(int(v)): {"name": inv.get(int(v)), "faces": int(c)}
                                           for v, c in zip(vals, cnt)}
                exp = set(pj.values()) if isinstance(pj, dict) else set()
                present = set(int(v) for v in vals)
                info["missing_values"] = sorted(exp - present)
                info["unknown_values"] = sorted(present - exp)
                ok = ok and not info["missing_values"] and not info["unknown_values"]
        else:
            ok = False
        measured[MESH] = info
    co = ctx.club_obj
    if co is None:
        measured[CLUB] = "missing object"
        ok = False
    else:
        sep = mo is None or (co is not mo and co.data is not getattr(mo, "data", None))
        measured[CLUB] = {"type": co.type, "separate_object": sep,
                          "parent": co.parent.name if co.parent else None}
        ok = ok and co.type == "MESH" and sep
    ev.criterion("G2.1", measured, th, ok,
                 "part_id read from GOB_mesh.data attributes; separate_object = GOB_club is a "
                 "different object and does not share GOB_mesh mesh data")


# ---------------------------------------------------------------- G2.2
def _edge_counts(ma, corner_mask):
    _, c = np.unique(ma.loop_edge[corner_mask], return_counts=True)
    return int((c == 1).sum()), int((c > 2).sum()), int(len(c))


def crit_g2_2(ev, ctx):
    th = "per part: boundary edges 0 and non-manifold edges (>2 faces of the part) 0; GOB_club same"
    if blocked(ev, ["G2.2"], th, ctx.m("blend", "parts") + [ctx.mesh_err, ctx.part_err]):
        return
    ma, ok, per = ctx.ma, True, {}
    for name in PART_NAMES:
        val = ctx.parts.get(name)
        fm = ma.part == val
        if not fm.any():
            per[name] = {"faces": 0}
            ok = False
            continue
        b, nm, ne = _edge_counts(ma, fm[ma.loop_poly])
        sh = ctx.shells_of(name)
        per[name] = {"faces": int(fm.sum()), "edges": ne, "boundary_edges": b,
                     "nonmanifold_edges": nm, "shells": len(sh),
                     "open_shells": [{"shell": s.label, "boundary": s.n_boundary,
                                      "nonmanifold": s.n_nonmanifold} for s in sh if not s.closed]}
        ok = ok and b == 0 and nm == 0
    used = np.zeros(ma.ne, bool)
    used[ma.loop_edge] = True
    measured = {MESH: per, "GOB_mesh_wire_edges": int((~used).sum())}
    if ctx.mc is None:
        measured[CLUB] = ctx.club_err or "missing object"
        ok = False
    else:
        mc = ctx.mc
        b, nm, ne = _edge_counts(mc, np.ones(mc.nl, bool))
        measured[CLUB] = {"faces": mc.npoly, "edges": ne, "boundary_edges": b, "nonmanifold_edges": nm,
                          "shells": len(ctx.club_shells)}
        ok = ok and mc.npoly > 0 and b == 0 and nm == 0
    ev.criterion("G2.2", measured, th, ok,
                 "edge use count over the faces of one part (an edge shared with another part counts "
                 "once per part -> boundary); boundary 0 for body means the tube ends are capped; "
                 "wire edges (no face) reported only")


# ---------------------------------------------------------------- G2.3
def crit_g2_3(ev, ctx):
    th = "every shell: signed volume > 0 (outward) and 0 adjacent face pairs with same-direction shared edge"
    if blocked(ev, ["G2.3"], th, ctx.m("blend", "parts") + [ctx.mesh_err, ctx.part_err]):
        return
    ok, rows, neg, inc_total = True, [], [], 0
    all_shells = [(MESH, s) for s in ctx.shells] + [(CLUB, s) for s in ctx.club_shells]
    for obj_name, s in all_shells:
        vol = s.signed_volume()
        inc = s.inconsistent_pairs()
        label = s.label if obj_name == MESH else f"{CLUB}#{s.idx}"
        rows.append({"shell": label, "faces": len(s.faces), "closed": s.closed,
                     "volume_cm3": round(vol * 1e6, 4), "inconsistent_pairs": inc})
        if not vol > 0:
            neg.append(label)
        inc_total += inc
    if ctx.mc is None:
        ok = False
    ok = ok and not neg and inc_total == 0
    ev.criterion("G2.3", {"n_shells": len(rows), "nonpositive_volume": neg,
                          "inconsistent_pairs_total": inc_total, "shells": rows,
                          "club": "included" if ctx.mc is not None else (ctx.club_err or "missing object")},
                 th, ok,
                 "volume = divergence theorem over loop triangles (face winding), origin at shell "
                 "centroid; winding pairs = shell edges used by exactly 2 faces whose corners traverse "
                 "the edge in the same vertex order; volume sign is meaningless for open shells (closed=false)")


# ---------------------------------------------------------------- G2.4
def mouth_overlap_faces():
    """G2.4 (T121b): {group: face index set} from mouth.json region.output_faces.by_group cavity / teeth / tongue
    (indices of the checked blend: output sha256 must match) -> (groups or None, note)."""
    if not MOUTH_JSON.exists():
        return None, f"no exclusion: {rel(MOUTH_JSON)} missing"
    try:
        with open(MOUTH_JSON, "r", encoding="utf-8") as f:
            mj = json.load(f)
    except (OSError, ValueError) as e:
        return None, f"no exclusion: {rel(MOUTH_JSON)} unreadable ({e})"
    out = mj.get("output") if isinstance(mj.get("output"), dict) else {}
    sha = goblib.sha256(BLEND)
    if out.get("sha256") != sha:
        return None, (f"no exclusion: {rel(MOUTH_JSON)} output sha256 {str(out.get('sha256'))[:12]} != "
                      f"{rel(BLEND)} {sha[:12]}")
    bg = (((mj.get("region") or {}).get("output_faces") or {}).get("by_group") or {})
    groups = {g: set(int(i) for i in (bg.get(g) or [])) for g in MOUTH_OVERLAP_GROUPS}
    return groups, f"{rel(MOUTH_JSON)} region.output_faces.by_group (output sha256 matches)"


def crit_g2_4(ev, ctx):
    th = ("faces with all vertices > 2 mm inside another closed shell (not a designed overlap part pair; "
          "same-part shells included) = 0; designed mouth-interior overlap excluded (2026-09-26 HR2 T121b): "
          "mouth.json cavity / teeth / tongue faces and head faces buried in a teeth shell")
    if blocked(ev, ["G2.4"], th, ctx.m("blend", "parts") + [ctx.mesh_err, ctx.part_err]):
        return
    ma = ctx.ma
    closed = [s for s in ctx.shells if s.closed]
    mgroups, mnote = mouth_overlap_faces()
    teeth = (mgroups or {}).get("teeth", set())
    excl = {g: 0 for g in MOUTH_OVERLAP_GROUPS + ("head_in_teeth_shell",)}
    pairs, total, tested = [], 0, 0
    for A in ctx.shells:
        for B in closed:
            if A is B or frozenset((A.part, B.part)) in OVERLAP_PAIRS:
                continue
            lo, hi = B.mn + INTERIOR_DEPTH, B.mx - INTERIOR_DEPTH
            if np.any(A.mx < lo) or np.any(A.mn > hi):
                continue
            P = ma.v[A.verts]
            cand = np.all((P >= lo) & (P <= hi), axis=1)
            if not cand.any():
                continue
            tested += 1
            depth = np.full(ma.nv, -np.inf)
            depth[A.verts[cand]] = shell_depth(B, P[cand])
            deep = depth[ma.loop_vert[A.corners]] > INTERIOR_DEPTH
            n_deep = np.bincount(ma.loop_poly[A.corners][deep], minlength=ma.npoly)
            faces = A.faces[n_deep[A.faces] == ma.loop_total[A.faces]]
            if len(faces):
                total += len(faces)
                b_teeth = bool(teeth) and set(B.faces.tolist()) <= teeth
                kept, ex_here = [], {}
                for fi in faces.tolist():
                    g = next((g for g in MOUTH_OVERLAP_GROUPS if fi in (mgroups or {}).get(g, ())), None)
                    if g is None and b_teeth and A.part == "head":
                        g = "head_in_teeth_shell"
                    if g is None:
                        kept.append(fi)
                    else:
                        excl[g] += 1
                        ex_here[g] = ex_here.get(g, 0) + 1
                pairs.append({"face_shell": A.label, "inside_shell": B.label, "faces": len(faces),
                              "sample_faces": cap(faces.tolist()), "excluded_designed_overlap": ex_here,
                              "remaining": len(kept), "remaining_faces": cap(kept)})
    remaining = total - sum(excl.values())
    ev.criterion("G2.4", {"buried_faces": total, "excluded_designed_overlap": excl,
                          "remaining_buried_faces": remaining, "exclusion_source": mnote,
                          "pairs": pairs, "shell_pairs_tested": tested,
                          "closed_shells_used_as_container": [s.label for s in closed],
                          "open_shells_skipped_as_container": [s.label for s in ctx.shells if not s.closed]},
                 th, remaining == 0,
                 "inside test: ray parity (3 skew rays, majority) + BVH nearest distance per vertex; "
                 "a face counts when every vertex depth > 2 mm; skipped part pairs: "
                 "(body, hand_l|hand_r|shoe_l|shoe_r|head|belt); GOB_club not included (separate object). "
                 "T121b (2026-09-26 HR2, main decision): designed mouth-interior overlap (hidden at mouth_open 0) = "
                 "buried faces listed in mouth.json region.output_faces.by_group cavity / teeth / tongue, plus head "
                 "faces buried in a shell made only of teeth faces (skin under a fang); counted per group in "
                 "excluded_designed_overlap; ok = remaining_buried_faces == 0; buried_faces = all (as before)")


# ---------------------------------------------------------------- G2.5
RING_NUM = re.compile(r"^(shoulder|elbow|wrist|hip|knee|ankle)_([lr])_(\d+)$")


def validate_rings(ctx):
    """Fill ctx.ring_verts (valid rings) and ctx.ring_problems; return report dict."""
    ma, loops = ctx.ma, ctx.loops
    rep = {"mesh_field": loops.get("mesh") if isinstance(loops, dict) else None}
    rings = loops.get("rings") if isinstance(loops, dict) else None
    if not isinstance(rings, dict):
        rep["error"] = "retopo_loops.json has no 'rings' object"
        return rep
    a, b = ma.e.min(axis=1), ma.e.max(axis=1)
    ekeys = set((a * ma.nv + b).tolist())
    for name, r in rings.items():
        probs = []
        if not isinstance(r, dict):
            ctx.ring_problems[name] = ["entry is not an object"]
            continue
        for fld in ("verts", "joint", "k", "center"):
            if fld not in r:
                probs.append(f"missing field {fld}")
        if "center" in r and not isinstance(r["center"], bool):
            probs.append("center is not bool")
        if "k" in r and (not isinstance(r["k"], int) or isinstance(r["k"], bool)):
            probs.append("k is not int")
        mnum = RING_NUM.match(name) or re.match(r"^(spine_01|spine_02|belt_top|belt_bot)_(\d+)$", name)
        if mnum and isinstance(r.get("k"), int) and r["k"] != int(mnum.groups()[-1]):
            probs.append(f"k={r['k']} differs from name suffix")
        vs = r.get("verts")
        ok_list = isinstance(vs, list) and all(isinstance(x, int) and not isinstance(x, bool) for x in vs)
        if not ok_list:
            probs.append("verts is not a list of int")
        else:
            arr = np.array(vs, dtype=np.int64)
            if len(arr) < 3:
                probs.append(f"only {len(arr)} verts")
            elif np.any(arr < 0) or np.any(arr >= ma.nv):
                probs.append("vertex index out of range")
            else:
                if len(np.unique(arr)) != len(arr):
                    probs.append("duplicate vertices")
                nxt = np.roll(arr, -1)
                keys = np.minimum(arr, nxt) * ma.nv + np.maximum(arr, nxt)
                bad = [int(i) for i, kk in enumerate(keys.tolist()) if kk not in ekeys]
                if bad:
                    probs.append({"missing_edges_at_positions": cap(bad), "n_missing_edges": len(bad),
                                  "n_verts": len(arr)})
                if not probs:
                    ctx.ring_verts[name] = arr
        if probs:
            ctx.ring_problems[name] = probs
    rep["n_rings"] = len(rings)
    rep["ring_sizes"] = {n: len(v) for n, v in ctx.ring_verts.items()}
    rep["_names"] = set(rings)
    return rep


def ring_group_check(ctx, names):
    """Count / naming rules (contract).  Returns (missing, count_errors, expected_names)."""
    rings = (ctx.loops or {}).get("rings") or {}
    missing, errs, expected = [], [], set()

    def ks(prefix):
        out = {}
        for n in names:
            m = re.match(rf"^{prefix}_(\d+)$", n)
            if m:
                out[int(m.group(1))] = n
        return out

    for x in SIDES:
        for j, (lo, hi) in (("shoulder", (2, 3)), ("hip", (2, 3))):
            got = ks(f"{j}_{x}")
            expected.update(got.values())
            if not (lo <= len(got) <= hi) or sorted(got) != list(range(len(got))):
                errs.append(f"{j}_{x}: rings k={sorted(got)} (need 2..3 rings, k = 0..n-1)")
        for j in ("elbow", "knee"):
            got = ks(f"{j}_{x}")
            expected.update(got.values())
            if sorted(got) != [0, 1, 2]:
                errs.append(f"{j}_{x}: rings k={sorted(got)} (need k = 0,1,2)")
            centers = [n for n in got.values() if isinstance(rings.get(n), dict) and rings[n].get("center") is True]
            if centers != [f"{j}_{x}_1"] or rings.get(f"{j}_{x}_1", {}).get("k") != 1:
                errs.append(f"{j}_{x}: center rings {centers} (need exactly {j}_{x}_1 with center=true, k=1)")
        for j in ("wrist", "ankle"):
            got = ks(f"{j}_{x}")
            expected.update(got.values())
            if sorted(got) != [0, 1]:
                errs.append(f"{j}_{x}: rings k={sorted(got)} (need k = 0,1)")
            end = f"{j}_{x}_end"
            expected.add(end)
            if end not in names:
                missing.append(end)
    for n in ("spine_01_0", "spine_02_0", "belt_top_0", "belt_bot_0"):
        expected.add(n)
        if n not in names:
            missing.append(n)
    for x in SIDES:
        for j, need in (("shoulder", 2), ("hip", 2), ("elbow", 3), ("knee", 3), ("wrist", 2), ("ankle", 2)):
            for k in range(need):
                if f"{j}_{x}_{k}" not in names:
                    missing.append(f"{j}_{x}_{k}")
    return missing, errs, expected


def crit_g2_5(ev, ctx, rep):
    th = ("rings present per contract (shoulder_x 2..3, hip_x 2..3, elbow_x/knee_x 3 with center k=1, "
          "wrist_x/ankle_x 2, wrist_x_end, ankle_x_end, spine_01_0, spine_02_0, belt_top_0, belt_bot_0); "
          "every ring a closed edge loop of GOB_mesh in listed order")
    if blocked(ev, ["G2.5"], th, ctx.m("blend", "loops") + [ctx.mesh_err]):
        return
    if "error" in rep:
        ev.criterion("G2.5", rep, th, False)
        return
    names = rep.pop("_names")
    missing, errs, expected = ring_group_check(ctx, names)
    bad = {n: p for n, p in ctx.ring_problems.items()}
    measured = {"mesh_field": rep["mesh_field"], "n_rings": rep["n_rings"], "missing": sorted(set(missing)),
                "count_errors": errs, "bad_rings": bad, "ring_sizes": rep["ring_sizes"],
                "unexpected_names": sorted(names - expected)}
    ok = rep["mesh_field"] == MESH and not missing and not errs and not bad
    ev.criterion("G2.5", measured, th, ok,
                 "closed loop = every consecutive pair and last->first is an existing GOB_mesh edge, >= 3 "
                 "distinct verts; fields verts/joint/k/center required, k must equal the numeric name "
                 "suffix; unexpected_names reported only")


# ---------------------------------------------------------------- G2.6
def crit_g2_6(ev, ctx):
    th = "elbow_x_1 / knee_x_1 vertex mean to pivot <= 3 mm; spine_01_0 / spine_02_0 mean z - pivot z <= 3 mm"
    if blocked(ev, ["G2.6"], th, ctx.m("blend", "loops", "pivots") + [ctx.mesh_err]):
        return
    ma, out, ok = ctx.ma, {}, True
    for j in ("elbow", "knee"):
        for x in SIDES:
            ring, pn = f"{j}_{x}_1", f"{j}_{x}"
            p = ctx.piv(pn)
            pc = vec3(p.get("co")) if p else None
            if ring not in ctx.ring_verts or pc is None:
                out[pn] = {"ring": ring, "error": "ring missing/invalid" if ring not in ctx.ring_verts
                           else "pivot missing/invalid"}
                ok = False
                continue
            c = ma.v[ctx.ring_verts[ring]].mean(axis=0)
            d = float(np.linalg.norm(c - pc))
            out[pn] = {"ring": ring, "center": c.tolist(), "pivot": pc.tolist(), "dist_mm": mm(d)}
            ok = ok and d <= CENTER_TOL
    for pn in ("spine_01", "spine_02"):
        ring = f"{pn}_0"
        p = ctx.piv(pn)
        pc = vec3(p.get("co")) if p else None
        if ring not in ctx.ring_verts or pc is None:
            out[pn] = {"ring": ring, "error": "ring missing/invalid" if ring not in ctx.ring_verts
                       else "pivot missing/invalid"}
            ok = False
            continue
        z = float(ma.v[ctx.ring_verts[ring], 2].mean())
        out[pn] = {"ring": ring, "ring_z_mm": mm(z), "pivot_z_mm": mm(pc[2]), "dz_mm": mm(abs(z - pc[2]))}
        ok = ok and abs(z - pc[2]) <= CENTER_TOL
    ev.criterion("G2.6", out, th, ok, "ring center = mean of ring vertex positions (world)")


# ---------------------------------------------------------------- G2.7
def _azimuth(P, o, a, ref):
    d = P - o
    s = d @ a
    rad = d - s[:, None] * a
    u = unit(ref - (ref @ a) * a)
    w = np.cross(a, u)
    phi = np.degrees(np.arctan2(rad @ w, rad @ u))
    return s, np.linalg.norm(rad, axis=1), phi


def crit_g2_7(ev, ctx):
    th = ("body poles (valence != 4) inside the shoulder / hip ring regions that lie in the bend sectors = 0 "
          "(shoulder: +Z/-Z +-30 deg about pivots shoulder_x.axis; hip: -Y/+Y +-30 deg about hip_x.axis)")
    if blocked(ev, ["G2.7"], th, ctx.m("blend", "parts", "loops", "pivots") + [ctx.mesh_err, ctx.part_err]):
        return
    ma = ctx.ma
    bval = ctx.parts.get("body")
    fm = ma.part == bval
    cm = fm[ma.loop_poly]
    body_verts = np.unique(ma.loop_vert[cm])
    be = np.unique(ma.loop_edge[cm])
    valence = np.bincount(ma.e[be].ravel(), minlength=ma.nv)
    is_body = np.zeros(ma.nv, bool)
    is_body[body_verts] = True
    poles = body_verts[valence[body_verts] != 4]
    kd = KDTree(len(body_verts))
    for i in body_verts.tolist():
        kd.insert(ma.v[i], i)
    kd.balance()
    regions, ok, n_in_total = {}, True, 0
    in_band_all = set()
    for joint, ref in (("shoulder", np.array([0.0, 0.0, 1.0])), ("hip", np.array([0.0, -1.0, 0.0]))):
        for x in SIDES:
            key = f"{joint}_{x}"
            rn = sorted(n for n in ctx.ring_verts if re.match(rf"^{key}_\d+$", n))
            p = ctx.piv(key)
            o = vec3(p.get("co")) if p else None
            a = vec3(p.get("axis")) if p else None
            if not rn or o is None or a is None:
                regions[key] = {"error": "no valid rings" if not rn else "pivot co/axis missing"}
                ok = False
                continue
            a = unit(a)
            ring_all = np.concatenate([ctx.ring_verts[n] for n in rn])
            region = set()
            for vi in ring_all.tolist():
                for _, idx, _ in kd.find_range(ma.v[vi], POLE_RADIUS):
                    region.add(idx)
            s_r, r_r, _ = _azimuth(ma.v[ring_all], o, a, ref)
            s_k = [float(((ma.v[ctx.ring_verts[n]] - o) @ a).mean()) for n in rn]
            s_bv, r_bv, _ = _azimuth(ma.v[body_verts], o, a, ref)
            between = (s_bv >= min(s_k)) & (s_bv <= max(s_k)) & (r_bv <= r_r.max())
            region.update(body_verts[between].tolist())
            reg = np.array(sorted(region), dtype=np.int64)
            rp = reg[valence[reg] != 4]
            s_p, r_p, phi = _azimuth(ma.v[rp], o, a, ref)
            band = (np.abs(phi) <= BAND_DEG) | (np.abs(phi) >= 180.0 - BAND_DEG)
            rows_in = [{"v": int(v), "valence": int(valence[v]), "azimuth_deg": round(float(ph), 2),
                        "s_mm": mm(ss), "r_mm": mm(rr)}
                       for v, ph, ss, rr, bb in zip(rp.tolist(), phi, s_p, r_p, band) if bb]
            rows_out = [{"v": int(v), "valence": int(valence[v]), "azimuth_deg": round(float(ph), 2)}
                        for v, ph, bb in zip(rp.tolist(), phi, band) if not bb]
            in_band_all.update(rp[band].tolist())
            n_in_total += len(rows_in)
            regions[key] = {"rings": rn, "ring_s_mm": [mm(s) for s in s_k], "region_verts": len(reg),
                            "poles": len(rp), "poles_in_band": len(rows_in),
                            "in_band": cap(rows_in), "outside_band": cap(rows_out)}
    ctx.poles_in = sorted(in_band_all)
    ctx.poles_other = sorted(set(poles.tolist()) - in_band_all)
    ok = ok and n_in_total == 0
    ev.criterion("G2.7", {"poles_in_band_total": n_in_total, "body_poles_total": len(poles),
                          "regions": regions}, th, ok,
                 "valence = number of body-face edges at the vertex; region = body verts within 30 mm of any "
                 "ring vertex of <joint>_x_* U body verts with axial s between min/max ring mean s and axis "
                 "distance <= max ring vertex axis distance (axis = pivots <joint>_x co/axis); azimuth 0 = "
                 "reference (+Z shoulder, -Y hip) projected perpendicular to the axis; band = |az| <= 30 or "
                 ">= 150")


# ---------------------------------------------------------------- G2.8
def crit_g2_8(ev, ctx):
    th = f"GOB_mesh triangles in [{TRI_RANGE[0]}, {TRI_RANGE[1]}]; GOB_club triangles < {CLUB_TRI_MAX}"
    if blocked(ev, ["G2.8"], th, ctx.m("blend") + [ctx.mesh_err]):
        return
    ma = ctx.ma
    measured = {"GOB_mesh_tris": ma.ntri}
    note = "loop_triangles count of obj.data"
    if ma.part is not None and ctx.parts:
        tp = ma.part[ma.tri_poly]
        measured["per_part"] = {n: int((tp == v).sum()) for n, v in ctx.parts.items()}
    else:
        note += "; per-part breakdown unavailable (" + (ctx.miss.get("parts") or ctx.part_err) + ")"
    ok = TRI_RANGE[0] <= ma.ntri <= TRI_RANGE[1]
    if ctx.mc is None:
        measured["GOB_club_tris"] = ctx.club_err or "missing object"
        ok = False
    else:
        measured["GOB_club_tris"] = ctx.mc.ntri
        ok = ok and ctx.mc.ntri < CLUB_TRI_MAX
    ev.criterion("G2.8", measured, th, ok, note)


# ---------------------------------------------------------------- G2.9
def uv_islands(ma):
    """Island id per face: faces joined across edges whose two endpoint UVs match in both faces."""
    c = np.arange(ma.nl)
    order = np.argsort(ma.loop_edge, kind="stable")
    es = ma.loop_edge[order]
    _, start, cnt = np.unique(es, return_index=True, return_counts=True)
    two = cnt == 2
    c1, c2 = order[start[two]], order[start[two] + 1]
    n1, n2 = ma.loop_next[c1], ma.loop_next[c2]
    uv = ma.uv
    a1, a2 = ma.loop_vert[c1], ma.loop_vert[c2]
    # uv of face 2 at vertex a1
    rev = a1 != a2
    u2_a1 = np.where(rev[:, None], uv[n2], uv[c2])
    u2_b1 = np.where(rev[:, None], uv[c2], uv[n2])
    match = (np.abs(uv[c1] - u2_a1).max(axis=1) <= UV_MATCH) & (np.abs(uv[n1] - u2_b1).max(axis=1) <= UV_MATCH)
    f1, f2 = ma.loop_poly[c1][match], ma.loop_poly[c2][match]
    roots = _components(ma.npoly, f1, f2)
    _, isl = np.unique(roots, return_inverse=True)
    del c
    return isl.reshape(-1)


def uv_overlap_pixels(ma, island):
    """Pixels (UV_RES^2, pixel-centre sampling) covered by triangles of >= 2 different islands."""
    owner = np.full((UV_RES, UV_RES), -1, dtype=np.int32)
    conflict = np.zeros((UV_RES, UV_RES), dtype=bool)
    T = ma.uv[ma.tri_l] * UV_RES
    tri_isl = island[ma.tri_poly]
    degenerate = 0
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
        csub[inside & (sub >= 0) & (sub != isl)] = True
        sub[inside & (sub < 0)] = isl
    return int(conflict.sum()), degenerate, int((owner >= 0).sum())


def _uv_basic(ma):
    uv = ma.uv
    finite = np.all(np.isfinite(uv), axis=1)
    out = ~finite | np.any(uv < -UV_EPS, axis=1) | np.any(uv > 1.0 + UV_EPS, axis=1)
    isl = uv_islands(ma)
    ov, degen, covered = uv_overlap_pixels(ma, isl)
    return {"uv_layer": ma.uv_name, "n_uv_layers": ma.n_uv_layers, "loops_out_of_0_1": int(out.sum()),
            "nonfinite_loops": int((~finite).sum()), "uv_min": uv[finite].min(axis=0).tolist() if finite.any() else None,
            "uv_max": uv[finite].max(axis=0).tolist() if finite.any() else None,
            "islands": int(isl.max()) + 1 if len(isl) else 0, "overlap_pixels": ov,
            "degenerate_uv_tris": degen, "covered_pixels": covered}, isl


def crit_g2_9(ev, ctx):
    th = (f"active UV layer; all loop UV in [0,1]; UV island overlap pixels = 0 ({UV_RES}^2 raster); "
          f"head front texel density >= body mean x {DENSITY_RATIO} (GOB_club: [0,1] + overlap only)")
    if blocked(ev, ["G2.9"], th, ctx.m("blend", "parts") + [ctx.mesh_err, ctx.part_err]):
        return
    ma, measured, ok = ctx.ma, {}, True
    if ma.uv is None:
        measured[MESH] = "no active UV layer"
        ok = False
    else:
        info, _ = _uv_basic(ma)
        tri_uv = ma.uv[ma.tri_l]
        a2 = ((tri_uv[:, 1, 0] - tri_uv[:, 0, 0]) * (tri_uv[:, 2, 1] - tri_uv[:, 0, 1])
              - (tri_uv[:, 1, 1] - tri_uv[:, 0, 1]) * (tri_uv[:, 2, 0] - tri_uv[:, 0, 0]))
        poly_uv_area = np.bincount(ma.tri_poly, weights=0.5 * np.abs(a2), minlength=ma.npoly)
        head = (ma.part == ctx.parts.get("head", -1)) & (ma.poly_normal @ np.array([0.0, -1.0, 0.0]) > HEAD_FRONT_DOT)
        body = ma.part == ctx.parts.get("body", -1)
        dens = {}
        for key, m in (("head_front", head), ("body", body)):
            A3, Auv = float(ma.poly_area[m].sum()), float(poly_uv_area[m].sum())
            dens[key] = {"faces": int(m.sum()), "area3d_m2": round(A3, 6), "uv_area": round(Auv, 6),
                         "density_uv_per_m2": (Auv / A3) if A3 > 0 else None}
        dh, db = dens["head_front"]["density_uv_per_m2"], dens["body"]["density_uv_per_m2"]
        ratio = (dh / db) if (dh is not None and db) else None
        dens["ratio_area"] = ratio
        dens["ratio_linear_sqrt"] = math.sqrt(ratio) if ratio is not None and ratio >= 0 else None
        info["texel_density"] = dens
        info["per_material_slot_report"] = _uv_per_slot(ctx)
        measured[MESH] = info
        ok = ok and info["loops_out_of_0_1"] == 0 and info["overlap_pixels"] == 0 and \
            ratio is not None and ratio >= DENSITY_RATIO
    if ctx.mc is None:
        measured[CLUB] = ctx.club_err or "missing object"
        ok = False
    elif ctx.mc.uv is None:
        measured[CLUB] = "no active UV layer"
        ok = False
    else:
        info, _ = _uv_basic(ctx.mc)
        measured[CLUB] = info
        ok = ok and info["loops_out_of_0_1"] == 0 and info["overlap_pixels"] == 0
    ev.criterion("G2.9", measured, th, ok,
                 "islands = faces joined across edges whose endpoint UVs match (<= 1e-6) in both faces; "
                 "overlap = pixel centres inside triangles of >= 2 different islands; texel density = "
                 "sum UV area / sum 3D area (m^2), head front = head faces with normal . (0,-1,0) > 0.7; "
                 "ok uses ratio_area; every face is judged, the HR2 mouth faces included; per_material_slot_report "
                 "(report only) = faces / loops out of [0,1] / UV islands touched per GOB_mesh material slot")


def _uv_per_slot(ctx):
    """G2.9 report: per GOB_mesh material slot, faces, loops out of [0,1] and UV islands touched."""
    ma, mo = ctx.ma, ctx.mesh_obj
    mi = np.empty(ma.npoly, dtype=np.int32)
    mo.data.polygons.foreach_get("material_index", mi)
    uv = ma.uv
    bad_loop = ~np.all(np.isfinite(uv), axis=1) | np.any(uv < -UV_EPS, axis=1) | np.any(uv > 1.0 + UV_EPS, axis=1)
    isl = uv_islands(ma)
    names = [s.material.name if s.material else None for s in mo.material_slots]
    out = {}
    for k in sorted(set(int(x) for x in mi.tolist())):
        fm = mi == k
        out[str(k)] = {"slot": names[k] if k < len(names) else None, "faces": int(fm.sum()),
                       "loops_out_of_0_1": int(bad_loop[fm[ma.loop_poly]].sum()),
                       "uv_islands": int(len(np.unique(isl[fm])))}
    return out


# ---------------------------------------------------------------- G2.10
def _normals_info(obj):
    me = obj.data
    nl = len(me.loops)
    has = getattr(me, "has_custom_normals", None)
    attr = me.attributes.get("custom_normal")
    buf = np.empty(nl * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", buf)
    cn = buf.reshape(nl, 3)
    nonfinite = int((~np.all(np.isfinite(cn), axis=1)).sum())
    ln = np.linalg.norm(np.nan_to_num(cn), axis=1)
    return {"has_custom_normals": has,
            "custom_normal_attribute": None if attr is None else {"domain": attr.domain,
                                                                  "data_type": attr.data_type},
            "corners": nl, "nan_or_inf_corner_normals": nonfinite,
            "zero_length_corner_normals": int((ln < 1e-6).sum())}


def crit_g2_10(ev, ctx):
    th = "has_custom_normals true and 0 NaN/inf corner normals (GOB_mesh, GOB_club)"
    if blocked(ev, ["G2.10"], th, ctx.m("blend") + [ctx.mesh_err]):
        return
    measured, ok = {}, True
    for name, obj, err in ((MESH, ctx.mesh_obj, ""), (CLUB, ctx.club_obj, ctx.club_err)):
        if obj is None or obj.type != "MESH":
            measured[name] = err or "missing object"
            ok = False
            continue
        info = _normals_info(obj)
        measured[name] = info
        ok = ok and info["has_custom_normals"] is True and info["nan_or_inf_corner_normals"] == 0
    ev.criterion("G2.10", measured, th, ok,
                 "Mesh.has_custom_normals + Mesh.corner_normals; zero-length count reported only")


# ---------------------------------------------------------------- G2.17
def crit_g2_17(ev, ctx):
    th = (f"GOB_mesh material slots exactly {list(MAT_SLOTS)} (this order), every face material_index < "
          f"{len(MAT_SLOTS)}; shape keys exactly {list(SHAPE_KEYS)}: Key.use_relative True, reference key "
          f"'Basis', mouth_open.relative_key 'Basis'")
    if blocked(ev, ["G2.17"], th, ctx.m("blend") + [ctx.mesh_err]):
        return
    mo, ma = ctx.mesh_obj, ctx.ma
    me = mo.data
    slots = [s.material.name if s.material else None for s in mo.material_slots]
    mi = np.empty(ma.npoly, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    n_sl = max(len(slots), int(mi.max()) + 1 if len(mi) else 0)
    fps = np.bincount(mi, minlength=n_sl)
    tps = np.bincount(mi[ma.tri_poly], minlength=n_sl)
    per_slot = {}
    for k in range(n_sl):
        row = {"slot": slots[k] if k < len(slots) else None, "faces": int(fps[k]), "tris": int(tps[k])}
        if ma.part is not None:
            vals, cnt = np.unique(ma.part[mi == k], return_counts=True)
            row["faces_per_part_report"] = {ctx.part_names.get(int(v), f"part_{int(v)}"): int(c)
                                            for v, c in zip(vals, cnt)}
        per_slot[str(k)] = row
    slots_ok = slots == list(MAT_SLOTS) and (not len(mi) or int(mi.max()) < len(MAT_SLOTS))
    measured = {"material_slots": slots, "n_slots": len(slots), "per_slot": per_slot,
                "modifiers_report": [m.type for m in mo.modifiers]}
    key = me.shape_keys
    keys_ok = False
    if key is None:
        measured["shape_keys"] = None
    else:
        kbs = list(key.key_blocks)
        names = [kb.name for kb in kbs]
        ref = key.reference_key
        nv = len(me.vertices)
        base = np.empty(nv * 3, dtype=np.float32)
        (ref if ref is not None else kbs[0]).data.foreach_get("co", base)
        base = base.reshape(nv, 3).astype(np.float64)
        co = np.empty(nv * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
        rows = {}
        for kb in kbs:
            c = np.empty(nv * 3, dtype=np.float32)
            kb.data.foreach_get("co", c)
            d = np.linalg.norm(c.reshape(nv, 3).astype(np.float64) - base, axis=1)
            rows[kb.name] = {"relative_key": kb.relative_key.name if kb.relative_key else None,
                             "value": rnd4(kb.value), "slider": [rnd4(kb.slider_min), rnd4(kb.slider_max)],
                             "mute": bool(kb.mute), "vertex_group": kb.vertex_group or None,
                             "interpolation": kb.interpolation,
                             "max_disp_vs_basis_mm": mm(d.max()) if nv else None,
                             "n_verts_moved": int((d > KEY_MOVE_EPS).sum())}
        mouth = rows.get("mouth_open")
        measured["shape_keys"] = {"names": names, "use_relative": bool(key.use_relative),
                                  "reference_key": ref.name if ref is not None else None, "keys": rows,
                                  "vertex_co_vs_basis_max_mm_report": (
                                      mm(np.abs(co.reshape(nv, 3) - base).max()) if nv else None),
                                  "key_animation_data_report": key.animation_data is not None}
        keys_ok = (names == list(SHAPE_KEYS) and bool(key.use_relative) and ref is not None
                   and ref.name == "Basis" and mouth is not None and mouth["relative_key"] == "Basis")
    ev.criterion("G2.17", measured, th, bool(slots_ok and keys_ok),
                 "2026-09-26 HR2 (rig/data/hr2_contract.md): mesh contract of the retopo final. material slots = "
                 "GOB_mesh.material_slots material names; faces / tris per slot from polygon material_index "
                 "(faces_per_part_report = part_id of those faces, report only); shape keys from "
                 "GOB_mesh.data.shape_keys; max_disp_vs_basis_mm / n_verts_moved (> "
                 f"{KEY_MOVE_EPS * 1000:g} mm) = key block co vs reference key co (report); "
                 "vertex_co_vs_basis = obj.data vertex co (the geometry every other G2 row reads) vs Basis, report "
                 "only; modifiers reported only")


# ---------------------------------------------------------------- G2.14
def _depth_stats(depth, thr):
    fin = depth[np.isfinite(depth)]
    return {"n": int(len(depth)), "min_depth_mm": mm(depth.min()) if len(depth) and np.isfinite(depth.min()) else None,
            "median_depth_mm": mm(np.median(fin)) if len(fin) else None,
            "violations": int((depth < thr).sum())}


def crit_g2_14(ev, ctx):
    tha = "wrist_x_end ring verts inside hand_x, ankle_x_end ring verts inside shoe_x: depth >= 3 mm"
    thb = "head part verts with z <= head min z + 2 mm inside body: depth >= 3 mm"
    thc = "body verts with z in [mean z belt_bot_0, mean z belt_top_0] inside belt: depth >= 0.5 mm"
    base = [ctx.mesh_err, ctx.part_err]
    ma = ctx.ma
    note = "depth = signed inside depth w.r.t. the part's shells (max over shells; ray parity + BVH nearest)"
    # a
    if not blocked(ev, ["G2.14a"], tha, ctx.m("blend", "parts", "loops") + base):
        out, ok = {}, True
        for ring_j, part_j in (("wrist", "hand"), ("ankle", "shoe")):
            for x in SIDES:
                ring, part = f"{ring_j}_{x}_end", f"{part_j}_{x}"
                if ring not in ctx.ring_verts:
                    out[ring] = {"error": "ring missing/invalid"}
                    ok = False
                    continue
                sh = ctx.shells_of(part)
                d = part_depth(sh, ma.v[ctx.ring_verts[ring]])
                st = _depth_stats(d, END_DEPTH)
                st["inside_part"] = part
                st["part_shells"] = [s.label for s in sh]
                out[ring] = st
                ok = ok and bool(sh) and st["violations"] == 0
        ev.criterion("G2.14a", out, tha, ok, note)
    # b
    if not blocked(ev, ["G2.14b"], thb, ctx.m("blend", "parts") + base):
        hv = ctx.verts_of_part("head")
        if not len(hv):
            ev.criterion("G2.14b", "head part has no faces", thb, False)
        else:
            zmin = float(ma.v[hv, 2].min())
            sel = hv[ma.v[hv, 2] <= zmin + HEAD_BOTTOM_BAND]
            sh = ctx.shells_of("body")
            d = part_depth(sh, ma.v[sel])
            st = _depth_stats(d, END_DEPTH)
            st.update({"head_min_z_mm": mm(zmin), "body_shells": [s.label for s in sh],
                       "worst": cap([{"v": int(v), "depth_mm": mm(dd)} for v, dd in
                                     sorted(zip(sel.tolist(), d.tolist()), key=lambda t: t[1]) if dd < END_DEPTH])})
            ev.criterion("G2.14b", st, thb, bool(sh) and st["violations"] == 0,
                         note + "; head = all shells of part head (bottom band over the whole part)")
    # c
    if not blocked(ev, ["G2.14c"], thc, ctx.m("blend", "parts", "loops") + base):
        if "belt_bot_0" not in ctx.ring_verts or "belt_top_0" not in ctx.ring_verts:
            ev.criterion("G2.14c", "belt_bot_0 / belt_top_0 ring missing or invalid", thc, False)
        else:
            zb = float(ma.v[ctx.ring_verts["belt_bot_0"], 2].mean())
            zt = float(ma.v[ctx.ring_verts["belt_top_0"], 2].mean())
            bv = ctx.verts_of_part("body")
            sel = bv[(ma.v[bv, 2] >= min(zb, zt)) & (ma.v[bv, 2] <= max(zb, zt))]
            sh = ctx.shells_of("belt")
            d = part_depth(sh, ma.v[sel])
            st = _depth_stats(d, BELT_DEPTH)
            st.update({"z_range_mm": [mm(zb), mm(zt)], "belt_shells": [s.label for s in sh],
                       "worst": cap([{"v": int(v), "depth_mm": mm(dd)} for v, dd in
                                     sorted(zip(sel.tolist(), d.tolist()), key=lambda t: t[1]) if dd < BELT_DEPTH])})
            ev.criterion("G2.14c", st, thc, bool(sh) and len(sel) > 0 and st["violations"] == 0,
                         note + "; depth is to the nearest belt surface (inner or outer)")


# ---------------------------------------------------------------- G2.16
def _fit_sphere(p):
    A = np.column_stack([2 * p, np.ones(len(p))])
    b = (p ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:3]
    return c, float(math.sqrt(max(sol[3] + c @ c, 0.0)))


def _ransac_sphere(p):
    rng = np.random.default_rng(RANSAC["seed"])
    idx = rng.integers(0, len(p), size=(RANSAC["iters"], 4))
    best_n, best = -1, None
    for q in p[idx]:
        A = 2 * (q[1:] - q[0])
        b = (q[1:] ** 2).sum(1) - (q[0] ** 2).sum()
        if abs(np.linalg.det(A)) < 1e-12:
            continue
        c = np.linalg.solve(A, b)
        r = float(np.linalg.norm(q[0] - c))
        if not (RANSAC["rmin"] <= r <= RANSAC["rmax"]):
            continue
        n = int((np.abs(np.linalg.norm(p - c, axis=1) - r) < RANSAC["tol"]).sum())
        if n > best_n:
            best_n, best = n, (c, r)
    if best is None:
        return None
    c, r = best
    inl = p
    for _ in range(RANSAC["refits"]):
        inl = p[np.abs(np.linalg.norm(p - c, axis=1) - r) < RANSAC["tol"]]
        c, r = _fit_sphere(inl)
    return c, r, len(inl)


def load_src(ctx):
    """Append SRC_hi / SRC_club_hi from the source blend, read world vertices, remove them again."""
    names = []
    with bpy.data.libraries.load(str(SRC_BLEND), link=False) as (dfrom, dto):
        names = [n for n in (SRC_BODY, SRC_CLUB) if n in dfrom.objects]
        dto.objects = list(names)
    for n, ob in zip(names, dto.objects):
        if ob is None or ob.type != "MESH":
            continue
        me = ob.data
        co = np.empty(len(me.vertices) * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
        mw = np.array(ob.matrix_world, dtype=np.float64)
        ctx.src[n] = co.reshape(-1, 3).astype(np.float64) @ mw[:3, :3].T + mw[:3, 3]
        bpy.data.objects.remove(ob, do_unlink=True)
        if me.users == 0:
            bpy.data.meshes.remove(me)


def _hand_sphere(ctx, part):
    """Least-squares sphere of the part's GOB_mesh verts -> dict or None."""
    hv = ctx.verts_of_part(part)
    if len(hv) < 4:
        return None
    P = ctx.ma.v[hv]
    c, r = _fit_sphere(P)
    res = np.linalg.norm(P - c, axis=1) - r
    return {"center": [round(float(x), 6) for x in c], "r_mm": mm(r), "d_mm": mm(2 * r),
            "rms_mm": mm(math.sqrt(float((res ** 2).mean()))), "max_abs_res_mm": mm(np.abs(res).max()),
            "n_verts": int(len(hv)), "_c": c, "_r": r}


def _line_hits(shells, o, d):
    """Signed t (m) along unit d from o of every surface hit of the shells on the full line."""
    ts = []
    for sh in shells:
        bvh = sh.bvh()
        for sgn in (1.0, -1.0):
            dv = Vector((d * sgn).tolist())
            p = Vector(o.tolist())
            for _ in range(512):
                hit = bvh.ray_cast(p, dv)
                if hit[0] is None:
                    break
                ts.append(sgn * float((np.array(hit[0]) - o) @ (d * sgn)))
                p = hit[0] + dv * RAY_EPS
    return sorted(ts)


def crit_g2_16(ev, ctx):
    th = (f"(a) every GOB_club surface sample with grip_r axial s in [-{HANDLE_HALF * 1000:g}, +{HANDLE_HALF * 1000:g}] mm "
          f"inside hand_r shell by >= {HANDLE_DEPTH * 1000:g} mm; (b) grip_r axis line crosses hand_r surface "
          f"in front of and behind grip_r, chord (front + back distance) >= {CHORD_RATIO:g} x ball diameter "
          f"(hand_r least-squares sphere)")
    if blocked(ev, ["G2.16"], th, ctx.m("blend", "parts", "pivots") + [ctx.mesh_err, ctx.part_err, ctx.club_err]):
        return
    g = ctx.piv("grip_r")
    gc = vec3(g.get("co")) if g else None
    ga = vec3(g.get("axis")) if g else None
    if gc is None or ga is None or np.linalg.norm(ga) == 0:
        ev.criterion("G2.16", "pivots.json grip_r co/axis missing or invalid", th, False)
        return
    ga = unit(ga)
    spheres = {p: _hand_sphere(ctx, p) for p in ("hand_l", "hand_r")}
    rep_sph = {p: (None if s is None else {k: v for k, v in s.items() if not k.startswith("_")})
               for p, s in spheres.items()}
    shells = ctx.shells_of("hand_r")
    if not shells or spheres["hand_r"] is None:
        ev.criterion("G2.16", {"hand_spheres": rep_sph, "hand_r_shells": len(shells)}, th, False,
                     "hand_r part has no faces (or < 4 verts)")
        return
    # (a) handle section inside hand_r: uniform barycentric samples on GOB_club faces crossing |s| <= HALF
    T = ctx.mc.v[ctx.mc.tri_v]
    ts_ = (T - gc) @ ga
    cross = np.nonzero((ts_.min(axis=1) <= HANDLE_HALF) & (ts_.max(axis=1) >= -HANDLE_HALF))[0]
    step, C = HANDLE_STEP, np.zeros((0, 3))
    for _ in range(6):
        pts = []
        for ti in cross.tolist():
            a, b, c = T[ti]
            e = max(np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c))
            n = max(1, int(math.ceil(e / step)))
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
            m = (i + j) <= n
            u, w = i[m] / n, j[m] / n
            pts.append(a + np.outer(u, b - a) + np.outer(w, c - a))
        C = np.unique(np.round(np.concatenate(pts), 9), axis=0) if pts else np.zeros((0, 3))
        C = C[np.abs((C - gc) @ ga) <= HANDLE_HALF]
        if len(C) >= HANDLE_MIN_N or not len(cross):
            break
        step *= 0.5
    sc = (C - gc) @ ga
    dep = part_depth(shells, C) if len(C) else np.zeros(0)
    bad = np.nonzero(dep < HANDLE_DEPTH)[0]
    ctx.g216_flagged = C[bad].tolist()
    order = np.argsort(dep, kind="stable")
    rows = [{"s_mm": mm(sc[i]), "depth_mm": mm(dep[i]),
             "co": [round(float(x), 5) for x in C[i]]} for i in order[:LIST_CAP].tolist()]
    ok_a = len(C) > 0 and len(bad) == 0
    # (b) grip axis chord through hand_r
    ts = _line_hits(shells, gc, ga)
    fr = [t for t in ts if t > 0]
    bk = [t for t in ts if t < 0]
    t_front = min(fr) if fr else None
    t_back = -max(bk) if bk else None
    chord = (t_front + t_back) if (t_front is not None and t_back is not None) else None
    diam = 2.0 * spheres["hand_r"]["_r"]
    ok_b = chord is not None and chord >= CHORD_RATIO * diam
    gdep = float(part_depth(shells, gc[None, :])[0])
    ev.criterion("G2.16", {
        "handle": {"n_samples": int(len(C)), "n_faces_crossing": int(len(cross)), "step_mm": mm(step),
                   "n_below_depth": int(len(bad)),
                   "min_depth_mm": mm(dep.min()) if len(dep) else None,
                   "median_depth_mm": mm(np.median(dep)) if len(dep) else None,
                   "shallowest": rows, "ok": bool(ok_a)},
        "axis_chord": {"t_front_mm": mm(t_front), "t_back_mm": mm(t_back), "chord_mm": mm(chord),
                       "ball_d_mm": mm(diam), "limit_mm": mm(CHORD_RATIO * diam),
                       "chord_over_d": None if chord is None else round(chord / diam, 4),
                       "line_hits_mm": cap([mm(t) for t in ts]), "n_line_hits": len(ts),
                       "grip_co_depth_mm": mm(gdep), "ok": bool(ok_b)},
        "hand_spheres": rep_sph,
        "hand_r_shells": [{"label": s.label, "closed": s.closed, "verts": int(len(s.verts))} for s in shells],
        "grip_r": {"co": gc.tolist(), "axis": ga.tolist()}},
        th, bool(ok_a and ok_b),
        "s = (p - grip_r.co) . grip_r.axis (pivots.json); (a) p = uniform barycentric grid samples (edge "
        "subdivision n = ceil(longest edge / step), step 1 mm, halved until n_samples >= 200) on the GOB_club "
        "world triangles whose s range overlaps [-20, +20] mm, duplicates merged, kept when |s| <= 20 mm; "
        "ok_a needs n_samples > 0; depth = signed inside "
        "depth w.r.t. hand_r shells (BVH nearest distance, + inside by ray-parity majority of 3 directions, "
        "max over hand_r shells); shallowest = up to 40 handle samples by ascending depth; axis chord: all "
        "hand_r surface hits on the grip_r axis line (rays both ways from grip_r.co, re-cast past each hit), "
        "t_front = nearest hit at t > 0 (club head side), t_back = nearest hit at t < 0, chord = t_front + "
        "t_back; ball = least-squares sphere (algebraic fit) of all GOB_mesh verts of the part, rms/max_abs "
        "= radial residuals; old core-cylinder / palm-sphere / stub logic removed (ball fist, 2026-09-25)")


# ---------------------------------------------------------------- renders
_TEMP = {"objs": [], "curves": []}


def _curve_obj(name, splines, radius, rgba, cyclic):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = radius
    cu.bevel_resolution = 3
    cu.use_fill_caps = True
    for pts in splines:
        pts = np.asarray(pts, dtype=np.float64)
        sp = cu.splines.new("POLY")
        sp.points.add(len(pts) - 1)
        sp.points.foreach_set("co", np.hstack([pts, np.ones((len(pts), 1))]).ravel().tolist())
        sp.use_cyclic_u = cyclic
    ob = bpy.data.objects.new(name, cu)
    ob.color = rgba
    bpy.context.scene.collection.objects.link(ob)
    _TEMP["objs"].append(ob)
    _TEMP["curves"].append(cu)
    return ob


def _clear_temp():
    for ob in _TEMP["objs"]:
        bpy.data.objects.remove(ob, do_unlink=True)
    for cu in _TEMP["curves"]:
        bpy.data.curves.remove(cu)
    _TEMP["objs"].clear()
    _TEMP["curves"].clear()


def _ring_color(name):
    fam = name.split("_")[0]
    base = np.array(RING_COLORS.get(fam, (1.0, 1.0, 1.0)))
    m = re.search(r"_(\d+)$", name)
    f = 1.0
    if name.endswith("_end"):
        f = 0.45
    elif m and fam in ("shoulder", "hip", "elbow", "knee", "wrist", "ankle"):
        f = {0: 1.0, 1: 0.7, 2: 0.5}.get(int(m.group(1)), 0.4)
        if fam in ("elbow", "knee") and int(m.group(1)) == 1:
            f = 1.0
        elif fam in ("elbow", "knee"):
            f = 0.55
    return tuple(np.clip(base * f, 0, 1).tolist()) + (1.0,)


def _projected_h(view, mn, mx, pad):
    d, right, up = goblib._view_basis(view)
    corners = [Vector((x, y, z)) for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
    us = [c.dot(right) for c in corners]
    vs = [c.dot(up) for c in corners]
    w, h = max(us) - min(us), max(vs) - min(vs)
    if pad:
        h += 0.1 * max(w, h)
    return h, d


def render_job(ctx, out, view, R, frame_pts, pad, rings, markers, wire=True):
    """rings: [(name, pts)]; markers: [(pts, rgba)].  R: 4x4 applied to every rendered object."""
    ma, mesh = ctx.ma, ctx.mesh_obj
    R3 = np.array(R.to_3x3(), dtype=np.float64)
    P = (ma.v if frame_pts is None else np.asarray(frame_pts)) @ R3.T
    if frame_pts is None:  # whole mesh; explicit frame (curve bboxes are not used for framing)
        pad = 0.05 * float((P.max(axis=0) - P.min(axis=0)).max())
    mn, mx = P.min(axis=0) - pad, P.max(axis=0) + pad
    h, d = _projected_h(view, mn, mx, False)
    frame = (tuple(mn), tuple(mx))
    px = h / 1024.0
    d_obj = R3.T @ np.array(d)
    m0 = mesh.matrix_world.copy()
    colors = {mesh.name: MESH_COLOR}
    try:
        for name, pts in rings:
            ob = _curve_obj(f"_g2ring_{name}", [pts], 2.5 * px, _ring_color(name), True)
            colors[ob.name] = tuple(ob.color)
        for k, (pts, rgba) in enumerate(markers):
            if not len(pts):
                continue
            r = 5.0 * px
            segs = [np.stack([p - d_obj * r, p - d_obj * (r + px)]) for p in np.asarray(pts)]
            ob = _curve_obj(f"_g2mark_{k}", segs, r, rgba, False)
            colors[ob.name] = rgba
        mesh.matrix_world = R @ m0
        for ob in _TEMP["objs"]:
            ob.matrix_world = R.copy()
        bpy.context.view_layer.update()
        goblib.ortho_render([mesh] + list(_TEMP["objs"]), view, out, res_h=1024, frame_bbox=frame,
                            wire=wire, colors=colors)
    finally:
        mesh.matrix_world = m0
        _clear_temp()
        bpy.context.view_layer.update()
    return out


def do_renders(ev, ctx, outdir):
    info = {"written": [], "skipped": {}, "views": {}}
    if ctx.ma is None:
        info["skipped"]["all"] = ctx.miss.get("blend") or ctx.mesh_err
        return info
    ma = ctx.ma
    rings = [(n, ma.v[v]) for n, v in sorted(ctx.ring_verts.items())]
    markers = [(ma.v[ctx.poles_other], POLE_OUT_COLOR), (ma.v[ctx.poles_in], POLE_IN_COLOR)]
    I4 = Matrix.Identity(4)

    def ring_pts(prefix):
        sel = [ma.v[v] for n, v in ctx.ring_verts.items() if re.match(rf"^{prefix}_\d+$", n)]
        return np.concatenate(sel) if sel else None

    def piv_box(name):
        p = ctx.piv(name)
        c = vec3(p.get("co")) if p else None
        return None if c is None else np.array([c - 0.12, c + 0.12])

    jobs = [
        ("loops_front.png", "front", I4, None, 0.0, "front, whole GOB_mesh"),
        ("loops_shoulder_l.png", "three_quarter", Matrix.Rotation(math.radians(-90.0), 4, "Z"),
         "shoulder_l", 0.06,
         "three_quarter camera with the mesh turned -90 deg about Z (= front-left 45 deg, 15 deg above)"),
        ("loops_shoulder_r.png", "three_quarter", I4, "shoulder_r", 0.06, "three_quarter (front-right 45 deg, 15 deg above)"),
        ("loops_hip.png", "front", Matrix.Rotation(math.radians(-25.0), 4, "X"), "hip", 0.10,
         "front camera with the mesh tilted -25 deg about X (= from 25 deg below the front)"),
    ]
    for fname, view, R, key, pad, desc in jobs:
        out = outdir / fname
        fp = None
        if key == "hip":
            parts = [x for x in (ring_pts("hip_l"), ring_pts("hip_r")) if x is not None]
            fp = np.concatenate(parts) if parts else None
            if fp is None:
                boxes = [b for b in (piv_box("hip_l"), piv_box("hip_r")) if b is not None]
                fp = np.concatenate(boxes) if boxes else None
        elif key is not None:
            fp = ring_pts(key)
            if fp is None:
                fp = piv_box(key)
        if key is not None and fp is None:
            info["skipped"][fname] = "no rings and no pivot for framing"
            continue
        render_job(ctx, out, view, R, fp, pad, rings, markers, wire=True)
        ev.render(out)
        info["written"].append(fname)
        info["views"][fname] = desc
    # G2.16 hand_r close-ups (GOB_club excluded)
    hv = ctx.verts_of_part("hand_r")
    if len(hv):
        fl = np.array(ctx.g216_flagged, dtype=np.float64).reshape(-1, 3)
        mk = [(fl, POLE_IN_COLOR)] if len(fl) else []
        for fname, view in (("hand_r_top.png", "top"), ("hand_r_front.png", "front")):
            out = outdir / fname
            render_job(ctx, out, view, I4, ma.v[hv], 0.02, [], mk, wire=True)
            ev.render(out)
            info["written"].append(fname)
            info["views"][fname] = f"{view}, frame = hand_r part verts bbox + 20 mm, GOB_mesh only, " \
                                   "red = G2.16 GOB_club handle samples < 1 mm inside hand_r"
    else:
        info["skipped"]["hand_r_*.png"] = "hand_r part has no faces (or parts.json/part_id unavailable)"
    return info


# ---------------------------------------------------------------- main
_STATE = {"missing": []}


def _load_json(path, ctx, key):
    if not path.exists():
        _STATE["missing"].append(rel(path))
        ctx.miss[key] = f"missing input: {rel(path)}"
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def setup(ctx):
    mo = bpy.data.objects.get(MESH)
    co = bpy.data.objects.get(CLUB)
    ctx.mesh_obj, ctx.club_obj = mo, co
    if mo is None or mo.type != "MESH":
        ctx.mesh_err = "missing object: GOB_mesh" if mo is None else f"GOB_mesh is {mo.type}, not MESH"
        ctx.mesh_obj = None if mo is None else mo
        ctx.ma = None
    else:
        ctx.ma = MeshArrays(mo)
        if ctx.ma.part is None:
            ctx.part_err = f"GOB_mesh part_id FACE INT attribute missing/invalid ({ctx.ma.part_attr})"
    if co is None or co.type != "MESH":
        ctx.club_err = "missing object: GOB_club" if co is None else f"GOB_club is {co.type}, not MESH"
    else:
        ctx.mc = MeshArrays(co)
        if ctx.mc.npoly:
            ctx.club_shells, _ = build_shells(ctx.mc, np.zeros(ctx.mc.npoly, np.int64), {0: CLUB})
    if isinstance(ctx.parts, dict):
        ctx.part_names = {v: k for k, v in ctx.parts.items() if isinstance(v, int)}
        if ctx.ma is not None and ctx.ma.part is not None and ctx.ma.npoly:
            ctx.shells, ctx.shell_of_face = build_shells(ctx.ma, ctx.ma.part, ctx.part_names)
    elif ctx.parts is not None:
        ctx.part_err = ctx.part_err or "parts.json is not an object"


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
    if not SRC_BLEND.exists():
        _STATE["missing"].append(rel(SRC_BLEND))
        ctx.miss["src"] = f"missing input: {rel(SRC_BLEND)}"
    else:
        ev.add_input(SRC_BLEND)
    ev.add_stage_inputs("s02")

    ctx.parts = _load_json(PARTS_JSON, ctx, "parts")
    ctx.loops = _load_json(LOOPS_JSON, ctx, "loops")
    ctx.pivots = _load_json(PIVOTS_JSON, ctx, "pivots")

    rep = {}
    if not ctx.miss.get("blend"):
        setup(ctx)
        if ctx.ma is not None and ctx.loops is not None:
            rep = validate_rings(ctx)
        if not ctx.miss.get("src"):
            load_src(ctx)

    try:
        crit_g2_1(ev, ctx)
        crit_g2_2(ev, ctx)
        crit_g2_3(ev, ctx)
        crit_g2_4(ev, ctx)
        crit_g2_5(ev, ctx, rep)
        crit_g2_6(ev, ctx)
        crit_g2_7(ev, ctx)
        crit_g2_8(ev, ctx)
        crit_g2_9(ev, ctx)
        crit_g2_10(ev, ctx)
        crit_g2_14(ev, ctx)
        crit_g2_16(ev, ctx)
        crit_g2_17(ev, ctx)
        th15 = "[U] user visual approval of the loop renders (ok = the 4 loop renders were written)"
        if ctx.miss.get("blend") or ctx.ma is None:
            ev.criterion("G2.15", ctx.miss.get("blend") or ctx.mesh_err, th15, False)
        else:
            info = do_renders(ev, ctx, outdir)
            need = {"loops_front.png", "loops_shoulder_l.png", "loops_shoulder_r.png", "loops_hip.png"}
            info["ring_objects"] = len(ctx.ring_verts)
            info["pole_markers"] = {"in_band_red": len(ctx.poles_in), "other_yellow": len(ctx.poles_other)}
            ev.criterion("G2.15", info, th15, need <= set(info["written"]),
                         "rings = beveled curves through ring verts (colour per joint, center rings bright, "
                         "_end rings dark); poles = camera-facing disks (red: G2.7 in-band, yellow: other body "
                         "poles); black wire = GOB_mesh edges; temporary objects removed, blend not saved")
    finally:
        ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
