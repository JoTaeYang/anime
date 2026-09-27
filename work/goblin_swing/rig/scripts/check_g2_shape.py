"""check_g2_shape - Gate G2 shape checker (design doc d-23 section 7, G2.11 .. G2.13).

Compares the retopo union RETOPO = GOB_mesh U GOB_club (rig/gob_r01_retopo.blend) with
the source union SRC = SRC_hi U SRC_club_hi (meshes appended from rig/gob_r00_source.blend
with bpy.data.libraries.load; that file is never modified).  Unions are compared because
the source club is cut inside the fist and leaves a shaft stub on the body mesh, while the
retopo rebuilds the club as one lathe and trims the stub; hidden regions cancel out.

  G2.11a  RETOPO->SRC surface deviation, G2.11b  SRC->RETOPO, G2.11c  per-part table;
          hidden and interface points are excluded from the judgement (spec G2.11)
  G2.12_<view>  ortho silhouette contour max deviation (front, side, three_quarter)
  G2.13   renders in rig/inspect/G2/: overlay_<view>.png (grey = both, red = SRC only,
          blue = RETOPO only), head_closeup_src.png, head_closeup_retopo.png and
          head_closeup.png (left SRC with a red strip, right RETOPO with a blue strip)

HR2 (2026-09-26, rig/data/hr2_contract.md): GOB_mesh is read in its Basis shape (closed mouth:
show_only_shape_key with the reference key active, in memory only).  Two more design-change
regions are excluded from the G2.11 judgement in BOTH directions and from the G2.12 judged contour,
and reported (counts + deviation stats):
  mouth         rig/data/mouth.json 'region' (written by s02e for the checked blend; its output
                sha256 must equal the checked blend): RETOPO triangles of region.output_faces
                (all_new + by_group) / region.faces, or of material slot >= 1, or whose vertices are all
                in region.verts + region.output_verts + new_vertices (appended_range, groups) +
                boundary_loop with >= 1 not on the boundary loop; plus every point (RETOPO head /
                SRC_hi) whose (x, z) lies inside the region polygon (region.boundary_loop_co or another
                world polygon when given, else the boundary_loop vertices in Basis world) with y inside
                the mouth vertex y range +- MOUTH_Y_MARGIN (or region.y_range)
  head zones    rig/data/head_design_zones.json (written by s02c): every entry whose 'use' starts with
                'exclude' (an entry without 'use' only when it is in ZONE_KEYS; eye_gap_xneg must be
                present; record-only entries are listed): (x, z) inside any polygons_xz AND y in y_range
                AND, when the entry has a depth_rule {axis, apex, exclude_depth_min_m}, (local front
                depth - point depth) > exclude_depth_min_m, depth = (p - apex) . axis, local front =
                first SRC_hi hit of a ray along -axis from the point's eye-plane position + 0.15 m along
                +axis (no hit -> not excluded); RETOPO head and SRC_hi points
A missing / unusable mouth.json or head_design_zones.json blocks G2.11 and G2.12 with the reason.

Run:  bl.ps1 -Script check_g2_shape.py -Blend gob_r01_retopo.blend
Output: rig/inspect/G2/check_g2_shape.json.  No blend is saved; temporary objects,
meshes, materials and images are removed.
Exit: 0 when the evidence was written; 2 on a script error (run_main) or when an input
file is missing (the evidence is still written, then "missing input" is printed).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.  Distances in mm.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402
import compare_g9 as g9  # noqa: E402  (G2.12: G9.6 close_disk / fill_small_holes reused, T121e)

import json  # noqa: E402
import shutil  # noqa: E402
import tempfile  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

GATE = "G2"
CHECKER = "check_g2_shape"
BLEND = goblib.RIG / "gob_r01_retopo.blend"
SRC_BLEND = goblib.RIG / "gob_r00_source.blend"
PARTS = goblib.DATA / "parts.json"
PIVOTS = goblib.DATA / "pivots.json"
MOUTH_JSON = goblib.DATA / "mouth.json"                # HR2 mouth design-change region (s02e)
ZONES_JSON = goblib.DATA / "head_design_zones.json"    # HR2 head design-change zones (s02c)
ZONE_KEYS = ("eye_gap_xneg",)                          # required zones (hr2_contract.md, user decision 2026-09-26)
ZONE_RAY_START = 0.15       # m: depth rule ray start in front of the apex plane (head_design_zones depth_rule)
MOUTH_Y_MARGIN = 0.005      # m: mouth region y range = mouth vertex (Basis) y range +- this
MOUTH_POLY_KEYS = ("boundary_loop_co", "polygon_world", "world_polygon", "boundary_world", "boundary_loop_world",
                   "polygon")
MOUTH_POLY_XZ_KEYS = ("polygon_xz", "polygons_xz", "boundary_xz")

RET_BODY = "GOB_mesh"
RET_CLUB = "GOB_club"
SRC_BODY = "SRC_hi"
SRC_CLUB = "SRC_club_hi"
PART_ATTR = "part_id"
CLUB_PART = "club"

N_SAMPLES = 80_000          # area-weighted samples per union (>= 50k kept after hiding)
SEED = 0
HIDE_DEPTH = 0.001          # m: deeper than this inside another closed shell -> hidden
IFACE_TOL = 0.0001          # m: within this of another closed shell's surface -> interface
TH_DEV = {"mean_mm": 1.0, "p95_mm": 2.0, "max_mm": 3.0}
TH_SIL_MM = 3.0
SIL_RES_H = 2048          # G2.12 silhouette / overlay render height (px)
CLOSEUP_RES_H = 1024      # G2.13 head close-up render height (px)
VIEWS = ("front", "side", "three_quarter")
FRAME_PAD = 0.05            # fraction of the largest bbox extent, added on every side
TOP_N = 20
GAP_EDGE_SPAN = 0.05        # m: club window beyond gap_hi for r_grip_front (as G2.16)
GRIP_EXCL_MARGIN = 0.003    # m: design-change region radius = GOB_club bounding-ring max radius + this
LATHE_RING_TOL = 1e-5       # m: GOB_club verts within this s of a ring's s belong to that ring
SEAM_PIVOTS = ("wrist_l", "wrist_r", "ankle_l", "ankle_r")  # user exception 2026-09-25
SEAM_HALF_LEN = 0.040       # m: seam band |s| <= this along the pivot axis
SEAM_MARGIN = 0.020         # m: seam band axis distance <= pivot radius + this
SEAM_TOP_N = 10
BALL_HANDS = (("hand_l", "wrist_l"), ("hand_r", "wrist_r"))  # ball fist design change 2026-09-25
BALL_MARGIN = 0.030         # m: (T61, superseded by HAND_AXIS_R; kept for ball_report region_r_mm)
HAND_AXIS_R = 0.200         # m: SRC hand region = s > 0 from wrist pivot AND wrist-axis distance <= this
HAND_DILATE_PX = 2          # px: G2.12 hand mask dilation (raster edge safety)
# fixed, non axis-aligned ray directions for the inside (parity) test
RAY_DIRS = [Vector(v).normalized() for v in
            ((0.2673, 0.5345, 0.8018), (-0.7071, 0.1414, 0.6928), (0.3015, -0.9045, -0.3015))]
RAY_STEP = 1e-6
RAY_MAX_HITS = 512

OVERLAY_BG = (1.0, 1.0, 1.0)
OVERLAY_BOTH = (0.55, 0.55, 0.55)
OVERLAY_SRC = (0.90, 0.10, 0.10)
OVERLAY_RET = (0.10, 0.30, 0.95)
CLAY = (0.80, 0.78, 0.74, 1.0)


# ---------------------------------------------------------------- helpers
def rel(p):
    return goblib._rel(p)


def mm(x):
    return None if x is None else round(float(x) * 1000.0, 4)


class Union:
    """World-space triangle soup of several meshes with per-triangle labels and shells.

    v (V,3) float64, t (T,3) int64, tri_label (T,) int -> labels[], tri_shell (T,) int,
    shell_closed (S,) bool (every edge of the shell used by exactly two triangles).
    """

    def __init__(self, name):
        self.name = name
        self.labels = []
        self._v, self._t, self._lab = [], [], []
        self._nv = 0

    def label_id(self, label):
        if label not in self.labels:
            self.labels.append(label)
        return self.labels.index(label)

    def add(self, verts, tris, tri_labels):
        self._v.append(np.asarray(verts, dtype=np.float64))
        self._t.append(np.asarray(tris, dtype=np.int64) + self._nv)
        self._lab.append(np.asarray(tri_labels, dtype=np.int64))
        self._nv += len(verts)

    def finish(self):
        self.v = np.concatenate(self._v) if self._v else np.zeros((0, 3))
        self.t = np.concatenate(self._t) if self._t else np.zeros((0, 3), np.int64)
        self.tri_label = np.concatenate(self._lab) if self._lab else np.zeros(0, np.int64)
        del self._v, self._t, self._lab
        vlab = components(len(self.v), self.t)
        _, self.tri_shell = np.unique(vlab[self.t[:, 0]], return_inverse=True)
        self.tri_shell = self.tri_shell.reshape(-1)
        self.n_shells = int(self.tri_shell.max()) + 1 if len(self.t) else 0
        # closedness: undirected edge use count must be exactly 2
        e = np.sort(self.t[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
        key = e[:, 0] * max(1, len(self.v)) + e[:, 1]
        uk, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
        bad_edge = cnt[inv.reshape(-1)] != 2
        bad_tri = bad_edge.reshape(-1, 3).any(axis=1)
        self.shell_closed = np.ones(self.n_shells, dtype=bool)
        self.shell_closed[np.unique(self.tri_shell[bad_tri])] = False
        a, b, c = self.v[self.t[:, 0]], self.v[self.t[:, 1]], self.v[self.t[:, 2]]
        self.tri_area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        self._bvh = None
        self._shell_bvh = {}
        return self

    def bbox(self):
        return self.v.min(axis=0), self.v.max(axis=0)

    def bvh(self):
        if self._bvh is None:
            self._bvh = BVHTree.FromPolygons(self.v.tolist(), self.t.tolist(), epsilon=0.0)
        return self._bvh

    def shell_bvh(self, s):
        """(bvh, bbox_min, bbox_max) of shell s (local triangle order unused by callers)."""
        if s not in self._shell_bvh:
            tt = self.t[self.tri_shell == s]
            used, loc = np.unique(tt.ravel(), return_inverse=True)
            vv = self.v[used]
            bvh = BVHTree.FromPolygons(vv.tolist(), loc.reshape(-1, 3).tolist(), epsilon=0.0)
            self._shell_bvh[s] = (bvh, vv.min(axis=0), vv.max(axis=0))
        return self._shell_bvh[s]

    def sample(self, n, rng):
        """n area-weighted uniform surface samples -> (points (n,3), tri index (n,))."""
        p = self.tri_area / self.tri_area.sum()
        ti = rng.choice(len(self.t), size=n, p=p)
        r1 = np.sqrt(rng.random(n))
        r2 = rng.random(n)
        a, b, c = self.v[self.t[ti, 0]], self.v[self.t[ti, 1]], self.v[self.t[ti, 2]]
        pts = (1 - r1)[:, None] * a + (r1 * (1 - r2))[:, None] * b + (r1 * r2)[:, None] * c
        return pts, ti


def components(nv, tris):
    """Connected-component label per vertex (min vertex index of the component)."""
    lab = np.arange(nv, dtype=np.int64)
    if len(tris) == 0:
        return lab
    e = tris[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2)
    while True:
        la, lb = lab[e[:, 0]], lab[e[:, 1]]
        if np.array_equal(la, lb):
            return lab
        m = np.minimum(la, lb)
        np.minimum.at(lab, la, m)
        np.minimum.at(lab, lb, m)
        while True:  # pointer jumping
            nxt = lab[lab]
            if np.array_equal(nxt, lab):
                break
            lab = nxt


def mesh_tris(me, mw):
    """World verts (V,3), triangles (T,3), polygon index per triangle (T,)."""
    me.calc_loop_triangles()
    nv, nt = len(me.vertices), len(me.loop_triangles)
    co = np.empty(nv * 3, dtype=np.float64)
    me.vertices.foreach_get("co", co)
    co = co.reshape(nv, 3)
    m = np.array(mw, dtype=np.float64)
    v = co @ m[:3, :3].T + m[:3, 3]
    t = np.empty(nt * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", t)
    pi = np.empty(nt, dtype=np.int64)
    me.loop_triangles.foreach_get("polygon_index", pi)
    return v, t.reshape(nt, 3), pi


def read_eval_mesh(obj, want_part=False):
    """(verts, tris, part_id per tri or None, error string) of the evaluated object."""
    dg = bpy.context.evaluated_depsgraph_get()
    oe = obj.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        v, t, pi = mesh_tris(me, oe.matrix_world)
        part = None
        err = ""
        if want_part:
            at = me.attributes.get(PART_ATTR)
            if at is None:
                err = f"{obj.name}: face attribute '{PART_ATTR}' not found"
            elif at.domain != "FACE" or at.data_type != "INT":
                err = f"{obj.name}: '{PART_ATTR}' is {at.domain}/{at.data_type}, expected FACE/INT"
            else:
                pv = np.empty(len(me.polygons), dtype=np.int64)
                at.data.foreach_get("value", pv)
                part = pv[pi]
    finally:
        oe.to_mesh_clear()
    return v, t, part, err


def show_basis(obj):
    """HR2: evaluate obj in its Basis shape (show_only_shape_key + reference key active; in memory only)."""
    key = obj.data.shape_keys if obj.type == "MESH" else None
    if key is None:
        return {"shape_keys": None}
    names = [kb.name for kb in key.key_blocks]
    ref = key.reference_key
    info = {"shape_keys": names, "values_in_file": {kb.name: round(float(kb.value), 6) for kb in key.key_blocks},
            "reference_key": ref.name if ref is not None else None,
            "method": "object.show_only_shape_key True + active_shape_key_index = reference key (in memory)"}
    obj.show_only_shape_key = True
    obj.active_shape_key_index = names.index(ref.name) if ref is not None else 0
    obj.update_tag()
    bpy.context.view_layer.update()
    return info


def basis_world(obj):
    """World coordinates (V,3) of the reference key (obj.data vertex co when there are no shape keys)."""
    me = obj.data
    nv = len(me.vertices)
    co = np.empty(nv * 3, dtype=np.float64)
    key = me.shape_keys
    if key is not None and key.reference_key is not None:
        key.reference_key.data.foreach_get("co", co)
    else:
        me.vertices.foreach_get("co", co)
    m = np.array(obj.matrix_world, dtype=np.float64)
    return co.reshape(nv, 3) @ m[:3, :3].T + m[:3, 3]


def _ints(x):
    return [int(i) for i in x] if isinstance(x, list) and all(isinstance(i, int) for i in x) else []


def in_poly_xz(pts, poly):
    """Even-odd test of the (x, z) of pts (n,3) against the closed 2D polygon poly (k,2)."""
    pts = np.asarray(pts, dtype=np.float64)
    x, z = pts[:, 0], pts[:, 2]
    ins = np.zeros(len(pts), dtype=bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        cond = (a[1] > z) != (b[1] > z)
        den = b[1] - a[1]
        den = den if den != 0.0 else 1e-300
        ins ^= cond & (x < a[0] + (z - a[1]) * (b[0] - a[0]) / den)
    return ins


def mouth_region(obj, v, t, blend_sha):
    """HR2 mouth design-change region from mouth.json 'region' (see module doc).
    v, t = evaluated GOB_mesh world verts / triangles (Basis).  Returns (region dict, error string)."""
    if not MOUTH_JSON.exists():
        return None, f"missing input: {rel(MOUTH_JSON)}"
    try:
        with open(MOUTH_JSON, "r", encoding="utf-8") as f:
            mj = json.load(f)
    except (OSError, ValueError) as e:
        return None, f"{rel(MOUTH_JSON)} unreadable: {e}"
    reg = mj.get("region") if isinstance(mj, dict) else None
    if not isinstance(reg, dict):
        return None, f"{rel(MOUTH_JSON)} has no 'region' object"
    out = mj.get("output") if isinstance(mj.get("output"), dict) else {}
    osha = out.get("sha256")
    if osha is not None and osha != blend_sha:
        return None, (f"{rel(MOUTH_JSON)} output {out.get('blend')!r} sha256 {str(osha)[:12]} != {rel(BLEND)} sha256 "
                      f"{blend_sha[:12]} (mouth.json was not written for the checked blend; indices unusable)")
    nv = len(v)
    vr = set(_ints(reg.get("verts"))) | set(_ints(reg.get("output_verts")))
    nvj = mj.get("new_vertices") if isinstance(mj.get("new_vertices"), dict) else {}
    ar = _ints(nvj.get("appended_range"))
    if len(ar) == 2:
        vr |= set(range(ar[0], ar[1] + 1))
    for g in (nvj.get("groups") or {}).values():
        vr |= set(_ints(g))
    bl = _ints(reg.get("boundary_loop"))
    if not vr:
        return None, f"{rel(MOUTH_JSON)} region has no vertex list (verts / new_vertices)"
    bad = [i for i in list(vr) + bl if i < 0 or i >= nv]
    if bad:
        return None, f"{rel(MOUTH_JSON)} vertex indices out of range for {RET_BODY} ({nv} verts): {sorted(bad)[:5]}"
    me = obj.data
    me.calc_loop_triangles()
    if len(me.loop_triangles) != len(t):
        return None, f"{RET_BODY} evaluated triangles {len(t)} != obj.data triangles {len(me.loop_triangles)}"
    tp = np.empty(len(t), dtype=np.int64)
    me.loop_triangles.foreach_get("polygon_index", tp)
    mi = np.empty(len(me.polygons), dtype=np.int64)
    me.polygons.foreach_get("material_index", mi)
    in_v = np.zeros(nv, dtype=bool)
    in_v[sorted(vr)] = True
    in_vb = in_v.copy()
    in_vb[bl] = True
    by_verts = in_vb[t].all(axis=1) & in_v[t].any(axis=1)
    by_mat = mi[tp] >= 1
    tri = by_verts | by_mat
    fl = _ints(reg.get("faces")) or _ints(reg.get("face_indices"))
    of = reg.get("output_faces") if isinstance(reg.get("output_faces"), dict) else {}
    fl = fl + _ints(of.get("all_new"))
    for g in (of.get("by_group") or {}).values():
        fl = fl + _ints(g)
    by_faces = np.isin(tp, fl) if fl else np.zeros(len(t), dtype=bool)
    tri |= by_faces
    poly, src = None, None
    for k in MOUTH_POLY_KEYS:
        p = reg.get(k)
        try:
            a = np.array(p, dtype=np.float64)
        except (TypeError, ValueError):
            continue
        if a.ndim == 2 and a.shape[1] == 3 and len(a) >= 3 and np.all(np.isfinite(a)):
            poly, src = a[:, [0, 2]], f"mouth.json region.{k} (world x, z)"
            break
    if poly is None:
        for k in MOUTH_POLY_XZ_KEYS:
            p = reg.get(k)
            try:
                a = np.array(p, dtype=np.float64)
            except (TypeError, ValueError):
                continue
            if a.ndim == 3 and a.shape[0] >= 1:
                a = a[0]
            if a.ndim == 2 and a.shape[1] == 2 and len(a) >= 3 and np.all(np.isfinite(a)):
                poly, src = a, f"mouth.json region.{k}"
                break
    if poly is None:
        if len(bl) < 3:
            return None, f"{rel(MOUTH_JSON)} region has no polygon and no boundary_loop (>= 3 verts)"
        poly, src = basis_world(obj)[bl][:, [0, 2]], "boundary_loop vertices, Basis world (x, z)"
    yr = reg.get("y_range")
    try:
        yr = [float(yr[0]), float(yr[1])]
        ysrc = "mouth.json region.y_range"
    except (TypeError, ValueError, IndexError, KeyError):
        used = np.unique(np.concatenate([t[tri].ravel(), np.nonzero(in_vb)[0]]))
        ys = v[used, 1]
        yr = [float(ys.min()) - MOUTH_Y_MARGIN, float(ys.max()) + MOUTH_Y_MARGIN]
        ysrc = f"mouth vertices (region tris + vertex lists, Basis) y range +- {MOUTH_Y_MARGIN * 1000:g} mm"
    info = {"source": rel(MOUTH_JSON), "output_blend": out.get("blend"), "output_sha256_matches": osha is not None,
            "n_region_verts": len(vr), "n_boundary_loop": len(bl),
            "tris": int(tri.sum()), "tris_by_verts": int(by_verts.sum()), "tris_by_material_slot_ge_1": int(by_mat.sum()),
            "tris_by_region_faces": int(by_faces.sum()), "polygon_source": src, "polygon_n": int(len(poly)),
            "polygon_xz_bbox_m": [[round(float(x), 5) for x in poly.min(axis=0)],
                                  [round(float(x), 5) for x in poly.max(axis=0)]],
            "y_range_m": [round(yr[0], 5), round(yr[1], 5)], "y_range_source": ysrc}
    return {"tri": tri, "poly": poly, "y": yr, "info": info}, ""


def in_mouth(pts, mr):
    pts = np.asarray(pts, dtype=np.float64)
    return in_poly_xz(pts, mr["poly"]) & (pts[:, 1] >= mr["y"][0]) & (pts[:, 1] <= mr["y"][1])


def load_zones():
    """HR2 head design zones from head_design_zones.json (see module doc) -> ({key: zone}, error string);
    record-only entries are returned under the key '_record_only' (info dict)."""
    if not ZONES_JSON.exists():
        return None, (f"missing input: {rel(ZONES_JSON)} (HR2 design-change zone eye_gap_xneg, written by s02c; "
                      "G2.11 / G2.12 cannot be judged without it)")
    try:
        with open(ZONES_JSON, "r", encoding="utf-8") as f:
            zj = json.load(f)
    except (OSError, ValueError) as e:
        return None, f"{rel(ZONES_JSON)} unreadable: {e}"
    if not isinstance(zj, dict):
        return None, f"{rel(ZONES_JSON)} is not an object"
    for k in ZONE_KEYS:
        if not isinstance(zj.get(k), dict):
            return None, f"{rel(ZONES_JSON)} has no '{k}' object"
    out, rec = {}, {}
    for k, z in zj.items():
        if not isinstance(z, dict):
            continue
        use = str(z.get("use", "exclude (no 'use' field; required zone)" if k in ZONE_KEYS else "")).strip()
        if not use.lower().startswith("exclude"):
            rec[k] = {"use": use, "applied": False}
            continue
        dr = z.get("depth_rule")
        depth = None
        if dr is not None:
            try:
                ax = np.array([float(x) for x in dr["axis"]], dtype=np.float64)
                ap = np.array([float(x) for x in dr["apex"]], dtype=np.float64)
                dmin = float(dr["exclude_depth_min_m"])
            except (TypeError, ValueError, KeyError):
                return None, f"{rel(ZONES_JSON)} {k}.depth_rule needs axis [3], apex [3], exclude_depth_min_m"
            if ax.shape != (3,) or ap.shape != (3,) or np.linalg.norm(ax) == 0:
                return None, f"{rel(ZONES_JSON)} {k}.depth_rule axis / apex invalid"
            depth = {"axis": ax / np.linalg.norm(ax), "apex": ap, "dmin": dmin}
        polys = []
        for p in z.get("polygons_xz") or []:
            try:
                a = np.array(p, dtype=np.float64)
            except (TypeError, ValueError):
                return None, f"{rel(ZONES_JSON)} {k}.polygons_xz entry unparseable"
            if a.ndim != 2 or a.shape[1] != 2 or len(a) < 3 or not np.all(np.isfinite(a)):
                return None, f"{rel(ZONES_JSON)} {k}.polygons_xz entry is not a list of >= 3 [x, z]"
            polys.append(a)
        yr = z.get("y_range")
        if polys:
            try:
                yr = [float(yr[0]), float(yr[1])]
            except (TypeError, ValueError, IndexError, KeyError):
                return None, f"{rel(ZONES_JSON)} {k} has polygons but no valid y_range"
        out[k] = {"polys": polys, "y": yr if polys else None, "depth": depth,
                  "info": {"source": rel(ZONES_JSON), "use": use, "n_polygons": len(polys),
                           "y_range_m": yr if polys else None, "cells": z.get("cells"),
                           "max_gap_mm": z.get("max_gap_mm"), "empty_zone": not polys,
                           "depth_rule": None if depth is None else {
                               "axis": [round(float(x), 6) for x in depth["axis"]],
                               "apex": [round(float(x), 6) for x in depth["apex"]],
                               "exclude_depth_min_mm": mm(depth["dmin"]), "ray_start_m": ZONE_RAY_START}}}
    out["_record_only"] = rec
    return out, ""


def in_zone(pts, zone, bvh=None):
    """Bool per point: inside the zone polygons / y range and (depth rule) deeper than exclude_depth_min
    below the local SRC_hi front (bvh = SRC_hi BVH; no hit -> not excluded)."""
    pts = np.asarray(pts, dtype=np.float64)
    m = np.zeros(len(pts), dtype=bool)
    if not zone["polys"]:
        return m
    for p in zone["polys"]:
        m |= in_poly_xz(pts, p)
    m &= (pts[:, 1] >= zone["y"][0]) & (pts[:, 1] <= zone["y"][1])
    dr = zone["depth"]
    if dr is None or not m.any():
        return m
    a, c, dmin = dr["axis"], dr["apex"], dr["dmin"]
    d_neg = Vector(-a)
    for i in np.nonzero(m)[0].tolist():
        p = pts[i]
        dp = float((p - c) @ a)
        o = p - dp * a + ZONE_RAY_START * a
        loc = bvh.ray_cast(Vector(o), d_neg)[0] if bvh is not None else None
        m[i] = loc is not None and (float((np.array(loc) - c) @ a) - dp) > dmin
    return m


def design_tris(ret_u, src_u, design):
    """{region: (world tris (N,3,3), counts)} of the HR2 design regions for the G2.12 masks: RETOPO tris
    in the region (mouth: region tris or head tris with centroid inside; zone: head tris with centroid
    inside) and SRC_hi tris whose centroid is inside."""
    out = {}
    if not design:
        return out
    r_cen = ret_u.v[ret_u.t].mean(axis=1)
    s_cen = src_u.v[src_u.t].mean(axis=1)
    r_head = (ret_u.tri_label == ret_u.labels.index("head")) if "head" in ret_u.labels else         np.zeros(len(ret_u.t), dtype=bool)
    s_body = (src_u.tri_label == src_u.labels.index(SRC_BODY)) if SRC_BODY in src_u.labels else         np.zeros(len(src_u.t), dtype=bool)
    mr = design.get("mouth")
    if mr is not None:
        sr = design["ret_mouth_tri"] | (r_head & in_mouth(r_cen, mr))
        ss = s_body & in_mouth(s_cen, mr)
        out["mouth"] = (np.concatenate([ret_u.v[ret_u.t[sr]], src_u.v[src_u.t[ss]]]),
                        {"retopo_tris": int(sr.sum()), "src_tris": int(ss.sum())})
    for k, z in zone_items(design):
        sr = r_head & in_zone(r_cen, z, design.get("src_hi_bvh"))
        ss = s_body & in_zone(s_cen, z, design.get("src_hi_bvh"))
        out[k] = (np.concatenate([ret_u.v[ret_u.t[sr]], src_u.v[src_u.t[ss]]]),
                  {"retopo_tris": int(sr.sum()), "src_tris": int(ss.sum())})
    return out


def zone_items(design):
    return [(k, z) for k, z in (design.get("zones") or {}).items() if k != "_record_only"]


def inside_shell(bvh, p):
    """Ray parity inside test, majority vote over RAY_DIRS."""
    votes = 0
    for d in RAY_DIRS:
        o = p.copy()
        hits = 0
        last = -1
        for _ in range(RAY_MAX_HITS):
            loc, _n, idx, _dist = bvh.ray_cast(o, d)
            if loc is None:
                break
            if idx != last:
                hits += 1
            last = idx
            o = loc + d * RAY_STEP
        votes += hits & 1
    return votes * 2 > len(RAY_DIRS)


def occlusion(u, pts, pt_shell):
    """hidden: point is > HIDE_DEPTH inside another closed shell of the same union.
    interface: not hidden and within IFACE_TOL of another closed shell's surface."""
    n = len(pts)
    hidden = np.zeros(n, dtype=bool)
    iface = np.zeros(n, dtype=bool)
    for s in range(u.n_shells):
        if not u.shell_closed[s]:
            continue
        bvh, mn, mx = u.shell_bvh(s)
        inb = np.all((pts >= mn - IFACE_TOL) & (pts <= mx + IFACE_TOL), axis=1)
        cand = np.nonzero(inb & (pt_shell != s) & ~hidden)[0]
        for i in cand:
            p = Vector(pts[i])
            loc, _n, _idx, d = bvh.find_nearest(p)
            if loc is None:
                continue
            if d <= IFACE_TOL:
                iface[i] = True
            elif d > HIDE_DEPTH and inside_shell(bvh, p):
                hidden[i] = True
    iface &= ~hidden
    return hidden, iface


def nearest(u, pts):
    """Distance (m), nearest location, triangle index of pts on union u."""
    bvh = u.bvh()
    n = len(pts)
    dist = np.full(n, np.inf)
    loc = np.zeros((n, 3))
    tri = np.full(n, -1, dtype=np.int64)
    for i in range(n):
        l, _nrm, idx, d = bvh.find_nearest(Vector(pts[i]))
        if l is not None:
            dist[i], loc[i], tri[i] = d, l, idx
    return dist, loc, tri


def stats(d):
    d = np.asarray(d, dtype=np.float64)
    if len(d) == 0:
        return {"n": 0, "mean_mm": None, "p95_mm": None, "max_mm": None}
    return {"n": int(len(d)), "mean_mm": mm(d.mean()), "p95_mm": mm(np.percentile(d, 95)),
            "max_mm": mm(d.max())}


def dev_ok(st):
    return (st["n"] > 0 and st["mean_mm"] <= TH_DEV["mean_mm"]
            and st["p95_mm"] <= TH_DEV["p95_mm"] and st["max_mm"] <= TH_DEV["max_mm"])


def one_direction(src_u, dst_u, pts, pt_tri, pt_labels_own, excl, band=None, sub=None):
    """Deviation of samples of src_u (pts) to surface of dst_u.

    Judged points = not hidden, not interface and not in the design-change region
    (excl, bool per point; spec G2.11); stats, top and the per-part table use only
    them.  pt_labels_own: part name per point, or None -> label by nearest dst triangle.
    band (bool per point, seam band): adds report-only outside_seam / seam_band /
    n_seam_band over the judged points; the judgement itself is unchanged.
    """
    pt_shell = src_u.tri_shell[pt_tri]
    hidden, iface = occlusion(src_u, pts, pt_shell)
    dist, loc, tri = nearest(dst_u, pts)
    if pt_labels_own is None:
        labels = np.array([dst_u.labels[k] for k in dst_u.tri_label[tri]], dtype=object)
    else:
        labels = pt_labels_own
    keep = ~hidden
    design = excl & keep & ~iface
    judged = keep & ~iface & ~design
    st = stats(dist[judged])
    st_i = stats(dist[keep & ~design])
    ji = np.nonzero(judged)[0]
    order = ji[np.argsort(-dist[ji], kind="stable")][:TOP_N]
    top = [{"co_m": [round(float(x), 5) for x in pts[i]], "dev_mm": mm(dist[i]),
            "part": labels[i], "nearest_m": [round(float(x), 5) for x in loc[i]]}
           for i in order]
    measured = dict(st)
    measured.update({"n_samples": int(len(pts)), "n_hidden": int(hidden.sum()),
                     "n_interface": int(iface.sum()), "n_design_excluded": int(design.sum()),
                     "incl_interface": st_i, "top": top})
    if sub is not None:  # design-region sub-masks (report): excluded count per region
        measured["n_design_excluded_by"] = {k: int((m & design).sum()) for k, m in sub.items()}
        # report only: deviation stats inside each design region (non-hidden, non-interface points)
        measured["design_region_stats_report"] = {k: stats(dist[m & keep & ~iface]) for k, m in sub.items()}
    if band is not None:
        out = judged & ~band
        oi = np.nonzero(out)[0]
        o_order = oi[np.argsort(-dist[oi], kind="stable")][:SEAM_TOP_N]
        o_st = stats(dist[out])
        o_st["top"] = [{"co_m": [round(float(x), 5) for x in pts[i]], "dev_mm": mm(dist[i]),
                        "part": labels[i]} for i in o_order]
        measured["outside_seam"] = o_st
        measured["seam_band"] = stats(dist[judged & band])
        measured["n_seam_band"] = int((judged & band).sum())
    per_part = {}
    for lab in sorted(set(labels.tolist())):
        sel = labels == lab
        row = stats(dist[sel & judged])
        row["n_hidden"] = int((sel & hidden).sum())
        row["n_interface"] = int((sel & iface).sum())
        row["n_design_excluded"] = int((sel & design).sum())
        per_part[lab] = row
    return measured, per_part


# ---------------------------------------------------------------- data loading
def load_src_meshes(tmp):
    """Append SRC_hi / SRC_club_hi meshes; returns {name: mesh}. Registers new IDs in tmp."""
    mats_before = {m.as_pointer() for m in bpy.data.materials}
    imgs_before = {i.as_pointer() for i in bpy.data.images}
    with bpy.data.libraries.load(str(SRC_BLEND), link=False) as (df, dt):
        names = [n for n in (SRC_BODY, SRC_CLUB) if n in df.meshes]
        dt.meshes = list(names)
    out = {}
    for n, m in zip(names, dt.meshes):
        if m is not None:
            out[n] = m
            tmp["meshes"].append(m)
    tmp["materials"] += [m for m in bpy.data.materials if m.as_pointer() not in mats_before]
    tmp["images"] += [i for i in bpy.data.images if i.as_pointer() not in imgs_before]
    return out


def cleanup(tmp):
    for o in tmp["objects"]:
        try:
            bpy.data.objects.remove(o, do_unlink=True)
        except ReferenceError:
            pass
    for me in tmp["meshes"]:
        try:
            bpy.data.meshes.remove(me)
        except ReferenceError:
            pass
    for m in tmp["materials"]:
        try:
            bpy.data.materials.remove(m)
        except ReferenceError:
            pass
    for i in tmp["images"]:
        try:
            bpy.data.images.remove(i)
        except ReferenceError:
            pass


# ---------------------------------------------------------------- images
def read_png_rgba(png):
    """RGBA float array (H, W, 4), row 0 = image top."""
    img = bpy.data.images.load(str(Path(png).resolve()), check_existing=False)
    try:
        w, h = img.size
        buf = np.empty(w * h * img.channels, dtype=np.float32)
        img.pixels.foreach_get(buf)
        px = buf.reshape(h, w, img.channels)
        if img.channels == 3:
            px = np.concatenate([px, np.ones((h, w, 1), np.float32)], axis=2)
    finally:
        bpy.data.images.remove(img)
    return np.ascontiguousarray(px[::-1])


def write_png(path, rgba):
    """Write RGBA float array (H, W, 4), row 0 = image top, as 8-bit PNG."""
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w = rgba.shape[:2]
    img = bpy.data.images.new("_g2_png", width=w, height=h, alpha=True)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(rgba[::-1], dtype=np.float32).ravel())
        img.filepath_raw = str(path)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)
    return path


def overlay(mask_src, mask_ret):
    h, w = mask_src.shape
    out = np.empty((h, w, 4), dtype=np.float32)
    out[:, :, :3] = OVERLAY_BG
    out[:, :, 3] = 1.0
    out[mask_src & mask_ret, :3] = OVERLAY_BOTH
    out[mask_src & ~mask_ret, :3] = OVERLAY_SRC
    out[~mask_src & mask_ret, :3] = OVERLAY_RET
    return out


def over_white(rgba):
    a = rgba[:, :, 3:4]
    out = np.empty_like(rgba)
    out[:, :, :3] = rgba[:, :, :3] * a + (1.0 - a)
    out[:, :, 3] = 1.0
    return out


def padded_bbox(mn, mx, pad=FRAME_PAD):
    mn, mx = np.asarray(mn, float), np.asarray(mx, float)
    m = pad * float((mx - mn).max())
    return (tuple(mn - m), tuple(mx + m))


def frame_size(frame_bbox, view):
    """(width, height) in m of the ortho frame goblib.ortho_render uses for frame_bbox."""
    d, right, up = goblib._view_basis(view)
    mn, mx = frame_bbox
    cs = [Vector((x, y, z)) for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
    us = [c.dot(right) for c in cs]
    vs = [c.dot(up) for c in cs]
    return max(us) - min(us), max(vs) - min(vs)


def grip_exclusion(club_v, pivots, lathe_v=None):
    """Design-change region around grip_r (spec G2.11, same gap definition as G2.16).

    s = SRC_club_hi vertex axial coordinate from grip_r along its axis; gap = interval
    between adjacent sorted s values containing 0; r_grip_front = median axis distance of
    club verts with s in [gap_hi, gap_hi + 50 mm] (report); region = axis distance <= r_lathe_max
    + 3 mm (r_lathe_max = max axis distance of the GOB_club (lathe_v) ring verts nearest to the gap
    on each side: largest s < gap_lo and smallest s > gap_hi, 2026-09-25 T61e)
    + 3 mm and s in [gap_lo, gap_hi].  Returns (region dict, error string).
    """
    piv = pivots.get("pivots", pivots) if isinstance(pivots, dict) else {}
    g = piv.get("grip_r") if isinstance(piv, dict) else None
    try:
        gc = np.array([float(x) for x in g["co"]], dtype=np.float64)
        ga = np.array([float(x) for x in g["axis"]], dtype=np.float64)
    except (TypeError, KeyError, ValueError):
        return None, "pivots.json grip_r co/axis missing or invalid"
    if (gc.shape != (3,) or ga.shape != (3,) or not np.all(np.isfinite(np.r_[gc, ga]))
            or np.linalg.norm(ga) == 0):
        return None, "pivots.json grip_r co/axis missing or invalid"
    ga = ga / np.linalg.norm(ga)
    dc = np.asarray(club_v, dtype=np.float64) - gc
    sc = dc @ ga
    rc = np.linalg.norm(dc - np.outer(sc, ga), axis=1)
    ss = np.sort(sc)
    below, above = ss[ss < 0.0], ss[ss > 0.0]
    if not len(below) or not len(above) or len(below) + len(above) != len(ss):
        return None, "no empty interval of SRC_club_hi s containing 0 (club not cut at grip_r)"
    gap_lo, gap_hi = float(below[-1]), float(above[0])
    win = (sc >= gap_hi) & (sc <= gap_hi + GAP_EDGE_SPAN)
    if not win.any():
        return None, "gap_hi window contains no SRC_club_hi vertex"
    r_front = float(np.median(rc[win]))
    lv = np.zeros((0, 3)) if lathe_v is None else np.asarray(lathe_v, dtype=np.float64) - gc
    ls = lv @ ga
    lr = np.linalg.norm(lv - np.outer(ls, ga), axis=1)
    lo, hi = ls < gap_lo, ls > gap_hi
    if not lo.any() or not hi.any():
        return None, "no GOB_club ring on one side of [gap_lo, gap_hi] (lathe radius undefined)"
    rings = {}
    for key, s_r in (("lo", float(ls[lo].max())), ("hi", float(ls[hi].min()))):
        m = np.abs(ls - s_r) <= LATHE_RING_TOL
        rings[key] = {"s_mm": mm(s_r), "r_max_mm": mm(lr[m].max()), "n_verts": int(m.sum()), "_r": float(lr[m].max())}
    r_lathe = max(rings["lo"]["_r"], rings["hi"]["_r"])
    for rg in rings.values():
        rg.pop("_r")
    return {"co": gc, "axis": ga, "gap_lo": gap_lo, "gap_hi": gap_hi, "r_front": r_front,
            "r_lathe": r_lathe, "lathe_rings": rings, "n_in_gap": int(((ls >= gap_lo) & (ls <= gap_hi)).sum()),
            "r_excl": r_lathe + GRIP_EXCL_MARGIN, "n_window": int(win.sum())}, ""


def seam_cylinders(pivots):
    """[(name, co, unit axis, band radius)] for SEAM_PIVOTS present in pivots.json."""
    piv = pivots.get("pivots", pivots) if isinstance(pivots, dict) else {}
    out = []
    for name in SEAM_PIVOTS:
        g = piv.get(name) if isinstance(piv, dict) else None
        try:
            c = np.array([float(x) for x in g["co"]], dtype=np.float64)
            a = np.array([float(x) for x in g["axis"]], dtype=np.float64)
            r = float(g["radius"])
        except (TypeError, KeyError, ValueError):
            continue
        if c.shape != (3,) or a.shape != (3,) or np.linalg.norm(a) == 0:
            continue
        out.append((name, c, a / np.linalg.norm(a), r + SEAM_MARGIN))
    return out


def seam_band(pts, pivots):
    """Bool per point: inside any wrist/ankle seam band (spec G2.11 user exception):
    |s| <= SEAM_HALF_LEN along the pivot axis AND axis distance <= pivot radius + SEAM_MARGIN."""
    pts = np.asarray(pts, dtype=np.float64)
    m = np.zeros(len(pts), dtype=bool)
    for _n, c, a, rb in seam_cylinders(pivots):
        d = pts - c
        sa = d @ a
        ra = np.linalg.norm(d - np.outer(sa, a), axis=1)
        m |= (np.abs(sa) <= SEAM_HALF_LEN) & (ra <= rb)
    return m


def hand_balls(v, t, part, parts, pivots):
    """{hand: {c, r, wc, wa, n}} least-squares sphere of the GOB_mesh verts of each hand part
    (ball fist, spec G2.11/G2.12 2026-09-25) + its wrist pivot co / unit axis."""
    piv = pivots.get("pivots", pivots) if isinstance(pivots, dict) else {}
    out = {}
    for hand, wrist in BALL_HANDS:
        pid = parts.get(hand) if isinstance(parts, dict) else None
        w = piv.get(wrist) if isinstance(piv, dict) else None
        if pid is None or not isinstance(w, dict) or part is None:
            continue
        P = v[np.unique(t[part == int(pid)])]
        try:
            wc = np.array([float(x) for x in w["co"]], dtype=np.float64)
            wa = np.array([float(x) for x in w["axis"]], dtype=np.float64)
        except (TypeError, KeyError, ValueError):
            continue
        if len(P) < 4 or wc.shape != (3,) or wa.shape != (3,) or np.linalg.norm(wa) == 0:
            continue
        A = np.column_stack([2 * P, np.ones(len(P))])
        sol = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)[0]
        c = sol[:3]
        r = float(np.sqrt(max(sol[3] + c @ c, 0.0)))
        res = np.linalg.norm(P - c, axis=1) - r
        out[hand] = {"c": c, "r": r, "wc": wc, "wa": wa / np.linalg.norm(wa), "n": int(len(P)),
                     "rms": float(np.sqrt((res ** 2).mean()))}
    return out


def ball_report(balls):
    return {h: {"center_m": [round(float(x), 6) for x in b["c"]], "r_mm": mm(b["r"]),
                "region_r_mm": mm(b["r"] + BALL_MARGIN), "fit_rms_mm": mm(b["rms"]), "n_verts": b["n"]}
            for h, b in (balls or {}).items()}


def src_hand_regions(pts, balls):
    """{hand: bool per point}: (p - wrist.co) . wrist.axis > 0 AND distance from the wrist axis
    line <= HAND_AXIS_R (spec G2.11 2026-09-25 correction: whole original hand)."""
    pts = np.asarray(pts, dtype=np.float64)
    out = {}
    for h, b in (balls or {}).items():
        d = pts - b["wc"]
        sa = d @ b["wa"]
        out[h] = (sa > 0.0) & (np.linalg.norm(d - np.outer(sa, b["wa"]), axis=1) <= HAND_AXIS_R)
    return out


def hand_tris(ret_u, src_u, balls):
    """World triangles (N,3,3) of the G2.12 hand region + counts: RETOPO tris labelled hand_l /
    hand_r (ball shells) and SRC_hi tris whose centroid is in src_hand_regions."""
    tri_r, tri_s, cnt = [], [], {}
    for h, _w in BALL_HANDS:
        n_r = 0
        if h in ret_u.labels:
            sel = ret_u.tri_label == ret_u.labels.index(h)
            tri_r.append(ret_u.v[ret_u.t[sel]])
            n_r = int(sel.sum())
        n_s = 0
        if h in (balls or {}) and SRC_BODY in src_u.labels:
            body = src_u.tri_label == src_u.labels.index(SRC_BODY)
            cen = src_u.v[src_u.t].mean(axis=1)
            sel = body & src_hand_regions(cen, {h: balls[h]})[h]
            tri_s.append(src_u.v[src_u.t[sel]])
            n_s = int(sel.sum())
        cnt[h] = {"retopo_tris": n_r, "src_tris": n_s}
    allt = tri_r + tri_s
    return (np.concatenate(allt) if allt else np.zeros((0, 3, 3))), cnt


def hand_mask_2d(tris, view, frame_bbox, shape):
    """Bool image (row 0 = top): pixel centres covered by the projected hand triangles, dilated
    by HAND_DILATE_PX (square)."""
    _d, right, up, uc, vc, _wc, ps, _rw = frame_geom(frame_bbox, view, shape[0])
    hh, ww = shape
    mask = np.zeros(shape, dtype=bool)
    if not len(tris):
        return mask
    X = (tris @ right - uc) / ps + ww / 2.0
    Y = hh / 2.0 - (tris @ up - vc) / ps
    for x, y in zip(X, Y):
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(ww, np.ceil(x.max()) + 1))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(hh, np.ceil(y.max()) + 1))
        if x0 >= x1 or y0 >= y1:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        e = [(x[(k + 1) % 3] - x[k]) * (gy - y[k]) - (y[(k + 1) % 3] - y[k]) * (gx - x[k]) for k in range(3)]
        mask[y0:y1, x0:x1] |= ((e[0] >= 0) & (e[1] >= 0) & (e[2] >= 0)) | ((e[0] <= 0) & (e[1] <= 0) & (e[2] <= 0))
    k = HAND_DILATE_PX
    dil = mask.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            dil[max(0, dy):hh + min(0, dy), max(0, dx):ww + min(0, dx)] |=                 mask[max(0, -dy):hh + min(0, -dy), max(0, -dx):ww + min(0, -dx)]
    return dil


def frame_geom(frame_bbox, view, res_h):
    """(d, right, up, uc, vc, wc, px_size, res_w) of the goblib.ortho_render frame."""
    d, right, up = goblib._view_basis(view)
    mn, mx = frame_bbox
    cs = [Vector((x, y, z)) for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]
    us = [c.dot(right) for c in cs]
    vs = [c.dot(up) for c in cs]
    ws = [c.dot(d) for c in cs]
    w, h = max(us) - min(us), max(vs) - min(vs)
    res_w = max(1, int(round(res_h * w / h)))
    return (np.array(d), np.array(right), np.array(up), (max(us) + min(us)) / 2,
            (max(vs) + min(vs)) / 2, (max(ws) + min(ws)) / 2, h / res_h, res_w)


def seam_mask_2d(pivots, view, frame_bbox, shape):
    """Bool image (row 0 = top): pixel centres inside the projection of any seam-band
    cylinder (convex hull of both projected end circles) in the given ortho frame."""
    d, right, up, uc, vc, _wc, ps, _rw = frame_geom(frame_bbox, view, shape[0])
    hh, ww = shape
    mask = np.zeros(shape, dtype=bool)
    ang = np.linspace(0.0, 2 * np.pi, 72, endpoint=False)
    for _n, c, a, rb in seam_cylinders(pivots):
        e1 = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(a, e1)
        ring = rb * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
        P = np.concatenate([c + a * SEAM_HALF_LEN + ring, c - a * SEAM_HALF_LEN + ring])
        # world -> pixel (x = column, y = row from top), continuous coordinates
        px = (P @ right - uc) / ps + ww / 2.0
        py = hh / 2.0 - (P @ up - vc) / ps
        pts = sorted(set(zip(px.tolist(), py.tolist())))
        # monotone-chain convex hull (counter-clockwise in x/y)

        def cross(o, p1, p2):
            return (p1[0] - o[0]) * (p2[1] - o[1]) - (p1[1] - o[1]) * (p2[0] - o[0])
        lower, upper = [], []
        for q in pts:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
                lower.pop()
            lower.append(q)
        for q in reversed(pts):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
                upper.pop()
            upper.append(q)
        hull = np.array(lower[:-1] + upper[:-1])
        x0, x1 = int(max(0, np.floor(hull[:, 0].min()))), int(min(ww, np.ceil(hull[:, 0].max()) + 1))
        y0, y1 = int(max(0, np.floor(hull[:, 1].min()))), int(min(hh, np.ceil(hull[:, 1].max()) + 1))
        if x0 >= x1 or y0 >= y1:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        inside = np.ones(gx.shape, dtype=bool)
        for k in range(len(hull)):
            o, q = hull[k], hull[(k + 1) % len(hull)]
            inside &= ((q[0] - o[0]) * (gy - o[1]) - (q[1] - o[1]) * (gx - o[0])) >= 0
        mask[y0:y1, x0:x1] |= inside
    return mask


# ---------------------------------------------------------------- criteria
def crit_g2_11(ev, ret_u, src_u, part_names, grip, why, pivots=None, balls=None, design=None):
    """G2.11a/b/c two-way surface deviation, hidden, interface and design-change points
    excluded (grip = grip_exclusion region, balls = hand_balls ball-fist regions,
    design = HR2 mouth region + head zones, see module doc)."""
    th_c = "report-only"
    note_m = (f"area-weighted samples n={N_SAMPLES} per union (numpy default_rng seed {SEED}); "
              f"hidden = > {HIDE_DEPTH * 1000:g} mm inside another closed shell of the same union "
              f"(shell = connected component, closed = every edge used by 2 tris; inside = ray "
              f"parity majority of {len(RAY_DIRS)} fixed directions on the shell BVH; depth = "
              f"BVH find_nearest distance); interface = non-hidden point within "
              f"{IFACE_TOL * 1000:g} mm of another closed shell surface of the same union (e.g. "
              f"coincident cut caps); design-change region (fist shaft stub and empty fist hole "
              f"wall) = axis distance from pivots grip_r axis <= r_lathe_max + "
              f"{GRIP_EXCL_MARGIN * 1000:g} mm (= r_excl; r_lathe_max = max axis distance of the GOB_club "
              f"ring verts nearest to the gap on each side (largest s < gap_lo, smallest s > gap_hi, "
              f"ring = verts within {LATHE_RING_TOL * 1000:g} mm of that s; 'lathe_rings' lo/hi = s, "
              f"max radius, count), n_lathe_verts_in_gap = GOB_club verts inside the gap; "
              f"2026-09-25 T61e) and "
              f"axial s in [gap_lo, gap_hi], both directions, gap = interval of "
              f"sorted SRC_club_hi s containing 0; r_grip_front (report only) = median club axis "
              f"distance with s in [gap_hi, gap_hi + {GAP_EDGE_SPAN * 1000:g} mm]; hidden, interface "
              f"and design-region points are excluded from the judgement and counted in n_hidden / "
              f"n_interface / n_design_excluded (disjoint, in that priority); n, mean, p95, max, ok "
              f"and top use the judged points only; 'incl_interface' = stats with interface points "
              f"included (hidden and design-region points still excluded, report only); distance = "
              f"BVH find_nearest to the other union. top = {TOP_N} largest judged deviations "
              f"(co_m world m). Seam band (spec G2.11 user exception 2026-09-25): ok and the "
              f"top-level stats cover all judged points; main judges by ok_outside_seam per the "
              f"spec user exception (main 판정은 스펙 사용자 예외에 따라 ok_outside_seam 기준); "
              f"ok_outside_seam = 'outside_seam' stats meet mean <= {TH_DEV['mean_mm']:g}, p95 <= "
              f"{TH_DEV['p95_mm']:g}, max <= {TH_DEV['max_mm']:g} mm. seam band = any of {', '.join(SEAM_PIVOTS)} pivots with axial |s| <= "
              f"{SEAM_HALF_LEN * 1000:g} mm AND axis distance <= pivot radius + "
              f"{SEAM_MARGIN * 1000:g} mm (pivots.json co/axis/radius); 'outside_seam' = stats and "
              f"top {SEAM_TOP_N} of judged points outside the band; 'seam_band' = stats of judged "
              f"points inside the band; n_seam_band = their count; 'seam_cylinders' = band "
              f"definition used. Ball fist design change (spec G2.11, 2026-09-25): per hand, ball = "
              f"least-squares sphere of the GOB_mesh verts of that hand part; RETOPO->SRC: every "
              f"sample on a hand_l / hand_r shell (own part_id) is added to the design-change "
              f"exclusion; SRC->RETOPO (2026-09-25 correction, whole original hand): SRC_hi samples "
              f"with (p - wrist.co) . wrist.axis > 0 (fingertip side, pivots.json wrist_l/wrist_r) "
              f"AND distance from the wrist axis line <= {HAND_AXIS_R * 1000:g} mm are added "
              f"(SRC_club_hi samples not); n_design_excluded_by = excluded count per "
              f"region (grip / hand_l / hand_r / mouth / {' / '.join(ZONE_KEYS)}, a point may be in several); "
              f"'balls' = fits used. HR2 (2026-09-26, hr2_contract.md): GOB_mesh in its Basis shape "
              f"(closed mouth); mouth region (mouth.json) and {', '.join(ZONE_KEYS)} "
              f"(head_design_zones.json) are design changes excluded in both directions (RETOPO: mouth "
              f"region tris or head samples inside the region; SRC: SRC_hi samples inside), see the module "
              f"doc; 'design_regions' = definitions used; design_region_stats_report = deviation stats of the "
              f"non-hidden, non-interface points inside each region (report only)")
    if why:
        for cid in ("G2.11a", "G2.11b"):
            ev.criterion(cid, None, TH_DEV, False, why)
        ev.criterion("G2.11c", None, th_c, True, why)
        return
    rng = np.random.default_rng(SEED)
    r_pts, r_tri = ret_u.sample(N_SAMPLES, rng)
    s_pts, s_tri = src_u.sample(N_SAMPLES, rng)

    def in_region(pts):
        d = pts - grip["co"]
        sa = d @ grip["axis"]
        ra = np.linalg.norm(d - np.outer(sa, grip["axis"]), axis=1)
        return (ra <= grip["r_excl"]) & (sa >= grip["gap_lo"]) & (sa <= grip["gap_hi"])

    region = {"gap_lo_mm": mm(grip["gap_lo"]), "gap_hi_mm": mm(grip["gap_hi"]),
              "r_grip_front_mm": mm(grip["r_front"]), "r_lathe_max_mm": mm(grip["r_lathe"]),
              "lathe_rings": grip["lathe_rings"], "n_lathe_verts_in_gap": grip["n_in_gap"],
              "r_excl_mm": mm(grip["r_excl"]),
              "n_front_window_verts": grip["n_window"],
              "grip_co_m": [round(float(x), 6) for x in grip["co"]],
              "grip_axis": [round(float(x), 6) for x in grip["axis"]]}
    r_labels = np.array([ret_u.labels[k] for k in ret_u.tri_label[r_tri]], dtype=object)

    def regions(pts, hand_labels=None, src_body=None, tri=None):
        sub = {"grip": in_region(pts)}
        if hand_labels is None:  # SRC -> RETOPO: whole original hand (SRC_hi samples only)
            sub.update({h: m & src_body for h, m in src_hand_regions(pts, balls).items()})
            sub["mouth"] = in_mouth(pts, design["mouth"]) & src_body
            for k, z in zone_items(design):
                sub[k] = in_zone(pts, z, design["src_hi_bvh"]) & src_body
        else:  # RETOPO -> SRC: every sample on the hand_l / hand_r shells
            sub.update({h: hand_labels == h for h, _w in BALL_HANDS})
            head = hand_labels == "head"
            sub["mouth"] = design["ret_mouth_tri"][tri] | (in_mouth(pts, design["mouth"]) & head)
            for k, z in zone_items(design):
                sub[k] = in_zone(pts, z, design["src_hi_bvh"]) & head
        ex = np.zeros(len(pts), dtype=bool)
        for m in sub.values():
            ex |= m
        return ex, sub

    print(f"[{GATE}/{CHECKER}] G2.11a RETOPO->SRC: {len(r_pts)} samples")
    ex_a, sub_a = regions(r_pts, r_labels, tri=r_tri)
    m_a, pp_a = one_direction(ret_u, src_u, r_pts, r_tri, r_labels, ex_a,
                              seam_band(r_pts, pivots), sub_a)
    print(f"[{GATE}/{CHECKER}] G2.11b SRC->RETOPO: {len(s_pts)} samples")
    ex_b, sub_b = regions(s_pts, None, src_u.tri_label[s_tri] == src_u.labels.index(SRC_BODY)
                          if SRC_BODY in src_u.labels else np.zeros(len(s_pts), dtype=bool))
    m_b, pp_b = one_direction(src_u, ret_u, s_pts, s_tri, None, ex_b,
                              seam_band(s_pts, pivots), sub_b)
    m_a.update(region)
    m_b.update(region)
    m_a["balls"] = ball_report(balls)
    m_b["balls"] = ball_report(balls)
    dreg = {"mouth": design["mouth"]["info"], **{k: z["info"] for k, z in zone_items(design)},
            "zones_record_only": design["zones"].get("_record_only"), "basis": design.get("basis")}
    m_a["design_regions"] = dreg
    m_b["design_regions"] = dreg
    seams = [{"pivot": n, "co_m": [round(float(x), 6) for x in c],
              "axis": [round(float(x), 6) for x in a], "band_radius_mm": mm(rb),
              "half_len_mm": mm(SEAM_HALF_LEN)} for n, c, a, rb in seam_cylinders(pivots)]
    m_a["seam_cylinders"] = seams
    m_b["seam_cylinders"] = seams

    shells_r = {"n_shells": ret_u.n_shells, "n_closed": int(ret_u.shell_closed.sum())}
    shells_s = {"n_shells": src_u.n_shells, "n_closed": int(src_u.shell_closed.sum())}
    m_a["shells"] = shells_r
    m_b["shells"] = shells_s
    m_a["ok_outside_seam"] = bool(dev_ok(m_a["outside_seam"]))
    m_b["ok_outside_seam"] = bool(dev_ok(m_b["outside_seam"]))
    ev.criterion("G2.11a", m_a, TH_DEV, dev_ok(m_a),
                 "RETOPO=GOB_mesh U GOB_club samples -> SRC=SRC_hi U SRC_club_hi surface; "
                 "point part = own part_id (GOB_club = 'club'). " + note_m)
    ev.criterion("G2.11b", m_b, TH_DEV, dev_ok(m_b),
                 "SRC samples -> RETOPO surface; point part = part of the nearest RETOPO "
                 "triangle. " + note_m)
    ev.criterion("G2.11c", {"retopo_to_src": pp_a, "src_to_retopo": pp_b,
                            "part_names": part_names}, th_c, True,
                 "per-part stats over judged points (not hidden, not interface, not in the "
                 "design-change region); n_hidden / n_interface / n_design_excluded per part = "
                 "excluded points; RETOPO points by own part, SRC points by nearest RETOPO "
                 "triangle part")


def crit_g2_12(ev, ret_objs, src_objs, frame_bbox, tmpdir, why, pivots=None, balls=None, htris=None,
               dtris=None):
    """G2.12 silhouettes + G2.13 overlays (+ report-only seam-band / ball-fist / HR2 design-region detail;
    dtris = design_tris())."""

    def directed(a, b, max_elems=4_000_000):
        """Per-pixel min distance (px) from contour pixels a to contour pixels b."""
        out = np.empty(len(a))
        step = max(1, max_elems // max(1, len(b)))
        for i in range(0, len(a), step):
            ca = a[i:i + step]
            out[i:i + step] = np.sqrt(((ca[:, None, :] - b[None, :, :]) ** 2).sum(axis=2).min(axis=1))
        return out

    th_str = f"mm_judged <= {TH_SIL_MM:g} mm + 1 px (px = view mm_per_px)"
    if why:
        for v in VIEWS:
            ev.criterion(f"G2.12_{v}", None, th_str, False, why)
        return
    out_dir = goblib.INSPECT / GATE
    all_src, all_ret = src_objs, ret_objs
    # T121f (2026-09-26 HR2, main decision): character and club judged separately; the union is report-only
    pairs = (("character", [o for o in all_src if o.name == "_g2_" + SRC_BODY],
              [o for o in all_ret if o.name == RET_BODY], dtris),
             ("club", [o for o in all_src if o.name == "_g2_" + SRC_CLUB],
              [o for o in all_ret if o.name == RET_CLUB], None),
             ("union_report", all_src, all_ret, dtris))

    def one(v, src_objs, ret_objs, tag, dtris):
        """G2.12 measurement of one object pair in view v -> (measured, ok, lim)."""
        if not src_objs or not ret_objs:
            return f"missing objects for {tag}", False, None
        p_src = goblib.ortho_render(src_objs, v, Path(tmpdir) / f"sil_src_{v}_{tag}.png",
                                    res_h=SIL_RES_H, frame_bbox=frame_bbox)
        p_ret = goblib.ortho_render(ret_objs, v, Path(tmpdir) / f"sil_ret_{v}_{tag}.png",
                                    res_h=SIL_RES_H, frame_bbox=frame_bbox)
        m_src_raw = goblib.silhouette_mask(p_src)
        m_ret_raw = goblib.silhouette_mask(p_ret)
        # T121e (2026-09-26 HR2, main decision): same rule as G9.6 (compare_g9.c_g96_g97: close_disk then
        # fill_small_holes) before the contours, so enclosed small holes / slits do not count
        c_src, c_ret = g9.close_disk(m_src_raw), g9.close_disk(m_ret_raw)
        m_src, h_src = g9.fill_small_holes(c_src)
        m_ret, h_ret = g9.fill_small_holes(c_ret)
        hole_fill = {"rule": f"compare_g9.close_disk (disk r = {g9.CLOSE_RADIUS} px) then "
                             f"compare_g9.fill_small_holes (< {g9.HOLE_MAX_PX} px, 4-connected, not touching the border)",
                     "src": {"closing_added_px": int((c_src & ~m_src_raw).sum()),
                             "hole_px_filled": int((m_src & ~c_src).sum()), **h_src},
                     "retopo": {"closing_added_px": int((c_ret & ~m_ret_raw).sum()),
                                "hole_px_filled": int((m_ret & ~c_ret).sum()), **h_ret}}
        fw, fh = frame_size(frame_bbox, v)
        px = goblib.contour_max_dev_px(m_src, m_ret)
        mm_per_px = fh / SIL_RES_H * 1000.0
        dev_mm = px * mm_per_px
        if tag == "union_report":
            png = write_png(out_dir / f"overlay_{v}.png", overlay(m_src, m_ret))
            ev.render(png)
        measured = {"px": round(float(px), 4), "mm": round(float(dev_mm), 4),
                    "mm_per_px": round(mm_per_px, 5), "frame_h_m": round(fh, 5),
                    "frame_w_m": round(fw, 5), "image_hw": list(m_src.shape),
                    "src_px": int(m_src.sum()), "retopo_px": int(m_ret.sum()),
                    "src_only_px": int((m_src & ~m_ret).sum()),
                    "retopo_only_px": int((~m_src & m_ret).sum()),
                    "src_px_raw": int(m_src_raw.sum()), "retopo_px_raw": int(m_ret_raw.sum()),
                    "hole_fill": hole_fill}
        # report-only detail: worst contour pixel and seam-band exclusion
        ca, cb = goblib._contour(m_src), goblib._contour(m_ret)  # (row, col)
        if len(ca) and len(cb):
            da, db = directed(ca, cb), directed(cb, ca)
            sm = seam_mask_2d(pivots, v, frame_bbox, m_src.shape) if pivots is not None \
                else np.zeros(m_src.shape, dtype=bool)
            ia, ib = int(np.argmax(da)), int(np.argmax(db))
            side, rc = ("src", ca[ia]) if da[ia] >= db[ib] else ("retopo", cb[ib])
            row, col = int(rc[0]), int(rc[1])
            d3, r3, u3, uc, vc, wc, ps, _rw = frame_geom(frame_bbox, v, m_src.shape[0])
            uu = uc + (col + 0.5 - m_src.shape[1] / 2.0) * ps
            vv = vc + (m_src.shape[0] / 2.0 - (row + 0.5)) * ps
            world = r3 * uu + u3 * vv + d3 * wc
            oa = ~sm[ca[:, 0].astype(int), ca[:, 1].astype(int)]
            ob = ~sm[cb[:, 0].astype(int), cb[:, 1].astype(int)]
            px_out = max(float(da[oa].max()) if oa.any() else 0.0,
                         float(db[ob].max()) if ob.any() else 0.0)
            h_t, h_cnt = htris if htris is not None else (np.zeros((0, 3, 3)), {})
            bm = hand_mask_2d(h_t, v, frame_bbox, m_src.shape)
            ba = bm[ca[:, 0].astype(int), ca[:, 1].astype(int)]
            bb = bm[cb[:, 0].astype(int), cb[:, 1].astype(int)]
            xa = np.zeros(len(ca), dtype=bool)
            xb = np.zeros(len(cb), dtype=bool)
            dreg, dmasks = {}, {}
            for dk, (d_t, d_cnt) in (dtris or {}).items():
                dm = hand_mask_2d(d_t, v, frame_bbox, m_src.shape)
                dmasks[dk] = dm
                ra = dm[ca[:, 0].astype(int), ca[:, 1].astype(int)]
                rb = dm[cb[:, 0].astype(int), cb[:, 1].astype(int)]
                xa |= ra
                xb |= rb
                px_d = max(float(da[ra].max()) if ra.any() else 0.0, float(db[rb].max()) if rb.any() else 0.0)
                dreg[dk] = {"px": round(px_d, 4), "mm": round(px_d * mm_per_px, 4),
                            "n_contour_px_src": int(ra.sum()), "n_contour_px_retopo": int(rb.sum()),
                            "mask_px": int(dm.sum()), "tris": d_cnt, "dilate_px": HAND_DILATE_PX}
            ja, jb = oa & ~ba & ~xa, ob & ~bb & ~xb
            px_j = max(float(da[ja].max()) if ja.any() else 0.0,
                       float(db[jb].max()) if jb.any() else 0.0)
            px_h = max(float(da[ba].max()) if ba.any() else 0.0,
                       float(db[bb].max()) if bb.any() else 0.0)

            def px_world(r_, c_):
                u_ = uc + (c_ + 0.5 - m_src.shape[1] / 2.0) * ps
                w_ = vc + (m_src.shape[0] / 2.0 - (r_ + 0.5)) * ps
                return [round(float(x), 5) for x in (r3 * u_ + u3 * w_ + d3 * wc)]

            def mask_dist(mask, r_, c_):
                ys, xs = np.nonzero(mask)
                if not len(ys):
                    return None
                k = int(np.argmin((ys - r_) ** 2 + (xs - c_) ** 2))
                return {"px": round(float(np.hypot(ys[k] - r_, xs[k] - c_)), 3),
                        "nearest_mask_px_xy": [int(xs[k]), int(ys[k])], "nearest_mask_world_m": px_world(ys[k], xs[k])}

            # report only (T121b): location of the judged maximum
            jmax = None
            if ja.any() or jb.any():
                ka = int(np.nonzero(ja)[0][np.argmax(da[ja])]) if ja.any() else None
                kb = int(np.nonzero(jb)[0][np.argmax(db[jb])]) if jb.any() else None
                if kb is None or (ka is not None and da[ka] >= db[kb]):
                    j_side, jp, other, jd, other_mask = "src", ca[ka], cb, float(da[ka]), m_ret
                else:
                    j_side, jp, other, jd, other_mask = "retopo", cb[kb], ca, float(db[kb]), m_src
                jr, jc = int(jp[0]), int(jp[1])
                ko = int(np.argmin(((other - jp) ** 2).sum(axis=1)))
                orr, occ = int(other[ko][0]), int(other[ko][1])
                inside_other = bool(other_mask[jr, jc])
                o_side = "retopo" if j_side == "src" else "src"
                jmax = {"px": round(jd, 4), "mm": round(jd * mm_per_px, 4), "side": j_side,
                        "px_xy": [jc, jr], "world_m": px_world(jr, jc),
                        "in_src_mask": bool(m_src[jr, jc]), "in_retopo_mask": bool(m_ret[jr, jc]),
                        "nearest_other_contour_px_xy": [occ, orr], "nearest_other_contour_world_m": px_world(orr, occ),
                        "outside_silhouette": o_side if inside_other else j_side,
                        "outside_rule": "the judged contour pixel lies inside the other mask -> the other silhouette "
                                        "reaches further out there, else this one does",
                        "dist_to_design_masks_px": {k: mask_dist(m, jr, jc) for k, m in dmasks.items()},
                        "dist_to_hand_mask_px": mask_dist(bm, jr, jc), "dist_to_seam_mask_px": mask_dist(sm, jr, jc),
                        "masks_dilate_px": HAND_DILATE_PX,
                        "note": "mask distances are to the dilated masks (projected region tris + dilate_px); "
                                "distance to the undilated projection ~= px + dilate_px"}
            measured.update({
                "max_px_xy": [col, row], "max_side": side,
                "max_world_m": [round(float(x), 5) for x in world],
                "max_in_seam_band_2d": bool(sm[row, col]),
                "max_in_hand_region_2d": bool(bm[row, col]),
                "px_outside_seam": round(px_out, 4),
                "mm_outside_seam": round(px_out * mm_per_px, 4),
                "seam_mask_px": int(sm.sum()),
                "px_judged": round(px_j, 4), "mm_judged": round(px_j * mm_per_px, 4),
                "hand_region": {"px": round(px_h, 4), "mm": round(px_h * mm_per_px, 4),
                                "n_contour_px_src": int(ba.sum()), "n_contour_px_retopo": int(bb.sum()),
                                "mask_px": int(bm.sum()), "tris": h_cnt, "dilate_px": HAND_DILATE_PX,
                                "balls": ball_report(balls)},
                "design_regions_2d": dreg, "judged_max_report": jmax})
        lim = TH_SIL_MM + mm_per_px
        measured["limit_mm"] = round(lim, 4)
        mo = measured.get("mm_judged")
        ok = mo is not None and mo <= lim
        measured["objects"] = {"src": [o.name for o in src_objs], "retopo": [o.name for o in ret_objs],
                               "design_regions_applied": sorted(dtris or {})}
        return measured, bool(ok), lim

    for v in VIEWS:
        res = {tag: one(v, so, ro, tag, dt) for tag, so, ro, dt in pairs}
        measured = {tag: r[0] for tag, r in res.items()}
        for tag in ("character", "club", "union_report"):
            r = res[tag][0]
            measured[f"mm_judged_{tag}"] = r.get("mm_judged") if isinstance(r, dict) else None
        lim = res["character"][2] or res["club"][2]
        measured["limit_mm"] = round(lim, 4) if lim is not None else None
        ok = res["character"][1] and res["club"][1]
        ev.criterion(f"G2.12_{v}", measured, f"character and club each: {th_str}"
                     + (f" = {lim:.4f} mm" if lim is not None else ""), bool(ok),
                     "T121f (2026-09-26 HR2, main decision): judged per object - character = GOB_mesh vs SRC_hi "
                     "(seam band, hand region, mouth / eye design regions excluded), club = GOB_club vs SRC_club_hi "
                     "(seam band and hand region excluded; no design regions); ok = character ok and club ok; "
                     "union_report = GOB_mesh + GOB_club vs SRC_hi + SRC_club_hi as before (report only; the gap "
                     "between two objects is not a shape property); overlay_<view>.png = union. Each block: "
                     "masks: 2 px closing then enclosed holes < 500 px filled (G9.6 rule, T121e 2026-09-26; "
                     "hole_fill = counts per mask, *_px_raw = before); "
                     "mm_judged = symmetric contour max over contour pixels outside BOTH the projected "
                     "seam-band region and the projected hand (ball fist) region (spec G2.12 2026-09-25: "
                     "hand region (2026-09-25 correction) = pixel centres covered by the projected "
                     "triangles of the hand_l / hand_r ball shells (RETOPO) and of the SRC_hi triangles "
                     "whose centroid is in the G2.11 SRC hand region (s > 0 from wrist pivot, wrist-axis "
                     f"distance <= {HAND_AXIS_R * 1000:g} mm), dilated {HAND_DILATE_PX} px; tris = "
                     "triangle counts per hand; report-only 'hand_region' = max over the contour "
                     "pixels inside it + contour pixel counts); HR2 2026-09-26: the projected mouth region and "
                     f"{', '.join(ZONE_KEYS)} (tris from design_tris: RETOPO region / head tris + SRC_hi tris "
                     f"with centroid inside, dilated {HAND_DILATE_PX} px) are excluded from mm_judged too and "
                     "reported in design_regions_2d (GOB_mesh rendered in its Basis shape); mm_outside_seam = "
                     "seam-band exclusion only (reference). "
                     f"ok = mm_judged <= {TH_SIL_MM:g} mm + 1 px (limit_mm, 1 px = this view's "
                     "mm_per_px; spec G2.12, 2026-09-25 user decision: raster quantisation allowance, "
                     "seam-band projection excluded). goblib.ortho_render res_h "
                     f"{SIL_RES_H}, same frame_bbox for both unions (union of both bboxes, padded "
                     f"{FRAME_PAD:.0%} of the largest extent on every side); silhouette_mask -> "
                     "contour_max_dev_px (symmetric Hausdorff of contour pixels) = px / mm over the "
                     f"whole contour (kept for reference, mm = px * frame_h_m / {SIL_RES_H} * 1000); "
                     "px/mm_outside_seam = same symmetric max but only over contour pixels outside "
                     "the projected seam-band region (distances still to the full other contour); "
                     "max_px_xy = [x col, y row from image top] of the contour pixel with the "
                     "largest whole-contour distance; max_side = mask whose contour it belongs to; "
                     "max_world_m = that pixel centre back-projected onto the frame plane at the "
                     "frame depth centre; max_in_seam_band_2d = pixel inside the view projection of "
                     f"any seam-band cylinder (wrist/ankle pivots, |s| <= {SEAM_HALF_LEN * 1000:g} mm, "
                     f"radius + {SEAM_MARGIN * 1000:g} mm; convex hull of both projected end "
                     "circles); seam_mask_px = projected band area in px")


def render_head(ev, ret_objs, src_objs, head_bbox):
    """G2.13 head close-up: same front framing for SRC and RETOPO, plus side-by-side."""
    out_dir = goblib.INSPECT / GATE
    fb = padded_bbox(*head_bbox)
    p_src = goblib.ortho_render(src_objs, "front", out_dir / "head_closeup_src.png",
                                res_h=CLOSEUP_RES_H,
                                frame_bbox=fb, colors={o.name: CLAY for o in src_objs})
    p_ret = goblib.ortho_render(ret_objs, "front", out_dir / "head_closeup_retopo.png",
                                res_h=CLOSEUP_RES_H, frame_bbox=fb, colors={o.name: CLAY for o in ret_objs})
    a = over_white(read_png_rgba(p_src))
    b = over_white(read_png_rgba(p_ret))
    strip, gap = 12, 16
    h, w = a.shape[:2]
    out = np.ones((h + strip, w * 2 + gap, 4), dtype=np.float32)
    out[strip:, :w] = a
    out[strip:, w + gap:] = b
    out[:strip, :w, :3] = OVERLAY_SRC
    out[:strip, w + gap:, :3] = OVERLAY_RET
    p = write_png(out_dir / "head_closeup.png", out)
    for x in (p_src, p_ret, p):
        ev.render(x)


# ---------------------------------------------------------------- main
_STATE = {"missing": []}


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ev.add_input(g9.__file__)
    missing = _STATE["missing"]

    if not BLEND.exists():
        missing.append(rel(BLEND))
    else:
        loaded = bpy.data.filepath
        if not loaded or Path(loaded).resolve() != BLEND.resolve():
            print(f"[{GATE}/{CHECKER}] opening {rel(BLEND)} (loaded: {loaded!r})")
            bpy.ops.wm.open_mainfile(filepath=str(BLEND))
        ev.add_input(BLEND)
    if not SRC_BLEND.exists():
        missing.append(rel(SRC_BLEND))
    else:
        ev.add_input(SRC_BLEND)
    ev.add_stage_inputs("s02")
    pivots = None
    if not PIVOTS.exists():
        missing.append(rel(PIVOTS))
    else:
        ev.add_input(PIVOTS)
        with open(PIVOTS, "r", encoding="utf-8") as f:
            pivots = json.load(f)
    parts = None
    if not PARTS.exists():
        missing.append(rel(PARTS))
    else:
        with open(PARTS, "r", encoding="utf-8") as f:
            parts = json.load(f)

    miss_other = [m for m in missing if m != rel(PIVOTS)]
    why = ("missing input: " + ", ".join(miss_other)) if miss_other else ""
    why_11 = ("missing input: " + ", ".join(missing)) if missing else ""
    tmp = {"objects": [], "meshes": [], "materials": [], "images": []}
    tmpdir = tempfile.mkdtemp(prefix="g2shape_")
    try:
        ret_u = src_u = None
        ret_objs, src_objs = [], []
        head_bbox = None
        part_names = {}
        grip = None
        lathe_v = None
        balls = {}
        design = None
        basis = None
        if not why:
            errs = []
            id2name = {int(v): str(k) for k, v in parts.items()}
            part_names = {str(k): int(v) for k, v in parts.items()}
            ret_u = Union("RETOPO")
            for name in (RET_BODY, RET_CLUB):
                o = bpy.data.objects.get(name)
                if o is None or o.type != "MESH":
                    errs.append(f"object {name} not found / not a mesh in {rel(BLEND)}")
                    continue
                if name == RET_BODY:
                    basis = show_basis(o)
                v, t, part, err = read_eval_mesh(o, want_part=(name == RET_BODY))
                if err:
                    errs.append(err)
                    continue
                if name == RET_BODY:
                    bw = basis_world(o)
                    basis["eval_vs_basis_max_mm"] = (mm(np.abs(v - bw).max()) if len(bw) == len(v) and len(v)
                                                     else f"vertex count {len(v)} != {len(bw)}")
                    mr, m_err = mouth_region(o, v, t, goblib.sha256(BLEND))
                    zones, z_err = load_zones()
                    if m_err or z_err:
                        errs.append("HR2 design regions: " + "; ".join(x for x in (m_err, z_err) if x))
                    else:
                        design = {"mouth": mr, "zones": zones, "basis": basis, "_body_tri": mr["tri"]}
                    lid = {pid: ret_u.label_id(id2name.get(int(pid), f"part_{int(pid)}"))
                           for pid in np.unique(part)}
                    ret_u.add(v, t, np.array([lid[p] for p in part], dtype=np.int64))
                    balls = hand_balls(v, t, part, parts, pivots)
                    hid = parts.get("head")
                    if hid is not None and np.any(part == int(hid)):
                        hv = v[np.unique(t[part == int(hid)])]
                        head_bbox = (hv.min(axis=0), hv.max(axis=0))
                else:
                    ret_u.add(v, t, np.full(len(t), ret_u.label_id(CLUB_PART)))
                    lathe_v = v
                ret_objs.append(o)
            src_u = Union("SRC")
            meshes = load_src_meshes(tmp)
            for name in (SRC_BODY, SRC_CLUB):
                me = meshes.get(name)
                if me is None:
                    errs.append(f"mesh {name} not found in {rel(SRC_BLEND)}")
                    continue
                v, t, _pi = mesh_tris(me, np.eye(4))  # G1.1: SRC transforms are applied
                src_u.add(v, t, np.full(len(t), src_u.label_id(name)))
                if name == SRC_CLUB and pivots is not None:
                    grip, g_err = grip_exclusion(v, pivots, lathe_v)
                    if g_err and not why_11:
                        why_11 = g_err
                o = bpy.data.objects.new("_g2_" + name, me)
                tmp["objects"].append(o)
                src_objs.append(o)
            if errs:
                why = "; ".join(errs)
                why_11 = "; ".join(x for x in (why_11, why) if x)
            else:
                ret_u.finish()
                src_u.finish()
                rt = np.zeros(len(ret_u.t), dtype=bool)
                rt[:len(design["_body_tri"])] = design["_body_tri"]   # GOB_mesh triangles come first
                design["ret_mouth_tri"] = rt
                sh = src_u.tri_label == src_u.labels.index(SRC_BODY)
                design["src_hi_bvh"] = BVHTree.FromPolygons(src_u.v.tolist(), src_u.t[sh].tolist(), epsilon=0.0)
                print(f"[{GATE}/{CHECKER}] RETOPO tris={len(ret_u.t)} shells={ret_u.n_shells} "
                      f"closed={int(ret_u.shell_closed.sum())}; SRC tris={len(src_u.t)} "
                      f"shells={src_u.n_shells} closed={int(src_u.shell_closed.sum())}")

        crit_g2_11(ev, ret_u, src_u, part_names, grip, why_11, pivots, balls, design)

        frame_bbox = None
        if not why:
            r_mn, r_mx = ret_u.bbox()
            s_mn, s_mx = src_u.bbox()
            frame_bbox = padded_bbox(np.minimum(r_mn, s_mn), np.maximum(r_mx, s_mx))
        htris = hand_tris(ret_u, src_u, balls) if not why else None
        dtris = design_tris(ret_u, src_u, design) if not why else None
        crit_g2_12(ev, ret_objs, src_objs, frame_bbox, tmpdir, why, pivots, balls, htris, dtris)

        if not why:
            if head_bbox is not None:
                render_head(ev, ret_objs, src_objs, head_bbox)
            else:
                print(f"[{GATE}/{CHECKER}] head close-up skipped: no 'head' faces in {RET_BODY}")
    finally:
        cleanup(tmp)
        shutil.rmtree(tmpdir, ignore_errors=True)
    ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
