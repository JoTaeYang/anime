"""s02c_rigid.py - T10: rigid parts cut from SRC_hi (head, hands, shoes, belt) + club lathe.

Input : rig/work/r01b_limbs.blend (SRC_hi, SRC_club_hi, GOB_body_tmp)  - none of them is modified
Output: rig/work/r01c_rigid.blend (+ GOB_head, GOB_hand_l, GOB_hand_r, GOB_shoe_l, GOB_shoe_r, GOB_belt,
        GOB_club in collection GOB), renders rig/work/r01c_*.png, rig/data/head_design_zones.json (T120a: G2.11 /
        G2.12 design-change zones of the head, see head_design_zones)

Run:  bl.ps1 -Script s02c_rigid.py -Blend work/r01b_limbs.blend

Parts (world coordinates, transforms identity; GOB_club excepted):
  head   (T113, d-29 HR1: even-quad head)  Reference surface (build_head): SRC_hi above the head/torso crease.
         Crease per azimuth (1 deg) = min horizontal radius of the SRC meridian section in z [0.935, 0.995];
         separating line through the crease along the bisector of the head-side and body-side section
         tangents (8 mm chords); SRC triangles clipped on that line, component holding the head top; the open
         rim closed by a hidden skirt (strip HEAD_STRIP_DROP below the rim, HEAD_STRIP_IN inside SRC; ring
         HEAD_SKIRT_IN inward at z = strip min z - HEAD_SKIRT_DROP) and a planar cap.
         GOB_head (build_head_quads) is a new quad mesh on that surface:
         - lattice: box surface lattice, per-axis segments for HEAD_LATTICE_TRIS square quads.  Side row 0 = the
           skirt ring, row 1 = the strip (both as above, per rim vertex), row 2 = the crease rim (resampled by arc
           length, box corners matched by azimuth), rows 3.. = points HEAD_CREASE_S (arc) up the SRC meridian of
           each rim vertex (the underside fold); hidden cap = triangle fan on the ring.  All other lattice verts:
           ray from the head bbox centre onto the reference, then HEAD_ITERS x (tangential uniform Laplacian
           relax HEAD_RELAX + projection onto the smooth face reference = MLS quadric, radius HEAD_MLS_R, on
           HEAD_MLS_N SRC head samples without the eye / fang footprints).
         - eyes: seeds from a front polynomial y(x, z) (protrusion components, head_seeds); frame (eye_frame): axis
           = plateau plane normal, centre = dome apex over the eyeball-wall circle.  4 * EYE_BLOCK azimuths; per
           azimuth the SRC section profile (walked from the dome outward) gives the loops junction (face),
           undercut, equator, shoulder, plateau, wall bottom, dome edge (eye_features: descent-angle
           thresholds); the dome edge loop is closed by a Coons grid projected onto SRC.  Azimuths where the SRC
           eye lump is detached from the face (its back floats in front of the face) take the face point below
           as the junction.  An EYE_BLOCK x EYE_BLOCK cell hole on the lattice front is bridged to the junction
           loop by quads (equal vertex counts).
         - fangs (fang_geometry): flat plate + near-vertical walls; top outline = lines fitted to the plate verts,
           foot outline = top edges moved out by the measured wall foot; 12-vertex foot / top loops, wall quads,
           top = 3 patches of 2 x 2 quads; a FANG_BLOCK cell hole below the eye block (12 boundary verts, chosen
           by the containment margin of the foot triangle) bridged by quads.
         - straddle (finish_head): HEAD_STRADDLE_PASSES x each free lattice / crease-row / eye vertex moves along
           its normal by -0.5 x the mean signed distance (to the reference) of its visible faces' centres; rim,
           skirt, cap and fang verts stay.
         - designed hard edges (T113c): EDGE bool attribute HEAD_HARD_ATTR on GOB_head = the eye junction and wall
           bottom loops, the fang foot and top outlines and the 3 fang corner edges, and (T113d) the crease rim loop
           where the visible shell meets the hidden strip / skirt (s02d makes only these sharp).
         The fist size keeps its T60 reference: the head width is the x extent of the previous collapse-decimated,
         fitted head (HEAD_FIST_REF_BUDGET tris), computed and discarded.
  hand_x (T60, d-23 section 3 revised 2026-09-25) ball fist like the reference: a closed all-quad
         equal-angle cube sphere (FIST_N x FIST_N quads per cube face = 6 * FIST_N^2 quads, 12 * FIST_N^2
         tris), verts on a sphere of radius R, one cube face centred on the wrist axis (local +Z = axis,
         local X = world +Z orthogonalised).  Nothing is cut from SRC_hi, no stub / hole / cap.
         Size: R = FIST_RATIO * head width / 2 * FIST_R_ADJ; head width = x extent of the built GOB_head.
           FIST_RATIO = fist diameter / head width measured on the reference keyframes (ref/keyframes,
           896 x 1152 px): olive-green mask (G > 0.88 R, B < 0.72 G, G > 20; 5x5 closing bridges the
           grid lines, 3x3 opening, holes filled).  Head width = widest green run through the head
           (rows of the head only, run containing the face centre column).  Fist diameter = 2 x radius
           of an iterative least-squares circle fit to the mask boundary pixels within max(4 px, 8 %)
           of the current circle (start: largest inscribed circle in a box around the fist).
             key_0001 fist screen-left (club hand)  d 106.5 px / head 303 px = 0.351 (arc 320 deg)
             key_0001 fist screen-right             d 101.0 px / head 303 px = 0.333 (arc 310 deg)
             key_0009 fist screen-right             d 105.7 px / head 299 px = 0.354 (arc 290 deg)
           not used: key_0009 screen-left (motion blur, d 86.5 px, arc 220 deg), key_0015 (3/4 turned
           head joined with the far fist in the silhouette, head run 400 px; far fist occluded d 91.8 px;
           club fist merged with the body, arc 80 deg).  FIST_RATIO = mean of the three used = 0.346.
         Centre: c = wrist.co + d * wrist.axis (on the wrist axis, each side from its own pivots, the
         same d both sides).  d = the largest value (from R down, FIST_D_STEP steps) where on both sides
         every wrist _1 / _end ring vertex of GOB_body_tmp is >= FIST_RING_DEPTH + FIST_MARGIN inside
         the ball mesh (G2.14) and, on the right, the GOB_club lathe surface in s in [-20, +20] mm about
         grip_r is >= FIST_GRIP_DEPTH + FIST_MARGIN inside hand_r and the grip axis chord through the
         ball is >= FIST_CHORD_MIN of the diameter (G2.16).
  shoe_x SRC_hi beyond the plane s = WRIST_CUT (15 mm above the ankle pivot, axis = pivot axis, leg tube
         radius); verts with s in [cut, pivot) are pulled radially to r <= min(r_tube, r_bodytube) -
         HIDE_MARGIN (r_bodytube = ray from the axis to GOB_body_tmp), planar cap at the cut; where the SRC
         ankle flare stands outside the body tube (s > cut + COVER_START) the shell covers it instead
         (COVER_IN inside SRC, only where that is > COVER_GAP outside the body tube);
         after decimation the shell above s = SHOE_SEAM_S is replaced by a planar seam lip (lip_seam):
         clean rim just outside the tube, hidden inner ring and fan inside it.
  belt   visible band per azimuth (1 deg) = SRC meridian section between the bottom and top lip creases
         (most concave turn), decimated as an open band; then each rim is zipped to a hidden inner ring
         (BELT_IN inside GOB_body_tmp, BELT_Z_EXT outside the body belt rows, under the lips) and the
         rings are joined;
         body belt-band verts still < BELT_PUSH_DEPTH inside are fixed by moving rim / hidden verts of
         the nearest face along its normal (count printed).
  Shoes (closed) and the belt band (open) are decimated with the Decimate modifier
  (collapse, triangulate), degenerate slivers dissolved (clean_degenerate), then fitted to the dense
  pre-decimation surface (fit_surface: two-sided IRLS least squares, connectivity unchanged; hidden
  zones / rims fixed); the belt budget includes the hidden part added after decimation.
  Budgets: BUDGET per part, GOB_mesh (body + 6 rigid parts) <= GOB_MESH_MAX.
  Deviation report: hidden / interface points and the grip design-change region (axis distance <=
  GRIP_REGION_R, s in the SRC_club_hi fist gap about grip_r) are excluded.
  Hidden-zone clamps are re-applied after decimation (shoes).
  club   lathe (not decimated): 16 verts per ring, ring centre = circle fit of the SRC_club_hi section
         perpendicular to the grip_r axis, ring verts = ray hits from that centre (azimuth k * 22.5 deg,
         local X = 0 deg); fist gap = straight interpolation between the last back / first front ring;
         ring verts pushed out by CLUB_RING_SCALE so the 16-gon straddles the section; rings chosen
         greedily (max interpolation error) up to CLUB_RINGS; 4x4-quad caps whose
         inner verts are projected onto the dome ends along the axis.  Object origin = grip_r.co,
         local +Y = grip_r.axis, local Z = world +Z made orthogonal to the axis, local X = Y x Z;
         mesh data in that local frame (transform NOT applied).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402
from mathutils.kdtree import KDTree  # noqa: E402

IN_LOOPS = goblib.WORK / "loops_limbs.json"
OUT_BLEND = goblib.WORK / "r01c_rigid.blend"
COLL_NAME = "GOB"
BODY_NAME = "GOB_body_tmp"
PART_OBJ = {"head": "GOB_head", "hand_l": "GOB_hand_l", "hand_r": "GOB_hand_r", "shoe_l": "GOB_shoe_l",
            "shoe_r": "GOB_shoe_r", "belt": "GOB_belt", "club": "GOB_club"}
BUDGET = {"head": 2300, "hand_l": 470, "hand_r": 470, "shoe_l": 375, "shoe_r": 375, "belt": 450}
GRIP_REGION_R = 0.0469      # G2.11 grip design-change region radius about the grip_r axis (s in the fist gap)
GOB_MESH_MAX = 8800          # body + 6 rigid parts (T113: G2.8 ceiling 9500 minus ~700 for the d-29 mouth)
CLUB_RINGS = 30             # 29 segments * 32 + 2 caps * 32 = 992 tris
CLUB_NSEG = 16
CLUB_GAP_MARGIN = 0.004     # first / last ring this far from the fist cut faces of SRC_club_hi
CLUB_RING_SCALE = 2.0 / (1.0 + math.cos(math.pi / CLUB_NSEG))   # ring verts outside, edge mids inside

WRIST_CUT = -0.015          # cut plane s (limb axis, from the pivot); = wrist_x_0 / ankle_x_0
COVER_START = 0.005         # wrist / ankle zone: SRC-flare cover rule only this far past the cut ...
COVER_IN = 0.001            # ... with the shell this far inside SRC
COVER_GAP = 0.001           # shoes: cover only where SRC - COVER_IN is > this outside the body tube
FIST_REF = (("key_0001", "fist screen-left (club hand)", 106.5, 303),   # (frame, fist, diameter px, head px)
            ("key_0001", "fist screen-right", 101.0, 303),
            ("key_0009", "fist screen-right", 105.7, 299))
FIST_RATIO = sum(d / h for _f, _n, d, h in FIST_REF) / len(FIST_REF)   # fist diameter / head width = 0.346
FIST_R_ADJ = 1.0            # radius factor (allowed 0.85 .. 1.15)
FIST_N = 6                  # quads per cube-face edge: 6 * 36 = 216 quads = 432 tris (<= BUDGET 470)
FIST_RING_DEPTH = 0.003     # G2.14: wrist _1 / _end ring verts this far inside the ball ...
FIST_GRIP_DEPTH = 0.001     # G2.16: club handle (s in +-FIST_GRIP_S about grip_r) this far inside hand_r ...
FIST_MARGIN = 0.001         # ... + this margin (centre search)
FIST_GRIP_S = 0.020
FIST_CHORD_MIN = 0.80       # G2.16: grip-axis chord through the ball >= this * diameter
FIST_D_STEP = 0.0005        # centre search step along the wrist axis
SHOE_SEAM_S = -0.002        # shoes: shell clipped on the plane s = this (ankle axis) -> clean planar seam rim ...
SHOE_LIP_OUT = 0.001        # ... rim >= body tube + this ...
SHOE_LIP_DEPTH = 0.004      # ... then a hidden ring this far up the leg ...
SHOE_LIP_IN = 0.004         # ... this far inside the tube ...
SHOE_LIP_N = 16             # ... with this many verts, closed by a fan (lip_seam)
HIDE_MARGIN = 0.0015        # hidden zone radius = tube radius - this
HEAD_WIN = (0.935, 0.995)   # crease search window (z)
HEAD_SKIRT_IN = 0.015
HEAD_SKIRT_DROP = 0.010
HEAD_STRIP_DROP = 0.006     # skirt first follows SRC this far below the rim ...
HEAD_STRIP_IN = 0.0015      # ... this far inside SRC
HEAD_LATTICE_TRIS = 1500    # box lattice density (six box faces, before the holes): square quads of edge e
HEAD_ITERS = 12             # lattice: tangential uniform-Laplacian relax + re-projection passes
HEAD_RELAX = 0.5
HEAD_MLS_R = 0.050          # smooth face reference: moving least squares quadric radius ...
HEAD_MLS_N = 80000          # ... on this many SRC head samples (eye / fang footprints removed)
HEAD_MLS_MIN = 15
HEAD_FIT_WIN = dict(y_max=-0.12, ny_max=-0.5, z=(0.97, 1.34), x=0.25)   # eye / fang seed detection (front poly)
HEAD_PROT_RES = 0.0015
EYE_BLOCK = 4               # eye hole = EYE_BLOCK x EYE_BLOCK lattice cells; every eye loop has 4 * EYE_BLOCK verts
EYE_EXCL = 0.008            # MLS reference: SRC samples within the eye junction radius + this are not used
FANG_BLOCK = (4, 2)         # fang hole (columns, rows of cells) below the eye block; boundary 12 verts = fang loops
FANG_EXCL = 0.006           # MLS reference: SRC samples within this of the fang foot triangle are not used
FANG_FOOT_MAX = 0.012       # fang wall foot search distance outside the plate edge
HEAD_CREASE_S = (0.012, 0.032, 0.058, 0.090)   # lattice rows 3.. placed on the SRC meridian this far (arc) above the rim
HEAD_STRADDLE_PASSES = 2
HEAD_FIST_REF_BUDGET = 1650  # T60 fist size reference: x width of the collapse-decimated head at this budget
HEAD_HARD_ATTR = "gob_hard_edge"   # T113c: EDGE bool on GOB_head = designed hard edges (s02d: head sharp edges)
ZONE_STEP = 0.001            # T120a eye gap zone: ray grid step in the eye plane ...
ZONE_GAP_MIN = 0.0005        # ... a cell is detached where the lump back and the face are > this apart ...
ZONE_DEPTH_MIN = 0.040       # ... and the lump back lies > this below the eye apex (not the dome / eyeball wall)
ZONE_BIN = 5.0               # azimuth bin (deg) for the zone sectors
ZONE_MARGIN = 0.008          # zone sector: outer radius = max(equator radius, detached cell radius) + this
ZONE_Y_MARGIN = 0.005
ZONE_HEAD_DEPTH = 0.12       # head verts counted for the zone y range: within this depth below the eye apex
ZONE_WALL_R_MIN = 0.050      # T120a-2 depth report: SRC ring outer-wall triangles = eye-plane radius >= this ...
ZONE_WALL_NA = 0.5           # ... |normal . axis| < this, inside the zone sector and within ZONE_HEAD_DEPTH
ZONE_DEPTH_EPS = 0.0005      # exclude_depth_min = min lump-back depth below the local front - this
BELT_IN = 0.002             # hidden inner rings inside GOB_body_tmp
BELT_Z_EXT = 0.0025         # hidden inner rings this far outside the body belt rows (belt_bot_0 / belt_top_0),
                            # i.e. under the lips, between the rows and the lip creases
BELT_COLS_DENSE = 360
BELT_PUSH_DEPTH = 0.0008
BELT_M_OUT = 64
BELT_INNER_N = 24
DEGEN_DIST = 0.0003         # clean_degenerate distance (m)
FIT_ITERS = 6               # post-decimation least-squares fit (fit_surface)
FIT_SAMPLES = 20000
FIT_LAMBDA = 0.05
FIT_D0 = 0.001              # IRLS residual scale
N_DEV = 30000               # deviation samples per object
HIDE_DEPTH = 0.001
IFACE_TOL = 0.0001
RAY_DIRS = [Vector(d).normalized() for d in ((0.577, 0.577, 0.578), (-0.6, 0.3, 0.742), (0.2, -0.7, -0.686))]

COLORS = {"GOB_body_tmp": (0.72, 0.72, 0.72, 1.0), "GOB_head": (0.45, 0.75, 0.40, 1.0),
          "GOB_hand_l": (0.35, 0.55, 0.95, 1.0), "GOB_hand_r": (0.95, 0.40, 0.35, 1.0),
          "GOB_shoe_l": (0.55, 0.40, 0.25, 1.0), "GOB_shoe_r": (0.75, 0.55, 0.30, 1.0),
          "GOB_belt": (0.95, 0.85, 0.30, 1.0), "GOB_club": (0.85, 0.55, 0.20, 1.0)}


def mm(x):
    return round(float(x) * 1000.0, 2)


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- mesh helpers
def obj_arrays(ob):
    me = ob.data
    me.calc_loop_triangles()
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    m = np.array(ob.matrix_world)
    if not np.allclose(m, np.eye(4), atol=1e-9):
        V = V @ m[:3, :3].T + m[:3, 3]
    T = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", T)
    return V, T.reshape(-1, 3)


class Surf:
    """Triangle surface with BVH, nearest / inside (ray parity) / signed depth."""

    def __init__(self, V, T):
        self.V = np.asarray(V, float)
        self.T = np.asarray(T, np.int64)
        self.bvh = BVHTree.FromPolygons(self.V.tolist(), self.T.tolist(), all_triangles=True)
        self.mn = self.V.min(0)
        self.mx = self.V.max(0)

    def dist(self, p):
        loc, _n, _i, d = self.bvh.find_nearest(Vector(p))
        return d if loc is not None else float("inf")

    def inside(self, p):
        if np.any(np.asarray(p) < self.mn) or np.any(np.asarray(p) > self.mx):
            return False
        votes = 0
        for dv in RAY_DIRS:
            o = Vector(p)
            n = 0
            for _ in range(256):
                loc, _nn, _i, _dd = self.bvh.ray_cast(o, dv, 10.0)
                if loc is None:
                    break
                n += 1
                o = loc + dv * 1e-6
            votes += n % 2
        return votes >= 2

    def depth(self, p):
        d = self.dist(p)
        return d if self.inside(p) else -d

    def depths(self, P):
        return np.array([self.depth(p) for p in np.asarray(P)])

    def ray(self, o, d, dist=1.0):
        loc, _n, _i, dd = self.bvh.ray_cast(Vector(o), Vector(d), dist)
        return (np.array(loc), dd) if loc is not None else (None, None)


def components(nv, faces_flat_pairs):
    """vertex labels (min index of the component) from an (E,2) edge array."""
    lab = np.arange(nv, dtype=np.int64)
    e = np.asarray(faces_flat_pairs, np.int64)
    if not len(e):
        return lab
    while True:
        la, lb = lab[e[:, 0]], lab[e[:, 1]]
        if np.array_equal(la, lb):
            return lab
        m = np.minimum(la, lb)
        np.minimum.at(lab, la, m)
        np.minimum.at(lab, lb, m)
        while True:
            nx = lab[lab]
            if np.array_equal(nx, lab):
                break
            lab = nx


def tri_edges(T):
    return np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])


def compact(V, T):
    used, inv = np.unique(T.ravel(), return_inverse=True)
    return V[used], inv.reshape(T.shape), used


def keep_component(V, T, seed_vertex):
    lab = components(len(V), tri_edges(T))
    keep = lab[T[:, 0]] == lab[seed_vertex]
    V2, T2, used = compact(V, T[keep])
    return V2, T2, used


def clip(V, T, f):
    """Keep the part of the triangle surface with f >= 0 (f per vertex, linear on edges).
    Cut vertices are shared per edge.  Returns V2, T2, is_cut (bool per V2 vertex)."""
    f = np.where(np.abs(f) < 1e-9, 1e-9, f)
    pos = f[T] >= 0
    npos = pos.sum(1)
    out = [T[npos == 3]]
    newp = {}
    extra = []
    nv = len(V)

    def cut(i, j):
        k = (i, j) if i < j else (j, i)
        if k not in newp:
            t = f[k[0]] / (f[k[0]] - f[k[1]])
            extra.append(V[k[0]] + t * (V[k[1]] - V[k[0]]))
            newp[k] = nv + len(extra) - 1
        return newp[k]

    add = []
    for tri, p in zip(T[(npos == 1) | (npos == 2)], pos[(npos == 1) | (npos == 2)]):
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        pa, pb, pc = bool(p[0]), bool(p[1]), bool(p[2])
        # rotate so that the odd vertex is first
        if pa == pb:
            a, b, c, pa, pb, pc = c, a, b, pc, pa, pb
        elif pa == pc:
            a, b, c, pa, pb, pc = b, c, a, pb, pc, pa
        if pa:   # a positive, b c negative
            add.append((a, cut(a, b), cut(a, c)))
        else:    # a negative, b c positive
            ab, ac = cut(a, b), cut(a, c)
            add.append((ab, b, c))
            add.append((ab, c, ac))
    if add:
        out.append(np.array(add, np.int64))
    T2 = np.concatenate(out)
    V2 = np.vstack([V, np.array(extra)]) if extra else V.copy()
    is_cut = np.zeros(len(V2), bool)
    is_cut[nv:] = True
    V3, T3, used = compact(V2, T2)
    return V3, T3, is_cut[used]


def boundary_loops(T):
    """Directed boundary loops following the face winding (edge a->b belongs to a face)."""
    d = {}
    for a, b in np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]).tolist():
        d[(a, b)] = d.get((a, b), 0) + 1
    nxt = {}
    for (a, b), n in d.items():
        if (b, a) not in d:
            if a in nxt:
                raise RuntimeError(f"boundary vertex {a} with two outgoing boundary edges")
            nxt[a] = b
    loops, seen = [], set()
    for s in list(nxt):
        if s in seen:
            continue
        lp, v = [], s
        while v not in seen:
            seen.add(v)
            lp.append(v)
            v = nxt[v]
        loops.append(lp)
    return loops


def cap_loop(P, loop, n_rings=4, relax=0):
    """Cap a boundary loop (directed as the face winding) with concentric rings toward the centroid.
    Returns (new points (k,3), faces list referencing loop verts (existing ids) and new ids offset by
    len(P)), new vertex ids."""
    L = np.array(loop)
    ring0 = P[L]
    cen = ring0.mean(0)
    nb = len(P)
    newpts = []
    rings = [list(L)]
    for j in range(1, n_rings):
        t = j / n_rings
        ids = []
        for q in ring0:
            newpts.append(q + t * (cen - q))
            ids.append(nb + len(newpts) - 1)
        rings.append(ids)
    newpts.append(cen)
    cid = nb + len(newpts) - 1
    faces = []
    n = len(L)
    for j in range(len(rings) - 1):
        ra, rb = rings[j], rings[j + 1]
        for i in range(n):
            i1 = (i + 1) % n
            faces.append((ra[i1], ra[i], rb[i], rb[i1]))
    rl = rings[-1]
    for i in range(n):
        faces.append((rl[(i + 1) % n], rl[i], cid))
    newpts = np.array(newpts)
    if relax:
        allP = np.vstack([P, newpts])
        inner = np.arange(nb, nb + len(newpts))
        nbrs = {v: set() for v in inner.tolist()}
        for f in faces:
            for q in range(len(f)):
                a, b = f[q], f[(q + 1) % len(f)]
                if a in nbrs:
                    nbrs[a].add(b)
                if b in nbrs:
                    nbrs[b].add(a)
        order = inner.tolist()
        nb_idx = [np.array(sorted(nbrs[v])) for v in order]
        for _ in range(relax):
            newX = np.array([allP[ix].mean(0) for ix in nb_idx])
            allP[inner] = newX
        newpts = allP[inner]
    return newpts, faces, list(range(nb, nb + len(newpts)))


def triangulate_faces(faces):
    out = []
    for f in faces:
        if len(f) == 3:
            out.append(tuple(f))
        else:
            for k in range(1, len(f) - 1):
                out.append((f[0], f[k], f[k + 1]))
    return np.array(out, np.int64)


def signed_volume(V, T):
    P = V[T] - V.mean(0)
    return float(np.einsum("ij,ij->i", P[:, 0], np.cross(P[:, 1], P[:, 2])).sum() / 6.0)


def orient_outward(V, T):
    """Make winding consistent (bmesh recalc) and outward (positive volume)."""
    me = bpy.data.meshes.new("_s02c_orient")
    me.from_pydata(V.tolist(), [], T.tolist())
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    V2, T2 = mesh_to_arrays(me)
    bpy.data.meshes.remove(me)
    if signed_volume(V2, T2) < 0:
        T2 = T2[:, ::-1].copy()
    return V2, T2


def mesh_to_arrays(me):
    me.calc_loop_triangles()
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    T = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", T)
    return V.reshape(-1, 3), T.reshape(-1, 3)


def decimate(name, V, T, target):
    """Collapse decimation (Decimate modifier, triangulate, no symmetry) to the largest ratio giving
    <= target tris.  (A modifier vertex group blocks collapses completely in 5.1, so none is used.)"""
    me = bpy.data.meshes.new(f"_s02c_dec_{name}")
    me.from_pydata(V.tolist(), [], T.tolist())
    me.validate(clean_customdata=False)
    ob = bpy.data.objects.new(f"_s02c_dec_{name}", me)
    bpy.context.scene.collection.objects.link(ob)
    mod = ob.modifiers.new("dec", "DECIMATE")
    mod.decimate_type = "COLLAPSE"
    mod.use_collapse_triangulate = True
    mod.use_symmetry = False
    n0 = len(T)

    def count(r):
        mod.ratio = r
        dg = bpy.context.evaluated_depsgraph_get()
        oe = ob.evaluated_get(dg)
        m2 = oe.to_mesh()
        n = sum(len(p.vertices) - 2 for p in m2.polygons)
        oe.to_mesh_clear()
        return n

    lo, hi = 0.0, min(1.0, 1.2 * target / n0)
    while count(hi) <= target and hi < 1.0:
        hi = min(1.0, hi * 1.5)
    best = lo
    for _ in range(18):
        mid = 0.5 * (lo + hi)
        if count(mid) <= target:
            best, lo = mid, mid
        else:
            hi = mid
    mod.ratio = best
    dg = bpy.context.evaluated_depsgraph_get()
    oe = ob.evaluated_get(dg)
    m2 = oe.to_mesh()
    V2, T2 = mesh_to_arrays(m2)
    oe.to_mesh_clear()
    bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.meshes.remove(me)
    return clean_degenerate(V2, T2)


def beautify_flat(V, T, vmask):
    """Edge-flip beautify (bmesh beautify_fill) of the faces whose verts are all in vmask (a planar
    region, e.g. the hidden head cap), removing long sliver triangles; verts unchanged."""
    me = bpy.data.meshes.new("_s02c_beaut")
    me.from_pydata(V.tolist(), [], T.tolist())
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    faces = [f for f in bm.faces if all(vmask[v.index] for v in f.verts)]
    edges = [e for e in bm.edges if all(vmask[v.index] for v in e.verts) and not e.is_boundary]
    if faces:
        bmesh.ops.beautify_fill(bm, faces=faces, edges=edges)
    bm.to_mesh(me)
    bm.free()
    V2, T2 = mesh_to_arrays(me)
    bpy.data.meshes.remove(me)
    return V2, T2


def clean_degenerate(V, T):
    """Collapse the sliver / zero-area triangles the collapse decimator leaves in planar regions
    (bmesh dissolve_degenerate, DEGEN_DIST), re-triangulate; unused verts removed."""
    me = bpy.data.meshes.new("_s02c_degen")
    me.from_pydata(V.tolist(), [], T.tolist())
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    for _ in range(4):
        n0 = len(bm.faces)
        bmesh.ops.dissolve_degenerate(bm, dist=DEGEN_DIST, edges=bm.edges[:])
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        if len(bm.faces) == n0:
            break
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
    bm.to_mesh(me)
    bm.free()
    V2, T2 = mesh_to_arrays(me)
    bpy.data.meshes.remove(me)
    return V2, T2


def fit_surface(V, T, Pd, Td, fixed, iters=FIT_ITERS, n_samples=FIT_SAMPLES, lam=FIT_LAMBDA, seed=0):
    """Post-decimation shape fit (connectivity unchanged): least-squares vertex positions minimising
    the two-sided squared distance between the decimated surface (V, T) and the dense pre-decimation
    surface (Pd, Td) - dense samples -> nearest decimated point (barycentric) and decimated samples ->
    nearest dense point, each weighted 1 + (d / FIT_D0)^2 (IRLS toward the max) - plus
    lam * |L (V - V_prev)|^2 (uniform Laplacian of the displacement).
    `fixed` verts keep their position.  Returns the fitted V."""
    rng = np.random.default_rng(seed)
    V = np.asarray(V, float).copy()
    n = len(V)
    tgt = Surf(Pd, Td)
    tp, _ti = sample_surface(Pd, Td, n_samples, rng)
    adj = [set() for _ in range(n)]
    for a_, b_, c_ in T.tolist():
        adj[a_] |= {b_, c_}
        adj[b_] |= {a_, c_}
        adj[c_] |= {a_, b_}
    Lm = np.eye(n)
    for i, nb in enumerate(adj):
        if nb:
            Lm[i, list(nb)] -= 1.0 / len(nb)
    LtL = Lm.T @ Lm
    fixed = np.asarray(fixed, bool)

    def bary(p, tri):
        a_, b_, c_ = V[tri]
        v0, v1, v2 = b_ - a_, c_ - a_, p - a_
        d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
        d20, d21 = v2 @ v0, v2 @ v1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-20:
            return np.array([1 / 3, 1 / 3, 1 / 3])
        w1 = (d11 * d20 - d01 * d21) / den
        w2 = (d00 * d21 - d01 * d20) / den
        return np.clip(np.array([1 - w1 - w2, w1, w2]), 0.0, 1.0)

    for _ in range(iters):
        S = Surf(V, T)
        idx, wts, rhs, dd = [], [], [], []
        for p in tp:
            loc, _n, fi, d_ = S.bvh.find_nearest(Vector(p))
            if loc is None:
                continue
            w = bary(np.array(loc), T[fi])
            idx.append(T[fi])
            wts.append(w / max(w.sum(), 1e-12))
            rhs.append(p)
            dd.append(d_)
        rp, rti = sample_surface(V, T, n_samples, rng)
        for p, fi in zip(rp, rti):
            loc, _n, _f, d_ = tgt.bvh.find_nearest(Vector(p))
            if loc is None:
                continue
            w = bary(p, T[fi])
            idx.append(T[fi])
            wts.append(w / max(w.sum(), 1e-12))
            rhs.append(np.array(loc))
            dd.append(d_)
        idx, wts, rhs = np.array(idx), np.array(wts), np.array(rhs)
        # IRLS toward the max deviation: residual weight 1 + (d / FIT_D0)^2
        ew = np.sqrt(1.0 + (np.array(dd) / FIT_D0) ** 2)
        wts = wts * ew[:, None]
        rhs_w = rhs * ew[:, None]
        AtA = np.zeros((n, n))
        Atb = np.zeros((n, 3))
        for a_ in range(3):
            np.add.at(Atb, idx[:, a_], wts[:, a_:a_ + 1] * rhs_w)
            for b_ in range(3):
                np.add.at(AtA, (idx[:, a_], idx[:, b_]), wts[:, a_] * wts[:, b_])
        reg = lam * float(np.mean(np.diag(AtA)))
        AtA += reg * LtL
        Atb += reg * (LtL @ V)
        big = 1e6 * max(1.0, float(np.max(np.diag(AtA))))
        fi_ = np.nonzero(fixed)[0]
        AtA[fi_, fi_] += big
        Atb[fi_] += big * V[fi_]
        V = np.linalg.solve(AtA, Atb)
    return V


# ---------------------------------------------------------------- meridian sections (around world z)
def section_chain(V, T, theta):
    """SRC section with the half plane through the z axis at azimuth theta (triangles whose centroid is
    on that side).  Returns points (P,3), adjacency list, e."""
    e = np.array([math.cos(theta), math.sin(theta), 0.0])
    n = np.array([-math.sin(theta), math.cos(theta), 0.0])
    cen = V[T].mean(1)
    TT = T[cen @ e > 0.0]
    d = V @ n
    d = np.where(np.abs(d) < 1e-12, 1e-12, d)
    pos = d[TT] >= 0
    cnt = pos.sum(1)
    m = (cnt == 1) | (cnt == 2)
    TT, P = TT[m], pos[m]
    ea, eb = TT, TT[:, [1, 2, 0]]
    cr = P != P[:, [1, 2, 0]]
    ia = ea[cr].reshape(-1, 2)
    ib = eb[cr].reshape(-1, 2)
    lo, hi = np.minimum(ia, ib), np.maximum(ia, ib)
    nv = len(V)
    keys = lo * nv + hi
    uk, inv = np.unique(keys.ravel(), return_inverse=True)
    inv = inv.reshape(-1, 2)
    a_, b_ = uk // nv, uk % nv
    t = d[a_] / (d[a_] - d[b_])
    pts = V[a_] + t[:, None] * (V[b_] - V[a_])
    adj = [[] for _ in range(len(pts))]
    for x, y in inv.tolist():
        adj[x].append(y)
        adj[y].append(x)
    return pts, adj, e


def walk(pts, adj, i0, j1, length):
    path, acc = [i0, j1], [0.0, float(np.linalg.norm(pts[j1] - pts[i0]))]
    prev, cur = i0, j1
    while acc[-1] < length:
        nx = [k for k in adj[cur] if k != prev]
        if not nx:
            break
        prev, cur = cur, nx[0]
        if cur == i0:
            break
        acc.append(acc[-1] + float(np.linalg.norm(pts[cur] - pts[prev])))
        path.append(cur)
    return pts[path], np.array(acc)


def at_len(poly, acc, x):
    x = min(x, acc[-1])
    return np.array([np.interp(x, acc, poly[:, k]) for k in range(3)])


def rz(p, e):
    return np.array([p @ e, p[2]])


# ---------------------------------------------------------------- head
def build_head(src, body):
    V, T = src.V, src.T
    cen = V[T].mean(1)
    reg = (cen[:, 2] > 0.88) & (np.hypot(cen[:, 0], cen[:, 1]) < 0.42)
    Vr, Tr, used = compact(V, T[reg])
    nth = 360
    C = np.zeros((nth, 2))
    NR = np.zeros((nth, 2))
    info = []
    for i in range(nth):
        th = 2 * math.pi * i / nth
        pts, adj, e = section_chain(Vr, Tr, th)
        r = pts @ e
        z = pts[:, 2]
        cand = np.nonzero((z > HEAD_WIN[0]) & (z < HEAD_WIN[1]) & (r > 0.05))[0]
        if not len(cand):
            raise RuntimeError(f"head crease: no section points at azimuth {i} deg")
        ic = int(cand[np.argmin(r[cand])])
        if len(adj[ic]) != 2:
            raise RuntimeError(f"head crease: chain point with {len(adj[ic])} neighbours at azimuth {i}")
        sides = []
        for j1 in adj[ic]:
            poly, acc = walk(pts, adj, ic, j1, 0.030)
            sides.append((at_len(poly, acc, 0.008), at_len(poly, acc, 0.025)))
        hs = 0 if sides[0][1][2] > sides[1][1][2] else 1
        c2 = rz(pts[ic], e)
        h2 = rz(sides[hs][0], e) - c2
        b2 = rz(sides[1 - hs][0], e) - c2
        dvec = unit(unit(h2) + unit(b2))
        nrm = np.array([-dvec[1], dvec[0]])
        if nrm @ h2 < 0:
            nrm = -nrm
        C[i], NR[i] = c2, nrm
        info.append((i, c2[0], c2[1], math.degrees(math.atan2(dvec[1], dvec[0]))))
    # separating function (interpolated over azimuth)
    thv = np.mod(np.arctan2(Vr[:, 1], Vr[:, 0]), 2 * math.pi) / (2 * math.pi) * nth
    i0 = np.floor(thv).astype(int) % nth
    i1 = (i0 + 1) % nth
    t = thv - np.floor(thv)
    R2 = np.stack([np.hypot(Vr[:, 0], Vr[:, 1]), Vr[:, 2]], 1)
    f0 = ((R2 - C[i0]) * NR[i0]).sum(1)
    f1 = ((R2 - C[i1]) * NR[i1]).sum(1)
    f = (1 - t) * f0 + t * f1
    Vc, Tc, is_cut = clip(Vr, Tr, f)
    top = int(np.argmax(Vc[:, 2]))
    Vh, Th, used_h = keep_component(Vc, Tc, top)
    loops = boundary_loops(Th)
    loops.sort(key=lambda lp: -len(lp))
    rim = loops[0]
    P = Vh.copy()
    faces = [tuple(x) for x in Th.tolist()]
    extra_holes = 0
    for lp in loops[1:]:   # classification holes on the head surface (should not happen)
        extra_holes += 1
        npnts, nf, _ = cap_loop(P, lp, n_rings=2, relax=20)
        P = np.vstack([P, npnts])
        faces += nf
    rimP = P[np.array(rim)]
    # strip: rim -> SRC surface HEAD_STRIP_DROP below the rim, HEAD_STRIP_IN inside it (hidden where the
    # body follows SRC, covers SRC where the body lies further inside)
    strip = []
    for q in rimP:
        loc, nrm, _i, _d = src.bvh.find_nearest(Vector((q[0], q[1], q[2] - HEAD_STRIP_DROP)))
        strip.append(np.array(loc) - HEAD_STRIP_IN * np.array(nrm).clip(-1, 1))
    strip = np.array(strip)
    z_skirt = float(strip[:, 2].min()) - HEAD_SKIRT_DROP
    ring = []
    for q in strip:
        rr = math.hypot(q[0], q[1])
        sc = (rr - HEAD_SKIRT_IN) / rr
        ring.append((q[0] * sc, q[1] * sc, z_skirt))
    nb = len(P)
    P = np.vstack([P, strip, np.array(ring)])
    n = len(rim)
    sid = list(range(nb, nb + n))
    kid = list(range(nb + n, nb + 2 * n))
    for i in range(n):
        i1 = (i + 1) % n
        faces.append((rim[i1], rim[i], sid[i], sid[i1]))
        faces.append((sid[i1], sid[i], kid[i], kid[i1]))
    # cap on the skirt ring: the skirt quads use kid[i] -> kid[i1]; the cap needs kid[i1] -> kid[i]
    npnts, nf, _ = cap_loop(P, kid, n_rings=4)
    P = np.vstack([P, npnts])
    faces += nf
    Tt = triangulate_faces(faces)
    meta = {"crease": info, "rim_verts": n, "rim_z": (float(rimP[:, 2].min()), float(rimP[:, 2].max())),
            "z_skirt": z_skirt, "extra_holes": extra_holes, "dense_tris": len(Tt)}
    return P, Tt, meta


# ---------------------------------------------------------------- head: even quads (T113)
def _fit_terms(x, z):
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.stack([np.ones_like(x), x, z, x * x, x * z, z * z, x ** 3, x * x * z, x * z * z, z ** 3,
                     x ** 4, x * x * z * z, z ** 4], -1)


def _tri_normals(V, T):
    n = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]])
    ln = np.linalg.norm(n, axis=1)
    return n / np.maximum(ln, 1e-20)[:, None], 0.5 * ln


def _circle_fit(u, v):
    A = np.column_stack([2 * u, 2 * v, np.ones(len(u))])
    s, *_r = np.linalg.lstsq(A, u * u + v * v, rcond=None)
    return s[0], s[1], math.sqrt(max(s[2] + s[0] ** 2 + s[1] ** 2, 0.0))


def _frame(a):
    x = np.cross([0.0, 0.0, 1.0], a)
    x /= np.linalg.norm(x)
    return x, np.cross(a, x)


def _ray(bvh, o, d, dist=1.0):
    loc, nrm, _i, _d = bvh.ray_cast(Vector(o), Vector(d), dist)
    return (None, None) if loc is None else (np.array(loc), np.array(nrm))


def head_seeds(P, T, onsrc):
    """Eye / fang seeds on the SRC head: front polynomial y(x, z) (3 passes, residual < -2.5 mm rejected),
    protrusion verts (residual < -HEAD_PROT_RES) grouped by mesh edges; per side the largest component with
    z_min > 1.10 (eye: bbox centre) and with z in 1.00..1.12, |x| 0.03..0.18 (fang: base corners / apex)."""
    fnP, faP = _tri_normals(P, T)
    N = np.zeros_like(P)
    np.add.at(N, T.ravel(), np.repeat(fnP * faP[:, None], 3, axis=0))
    N /= np.maximum(np.linalg.norm(N, axis=1), 1e-20)[:, None]
    w = HEAD_FIT_WIN
    base = onsrc & (P[:, 1] < w["y_max"]) & (N[:, 1] < w["ny_max"]) & (P[:, 2] > w["z"][0]) & (P[:, 2] < w["z"][1]) \
        & (np.abs(P[:, 0]) < w["x"])
    m = base.copy()
    for _ in range(3):
        c, *_r = np.linalg.lstsq(_fit_terms(P[m, 0], P[m, 2]), P[m, 1], rcond=None)
        res = P[:, 1] - _fit_terms(P[:, 0], P[:, 2]) @ c
        m = base & (res > -0.0025)
    prot = onsrc & (P[:, 1] < w["y_max"]) & (res < -HEAD_PROT_RES)
    e = tri_edges(T)
    e = e[prot[e[:, 0]] & prot[e[:, 1]]]
    lab = np.where(prot, components(len(P), e), -1)
    comps = [np.nonzero(lab == L)[0] for L in np.unique(lab[lab >= 0])]
    seeds = {}
    for side, sg in (("l", -1.0), ("r", 1.0)):
        ce = [ids for ids in comps if P[ids, 2].min() > 1.10 and 0.03 < sg * P[ids, 0].mean() < 0.26]
        cf = [ids for ids in comps if P[ids, 2].min() > 1.00 and P[ids, 2].max() < 1.12 and 0.03 < sg * P[ids, 0].mean() < 0.18]
        if not ce or not cf:
            raise RuntimeError(f"head seeds {side}: eye components {len(ce)} fang components {len(cf)}")
        Q = P[max(ce, key=len)]
        eye = np.array([(Q[:, 0].min() + Q[:, 0].max()) / 2, (Q[:, 2].min() + Q[:, 2].max()) / 2])
        Q = P[max(cf, key=len)]
        zb = Q[:, 2].min()
        low = Q[:, 2] < zb + 0.004
        fang = np.array([[Q[low][:, 0].min(), zb], [Q[low][:, 0].max(), zb], [Q[np.argmax(Q[:, 2]), 0], Q[:, 2].max()]])
        seeds[side] = {"eye_xz": eye, "fang_xz": fang}
    return seeds


class _SrcFaces:
    def __init__(self, src):
        self.src = src
        self.fn, self.fa = _tri_normals(src.V, src.T)
        self.cen = src.V[src.T].mean(1)


def eye_frame(sf, xz):
    """Eye frame: axis a = normal of the plateau plane (front-surface ray hits at r 44 / 50 / 56 mm, 72 azimuths),
    centre c = the dome apex on the axis through the circle fitted to the eyeball wall (faces with |n.a| < 0.5,
    r < 60 mm, 5..40 mm below the apex); start: axis = area normal of the face annulus 105..125 mm (x, z) about the
    seed, 5 iterations."""
    src = sf.src
    c, _n = _ray(src.bvh, (xz[0], -1.0, xz[1]), (0.0, 1.0, 0.0), 2.0)
    d2 = np.linalg.norm(sf.cen[:, [0, 2]] - xz, axis=1)
    m = (d2 > 0.105) & (d2 < 0.125) & (sf.cen[:, 1] < -0.1) & (sf.fn[:, 1] < -0.3)
    a = unit((sf.fn[m] * sf.fa[m, None]).sum(0))
    info = {}
    for _ in range(5):
        x, y = _frame(a)
        sel = np.linalg.norm(sf.cen - c, axis=1) < 0.12
        D = sf.cen[sel] - c
        u, v, h = D @ x, D @ y, D @ a
        r = np.hypot(u, v)
        w_ = (np.abs(sf.fn[sel] @ a) < 0.5) & (r < 0.06) & (h > -0.04) & (h < -0.005)
        u0, v0, R0 = _circle_fit(u[w_], v[w_])
        c, _n = _ray(src.bvh, c + u0 * x + v0 * y + 0.2 * a, -a, 0.5)
        pts = []
        for k in range(72):
            ph = 2 * math.pi * k / 72
            for rr in (0.044, 0.050, 0.056):
                q, _n = _ray(src.bvh, c + rr * (math.cos(ph) * x + math.sin(ph) * y) + 0.1 * a, -a, 0.3)
                if q is not None:
                    pts.append(q)
        pts = np.array(pts)
        mu = pts.mean(0)
        n = np.linalg.svd(pts - mu)[2][2]
        a = n if n @ a > 0 else -n
        info = {"wall_r": R0, "plateau_rms": float(np.sqrt((((pts - mu) @ a) ** 2).mean()))}
    return c, a, info


def _walk_chain(pts, adj, r, start_r=0.012):
    """section chain walked outward from the dome point nearest to r = start_r (front-most there), first step
    toward larger r."""
    cand = np.nonzero(np.abs(r - start_r) < 0.003)[0]
    i0 = int(cand[np.argmax(pts[cand, 2])])
    nx0 = sorted(adj[i0], key=lambda q: -r[q])
    order, seen, prev, cur = [i0], {i0}, -1, i0
    if nx0:
        prev, cur = i0, nx0[0]
        order.append(cur)
        seen.add(cur)
    while len(order) < 20000:
        nx_ = [q for q in adj[cur] if q != prev and q not in seen]
        if not nx_:
            break
        prev, cur = cur, nx_[0]
        seen.add(cur)
        order.append(cur)
    return order


def eye_profile(sf, c, a, phi, step=0.0005):
    """SRC section in the half plane at azimuth phi about the eye axis, walked from the apex outward, resampled
    every `step`; returns (r, h) (m) and the 3D points."""
    x, y = _frame(a)
    sel = np.linalg.norm(sf.cen - c, axis=1) < 0.17
    Vr, Tr, _u = compact(sf.src.V, sf.src.T[sel])
    R = np.column_stack([x, y, a])
    L = (Vr - c) @ R
    pts, adj, e2 = section_chain(L, Tr, phi)
    order = _walk_chain(pts, adj, pts @ e2)
    Q = pts[order]
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])
    t = np.arange(0.0, s[-1], step)
    Qs = np.column_stack([np.interp(t, s, Q[:, k]) for k in range(3)])
    rh = np.column_stack([Qs @ e2, Qs[:, 2]])
    return rh, c + Qs @ R.T


def eye_features(rh, win=4):
    """indices on a resampled profile: dome edge (descent angle > 60 deg), wall bottom (< 30 deg after it),
    shoulder (> 45 deg, >= 5 mm after the wall bottom), equator (first > 90 deg after the shoulder = max r),
    junction (after the equator: first < 60 deg = the chain turns out onto the face; 'merged') or back (first
    > 160 deg = the chain runs inward along the lump back; 'detached')."""
    d = rh[win:] - rh[:-win]
    ang = np.degrees(np.arctan2(-d[:, 1], d[:, 0]))
    ang = np.concatenate([ang, np.full(win, ang[-1])])

    def first(cond, start):
        idx = np.nonzero(cond[start:])[0]
        return int(idx[0]) + start if len(idx) else None
    i_de = first(ang > 60, 0)
    i_wb = first(ang < 30, i_de)
    i_sh = first(ang > 45, i_wb + 10)
    i_eq = first(ang > 90, i_sh)
    i_j = first(ang < 60, i_eq + 4)
    i_b = first(ang > 160, i_eq + 4)
    if None in (i_de, i_wb, i_sh, i_eq) or (i_j is None and i_b is None):
        raise RuntimeError(f"eye profile features not found: {i_de} {i_wb} {i_sh} {i_eq} {i_j} {i_b}")
    kind = "merged" if i_b is None or (i_j is not None and i_j < i_b) else "detached"
    return {"de": i_de, "wb": i_wb, "sh": i_sh, "eq": i_eq, "end": i_j if kind == "merged" else i_b, "kind": kind}


def _coons(loop, n):
    """n x n grid inside a closed loop of 4 n points (sides = consecutive n-segment runs). Returns (n+1, n+1, 3)."""
    L = np.asarray(loop)
    G = np.zeros((n + 1, n + 1, L.shape[1]))
    for i in range(n + 1):
        G[i, 0] = L[i]
        G[n, i] = L[n + i]
        G[n - i, n] = L[2 * n + i]
        G[0, n - i] = L[(3 * n + i) % (4 * n)]
    for i in range(1, n):
        for j in range(1, n):
            t, s = i / n, j / n
            G[i, j] = ((1 - t) * G[0, j] + t * G[n, j] + (1 - s) * G[i, 0] + s * G[i, n]
                       - ((1 - t) * (1 - s) * G[0, 0] + t * (1 - s) * G[n, 0] + t * s * G[n, n] + (1 - t) * s * G[0, n]))
    return G


def build_eye(sf, c, a, K):
    """Eye rings: K azimuths; per azimuth the SRC profile loops (outer -> inner) junction (face), undercut (arc
    mid equator..junction), equator, shoulder, plateau mid, wall bottom, dome edge; the dome edge loop is closed by
    an (K/4 x K/4) Coons grid projected onto SRC along the axis.  Detached azimuths (the lump back floats in
    front of the face): junction = the face below the lump back rim (ray along -a).
    Returns loops (list outer -> inner of (K,3)), cap grid (n+1, n+1, 3), info."""
    x, y = _frame(a)
    names = ("junction", "undercut", "equator", "shoulder", "plateau", "wall_bottom", "dome_edge")
    loops = {k: [] for k in names}
    kinds = []
    for k in range(K):
        ph = 2 * math.pi * k / K
        rh, P3 = eye_profile(sf, c, a, ph)
        f = eye_features(rh)
        kinds.append(f["kind"])
        pj = P3[f["end"]]
        if f["kind"] == "detached":
            q, _n = _ray(sf.src.bvh, pj - 0.0005 * a, -a, 0.08)
            pj = q if q is not None else pj
        loops["junction"].append(pj)
        loops["undercut"].append(P3[(f["eq"] + f["end"]) // 2])
        loops["equator"].append(P3[f["eq"]])
        loops["shoulder"].append(P3[f["sh"]])
        loops["plateau"].append(P3[(f["wb"] + f["sh"]) // 2])
        loops["wall_bottom"].append(P3[f["wb"]])
        loops["dome_edge"].append(P3[f["de"]])
    loops = [np.array(loops[k]) for k in names]
    n = K // 4
    G = _coons(loops[-1], n)
    for i in range(1, n):
        for j in range(1, n):
            q, _n = _ray(sf.src.bvh, G[i, j] + 0.1 * a, -a, 0.3)
            if q is not None:
                G[i, j] = q
    jr = np.linalg.norm((loops[0] - c) - np.outer((loops[0] - c) @ a, a), axis=1)
    return loops, G, {"loops": names, "detached_azimuths": int(sum(k_ == "detached" for k_ in kinds)),
                      "junction_r_mm": [mm(jr.min()), mm(jr.max())]}


def fang_geometry(src, tri_xz):
    """Fang = flat plate (plane y = plate_y) with near-vertical walls.  Top outline: per seed edge the plate verts
    (y within 0.4 mm of the plate) within 4 mm of that edge line give a line (PCA) moved out to their 98th percentile;
    corners = line intersections.  Foot outline: each top edge moved out by the wall foot distance (max over 5
    samples along the edge of the first outward offset where a +y ray lands within 1 mm of the face level found
    FANG_FOOT_MAX out).  Returns top / foot triangles (x, z) (corners base_l, base_r, apex) and plate_y."""
    V = src.V
    tri = np.asarray(tri_xz, float)
    cxz = tri.mean(0)
    near = (np.abs(V[:, 0] - cxz[0]) < 0.08) & (np.abs(V[:, 2] - cxz[1]) < 0.08) & (V[:, 1] < -0.2)
    ins = near & (_tri_dist2d(V[:, [0, 2]], tri) < 0.003)
    plate_y = float(np.percentile(V[ins, 1], 2))
    pl = ins & (V[:, 1] < plate_y + 0.0004)
    Q = V[pl][:, [0, 2]]
    lines = []
    for i in range(3):
        p0, p1 = tri[i], tri[(i + 1) % 3]
        d = unit(p1 - p0)
        nrm = np.array([d[1], -d[0]])
        if (cxz - p0) @ nrm > 0:
            nrm = -nrm
        dist = (Q - p0) @ nrm
        sel = np.abs(dist) < 0.004
        q = Q[sel]
        mu = q.mean(0)
        dd = np.linalg.svd(q - mu)[2][0]
        n2 = np.array([dd[1], -dd[0]])
        if n2 @ nrm < 0:
            n2 = -n2
        off = float(np.percentile((q - mu) @ n2, 98))
        lines.append((mu + off * n2, dd, n2))

    def corners(ls):
        out = []
        for i in range(3):
            (p, d, _n), (q, e, _m) = ls[i - 1], ls[i]
            A = np.column_stack([d, -e])
            t = np.linalg.solve(A, q - p)
            out.append(p + t[0] * d)
        return np.array(out)
    top = corners(lines)
    foots = []
    for i, (p, d, n2) in enumerate(lines):
        a_, b_ = top[i], top[(i + 1) % 3]
        best = 0.0
        for t in np.linspace(0.2, 0.8, 5):
            q = a_ + t * (b_ - a_)
            y1 = _ray(src.bvh, (q[0] + FANG_FOOT_MAX * n2[0], -1.0, q[1] + FANG_FOOT_MAX * n2[1]), (0, 1, 0), 2.0)[0][1]
            y2 = _ray(src.bvh, (q[0] + 0.75 * FANG_FOOT_MAX * n2[0], -1.0, q[1] + 0.75 * FANG_FOOT_MAX * n2[1]),
                      (0, 1, 0), 2.0)[0][1]
            for off in np.arange(0.0, FANG_FOOT_MAX, 0.00025):
                hy = _ray(src.bvh, (q[0] + off * n2[0], -1.0, q[1] + off * n2[1]), (0, 1, 0), 2.0)[0][1]
                yface = y1 + (y1 - y2) / (0.25 * FANG_FOOT_MAX) * (off - FANG_FOOT_MAX)
                if hy > yface - 0.0007:
                    best = max(best, off)
                    break
        foots.append(best)
    foot = corners([(p + f_ * n2, d, n2) for (p, d, n2), f_ in zip(lines, foots)])
    return top, foot, plate_y, foots


def _tri_dist2d(Q, tri):
    a, b, c = tri

    def seg(p, q):
        d = q - p
        t = np.clip(((Q - p) @ d) / (d @ d), 0, 1)
        return np.linalg.norm(Q - (p + t[:, None] * d), axis=1)

    def cr(p, q):
        return (q[0] - p[0]) * (Q[:, 1] - p[1]) - (q[1] - p[1]) * (Q[:, 0] - p[0])
    s1, s2, s3 = cr(a, b), cr(b, c), cr(c, a)
    ins = ((s1 >= 0) & (s2 >= 0) & (s3 >= 0)) | ((s1 <= 0) & (s2 <= 0) & (s3 <= 0))
    d = np.minimum(np.minimum(seg(a, b), seg(b, c)), seg(c, a))
    d[ins] = 0.0
    return d


class MLSRef:
    """Moving least squares quadric surface on oriented samples (radius R, Gaussian weights exp(-(d / (R/2))^2))."""

    def __init__(self, P, N):
        self.P, self.N = np.asarray(P, float), np.asarray(N, float)
        self.kd = KDTree(len(self.P))
        for i, p in enumerate(self.P):
            self.kd.insert(Vector(p), i)
        self.kd.balance()

    def project(self, q, R):
        for rr in (R, 2 * R, 3 * R):
            res = self.kd.find_range(Vector(q), rr)
            if len(res) >= HEAD_MLS_MIN:
                break
        else:
            return None
        idx = np.fromiter((r_[1] for r_ in res), np.int64, len(res))
        d = np.fromiter((r_[2] for r_ in res), float, len(res))
        w = np.exp(-(d / (0.5 * rr)) ** 2)
        n = unit((self.N[idx] * w[:, None]).sum(0))
        u = unit(np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0]))
        v = np.cross(n, u)
        D = self.P[idx] - q
        a, b, h = D @ u, D @ v, D @ n
        A = np.stack([np.ones_like(a), a, b, a * a, a * b, b * b], 1)
        sw = np.sqrt(w)
        cf, *_r = np.linalg.lstsq(A * sw[:, None], h * sw, rcond=None)
        return q + cf[0] * n


def _box_lattice(nx, ny, nz):
    """box surface lattice: vertex keys (i, j, k) on the surface of [0,nx]x[0,ny]x[0,nz]; quads per box face
    (winding fixed later)."""
    idx, keys, Q = {}, [], []

    def vid(i, j, k):
        key = (i, j, k)
        if key not in idx:
            idx[key] = len(keys)
            keys.append(key)
        return idx[key]
    for i in (0, nx):
        for j in range(ny):
            for k in range(nz):
                Q.append((("x", i), (i, j, k), [vid(i, j, k), vid(i, j + 1, k), vid(i, j + 1, k + 1), vid(i, j, k + 1)]))
    for j in (0, ny):
        for i in range(nx):
            for k in range(nz):
                Q.append((("y", j), (i, j, k), [vid(i, j, k), vid(i + 1, j, k), vid(i + 1, j, k + 1), vid(i, j, k + 1)]))
    for k in (0, nz):
        for i in range(nx):
            for j in range(ny):
                Q.append((("z", k), (i, j, k), [vid(i, j, k), vid(i + 1, j, k), vid(i + 1, j + 1, k), vid(i, j + 1, k)]))
    return idx, keys, Q


def _perimeter(nx, ny):
    """(i, j) of the box side perimeter, counter-clockwise seen from +z, starting at (0, 0)."""
    out = [(i, 0) for i in range(nx)] + [(nx, j) for j in range(ny)] + [(i, ny) for i in range(nx, 0, -1)] \
        + [(0, j) for j in range(ny, 0, -1)]
    return out


def _poly_margin(poly, q):
    """signed distance of the 2D point q to the closed polygon boundary (+ inside)."""
    d = min(np.linalg.norm(q - (a + np.clip((q - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1) * (b - a)))
            for a, b in zip(poly, np.roll(poly, -1, axis=0)))
    ins = False
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        if (a[1] > q[1]) != (b[1] > q[1]) and q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            ins = not ins
    return d if ins else -d


def _crease_rows(src, rim, S):
    """per rim point: the SRC meridian section (half plane through the world z axis at its azimuth) walked up
    from the rim; points at arc lengths S."""
    cen = src.V[src.T].mean(1)
    reg = (cen[:, 2] > 0.88) & (np.hypot(cen[:, 0], cen[:, 1]) < 0.42)
    Vr, Tr, _u = compact(src.V, src.T[reg])
    out = np.zeros((len(S), len(rim), 3))
    for t, q in enumerate(rim):
        th = math.atan2(q[1], q[0])
        pts, adj, _e = section_chain(Vr, Tr, th)
        ic = int(np.argmin(np.linalg.norm(pts - q, axis=1)))
        best = None
        for j1 in adj[ic]:
            poly, acc = walk(pts, adj, ic, j1, max(S) + 0.02)
            if best is None or poly[-1][2] > best[0][-1][2]:
                best = (poly, acc)
        for k, s_ in enumerate(S):
            out[k, t] = at_len(best[0], best[1], s_)
    return out


def _rim_loop(P, T, hid_T):
    """ordered crease rim (edges between visible and hidden triangles), counter-clockwise seen from +z."""
    cnt = {}
    for t, h in zip(T.tolist(), hid_T.tolist()):
        for k in range(3):
            e = (min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3]))
            cnt.setdefault(e, set()).add(h)
    edges = [e for e, s in cnt.items() if s == {True, False}]
    adj = {}
    for a_, b_ in edges:
        adj.setdefault(a_, []).append(b_)
        adj.setdefault(b_, []).append(a_)
    start = edges[0][0]
    order, prev, cur = [start], None, start
    while True:
        nx_ = [q for q in adj[cur] if q != prev]
        if not nx_ or nx_[0] == start:
            break
        prev, cur = cur, nx_[0]
        order.append(cur)
    R = P[order]
    cen = R.mean(0)
    ang = np.unwrap(np.arctan2(R[:, 1] - cen[1], R[:, 0] - cen[0]))
    if ang[-1] < ang[0]:
        R = R[::-1]
    return R


def _resample_closed(R, starts, counts):
    """points on the closed polyline R: between consecutive start params (arc length) `counts` equal steps."""
    Rc = np.vstack([R, R[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Rc, axis=0), axis=1))])
    Ltot = s[-1]
    out = []
    for q in range(len(starts)):
        s0, s1 = starts[q], starts[(q + 1) % len(starts)]
        if s1 <= s0:
            s1 += Ltot
        for t in range(counts[q]):
            ss = (s0 + (s1 - s0) * t / counts[q]) % Ltot
            out.append([np.interp(ss, s, Rc[:, k]) for k in range(3)])
    return np.array(out)


def build_head_quads(src, P, T, meta):
    """T113 / HR1 even-quad head on the closed dense head (P, T) of build_head (see the module docstring).
    Returns V, polygons (quads + the hidden cap fan), extra (masks, eye / fang geometry, info)."""
    onsrc = np.array([src.dist(p) for p in P]) < 0.0001
    hid_T = ~onsrc[T].all(1)
    bvh_c = BVHTree.FromPolygons(P.tolist(), T.tolist(), all_triangles=True)
    sf = _SrcFaces(src)
    info = {}
    # ---- features
    seeds = head_seeds(P, T, onsrc)
    K = 4 * EYE_BLOCK
    eyes, fangs = {}, {}
    for side in ("l", "r"):
        c, a, fi = eye_frame(sf, seeds[side]["eye_xz"])
        loops, G, ei = build_eye(sf, c, a, K)
        eyes[side] = {"c": c, "a": a, "loops": loops, "cap": G}
        top, foot, plate_y, foots = fang_geometry(src, seeds[side]["fang_xz"])
        fangs[side] = {"top": top, "foot": foot, "plate_y": plate_y}
        info[f"eye_{side}"] = {"centre": np.round(c, 5).tolist(), "axis": np.round(a, 4).tolist(),
                               "wall_r_mm": mm(fi["wall_r"]), "plateau_rms_mm": mm(fi["plateau_rms"]), **ei}
        info[f"fang_{side}"] = {"plate_y_mm": mm(plate_y), "foot_mm": [mm(f_) for f_ in foots],
                                "top_xz": np.round(top, 4).tolist(), "foot_xz": np.round(foot, 4).tolist()}
    # ---- smooth face reference (MLS) without the eye / fang footprints
    vis_T = T[~hid_T]
    rng = np.random.default_rng(3)
    sp, sti = sample_surface(P, vis_T, 2 * HEAD_MLS_N, rng)
    sn = _tri_normals(P, vis_T)[0][sti]
    keep = np.ones(len(sp), bool)
    for side in ("l", "r"):
        e = eyes[side]
        D = sp - e["c"]
        r = np.linalg.norm(D - np.outer(D @ e["a"], e["a"]), axis=1)
        jr = np.linalg.norm((e["loops"][0] - e["c"]) - np.outer((e["loops"][0] - e["c"]) @ e["a"], e["a"]), axis=1).max()
        keep &= ~((r < jr + EYE_EXCL) & (D @ e["a"] > -0.12))
        keep &= ~((_tri_dist2d(sp[:, [0, 2]], fangs[side]["foot"]) < FANG_EXCL) & (sp[:, 1] < -0.1))
    ref = MLSRef(sp[keep][:HEAD_MLS_N], sn[keep][:HEAD_MLS_N])
    info["mls_samples"] = int(min(keep.sum(), HEAD_MLS_N))
    # ---- lattice
    lo, hi = P.min(0), P.max(0)
    cen = (lo + hi) / 2
    W, D_, H = hi - lo
    rimP = _rim_loop(P, T, hid_T)
    zr = float(rimP[:, 2].mean())
    e_ = math.sqrt(2 * (W * D_ + D_ * H + W * H) / (HEAD_LATTICE_TRIS / 2))
    nx, ny = max(4, round(W / e_)), max(4, round(D_ / e_))
    per = _perimeter(nx, ny)
    # rim: corners of the box perimeter matched to the rim azimuth, equal arc length between them
    Rc = np.vstack([rimP, rimP[:1]])
    s_r = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Rc, axis=0), axis=1))])
    az_r = np.arctan2(rimP[:, 1] - cen[1], rimP[:, 0] - cen[0])
    starts = []
    for (ci, cj) in ((0, 0), (nx, 0), (nx, ny), (0, ny)):
        az = math.atan2((cj / ny - 0.5) * D_, (ci / nx - 0.5) * W)
        starts.append(float(s_r[int(np.argmin(np.abs(np.angle(np.exp(1j * (az_r - az))))))]))
    rim = _resample_closed(rimP, starts, [nx, ny, nx, ny])
    strip = []
    for q in rim:
        loc, nrm, _i, _d = src.bvh.find_nearest(Vector((q[0], q[1], q[2] - HEAD_STRIP_DROP)))
        strip.append(np.array(loc) - HEAD_STRIP_IN * np.array(nrm).clip(-1, 1))
    strip = np.array(strip)
    z_skirt = meta["z_skirt"]
    ring = []
    for q in strip:
        rr = math.hypot(q[0] - cen[0], q[1] - cen[1])
        sc = (rr - HEAD_SKIRT_IN) / rr
        ring.append((cen[0] + (q[0] - cen[0]) * sc, cen[1] + (q[1] - cen[1]) * sc, z_skirt))
    ring = np.array(ring)
    crease = _crease_rows(src, rim, HEAD_CREASE_S)
    nkp = 2 + len(HEAD_CREASE_S)          # rows 0..nkp pinned: ring, strip, rim, crease rows
    nzv = len(HEAD_CREASE_S) + max(3, round((hi[2] - float(crease[-1][:, 2].mean())) / e_))
    nz = nzv + 2
    idx, keys, Q = _box_lattice(nx, ny, nz)
    keys = np.array(keys)
    X = np.zeros((len(keys), 3))
    role = np.zeros(len(keys), np.int8)   # 0 free, 1 fixed (ring / strip / rim), 2 cap interior (dropped), 3 crease rows
    pidx = {p: t for t, p in enumerate(per)}
    for v, (i, j, k) in enumerate(keys.tolist()):
        side_v = (i in (0, nx)) or (j in (0, ny))
        if k == 0 and not side_v:
            role[v] = 2
            continue
        if side_v and k <= nkp:
            t = pidx[(i, j)]
            X[v] = (ring, strip, rim, *crease)[k][t]
            role[v] = 1 if k <= 2 else 3
            continue
        zc = float(crease[-1][:, 2].mean())
        b = np.array([lo[0] + i / nx * W, lo[1] + j / ny * D_, zc + (k - nkp) / (nz - nkp) * (hi[2] - zc)])
        d = unit(b - cen)
        q, _n = _ray(bvh_c, cen, d, 2.0)
        X[v] = q if q is not None else cen + d * 0.2
    faces_l = [(tag, cell, q) for tag, cell, q in Q if tag != ("z", 0)]
    free = role == 0
    nb = [set() for _ in range(len(keys))]
    for _t, _c, q in faces_l:
        for a_ in range(4):
            nb[q[a_]] |= {q[(a_ + 1) % 4], q[(a_ + 3) % 4]}

    def relax_project(X, free, nb, iters):
        nbl = [np.array(sorted(s)) for s in nb]
        fr = np.nonzero(free)[0]
        for _ in range(iters):
            N = np.zeros_like(X)
            for tag, cell, q in faces_l_cur[0]:
                p_ = X[list(q)]
                n_ = np.cross(p_[2] - p_[0], p_[3] - p_[1]) if len(q) == 4 else np.cross(p_[1] - p_[0], p_[2] - p_[0])
                for v in q:
                    N[v] += n_
            N /= np.maximum(np.linalg.norm(N, axis=1), 1e-20)[:, None]
            Y = X.copy()
            for v in fr:
                dv = X[nbl[v]].mean(0) - X[v]
                dv -= (dv @ N[v]) * N[v]
                q = ref.project(X[v] + HEAD_RELAX * dv, HEAD_MLS_R)
                Y[v] = q if q is not None else X[v] + HEAD_RELAX * dv
            X = Y
        return X
    faces_l_cur = [faces_l]
    fr0 = free.copy()
    for v in np.nonzero(fr0)[0]:
        q = ref.project(X[v], HEAD_MLS_R)
        if q is not None:
            X[v] = q
    X = relax_project(X, free, nb, HEAD_ITERS)
    # ---- holes on the front face (j = 0): eye block (EYE_BLOCK^2 cells) and fang block below it
    blocks = {}
    for side in ("l", "r"):
        ec = eyes[side]["c"]
        best = None
        for i0 in range(0, nx - EYE_BLOCK + 1):
            for k0 in range(nkp + FANG_BLOCK[1], nz - EYE_BLOCK + 1):
                vc = idx[(i0 + EYE_BLOCK // 2, 0, k0 + EYE_BLOCK // 2)]
                dd = np.linalg.norm(X[vc][[0, 2]] - ec[[0, 2]])
                if best is None or dd < best[0]:
                    best = (dd, i0, k0)
        _d, i0, k0 = best
        fcols, frows = FANG_BLOCK
        best = None
        for i1 in range(max(0, i0 - fcols + 1), min(nx - fcols, i0 + EYE_BLOCK - 1) + 1):
            bk = k0 - frows
            ring_ = [(i, bk) for i in range(i1, i1 + fcols)] + [(i1 + fcols, k) for k in range(bk, bk + frows)] \
                + [(i, bk + frows) for i in range(i1 + fcols, i1, -1)] + [(i1, k) for k in range(bk + frows, bk, -1)]
            poly = np.array([X[idx[(i, 0, k)]][[0, 2]] for i, k in ring_])
            marg = min(_poly_margin(poly, q) for q in fangs[side]["foot"])
            if best is None or marg > best[0]:
                best = (marg, i1)
        i1 = best[1]
        info[f"fang_block_margin_{side}_mm"] = mm(best[0])
        if k0 - frows < nkp:
            raise RuntimeError(f"fang block {side}: rows {k0 - frows}.. reach the rim rows")
        blocks[side] = {"eye": (i0, k0, EYE_BLOCK, EYE_BLOCK), "fang": (i1, k0 - frows, fcols, frows)}
    info["blocks"] = {s_: {k_: list(v_) for k_, v_ in b_.items()} for s_, b_ in blocks.items()}
    cells_out = set()
    for b_ in blocks.values():
        for (bi, bk, bw, bh) in b_.values():
            for i in range(bi, bi + bw):
                for k in range(bk, bk + bh):
                    if (i, k) in cells_out:
                        raise RuntimeError("head: eye / fang blocks overlap")
                    cells_out.add((i, k))
    faces_l = [(tag, cell, q) for tag, cell, q in faces_l if not (tag == ("y", 0) and (cell[0], cell[2]) in cells_out)]

    def block_loop(bi, bk, bw, bh):
        """boundary vertex ids of a front block, counter-clockwise in (x, z) seen from the front."""
        pts = [(i, bk) for i in range(bi, bi + bw)] + [(bi + bw, k) for k in range(bk, bk + bh)] \
            + [(i, bk + bh) for i in range(bi + bw, bi, -1)] + [(bi, k) for k in range(bk + bh, bk, -1)]
        return [idx[(i, 0, k)] for i, k in pts]
    # ---- assemble: lattice verts + feature verts
    Vl = [X[v] for v in range(len(keys))]
    faces = [q for _t, _c, q in faces_l]
    ring_ids = [idx[(i, j, 0)] for (i, j) in per]
    feat = set()
    fang_set = set()
    hard = []                                 # designed hard edges (vertex id pairs, pre-compaction)

    def ring_pairs(ids):
        return [(ids[t], ids[(t + 1) % len(ids)]) for t in range(len(ids))]
    hard += ring_pairs([idx[(i, j, 2)] for (i, j) in per])   # crease rim: visible shell | hidden strip (T113d)

    def add(p):
        Vl.append(np.asarray(p, float))
        return len(Vl) - 1
    n_des_tri = {"cap_fan": 0}
    for side in ("l", "r"):
        e = eyes[side]
        L = [[add(p) for p in lp] for lp in e["loops"]]
        hard += ring_pairs(L[0]) + ring_pairs(L[5])          # ring outer edge (junction), eyeball base (wall bottom)
        for lp in L:
            feat.update(lp)
        H_ = block_loop(*blocks[side]["eye"])
        PA = np.array([Vl[v] for v in H_])
        PB = np.array([Vl[v] for v in L[0]])
        off = min(range(K), key=lambda o: sum(np.linalg.norm(PA[t] - PB[(t + o) % K]) for t in range(K)))
        for t in range(K):
            faces.append((H_[t], H_[(t + 1) % K], L[0][(t + 1 + off) % K], L[0][(t + off) % K]))
        for m in range(len(L) - 1):
            for t in range(K):
                faces.append((L[m][t], L[m][(t + 1) % K], L[m + 1][(t + 1) % K], L[m + 1][t]))
        n = K // 4
        G = e["cap"]
        gid = np.zeros((n + 1, n + 1), np.int64)
        inner = L[-1]
        for i in range(n + 1):
            gid[i, 0] = inner[i]
            gid[n, i] = inner[n + i]
            gid[n - i, n] = inner[2 * n + i]
            gid[0, n - i] = inner[(3 * n + i) % (4 * n)]
        for i in range(1, n):
            for j in range(1, n):
                gid[i, j] = add(G[i, j])
                feat.add(int(gid[i, j]))
        for i in range(n):
            for j in range(n):
                faces.append((int(gid[i, j]), int(gid[i + 1, j]), int(gid[i + 1, j + 1]), int(gid[i, j + 1])))
        # fang: foot loop and top loop (12 verts: corners + 3 per edge), top = 3 patches of 2 x 2 quads
        f = fangs[side]
        py = f["plate_y"]
        fpts, tpts = [], []
        for tri, lst in ((f["foot"], fpts), (f["top"], tpts)):
            for i in range(3):
                for t in range(4):
                    lst.append(tri[i] + t / 4 * (tri[(i + 1) % 3] - tri[i]))
        foot_ids = []
        for q in fpts:
            h_, _n = _ray(src.bvh, (q[0], -1.0, q[1]), (0.0, 1.0, 0.0), 2.0)
            foot_ids.append(add(h_))
        top_ids = [add((q[0], py, q[1])) for q in tpts]
        O = f["top"].mean(0)
        oid = add((O[0], py, O[1]))
        spoke = {}
        for i in range(3):
            m = tpts[4 * i + 2]
            spoke[i] = add((0.5 * (m[0] + O[0]), py, 0.5 * (m[1] + O[1])))
        new_top = [oid] + list(spoke.values())
        for i in range(3):
            c0, a1, m01, m20, a11 = 4 * i, 4 * i + 1, 4 * i + 2, (4 * i + 10) % 12, (4 * i + 11) % 12
            sp_a, sp_c = spoke[i], spoke[(i + 2) % 3]
            pc = 0.25 * (np.array(Vl[top_ids[c0]]) + np.array(Vl[top_ids[m01]]) + np.array(Vl[oid]) + np.array(Vl[top_ids[m20]]))
            gc = add(pc)
            new_top.append(gc)
            G = [[top_ids[c0], top_ids[a11], top_ids[m20]], [top_ids[a1], gc, sp_c], [top_ids[m01], sp_a, oid]]
            for u in range(2):
                for w_ in range(2):
                    faces.append((G[u][w_], G[u + 1][w_], G[u + 1][w_ + 1], G[u][w_ + 1]))
        feat.update(foot_ids + top_ids + new_top)
        fang_set.update(foot_ids + top_ids + new_top)
        FB = block_loop(*blocks[side]["fang"])
        nF = len(FB)
        if nF != len(foot_ids):
            raise RuntimeError(f"fang {side}: block boundary {nF} verts != fang loop {len(foot_ids)}")
        PA = np.array([Vl[v] for v in FB])
        PB = np.array([Vl[v] for v in foot_ids])
        off = min(range(nF), key=lambda o: sum(np.linalg.norm(PA[t] - PB[(t + o) % nF]) for t in range(nF)))
        for t in range(nF):
            faces.append((FB[t], FB[(t + 1) % nF], foot_ids[(t + 1 + off) % nF], foot_ids[(t + off) % nF]))
        for t in range(12):
            faces.append((foot_ids[t], foot_ids[(t + 1) % 12], top_ids[(t + 1) % 12], top_ids[t]))
        hard += ring_pairs(foot_ids) + ring_pairs(top_ids) + [(foot_ids[t], top_ids[t]) for t in (0, 4, 8)]
    # hidden cap: fan from the ring centre
    cc = add(np.array(Vl)[ring_ids].mean(0))
    for t in range(len(ring_ids)):
        faces.append((ring_ids[t], ring_ids[(t + 1) % len(ring_ids)], cc))
    n_des_tri["cap_fan"] = len(ring_ids)
    Vl = np.array(Vl)
    # compact (drop unused lattice verts: cap interior, block interiors)
    used = sorted({v for f_ in faces for v in f_})
    remap = {v: t for t, v in enumerate(used)}
    Vc = Vl[used]
    faces = [tuple(remap[v] for v in f_) for f_ in faces]
    hard = [(remap[a_], remap[b_]) for a_, b_ in hard]
    free_c = np.array([(v < len(keys) and role[v] == 0) for v in used])
    feat_c = np.array([v in feat for v in used])
    fang_c = np.array([v in fang_set for v in used])
    pinned_c = np.array([(v < len(keys) and role[v] in (1, 3)) or v == cc for v in used])
    crease_c = np.array([v < len(keys) and role[v] == 3 for v in used])
    # ---- final relax of the free lattice verts (hole boundaries now tied to the features)
    nb2 = [set() for _ in range(len(Vc))]
    for q in faces:
        for a_ in range(len(q)):
            nb2[q[a_]] |= {q[(a_ + 1) % len(q)], q[(a_ - 1) % len(q)]}
    faces_l_cur[0] = [(None, None, q) for q in faces]
    Vc = relax_project(Vc, free_c, nb2, HEAD_ITERS)
    info["lattice"] = {"segments": [nx, ny, nz], "visible_rows": nzv, "edge_target_mm": mm(e_)}
    info["designed_triangles"] = n_des_tri
    Vc, faces, st = finish_head(Vc, faces, free_c | crease_c | (feat_c & ~fang_c), bvh_c, hid_T)
    info["straddle"] = st
    return Vc, faces, {"free": free_c, "feat": feat_c, "fang": fang_c, "pinned": pinned_c, "bvh_c": bvh_c, "hid_T": hid_T,
                       "eyes": eyes, "fangs": fangs, "info": info, "hard_edges": hard}


def _hull2d(Q):
    """convex hull (counter-clockwise) of 2D points (monotone chain)."""
    Q = sorted(set(map(tuple, np.round(np.asarray(Q, float), 7).tolist())))
    if len(Q) < 3:
        return np.array(Q)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in Q:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(Q):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def _in_poly(poly, q):
    ins = False
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        if (a[1] > q[1]) != (b[1] > q[1]) and q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            ins = not ins
    return ins


def head_design_zones(src, Vh, hx):
    """T120a (hr2_contract, user decision 2026-09-26): per eye, the area where the SRC_hi eye lump is detached from
    the face.  Rays along -axis on a ZONE_STEP grid of the eye plane inside the junction radius (junction loop radius
    interpolated over azimuth); a cell is detached where the ray has >= 4 SRC hits (lump front, lump back, face, head
    back), the lump back lies > ZONE_DEPTH_MIN below the apex and the lump back -> face distance is > ZONE_GAP_MIN.
    Zone: per contiguous run of ZONE_BIN azimuth bins holding detached cells, the sector from the centre to
    max(equator radius, detached cell radius) + ZONE_MARGIN (covers the new head's filling wall), projected to the
    face (x, z) plane as the convex hull of its points at the apex plane and 100 mm behind it; y range = SRC hits of
    the detached cells and the head verts inside the polygon within ZONE_HEAD_DEPTH below the apex, +- ZONE_Y_MARGIN.
    Depth rule (T120a-2): a point inside the polygon / y range is excluded only if it lies more than
    exclude_depth_min_m below the local front (first SRC_hi hit of a ray along -axis through its eye-plane position),
    so the eye front (eyeball, ring face, upper outer wall) stays judged; exclude_depth_min_m = the SRC lump thickness
    at its thinnest detached cell (min over the detached cells of the lump-back depth below the front) -
    ZONE_DEPTH_EPS, so the whole detached lump back is excluded.  Reported with it: the depth below the front of the
    SRC ring outer-wall triangles (eye-plane radius >= ZONE_WALL_R_MIN, |n . axis| < ZONE_WALL_NA, not deeper than
    their ray's first exit hit) and the share of them deeper than the limit (the lower side wall, then excluded).  Only eye_gap_xneg is a design-change zone (hr2_contract); eye_gap_xpos is for the record."""
    out = {}
    for side, key in (("l", "eye_gap_xneg"), ("r", "eye_gap_xpos")):
        e = hx["eyes"][side]
        c, a = e["c"], e["a"]
        x, y = _frame(a)

        def polar(L):
            D = L - c
            return np.mod(np.arctan2(D @ y, D @ x), 2 * math.pi), np.linalg.norm(D - np.outer(D @ a, a), axis=1)
        pj, rj = polar(e["loops"][0])
        pe, re_ = polar(e["loops"][2])
        oj, oe = np.argsort(pj), np.argsort(pe)

        def r_at(ph, P_, R_, o_):
            return np.interp(ph, np.concatenate([P_[o_] - 2 * math.pi, P_[o_], P_[o_] + 2 * math.pi]), np.tile(R_[o_], 3))
        rmax = float(rj.max())
        cells, gaps, pts, backs = [], [], [], []
        for u in np.arange(-rmax, rmax + 1e-9, ZONE_STEP):
            for v in np.arange(-rmax, rmax + 1e-9, ZONE_STEP):
                r = math.hypot(u, v)
                ph = math.atan2(v, u) % (2 * math.pi)
                if r >= r_at(ph, pj, rj, oj):
                    continue
                o = Vector(c + u * x + v * y + 0.15 * a)
                hits = []
                for _ in range(8):
                    loc, _n, _i, _d = src.bvh.ray_cast(o, Vector(-a), 2.0)
                    if loc is None:
                        break
                    hits.append(np.array(loc))
                    o = loc - Vector(a) * 1e-5
                if len(hits) < 4:
                    continue
                d1, d2 = float((hits[1] - c) @ a), float((hits[2] - c) @ a)
                gap = d1 - d2
                if gap > ZONE_GAP_MIN and d1 < -ZONE_DEPTH_MIN and gap < 0.1:
                    cells.append((ph, r))
                    gaps.append(gap)
                    backs.append(float((hits[0] - c) @ a) - d1)
                    pts += hits[:3]
        z = {"_basis": "T113b / T113d measurement, user decision 2026-09-26 (hr2_contract): SRC_hi eye lump detached "
                       "from the face; G2.11 both directions and G2.12 excluded inside, values reported",
             "rule": " ".join(ln.strip() for ln in head_design_zones.__doc__.splitlines()[1:]).strip(),
             "use": "exclude (G2.11 both directions, G2.12)" if key == "eye_gap_xneg" else
                    "record only (not a design-change zone; hr2_contract names eye_gap_xneg only)",
             "eye_centre": np.round(c, 5).tolist(), "eye_axis": np.round(a, 5).tolist(),
             "cells": len(cells), "polygons_xz": [], "y_range": None}
        if cells:
            ph_c = np.array([p for p, _r in cells])
            r_c = np.array([r for _p, r in cells])
            nb = int(round(360 / ZONE_BIN))
            b_c = (np.degrees(ph_c) // ZONE_BIN).astype(int) % nb
            occ = np.zeros(nb, bool)
            occ[b_c] = True
            runs, start = [], None
            k0 = int(np.argmin(occ)) if not occ.all() else 0
            for t in range(nb + 1):
                k = (k0 + t) % nb
                if occ[k] and start is None:
                    start = k
                if (not occ[k] or t == nb) and start is not None:
                    runs.append((start, (k - 1) % nb))
                    start = None
            polys, cover = [], []
            for b0, b1 in runs:
                nbins = (b1 - b0) % nb + 1
                phs = np.radians(np.linspace(b0 * ZONE_BIN, (b0 + nbins) * ZONE_BIN, 4 * nbins + 1))
                sel = np.isin(b_c, [(b0 + t) % nb for t in range(nbins)])
                r_cells = float(r_c[sel].max())
                Q = []
                for d in (0.0, -0.10):
                    Q.append((c + d * a)[[0, 2]])
                    for ph in phs:
                        ro = max(float(r_at(ph % (2 * math.pi), pe, re_, oe)), r_cells) + ZONE_MARGIN
                        q = c + d * a + ro * (math.cos(ph) * x + math.sin(ph) * y)
                        Q.append(q[[0, 2]])
                poly = _hull2d(Q)
                polys.append(poly)
                cover.append({"azimuth_deg": [round(b0 * ZONE_BIN, 1), round(((b0 + nbins) * ZONE_BIN) % 360, 1)],
                              "span_deg": nbins * ZONE_BIN, "cells": int(sel.sum())})
            P_ = np.array(pts)
            inside_h = np.array([((q - c) @ a > -ZONE_HEAD_DEPTH) and any(_in_poly(pl, q[[0, 2]]) for pl in polys)
                                 for q in Vh])
            ys = np.concatenate([P_[:, 1], Vh[inside_h, 1]])
            z.update({"polygons_xz": [np.round(pl, 5).tolist() for pl in polys],
                      "y_range": [round(float(ys.min()) - ZONE_Y_MARGIN, 5), round(float(ys.max()) + ZONE_Y_MARGIN, 5)],
                      "max_gap_mm": mm(max(gaps)), "mean_gap_mm": mm(np.mean(gaps)),
                      "area_mm2": round(len(cells) * ZONE_STEP * ZONE_STEP * 1e6, 1), "runs": cover,
                      "azimuth_convention": "eye plane about the eye axis; 0 deg = eye frame x (horizontal, "
                                            "cross(+Z, axis)), 90 deg = eye frame y (up)",
                      "src_hits_inside": f"{sum(any(_in_poly(pl, q[[0, 2]]) for pl in polys) for q in P_)}/{len(P_)}",
                      "head_verts_inside": int(inside_h.sum())})

        def front_depths(q):
            """depths of the first hit (front) and the second hit (first exit) along -axis at q's eye-plane position."""
            uv = (q - c) - ((q - c) @ a) * a
            o, out_ = Vector(c + uv + 0.15 * a), []
            for _ in range(2):
                loc, _n, _i, _d = src.bvh.ray_cast(o, Vector(-a), 2.0)
                if loc is None:
                    break
                out_.append(float((np.array(loc) - c) @ a))
                o = loc - Vector(a) * 1e-5
            return out_
        tc = src.V[src.T].mean(1)
        tn = np.cross(src.V[src.T[:, 1]] - src.V[src.T[:, 0]], src.V[src.T[:, 2]] - src.V[src.T[:, 0]])
        tn /= np.maximum(np.linalg.norm(tn, axis=1), 1e-20)[:, None]
        Dt = tc - c
        dd = Dt @ a
        uvv = Dt - np.outer(dd, a)
        rr = np.linalg.norm(uvv, axis=1)
        pph = np.mod(np.arctan2(uvv @ y, uvv @ x), 2 * math.pi)
        wsel = (dd > -ZONE_HEAD_DEPTH) & (dd < 0.01) & (rr >= ZONE_WALL_R_MIN) & (np.abs(tn @ a) < ZONE_WALL_NA) \
            & (rr <= r_at(pph, pe, re_, oe) + ZONE_MARGIN)
        tw = []
        for i in np.nonzero(wsel)[0]:
            f_ = front_depths(tc[i])
            if len(f_) == 2 and dd[i] >= f_[1] - ZONE_DEPTH_EPS:
                tw.append(f_[0] - dd[i])
        tw = np.array(tw)
        bk = np.array(backs) if backs else np.zeros(0)
        dmin = (float(bk.min()) - ZONE_DEPTH_EPS) if len(bk) else None
        z["depth_rule"] = {"axis": np.round(a, 6).tolist(), "apex": np.round(c, 6).tolist(),
                           "exclude_depth_min_m": round(dmin, 5) if dmin is not None else None,
                           "local_front": "first SRC_hi hit of a ray along -axis through the point's eye-plane position "
                                          "(ray start: that position + 0.15 m along +axis from the apex plane)",
                           "exclude_if": "(x, z) inside polygons_xz and y inside y_range and (local_front_depth - "
                                         "point_depth) > exclude_depth_min_m, depths = (p - apex) . axis"}
        z["depth_basis"] = {"src_outer_wall_triangles": int(len(tw)),
                            "outer_wall_depth_below_front_mm": {"max": mm(tw.max()), "p99": mm(np.percentile(tw, 99)),
                                                                "median": mm(np.median(tw))},
                            "lump_back_depth_below_front_mm": ({"min": mm(bk.min()), "p5": mm(np.percentile(bk, 5)),
                                                                "median": mm(np.median(bk))} if len(bk) else None),
                            "detached_cells_back_excluded": (f"{int((bk > dmin).sum())}/{len(bk)}" if len(bk) else None),
                            "outer_wall_triangles_deeper_than_limit": (f"{int((tw > dmin).sum())}/{len(tw)}"
                                                                       if dmin is not None else None)}
        out[key] = z
    return out


def tag_hard_edges(ob, pairs):
    """EDGE bool attribute HEAD_HARD_ATTR = True on the edges between the given vertex pairs; returns the count."""
    me = ob.data
    E = np.empty(len(me.edges) * 2, np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    key = {(min(a_, b_), max(a_, b_)): i for i, (a_, b_) in enumerate(E.tolist())}
    flag = np.zeros(len(E), bool)
    for a_, b_ in pairs:
        flag[key[(min(a_, b_), max(a_, b_))]] = True
    att = me.attributes.new(HEAD_HARD_ATTR, "BOOLEAN", "EDGE")
    att.data.foreach_set("value", flag)
    return int(flag.sum())


def _poly_normals(V, faces):
    N = np.zeros((len(faces), 3))
    for t, f in enumerate(faces):
        p_ = V[list(f)]
        N[t] = np.cross(p_[2] - p_[0], p_[3] - p_[1]) if len(f) == 4 else np.cross(p_[1] - p_[0], p_[2] - p_[0])
    return N


def finish_head(V, faces, movable, bvh_c, hid_T):
    """consistent outward winding (bmesh recalc on the closed mesh, positive volume), then the straddle offset:
    each movable vertex moves along its normal by -0.5 x the mean signed distance (to the closed SRC head,
    + = outside) of the centres of its visible faces."""
    me = bpy.data.meshes.new("_s02c_headq")
    me.from_pydata(V.tolist(), [], [list(f) for f in faces])
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    faces = [tuple(p.vertices) for p in me.polygons]
    V = np.array([v.co[:] for v in me.vertices])
    bpy.data.meshes.remove(me)
    T = np.array([(f[0], f[k], f[k + 1]) for f in faces for k in range(1, len(f) - 1)])
    P_ = V[T] - V.mean(0)
    if np.einsum("ij,ij->i", P_[:, 0], np.cross(P_[:, 1], P_[:, 2])).sum() < 0:
        faces = [tuple(reversed(f)) for f in faces]
    FN = _poly_normals(V, faces)
    VN = np.zeros_like(V)
    for t, f in enumerate(faces):
        for v in f:
            VN[v] += FN[t]
    VN /= np.maximum(np.linalg.norm(VN, axis=1), 1e-20)[:, None]
    acc, cnt = np.zeros(len(V)), np.zeros(len(V))
    for t, f in enumerate(faces):
        q = V[list(f)].mean(0)
        loc, nrm, fi, _d = bvh_c.find_nearest(Vector(q))
        if loc is None or hid_T[fi]:
            continue
        sd = float((q - np.array(loc)) @ np.array(nrm))
        for v in f:
            acc[v] += sd
            cnt[v] += 1
    mv = movable & (cnt > 0)
    off = np.where(mv, -0.5 * acc / np.maximum(cnt, 1), 0.0)
    V = V + off[:, None] * VN
    tot = off.copy()
    for _ in range(HEAD_STRADDLE_PASSES - 1):
        acc, cnt = np.zeros(len(V)), np.zeros(len(V))
        for t, f in enumerate(faces):
            q = V[list(f)].mean(0)
            loc, nrm, fi, _d = bvh_c.find_nearest(Vector(q))
            if loc is None or hid_T[fi]:
                continue
            sd = float((q - np.array(loc)) @ np.array(nrm))
            for v in f:
                acc[v] += sd
                cnt[v] += 1
        off = np.where(mv & (cnt > 0), -0.5 * acc / np.maximum(cnt, 1), 0.0)
        V = V + off[:, None] * VN
        tot += off
    off = tot
    return V, faces, {"moved": int(mv.sum()), "mean_mm": mm(off[mv].mean()), "min_mm": mm(off[mv].min()),
                      "max_mm": mm(off[mv].max())}


# ---------------------------------------------------------------- limbs (hands / shoes)
def limb_frame(piv, joint):
    p = piv[joint]
    o = np.array(p["co"], float)
    a = unit(p["axis"])
    return o, a, float(p["radius"])


def clamp_hidden(P, o, a, r_tube, body, s_lo, s_hi=0.0, cover_src=None, cover_gap=None, cover_start=COVER_START):
    """Pull verts with s in [s_lo, s_hi) radially to r <= min(r_tube, r_body_ray) - HIDE_MARGIN.
    cover_src (all hands / shoes): for s > s_lo + COVER_START the limit is max(that, r_src_ray - COVER_IN), capped at
    r_src_ray - so where the SRC wrist flare stands outside the body tube the hand shell covers it
    (COVER_IN inside SRC) instead of hiding.  cover_gap (hand_l / shoes): the cover applies only where
    SRC - COVER_IN stands more than cover_gap outside the body tube; elsewhere the vertex stays hidden,
    so the shell cannot poke through the tube's flat 12-gon facets."""
    d = P - o
    s = d @ a
    radv = d - np.outer(s, a)
    r = np.linalg.norm(radv, axis=1)
    zone = np.nonzero((s >= s_lo - 1e-6) & (s < s_hi))[0]
    n_moved, lim_min = 0, float("inf")
    P = P.copy()
    for i in zone.tolist():
        if r[i] < 1e-9:
            continue
        u = radv[i] / r[i]
        c = o + s[i] * a
        hit, dd = body.ray(c, u, 0.3)
        rb = dd if hit is not None else r_tube - 0.003
        lim = min(r_tube, rb) - HIDE_MARGIN
        if cover_src is not None and s[i] > s_lo + cover_start:
            hs, ds = cover_src.ray(c, u, 0.3)
            if hs is not None and (cover_gap is None or ds - COVER_IN > rb + cover_gap):
                lim = min(max(lim, ds - COVER_IN), ds)
        lim_min = min(lim_min, lim)
        if r[i] > lim:
            P[i] = c + u * lim
            n_moved += 1
    return P, len(zone), n_moved, lim_min


def lip_seam(P, T, o, a, body, s_seam):
    """Shoes: replace the shell part below s_seam (limb axis) by a clean seam lip.  The decimated shell is
    clipped on the plane s = s_seam (planar rim, no saw-tooth against the tube facets), the rim verts are
    pushed radially to >= body tube + SHOE_LIP_OUT, zipped to a SHOE_LIP_N ring SHOE_LIP_DEPTH further
    up the leg and SHOE_LIP_IN inside the tube (hidden), and closed by a fan.  Returns P, T, rim verts, pushed."""
    ref = np.array([0.0, -1.0, 0.0])
    u = unit(ref - (ref @ a) * a)
    w = np.cross(a, u)
    Vc, Tc, _ = clip(P, T, (P - o) @ a - s_seam)
    far = int(np.argmax((Vc - o) @ a))
    Vc, Tc, _ = keep_component(Vc, Tc, far)
    loops = boundary_loops(Tc)
    if len(loops) != 1:
        raise RuntimeError(f"lip_seam: {len(loops)} rim loops at s = {s_seam}")
    rim = loops[0]
    Vc = Vc.copy()
    n_push = 0
    c0 = o + s_seam * a
    for i in rim:
        d = Vc[i] - c0
        d = d - (d @ a) * a
        rr = float(np.linalg.norm(d))
        uu = d / rr
        hit, dd = body.ray(c0, uu, 0.3)
        if hit is not None and rr < dd + SHOE_LIP_OUT:
            Vc[i] = c0 + uu * (dd + SHOE_LIP_OUT)
            n_push += 1
    c1 = o + (s_seam - SHOE_LIP_DEPTH) * a
    ring = []
    for k in range(SHOE_LIP_N):
        t = 2 * math.pi * (k + 0.5) / SHOE_LIP_N
        uu = math.cos(t) * u + math.sin(t) * w
        hit, dd = body.ray(c1, uu, 0.3)
        rb = dd if hit is not None else 0.05
        ring.append(c1 + uu * (rb - SHOE_LIP_IN))
    nb = len(Vc)
    V2 = np.vstack([Vc, np.array(ring), c1[None]])
    kid = list(range(nb, nb + SHOE_LIP_N))
    cid = nb + SHOE_LIP_N

    # zipper rim (walked in its loop order, angles unwrapped along it) <-> inner ring (by angle)
    def ang(ids):
        q = V2[np.array(ids)] - o
        return np.arctan2(q @ w, q @ u)

    ra = np.unwrap(ang(rim))
    if ra[-1] < ra[0]:
        rim = rim[::-1]
        ra = np.unwrap(ang(rim))
    k0 = int(np.argmin(np.mod(ra, 2 * math.pi)))
    rim = rim[k0:] + rim[:k0]
    ra = np.unwrap(ang(rim))
    ra = ra - 2 * math.pi * math.floor(ra[0] / (2 * math.pi))
    ta = np.append(ra, ra[0] + 2 * math.pi)
    B = kid
    tb = np.mod(ang(B), 2 * math.pi)
    ob_ = np.argsort(tb)
    B = [B[i] for i in ob_]
    tb = np.append(tb[ob_], tb[ob_][0] + 2 * math.pi)
    A = rim
    faces = [tuple(f) for f in Tc.tolist()]
    i = j = 0
    na, nb_ = len(A), len(B)
    while i < na or j < nb_:
        if j >= nb_ or (i < na and ta[i + 1] <= tb[j + 1]):
            faces.append((A[i % na], A[(i + 1) % na], B[j % nb_]))
            i += 1
        else:
            faces.append((A[i % na], B[(j + 1) % nb_], B[j % nb_]))
            j += 1
    for k in range(SHOE_LIP_N):
        faces.append((kid[k], kid[(k + 1) % SHOE_LIP_N], cid))
    V3, T3 = orient_outward(V2, triangulate_faces(faces))
    return V3, T3, len(rim), n_push


def build_limb_part(src, body, piv, joint, side, part):
    o, a, r_tube = limb_frame(piv, joint)
    V, T = src.V, src.T
    sv = (V - o) @ a
    sgn = 1.0 if side == "l" else -1.0
    near = np.linalg.norm(V - o, axis=1) < 0.40
    vsel = near & (sv > WRIST_CUT - 0.01) & (V[:, 0] * sgn > 0.05)
    tsel = vsel[T].any(1)
    Vr, Tr, _ = compact(V, T[tsel])
    f = (Vr - o) @ a - WRIST_CUT
    Vc, Tc, is_cut = clip(Vr, Tr, f)
    seed = int(np.argmax((Vc - o) @ a)) if joint.startswith("wrist") else int(np.argmin(Vc[:, 2]))
    Vp, Tp, used = keep_component(Vc, Tc, seed)
    meta = {"src_tris_region": len(Tp)}
    P = Vp.copy()
    faces = [tuple(x) for x in Tp.tolist()]
    # hidden zone clamp (before the cut cap)
    P, n_zone, n_moved, lim_min = clamp_hidden(P, o, a, r_tube, body, WRIST_CUT, cover_src=src,
                                               cover_gap=COVER_GAP, cover_start=COVER_START)
    meta.update({"zone_verts": n_zone, "zone_moved": n_moved, "zone_lim_min": lim_min})
    loops = boundary_loops(np.array(faces))
    s_loop = [float(np.mean((P[np.array(lp)] - o) @ a)) for lp in loops]
    cut_i = int(np.argmin(np.abs(np.array(s_loop) - WRIST_CUT)))
    meta["loops"] = [(len(lp), round(s_ * 1000, 1)) for lp, s_ in zip(loops, s_loop)]
    n_hole_caps = 0
    for k, lp in enumerate(loops):
        if k == cut_i:
            npnts, nf, _ = cap_loop(P, lp, n_rings=4)
        else:
            raise RuntimeError(f"{part}: unexpected open boundary loop ({len(lp)} verts)")
        P = np.vstack([P, npnts])
        faces += nf
    meta["hole_caps"] = n_hole_caps
    Tt = triangulate_faces(faces)
    meta["dense_tris"] = len(Tt)
    return P, Tt, meta, (o, a, r_tube)


# ---------------------------------------------------------------- ball fists (T60)
def quad_sphere(n):
    """Unit equal-angle cube sphere: 6 cube faces x n x n quads (tan-spaced grid), shared verts, outward
    winding.  Local +Z is a cube-face centre.  Returns V (k,3), Q (6 n^2, 4)."""
    t = np.tan(np.linspace(-math.pi / 4, math.pi / 4, n + 1))
    idx, V, Q = {}, [], []
    for k in range(3):
        for sg in (1.0, -1.0):
            nrm = sg * np.eye(3)[k]
            u = np.eye(3)[(k + 1) % 3]
            v = np.cross(nrm, u)                    # u x v = nrm -> (i,j)->(i+1,j)->(i+1,j+1) is CCW outside
            ids = np.empty((n + 1, n + 1), np.int64)
            for i in range(n + 1):
                for j in range(n + 1):
                    p = unit(nrm + t[i] * u + t[j] * v)
                    key = tuple(np.round(p, 6).tolist())
                    if key not in idx:
                        idx[key] = len(V)
                        V.append(p)
                    ids[i, j] = idx[key]
            for i in range(n):
                for j in range(n):
                    Q.append((ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]))
    return np.array(V), np.array(Q, np.int64)


def fist_ball(c, a, R, n=FIST_N):
    """Ball fist: quad sphere of vertex radius R at c, cube-face centre on the axis a (local +Z = a,
    local X = world +Z orthogonalised to a).  Returns V, Q (quads), T (tris)."""
    ref = np.array([0.0, 0.0, 1.0])
    u = unit(ref - (ref @ a) * a)
    w = np.cross(a, u)
    Vu, Q = quad_sphere(n)
    V = c + R * (Vu @ np.column_stack([u, w, a]).T)
    return V, Q, triangulate_faces(Q.tolist())


def grip_handle_points(Pw, Tc, gc, ga, s_half=FIST_GRIP_S, step=0.001):
    """GOB_club lathe surface points (world) with s in [-s_half, +s_half] about grip_r: plane sections
    of the lathe every `step` (section points = the 16-gon corners on the lathe edges) + lathe verts."""
    pts = [plane_section(Pw, Tc, gc + s * ga, ga) for s in np.arange(-s_half, s_half + 1e-9, step)]
    sv = (Pw - gc) @ ga
    vin = Pw[np.abs(sv) <= s_half]
    return np.vstack(pts + [vin]), len(vin)


def axis_chord(S, gc, ga, L=1.0):
    """Entry / exit of the grip axis line through the closed mesh S: (s_in, s_out) about gc, or None."""
    h1, d1 = S.ray(gc - L * ga, ga, 2 * L)
    h2, d2 = S.ray(gc + L * ga, -ga, 2 * L)
    if h1 is None or h2 is None:
        return None
    return d1 - L, L - d2


def fist_eval(d, R, piv, bodyV, loops_json, grip_pts, gc, ga):
    """Both balls at c = wrist.co + d * wrist.axis; ring depths (wrist _1 / _end), grip handle depth,
    grip-axis chord.  Returns dict."""
    out = {"d": d, "R": R}
    for x in ("l", "r"):
        o, a, _rt = limb_frame(piv, f"wrist_{x}")
        c = o + d * a
        V, Q, T = fist_ball(c, a, R)
        S = Surf(V, T)
        ring_d = {k: S.depths(bodyV[loops_json[f"wrist_{x}_{k}"]["verts"]]) for k in ("1", "end")}
        out[x] = {"c": c, "V": V, "Q": Q, "T": T, "S": S, "ring": ring_d,
                  "ring_min": min(float(v.min()) for v in ring_d.values())}
    S = out["r"]["S"]
    out["grip_depth"] = S.depths(grip_pts)
    out["chord"] = axis_chord(S, gc, ga)
    return out


# ---------------------------------------------------------------- belt
def chain_polyline(pts, adj, start, zlo, zhi):
    """Ordered polyline through `start` in both directions while z stays in [zlo, zhi]."""
    halves = []
    for j1 in adj[start]:
        path, prev, cur = [start, j1], start, j1
        while True:
            if not (zlo <= pts[cur][2] <= zhi):
                break
            nx = [k for k in adj[cur] if k != prev]
            if not nx or nx[0] == start:
                break
            prev, cur = cur, nx[0]
            path.append(cur)
        halves.append(path)
    a_, b_ = halves
    poly = pts[a_[::-1] + b_[1:]]
    if poly[0][2] > poly[-1][2]:
        poly = poly[::-1]
    return poly


def resample(poly, step=None, n=None):
    L = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))])
    q = np.arange(0.0, L[-1], step) if step else np.linspace(0.0, L[-1], n)
    return np.stack([np.interp(q, L, poly[:, k]) for k in range(poly.shape[1])], 1), q


def belt_section(Vr, Tr, th):
    """2D (r, z) SRC meridian polyline (0.5 mm steps, z increasing) through the belt at azimuth th,
    with indices of the bottom / top lip creases (most concave turn)."""
    pts, adj, e = section_chain(Vr, Tr, th)
    r = pts @ e
    z = pts[:, 2]
    cand = np.nonzero((z > 0.50) & (z < 0.56))[0]
    start = int(cand[np.argmax(r[cand])])
    poly3 = chain_polyline(pts, adj, start, 0.395, 0.615)
    rs, _ = resample(np.stack([poly3 @ e, poly3[:, 2]], 1), step=0.0005)
    k = 6
    v1 = rs[k:-k] - rs[:-2 * k]
    v2 = rs[2 * k:] - rs[k:-k]
    v1 = v1 / np.linalg.norm(v1, axis=1)[:, None]
    v2 = v2 / np.linalg.norm(v2, axis=1)[:, None]
    crs = np.concatenate([np.zeros(k), v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0], np.zeros(k)])
    zz = rs[:, 1]
    idx = np.arange(len(rs))
    wb = np.nonzero((zz > 0.418) & (zz < 0.446) & (idx >= k) & (idx < len(rs) - k))[0]
    wt = np.nonzero((zz > 0.565) & (zz < 0.592) & (idx >= k) & (idx < len(rs) - k))[0]
    ib = int(wb[np.argmin(crs[wb])])
    it = int(wt[np.argmin(crs[wt])])
    return rs, ib, it, e


def build_belt_band(src, bodyV, loops_json):
    """Visible belt band (open): BELT_COLS_DENSE columns x BELT_M_OUT rows, each column = SRC meridian
    polyline from the bottom lip crease to the top lip crease (arc-resampled)."""
    V, T = src.V, src.T
    cen = V[T].mean(1)
    reg = (cen[:, 2] > 0.37) & (cen[:, 2] < 0.64)
    Vr, Tr, _ = compact(V, T[reg])
    zb_row = float(bodyV[loops_json["belt_bot_0"]["verts"], 2].mean())
    zt_row = float(bodyV[loops_json["belt_top_0"]["verts"], 2].mean())
    ncol, M = BELT_COLS_DENSE, BELT_M_OUT
    cols, creases = [], []
    for i in range(ncol):
        rs, ib, it, e = belt_section(Vr, Tr, 2 * math.pi * i / ncol)
        outer, _ = resample(rs[ib:it + 1], n=M)
        cols.append(np.stack([outer[:, 0] * e[0], outer[:, 0] * e[1], outer[:, 1]], 1))
        creases.append((rs[ib][0], rs[ib][1], rs[it][0], rs[it][1]))
    P = np.concatenate(cols)
    faces = []
    for i in range(ncol):
        i1 = (i + 1) % ncol
        for j in range(M - 1):
            faces.append((i * M + j, i1 * M + j, i1 * M + j + 1, i * M + j + 1))
    Tt = triangulate_faces(faces)
    cr = np.array(creases)
    meta = {"rows_z": (zb_row, zt_row), "crease_bot_z": (cr[:, 1].min(), cr[:, 1].max()),
            "crease_top_z": (cr[:, 3].min(), cr[:, 3].max()), "dense_tris": len(Tt)}
    return P, Tt, meta


def zipper(P, A, B):
    """Triangle strip between two closed rings (vertex id lists) ordered by azimuth around z."""
    def order(ids):
        ids = np.array(ids)
        ang = np.mod(np.arctan2(P[ids, 1], P[ids, 0]), 2 * math.pi)
        o_ = np.argsort(ang)
        return ids[o_], ang[o_]
    A, ta = order(A)
    B, tb = order(B)
    ta = np.append(ta, ta[0] + 2 * math.pi)
    tb = np.append(tb, tb[0] + 2 * math.pi)
    na, nb = len(A), len(B)
    i = j = 0
    faces = []
    while i < na or j < nb:
        adv_a = j >= nb or (i < na and ta[i + 1] <= tb[j + 1])
        if adv_a:
            faces.append((A[i % na], A[(i + 1) % na], B[j % nb]))
            i += 1
        else:
            faces.append((A[i % na], B[(j + 1) % nb], B[j % nb]))
            j += 1
    return faces


def close_belt(P, T, body, meta):
    """Hidden part of the belt shell built after decimating the band: each band rim (lip crease) is
    zipped to a BELT_INNER_N-vertex inner ring BELT_IN inside GOB_body_tmp at the body belt row z -/+
    BELT_Z_EXT (under the lip, inside the body, which dives under the belt there), and the two inner
    rings are joined by quads."""
    loops = boundary_loops(T)
    if len(loops) != 2:
        raise RuntimeError(f"belt band: {len(loops)} boundary loops after decimation (expected 2)")
    loops.sort(key=lambda lp: float(P[np.array(lp), 2].mean()))
    rim_b, rim_t = loops
    zb_row, zt_row = meta["rows_z"]
    z_lo = zb_row - BELT_Z_EXT
    z_hi = zt_row + BELT_Z_EXT
    if not (float(P[np.array(rim_b), 2].max()) < z_lo < z_hi < float(P[np.array(rim_t), 2].min())):
        print(f"[s02c] belt WARNING: inner rings z {mm(z_lo)}/{mm(z_hi)} not strictly between the rims "
              f"(bottom rim max z {mm(P[np.array(rim_b), 2].max())}, top rim min z {mm(P[np.array(rim_t), 2].min())})")
    K = BELT_INNER_N
    new, ids = [], {}
    for key, zq in (("b", z_lo), ("t", z_hi)):
        ids[key] = []
        for k in range(K):
            th = 2 * math.pi * (k + 0.5) / K
            e = np.array([math.cos(th), math.sin(th), 0.0])
            hit, dd = body.ray((0.0, 0.0, zq), e, 1.0)
            if hit is None:
                raise RuntimeError(f"belt: no body hit at z {zq:.4f}")
            new.append(e * (dd - BELT_IN) + np.array([0.0, 0.0, zq]))
            ids[key].append(len(P) + len(new) - 1)
    P2 = np.vstack([P, np.array(new)])
    faces = [tuple(f) for f in T.tolist()]
    faces += zipper(P2, rim_b, ids["b"])
    faces += zipper(P2, rim_t, ids["t"])
    for k in range(K):
        k1 = (k + 1) % K
        faces.append((ids["b"][k], ids["b"][k1], ids["t"][k1], ids["t"][k]))
    movable = np.zeros(len(P2), bool)
    movable[np.array(rim_b + rim_t)] = True
    movable[len(P):] = True
    return P2, triangulate_faces(faces), (len(rim_b), len(rim_t), z_lo, z_hi), movable


def push_out(V, T, pts, target, movable, iters=20):
    """Containment touch-up: for every point less than `target` inside the closed shell (V, T), move the
    movable verts (lip-crease rims, hidden rings) of the nearest face along that face's outward normal.
    Returns V, number of vertex moves."""
    V = V.copy()
    moves = 0
    for _ in range(iters):
        S = Surf(V, T)
        bad = 0
        for b in pts:
            loc, _n, fi, dist = S.bvh.find_nearest(Vector(b))
            ins = S.inside(b)
            if ins and dist >= target:
                continue
            bad += 1
            f = T[fi]
            a_, b_, c_ = V[f]
            nrm = unit(np.cross(b_ - a_, c_ - a_))
            step = (target - dist + 0.0002) if ins else (dist + target + 0.0002)
            m = f[movable[f]]
            if not len(m):
                m = f
            V[m] += nrm * step
            moves += len(m)
        if not bad:
            break
    return V, moves


# ---------------------------------------------------------------- club
def plane_section(V, T, co, no):
    d = (V - co) @ no
    d = np.where(np.abs(d) < 1e-12, 1e-12, d)
    pos = d[T] >= 0
    cnt = pos.sum(1)
    m = (cnt == 1) | (cnt == 2)
    TT, Pm = T[m], pos[m]
    cr = Pm != Pm[:, [1, 2, 0]]
    ia = TT[cr]
    ib = TT[:, [1, 2, 0]][cr]
    t = d[ia] / (d[ia] - d[ib])
    return V[ia] + t[:, None] * (V[ib] - V[ia])


def circle_fit(u, v):
    A = np.column_stack([2 * u, 2 * v, np.ones(len(u))])
    b = u * u + v * v
    cu, cv, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
    return float(cu), float(cv), float(math.sqrt(max(c0 + cu * cu + cv * cv, 0.0)))


def club_frame(piv):
    g = piv["grip_r"]
    o = np.array(g["co"], float)
    Y = unit(g["axis"])
    wz = np.array([0.0, 0.0, 1.0])
    Z = unit(wz - (wz @ Y) * Y)
    X = np.cross(Y, Z)
    return o, X, Y, Z


def build_club(club, piv):
    o, X, Y, Z = club_frame(piv)
    V, T = club.V, club.T
    sv = (V - o) @ Y
    gap_lo, gap_hi = float(sv[sv < 0].max()), float(sv[sv > 0].min())
    s_min, s_max = float(sv.min()), float(sv.max())
    phis = [2 * math.pi * k / CLUB_NSEG for k in range(CLUB_NSEG)]
    dirs = [math.cos(p) * X + math.sin(p) * Z for p in phis]
    stations = []
    for lo_, hi_ in ((s_min + 0.0015, gap_lo - CLUB_GAP_MARGIN), (gap_hi + CLUB_GAP_MARGIN, s_max - 0.0015)):
        for s in np.arange(lo_, hi_ + 1e-9, 0.001):
            sec = plane_section(V, T, o + s * Y, Y)
            if len(sec) < 8:
                continue
            u, v = (sec - o) @ X, (sec - o) @ Z
            cu, cv, R = circle_fit(u, v)
            if R < 0.004:
                continue
            c3 = o + s * Y + cu * X + cv * Z
            ring = []
            for dv in dirs:
                hit, dd = club.ray(c3, dv, 0.5)
                q = hit if (hit is not None and abs(dd - R) < 0.25 * R) else c3 + R * dv
                ring.append(c3 + (q - c3) * CLUB_RING_SCALE)
            stations.append({"s": float(s), "c": c3, "R": R, "ring": np.array(ring),
                             "piece": 0 if s < 0 else 1})
    ss = np.array([st["s"] for st in stations])
    ib = int(np.nonzero(ss < 0)[0].max())
    ifr = int(np.nonzero(ss > 0)[0].min())
    chosen = sorted({0, ib, ifr, len(stations) - 1})
    rings_all = np.array([st["ring"] for st in stations])        # (n,16,3)

    def seg_err(i, j):
        if j - i < 2:
            return 0.0, None
        t = (ss[i + 1:j] - ss[i]) / (ss[j] - ss[i])
        interp = rings_all[i][None] * (1 - t)[:, None, None] + rings_all[j][None] * t[:, None, None]
        err = np.linalg.norm(interp - rings_all[i + 1:j], axis=2).max(1)
        k = int(np.argmax(err))
        return float(err[k]), i + 1 + k

    while len(chosen) < CLUB_RINGS:
        best = (-1.0, None)
        for a_, b_ in zip(chosen, chosen[1:]):
            if a_ == ib and b_ == ifr:
                continue   # fist gap: straight interpolation
            e_, k_ = seg_err(a_, b_)
            if k_ is not None and e_ > best[0]:
                best = (e_, k_)
        if best[1] is None:
            break
        chosen = sorted(chosen + [best[1]])
    max_err = max(seg_err(a_, b_)[0] for a_, b_ in zip(chosen, chosen[1:]) if not (a_ == ib and b_ == ifr))
    rings = [stations[i]["ring"] for i in chosen]
    P = []
    faces = []
    n = CLUB_NSEG
    for rg in rings:
        P.extend(rg.tolist())
    nr = len(rings)
    for j in range(nr - 1):
        for k in range(n):
            k1 = (k + 1) % n
            faces.append((j * n + k, j * n + k1, (j + 1) * n + k1, (j + 1) * n + k))
    P = np.array(P)
    # caps: 5x5 grid (4x4 quads), boundary = 16 ring verts, inner verts projected along the axis
    per = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4), (3, 4), (4, 4), (4, 3), (4, 2), (4, 1),
           (4, 0), (3, 0), (2, 0), (1, 0)]
    for end, rid, dirn in (("butt", 0, -Y), ("head", nr - 1, Y)):
        ring_ids = list(range(rid * n, rid * n + n))
        G = {ij: P[ring_ids[m]] for m, ij in enumerate(per)}
        gid = {ij: ring_ids[m] for m, ij in enumerate(per)}
        newp = []
        for i in (1, 2, 3):
            for j in (1, 2, 3):
                uu, vv = i / 4.0, j / 4.0
                p = ((1 - uu) * G[(0, j)] + uu * G[(4, j)] + (1 - vv) * G[(i, 0)] + vv * G[(i, 4)]
                     - ((1 - uu) * (1 - vv) * G[(0, 0)] + uu * (1 - vv) * G[(4, 0)]
                        + (1 - uu) * vv * G[(0, 4)] + uu * vv * G[(4, 4)]))
                hit, _dd = club.ray(p - dirn * 0.002, dirn, 0.2)
                if hit is not None:
                    p = hit
                newp.append(p)
                gid[(i, j)] = len(P) + len(newp) - 1
        P = np.vstack([P, np.array(newp)])
        for i in range(4):
            for j in range(4):
                q = (gid[(i, j)], gid[(i + 1, j)], gid[(i + 1, j + 1)], gid[(i, j + 1)])
                faces.append(q)
    Tt = triangulate_faces(faces)
    prof = [(stations[i]["s"], float(np.linalg.norm(stations[i]["ring"] - stations[i]["c"], axis=1).mean()),
             float(np.linalg.norm(stations[i]["ring"] - stations[i]["c"], axis=1).min()),
             float(np.linalg.norm(stations[i]["ring"] - stations[i]["c"], axis=1).max()),
             float((stations[i]["c"] - o) @ X), float((stations[i]["c"] - o) @ Z)) for i in chosen]
    meta = {"frame": (o, X, Y, Z), "gap": (gap_lo, gap_hi), "s_range": (s_min, s_max),
            "n_stations": len(stations), "rings": nr, "profile": prof, "max_ring_interp_err": max_err}
    return P, Tt, meta


# ---------------------------------------------------------------- grip (G2.11 grip region, SRC_club_hi fist gap)
def grip_core(club, piv):
    g = piv["grip_r"]
    gc, ga = np.array(g["co"], float), unit(g["axis"])
    dc = club.V - gc
    sc = dc @ ga
    rc = np.linalg.norm(dc - np.outer(sc, ga), axis=1)
    gap_lo, gap_hi = float(sc[sc < 0].max()), float(sc[sc > 0].min())
    r_lo = float(np.median(rc[(sc >= gap_lo - 0.05) & (sc <= gap_lo)]))
    r_hi = float(np.median(rc[(sc >= gap_hi) & (sc <= gap_hi + 0.05)]))
    r_grip = min(r_lo, r_hi)
    return gc, ga, r_grip, r_grip - 0.005, (gap_lo, gap_hi, r_lo, r_hi)


# ---------------------------------------------------------------- objects
def make_obj(name, V, T, matrix=None):
    coll = bpy.data.collections.get(COLL_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLL_NAME)
        bpy.context.scene.collection.children.link(coll)
    old = bpy.data.objects.get(name)
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    me = bpy.data.meshes.new(name)
    me.from_pydata(np.asarray(V).tolist(), [], [[int(i) for i in f] for f in T])
    me.validate(clean_customdata=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    if matrix is not None:
        ob.matrix_world = matrix
    return ob


def topo_stats(V, T):
    e = np.sort(tri_edges(T), axis=1)
    _, cnt = np.unique(e[:, 0] * len(V) + e[:, 1], return_counts=True)
    lab = components(len(V), tri_edges(T))
    shells = len(np.unique(lab[T[:, 0]]))
    # per-shell signed volumes
    vols = []
    for L in np.unique(lab[T[:, 0]]):
        vols.append(signed_volume(V, T[lab[T[:, 0]] == L]))
    return {"tris": len(T), "verts": len(V), "shells": shells, "boundary": int((cnt == 1).sum()),
            "nonmanifold": int((cnt > 2).sum()), "vol_cm3": [round(v * 1e6, 3) for v in vols]}


def sample_surface(V, T, n, rng):
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    ti = rng.choice(len(T), size=n, p=area / area.sum())
    r1 = np.sqrt(rng.random(n))
    r2 = rng.random(n)
    pts = (1 - r1)[:, None] * a[ti] + (r1 * (1 - r2))[:, None] * b[ti] + (r1 * r2)[:, None] * c[ti]
    return pts, ti


def shells_of(name, V, T):
    lab = components(len(V), tri_edges(T))
    out = []
    for k, L in enumerate(np.unique(lab[T[:, 0]])):
        TT = T[lab[T[:, 0]] == L]
        Vs, Ts, _ = compact(V, TT)
        e = np.sort(tri_edges(Ts), axis=1)
        _, cnt = np.unique(e[:, 0] * len(Vs) + e[:, 1], return_counts=True)
        out.append({"name": name, "k": k, "surf": Surf(Vs, Ts), "closed": bool((cnt == 2).all()),
                    "tri_mask": lab[T[:, 0]] == L})
    return out


def occluded(pts, own_shell_ids, shells):
    hidden = np.zeros(len(pts), bool)
    iface = np.zeros(len(pts), bool)
    for sid, sh in enumerate(shells):
        if not sh["closed"]:
            continue
        S = sh["surf"]
        inb = np.all((pts >= S.mn - IFACE_TOL) & (pts <= S.mx + IFACE_TOL), axis=1)
        for i in np.nonzero(inb & (own_shell_ids != sid) & ~hidden)[0].tolist():
            d = S.dist(pts[i])
            if d <= IFACE_TOL:
                iface[i] = True
            elif d > HIDE_DEPTH and S.inside(pts[i]):
                hidden[i] = True
    return hidden, iface & ~hidden


def stats(d):
    if not len(d):
        return "n=0"
    return (f"n={len(d)} mean={mm(d.mean())} p95={mm(np.percentile(d, 95))} max={mm(d.max())} "
            f"n>2mm={int((d > 0.002).sum())} n>3mm={int((d > 0.003).sum())}")


# ---------------------------------------------------------------- main
def main():
    piv = goblib.load_json("pivots.json")["pivots"]
    for n_ in ("SRC_hi", "SRC_club_hi", BODY_NAME):
        if bpy.data.objects.get(n_) is None:
            raise RuntimeError(f"missing input object {n_}")
    with open(IN_LOOPS, "r", encoding="utf-8") as f:
        loops_json = json.load(f)["rings"]
    src = Surf(*obj_arrays(bpy.data.objects["SRC_hi"]))
    club = Surf(*obj_arrays(bpy.data.objects["SRC_club_hi"]))
    bodyV, bodyT = obj_arrays(bpy.data.objects[BODY_NAME])
    body = Surf(bodyV, bodyT)
    print(f"[s02c] SRC_hi verts={len(src.V)} tris={len(src.T)}; SRC_club_hi tris={len(club.T)}; "
          f"{BODY_NAME} verts={len(bodyV)} tris={len(bodyT)}")

    # grip (G2.11 grip region)
    gc, ga, r_grip, r_core, gapinfo = grip_core(club, piv)
    print(f"[s02c] grip: r_grip={mm(r_grip)} r_core={mm(r_core)} gap=[{mm(gapinfo[0])},{mm(gapinfo[1])}]")

    parts = {}
    metas = {}
    # ---- head
    P, T, meta = build_head(src, body)
    print(f"[s02c] head: rim verts={meta['rim_verts']} rim z=[{mm(meta['rim_z'][0])},{mm(meta['rim_z'][1])}] "
          f"skirt z={mm(meta['z_skirt'])} extra holes capped={meta['extra_holes']} dense tris={meta['dense_tris']}")
    cr = np.array([c[1:] for c in meta["crease"]])
    print(f"[s02c] head crease: r mm min/max={mm(cr[:, 0].min())}/{mm(cr[:, 0].max())} "
          f"z mm min/max={mm(cr[:, 1].min())}/{mm(cr[:, 1].max())} bisector deg min/max="
          f"{round(cr[:, 2].min(), 1)}/{round(cr[:, 2].max(), 1)}; every 30 deg (az, r, z, bis): "
          + ", ".join(f"({c[0]},{mm(c[1])},{mm(c[2])},{round(c[3], 1)})" for c in meta["crease"][::30]))
    P, T = orient_outward(P, T)
    # T60 fist size reference: the previous collapse-decimated, fitted head (only its x width is used)
    Pd, Td = decimate("head", P, T, HEAD_FIST_REF_BUDGET)
    Pd, Td = beautify_flat(Pd, Td, Pd[:, 2] < meta["z_skirt"] + 0.0005)   # hidden planar cap: no slivers
    Pd = fit_surface(Pd, Td, P, T, Pd[:, 2] < meta["rim_z"][0] - 0.001)   # skirt / cap fixed
    fist_ref_head = Pd
    Vh, head_polys, hx = build_head_quads(src, P, T, meta)
    parts["head"] = (Vh, triangulate_faces(head_polys))
    metas["head"] = meta
    hval = {}
    for f_ in head_polys:
        for k_ in range(len(f_)):
            e_ = (min(f_[k_], f_[k_ - 1]), max(f_[k_], f_[k_ - 1]))
            hval[e_] = 1
    vcount = np.zeros(len(Vh), np.int64)
    for a_, b_ in hval:
        vcount[a_] += 1
        vcount[b_] += 1
    vh_, vn_ = np.unique(vcount, return_counts=True)
    print(f"[s02c] HEAD quads: polys={len(head_polys)} tris={len(parts['head'][1])} quads="
          f"{sum(len(f_) == 4 for f_ in head_polys)} triangles={sum(len(f_) == 3 for f_ in head_polys)} "
          f"verts={len(Vh)} valence={dict(zip(vh_.tolist(), vn_.tolist()))}")
    print("[s02c] HEAD build " + json.dumps(goblib._jsonable(hx["info"])))
    zones = head_design_zones(src, Vh, hx)
    zpath = goblib.save_json("head_design_zones.json", zones)
    print(f"[s02c] HEAD_DESIGN_ZONES {zpath}: " + json.dumps(goblib._jsonable(
        {k: {kk: vv for kk, vv in v.items() if kk not in ("polygons_xz", "rule", "_basis", "use")}
         for k, v in zones.items()})))

    # ---- club lathe (needed by the hand_r ball placement; object made below)
    Pw, Tc, cmeta = build_club(club, piv)
    Pw, Tc = orient_outward(Pw, Tc)

    # ---- hands: ball fists (T60)
    hv_ = fist_ref_head
    head_w = float(hv_[:, 0].max() - hv_[:, 0].min())
    R_f = FIST_RATIO * head_w / 2.0 * FIST_R_ADJ
    print("[s02c] FIST_REF (reference keyframes, fist diameter / head width, px; circle fit on the green mask "
          "boundary, see docstring): " + "; ".join(f"{f_} {n_}: {d_} / {h_} = {d_ / h_:.3f}" for f_, n_, d_, h_ in FIST_REF)
          + f" -> FIST_RATIO (mean) = {FIST_RATIO:.4f}")
    print(f"[s02c] FIST_SIZE: head x width (T60 reference head, {HEAD_FIST_REF_BUDGET} tris) = {mm(head_w)} mm (x [{mm(hv_[:, 0].min())}, {mm(hv_[:, 0].max())}]) "
          f"-> R = {FIST_RATIO:.4f} * {mm(head_w)} / 2 * FIST_R_ADJ {FIST_R_ADJ} = {mm(R_f)} mm "
          f"(target radius {mm(FIST_RATIO * head_w / 2.0)} mm, allowed {mm(0.85 * FIST_RATIO * head_w / 2.0)}.."
          f"{mm(1.15 * FIST_RATIO * head_w / 2.0)})")
    grip_pts, n_grip_v = grip_handle_points(Pw, Tc, gc, ga)
    fe, n_try = None, 0
    for d_ in np.arange(R_f, -1e-9, -FIST_D_STEP):
        n_try += 1
        ev = fist_eval(float(d_), R_f, piv, bodyV, loops_json, grip_pts, gc, ga)
        if min(ev["l"]["ring_min"], ev["r"]["ring_min"]) < FIST_RING_DEPTH + FIST_MARGIN:
            continue
        ch = ev["chord"]
        if (ev["grip_depth"].min() < FIST_GRIP_DEPTH + FIST_MARGIN or ch is None
                or ch[1] - ch[0] < FIST_CHORD_MIN * 2.0 * R_f):
            continue
        fe = ev
        break
    if fe is None:
        raise RuntimeError(f"ball fist: no centre d in [0, {mm(R_f)}] mm meets the ring / grip constraints (R {mm(R_f)} mm)")
    print(f"[s02c] FIST_CENTRE search: d from {mm(R_f)} mm down in {mm(FIST_D_STEP)} mm steps, {n_try} tried -> "
          f"d = {mm(fe['d'])} mm along each wrist axis (ring >= {mm(FIST_RING_DEPTH + FIST_MARGIN)} mm, grip >= "
          f"{mm(FIST_GRIP_DEPTH + FIST_MARGIN)} mm, chord >= {FIST_CHORD_MIN} x diameter)")
    fist_q = {}
    for x in ("l", "r"):
        part = f"hand_{x}"
        V_, Q_, T_ = fe[x]["V"], fe[x]["Q"], fe[x]["T"]
        if len(T_) > BUDGET[part]:
            raise RuntimeError(f"{part}: {len(T_)} tris > budget {BUDGET[part]}")
        parts[part] = (V_, T_)
        fist_q[part] = Q_
        metas[part] = {"centre": fe[x]["c"], "R": R_f, "d": fe["d"]}
        print(f"[s02c] {part}: ball fist quads={len(Q_)} tris={len(T_)} verts={len(V_)} centre="
              f"{np.round(fe[x]['c'], 5).tolist()} R={mm(R_f)} mm")

    # ---- shoes
    limb_ctx = {}
    for part, joint, side in (("shoe_l", "ankle_l", "l"), ("shoe_r", "ankle_r", "r")):
        P, T, meta, fr = build_limb_part(src, body, piv, joint, side, part)
        P, T = orient_outward(P, T)
        o, a, r_tube = fr
        target = BUDGET[part] - (2 * SHOE_LIP_N + 40 if part.startswith("shoe") else 0)
        for _ in range(5):
            Pd, Td = decimate(part, P, T, target)
            Pd = fit_surface(Pd, Td, P, T, (Pd - o) @ a < 0.0)               # hidden zone / cut cap fixed
            Pd, nz, nm_, lim = clamp_hidden(Pd, o, a, r_tube, body, WRIST_CUT, cover_src=src,
                                            cover_gap=COVER_GAP, cover_start=COVER_START)
            if not part.startswith("shoe"):
                break
            Pd, Td, n_rim, n_push = lip_seam(Pd, Td, o, a, body, SHOE_SEAM_S)
            meta["lip"] = (n_rim, n_push, target)
            if len(Td) <= BUDGET[part]:
                break
            target -= len(Td) - BUDGET[part]
        meta["post_zone_moved"] = nm_
        parts[part] = (Pd, Td)
        metas[part] = meta
        limb_ctx[part] = fr
        print(f"[s02c] {part}: region tris={meta['src_tris_region']} loops(n,s mm)={meta['loops']} "
              f"zone verts={meta['zone_verts']} moved={meta['zone_moved']} lim_min={mm(meta['zone_lim_min'])} "
              f"hole caps={meta['hole_caps']} dense tris={meta['dense_tris']} post-decimate zone moved="
              f"{meta['post_zone_moved']}"
              + (f" seam lip: s={mm(SHOE_SEAM_S)} rim verts={meta['lip'][0]} rim pushed={meta['lip'][1]} "
                 f"inner ring {SHOE_LIP_N} verts, decimate target {meta['lip'][2]}" if "lip" in meta else ""))

    # ---- belt
    P, T, meta = build_belt_band(src, bodyV, loops_json)
    print(f"[s02c] belt: body rows z=[{mm(meta['rows_z'][0])},{mm(meta['rows_z'][1])}] lip crease bot z "
          f"[{mm(meta['crease_bot_z'][0])},{mm(meta['crease_bot_z'][1])}] top z [{mm(meta['crease_top_z'][0])},"
          f"{mm(meta['crease_top_z'][1])}] dense band tris={meta['dense_tris']}")
    target = BUDGET["belt"] - 2 * BELT_INNER_N - 100
    for _ in range(4):   # band budget = belt budget - hidden closing tris (depends on the rim sizes)
        Pb_, Tb_ = decimate("belt", P, T, target)
        rimv = np.zeros(len(Pb_), bool)
        for lp in boundary_loops(Tb_):
            rimv[np.array(lp)] = True
        Pb_ = fit_surface(Pb_, Tb_, P, T, rimv)                              # lip-crease rims fixed
        Pc, Tcl, info, movable = close_belt(Pb_, Tb_, body, meta)
        if len(Tcl) <= BUDGET["belt"]:
            break
        target -= len(Tcl) - BUDGET["belt"]
    P, T = orient_outward(Pc, Tcl)
    zb_, zt_ = meta["rows_z"]
    P, n_mv = push_out(P, T, bodyV[(bodyV[:, 2] >= zb_) & (bodyV[:, 2] <= zt_)], BELT_PUSH_DEPTH, movable)
    print(f"[s02c] belt: containment touch-up (body belt-band verts >= {mm(BELT_PUSH_DEPTH)} mm inside; only rim / "
          f"hidden verts moved along the nearest face normal): vertex moves={n_mv}")
    print(f"[s02c] belt: band decimated to {len(Tb_)} tris, rims bot/top verts={info[0]}/{info[1]}, hidden inner "
          f"rings {BELT_INNER_N} verts at z {mm(info[2])} / {mm(info[3])}, total {len(T)} tris")
    parts["belt"] = (P, T)
    metas["belt"] = meta

    # ---- club
    o, X, Y, Z = cmeta["frame"]
    R = np.column_stack([X, Y, Z])
    Pl = (Pw - o) @ R
    mw = Matrix.Translation(Vector(o)) @ Matrix([list(R[0]), list(R[1]), list(R[2])]).to_4x4()

    # ---- objects
    objs = {}
    for part in ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt"):
        V_, T_ = parts[part]
        objs[part] = make_obj(PART_OBJ[part], V_, head_polys if part == "head" else fist_q.get(part, T_))
    n_hard = tag_hard_edges(objs["head"], hx["hard_edges"])
    print(f"[s02c] HEAD hard edges (attribute {HEAD_HARD_ATTR}: crease rim loop ({len(_perimeter(*hx['info']['lattice']['segments'][:2]))} "
          f"edges), eye junction + wall-bottom loops, fang foot / top outlines + fang corner edges) = {n_hard}")
    objs["club"] = make_obj(PART_OBJ["club"], Pl, Tc, matrix=mw)
    bpy.context.view_layer.update()

    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)

    # ---- renders
    body_ob = bpy.data.objects[BODY_NAME]
    all_objs = [body_ob] + [objs[k] for k in ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt", "club")]
    cols = {o_.name: COLORS[o_.name] for o_ in all_objs}
    renders = []
    W = goblib.WORK
    renders.append(goblib.ortho_render(all_objs, "front", W / "r01c_all_front.png", colors=cols))
    renders.append(goblib.ortho_render(all_objs, "three_quarter", W / "r01c_all_three_quarter.png", colors=cols))
    renders.append(goblib.ortho_render([objs["hand_r"]], "top", W / "r01c_hand_r_top.png", wire=True,
                                       colors={PART_OBJ["hand_r"]: COLORS["GOB_hand_r"]}))
    renders.append(goblib.ortho_render([objs["hand_r"]], "front", W / "r01c_hand_r_front.png", wire=True,
                                       colors={PART_OBJ["hand_r"]: COLORS["GOB_hand_r"]}))
    wl = np.array(piv["wrist_l"]["co"])
    renders.append(goblib.ortho_render([body_ob, objs["hand_l"]], "three_quarter", W / "r01c_wrist_l.png",
                                       colors={BODY_NAME: COLORS[BODY_NAME], "GOB_hand_l": COLORS["GOB_hand_l"]},
                                       frame_bbox=((wl[0] - 0.09, wl[1] - 0.09, wl[2] - 0.09),
                                                   (wl[0] + 0.07, wl[1] + 0.09, wl[2] + 0.09))))
    wr = np.array(piv["wrist_r"]["co"])
    renders.append(goblib.ortho_render([body_ob, objs["hand_r"]], "three_quarter", W / "r01c_wrist_r.png",
                                       colors={BODY_NAME: COLORS[BODY_NAME], "GOB_hand_r": COLORS["GOB_hand_r"]},
                                       frame_bbox=((wr[0] - 0.07, wr[1] - 0.09, wr[2] - 0.09),
                                                   (wr[0] + 0.09, wr[1] + 0.09, wr[2] + 0.09))))
    renders.append(goblib.ortho_render([body_ob, objs["hand_r"]], "front", W / "r01c_wrist_r_side.png",
                                       colors={BODY_NAME: COLORS[BODY_NAME], "GOB_hand_r": COLORS["GOB_hand_r"]},
                                       frame_bbox=((wr[0] - 0.07, wr[1] - 0.09, wr[2] - 0.09),
                                                   (wr[0] + 0.09, wr[1] + 0.09, wr[2] + 0.09))))
    for side in ("l", "r"):
        an = np.array(piv[f"ankle_{side}"]["co"])
        renders.append(goblib.ortho_render([body_ob, objs[f"shoe_{side}"]], "three_quarter",
                                           W / f"r01c_ankle_{side}.png",
                                           colors={BODY_NAME: COLORS[BODY_NAME],
                                                   f"GOB_shoe_{side}": COLORS[f"GOB_shoe_{side}"]},
                                           frame_bbox=((an[0] - 0.10, an[1] - 0.10, an[2] - 0.08),
                                                       (an[0] + 0.10, an[1] + 0.10, an[2] + 0.10))))
    renders.append(goblib.ortho_render([objs["head"]], "front", W / "r01c_head_front.png", wire=True,
                                       colors={"GOB_head": COLORS["GOB_head"]}))
    renders.append(goblib.ortho_render([body_ob, objs["belt"]], "side", W / "r01c_belt_side.png",
                                       colors={BODY_NAME: COLORS[BODY_NAME], "GOB_belt": COLORS["GOB_belt"]},
                                       frame_bbox=((-0.42, -0.36, 0.36), (0.42, 0.36, 0.65))))
    renders.append(goblib.ortho_render([objs["club"]], "top", W / "r01c_club.png", wire=True,
                                       colors={"GOB_club": COLORS["GOB_club"]}))

    # ---------------- reports
    print("[s02c] OUTPUT " + json.dumps({"blend": str(OUT_BLEND), "renders": [str(p) for p in renders]}))
    world = {}
    for part in ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt"):
        world[part] = parts[part]
    world["club"] = (Pw, Tc)
    total = len(bodyT)
    print("[s02c] TOPOLOGY (object: tris verts shells boundary_edges nonmanifold_edges signed_volume_cm3 per shell):")
    bt = topo_stats(bodyV, bodyT)
    print(f"   {BODY_NAME}: tris={bt['tris']} (input, unchanged)")
    for part, (V_, T_) in world.items():
        st = topo_stats(V_, T_)
        if part != "club":
            total += st["tris"]
        sign = "+" if all(v > 0 for v in st["vol_cm3"]) else "-"
        print(f"   {PART_OBJ[part]}: tris={st['tris']} verts={st['verts']} shells={st['shells']} "
              f"boundary={st['boundary']} nonmanifold={st['nonmanifold']} volume_sign={sign} vol={st['vol_cm3']}"
              + (f" budget={BUDGET[part]}" if part in BUDGET else " budget=<1000"))
    ob_c = objs["club"]
    mwn = np.array(ob_c.matrix_world)
    rigid = total - len(bodyT)
    print(f"[s02c] TRIS_TOTAL GOB_mesh expected (body {len(bodyT)} + 6 rigid parts {rigid}) = {total} "
          f"(limit {GOB_MESH_MAX}, margin {GOB_MESH_MAX - total}); GOB_club = {len(Tc)} (< 1000)")

    # ---- club frame / profile
    print(f"[s02c] CLUB frame: origin={np.round(o, 5).tolist()} (grip_r.co={np.round(piv['grip_r']['co'], 5).tolist()}) "
          f"X={np.round(X, 5).tolist()} Y={np.round(Y, 5).tolist()} Z={np.round(Z, 5).tolist()}; "
          f"matrix_world loc={np.round(mwn[:3, 3], 5).tolist()} "
          f"axis Y vs grip_r.axis angle={math.degrees(math.acos(min(1.0, float(unit(mwn[:3, 1]) @ unit(piv['grip_r']['axis']))))):.4f} deg "
          f"scale={np.round(np.linalg.norm(mwn[:3, :3], axis=0), 6).tolist()}")
    print(f"[s02c] CLUB s range=[{mm(cmeta['s_range'][0])},{mm(cmeta['s_range'][1])}] gap=[{mm(cmeta['gap'][0])},"
          f"{mm(cmeta['gap'][1])}] stations={cmeta['n_stations']} rings={cmeta['rings']} "
          f"max ring interpolation err={mm(cmeta['max_ring_interp_err'])} mm")
    print("[s02c] CLUB profile (s mm, mean r, min r, max r, centre X, centre Z; mm):")
    print("   " + " ".join(f"({mm(p[0])},{mm(p[1])},{mm(p[2])},{mm(p[3])},{mm(p[4])},{mm(p[5])})" for p in cmeta["profile"]))

    # ---- boundary depths
    surf = {part: [sh for sh in shells_of(part, *world[part])] for part in world}

    def part_depth(part, pts):
        return np.max(np.stack([sh["surf"].depths(pts) for sh in surf[part]]), axis=0)

    print("[s02c] END_RING_DEPTH (body _end ring verts inside the rigid shell, mm; >= 3 required by G2.14a):")
    for ring, part in (("wrist_l_end", "hand_l"), ("wrist_r_end", "hand_r"), ("ankle_l_end", "shoe_l"),
                       ("ankle_r_end", "shoe_r")):
        d = part_depth(part, bodyV[loops_json[ring]["verts"]])
        print(f"   {ring} in {PART_OBJ[part]}: min={mm(d.min())} mean={mm(d.mean())} n<3mm={int((d < 0.003).sum())}")
    hv = world["head"][0]
    zmin = float(hv[:, 2].min())
    band = hv[hv[:, 2] <= zmin + 0.002]
    dh = body.depths(band)
    print(f"[s02c] HEAD_BOTTOM_BAND: head min z={mm(zmin)} band verts={len(band)} depth in body min={mm(dh.min())} "
          f"violations(<3mm)={int((dh < 0.003).sum())}")
    zb = float(bodyV[loops_json["belt_bot_0"]["verts"], 2].mean())
    zt = float(bodyV[loops_json["belt_top_0"]["verts"], 2].mean())
    selb = bodyV[(bodyV[:, 2] >= zb) & (bodyV[:, 2] <= zt)]
    db = part_depth("belt", selb)
    print(f"[s02c] BELT_BODY_DEPTH: z=[{mm(zb)},{mm(zt)}] body verts={len(selb)} depth in belt min={mm(db.min())} "
          f"violations(<0.5mm)={int((db < 0.0005).sum())}")
    print(f"[s02c] FIST (ball fists; depth = + inside the ball mesh, mm; G2.14 rings >= 3, G2.16 grip >= 1):")
    for x in ("l", "r"):
        part = f"hand_{x}"
        o_, a_, rt_ = limb_frame(piv, f"wrist_{x}")
        V_, T_ = world[part]
        c_ = metas[part]["centre"]
        S_ = surf[part][0]["surf"]
        fc = np.linalg.norm(V_[T_].mean(1) - c_, axis=1)
        dc = c_ - o_
        print(f"   {part}: centre={np.round(c_, 5).tolist()} = wrist_{x} + {mm(dc @ a_)} mm * axis (radial offset "
              f"{mm(np.linalg.norm(dc - (dc @ a_) * a_))} mm) R(vertex)={mm(R_f)} min tri-centroid radius={mm(fc.min())} "
              f"tube r {mm(rt_)} enters the ball at s={mm(fe['d'] - math.sqrt(max(R_f ** 2 - rt_ ** 2, 0.0)))} mm")
        for k in ("0", "1", "end"):
            vv = bodyV[loops_json[f"wrist_{x}_{k}"]["verts"]]
            dd = S_.depths(vv)
            print(f"      wrist_{x}_{k}: s={mm(((vv - o_) @ a_).mean())} depth in ball min={mm(dd.min())} "
                  f"mean={mm(dd.mean())} max={mm(dd.max())} n<3mm={int((dd < 0.003).sum())}")
        sb = (bodyV - o_) @ a_
        rb = np.linalg.norm((bodyV - o_) - np.outer(sb, a_), axis=1)
        ext = bodyV[(sb > 0.0005) & (rb < 0.1) & (sb < 0.3)]
        de = S_.depths(ext)
        print(f"      body verts past wrist_{x}_1 (extension + cap, s > 0.5 mm): n={len(ext)} depth in ball "
              f"min={mm(de.min())} n<3mm={int((de < 0.003).sum())}")
    gd = fe["grip_depth"]
    ch = fe["chord"]
    csv = (Pw - gc) @ ga
    print(f"   hand_r grip (G2.16): GOB_club lathe points with s in [{mm(-FIST_GRIP_S)}, {mm(FIST_GRIP_S)}] mm about "
          f"grip_r = {len(grip_pts)} (plane sections every 1 mm; lathe verts in range {n_grip_v}) depth in hand_r "
          f"min={mm(gd.min())} mean={mm(gd.mean())} n<1mm={int((gd < 0.001).sum())}")
    print(f"   hand_r grip axis: enters the ball at s={mm(ch[0])} exits at s={mm(ch[1])} chord={mm(ch[1] - ch[0])} mm "
          f"= {(ch[1] - ch[0]) / (2 * R_f):.3f} x diameter {mm(2 * R_f)}; GOB_club s range [{mm(csv.min())}, "
          f"{mm(csv.max())}] -> handle out of the ball behind by {mm(ch[0] - csv.min())} mm, in front by "
          f"{mm(csv.max() - ch[1])} mm; grip_r distance to the ball centre="
          f"{mm(np.linalg.norm(gc - metas['hand_r']['centre']))} mm, axis-line distance="
          f"{mm(np.linalg.norm(np.cross(metas['hand_r']['centre'] - gc, ga)))} mm")
    for part in ("shoe_l", "shoe_r"):
        o_, a_, rt_ = limb_ctx[part]
        V_ = world[part][0]
        s2 = (V_ - o_) @ a_
        z_ = V_[(s2 >= WRIST_CUT - 1e-6) & (s2 < 0)]
        dz = body.depths(z_) if len(z_) else np.array([np.nan])
        print(f"[s02c] HIDDEN_ZONE {PART_OBJ[part]}: verts s in [{mm(WRIST_CUT)},0) = {len(z_)} depth in body "
              f"min={mm(np.nanmin(dz))} n<0={int((dz < 0).sum())}"
              + f" (cover rule for s > {mm(WRIST_CUT + COVER_START)} mm: verts covering the SRC flare "
                f"outside the body tube are expected)")

    # ---- deviation (hidden / interface excluded)
    rng = np.random.default_rng(0)
    all_shells = [{"name": "body", "k": 0, "surf": body, "closed": True}]
    for part in world:
        all_shells += surf[part]
    src_union_V = np.vstack([src.V, club.V])
    src_union_T = np.vstack([src.T, club.T + len(src.V)])
    src_u = Surf(src_union_V, src_union_T)
    def in_grip(q):
        dq = q - gc
        sq = dq @ ga
        rq = np.linalg.norm(dq - np.outer(sq, ga), axis=1)
        return (rq <= GRIP_REGION_R) & (sq >= gapinfo[0]) & (sq <= gapinfo[1])

    print(f"[s02c] DEVIATION retopo part -> SRC_hi U SRC_club_hi (hidden >1 mm inside another retopo shell and "
          f"interface <0.1 mm excluded; grip design-change region excluded for every object: axis distance <= "
          f"{mm(GRIP_REGION_R)} mm and s in [{mm(gapinfo[0])}, {mm(gapinfo[1])}] mm about grip_r), mm:")
    for part in world:
        V_, T_ = world[part]
        pts, ti = sample_surface(V_, T_, N_DEV, rng)
        own = np.full(len(pts), -1)
        base = next(i for i, sh in enumerate(all_shells) if sh["name"] == part)
        for j, sh in enumerate(surf[part]):
            own[sh["tri_mask"][ti]] = base + j
        hidden, iface = occluded(pts, own, all_shells)
        grip = in_grip(pts)
        judged = ~hidden & ~iface & ~grip
        pj = pts[judged]
        d = np.array([src_u.dist(p) for p in pj])
        w = int(np.argmax(d))
        print(f"   {PART_OBJ[part]}: {stats(d)} (samples={len(pts)} hidden={int(hidden.sum())} "
              f"interface={int(iface.sum())} grip-region excluded={int((grip & ~hidden & ~iface).sum())}) "
              f"worst at {np.round(pj[w], 4).tolist()}")
    # reverse
    ret_V, ret_T, ret_lab = [bodyV], [bodyT], [np.full(len(bodyT), -1)]
    names = list(world)
    nv = len(bodyV)
    for k, part in enumerate(names):
        V_, T_ = world[part]
        ret_V.append(V_)
        ret_T.append(T_ + nv)
        ret_lab.append(np.full(len(T_), k))
        nv += len(V_)
    ret_u = Surf(np.vstack(ret_V), np.vstack(ret_T))
    ret_lab = np.concatenate(ret_lab)
    hi_sh = shells_of("SRC_hi", src.V, src.T)
    club_sh = shells_of("SRC_club_hi", club.V, club.T)
    src_shells = hi_sh + club_sh
    npts = 120000
    pts, ti = sample_surface(src_union_V, src_union_T, npts, rng)
    own = np.full(len(pts), -1)
    nt_hi = len(src.T)
    in_hi = ti < nt_hi
    for j, sh in enumerate(hi_sh):
        own[in_hi & sh["tri_mask"][np.minimum(ti, nt_hi - 1)]] = j
    tcl = np.maximum(ti - nt_hi, 0)
    for j, sh in enumerate(club_sh):
        own[~in_hi & sh["tri_mask"][tcl]] = len(hi_sh) + j
    hidden, iface = occluded(pts, own, src_shells)
    grip = in_grip(pts)
    judged = ~hidden & ~iface & ~grip
    dist = np.zeros(len(pts))
    lab = np.full(len(pts), -2)
    for i in np.nonzero(judged)[0].tolist():
        loc, _n, idx, dd = ret_u.bvh.find_nearest(Vector(pts[i]))
        dist[i] = dd
        lab[i] = ret_lab[idx]
    print(f"[s02c] DEVIATION SRC_hi U SRC_club_hi -> retopo union (samples={npts} hidden={int(hidden.sum())} "
          f"interface={int(iface.sum())} grip-region excluded={int((grip & ~hidden & ~iface).sum())}), by nearest "
          f"retopo object, mm:")
    for k, nm_ in [(-1, BODY_NAME)] + [(k, PART_OBJ[part]) for k, part in enumerate(names)]:
        m_ = judged & (lab == k)
        d_ = dist[m_]
        wl = np.round(pts[m_][int(np.argmax(d_))], 4).tolist() if len(d_) else None
        print(f"   {nm_}: {stats(d_)} worst at {wl}")
    print("[s02c] done")


if __name__ == "__main__":
    goblib.run_main(main)
