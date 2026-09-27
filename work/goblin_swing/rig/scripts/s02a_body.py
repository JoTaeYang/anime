"""s02a_body.py - T8: deforming torso shell (egg torso + shoulder / hip ring ports).

Input : rig/gob_r00_source.blend (SRC_hi, SRC_club_hi)
Output: rig/work/r01a_body.blend (SRC_hi, SRC_club_hi untouched + GOB_body_tmp in collection GOB)
        rig/work/loops_body.json, rig/work/r01a_body_front.png, rig/work/r01a_body_shoulder_r.png

Run:  bl.ps1 -Script s02a_body.py -Blend gob_r00_source.blend

Topology (script generated; quads, plus triangles only at the extra shoulder stations):
  * lat-long rows around the z axis (N_COL columns) at fixed z (row_z in main, printed as ROWS):
    BOTTOM_ROW_Z, LOW_ROW_Z, EGG_ROW_BELOW (egg just under the bottom lip), belt_bot_0 (BELT_ROW_INSET
    inside the belt band), 2 under-belt rows, belt_top_0 (BELT_ROW_INSET inside the band), spine_01_0
    (pivot z, egg just above the top lip), 4 rows, spine_02_0, 0.815, SH_ROWS_Z (shoulder blocks),
    TOP_ROWS_Z (egg up to the head crease), seam row.
  * bottom patch (M x M quad grid, boundary = row 0) holding the two hip ports.
  * top cap (M x M quad grid, boundary = seam row) closed under the head (hidden, not projected).
  * each port = a 4 x 2 face block of the grid replaced by concentric rings: k=0 = block boundary on
    the torso (12 verts; its 4 block corners are 5-poles), k=1 = crease, shoulders only: PORT_MID_RINGS
    unnamed rings, k=2 = open 12-vertex tube port at azimuth 30 m (T9 bridges a 12-gon tube to it).
    First ring vertex: +Z (shoulder) / -Y (hip) about the pivot axis.
  * shoulders: k0 / k1 / mid stations at optimised azimuths (within PORT_AZ_SWING of 30 m, gaps <=
    PORT_MAX_GAP, minimising the per-degree ring interpolation error) and PORT_INSERTS extra k1 / mid
    stations joined to k0 and k2 by one triangle each (apex = 5-pole) outside the bend sectors.
Ring placement: every ring vertex lies on the SRC_hi meridian section (plane through the pivot
axis at that azimuth).  The arm/leg-torso crease (no fillet) is captured by a ring:
  k2 = tube point at axial s = max(-5 mm, crease s + 10 mm);  k1 = the crease;  mids between them;
  k0 = RING_K0_GAP (per azimuth: shoulders 25 mm upper half -> 45 mm at the bottom; hips 25 mm) beyond
  the crease along the torso surface, capped at the head crease of that azimuth (HEAD_ZC - 1 mm), or,
  where the crease itself is at the head crease (within HEAD_CREASE_TOL: arm top running under the
  head), the same gap straight inward (hidden).
Projection: rows / shoulder-zone verts (z free: rows around the block within SH_ZONE and the top rows over
the block within SH_TOP_CYL) -> cylindrical projection r_T(theta, z) from a horizontal
ray table of SRC_hi (belt band [BELT_BAND_Z0, BELT_BAND_Z1] = between the lip creases replaced by a
Hermite under-belt surface, raw egg kept outside it; above the head crease (radius minimum) / over
the arm root by linear extrapolation of the egg); bottom patch -> ray from (0,0,0.55); cap, seam
row and top rows right over the arm root -> not projected (membrane, inside the head).
Report: SRC_TO_BODY = SRC_hi -> body deviation over the SRC area the body owns (src_to_body_report).
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
from mathutils import Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

MESH_NAME = "GOB_body_tmp"
COLL_NAME = "GOB"
OUT_BLEND = goblib.WORK / "r01a_body.blend"
OUT_LOOPS = goblib.WORK / "loops_body.json"
OUT_FRONT = goblib.WORK / "r01a_body_front.png"
OUT_SHR = goblib.WORK / "r01a_body_shoulder_r.png"

# ---------------------------------------------------------------- layout constants
N_COL = 56                  # lat-long columns (4 * M_CAP)
M_CAP = 14                  # bottom / top patch cells per side
N_RING = 12                 # port ring vertex count
# ring spacing (T16 adjusts only these)
# k0 arc beyond k1 (= crease) on the torso side: (upper half az 270..90, ring bottom az 180), blend
# upper + (bottom - upper) * max(0, -cos az) ** RING_K0_GAP_POW.  Shoulders: widened under the arm root
# (armpit: stretch zone when the arm is raised or moved forward); upper half unchanged (under the head,
# capped at the head crease anyway; front / back already 30..70 mm crease flare).
RING_K0_GAP = {"shoulder": (0.025, 0.045), "hip": (0.025, 0.025)}
RING_K0_GAP_POW = 1.0
K2_GAP = 0.010              # k2 at least this far out along the tube from the crease
K2_S_DEFAULT = -0.005       # k2 axial position (pivot axis) when the crease is farther in
HEAD_CREASE_TOL = 0.002     # port crease within this of the head crease z = arm top meeting the head

BELT_BOT = 0.4303           # pivots.json spine_01 method: belt bottom step
BELT_BAND_Z0 = 0.4285       # below every bottom lip crease of SRC_hi (min 0.4289, s02c measurement)
BELT_BAND_Z1 = 0.5790       # above every top lip crease of SRC_hi (max 0.5786)
BELT_ROW_INSET = 0.006      # belt_bot_0 / belt_top_0 rows this far inside the belt band (from BELT_BOT / spine_01)
EGG_ROW_BELOW = 0.4280      # row on the egg just below the bottom lip
BOTTOM_ROW_Z = 0.357        # lowest lat row (bottom patch boundary), between the hip ports and LOW_ROW_Z
LOW_ROW_Z = 0.392           # egg row between the bottom patch boundary and EGG_ROW_BELOW
TOP_ROWS_Z = (0.968,)       # rows between the last shoulder row and the seam: egg up to the head crease
HEAD_SEAM_ROW_Z = 0.990     # hidden seam row (above the head crease)
C_BOTTOM = np.array([0.0, 0.0, 0.55])   # ray origin for the bottom patch

# shoulder blocks: rows SH_ROWS_Z, columns c-2..c+2 ; hips: patch cols ci-2..ci+2, rows cj-1..cj+1
SH_ROWS_Z = (0.865, 0.915, 0.950)   # lattice rows holding the shoulder port blocks (ROW_A = first)
HIP_CI = {"hip_l": 11, "hip_r": 3}
HIP_CJ = 7
SH_C = {"shoulder_l": 0, "shoulder_r": N_COL // 2}
SH_ZONE = 5                 # |col - c| <= this in the row below SH_ROWS_Z and SH_ROWS_Z -> cylindrical (z free)
SH_HIDDEN = 1               # |col - c| <= this in the seam row -> hidden membrane
PORT_MID_RINGS = {"shoulder": 2, "hip": 0}   # unnamed rings between k1 (crease) and k2 (tube port)
PORT_MID_BIAS = 0.65        # mid arc fraction exponent (< 1: mids closer to the crease, where the arm-root flare is)
PORT_AZ_SWING = {"shoulder": 15, "hip": 0}   # k0 / k1 / mid station azimuth freedom around 30 m (deg)
PORT_MAX_GAP = 34           # max station gap (deg): tube polygon chord error at the arm top / bottom
PORT_INSERTS = {"shoulder_l": 4, "shoulder_r": 5, "hip_l": 0, "hip_r": 0}   # extra k1 / mid stations
SH_TOP_HIDDEN = 1           # |col - c| <= this in the TOP_ROWS_Z rows -> hidden membrane
SH_TOP_CYL = 2              # |col - c| <= this in the TOP_ROWS_Z rows -> z free (above the k0 top edge; rest
                            # self-intersection of the dome edge with the k0 strip when fixed at the row z)

TABLE_DTH = math.radians(0.5)
TABLE_Z0, TABLE_Z1, TABLE_DZ = 0.30, 1.02, 0.002

# ring k0 slot (grid position) for each azimuth index m (az = 30 m deg)
# shoulder: (row, col offset) ; hip: (patch i offset, patch j offset)
SH_SLOTS = [(2, 0), (2, -1), (2, -2), (1, -2), (0, -2), (0, -1), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1)]
HIP_SLOTS = [(0, -1), (-1, -1), (-2, -1), (-2, 0), (-2, 1), (-1, 1), (0, 1), (1, 1), (2, 1), (2, 0), (2, -1), (1, -1)]

PORTS = (("shoulder_l", (0.0, 0.0, 1.0)), ("shoulder_r", (0.0, 0.0, 1.0)),
         ("hip_l", (0.0, -1.0, 0.0)), ("hip_r", (0.0, -1.0, 0.0)))


def mm(x):
    return round(float(x) * 1000.0, 2)


# ---------------------------------------------------------------- source
class Src:
    def __init__(self):
        ob = bpy.data.objects.get("SRC_hi")
        if ob is None or ob.type != "MESH":
            raise RuntimeError("missing input object SRC_hi")
        if bpy.data.objects.get("SRC_club_hi") is None:
            raise RuntimeError("missing input object SRC_club_hi")
        me = ob.data
        mw = ob.matrix_world
        v = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", v)
        v = v.reshape(-1, 3)
        if not np.allclose(np.array(mw), np.eye(4), atol=1e-9):
            m = np.array(mw)
            v = v @ m[:3, :3].T + m[:3, 3]
        me.calc_loop_triangles()
        t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
        me.loop_triangles.foreach_get("vertices", t)
        self.V = v
        self.T = t.reshape(-1, 3)
        self.bvh = BVHTree.FromPolygons(v.tolist(), self.T.tolist())

    def nearest(self, p):
        loc, _n, _i, d = self.bvh.find_nearest(Vector(p))
        return (np.array(loc), d) if loc is not None else (None, None)

    def ray(self, o, d, dist=2.0):
        loc, _n, _i, dd = self.bvh.ray_cast(Vector(o), Vector(d), dist)
        return (np.array(loc), dd) if loc is not None else (None, None)

    def inside(self, p):
        """ray parity over 3 skew rays (majority)."""
        votes = 0
        for d in ((0.577, 0.577, 0.578), (-0.6, 0.3, 0.742), (0.2, -0.7, -0.686)):
            n = 0
            o = Vector(p)
            dv = Vector(d).normalized()
            for _ in range(64):
                loc, _nn, _i, dd = self.bvh.ray_cast(o, dv, 10.0)
                if loc is None:
                    break
                n += 1
                o = loc + dv * 1e-5
            votes += n % 2
        return votes >= 2


# ---------------------------------------------------------------- pivots
def load_pivots():
    return goblib.load_json("pivots.json")["pivots"]


def port_frame(piv, name, ref):
    p = piv[name]
    o = np.array(p["co"], float)
    a = np.array(p["axis"], float)
    a /= np.linalg.norm(a)
    ref = np.array(ref, float)
    u = ref - (ref @ a) * a
    u /= np.linalg.norm(u)
    w = np.cross(a, u)
    return o, a, u, w, float(p["radius"])


# ---------------------------------------------------------------- meridian walk
def section_walk(src, o, a, e, R, s_start=0.03):
    """Trace the SRC_hi section curve in the half plane (a, e) through o, starting on the tube at
    s = s_start and walking toward decreasing s (arm -> torso).  Returns resampled 3D polyline
    (0.5 mm steps) and its arc lengths."""
    n = np.cross(a, e)
    n /= np.linalg.norm(n)
    d = (src.V - o) @ n
    neg = d[src.T] < 0
    m = neg.any(1) & ~neg.all(1)
    TT = src.T[m]
    ek, pts, adj = {}, [], []
    for tri in TT:
        cr = []
        for x, y in ((0, 1), (1, 2), (2, 0)):
            i, j = int(tri[x]), int(tri[y])
            if (d[i] < 0) != (d[j] < 0):
                k = (i, j) if i < j else (j, i)
                if k not in ek:
                    ek[k] = len(pts)
                    uu = d[i] / (d[i] - d[j])
                    pts.append(src.V[i] + uu * (src.V[j] - src.V[i]))
                    adj.append([])
                cr.append(ek[k])
        if len(cr) == 2:
            adj[cr[0]].append(cr[1])
            adj[cr[1]].append(cr[0])
    pts = np.array(pts)
    S = (pts - o) @ a
    Rr = (pts - o) @ e
    cand = np.where((Rr > 0.5 * R) & (Rr < 1.8 * R))[0]
    i0 = int(cand[np.argmin(np.abs(S[cand] - s_start) + 0.5 * np.abs(Rr[cand] - R))])
    cur = min(adj[i0], key=lambda j: S[j])
    prev, path = i0, [i0, cur]
    for _ in range(50000):
        nx = [j for j in adj[cur] if j != prev]
        if not nx:
            break
        prev, cur = cur, nx[0]
        if cur == path[0]:
            break
        path.append(cur)
    p = pts[np.array(path)]
    L = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    q = np.arange(0.0, L[-1], 0.0005)
    out = np.stack([np.interp(q, L, p[:, k]) for k in range(3)], 1)
    return out, q


def at_arc(poly, L, x):
    x = min(max(x, 0.0), L[-1])
    return np.array([np.interp(x, L, poly[:, k]) for k in range(3)])


def ring_points(src, o, a, u, w, R, azd, n_mid, name):
    """Ring points of one port azimuth (deg): k2 (tube), k1 (crease), k0 (torso side), unnamed mids."""
    az = math.radians(azd)
    e = math.cos(az) * u + math.sin(az) * w
    poly, L = section_walk(src, o, a, e, R)
    s = (poly - o) @ a
    r = (poly - o) @ e
    k = 8  # 4 mm chord
    ds, dr = s[k:] - s[:-k], r[k:] - r[:-k]
    ang = np.degrees(np.arctan2(dr, -ds))
    hit = np.where((np.abs(ang) > 35.0) & (s[:-k] < 0.02))[0]
    if not len(hit):
        raise RuntimeError(f"{name} az {azd}: no crease found on the meridian walk")
    ic = int(hit[0] + k // 2)
    s_c = float(s[ic])
    Lcrease = float(L[ic])
    crease_p = poly[ic]
    # k2: on the tube, axial s = max(K2_S_DEFAULT, crease s + K2_GAP)
    s2 = max(K2_S_DEFAULT, s_c + K2_GAP)
    j2 = np.where(s[:ic + 1] <= s2)[0]
    L2 = float(L[j2[0]]) if len(j2) else 0.0
    Lc = Lcrease - L2
    # default k1 on the crease, k0 gap (RING_K0_GAP at this azimuth) beyond it on the torso surface, but not above the
    # head crease of that azimuth (HEAD_ZC - 1 mm); where the crease itself is with the head (within
    # HEAD_CREASE_TOL of the head crease z: arm top runs under the head) k0 goes straight inward (-a).
    zc_head = head_zc(crease_p)
    head_type = float(crease_p[2]) > zc_head - HEAD_CREASE_TOL
    p2, p1 = at_arc(poly, L, L2), crease_p.copy()
    g_top, g_bot = RING_K0_GAP[name.split("_")[0]]
    gap = g_top + (g_bot - g_top) * max(0.0, -math.cos(az)) ** RING_K0_GAP_POW
    if head_type:
        p0 = p1 - gap * a
        rule = "k1=crease,k0=inward(head)"
    else:
        if Lcrease + gap > L[-1] - 0.002:
            raise RuntimeError(f"{name} az {azd}: meridian walk too short ({L[-1]:.3f} m)")
        L0 = Lcrease + gap
        beyond = np.nonzero((L > Lcrease) & (L < L0))[0]
        capped = [j for j in beyond.tolist() if poly[j][2] > head_zc(poly[j]) - 0.001]
        if capped:
            L0 = float(L[capped[0]])
        p0 = at_arc(poly, L, L0)
        rule = "k1=crease,k0=torso" + (f"(capped at head crease, {mm(L0 - Lcrease)} mm)" if capped else "")
    # unnamed rings between k1 and k2, on the SRC section (arc fraction ((n-j)/(n+1))**PORT_MID_BIAS
    # from k2 toward k1; j = 0 next to k1)
    mids = [at_arc(poly, L, L2 + (Lcrease - L2) * ((n_mid - j) / (n_mid + 1)) ** PORT_MID_BIAS)
            for j in range(n_mid)]
    info = {"az": azd, "rule": rule, "crease_s_mm": mm(s_c), "crease_z": round(float(crease_p[2]), 4),
            "Lc_mm": mm(Lc), "k0_gap_mm": mm(gap)}
    return p0, p1, mids, p2, info


def choose_azimuths(dense, n, swing):
    """Station azimuths phi_m in [30 m - swing, 30 m + swing] (1 deg) minimising the max distance between
    the per-degree k1 / mid / k0 points and their linear interpolation between neighbouring stations."""
    P = np.array([np.vstack([d[0], d[1]] + d[2]) for d in dense])     # (360, K, 3)

    def seg_err(pa, pb):
        if pb <= pa:
            pb += 360
        ts = np.arange(pa + 1, pb)
        if not len(ts):
            return 0.0
        A_, B_ = P[pa % 360], P[pb % 360]
        t = ((ts - pa) / (pb - pa))[:, None, None]
        return float(np.linalg.norm(A_[None] * (1 - t) + B_[None] * t - P[ts % 360], axis=2).max())

    phi = [30 * m for m in range(n)]
    for _ in range(6):
        changed = False
        for m in range(n):
            best = None
            for cand in range(30 * m - swing, 30 * m + swing + 1):
                c = cand % 360
                g0 = (c - phi[m - 1]) % 360
                g1 = (phi[(m + 1) % n] - c) % 360
                if max(g0, g1) > PORT_MAX_GAP and max(g0, g1) > max((phi[m] - phi[m - 1]) % 360,
                                                                    (phi[(m + 1) % n] - phi[m]) % 360):
                    continue
                e_ = max(seg_err(phi[m - 1], c), seg_err(c, phi[(m + 1) % n]))
                if best is None or e_ < best[0] - 1e-9:
                    best = (e_, c)
            if best[1] != phi[m]:
                phi[m] = best[1]
                changed = True
        if not changed:
            break
    errs = [seg_err(phi[m], phi[(m + 1) % n]) for m in range(n)]
    uni = [seg_err(30 * m, 30 * ((m + 1) % n)) for m in range(n)]
    return phi, max(errs), max(uni)


def port_rings(src, piv, name, ref):
    """k0 / k1 / mids at station azimuths (shoulders: optimised within +-PORT_AZ_SWING of 30 m; hips
    30 m); k2 always at 30 m (tube phase)."""
    o, a, u, w, R = port_frame(piv, name, ref)
    kind_ = name.split("_")[0]
    n_mid = PORT_MID_RINGS[kind_]
    rings = {0: [], 1: [], 2: []}
    rings.update({("m", j): [] for j in range(n_mid)})
    info = []
    if PORT_AZ_SWING[kind_]:
        dense = [ring_points(src, o, a, u, w, R, azd, n_mid, name) for azd in range(360)]
        phi, err_opt, err_uni = choose_azimuths(dense, N_RING, PORT_AZ_SWING[kind_])
    else:
        dense = None
        phi, err_opt, err_uni = [30 * m for m in range(N_RING)], None, None
    for m in range(N_RING):
        p0, p1, mids, _p2, inf = dense[phi[m]] if dense else ring_points(src, o, a, u, w, R, phi[m], n_mid, name)
        p2 = dense[30 * m][3] if dense else _p2
        for kk, pp in ((2, p2), (1, p1), (0, p0)):
            rings[kk].append(pp)
        for j in range(n_mid):
            rings[("m", j)].append(mids[j])
        inf = dict(inf)
        inf["k2_az"] = 30 * m
        info.append(inf)
    for kk in rings:
        rings[kk] = np.array(rings[kk])
    ins = {}
    if dense and PORT_INSERTS[name]:
        ins = choose_inserts(dense, phi, PORT_INSERTS[name], n_mid)
        for m, d in ins.items():
            p0, p1, mids, _p2, _inf = dense[d["az"]]
            d[1] = p1
            for j in range(n_mid):
                d[("m", j)] = mids[j]
    rings["ins"] = ins
    return rings, info, (o, a, u, w, R), (phi, err_opt, err_uni)


def in_bend_band(azd):
    a_ = azd % 360.0
    return min(a_, 360.0 - a_) <= 30.0 or abs(a_ - 180.0) <= 30.0


def choose_inserts(dense, phi, n_ins, n_mid):
    """Extra k1 / mid stations (triangle transitions to k0 and k2, apex verts = valence-5 poles) in the
    segments with the largest k1 / mid interpolation error; the X station and both apexes (k0 at its
    station azimuth, k2 at 30 m) must lie outside the bend bands (+-30 deg of 0 and 180)."""
    P = np.array([np.vstack([d[1]] + d[2]) for d in dense])     # k1 + mids

    def seg_err(pa, pb):
        if pb <= pa:
            pb += 360
        ts = np.arange(pa + 1, pb)
        if not len(ts):
            return 0.0
        t = ((ts - pa) / (pb - pa))[:, None, None]
        return float(np.linalg.norm(P[pa % 360][None] * (1 - t) + P[pb % 360][None] * t - P[ts % 360], axis=2).max())

    n = len(phi)
    ins = {}
    for _ in range(n_ins):
        best = None
        for m in range(n):
            if m in ins:
                continue
            m1 = (m + 1) % n
            a0 = [j for j in (m, m1) if not in_bend_band(phi[j])]
            a2 = [j for j in (m, m1) if not in_bend_band(30.0 * j)]
            if not a0 or not a2:
                continue
            pa, pb = phi[m], phi[m1] if phi[m1] > phi[m] else phi[m1] + 360
            e_seg = seg_err(pa, pb)
            if best is not None and e_seg <= best[0]:
                continue
            bx = None
            for x in range(pa + 2, pb - 1):
                if in_bend_band(x):
                    continue
                e_ = max(seg_err(pa, x), seg_err(x, pb))
                if bx is None or e_ < bx[0]:
                    bx = (e_, x % 360)
            if bx is None:
                continue
            apex0 = max(a0, key=lambda j: -abs((phi[j] % 180) - 90))
            apex2 = max(a2, key=lambda j: -abs(((30.0 * j) % 180) - 90))
            best = (e_seg, m, bx[1], apex0, apex2, bx[0])
        if best is None:
            break
        _e, m, x, apex0, apex2, e_after = best
        ins[m] = {"az": x, "apex_k0": apex0, "apex_k2": apex2, "err_before": _e, "err_after": e_after}
    return ins


def head_zc(p):
    """head / torso crease z (HEAD_ZC table, 1 deg) at the azimuth of p around the z axis."""
    th = math.degrees(math.atan2(p[1], p[0])) % 360.0
    return float(np.interp(th, np.arange(360.0), HEAD_ZC, period=360.0))


# ---------------------------------------------------------------- cylindrical target table
class RTable:
    def __init__(self, src, piv):
        self.th = np.arange(0.0, 2 * math.pi, TABLE_DTH)
        self.z = np.arange(TABLE_Z0, TABLE_Z1 + 1e-9, TABLE_DZ)
        nth, nz = len(self.th), len(self.z)
        r = np.full((nth, nz), np.nan)
        hp = np.full((nth, nz, 3), np.nan)
        ray = src.bvh.ray_cast
        for i, t in enumerate(self.th):
            dvec = Vector((math.cos(t), math.sin(t), 0.0))
            for jz, z in enumerate(self.z):
                loc, _n, _f, dist = ray(Vector((0.0, 0.0, z)), dvec, 0.6)
                if loc is not None:
                    r[i, jz] = dist
                    hp[i, jz] = loc
        # hits on the arm tubes (incl. the root region) are not torso
        for n in ("shoulder_l", "shoulder_r"):
            oa, aa, _u, _w, Ra = port_frame(piv, n, (0, 0, 1))
            d = hp - oa
            ss = d @ aa
            rad = np.linalg.norm(d - ss[..., None] * aa, axis=-1)
            r[(ss > -0.09) & (rad < Ra + 0.006)] = np.nan
        self.raw = r.copy()
        zb, zt = BELT_BOT, float(piv["spine_01"]["co"][2])
        self.zb, self.zt = zb, zt
        self.valid_top = np.zeros(nth)
        for i in range(nth):
            col = r[i]
            # belt band -> Hermite under-belt surface between the lip creases (egg kept outside the band;
            # inside it the raw hit is used wherever it is not the belt, i.e. not > Hermite + 1 mm)
            z0, z1 = BELT_BAND_Z0, BELT_BAND_Z1
            r0, r0b = self._at(col, z0), self._at(col, z0 - 0.010)
            r1, r1b = self._at(col, z1), self._at(col, z1 + 0.010)
            m0, m1 = (r0 - r0b) / 0.010, (r1b - r1) / 0.010
            h = z1 - z0
            for jz, z in enumerate(self.z):
                if z0 < z < z1:
                    tt = (z - z0) / h
                    h00, h10 = 2 * tt ** 3 - 3 * tt ** 2 + 1, tt ** 3 - 2 * tt ** 2 + tt
                    h01, h11 = -2 * tt ** 3 + 3 * tt ** 2, tt ** 3 - tt ** 2
                    rv = h00 * r0 + h10 * h * m0 + h01 * r1 + h11 * h * m1
                    if np.isfinite(col[jz]) and col[jz] < rv + 0.001:
                        rv = col[jz]
                    col[jz] = rv
            # top: first invalid cell or head crease (r increasing) above z 0.80 -> extrapolate
            j80 = int(np.searchsorted(self.z, 0.80))
            jtop = nz
            rmin, jmin = np.inf, j80
            for jz in range(j80, nz):
                if not np.isfinite(col[jz]):
                    jtop = jz
                    break
                if col[jz] > rmin + 0.0015:
                    jtop = max(j80 + 1, jmin + 1)     # raw kept up to the crease (radius minimum)
                    break
                if col[jz] < rmin:
                    rmin, jmin = col[jz], jz
            jtop = max(jtop, j80 + 2)
            ja, jb = max(jtop - 7, j80), jtop - 1
            zz, rr = self.z[ja:jb + 1], col[ja:jb + 1]
            ok = np.isfinite(rr)
            if ok.sum() >= 2:
                slope, icpt = np.polyfit(zz[ok], rr[ok], 1)
            else:
                slope, icpt = 0.0, col[jtop - 1]
            slope = min(slope, 0.0)
            base_z, base_r = self.z[jtop - 1], col[jtop - 1]
            for jz in range(jtop, nz):
                col[jz] = max(0.02, base_r + slope * (self.z[jz] - base_z))
            self.valid_top[i] = base_z
            # fill holes below (should not happen above row 0) by nearest valid
            bad = ~np.isfinite(col)
            if bad.any():
                good = np.where(~bad)[0]
                for jz in np.where(bad)[0]:
                    col[jz] = col[good[np.argmin(np.abs(good - jz))]]
            r[i] = col
        self.r = r

    def _at(self, col, z):
        jz = (z - TABLE_Z0) / TABLE_DZ
        j0 = int(math.floor(jz))
        f = jz - j0
        a, b = col[j0], col[j0 + 1]
        if not np.isfinite(a):
            return b
        if not np.isfinite(b):
            return a
        return a * (1 - f) + b * f

    def lookup(self, th, z):
        th = np.mod(np.asarray(th, float), 2 * math.pi)
        z = np.clip(np.asarray(z, float), TABLE_Z0, TABLE_Z1 - 1e-6)
        fi = th / TABLE_DTH
        i0 = np.floor(fi).astype(int) % len(self.th)
        i1 = (i0 + 1) % len(self.th)
        ft = fi - np.floor(fi)
        fz = (z - TABLE_Z0) / TABLE_DZ
        j0 = np.minimum(np.floor(fz).astype(int), len(self.z) - 2)
        gz = fz - j0
        r00, r01 = self.r[i0, j0], self.r[i0, j0 + 1]
        r10, r11 = self.r[i1, j0], self.r[i1, j0 + 1]
        return (r00 * (1 - gz) + r01 * gz) * (1 - ft) + (r10 * (1 - gz) + r11 * gz) * ft

    def point(self, th, z):
        rr = self.lookup(th, z)
        th = np.asarray(th, float)
        return np.stack([rr * np.cos(th), rr * np.sin(th), np.asarray(z, float) * np.ones_like(rr)], -1)


# ---------------------------------------------------------------- topology
FIXED, ROW, CYL, RAY, HIDDEN = 0, 1, 2, 3, 4


class Builder:
    def __init__(self):
        self.P = []       # positions
        self.kind = []
        self.zrow = []
        self.faces = []

    def add(self, p, kind, zr=np.nan):
        self.P.append(np.array(p, float))
        self.kind.append(kind)
        self.zrow.append(zr)
        return len(self.P) - 1


def square_boundary(M):
    """patch boundary (i, j) in ccw order starting at (M, M/2) (theta = 0)."""
    h = M // 2
    seq = [(M, j) for j in range(h, M)]
    seq += [(i, M) for i in range(M, 0, -1)]
    seq += [(0, j) for j in range(M, 0, -1)]
    seq += [(i, 0) for i in range(0, M)]
    seq += [(M, j) for j in range(0, h)]
    assert len(seq) == 4 * M
    return seq


def sq2disk(i, j, M):
    uu, vv = 2.0 * i / M - 1.0, 2.0 * j / M - 1.0
    return uu * math.sqrt(max(0.0, 1 - vv * vv / 2)), vv * math.sqrt(max(0.0, 1 - uu * uu / 2))


def build_topology(rt, row_z, port):
    b = Builder()
    nrow = len(row_z)
    seam = nrow - 1
    ROW_A = next(i for i, z in enumerate(row_z) if abs(z - SH_ROWS_Z[0]) < 1e-9)
    removed_lat = set()
    # ---- shoulder blocks: ring k0 occupies block boundary
    sh_ring_slots = {}
    for name, c in SH_C.items():
        slots = []
        for (dr, dc) in SH_SLOTS:
            slots.append((ROW_A + dr, (c + dc) % N_COL))
        sh_ring_slots[name] = slots
        for dc in (-1, 0, 1):
            removed_lat.add((ROW_A + 1, (c + dc) % N_COL))
    L = {}
    for rI in range(nrow):
        for j in range(N_COL):
            if (rI, j) in removed_lat:
                continue
            th = 2 * math.pi * j / N_COL
            z = row_z[rI]
            kind = ROW
            for name, c in SH_C.items():
                dc = min((j - c) % N_COL, (c - j) % N_COL)
                if ROW_A - 1 <= rI <= ROW_A + 2 and dc <= SH_ZONE:
                    kind = CYL
                if ROW_A + 2 < rI < seam and dc <= SH_TOP_CYL:
                    kind = CYL                              # z free: top rows over the block stay above k0
                if (rI == seam and dc <= SH_HIDDEN) or (ROW_A + 2 < rI < seam and dc <= SH_TOP_HIDDEN):
                    kind = HIDDEN                           # seam / top rows over the arm root: membrane
            L[(rI, j)] = b.add(rt.point(th, z), kind, z)
    for name, slots in sh_ring_slots.items():
        for m, key in enumerate(slots):
            vi = L[key]
            b.P[vi] = port[name][0][m]
            b.kind[vi] = FIXED
    # lat faces
    for rI in range(nrow - 1):
        for j in range(N_COL):
            j1 = (j + 1) % N_COL
            skip = False
            for name, c in SH_C.items():
                if rI in (ROW_A, ROW_A + 1) and ((j - (c - 2)) % N_COL) < 4:
                    skip = True
            if skip:
                continue
            b.faces.append((L[(rI, j)], L[(rI, j1)], L[(rI + 1, j1)], L[(rI + 1, j)]))
    # ---- bottom patch
    M = M_CAP
    bnd = square_boundary(M)
    Pb = {}
    for k, ij in enumerate(bnd):
        Pb[ij] = L[(0, k)]
    hip_slot_ij = {}
    removed_p = set()
    for name, ci in HIP_CI.items():
        hip_slot_ij[name] = [(ci + di, HIP_CJ + dj) for (di, dj) in HIP_SLOTS]
        for di in (-1, 0, 1):
            removed_p.add((ci + di, HIP_CJ))
    r0 = float(np.mean([np.linalg.norm(b.P[L[(0, j)]][:2]) for j in range(N_COL)]))
    for i in range(1, M):
        for j in range(1, M):
            if (i, j) in removed_p:
                continue
            x, y = sq2disk(i, j, M)
            rho2 = x * x + y * y
            tgt = np.array([r0 * x, r0 * y, row_z[0] - 0.12 * (1 - rho2)])
            loc, _ = SRC.ray(C_BOTTOM, tgt - C_BOTTOM)
            Pb[(i, j)] = b.add(loc if loc is not None else tgt, RAY)
    for name, slots in hip_slot_ij.items():
        for m, key in enumerate(slots):
            vi = Pb[key]
            b.P[vi] = port[name][0][m]
            b.kind[vi] = FIXED
    for i in range(M):
        for j in range(M):
            skip = False
            for name, ci in HIP_CI.items():
                if ci - 2 <= i < ci + 2 and HIP_CJ - 1 <= j < HIP_CJ + 1:
                    skip = True
            if skip:
                continue
            b.faces.append((Pb[(i, j)], Pb[(i, j + 1)], Pb[(i + 1, j + 1)], Pb[(i + 1, j)]))
    # ---- top cap
    Pt = {}
    for k, ij in enumerate(bnd):
        Pt[ij] = L[(seam, k)]
    rs = float(np.mean([np.linalg.norm(b.P[L[(seam, j)]][:2]) for j in range(N_COL)]))
    for i in range(1, M):
        for j in range(1, M):
            x, y = sq2disk(i, j, M)
            Pt[(i, j)] = b.add((rs * x, rs * y, row_z[seam] + 0.01 * (1 - x * x - y * y)), HIDDEN)
    for i in range(M):
        for j in range(M):
            b.faces.append((Pt[(i, j)], Pt[(i + 1, j)], Pt[(i + 1, j + 1)], Pt[(i, j + 1)]))
    # ---- port strips
    ring_idx = {}
    for name, _ref in PORTS:
        if name in SH_C:
            k0 = [L[key] for key in sh_ring_slots[name]]
        else:
            k0 = [Pb[key] for key in hip_slot_ij[name]]
        ins = port[name].get("ins", {})        # {m: {"az", "apex_k0", "apex_k2", 1: p, ("m", j): p}}
        mid_keys = sorted(k_ for k_ in port[name] if isinstance(k_, tuple))

        def ring_with_x(key):
            base = [b.add(p, FIXED) for p in port[name][key]]
            xs = {m: b.add(d[key], FIXED) for m, d in ins.items()}
            return base, xs

        k1 = ring_with_x(1)
        mids = [ring_with_x(key) for key in mid_keys]
        k2 = [b.add(p, FIXED) for p in port[name][2]]
        chain = [(k0, {})] + [k1] + mids + [(k2, {})]
        for (ra, xa), (rb, xb) in zip(chain, chain[1:]):
            for m in range(N_RING):
                m1 = (m + 1) % N_RING
                if (m in xa) == (m in xb):
                    if m in xa:
                        b.faces.append((ra[m], xa[m], xb[m], rb[m]))
                        b.faces.append((xa[m], ra[m1], rb[m1], xb[m]))
                    else:
                        b.faces.append((ra[m], ra[m1], rb[m1], rb[m]))
                elif m in xb:      # ra = k0 side without the extra vertex: triangle apex on ra
                    if ins[m]["apex_k0"] == m:
                        b.faces.append((ra[m], xb[m], rb[m]))
                        b.faces.append((ra[m], ra[m1], rb[m1], xb[m]))
                    else:
                        b.faces.append((ra[m1], rb[m1], xb[m]))
                        b.faces.append((ra[m], ra[m1], xb[m], rb[m]))
                else:              # rb = k2 side without the extra vertex: triangle apex on rb
                    if ins[m]["apex_k2"] == m:
                        b.faces.append((ra[m], xa[m], rb[m]))
                        b.faces.append((xa[m], ra[m1], rb[m1], rb[m]))
                    else:
                        b.faces.append((xa[m], ra[m1], rb[m1]))
                        b.faces.append((ra[m], xa[m], rb[m1], rb[m]))
        k1_loop = []
        for m in range(N_RING):
            k1_loop.append(k1[0][m])
            if m in k1[1]:
                k1_loop.append(k1[1][m])
        ring_idx[name] = [k0, k1_loop, k2]
    rows = {rI: [L[(rI, j)] for j in range(N_COL)] for rI in range(nrow)
            if all((rI, j) in L for j in range(N_COL))}
    return b, ring_idx, rows


# ---------------------------------------------------------------- relaxation
def relax(b, rt, iters=400, lam=0.6):
    X = np.array(b.P)
    kind = np.array(b.kind)
    zrow = np.array(b.zrow)
    n = len(X)
    nb = [set() for _ in range(n)]
    for f in b.faces:
        for q in range(len(f)):
            a_, c_ = f[q], f[(q + 1) % len(f)]
            nb[a_].add(c_)
            nb[c_].add(a_)
    src_i = np.concatenate([np.full(len(s), i) for i, s in enumerate(nb)])
    dst_i = np.concatenate([np.array(sorted(s), dtype=int) for s in nb])
    deg = np.array([len(s) for s in nb], float)
    m_row, m_cyl = kind == ROW, kind == CYL
    m_ray = np.where(kind == RAY)[0]
    free = kind != FIXED
    last = 0.0
    for _it in range(iters):
        acc = np.zeros_like(X)
        np.add.at(acc, src_i, X[dst_i])
        avg = acc / deg[:, None]
        Y = X.copy()
        Y[free] = X[free] + lam * (avg[free] - X[free])
        th = np.arctan2(Y[:, 1], Y[:, 0])
        if m_row.any():
            Y[m_row] = rt.point(th[m_row], zrow[m_row])
        if m_cyl.any():
            Y[m_cyl] = rt.point(th[m_cyl], Y[m_cyl, 2])
        for vi in m_ray:
            loc, _ = SRC.ray(C_BOTTOM, Y[vi] - C_BOTTOM)
            if loc is not None:
                Y[vi] = loc
        last = float(np.abs(Y - X).max())
        X = Y
    return X, last


# ---------------------------------------------------------------- mesh object
def make_object(X, faces):
    me = bpy.data.meshes.new(MESH_NAME)
    me.from_pydata([tuple(p) for p in X], [], [tuple(f) for f in faces])
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()
    coll = bpy.data.collections.get(COLL_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLL_NAME)
        bpy.context.scene.collection.children.link(coll)
    old = bpy.data.objects.get(MESH_NAME)
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    ob = bpy.data.objects.new(MESH_NAME, me)
    coll.objects.link(ob)
    return ob


def mesh_arrays(ob):
    me = ob.data
    X = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", X)
    X = X.reshape(-1, 3)
    E = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    return X, E


# ---------------------------------------------------------------- reports
def head_crease_table():
    """z of the head / torso (or arm top) crease per degree: argmin of the horizontal ray distance from the
    z axis over z in [0.935, 0.995] (1 mm steps)."""
    zs = np.arange(0.935, 0.9951, 0.001)
    zc = np.zeros(360)
    for i in range(360):
        th = math.radians(i)
        dv = Vector((math.cos(th), math.sin(th), 0.0))
        best = (np.inf, zs[0])
        for z in zs:
            loc, _n, _f, dd = SRC.bvh.ray_cast(Vector((0.0, 0.0, float(z))), dv, 1.0)
            if loc is not None and dd < best[0]:
                best = (dd, float(z))
        zc[i] = best[1]
    return zc


def src_to_body_report(X, faces, rt, piv, ring_idx, frames, n_samples=200000):
    """SRC_hi -> body surface deviation over the SRC area the body owns (area-weighted samples).
    Excluded (other parts / not this stage): beyond each port's outer ring k2 (limb tubes, azimuth-
    interpolated k2 axial position, radius < tube + 30 mm; or axial > 120 mm: hands / shoes), head (z above the head crease of that azimuth with face normal z
    < 0.5, or z > 0.995), belt (z in [BELT_BAND_Z0, BELT_BAND_Z1] and horizontal radius > under-belt
    surface + 2 mm, or a horizontal face: lip underside / top, |normal z| > 0.7)."""
    rng = np.random.default_rng(0)
    V, T = SRC.V, SRC.T
    a_, b_, c_ = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    cr = np.cross(b_ - a_, c_ - a_)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    ti = rng.choice(len(T), size=n_samples, p=area / area.sum())
    r1 = np.sqrt(rng.random(n_samples))
    r2 = rng.random(n_samples)
    P = (1 - r1)[:, None] * a_[ti] + (r1 * (1 - r2))[:, None] * b_[ti] + (r1 * r2)[:, None] * c_[ti]
    nz = cr[ti, 2] / np.maximum(np.linalg.norm(cr[ti], axis=1), 1e-12)
    keep = np.ones(n_samples, bool)
    why = {}
    for name, ref in PORTS:
        o, a, u, w, R = frames[name]
        k2 = X[np.array(ring_idx[name][2])]
        s2, _r2, az2 = azimuth(k2, o, a, np.array(ref))
        s_p, r_p, az_p = azimuth(P, o, a, np.array(ref))
        oz = np.argsort(az2)
        s_lim = np.interp(az_p, az2[oz], s2[oz], period=360.0)
        m = ((s_p > s_lim) & (r_p < R + 0.03)) | (s_p > 0.12)
        why["limb_" + name] = int((m & keep).sum())
        keep &= ~m
    zc = HEAD_ZC
    th = np.degrees(np.mod(np.arctan2(P[:, 1], P[:, 0]), 2 * math.pi))
    zc_p = np.interp(th, np.arange(360.0), zc, period=360.0)
    m = ((P[:, 2] > zc_p) & (nz < 0.5)) | ((P[:, 2] > zc_p - 0.010) & (nz < -0.3)) | (P[:, 2] > 0.995)
    why["head"] = int((m & keep).sum())
    keep &= ~m
    rh = np.hypot(P[:, 0], P[:, 1])
    band = (P[:, 2] > BELT_BAND_Z0) & (P[:, 2] < BELT_BAND_Z1)
    m = band & ((rh > rt.lookup(np.radians(th), P[:, 2]) + 0.002) | (nz < -0.7) | (nz > 0.7))
    why["belt"] = int((m & keep).sum())
    keep &= ~m
    tris = []
    for f in faces:
        for k in range(1, len(f) - 1):
            tris.append((f[0], f[k], f[k + 1]))
    bvh = BVHTree.FromPolygons([tuple(p) for p in X], tris)
    idx = np.nonzero(keep)[0]
    d = np.array([bvh.find_nearest(Vector(P[i]))[3] for i in idx])
    return P[idx], d, why


def topo_report(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.edges.ensure_lookup_table()
    boundary = [e for e in bm.edges if len(e.link_faces) == 1]
    nonman = [e for e in bm.edges if len(e.link_faces) > 2]
    wire = [e for e in bm.edges if len(e.link_faces) == 0]
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for e in boundary:
        a_, c_ = find(e.verts[0].index), find(e.verts[1].index)
        parent[a_] = c_
    loops = {}
    for e in boundary:
        loops.setdefault(find(e.verts[0].index), []).append(e)
    loop_sizes = sorted(len(v) for v in loops.values())
    tris = sum(len(f.verts) - 2 for f in bm.faces)
    nquads = sum(1 for f in bm.faces if len(f.verts) == 4)
    # orientation: faces within 3 mm of SRC_hi -> face normal vs nearest SRC_hi normal
    near, agree = 0, 0
    for f in bm.faces:
        loc, nrm, _i, dd = SRC.bvh.find_nearest(f.calc_center_median())
        if loc is not None and dd < 0.003:
            near += 1
            agree += 1 if f.normal.dot(nrm) > 0 else 0
    rep = {"verts": len(bm.verts), "faces": len(bm.faces), "quads": nquads, "tris": tris,
           "boundary_edges": len(boundary), "nonmanifold_edges": len(nonman), "wire_edges": len(wire),
           "open_boundary_loops": len(loops), "open_loop_sizes": loop_sizes,
           "faces_near_src": near, "faces_normal_agree_src": agree}
    bm.free()
    return rep


def azimuth(P, o, a, ref):
    d = P - o
    s = d @ a
    rad = d - s[:, None] * a
    u = ref - (ref @ a) * a
    u /= np.linalg.norm(u)
    w = np.cross(a, u)
    return s, np.linalg.norm(rad, axis=1), np.degrees(np.arctan2(rad @ w, rad @ u))


def main():
    global SRC
    piv = load_pivots()
    SRC = Src()
    print(f"[s02a] SRC_hi verts={len(SRC.V)} tris={len(SRC.T)}")

    # rows
    z_sp1 = float(piv["spine_01"]["co"][2])
    z_sp2 = float(piv["spine_02"]["co"][2])
    z_bb, z_bt = BELT_BOT + BELT_ROW_INSET, z_sp1 - BELT_ROW_INSET
    row_z = [BOTTOM_ROW_Z, LOW_ROW_Z, EGG_ROW_BELOW, z_bb]
    row_z += list(np.linspace(z_bb, z_bt, 4)[1:])                 # 2 under-belt rows + belt_top
    row_z += [z_sp1]                                              # spine_01 (egg just above the top lip)
    row_z += list(np.linspace(z_sp1, z_sp2, 6)[1:])               # 4 rows + spine_02
    row_z += [0.815] + list(SH_ROWS_Z) + list(TOP_ROWS_Z) + [HEAD_SEAM_ROW_Z]
    row_z = [float(z) for z in row_z]
    def ri(z):
        return next(i for i, zz in enumerate(row_z) if abs(zz - z) < 1e-12)
    row_name = {ri(z_bb): ["belt_bot_0"], ri(z_bt): ["belt_top_0"], ri(z_sp1): ["spine_01_0"], ri(z_sp2): ["spine_02_0"]}
    print(f"[s02a] ROWS z (mm): {[mm(z) for z in row_z]} names={row_name}")

    global HEAD_ZC
    HEAD_ZC = head_crease_table()
    print(f"[s02a] head crease z per azimuth (mm): min={mm(HEAD_ZC.min())} max={mm(HEAD_ZC.max())} every 30 deg "
          f"{[mm(z) for z in HEAD_ZC[::30]]}")
    # port rings on the source surface
    port, port_info, frames = {}, {}, {}
    for name, ref in PORTS:
        rings, info, fr, azs = port_rings(SRC, piv, name, ref)
        port[name] = rings
        port_info[name] = info
        frames[name] = fr
        print(f"[s02a] {name} rings (k0/k1/mid station az -> k2 az): " + ", ".join(
            f"az{d['az']}->{d['k2_az']}:{d['rule']}(Lc={d['Lc_mm']},gap={d['k0_gap_mm']})" for d in info))
        if rings["ins"]:
            print(f"[s02a] {name} extra k1/mid stations: " + ", ".join(
                f"seg {m} az {d['az']} (apex k0 st {d['apex_k0']}, k2 st {d['apex_k2']}, err {mm(d['err_before'])}"
                f"->{mm(d['err_after'])} mm)" for m, d in sorted(rings["ins"].items())))
        if azs[1] is not None:
            print(f"[s02a] {name} station azimuths {azs[0]}: max ring interpolation err {mm(azs[1])} mm "
                  f"(uniform 30 deg: {mm(azs[2])} mm)")

    rt = RTable(SRC, piv)
    print(f"[s02a] r table {rt.r.shape}, belt band [{rt.zb:.4f},{rt.zt:.4f}], "
          f"valid_top z min/median/max = {rt.valid_top.min():.3f}/{np.median(rt.valid_top):.3f}/{rt.valid_top.max():.3f}")

    b, ring_idx, rows = build_topology(rt, row_z, port)
    X, last = relax(b, rt)
    print(f"[s02a] relaxation: {len(X)} verts, last max step {mm(last)} mm")

    ob = make_object(X, b.faces)
    Xo, E = mesh_arrays(ob)

    # ---------------- loops json
    rings_json = {}
    for name, _ref in PORTS:
        for k in range(3):
            rings_json[f"{name}_{k}"] = {"verts": [int(v) for v in ring_idx[name][k]], "joint": name, "k": k,
                                         "center": False}
    for rI, names in row_name.items():
        for nm_ in names:
            rings_json[nm_] = {"verts": [int(v) for v in rows[rI]], "joint": nm_[:-2], "k": 0, "center": False}
    OUT_LOOPS.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_LOOPS, "w", encoding="utf-8") as f:
        json.dump({"mesh": MESH_NAME, "rings": rings_json}, f, indent=1)

    # ---------------- save blend (input file untouched: copy; no .blend1 backup)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)

    # ---------------- renders
    goblib.ortho_render([ob], "front", OUT_FRONT, wire=True)
    so = np.array(piv["shoulder_r"]["co"])
    goblib.ortho_render([ob], "three_quarter", OUT_SHR, wire=True,
                        frame_bbox=((so[0] - 0.17, so[1] - 0.17, so[2] - 0.17),
                                    (so[0] + 0.17, so[1] + 0.17, so[2] + 0.13)))

    # ---------------- reports
    topo = topo_report(ob)
    print("[s02a] OUTPUT " + json.dumps({"blend": str(OUT_BLEND), "loops": str(OUT_LOOPS),
                                         "renders": [str(OUT_FRONT), str(OUT_SHR)]}))
    print(f"[s02a] TRIS {topo['tris']} (quads {topo['quads']}, faces {topo['faces']}, verts {topo['verts']})")
    print(f"[s02a] OPEN_BOUNDARY_LOOPS {topo['open_boundary_loops']} sizes={topo['open_loop_sizes']} "
          f"boundary_edges={topo['boundary_edges']}")
    print(f"[s02a] NONMANIFOLD_EDGES {topo['nonmanifold_edges']} wire_edges={topo['wire_edges']} "
          f"orientation: {topo['faces_normal_agree_src']}/{topo['faces_near_src']} faces within 3 mm of SRC_hi "
          f"agree with the SRC_hi normal")

    # ring closedness + sizes
    ekeys = set((np.minimum(E[:, 0], E[:, 1]) * len(Xo) + np.maximum(E[:, 0], E[:, 1])).tolist())
    print("[s02a] RINGS (name: n_verts closed_loop center_z_mm):")
    for nm_, r in rings_json.items():
        vs = np.array(r["verts"])
        nxt = np.roll(vs, -1)
        closed = all(((min(x, y) * len(Xo) + max(x, y)) in ekeys) for x, y in zip(vs.tolist(), nxt.tolist()))
        print(f"   {nm_}: n={len(vs)} closed={closed} center_z={mm(Xo[vs, 2].mean())}")

    # outer port ring radius / phase
    for name, ref in PORTS:
        o, a, u, w, R = frames[name]
        vs = np.array(ring_idx[name][2])
        s_, r_, az_ = azimuth(Xo[vs], o, a, np.array(ref))
        print(f"[s02a] PORT {name}_2: pivot_radius={mm(R)} ring_r mean={mm(r_.mean())} min={mm(r_.min())} "
              f"max={mm(r_.max())} s range=[{mm(s_.min())},{mm(s_.max())}] first_vertex_az={round(float(az_[0]), 2)} "
              f"az_step_mean={round(float(np.mean(np.diff(np.unwrap(np.radians(az_))))) * 180 / math.pi, 2)}")

    # spine / belt z
    for rI, names in row_name.items():
        zc = float(Xo[rows[rI], 2].mean())
        for nm_ in names:
            if nm_.startswith("spine"):
                pz = float(piv[nm_[:-2]]["co"][2])
                print(f"[s02a] LOOP_Z {nm_}: ring_z={mm(zc)} pivot_z={mm(pz)} dz={mm(abs(zc - pz))}")
            else:
                tgt = z_bb if nm_ == "belt_bot_0" else z_bt
                print(f"[s02a] LOOP_Z {nm_}: ring_z={mm(zc)} target_z={mm(tgt)} (lip row {mm(BELT_BOT if nm_ == 'belt_bot_0' else z_sp1)} "
                      f"-/+ inset {mm(BELT_ROW_INSET)}) dz={mm(abs(zc - tgt))} "
                      f"(z spread {mm(Xo[rows[rI], 2].max() - Xo[rows[rI], 2].min())})")

    # poles (valence; outer ring k=2 counted +1: T9 bridges a tube to it)
    val = np.bincount(E.ravel(), minlength=len(Xo)).astype(int)
    val_eff = val.copy()
    for name, _ref in PORTS:
        val_eff[np.array(ring_idx[name][2])] += 1
    kd_pts = Xo
    for name, ref in PORTS:
        o, a, u, w, R = frames[name]
        allv = np.concatenate([np.array(x) for x in ring_idx[name]])
        d2 = ((kd_pts[:, None, :] - kd_pts[allv][None, :, :]) ** 2).sum(-1).min(1)
        region = set(np.where(d2 <= 0.030 ** 2)[0].tolist())
        s_all, r_all, _ = azimuth(kd_pts, o, a, np.array(ref))
        s_k = [float(((Xo[np.array(x)] - o) @ a).mean()) for x in ring_idx[name]]
        _, r_ring, _ = azimuth(Xo[allv], o, a, np.array(ref))
        between = (s_all >= min(s_k)) & (s_all <= max(s_k)) & (r_all <= r_ring.max())
        region |= set(np.where(between)[0].tolist())
        reg = np.array(sorted(region))
        poles = reg[val_eff[reg] != 4]
        _, _, phi = azimuth(Xo[poles], o, a, np.array(ref))
        band = (np.abs(phi) <= 30.0) | (np.abs(phi) >= 150.0)
        print(f"[s02a] POLES {name}: region_verts={len(reg)} poles={len(poles)} in_band={int(band.sum())}")
        for v, ph, bb in sorted(zip(poles.tolist(), phi.tolist(), band.tolist()), key=lambda t: t[1]):
            print(f"   v{v} valence={val[v]} eff={val_eff[v]} az={ph:7.2f} in_band={bool(bb)}")
    print(f"[s02a] VALENCE_HIST (effective) {dict(zip(*np.unique(val_eff, return_counts=True)))}")

    # distance to SRC_hi
    dist = np.array([SRC.nearest(p)[1] for p in Xo])
    kind = np.array(b.kind)
    zz = Xo[:, 2]
    th_i = np.round(np.mod(np.arctan2(Xo[:, 1], Xo[:, 0]), 2 * math.pi) / TABLE_DTH).astype(int) % len(rt.th)
    projected = (kind == ROW) | (kind == CYL)
    hidden = (kind == HIDDEN) | (projected & (zz > rt.valid_top[th_i] + 0.002))
    for name, _ref in PORTS:   # ring k0 verts moved straight inward under the head
        for m, d_ in enumerate(port_info[name]):
            if "inward" in d_["rule"]:
                hidden[ring_idx[name][0][m]] = True
    underbelt = projected & (zz > z_bb - 1e-6) & (zz < z_bt + 1e-6)
    vis = ~hidden & ~underbelt

    def st(mask):
        d = dist[mask]
        return (f"n={len(d)} mean={mm(d.mean())} p95={mm(np.percentile(d, 95))} max={mm(d.max())}"
                if len(d) else "n=0")
    print(f"[s02a] DIST_TO_SRC all: {st(np.ones(len(Xo), bool))} (mm)")
    print(f"[s02a] DIST_TO_SRC visible (rings + projected verts below the head crease, outside belt band): "
          f"{st(vis)} (mm)")
    print(f"[s02a] DIST_TO_SRC belt band rows z[{round(z_bb, 5)},{round(z_bt, 5)}] (under belt): {st(underbelt)} (mm)")
    print(f"[s02a] DIST_TO_SRC above head crease (seam row, cap, extrapolated egg, head-type k0; inside head): "
          f"{st(hidden)} (mm)")
    outside = [int(i) for i in np.where(hidden)[0] if not SRC.inside(Xo[i])]
    print(f"[s02a] HIDDEN verts outside SRC_hi volume: {len(outside)} of {int(hidden.sum())}"
          + (f" {outside[:20]}" if outside else ""))
    Pd, dd, why = src_to_body_report(Xo, b.faces, rt, piv, ring_idx, frames)
    w_ = int(np.argmax(dd))
    print(f"[s02a] SRC_TO_BODY (SRC_hi area samples 200000, excluded: {why}): n={len(dd)} mean={mm(dd.mean())} "
          f"p95={mm(np.percentile(dd, 95))} max={mm(dd.max())} n>2mm={int((dd > 0.002).sum())} "
          f"n>3mm={int((dd > 0.003).sum())} max at {np.round(Pd[w_], 4).tolist()}")
    order = np.argsort(-dd)
    tops = []
    for i in order:
        if dd[i] <= 0.003 or len(tops) >= 8:
            break
        if all(np.linalg.norm(Pd[i] - q) > 0.03 for q, _ in tops):
            tops.append((Pd[i], dd[i]))
    print("[s02a] SRC_TO_BODY worst clusters (> 3 mm, 30 mm apart): "
          + ", ".join(f"{mm(v)} at {np.round(q, 3).tolist()}" for q, v in tops))
    print("[s02a] done")


SRC = None
HEAD_ZC = None

if __name__ == "__main__":
    goblib.run_main(main)
