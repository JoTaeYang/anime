"""s01_pivots - measure joint pivot candidates on the normalized source mesh.

Design doc d-23 section 2 (DEF skeleton, position rule), section 7 G1 (G1.5-G1.7) and
section 8 (elbow / knee default = 50 %); shoulder / shoulder_root / spine_02 per section 2 user decision.

  input : rig/gob_r00_source.blend (SRC_hi, SRC_club_hi), read only - never saved
  output: rig/data/pivots.json
          rig/inspect/G1/pivots_front.png, pivots_side.png, hand_r_top.png, hand_r_front.png

All coordinates are world metres (front -Y, up +Z, character left +X).  Left and right
are measured independently (no mirroring).

Tubes are measured with planar sections of SRC_hi: arms with planes x = const, legs with
planes z = const.  Each closed section loop is resampled by arc length and fitted with
an algebraic (Kasa) circle; the fitted radius is the "section radius".  A tube is tracked
slice by slice (step STEP) from a seed slice, choosing the loop whose area centroid is
nearest the previous one; the track is lost when no loop is within TRACK_DIST * r_seed
with area radius < TRACK_RMAX * r_seed (= the tube merged into the torso).

Run:  bl.ps1 -Script s01_pivots.py -Blend gob_r00_source.blend
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import math  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

STEP = 0.002             # slice step (m)
TORSO_STEP = 0.0025      # torso profile slice step (m)
TRACK_DIST = 1.5         # max centroid jump between slices, x r_seed
TRACK_RMAX = 3.0         # max area radius of a tube loop, x r_seed
TUBE_TOL = 1.1           # tube / non-tube radius threshold, x r_tube
PELVIS_UP = 0.03         # pelvis = mean hip z + this
MARKER_R = 0.008
TEXT_SIZE = 0.022
LABEL_GAP = 0.03
LEADER_R = 0.0012

REQUIRED = ["shoulder_root_l", "shoulder_root_r", "shoulder_l", "shoulder_r", "elbow_l",
            "elbow_r", "wrist_l", "wrist_r", "hip_l", "hip_r", "knee_l", "knee_r", "ankle_l",
            "ankle_r", "pelvis", "spine_01", "spine_02", "head", "grip_r", "foot_tip_l",
            "foot_tip_r", "heel_l", "heel_r"]
PAIRS = ["shoulder_root", "shoulder", "elbow", "wrist", "hip", "knee", "ankle", "foot_tip",
         "heel"]


# ---------------------------------------------------------------- mesh io
def mesh_co(me):
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    return co.reshape(-1, 3)


def mesh_tris(me):
    me.calc_loop_triangles()
    t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", t)
    return t.reshape(-1, 3)


def world_mesh(name):
    """World-space (co, tris) of an evaluated object, copied into numpy (mesh untouched)."""
    ob = bpy.data.objects[name]
    dg = bpy.context.evaluated_depsgraph_get()
    oe = ob.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        co = mesh_co(me)
        tris = mesh_tris(me)
    finally:
        oe.to_mesh_clear()
    mw = np.array(ob.matrix_world)
    co = co @ mw[:3, :3].T + mw[:3, 3]
    return co, tris


# ---------------------------------------------------------------- sections
_OTHER = {0: [1, 2], 1: [0, 2], 2: [0, 1]}


def section(co, tris, ax, w0):
    """Closed loops of the plane co[ax] = w0 as (N,2) arrays in the two other axes."""
    uv = _OTHER[ax]
    s = co[:, ax] - w0
    s[s == 0.0] = 1e-12
    pos = s[tris] > 0
    cross = tris[pos.any(1) & ~pos.all(1)]
    if len(cross) == 0:
        return []
    n = len(co)
    segs = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        va, vb = cross[:, a], cross[:, b]
        diff = (s[va] > 0) != (s[vb] > 0)
        segs.append(np.where(diff, np.minimum(va, vb) * n + np.maximum(va, vb), -1))
    pairs = np.sort(np.stack(segs, 1), 1)[:, 1:]
    adj = {}
    for k1, k2 in pairs.tolist():
        adj.setdefault(k1, []).append(k2)
        adj.setdefault(k2, []).append(k1)

    def point(k):
        a, b = divmod(k, n)
        f = s[a] / (s[a] - s[b])
        return co[a, uv] + (co[b, uv] - co[a, uv]) * f

    loops, seen = [], set()
    for start in adj:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        closed = False
        while True:
            nb = adj[cur]
            nxt = nb[0] if nb[0] != prev else (nb[1] if len(nb) > 1 else None)
            if nxt is None or nxt == start:
                closed = nxt == start
                break
            if nxt in seen:
                break
            seen.add(nxt)
            loop.append(nxt)
            prev, cur = cur, nxt
        if closed and len(loop) >= 3:
            loops.append(np.array([point(k) for k in loop]))
    return loops


def area_centroid(p):
    x, y = p[:, 0], p[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    c = x * y1 - x1 * y
    a = c.sum() / 2.0
    if abs(a) < 1e-14:
        return 0.0, np.array([x.mean(), y.mean()])
    return abs(a), np.array([((x + x1) * c).sum() / (6.0 * a), ((y + y1) * c).sum() / (6.0 * a)])


def resample(p, n=128):
    q = np.vstack([p, p[:1]])
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, cum[-1], n, endpoint=False)
    return np.stack([np.interp(t, cum, q[:, 0]), np.interp(t, cum, q[:, 1])], 1)


def fit_circle(p):
    """Kasa algebraic circle fit on arc-length resampled loop points -> (center(2), r)."""
    q = resample(p)
    A = np.column_stack([2 * q[:, 0], 2 * q[:, 1], np.ones(len(q))])
    b = (q ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:2]
    return c, float(math.sqrt(max(sol[2] + c @ c, 0.0)))


def point_in_poly(p, q):
    x, y = p[:, 0], p[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    with np.errstate(divide="ignore", invalid="ignore"):
        xi = (x1 - x) * (q[1] - y) / (y1 - y) + x
    c = ((y > q[1]) != (y1 > q[1])) & (q[0] < xi)
    return int(c.sum()) % 2 == 1


def loop_info(p):
    a, cen = area_centroid(p)
    fc, fr = fit_circle(p)
    return {"loop": p, "area": a, "centroid": cen, "req": math.sqrt(a / math.pi),
            "fc": fc, "fr": fr, "lo": p.min(0), "hi": p.max(0)}


# ---------------------------------------------------------------- tube tracking
def track(co, tris, ax, w0, dw, c0, r_seed, stop):
    """Track a tube loop from slice w0 in steps dw until lost or stop(info) is True.

    Returns list of (w, info); the slice where the track is lost is not included."""
    out, prev, w = [], np.asarray(c0, float), w0
    for _ in range(2000):
        loops = [loop_info(p) for p in section(co, tris, ax, w)]
        cand = [li for li in loops if np.linalg.norm(li["centroid"] - prev) < TRACK_DIST * r_seed
                and li["req"] < TRACK_RMAX * r_seed]
        if not cand:
            break
        li = min(cand, key=lambda li: np.linalg.norm(li["centroid"] - prev))
        out.append((w, li))
        if stop(li):
            break
        prev = li["centroid"]
        w += dw
    return out


def fit_line(pts):
    """Least-squares 3D line through pts -> (mean, unit direction)."""
    pts = np.asarray(pts, float)
    m = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - m)
    return m, vt[0] / np.linalg.norm(vt[0])


def line_at(m, d, ax, w):
    """Point of line (m, d) where coordinate ax == w."""
    return m + d * ((w - m[ax]) / d[ax])


def to3(ax, w, uv):
    p = np.zeros(3)
    p[ax] = w
    p[_OTHER[ax]] = uv
    return p


def measure_arm(co, tris, side):
    """side +1 = left (+X), -1 = right (-X)."""
    tag = "l" if side > 0 else "r"
    xmax = float((co[:, 0] * side).max())
    # seed: first slice outward from x=0 whose only loop on this side is small (tube)
    seed = None
    t = 0.05
    while t < xmax:
        loops = [loop_info(p) for p in section(co, tris, 0, side * t)]
        if len(loops) == 1 and loops[0]["req"] < 0.1:
            seed = (t, loops[0])
            break
        t += 0.005
    if seed is None:
        raise RuntimeError(f"arm_{tag}: no seed slice")
    t0, s0 = seed
    r_seed = s0["fr"]
    inward = track(co, tris, 0, side * t0, -side * STEP, s0["centroid"], r_seed, lambda li: False)
    outward = track(co, tris, 0, side * t0, side * STEP, s0["centroid"], r_seed,
                    lambda li: li["fr"] > 1.3 * r_seed)
    sl = sorted(inward[1:] + outward, key=lambda e: side * e[0])  # proximal -> distal
    ts = np.array([side * w for w, _ in sl])
    fr = np.array([li["fr"] for _, li in sl])
    t_root = ts[0]
    i_w0 = next((i for i in range(len(sl)) if ts[i] > t0 and fr[i] > TUBE_TOL * r_seed), len(sl) - 1)
    mid = (ts >= t_root + 0.25 * (ts[i_w0] - t_root)) & (ts <= t_root + 0.75 * (ts[i_w0] - t_root))
    r_arm = float(np.median(fr[mid]))
    i_sh = next(i for i in range(len(sl)) if fr[i] <= TUBE_TOL * r_arm)
    t_mid = 0.5 * (t_root + ts[i_w0])
    i_wr = next(i for i in range(len(sl)) if ts[i] > t_mid and fr[i] > TUBE_TOL * r_arm)
    tube = [to3(0, w, li["fc"]) for w, li in sl[i_sh:i_wr]]
    m, d = fit_line(tube)
    if d[0] * side < 0:
        d = -d
    x_root, x_sh, x_wr = side * t_root, side * ts[i_sh], side * ts[i_wr]
    x_el = 0.5 * (x_sh + x_wr)
    root_loop = sl[0][1]
    x_joint = x_root  # arm-torso junction (no fillet on this model; d-23 section 2 user decision)
    res = {
        f"shoulder_root_{tag}": {
            "co": line_at(m, d, 0, 0.5 * x_joint), "axis": d, "radius": r_arm,
            "method": f"shoulder bone head (d-23 section 2 user decision 2026-09-23): point on the arm "
                      f"tube axis line extended into the torso at x = 0.5 * shoulder.x "
                      f"= {0.5 * x_joint:+.4f}"},
        f"shoulder_{tag}": {
            "co": line_at(m, d, 0, x_joint), "axis": d, "radius": r_arm,
            "method": f"arm-torso junction (d-23 section 2 user decision 2026-09-23): x-slices "
                      f"(step {STEP*1000:.0f} mm) tracked inward from seed x={side*t0:+.3f}; innermost "
                      f"slice where the arm section is still a separate closed loop (next slice merges "
                      f"with torso); point on tube axis line at that x; slice fit "
                      f"r={root_loop['fr']*1000:.1f} mm (first slice with r <= {TUBE_TOL}*r_arm: "
                      f"x={x_sh:+.4f}, r={fr[i_sh]*1000:.1f} mm; no fillet)"},
        f"elbow_{tag}": {
            "co": line_at(m, d, 0, x_el), "axis": d, "radius": r_arm,
            "method": "50% of shoulder -> wrist (d-23 section 8 default); point on tube axis line"},
        f"wrist_{tag}": {
            "co": line_at(m, d, 0, x_wr), "axis": d, "radius": r_arm,
            "method": f"first slice outward from tube middle with circle-fit r > {TUBE_TOL}*r_arm; "
                      f"slice fit r={fr[i_wr]*1000:.1f} mm; point on tube axis line"},
    }
    tube_fc = np.array([to3(0, w, li["fc"]) for w, li in sl[i_sh:i_wr]])
    dev = np.linalg.norm(np.cross(tube_fc - m, d), axis=1)
    info = {"r_arm": r_arm, "r_seed": r_seed, "seed_x": side * t0, "n_slices": len(sl),
            "n_tube": i_wr - i_sh, "axis_dev_max": float(dev.max()),
            "root_zrange": (float(root_loop["lo"][1]), float(root_loop["hi"][1])),
            "profile": [(side * t, f) for t, f in zip(ts, fr)]}
    return res, info


def measure_leg(co, tris, side, z_torso_bottom):
    tag = "l" if side > 0 else "r"
    best = None
    z = 0.01
    while z < z_torso_bottom:
        loops = [loop_info(p) for p in section(co, tris, 2, z)]
        mine = [li for li in loops if li["centroid"][0] * side > 0]
        if len(mine) == 1 and (best is None or mine[0]["fr"] < best[1]["fr"]):
            best = (z, mine[0])
        z += 0.005
    if best is None:
        raise RuntimeError(f"leg_{tag}: no seed slice")
    z0, s0 = best
    r_seed = s0["fr"]
    up = track(co, tris, 2, z0, STEP, s0["centroid"], r_seed, lambda li: False)
    down = track(co, tris, 2, z0, -STEP, s0["centroid"], r_seed, lambda li: li["fr"] > 1.3 * r_seed)
    sl = sorted(up + down[1:], key=lambda e: -e[0])  # proximal (top) -> distal
    zs = np.array([w for w, _ in sl])
    fr = np.array([li["fr"] for _, li in sl])
    z_hip = zs[0]
    i_a0 = next((i for i in range(len(sl)) if zs[i] < z0 and fr[i] > TUBE_TOL * r_seed), len(sl) - 1)
    span = z_hip - zs[i_a0]
    mid = (zs <= z_hip - 0.25 * span) & (zs >= z_hip - 0.75 * span)
    r_leg = float(np.median(fr[mid]))
    z_mid = 0.5 * (z_hip + zs[i_a0])
    i_an = next(i for i in range(len(sl)) if zs[i] < z_mid and fr[i] > TUBE_TOL * r_leg)
    i_top = next((i for i in range(len(sl)) if fr[i] <= TUBE_TOL * r_leg), 0)
    tube = [to3(2, w, li["fc"]) for w, li in sl[i_top:i_an]]
    m, d = fit_line(tube)
    if d[2] > 0:
        d = -d
    z_an = zs[i_an]
    z_kn = 0.5 * (z_hip + z_an)
    res = {
        f"hip_{tag}": {
            "co": line_at(m, d, 2, z_hip), "axis": d, "radius": r_leg,
            "method": f"z-slices (step {STEP*1000:.0f} mm) tracked upward from seed z={z0:.3f}; "
                      f"last slice before the leg section merges with the torso section; "
                      f"point on tube axis line; slice fit r={fr[0]*1000:.1f} mm"},
        f"knee_{tag}": {
            "co": line_at(m, d, 2, z_kn), "axis": d, "radius": r_leg,
            "method": "50% of hip -> ankle (d-23 section 8 default); point on tube axis line"},
        f"ankle_{tag}": {
            "co": line_at(m, d, 2, z_an), "axis": d, "radius": r_leg,
            "method": f"first slice downward from tube middle with circle-fit r > {TUBE_TOL}*r_leg; "
                      f"slice fit r={fr[i_an]*1000:.1f} mm; point on tube axis line"},
    }
    tube_fc = np.array(tube)
    dev = np.linalg.norm(np.cross(tube_fc - m, d), axis=1)
    info = {"r_leg": r_leg, "r_seed": r_seed, "seed_z": z0, "n_slices": len(sl),
            "n_tube": i_an - i_top, "axis_dev_max": float(dev.max()), "z_ankle": z_an}
    return res, info


def measure_feet(co, tris, side, z_ankle):
    tag = "l" if side > 0 else "r"
    sel = co[(co[:, 2] < z_ankle) & (co[:, 0] * side > 0)]
    tip = sel[np.argmin(sel[:, 1])]
    heel = sel[np.argmax(sel[:, 1])]
    return {
        f"foot_tip_{tag}": {"co": [tip[0], tip[1], 0.0],
                            "method": f"shoe verts (z < ankle z {z_ankle:.3f}, x sign side): min-y vertex "
                                      f"(z {tip[2]:.3f}) projected to z=0"},
        f"heel_{tag}": {"co": [heel[0], heel[1], 0.0],
                        "method": f"shoe verts (z < ankle z {z_ankle:.3f}, x sign side): max-y vertex "
                                  f"(z {heel[2]:.3f}) projected to z=0"},
    }


# ---------------------------------------------------------------- torso
def torso_profile(co, tris, z0, z1):
    rows = []
    z = z0
    while z < z1:
        loops = section(co, tris, 2, z)
        cen = [p for p in loops if point_in_poly(p, (0.0, 0.0))]
        if cen:
            p = max(cen, key=lambda p: area_centroid(p)[0])
            a, _ = area_centroid(p)
            d = np.hypot(p[:, 0], p[:, 1])
            fb = np.abs(p[:, 0]) <= np.abs(p[:, 1])
            rows.append((z, float(np.abs(p[:, 0]).max()), math.sqrt(a / math.pi),
                         float(np.median(d[fb])) if fb.any() else float("nan")))
        z += TORSO_STEP
    return np.array(rows)  # z, half width, area radius, front/back median radius


def step_excess(v, win=6):
    dv = np.diff(v)
    ex = np.zeros_like(dv)
    for i in range(len(dv)):
        nb = np.concatenate([dv[max(0, i - win):i], dv[i + 1:i + 1 + win]])
        ex[i] = dv[i] - (np.median(nb) if len(nb) else 0.0)
    return dv, ex


def measure_torso(co, tris, hips, arm_band):
    z_hip = max(h["co"][2] for h in hips)
    top = float(co[:, 2].max())
    prof = torso_profile(co, tris, z_hip + 0.01, top - 0.01)
    zc, hw, req, rfb = prof[:, 0], prof[:, 1], prof[:, 2], prof[:, 3]
    # belt: steps of the area radius between (hip + 5 cm) and the arm band
    sel = np.where((zc >= z_hip + 0.05) & (zc <= arm_band[0]))[0]
    dv, ex = step_excess(req[sel])
    ib = int(np.argmax(ex))
    after = np.arange(len(ex)) > ib
    it = int(np.argmin(np.where(after, ex, np.inf)))
    zb = 0.5 * (zc[sel][ib] + zc[sel][ib + 1])
    zt = 0.5 * (zc[sel][it] + zc[sel][it + 1])
    # egg max half width: exclude the arm band and the belt band
    ok = ((zc < arm_band[0]) | (zc > arm_band[1])) & ((zc < zb) | (zc > zt)) & (zc < top - 0.3)
    i2 = int(np.argmax(np.where(ok, hw, -np.inf)))
    i2_all = int(np.argmax(np.where((zc < arm_band[0]) | (zc > arm_band[1]), np.where(zc < top - 0.3, hw, -np.inf), -np.inf)))
    # head: minimum of the front/back radius (|x| <= |y| points, unaffected by the arms)
    hsel = (zc > zc[i2]) & (zc < top - 0.1)
    ih = int(np.argmin(np.where(hsel, rfb, np.inf)))
    return {
        "spine_01": {"co": [0.0, 0.0, zt],
                     "method": f"belt top: largest negative step of torso area-radius profile "
                               f"(z-slices {TORSO_STEP*1000:.1f} mm, central loop containing (0,0), "
                               f"step minus local median step) = {dv[it]*1000:+.1f} mm between z "
                               f"{zc[sel][it]:.4f}/{zc[sel][it+1]:.4f}; belt bottom (largest positive "
                               f"step {dv[ib]*1000:+.1f} mm) at z {zb:.4f}"},
        "spine_02": {"co": [0.0, 0.0, 0.5 * (zt + zc[ih])],
                     "method": f"z = (spine_01.z + head.z) / 2 (d-23 section 2 user decision 2026-09-23). "
                               f"Reference only: max half width (max|x|) of central torso loop, excluding "
                               f"arm band z[{arm_band[0]:.3f},{arm_band[1]:.3f}] and belt band "
                               f"z[{zb:.4f},{zt:.4f}], is at z {zc[i2]:.4f} (hw={hw[i2]*1000:.1f} mm); "
                               f"including belt at z {zc[i2_all]:.4f} (hw={hw[i2_all]*1000:.1f} mm)"},
        "head": {"co": [0.0, 0.0, zc[ih]],
                 "method": f"head box / torso crease: z of minimum front-back radius (median |p| of "
                           f"central-loop points with |x| <= |y|) above spine_02 = {rfb[ih]*1000:.1f} mm "
                           f"(slope kink of the profile)"},
    }, {"belt_bottom": zb, "belt_top": zt, "profile": prof}


# ---------------------------------------------------------------- grip
def fit_sphere(p):
    A = np.column_stack([2 * p, np.ones(len(p))])
    b = (p ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:3]
    return c, float(math.sqrt(max(sol[3] + c @ c, 0.0)))


def ransac_sphere(p, rmin, rmax, tol, iters=4000, seed=0):
    """Sphere with the most inliers (|dist - r| < tol) among random 4-point fits, refit on inliers."""
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(p), size=(iters, 4))
    best_n, best = -1, None
    for q in p[idx]:
        A = np.column_stack([2 * (q[1:] - q[0])])
        b = (q[1:] ** 2).sum(1) - (q[0] ** 2).sum()
        if abs(np.linalg.det(A)) < 1e-12:
            continue
        c = np.linalg.solve(A, b)
        r = float(np.linalg.norm(q[0] - c))
        if not (rmin <= r <= rmax):
            continue
        n = int((np.abs(np.linalg.norm(p - c, axis=1) - r) < tol).sum())
        if n > best_n:
            best_n, best = n, (c, r)
    c, r = best
    for _ in range(5):
        inl = p[np.abs(np.linalg.norm(p - c, axis=1) - r) < tol]
        c, r = fit_sphere(inl)
    return c, r, inl


def measure_grip(co, club, wrist_r):
    hand = co[co[:, 0] < wrist_r[0]]
    c, r, keep = ransac_sphere(hand, 0.04, 0.2, 0.001)
    rms = float(np.sqrt(np.mean((np.linalg.norm(keep - c, axis=1) - r) ** 2)))
    m = club.mean(0)
    _, _, vt = np.linalg.svd(club - m)
    a = vt[0] / np.linalg.norm(vt[0])
    s = (club - m) @ a
    rad = np.linalg.norm((club - m) - np.outer(s, a), axis=1)
    lo, hi = s.min(), s.max()
    L = hi - lo
    r_lo = np.median(rad[s < lo + 0.1 * L])
    r_hi = np.median(rad[s > hi - 0.1 * L])
    if r_lo > r_hi:  # head at the low end -> flip so axis points to the head
        a = -a
    grip = m + a * ((c - m) @ a)
    return {"grip_r": {
        "co": grip, "axis": a,
        "method": f"palm sphere: RANSAC sphere (4-point fits, r 40-200 mm, inlier |d-r| < 1 mm, "
                  f"refit on inliers) of SRC_hi verts with x < wrist_r.x ({len(hand)} verts, "
                  f"{len(keep)} inliers) -> center ({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}) "
                  f"r={r*1000:.1f} mm rms={rms*1000:.2f} mm; "
                  f"projected on SRC_club_hi PCA major axis ({len(club)} verts, both pieces); "
                  f"axis points to the larger-radius end (club head); "
                  f"sphere-to-axis distance {np.linalg.norm(c - grip)*1000:.1f} mm"}}, \
        {"palm_c": c, "palm_r": r, "club_mean": m}


# ---------------------------------------------------------------- render helpers
_TEMP = {"objs": [], "data": [], "mats": []}


def _link(ob):
    bpy.context.scene.collection.objects.link(ob)
    _TEMP["objs"].append(ob)
    return ob


def make_marker(name, co, col):
    me = bpy.data.meshes.new("_piv_m_" + name)
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=8, radius=MARKER_R)
    bm.to_mesh(me)
    bm.free()
    _TEMP["data"].append(me)
    ob = bpy.data.objects.new("_piv_m_" + name, me)
    ob.location = Vector(co)
    ob.color = col
    ob.show_in_front = True
    return _link(ob)


def make_leader(name, a, b, col):
    a, b = Vector(a), Vector(b)
    L = (b - a).length
    if L < 1e-5:
        return None
    me = bpy.data.meshes.new("_piv_l_" + name)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=6, radius1=LEADER_R, radius2=LEADER_R, depth=L)
    bm.to_mesh(me)
    bm.free()
    _TEMP["data"].append(me)
    ob = bpy.data.objects.new("_piv_l_" + name, me)
    q = Vector((0, 0, 1)).rotation_difference((b - a).normalized())
    ob.matrix_world = Matrix.Translation((a + b) / 2) @ q.to_matrix().to_4x4()
    ob.color = col
    ob.show_in_front = True
    return _link(ob)


def make_label(name, text, co, rot, align, col):
    cu = bpy.data.curves.new("_piv_t_" + name, "FONT")
    cu.body = text
    cu.size = TEXT_SIZE
    cu.align_x = align
    cu.align_y = "CENTER"
    _TEMP["data"].append(cu)
    ob = bpy.data.objects.new("_piv_t_" + name, cu)
    ob.matrix_world = Matrix.Translation(Vector(co)) @ rot
    ob.color = col
    ob.show_in_front = True
    return _link(ob)


def cleanup_temp():
    for ob in _TEMP["objs"]:
        bpy.data.objects.remove(ob, do_unlink=True)
    for d in _TEMP["data"]:
        if isinstance(d, bpy.types.Mesh):
            bpy.data.meshes.remove(d)
        else:
            bpy.data.curves.remove(d)
    _TEMP["objs"].clear()
    _TEMP["data"].clear()


def spread(vals, gap):
    """Positions near vals (sorted order kept) with min spacing gap, centered on the group."""
    order = np.argsort(vals)
    v = np.array(vals, float)[order]
    out = v.copy()
    for i in range(1, len(out)):
        out[i] = max(out[i], out[i - 1] + gap)
    out -= (out.mean() - v.mean())
    res = np.empty_like(out)
    res[order] = out
    return res


COL_L = (0.9, 0.1, 0.1, 1.0)
COL_R = (0.1, 0.35, 1.0, 1.0)
COL_C = (0.0, 0.65, 0.2, 1.0)
COL_G = (0.85, 0.0, 0.85, 1.0)
COL_TXT = (0.05, 0.05, 0.05, 1.0)
COL_MESH = (0.82, 0.82, 0.82, 1.0)


def pivot_color(name):
    if name == "grip_r":
        return COL_G
    if name.endswith("_l"):
        return COL_L
    if name.endswith("_r"):
        return COL_R
    return COL_C


def render_overlay(piv, view, out_png, frame, groups, markers):
    """groups: list of (names, kind, param): kind 'col' -> column of horizontal labels at
    image-u = param[0] (align param[1]); kind 'row' -> row of vertical labels starting at
    image-v = param[0]."""
    d, right, up = goblib._view_basis(view)
    right, up, d = np.array(right), np.array(up), np.array(d)
    toward = -d  # toward the camera
    rot_h = Matrix((right.tolist(), up.tolist(), toward.tolist())).transposed().to_4x4()
    rot_v = Matrix((up.tolist(), (-right).tolist(), toward.tolist())).transposed().to_4x4()
    for n in markers:
        make_marker(n, piv[n]["co"], pivot_color(n))
    for names, kind, param in groups:
        cos = [np.array(piv[n]["co"], float) for n in names]
        if kind == "col":
            u_col, align = param
            vs = spread([c @ up for c in cos], LABEL_GAP)
            for n, c, v in zip(names, cos, vs):
                w = c @ d - 0.5
                lab = right * u_col + up * v + d * w
                end = lab - right * (0.006 if align == "LEFT" else -0.006)
                make_leader(n, c, end, COL_TXT)
                make_label(n, n, lab, rot_h, align, pivot_color(n))
        else:
            v_row = param[0]
            us = spread([c @ right for c in cos], LABEL_GAP)
            for n, c, u in zip(names, cos, us):
                w = c @ d - 0.5
                lab = right * u + up * v_row + d * w
                make_leader(n, c, lab - up * 0.006, COL_TXT)
                make_label(n, n, lab, rot_v, "LEFT", pivot_color(n))
    objs = [bpy.data.objects["SRC_hi"], bpy.data.objects["SRC_club_hi"]] + list(_TEMP["objs"])
    colors = {"SRC_hi": COL_MESH, "SRC_club_hi": (0.7, 0.66, 0.6, 1.0)}
    for ob in _TEMP["objs"]:
        colors[ob.name] = tuple(ob.color)
    try:
        goblib.ortho_render(objs, view, out_png, res_h=1024, frame_bbox=frame, colors=colors)
    finally:
        cleanup_temp()
    print(f"render: {goblib._rel(out_png)}")


# ---------------------------------------------------------------- main
def main():
    co, tris = world_mesh("SRC_hi")
    club, _ = world_mesh("SRC_club_hi")
    print(f"SRC_hi verts={len(co)} tris={len(tris)} bbox={co.min(0).round(4)}..{co.max(0).round(4)}")
    print(f"SRC_club_hi verts={len(club)}")

    piv, info = {}, {}
    for side in (1, -1):
        r, i = measure_arm(co, tris, side)
        piv.update(r)
        info["arm_" + ("l" if side > 0 else "r")] = i
    # lowest z where a section loop contains the torso axis (0,0) -> torso bottom
    z = 0.01
    while z < 1.0:
        if any(point_in_poly(p, (0.0, 0.0)) for p in section(co, tris, 2, z)):
            break
        z += 0.005
    z_torso_bottom = z
    for side in (1, -1):
        r, i = measure_leg(co, tris, side, z_torso_bottom)
        piv.update(r)
        tag = "l" if side > 0 else "r"
        info["leg_" + tag] = i
        piv.update(measure_feet(co, tris, side, i["z_ankle"]))
    zr = [info["arm_l"]["root_zrange"], info["arm_r"]["root_zrange"]]
    arm_band = (min(a for a, _ in zr) - 0.005, max(b for _, b in zr) + 0.005)
    tor, tinfo = measure_torso(co, tris, [piv["hip_l"], piv["hip_r"]], arm_band)
    piv.update(tor)
    piv["pelvis"] = {"co": [0.0, 0.0, 0.5 * (piv["hip_l"]["co"][2] + piv["hip_r"]["co"][2]) + PELVIS_UP],
                     "method": f"torso axis x=y=0; z = mean(hip_l.z, hip_r.z) + {PELVIS_UP} m"}
    g, ginfo = measure_grip(co, club, np.array(piv["wrist_r"]["co"]))
    piv.update(g)

    out = {"pivots": {k: {kk: (np.asarray(vv, float).tolist() if kk in ("co", "axis") else vv)
                          for kk, vv in piv[k].items()} for k in REQUIRED}}
    p = goblib.save_json("pivots.json", out)
    print(f"wrote {goblib._rel(p)}")

    # ---------------- stdout summary
    print("---- pivots (m)")
    for k in REQUIRED:
        e = out["pivots"][k]
        c = e["co"]
        extra = ""
        if "axis" in e:
            extra += " axis=(" + ",".join(f"{v:+.3f}" for v in e["axis"]) + ")"
        if "radius" in e:
            extra += f" r={e['radius']*1000:.1f}mm"
        print(f"{k:16s} ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f}){extra}")
    for t in ("l", "r"):
        a, lg = info["arm_" + t], info["leg_" + t]
        print(f"r_arm_{t}={a['r_arm']*1000:.1f}mm (seed x={a['seed_x']:+.3f} r={a['r_seed']*1000:.1f}, "
              f"tube slices={a['n_tube']}, axis dev max={a['axis_dev_max']*1000:.1f}mm)  "
              f"r_leg_{t}={lg['r_leg']*1000:.1f}mm (seed z={lg['seed_z']:.3f} r={lg['r_seed']*1000:.1f}, "
              f"tube slices={lg['n_tube']}, axis dev max={lg['axis_dev_max']*1000:.1f}mm)")
    print(f"arm band z={arm_band[0]:.4f}..{arm_band[1]:.4f} torso bottom z={z_torso_bottom:.3f} "
          f"belt z={tinfo['belt_bottom']:.4f}..{tinfo['belt_top']:.4f}")
    print(f"palm sphere c={np.round(ginfo['palm_c'], 4)} r={ginfo['palm_r']*1000:.1f}mm")
    print("---- mirror |co_l - mirrorX(co_r)| (mm)")
    for pn in PAIRS:
        cl = np.array(out["pivots"][pn + "_l"]["co"])
        cr = np.array(out["pivots"][pn + "_r"]["co"]) * np.array([-1.0, 1.0, 1.0])
        dv = (cl - cr) * 1000
        print(f"{pn:14s} err={np.linalg.norm(dv):6.1f}  dxyz=({dv[0]:+6.1f},{dv[1]:+6.1f},{dv[2]:+6.1f})")

    # ---------------- renders
    g1 = goblib.INSPECT / "G1"
    allv = np.vstack([co, club])
    mn, mx = allv.min(0), allv.max(0)
    pad = 0.05
    arm_l = ["shoulder_root_l", "shoulder_l", "elbow_l", "wrist_l"]
    arm_r = ["shoulder_root_r", "shoulder_r", "elbow_r", "wrist_r", "grip_r"]
    leg_l = ["hip_l", "knee_l", "ankle_l", "foot_tip_l", "heel_l"]
    leg_r = ["hip_r", "knee_r", "ankle_r", "foot_tip_r", "heel_r"]
    center = ["pelvis", "spine_01", "spine_02", "head"]
    v_row = float(club[:, 2].max()) + 0.03
    frame_f = ((mn[0] - pad, 0, mn[2] - pad), (mx[0] + pad, 0, mx[2] + pad))
    render_overlay(piv, "front", g1 / "pivots_front.png", frame_f, [
        (arm_l, "row", (v_row,)), (arm_r, "row", (v_row,)),
        (leg_l, "col", (0.47, "LEFT")), (leg_r, "col", (-0.47, "RIGHT")),
        (center, "col", (0.07, "LEFT")),
    ], REQUIRED)
    u_col = float(mx[1]) + 0.06
    frame_s = ((0, mn[1] - pad, mn[2] - pad), (0, u_col + 0.25, mx[2] + pad))
    side_labels = arm_l + center + leg_l + ["grip_r"]
    render_overlay(piv, "side", g1 / "pivots_side.png", frame_s, [
        (side_labels, "col", (u_col, "LEFT")),
    ], REQUIRED)

    hand = co[co[:, 0] < piv["wrist_r"]["co"][0] + 0.03]
    hmn, hmx = hand.min(0) - 0.04, hand.max(0) + 0.04
    fr_h = (tuple(hmn), tuple(hmx))
    print(f"hand_r frame bbox={np.round(hmn, 3)}..{np.round(hmx, 3)}")
    for view, nm in (("top", "hand_r_top.png"), ("front", "hand_r_front.png")):
        pth = goblib.ortho_render(["SRC_hi"], view, g1 / nm, res_h=1024, frame_bbox=fr_h,
                                  colors={"SRC_hi": COL_MESH})
        print(f"render: {goblib._rel(pth)}")
    left = [o.name for o in bpy.data.objects if o.name.startswith("_piv_")]
    print(f"temp objects left: {len(left)}; blend not saved (is_dirty={bpy.data.is_dirty})")


goblib.run_main(main)
