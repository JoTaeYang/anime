"""check_g1_source - Gate G1 SRC checker (design doc d-23 section 7, G1.1 .. G1.6).

Independently measures rig/gob_r00_source.blend (objects SRC_hi, SRC_club_hi) and
rig/data/source_prep.json, rig/data/pivots.json.  It does not import or reuse the
production scripts (s00_*, s01_*); only blend geometry and the data json are used.

Run:  bl.ps1 -Script check_g1_source.py -Blend gob_r00_source.blend
Output: rig/inspect/G1/check_g1_source.json.  G1.7 is visual only; the overlay renders
rig/inspect/G1/pivots_front.png / pivots_side.png are registered when present.

Exit: 0 when the evidence was written; 2 on a script error (run_main) or when an
input file is missing (the evidence is still written, then "missing input" is printed).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import json  # noqa: E402
from collections import defaultdict  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
import bpy  # noqa: E402

GATE = "G1"
CHECKER = "check_g1_source"
BLEND = goblib.RIG / "gob_r00_source.blend"
SRC_PREP = goblib.DATA / "source_prep.json"
PIVOTS = goblib.DATA / "pivots.json"
RENDERS = [goblib.INSPECT / "G1" / "pivots_front.png", goblib.INSPECT / "G1" / "pivots_side.png",
           goblib.INSPECT / "G1" / "hand_r_top.png", goblib.INSPECT / "G1" / "hand_r_front.png"]

BODY = "SRC_hi"
CLUB = "SRC_club_hi"
TOL_XFORM = 1e-6
H_NOMINAL = 1.388          # m
TOL_H = 0.002              # m
TOL_MINZ = 0.0005          # m
TOL_CX = 0.001             # m
TOL_BBOX = 0.001           # m
CLUB_HEAD_R = 0.030        # m
TORSO_BAND = (0.30, 0.60)  # fraction of H
N_SLICES = 10
HEAD_FRAC = 0.75           # head region z > minz + HEAD_FRAC*H
FEATURE_PCT = 95.0         # top dihedral-angle percentile = "feature" edges

REQUIRED_PIVOTS = [
    "shoulder_root_l", "shoulder_root_r", "shoulder_l", "shoulder_r", "elbow_l", "elbow_r",
    "wrist_l", "wrist_r", "hip_l", "hip_r", "knee_l", "knee_r", "ankle_l", "ankle_r",
    "pelvis", "spine_01", "spine_02", "head", "grip_r", "foot_tip_l", "foot_tip_r",
    "heel_l", "heel_r",
]
PAIRS = ["shoulder_root", "shoulder", "elbow", "wrist", "hip", "knee", "ankle", "foot_tip", "heel"]


# ---------------------------------------------------------------- helpers
def mm(x):
    return None if x is None else round(float(x) * 1000.0, 4)


def rel(p):
    return goblib._rel(p)


def vec3(v):
    """list of 3 finite numbers -> np.array, else None."""
    try:
        a = np.array([float(x) for x in v], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        return None
    return a


class MeshData:
    """World-space arrays of an object's evaluated mesh."""

    def __init__(self, obj):
        dg = bpy.context.evaluated_depsgraph_get()
        oe = obj.evaluated_get(dg)
        me = oe.to_mesh()
        try:
            mw = np.array(oe.matrix_world, dtype=np.float64)
            nv, ne, npoly, nl = len(me.vertices), len(me.edges), len(me.polygons), len(me.loops)
            co = np.empty(nv * 3, dtype=np.float64)
            me.vertices.foreach_get("co", co)
            co = co.reshape(nv, 3)
            self.v = co @ mw[:3, :3].T + mw[:3, 3]
            ev = np.empty(ne * 2, dtype=np.int64)
            me.edges.foreach_get("vertices", ev)
            self.e = ev.reshape(ne, 2)
            ls = np.empty(npoly, dtype=np.int64)
            lt = np.empty(npoly, dtype=np.int64)
            me.polygons.foreach_get("loop_start", ls)
            me.polygons.foreach_get("loop_total", lt)
            le = np.empty(nl, dtype=np.int64)
            me.loops.foreach_get("edge_index", le)
            pn = np.empty(npoly * 3, dtype=np.float64)
            me.polygons.foreach_get("normal", pn)
            pn = pn.reshape(npoly, 3)
        finally:
            oe.to_mesh_clear()
        offs = np.concatenate(([0], np.cumsum(lt)[:-1])) if npoly else np.zeros(0, np.int64)
        if not np.array_equal(ls, offs):
            raise RuntimeError(f"{obj.name}: polygon loops are not contiguous")
        self.loop_total = lt
        self.loop_edge = le
        self.loop_poly = np.repeat(np.arange(npoly), lt)
        nm = np.linalg.inv(mw[:3, :3]).T
        pn = pn @ nm.T
        ln = np.linalg.norm(pn, axis=1)
        ln[ln == 0] = 1.0
        self.pn = pn / ln[:, None]

    def bbox(self):
        return self.v.min(axis=0), self.v.max(axis=0)


def slice_loops(md, zc):
    """Closed section loops of the mesh with plane z=zc (edge-plane intersection).

    Returns (loops, n_open): loops = [{area, cx, cy, n}] (area m^2, area centroid);
    n_open = connected section components that are not a simple closed loop.
    """
    s = md.v[:, 2] - zc
    s = np.where(np.abs(s) < 1e-9, 1e-9, s)  # vertices on the plane count as above
    a, b = md.e[:, 0], md.e[:, 1]
    sa, sb = s[a], s[b]
    cross = (sa * sb) < 0
    t = np.zeros(len(md.e))
    t[cross] = sa[cross] / (sa[cross] - sb[cross])
    pts = md.v[a] + (md.v[b] - md.v[a]) * t[:, None]

    li = np.nonzero(cross[md.loop_edge])[0]
    pp = md.loop_poly[li]  # sorted: loops are contiguous per polygon
    ee = md.loop_edge[li]
    segs = []
    if len(li):
        _, start, cnt = np.unique(pp, return_index=True, return_counts=True)
        two = cnt == 2
        segs.extend(zip(ee[start[two]].tolist(), ee[start[two] + 1].tolist()))
        for st, c in zip(start[~two], cnt[~two]):
            for k in range(0, c - 1, 2):  # non-convex n-gon: pair in loop order
                segs.append((int(ee[st + k]), int(ee[st + k + 1])))

    adj = defaultdict(list)
    for x, y in segs:
        adj[x].append(y)
        adj[y].append(x)
    seen = set()
    loops, n_open = [], 0
    for s0 in adj:
        if s0 in seen:
            continue
        comp, stack = [], [s0]
        seen.add(s0)
        while stack:
            n = stack.pop()
            comp.append(n)
            for m in adj[n]:
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        if len(comp) < 3 or any(len(adj[n]) != 2 for n in comp):
            n_open += 1
            continue
        order, prev, cur = [s0], None, s0
        while True:
            n0, n1 = adj[cur]
            nxt = n1 if n0 == prev else n0
            if nxt == s0:
                break
            order.append(nxt)
            prev, cur = cur, nxt
            if len(order) > len(comp):
                break
        if len(order) != len(comp):
            n_open += 1
            continue
        p = pts[order][:, :2]
        x, y = p[:, 0], p[:, 1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cr = x * y1 - x1 * y
        area2 = cr.sum()
        if abs(area2) < 1e-14:
            n_open += 1
            continue
        cx = ((x + x1) * cr).sum() / (3.0 * area2)
        cy = ((y + y1) * cr).sum() / (3.0 * area2)
        loops.append({"area": abs(area2) / 2.0, "cx": cx, "cy": cy, "n": len(order)})
    return loops, n_open


def torso_slices(md):
    mn, mx = md.bbox()
    h = mx[2] - mn[2]
    out = []
    for f in np.linspace(TORSO_BAND[0], TORSO_BAND[1], N_SLICES):
        zc = mn[2] + f * h
        loops, n_open = slice_loops(md, zc)
        big = max(loops, key=lambda L: L["area"]) if loops else None
        out.append({"frac": round(float(f), 4), "z": zc, "n_loops": len(loops), "n_open": n_open,
                    "largest": big})
    return out


# ---------------------------------------------------------------- criteria
def crit_g1_1(ev, objs, missing):
    if missing:
        ev.criterion("G1.1", missing, "SRC_hi, SRC_club_hi MESH; loc 0, rot 0, scale 1 (tol 1e-6)",
                     False)
        return
    measured, ok = {}, True
    for name in (BODY, CLUB):
        o = objs.get(name)
        if o is None:
            measured[name] = "missing object"
            ok = False
            continue
        mw = np.array(o.matrix_world, dtype=np.float64)
        dev_mw = float(np.abs(mw - np.eye(4)).max())
        q = o.rotation_quaternion
        rot_dev = max(max(abs(x) for x in o.rotation_euler),
                      max(abs(q.w - 1.0), abs(q.x), abs(q.y), abs(q.z)),
                      abs(o.rotation_axis_angle[0]))
        loc_dev = max(abs(x) for x in o.location)
        scale_dev = max(abs(x - 1.0) for x in o.scale)
        delta_dev = max(max(abs(x) for x in o.delta_location),
                        max(abs(x) for x in o.delta_rotation_euler),
                        max(abs(x - 1.0) for x in o.delta_scale))
        this_ok = (o.type == "MESH" and loc_dev <= TOL_XFORM and rot_dev <= TOL_XFORM
                   and scale_dev <= TOL_XFORM and delta_dev <= TOL_XFORM and dev_mw <= TOL_XFORM)
        ok = ok and this_ok
        measured[name] = {
            "type": o.type, "parent": o.parent.name if o.parent else None,
            "location": list(o.location), "rotation_mode": o.rotation_mode,
            "rotation_euler": list(o.rotation_euler), "rotation_quaternion": list(q),
            "scale": list(o.scale), "max_abs_matrix_world_minus_I": dev_mw,
            "max_abs_delta_dev": delta_dev,
        }
    ev.criterion("G1.1", measured, "SRC_hi, SRC_club_hi MESH; loc 0, rot 0, scale 1 (tol 1e-6)", ok,
                 "rot checks euler, quaternion and axis-angle; matrix_world vs identity also checked "
                 "(covers parent/delta transforms)")


def crit_g1_2(ev, md, slices, missing):
    ths = {"G1.2a": "min z = 0 +-0.5 mm", "G1.2b": "torso egg center x = 0 +-1 mm",
           "G1.2c": "height = 1388 +-2 mm"}
    if missing or md is None:
        for k, th in ths.items():
            ev.criterion(k, missing or "missing object: SRC_hi", th, False)
        return
    mn, mx = md.bbox()
    h = mx[2] - mn[2]
    ev.criterion("G1.2a", {"min_z_mm": mm(mn[2])}, ths["G1.2a"], abs(mn[2]) <= TOL_MINZ,
                 "SRC_hi evaluated world vertices")
    found = [s for s in slices if s["largest"] is not None]
    cx = float(np.mean([s["largest"]["cx"] for s in found])) if found else None
    per = [{"frac": s["frac"], "z_mm": mm(s["z"]), "n_loops": s["n_loops"], "n_open": s["n_open"],
            "largest_area_cm2": (round(s["largest"]["area"] * 1e4, 3) if s["largest"] else None),
            "cx_mm": (mm(s["largest"]["cx"]) if s["largest"] else None),
            "cy_mm": (mm(s["largest"]["cy"]) if s["largest"] else None)} for s in slices]
    ok_b = cx is not None and len(found) == N_SLICES and abs(cx) <= TOL_CX
    ev.criterion("G1.2b", {"center_x_mm": mm(cx), "slices_with_loop": len(found), "slices": per},
                 ths["G1.2b"], ok_b,
                 f"{N_SLICES} horizontal slices z in [{TORSO_BAND[0]}H, {TORSO_BAND[1]}H] above min z; "
                 "edge-plane intersection, largest closed loop per slice, area centroid x averaged; "
                 "ok also requires a closed loop in every slice")
    ev.criterion("G1.2c", {"height_mm": mm(h), "max_z_mm": mm(mx[2])}, ths["G1.2c"],
                 abs(h - H_NOMINAL) <= TOL_H, "max z - min z of SRC_hi")


def crit_g1_3(ev, md, slices, missing):
    th = ("front is -Y: head feature-edge mean y < head bbox mid y and < torso center y, "
          "and front-half roughness > back-half roughness")
    if missing or md is None:
        ev.criterion("G1.3", missing or "missing object: SRC_hi", th, False)
        return
    note = (f"head region = vertices z > minz + {HEAD_FRAC}H. Per interior edge (exactly 2 faces, both "
            f"vertices in head) dihedral angle between face normals. Feature edges = top "
            f"{100 - FEATURE_PCT:.0f}% dihedral (eye rings, eyeballs, mouth creases); their mean midpoint y "
            "is compared with the head bbox mid y and the torso center y (mean largest-loop centroid y "
            "of the G1.2b slices). Roughness = mean dihedral of head edges in the -Y half vs +Y half "
            "(split at head bbox mid y).")
    mn, mx = md.bbox()
    h = mx[2] - mn[2]
    zc = mn[2] + HEAD_FRAC * h
    le, lp = md.loop_edge, md.loop_poly
    order = np.argsort(le, kind="stable")
    se = le[order]
    uniq, start, cnt = np.unique(se, return_index=True, return_counts=True)
    two = cnt == 2
    eidx = uniq[two]
    f1 = lp[order[start[two]]]
    f2 = lp[order[start[two] + 1]]
    va, vb = md.e[eidx, 0], md.e[eidx, 1]
    in_head = (md.v[va, 2] > zc) & (md.v[vb, 2] > zc)
    if in_head.sum() < 50:
        ev.criterion("G1.3", {"head_interior_edges": int(in_head.sum())}, th, False,
                     "detection failed: too few interior edges in head region. " + note)
        return
    dots = np.clip((md.pn[f1] * md.pn[f2]).sum(axis=1), -1.0, 1.0)
    ang = np.degrees(np.arccos(dots))[in_head]
    mid = ((md.v[va] + md.v[vb]) * 0.5)[in_head]
    hv = md.v[md.v[:, 2] > zc]
    head_mid_y = 0.5 * (hv[:, 1].min() + hv[:, 1].max())
    thr = float(np.percentile(ang, FEATURE_PCT))
    feat = ang >= thr
    feat_y = float(mid[feat, 1].mean())
    feat_x = float(mid[feat, 0].mean())
    front = mid[:, 1] < head_mid_y
    rough_f = float(ang[front].mean()) if front.any() else None
    rough_b = float(ang[~front].mean()) if (~front).any() else None
    feat_front_share = float((mid[feat, 1] < head_mid_y).mean())
    found = [s["largest"]["cy"] for s in slices if s["largest"] is not None]
    torso_y = float(np.mean(found)) if found else None
    measured = {
        "head_interior_edges": int(in_head.sum()), "feature_edges": int(feat.sum()),
        "feature_dihedral_threshold_deg": round(thr, 3),
        "feature_mean_y_mm": mm(feat_y), "feature_mean_x_mm": mm(feat_x),
        "feature_share_in_front_half": round(feat_front_share, 4),
        "head_bbox_y_mm": [mm(hv[:, 1].min()), mm(hv[:, 1].max())], "head_mid_y_mm": mm(head_mid_y),
        "torso_center_y_mm": mm(torso_y),
        "roughness_front_deg": None if rough_f is None else round(rough_f, 4),
        "roughness_back_deg": None if rough_b is None else round(rough_b, 4),
    }
    if torso_y is None or rough_f is None or rough_b is None:
        ev.criterion("G1.3", measured, th, False,
                     "detection failed: no torso slice loop or an empty head half. " + note)
        return
    ok = feat_y < head_mid_y and feat_y < torso_y and rough_f > rough_b
    ev.criterion("G1.3", measured, th, ok, note)


def crit_g1_4(ev, md_body, md_club, prep, missing_blend, missing_prep):
    th_a = "all SRC_club_hi vertices x < 0"
    th_b = "SRC_hi vertices within 30 mm of club_head_center = 0"
    if missing_blend:
        ev.criterion("G1.4a", missing_blend, th_a, False)
    elif md_club is None:
        ev.criterion("G1.4a", "missing object: SRC_club_hi", th_a, False)
    else:
        mxx = float(md_club.v[:, 0].max())
        ev.criterion("G1.4a", {"club_max_x_mm": mm(mxx), "club_vertices": len(md_club.v)}, th_a,
                     mxx < 0.0)
    miss = "; ".join(x for x in (missing_blend, missing_prep) if x)
    if miss:
        ev.criterion("G1.4b", miss, th_b, False)
        return
    if md_body is None:
        ev.criterion("G1.4b", "missing object: SRC_hi", th_b, False)
        return
    c = vec3(((prep or {}).get("club_split") or {}).get("club_head_center"))
    if c is None:
        ev.criterion("G1.4b", "source_prep.json club_split.club_head_center missing or invalid",
                     th_b, False)
        return
    d = np.linalg.norm(md_body.v - c, axis=1)
    n = int((d < CLUB_HEAD_R).sum())
    ev.criterion("G1.4b", {"count_within_30mm": n, "club_head_center": c.tolist(),
                           "nearest_SRC_hi_vertex_mm": mm(d.min())}, th_b, n == 0)


def _islands(nv, edges):
    """Number of connected components (vertices joined by edges)."""
    parent = np.arange(nv)

    def find(i):
        r = i
        while parent[r] != r:
            r = parent[r]
        while parent[i] != r:
            parent[i], i = r, parent[i]
        return r

    for a, b in edges.tolist():
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return len({find(i) for i in range(nv)})


def crit_g1_4c(ev, md_body, md_club, missing):
    th = "SRC_hi vertices inside shaft cylinder (r_shaft + 3 mm) with s > club s_max = 0"
    if missing:
        ev.criterion("G1.4c", missing, th, False)
        return
    if md_body is None or md_club is None:
        ev.criterion("G1.4c", "missing object: " + ", ".join(
            n for n, m in ((BODY, md_body), (CLUB, md_club)) if m is None), th, False)
        return
    cv = md_club.v
    c0 = cv.mean(axis=0)
    _, _, vt = np.linalg.svd(cv - c0, full_matrices=False)
    axis = vt[0] / np.linalg.norm(vt[0])
    s = (cv - c0) @ axis
    rad = np.linalg.norm((cv - c0) - s[:, None] * axis, axis=1)
    lo, hi = float(s.min()), float(s.max())
    length = hi - lo
    r_lo = float(np.median(rad[s <= lo + 0.1 * length]))
    r_hi = float(np.median(rad[s >= hi - 0.1 * length]))
    if r_hi > r_lo:  # head at the +s end -> flip so head is -s, butt is +s
        axis, s = -axis, -s
        lo, hi = -hi, -lo
    r_shaft = float(np.median(rad[s >= hi - 0.2 * length]))
    r_cyl = r_shaft + 0.003
    bs = (md_body.v - c0) @ axis
    brad = np.linalg.norm((md_body.v - c0) - bs[:, None] * axis, axis=1)
    inside = brad < r_cyl
    beyond = inside & (bs > hi)
    n = int(beyond.sum())
    ev.criterion("G1.4c", {
        "count_beyond_butt": n, "r_shaft_mm": mm(r_shaft), "cylinder_r_mm": mm(r_cyl),
        "club_s_range_mm": [mm(lo), mm(hi)],
        "end_radius_median_mm": {"head_end": mm(max(r_lo, r_hi)), "butt_end": mm(min(r_lo, r_hi))},
        "SRC_hi_in_cylinder": int(inside.sum()),
        "SRC_hi_in_cylinder_s_max_mm": mm(bs[inside].max()) if inside.any() else None,
        "club_islands": _islands(len(cv), md_club.e),
        "axis": axis.tolist(), "axis_origin": c0.tolist(),
    }, th, n == 0,
        "axis = PCA major axis of all SRC_club_hi vertices through their mean; s = axial coordinate "
        "(butt +, head -); head end = end 10% (of s length) with larger median radius; r_shaft = "
        "median axis distance of club vertices in top 20% of s length; source_prep.json not used")


def crit_g1_5(ev, pivots, md_body, md_club, missing):
    th = "all required keys present (grip_r with axis); co inside SRC_hi U SRC_club_hi bbox +-1 mm"
    if missing:
        ev.criterion("G1.5", missing, th, False)
        return
    mds = [m for m in (md_body, md_club) if m is not None]
    if not mds:
        ev.criterion("G1.5", "missing object: SRC_hi and SRC_club_hi", th, False)
        return
    mn = np.min([m.bbox()[0] for m in mds], axis=0)
    mx = np.max([m.bbox()[1] for m in mds], axis=0)
    piv = pivots.get("pivots") if isinstance(pivots, dict) else None
    if not isinstance(piv, dict):
        ev.criterion("G1.5", "pivots.json has no 'pivots' object", th, False)
        return
    missing_keys, invalid, outside = [], [], {}
    for k in REQUIRED_PIVOTS:
        if k not in piv:
            missing_keys.append(k)
            continue
        ent = piv[k] if isinstance(piv[k], dict) else {}
        co = vec3(ent.get("co"))
        if co is None:
            invalid.append(f"{k}.co")
            continue
        if k == "grip_r" and vec3(ent.get("axis")) is None:
            missing_keys.append("grip_r.axis")
        if np.any(co < mn - TOL_BBOX) or np.any(co > mx + TOL_BBOX):
            below = np.maximum(mn - co, 0.0)
            above = np.maximum(co - mx, 0.0)
            outside[k] = {"co": co.tolist(), "outside_by_mm": [mm(x) for x in (below + above)]}
    ok = not missing_keys and not invalid and not outside
    ev.criterion("G1.5", {"missing": missing_keys, "invalid": invalid, "out_of_bbox": outside,
                          "bbox_min": mn.tolist(), "bbox_max": mx.tolist(),
                          "n_pivots": len(piv), "extra_keys": sorted(set(piv) - set(REQUIRED_PIVOTS))},
                 th, ok, "bbox = world vertex bbox of SRC_hi U SRC_club_hi (evaluated meshes)")


def crit_g1_6(ev, pivots, missing):
    th = "report-only"
    if missing:
        ev.criterion("G1.6", missing, th, True, "reference value only; input missing")
        return
    piv = (pivots.get("pivots") if isinstance(pivots, dict) else None) or {}
    out = {}
    for p in PAIRS:
        el = piv.get(f"{p}_l")
        er = piv.get(f"{p}_r")
        cl = vec3(el.get("co")) if isinstance(el, dict) else None
        cr = vec3(er.get("co")) if isinstance(er, dict) else None
        if cl is None or cr is None:
            out[p] = None
            continue
        mr = cr * np.array([-1.0, 1.0, 1.0])
        dv = cl - mr
        out[p] = {"err_mm": mm(np.linalg.norm(dv)), "dxyz_mm": [mm(x) for x in dv]}
    ev.criterion("G1.6", out, th, True,
                 "|co_l - mirrorX(co_r)|, mirrorX(x,y,z)=(-x,y,z); null = pair missing/invalid")


# ---------------------------------------------------------------- main
_STATE = {"missing": []}


def _load_json(path, missing):
    if not path.exists():
        missing.append(rel(path))
        return None, f"missing input: {rel(path)}"
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f), ""


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    missing = _STATE["missing"]

    miss_blend = ""
    if not BLEND.exists():
        missing.append(rel(BLEND))
        miss_blend = f"missing input: {rel(BLEND)}"
    else:
        loaded = bpy.data.filepath
        if not loaded or Path(loaded).resolve() != BLEND.resolve():
            print(f"[{GATE}/{CHECKER}] opening {rel(BLEND)} (loaded: {loaded!r})")
            bpy.ops.wm.open_mainfile(filepath=str(BLEND))
        ev.add_input(BLEND)
    ev.add_stage_inputs("s01")

    prep, miss_prep = _load_json(SRC_PREP, missing)
    pivots, miss_piv = _load_json(PIVOTS, missing)

    objs, md_body, md_club, slices = {}, None, None, []
    if not miss_blend:
        for name in (BODY, CLUB):
            o = bpy.data.objects.get(name)
            if o is not None:
                objs[name] = o
        if objs.get(BODY) is not None and objs[BODY].type == "MESH":
            md_body = MeshData(objs[BODY])
            slices = torso_slices(md_body)
        if objs.get(CLUB) is not None and objs[CLUB].type == "MESH":
            md_club = MeshData(objs[CLUB])

    crit_g1_1(ev, objs, miss_blend)
    crit_g1_2(ev, md_body, slices, miss_blend)
    crit_g1_3(ev, md_body, slices, miss_blend)
    crit_g1_4(ev, md_body, md_club, prep, miss_blend, miss_prep)
    crit_g1_4c(ev, md_body, md_club, miss_blend)
    crit_g1_5(ev, pivots, md_body, md_club, "; ".join(x for x in (miss_blend, miss_piv) if x))
    crit_g1_6(ev, pivots, miss_piv)

    for p in RENDERS:
        if p.exists():
            ev.render(p)
    ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
