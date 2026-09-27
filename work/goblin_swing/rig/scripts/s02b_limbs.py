"""s02b_limbs.py - T9: arm / leg tubes + joint loops bridged to the torso ports.

Input : rig/work/r01a_body.blend (SRC_hi, SRC_club_hi, GOB_body_tmp with 4 open 12-vertex ports)
Output: rig/work/r01b_limbs.blend (SRC_hi, SRC_club_hi untouched + GOB_body_tmp incl. tubes, closed)
        rig/work/loops_limbs.json, rig/work/r01b_limbs_front.png, r01b_limbs_arm_r.png, r01b_limbs_leg_l.png

Run:  bl.ps1 -Script s02b_limbs.py -Blend work/r01a_body.blend

Per limb (left / right each from its own pivots.json frame, no mirroring):
  frame  o = pivots[port].co, a = pivots[port].axis, u = ref (+Z arm / -Y leg) orthogonalised to a,
         w = a x u; ring vertex m at azimuth 30 m deg (e = cos u + sin w), the same frame as the
         s02a port rings, so port vertex m bridges to tube vertex m (no twist).
  loops  (axial s along a from o):  port k2 | [arm: mid loops, <= 40 mm apart] | elbow/knee _0 _1 _2
         (center = pivot s, supports +-r_tube * SUPPORT_OFFSET_FACTOR; JOINT_MID_LOOPS unnamed "jmid" loops
         evenly between _0 / _1 and between _1 / _2) | [arm: mid loops] |
         wrist/ankle _0 (pivot s - 15 mm), _1 (pivot s) | extension loops | _end + quad cap.
  tube loop verts: ray from the axis point c(s) along e onto SRC_hi (first hit); except wrist/ankle _1 =
         tube-radius circle (pivot radius) kept END1_IN inside SRC (radius = min(tube radius, SRC ray
         distance - END1_IN)) and wrist/ankle _0 limited to tube radius where the SRC hit is
         beyond tube radius + END0_TOL (the SRC flare is covered by the rigid shell, not the tube).
  extension (inside hand / shoe): _end loop = first s (1 mm steps, >= _1 + 10 mm) where every ring
         vertex at r_tube lies >= 10 mm inside SRC_hi (and the cap verts >= 2 mm); loops between
         _1 and _end every <= EXT_STEP at r_tube, shrunk per vertex only where needed to stay >= 2 mm
         inside SRC_hi.  Cap = 3 x 3 quad grid (Coons patch of the end ring, slightly domed outward).
Body vertices keep their indices (tube verts are appended); body rings are remapped by position check.
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

# support loop offset = r_tube * factor (T16 adjusts only this)
SUPPORT_OFFSET_FACTOR = {"elbow": 0.6, "knee": 0.6}
# unnamed loops evenly between _0 / _1 and between _1 / _2 of the elbow / knee (T16b: 60 deg section ratio)
JOINT_MID_LOOPS = {"elbow": 2, "knee": 1}

MESH_NAME = "GOB_body_tmp"
IN_LOOPS = goblib.WORK / "loops_body.json"
OUT_BLEND = goblib.WORK / "r01b_limbs.blend"
OUT_LOOPS = goblib.WORK / "loops_limbs.json"
OUT_FRONT = goblib.WORK / "r01b_limbs_front.png"
OUT_ARM_R = goblib.WORK / "r01b_limbs_arm_r.png"
OUT_LEG_L = goblib.WORK / "r01b_limbs_leg_l.png"

N_RING = 12
END_PAIR_GAP = 0.015        # wrist_x_0 / ankle_x_0 this far body-side of the pivot
END0_TOL = 0.001            # wrist_x_0 / ankle_x_0 verts (on SRC) limited to tube radius when beyond it + this
END1_IN = 0.0005            # wrist_x_1 / ankle_x_1 vertex radius = min(tube radius, SRC ray distance - this)
MID_LOOP_MAX_GAP = 0.040    # arm mid loops (port..elbow_0, elbow_2..wrist_0) spacing <= this
EXT_STEP = 0.010            # extension loops (between _1 and _end) spacing <= this
END_DEPTH_MIN = 0.010       # _end ring verts inside SRC_hi
EXT_DEPTH_MIN = 0.002       # every extension vertex inside SRC_hi
EXT_SHRINK_TARGET = 0.0025  # radius shrink target (>= EXT_DEPTH_MIN + margin)
END_SEARCH = (0.010, 0.150, 0.001)   # _end search: from _1 + 10 mm to _1 + 150 mm, 1 mm steps
CAP_DOME = 0.25             # cap interior pushed outward along the tube by <= CAP_DOME * r_tube

# (limb, side, port pivot, mid joint, end joint, ref, mid loops)
LIMBS = (("arm", "l", "shoulder_l", "elbow_l", "wrist_l", (0.0, 0.0, 1.0), True),
         ("arm", "r", "shoulder_r", "elbow_r", "wrist_r", (0.0, 0.0, 1.0), True),
         ("leg", "l", "hip_l", "knee_l", "ankle_l", (0.0, -1.0, 0.0), False),
         ("leg", "r", "hip_r", "knee_r", "ankle_r", (0.0, -1.0, 0.0), False))


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
        v = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", v)
        v = v.reshape(-1, 3)
        m = np.array(ob.matrix_world)
        if not np.allclose(m, np.eye(4), atol=1e-9):
            v = v @ m[:3, :3].T + m[:3, 3]
        me.calc_loop_triangles()
        t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
        me.loop_triangles.foreach_get("vertices", t)
        self.V = v
        self.T = t.reshape(-1, 3)
        self.bvh = BVHTree.FromPolygons(v.tolist(), self.T.tolist())

    def ray(self, o, d, dist=0.3):
        loc, _n, _i, dd = self.bvh.ray_cast(Vector(o), Vector(d), dist)
        return (np.array(loc), dd) if loc is not None else (None, None)

    def dist(self, p):
        loc, _n, _i, d = self.bvh.find_nearest(Vector(p))
        return d if loc is not None else float("inf")

    def inside(self, p):
        """ray parity over 3 skew rays (majority)."""
        votes = 0
        for d in ((0.577, 0.577, 0.578), (-0.6, 0.3, 0.742), (0.2, -0.7, -0.686)):
            n = 0
            o = Vector(p)
            dv = Vector(d).normalized()
            for _ in range(64):
                loc, _nn, _i, _dd = self.bvh.ray_cast(o, dv, 10.0)
                if loc is None:
                    break
                n += 1
                o = loc + dv * 1e-5
            votes += n % 2
        return votes >= 2

    def depth(self, p):
        """signed inside depth: + inside, - outside (m)."""
        d = self.dist(p)
        return d if self.inside(p) else -d


# ---------------------------------------------------------------- body input
def body_arrays(ob):
    me = ob.data
    X = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", X)
    X = X.reshape(-1, 3)
    m = np.array(ob.matrix_world)
    if not np.allclose(m, np.eye(4), atol=1e-9):
        raise RuntimeError(f"{MESH_NAME} has a non-identity transform")
    faces = [tuple(p.vertices) for p in me.polygons]
    return X, faces


def frame(piv, port, ref):
    p = piv[port]
    o = np.array(p["co"], float)
    a = np.array(p["axis"], float)
    a /= np.linalg.norm(a)
    r = np.array(ref, float)
    u = r - (r @ a) * a
    u /= np.linalg.norm(u)
    w = np.cross(a, u)
    return o, a, u, w


def dirs(u, w):
    return [math.cos(math.radians(30.0 * m)) * u + math.sin(math.radians(30.0 * m)) * w for m in range(N_RING)]


def azimuths(P, o, a, u, w):
    d = P - o
    s = d @ a
    rad = d - s[:, None] * a
    return s, np.linalg.norm(rad, axis=1), np.degrees(np.arctan2(rad @ w, rad @ u))


# ---------------------------------------------------------------- limb build
def surface_loop(o, a, E, s):
    c = o + s * a
    pts = []
    for m, e in enumerate(E):
        loc, _ = SRC.ray(c, e)
        if loc is None:
            raise RuntimeError(f"no SRC_hi hit from axis s={mm(s)} mm az={30 * m}")
        pts.append(loc)
    return np.array(pts)


def cap_grid(ring, c, a, r_tube):
    """4x4 grid (3x3 quads) whose 12 boundary verts are the ring (corners at m = 0,3,6,9).
    Returns (interior points dict {(i,j): p}, boundary map {(i,j): m})."""
    per = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (2, 3), (3, 3), (3, 2), (3, 1), (3, 0), (2, 0), (1, 0)]
    G = {ij: ring[m] for m, ij in enumerate(per)}
    bmap = {ij: m for m, ij in enumerate(per)}
    inner = {}
    for i in (1, 2):
        for j in (1, 2):
            uu, vv = i / 3.0, j / 3.0
            p = ((1 - uu) * G[(0, j)] + uu * G[(3, j)] + (1 - vv) * G[(i, 0)] + vv * G[(i, 3)]
                 - ((1 - uu) * (1 - vv) * G[(0, 0)] + uu * (1 - vv) * G[(3, 0)]
                    + (1 - uu) * vv * G[(0, 3)] + uu * vv * G[(3, 3)]))
            d = p - c
            d = d - (d @ a) * a
            rho = min(1.0, float(np.linalg.norm(d)) / r_tube)
            inner[(i, j)] = p + a * (CAP_DOME * r_tube * (1.0 - rho * rho))
    return inner, bmap


def build_limb(piv, limb, side, port, mid, end, ref, mid_loops, port_verts, X):
    o, a, u, w = frame(piv, port, ref)
    E = dirs(u, w)
    # port phase check (vertex m must sit at azimuth 30 m)
    Pp = X[np.array(port_verts)]
    s_p, _r_p, az_p = azimuths(Pp, o, a, u, w)
    dphi = (az_p - 30.0 * np.arange(N_RING) + 180.0) % 360.0 - 180.0
    if np.abs(dphi).max() > 15.0:
        raise RuntimeError(f"{port}_2 phase mismatch: az dev {np.round(dphi, 1).tolist()}")
    jm = mid.split("_")[0]
    je = end.split("_")[0]
    r_mid = float(piv[mid]["radius"])
    r_end = float(piv[end]["radius"])
    s_mid = float((np.array(piv[mid]["co"]) - o) @ a)
    s_end = float((np.array(piv[end]["co"]) - o) @ a)
    off = r_mid * SUPPORT_OFFSET_FACTOR[jm]
    s_port = float(s_p.mean())

    named = [(f"{jm}_{side}_0", s_mid - off, 0), (f"{jm}_{side}_1", s_mid, 1), (f"{jm}_{side}_2", s_mid + off, 2),
             (f"{je}_{side}_0", s_end - END_PAIR_GAP, 0), (f"{je}_{side}_1", s_end, 1)]
    seq = []   # (name or None, s, kind)
    if mid_loops:
        gap = named[0][1] - s_port
        n = max(1, math.ceil(gap / MID_LOOP_MAX_GAP))
        seq += [(None, s_port + gap * i / n, "mid") for i in range(1, n)]
    for i, (nm, s, _k) in enumerate(named[:3]):
        seq.append((nm, s, "joint"))
        if i < 2:
            n = JOINT_MID_LOOPS[jm] + 1
            seq += [(None, s + (named[i + 1][1] - s) * q / n, "jmid") for q in range(1, n)]
    if mid_loops:
        gap = named[3][1] - named[2][1]
        n = max(1, math.ceil(gap / MID_LOOP_MAX_GAP))
        seq += [(None, named[2][1] + gap * i / n, "mid") for i in range(1, n)]
    seq += [(nm, s, "joint") for nm, s, _k in named[3:]]
    ss = [s for _n, s, _k in seq]
    if any(b <= a_ for a_, b in zip(ss, ss[1:])) or ss[0] <= float(s_p.max()):
        raise RuntimeError(f"{limb}_{side}: loop order broken s={[mm(x) for x in ss]} port max s={mm(s_p.max())}")

    loops = []   # dicts: name, s, kind, P
    n_clamp0 = 0
    for nm, s, kind in seq:
        c = o + s * a
        if nm == f"{je}_{side}_1":        # tube-radius circle at the pivot, kept END1_IN inside SRC
            P = []
            for e in E:
                loc, dd = SRC.ray(c, e)
                rr = r_end if loc is None else min(r_end, dd - END1_IN)
                while SRC.depth(c + rr * e) < END1_IN and rr > 0.5 * r_end:   # nearest-surface check
                    rr -= 0.0002
                P.append(c + rr * e)
            P = np.array(P)
        else:
            P = surface_loop(o, a, E, s)
            if nm == f"{je}_{side}_0":    # on SRC, but not beyond tube radius + END0_TOL
                for m in range(N_RING):
                    rr = float(np.linalg.norm(P[m] - c))
                    if rr > r_end + END0_TOL:
                        P[m] = c + (P[m] - c) * (r_end / rr)
                        n_clamp0 += 1
        loops.append({"name": nm, "s": s, "kind": kind, "P": P})

    # ---- extension: find _end
    s1 = s_end
    s_e = None
    lo, hi, st = END_SEARCH
    cap_inner = None
    for k in range(int(round((hi - lo) / st)) + 1):
        s = s1 + lo + k * st
        c = o + s * a
        ring = np.array([c + r_end * e for e in E])
        dep = np.array([SRC.depth(p) for p in ring])
        if dep.min() < END_DEPTH_MIN:
            continue
        inner, bmap = cap_grid(ring, c, a, r_end)
        dci = np.array([SRC.depth(p) for p in inner.values()])
        if dci.min() < EXT_DEPTH_MIN:
            continue
        s_e, end_ring, cap_inner, cap_bmap = s, ring, inner, bmap
        break
    if s_e is None:
        raise RuntimeError(f"{limb}_{side}: no _end position with ring depth >= {mm(END_DEPTH_MIN)} mm")
    n = max(1, math.ceil((s_e - s1) / EXT_STEP))
    n_shrunk, r_min = 0, r_end
    for i in range(1, n):
        s = s1 + (s_e - s1) * i / n
        c = o + s * a
        P = []
        for e in E:
            r = r_end
            while SRC.depth(c + r * e) < EXT_SHRINK_TARGET and r > 0.2 * r_end:
                r -= 0.0005
            if r < r_end:
                n_shrunk += 1
                r_min = min(r_min, r)
            P.append(c + r * e)
        loops.append({"name": None, "s": s, "kind": "ext", "P": np.array(P)})
    loops.append({"name": f"{je}_{side}_end", "s": s_e, "kind": "end", "P": end_ring})
    info = {"frame": (o, a, u, w), "s_port": s_port, "s_port_min": float(s_p.min()), "s_port_max": float(s_p.max()),
            "r_mid": r_mid, "r_end": r_end, "off": off, "s_mid": s_mid, "s_end": s_end,
            "ext_shrunk_verts": n_shrunk, "end0_clamped": n_clamp0, "ext_r_min": r_min, "cap_inner": cap_inner, "cap_bmap": cap_bmap}
    return loops, info


# ---------------------------------------------------------------- mesh
def ring_closed(vs, ekeys, nv):
    vs = list(vs)
    return all((min(x, y) * nv + max(x, y)) in ekeys for x, y in zip(vs, vs[1:] + vs[:1]))


def topo(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    nb = sum(1 for e in bm.edges if len(e.link_faces) == 1)
    nm = sum(1 for e in bm.edges if len(e.link_faces) > 2)
    nw = sum(1 for e in bm.edges if len(e.link_faces) == 0)
    tris = sum(len(f.verts) - 2 for f in bm.faces)
    quads = sum(1 for f in bm.faces if len(f.verts) == 4)
    vol = bm.calc_volume(signed=True)
    rep = {"verts": len(bm.verts), "faces": len(bm.faces), "quads": quads, "tris": tris, "boundary": nb,
           "nonmanifold": nm, "wire": nw, "signed_volume_m3": round(vol, 6)}
    bm.free()
    return rep


def main():
    global SRC
    piv = goblib.load_json("pivots.json")["pivots"]
    SRC = Src()
    ob = bpy.data.objects.get(MESH_NAME)
    if ob is None or ob.type != "MESH":
        raise RuntimeError(f"missing input object {MESH_NAME}")
    with open(IN_LOOPS, "r", encoding="utf-8") as f:
        body_loops = json.load(f)
    X0, faces0 = body_arrays(ob)
    nv0 = len(X0)
    print(f"[s02b] SRC_hi verts={len(SRC.V)} tris={len(SRC.T)}; {MESH_NAME} verts={nv0} faces={len(faces0)}")
    t0 = topo(ob.data)
    print(f"[s02b] input body: boundary={t0['boundary']} nonmanifold={t0['nonmanifold']} tris={t0['tris']}")

    P_new = []
    new_faces = []
    limb_rings = {}
    limb_meta = {}
    ext_ids, tube_ids = [], []

    def add(p):
        P_new.append(np.array(p, float))
        return nv0 + len(P_new) - 1

    for limb, side, port, mid, end, ref, mid_loops in LIMBS:
        port_ring = body_loops["rings"][f"{port}_2"]["verts"]
        loops, info = build_limb(piv, limb, side, port, mid, end, ref, mid_loops, port_ring, X0)
        prev = list(port_ring)
        rows = [("port", f"{port}_2", info["s_port"])]
        for lp in loops:
            ids = [add(p) for p in lp["P"]]
            if lp["kind"] in ("ext", "end"):
                ext_ids.extend(ids)
            else:
                tube_ids.extend(ids)
            for m in range(N_RING):
                m1 = (m + 1) % N_RING
                new_faces.append((prev[m], prev[m1], ids[m1], ids[m]))
            prev = ids
            lp["ids"] = ids
            rows.append((lp["kind"], lp["name"] or "-", lp["s"]))
            if lp["name"]:
                jn, k = lp["name"].rsplit("_", 1)
                kk = int(k) if k.isdigit() else 2     # _end: next ordinal after _1
                limb_rings[lp["name"]] = {"verts": [int(v) for v in ids], "joint": jn, "k": kk,
                                          "center": lp["name"] in (f"elbow_{side}_1", f"knee_{side}_1")}
        # cap
        end_ids = loops[-1]["ids"]
        gid = {ij: end_ids[m] for ij, m in info["cap_bmap"].items()}
        for ij, p in info["cap_inner"].items():
            gid[ij] = add(p)
            ext_ids.append(gid[ij])
        for i in range(3):
            for j in range(3):
                new_faces.append((gid[(i, j)], gid[(i + 1, j)], gid[(i + 1, j + 1)], gid[(i, j + 1)]))
        info["rows"] = rows
        info["loops"] = loops
        limb_meta[f"{limb}_{side}"] = info

    # ---- new mesh (body verts first, unchanged order)
    X = np.vstack([X0, np.array(P_new)])
    me = bpy.data.meshes.new(MESH_NAME + "_new")
    me.from_pydata([tuple(p) for p in X], [], [tuple(f) for f in faces0] + new_faces)
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()
    old = ob.data
    ob.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    me.name = MESH_NAME

    Xn = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", Xn)
    Xn = Xn.reshape(-1, 3)
    E = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    nv = len(Xn)
    ekeys = set((np.minimum(E[:, 0], E[:, 1]) * nv + np.maximum(E[:, 0], E[:, 1])).tolist())

    # ---- rings json (body rings remapped by position check)
    n_remap = 0
    rings = {}
    for name, r in body_loops["rings"].items():
        vs = []
        for v in r["verts"]:
            if v < nv and np.linalg.norm(Xn[v] - X0[v]) < 1e-7:
                vs.append(int(v))
            else:
                vs.append(int(np.argmin(np.linalg.norm(Xn - X0[v], axis=1))))
                n_remap += 1
        rr = dict(r)
        rr["verts"] = vs
        rings[name] = rr
    rings.update(limb_rings)
    with open(OUT_LOOPS, "w", encoding="utf-8") as f:
        json.dump({"mesh": MESH_NAME, "rings": rings}, f, indent=1)

    # ---- save (input untouched)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)

    # ---- renders
    goblib.ortho_render([ob], "front", OUT_FRONT, wire=True)
    so, wr = np.array(piv["shoulder_r"]["co"]), np.array(piv["wrist_r"]["co"])
    goblib.ortho_render([ob], "front", OUT_ARM_R, wire=True,
                        frame_bbox=((wr[0] - 0.10, -0.12, so[2] - 0.12), (so[0] + 0.12, 0.18, so[2] + 0.12)))
    hp, an = np.array(piv["hip_l"]["co"]), np.array(piv["ankle_l"]["co"])
    goblib.ortho_render([ob], "front", OUT_LEG_L, wire=True,
                        frame_bbox=((hp[0] - 0.14, -0.15, an[2] - 0.09), (hp[0] + 0.14, 0.15, hp[2] + 0.07)))

    # ---- reports
    tp = topo(me)
    print("[s02b] OUTPUT " + json.dumps({"blend": str(OUT_BLEND), "loops": str(OUT_LOOPS),
                                         "renders": [str(OUT_FRONT), str(OUT_ARM_R), str(OUT_LEG_L)]}))
    print(f"[s02b] TRIS {tp['tris']} (quads {tp['quads']}, faces {tp['faces']}, verts {tp['verts']}; "
          f"body input tris {t0['tris']}, added {tp['tris'] - t0['tris']})")
    print(f"[s02b] BOUNDARY_EDGES {tp['boundary']} NONMANIFOLD_EDGES {tp['nonmanifold']} WIRE_EDGES {tp['wire']} "
          f"signed_volume={tp['signed_volume_m3']} m3")
    print(f"[s02b] body ring verts remapped: {n_remap}")
    print("[s02b] RINGS (name: n_verts closed_loop k center):")
    for nm_, r in rings.items():
        print(f"   {nm_}: n={len(r['verts'])} closed={ring_closed(r['verts'], ekeys, nv)} k={r['k']} "
              f"center={r['center']}")

    print("[s02b] CENTER_TO_PIVOT (ring vertex mean vs pivot):")
    for j in ("elbow", "knee"):
        for x in ("l", "r"):
            c = Xn[np.array(rings[f"{j}_{x}_1"]["verts"])].mean(axis=0)
            pc = np.array(piv[f"{j}_{x}"]["co"])
            print(f"   {j}_{x}_1: center={np.round(c, 4).tolist()} pivot={np.round(pc, 4).tolist()} "
                  f"dist={mm(np.linalg.norm(c - pc))} mm")

    print("[s02b] AXIAL_SPACING (s along the limb axis from the port pivot; mm; ring s = vertex mean):")
    for key, info in limb_meta.items():
        o, a, _u, _w = info["frame"]
        print(f"   {key}: r_mid={mm(info['r_mid'])} support_off={mm(info['off'])} "
              f"port_2 s range=[{mm(info['s_port_min'])},{mm(info['s_port_max'])}]")
        prev_s, prev_min = None, None
        rows = [("port", info["rows"][0][1], info["s_port"], info["s_port_min"], info["s_port_max"])]
        for lp in info["loops"]:
            sv = (Xn[np.array(lp["ids"])] - o) @ a
            rows.append((lp["kind"], lp["name"] or "-", float(sv.mean()), float(sv.min()), float(sv.max())))
        for kind, nm_, s, smin, smax in rows:
            gap = "" if prev_s is None else f" gap={mm(s - prev_s)} min_gap={mm(smin - prev_min)}"
            print(f"      {kind:5s} {nm_:14s} s={mm(s):8.2f}{gap}")
            prev_s, prev_min = s, smax
        if key.startswith("leg"):
            side = key[-1]
            k0 = [r for r in rows if r[1] == f"knee_{side}_0"][0]
            k2 = [r for r in rows if r[1] == f"knee_{side}_2"][0]
            a0 = [r for r in rows if r[1] == f"ankle_{side}_0"][0]
            print(f"      KNEE_SUPPORT_OVERLAP knee_{side}_0 min s - hip_{side}_2 max s = {mm(k0[3] - info['s_port_max'])} mm, "
                  f"ankle_{side}_0 min s - knee_{side}_2 max s = {mm(a0[3] - k2[4])} mm, "
                  f"overlap={bool(k0[3] <= info['s_port_max'] or a0[3] <= k2[4])}")
        print(f"      extension: verts shrunk below r_tube={info['ext_shrunk_verts']} "
              f"min r={mm(info['ext_r_min'])} (r_tube {mm(info['r_end'])})")

    print("[s02b] WRIST_ANKLE_RINGS (radius about the limb axis, mm; SRC depth: + inside SRC, - outside, mm):")
    for key, info in limb_meta.items():
        o, a, _u, _w = info["frame"]
        je = "wrist" if key.startswith("arm") else "ankle"
        side = key[-1]
        for k in ("0", "1"):
            vs = np.array(rings[f"{je}_{side}_{k}"]["verts"])
            d = Xn[vs] - o
            sv = d @ a
            rr = np.linalg.norm(d - np.outer(sv, a), axis=1)
            dep = np.array([SRC.depth(Xn[v]) for v in vs])
            print(f"   {je}_{side}_{k}: s={mm(sv.mean())} r min/mean/max={mm(rr.min())}/{mm(rr.mean())}/{mm(rr.max())} "
                  f"(tube r {mm(info['r_end'])}) SRC depth min/mean/max={mm(dep.min())}/{mm(dep.mean())}/{mm(dep.max())}"
                  + (f" clamped verts={info['end0_clamped']}" if k == "0" else ""))
    print("[s02b] END_RING_DEPTH (inside SRC_hi, mm):")
    for x in ("l", "r"):
        for j in ("wrist", "ankle"):
            vs = rings[f"{j}_{x}_end"]["verts"]
            d = np.array([SRC.depth(Xn[v]) for v in vs])
            print(f"   {j}_{x}_end: min={mm(d.min())} mean={mm(d.mean())} max={mm(d.max())}")
    dext = np.array([SRC.depth(Xn[v]) for v in ext_ids])
    print(f"[s02b] EXTENSION_VERTS n={len(ext_ids)} outside_SRC={int((dext < 0).sum())} "
          f"depth<{mm(EXT_DEPTH_MIN)}mm={int((dext < EXT_DEPTH_MIN).sum())} min_depth={mm(dext.min())} mm")
    dt = np.array([SRC.dist(Xn[v]) for v in tube_ids])
    print(f"[s02b] TUBE_LOOP_VERTS_DIST_TO_SRC n={len(dt)} mean={mm(dt.mean())} p95={mm(np.percentile(dt, 95))} "
          f"max={mm(dt.max())} mm")
    print("[s02b] done")


SRC = None

if __name__ == "__main__":
    goblib.run_main(main)
