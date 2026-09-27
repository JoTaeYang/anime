"""p20_retopo.py - T220 (P2.1): rig-ready retopo of the Clay Explorer (spec d-02 §1, §2 P2.1).

Input : C:/Users/whxod/Downloads/Meshy_AI_Clay_Explorer_0926122001_generate.fbx (read only; sha256 checked vs P0b)
        work/player/inspect/P0b/p0b_measure.json (landmarks, parts, skirt, colours; read only)
Output: work/player/rig/pl_r01_retopo.blend  collection `PL` with one mesh object `PL_mesh` (identity transform):
            all 20 parts as separate shells, face INT attribute `part_id`, one material per part (M_<part>, P0b
            debug colour, material_index = part_id), custom normals, UV map `UVMap`
        work/player/rig/data/parts.json         part_id map, slot map, rigid / rebuilt lists
        work/player/rig/data/retopo_loops.json  joint rings (vertex index loops of PL_mesh), skirt rings / columns
        work/player/inspect/P2a/*.png           part colours (front / side / back / 3/4), wireframe close-ups
                                               (elbow, knee, shoulder, skirt outside / below), UV layout

CLI:  blender --background --factory-startup --python p20_retopo.py

Copied / adapted logic (the goblin scripts are not imported and not modified):
- source import + part split: work/player/rig/scripts/p10_crude_rig.py (import_source, split_parts, sym, mirror)
- Surf (BVH nearest / ray parity inside / signed depth), sample_surface, occluded: work/goblin_swing/rig/scripts/
  s02c_rigid.py
- tube rings (12 verts, 30 deg azimuth frame from the limb axis and a reference direction, ray from the axis onto the
  source), joint support loops at r * SUPPORT_OFFSET_FACTOR, JOINT_MID_LOOPS, mid loops <= MID_LOOP_MAX_GAP, wrist
  _0 / _1 pair END_PAIR_GAP apart, extension into the rigid shell until the end ring is END_DEPTH_MIN inside, Coons
  cap grid domed by CAP_DOME: work/goblin_swing/rig/scripts/s02b_limbs.py
- straddle (each visible vertex moves along its normal by -0.5 x the mean signed distance of its faces' centres to the
  source, STRADDLE_PASSES passes): finish_head in s02c_rigid.py
- merge with part_id, custom normals (smooth, sharp only at designed hard edges, default corner normals stored as
  custom normals = the s02d head rule), UVs (make_uvs: smart project -> seams from islands -> angle-based unwrap ->
  average island scale -> head-front islands scaled -> pack), uv_report (2048 raster overlap), write_uv_png:
  work/goblin_swing/rig/scripts/s02d_finish.py
- ortho_render: work/goblin_swing/rig/scripts/goblib.py (plus views `below` and `below_front`)

Parts:
- rigid (source geometry unchanged; every source part already has consistent outward normals, 0 flips by bmesh
  recalc, so nothing is cleaned): head, eyes, scarf, scarf_tail, belt, pouch, sleeves, fists, cuffs, shoes.
  The shoes keep their open top rim (hidden inside the cuff); T222: shoe rim vertices at or outside the cuff
  surface (depth <= 0) are moved along the inward normal of the nearest cuff face until RIM_TUCK_DEPTH inside.
- arm_x: closed quad tube along shoulder -> wrist (P0b landmarks, symmetrised as p10). Rings (s from the shoulder
  pivot): shoulder_x_0 (capped end, inside the sleeve), shoulder_x_1, mid loops, elbow_x_0/1/2 (+ JOINT_MID_LOOPS),
  mid loops, wrist_x_0 / wrist_x_1 (wrist pivot), extension loops, wrist_x_end (inside the fist) + cap.
- leg_x: closed quad tube along hip -> ankle. Rings: hip_x_0 (capped end above the hip pivot, inside the tunic
  above the skirt ceiling), hip_x_1, mid loops, knee_x_0/1/2 (+ JOINT_MID_LOOPS), mid loops, ankle_x_0 / ankle_x_1
  (ankle pivot, inside the cuff), ankle_x_end (inside the shoe, under the cuff) + cap.
- tunic: N_COL columns (azimuth k * 360 / N_COL from the front (-Y) toward character left (+X); every 4th column is a
  skirt azimuth F, FL, L, BL, B, BR, R, FR), rows = horizontal rays from outside onto the source tunic at the row
  heights (hem, skirt rings, belt bottom, belt band, torso rows evenly spaced by arc length), neck rim row = rays
  from inside (NECK_ORIGIN_Z) raised per column until the point is NECK_HIDE inside the head (open rim, hidden).
  The bottom cap is removed; the hem is a SKIRT_T thick roll to an inner wall (outer skirt rows moved SKIRT_T
  horizontally toward the axis) that runs up to a flat ceiling at CEILING_Z (Coons 8 x 8 quad grid, corners at the
  diagonal azimuths); the leg tubes pass through the ceiling (designed overlap).
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
SRC = Path(r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Explorer_0926122001_generate.fbx")
P0B = REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json"
OUT_BLEND = RIG / "pl_r01_retopo.blend"
PARTS_JSON = RIG / "data" / "parts.json"
LOOPS_JSON = RIG / "data" / "retopo_loops.json"
INSPECT = REPO / "work" / "player" / "inspect" / "P2a"
MESH = "PL_mesh"
COLL = "PL"

PART_NAMES = ["head", "eye_l", "eye_r", "scarf", "scarf_tail", "tunic", "belt", "pouch", "sleeve_l", "sleeve_r",
              "arm_l", "arm_r", "fist_l", "fist_r", "leg_l", "leg_r", "cuff_l", "cuff_r", "shoe_l", "shoe_r"]
SLOTS = {"Player_Head": ["head", "eye_l", "eye_r"],
         "Armor_Default": [n for n in PART_NAMES if n not in ("head", "eye_l", "eye_r")]}
REBUILT = ["tunic", "arm_l", "arm_r", "leg_l", "leg_r"]
RIGID = [n for n in PART_NAMES if n not in REBUILT]
TRI_BUDGET = 7000

# ---- tubes (s02b values unless noted)
N_RING = 12
SUPPORT_OFFSET_FACTOR = {"elbow": 0.6, "knee": 0.6}
JOINT_MID_LOOPS = {"elbow": 2, "knee": 1}
MID_LOOP_MAX_GAP = 0.040
END_PAIR_GAP = 0.015
EXT_STEP = 0.010
END_DEPTH_MIN = 0.008       # wrist_x_end ring verts this far inside the fist (goblin 10 mm; fist r 76 mm)
EXT_DEPTH_MIN = 0.002       # wrist cap interior verts inside the fist
END_SEARCH = (0.010, 0.080, 0.001)
CAP_DOME = 0.25
ARM_ROOT_S = 0.060          # shoulder_x_0 (capped end): source tube root rim at s 0.049 .. 0.055, inside the sleeve
ARM_SH1_S = 0.090           # shoulder_x_1 (inside the sleeve: sleeve s 0.015 .. 0.18, r 0.06)
LEG_TOP_S = -0.020          # hip_x_0 (capped end) 20 mm above the hip pivot, inside the tunic above the ceiling
LEG_HIP1_S = 0.014          # hip_x_1 (source leg top rim z 0.4136 .. 0.4249)
ANKLE_END_S = 0.006         # ankle_x_end this far past the ankle pivot (source leg bottom z 0.101 .. 0.105)
RAY_MAX = 0.12
RIM_TUCK_DEPTH = 0.001      # T222: shoe rim verts outside the cuff end this far inside it
RIM_TUCK_STEP = 0.00005
STRADDLE_PASSES = 2

# ---- tunic
N_COL = 32
SKIRT_AZ = [("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315)]
Z_HEM = 0.290               # outer hem ring (source outer rim z 0.2879 .. 0.2933)
Z_SKIRT = [0.318, 0.348, 0.378]   # skirt rings between the hem and the belt bottom
Z_BELT_BOT = 0.408
Z_BELT_MID = 0.440
Z_BELT_TOP = 0.473
Z_TORSO_TOP = 0.740         # last horizontal-ray row (source wall above this is partly the top cap)
TORSO_ARC_MAX = 0.032       # torso rows (belt top .. Z_TORSO_TOP) evenly by mean arc length, spacing <= this
NECK_ORIGIN_Z = 0.700       # neck rim: rays from (cx, cy, this) in the column's meridian plane
NECK_HIDE = 0.003           # neck rim verts at least this far inside the head
SKIRT_T = 0.004             # hem roll / double-wall thickness
CEILING_Z = 0.400           # skirt ceiling (inner wall top); legs pass through it
CEIL_N = 8                  # ceiling Coons grid CEIL_N x CEIL_N quads (4 * CEIL_N = N_COL)
CAP_NZ = -0.9               # source bottom cap tris: normal z < this and z < CAP_ZMAX (P0b definition)
CAP_ZMAX = 0.312
TOP_ZMIN = 0.745            # source top region (inside the head) excluded from the tunic straddle

# ---- normals / UV (s02d values)
SHARP_ANGLE_RIGID = math.radians(60.0)
UV_ANGLE_LIMIT = math.radians(66.0)
PACK_MARGIN = 0.005
HEAD_FRONT_DOT = 0.7
HEAD_FRONT_SCALE = 1.45
UV_RES = 2048
UV_MATCH = 1e-6

# ---- deviation report
DEV_SAMPLES = 20000
HIDE_DEPTH = 0.001
RAY_DIRS = [Vector(d).normalized() for d in ((0.577, 0.577, 0.578), (-0.6, 0.3, 0.742), (0.2, -0.7, -0.686))]


def log(msg):
    print(f"[p20] {msg}")
    sys.stdout.flush()


def mm(x):
    return round(float(x) * 1000.0, 2)


def v3(v, nd=5):
    return [round(float(x), nd) + 0.0 for x in v]


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- source (p10)
def import_source(p0b):
    sha = hashlib.sha256(SRC.read_bytes()).hexdigest()
    log(f"SOURCE {SRC} sha256 {sha} (P0b {p0b['source_sha256']}, match {sha == p0b['source_sha256']})")
    if sha != p0b["source_sha256"]:
        raise RuntimeError("source sha256 differs from P0b")
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = bpy.ops.import_scene.fbx(filepath=str(SRC))
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if res != {"FINISHED"} or len(meshes) != 1:
        raise RuntimeError(f"import {res}, meshes {[o.name for o in meshes]}")
    ob = meshes[0]
    mw = ob.matrix_world.copy()
    ob.data.transform(mw)
    ob.matrix_world = Matrix.Identity(4)
    for o in list(bpy.data.objects):
        if o != ob:
            bpy.data.objects.remove(o, do_unlink=True)
    return ob, sha


def split_parts(ob, p0b):
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="LOOSE")
    bpy.ops.object.mode_set(mode="OBJECT")
    pieces = [o for o in bpy.data.objects if o.type == "MESH"]
    parts = p0b["parts"]
    out, used = {}, set()
    for o in pieces:
        me = o.data
        c = sum((v.co for v in me.vertices), Vector()) / len(me.vertices)
        tris = sum(len(p.vertices) - 2 for p in me.polygons)
        best = min(parts, key=lambda n: (Vector(parts[n]["centroid"]) - c).length)
        d = (Vector(parts[best]["centroid"]) - c).length
        if d > 0.01 or tris != parts[best]["tris"] or best in used:
            raise RuntimeError(f"piece {o.name}: nearest part {best} at {d:.4f} m, tris {tris} vs {parts[best]['tris']}")
        used.add(best)
        o.name = best
        me.name = best
        out[best] = o
    if set(out) != set(parts) or set(out) != set(PART_NAMES):
        raise RuntimeError(f"parts mismatch {sorted(set(parts) ^ set(out))}")
    return out


def sym(lm, base):
    l, r = np.array(lm[base + "_l"]["pos"]), np.array(lm[base + "_r"]["pos"])
    return np.array(((abs(l[0]) + abs(r[0])) / 2.0, (l[1] + r[1]) / 2.0, (l[2] + r[2]) / 2.0))


def mirror(v, side):
    return np.array((v[0] if side == "l" else -v[0], v[1], v[2]))


def part_arrays(ob):
    me = ob.data
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    F = [tuple(p.vertices) for p in me.polygons]
    return V, F


# ---------------------------------------------------------------- surface (s02c Surf)
class Surf:
    def __init__(self, V, F):
        self.V = np.asarray(V, float)
        T = [(f[0], f[k], f[k + 1]) for f in F for k in range(1, len(f) - 1)]
        self.T = np.asarray(T, np.int64)
        self.bvh = BVHTree.FromPolygons(self.V.tolist(), self.T.tolist(), all_triangles=True)
        self.mn = self.V.min(0)
        self.mx = self.V.max(0)
        a, b, c = self.V[self.T[:, 0]], self.V[self.T[:, 1]], self.V[self.T[:, 2]]
        n = np.cross(b - a, c - a)
        self.TN = n / np.maximum(np.linalg.norm(n, axis=1), 1e-20)[:, None]
        self.TC = (a + b + c) / 3.0

    def nearest(self, p):
        loc, nrm, idx, d = self.bvh.find_nearest(Vector(p))
        return (None, None, None, float("inf")) if loc is None else (np.array(loc), np.array(nrm), idx, d)

    def dist(self, p):
        return self.nearest(p)[3]

    def inside(self, p, dirs=None):
        if np.any(np.asarray(p) < self.mn) or np.any(np.asarray(p) > self.mx):
            return False
        votes = 0
        for dv in (dirs or RAY_DIRS):
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

    def depth(self, p, dirs=None):
        d = self.dist(p)
        return d if self.inside(p, dirs) else -d

    def ray(self, o, d, dist=1.0):
        loc, _n, _i, dd = self.bvh.ray_cast(Vector(o), Vector(d), dist)
        return (np.array(loc), dd) if loc is not None else (None, None)


# ---------------------------------------------------------------- tubes (s02b)
def coons(ring, n):
    """(n+1) x (n+1) grid whose 4n boundary points are `ring` (corner (0,0) = ring[0], first side along j).
    Returns (interior {(i,j): p}, boundary {(i,j): m})."""
    per = [(0, j) for j in range(n)] + [(i, n) for i in range(n)] + [(n, j) for j in range(n, 0, -1)] \
        + [(i, 0) for i in range(n, 0, -1)]
    G = {ij: np.asarray(ring[m], float) for m, ij in enumerate(per)}
    bmap = {ij: m for m, ij in enumerate(per)}
    inner = {}
    for i in range(1, n):
        for j in range(1, n):
            u, v = i / n, j / n
            inner[(i, j)] = ((1 - u) * G[(0, j)] + u * G[(n, j)] + (1 - v) * G[(i, 0)] + v * G[(i, n)]
                             - ((1 - u) * (1 - v) * G[(0, 0)] + u * (1 - v) * G[(n, 0)]
                                + (1 - u) * v * G[(0, n)] + u * v * G[(n, n)]))
    return inner, bmap


def cap_inner(ring, c, adir, r_tube):
    """s02b cap_grid: 3 x 3 quads, interior domed along adir by <= CAP_DOME * r_tube."""
    inner, bmap = coons(ring, 3)
    for ij, p in inner.items():
        d = p - c
        d = d - (d @ adir) * adir
        rho = min(1.0, float(np.linalg.norm(d)) / r_tube)
        inner[ij] = p + adir * (CAP_DOME * r_tube * (1.0 - rho * rho))
    return inner, bmap


def tube_frame(o, tip, ref):
    a = unit(tip - o)
    r = np.asarray(ref, float)
    u = unit(r - (r @ a) * a)
    w = np.cross(a, u)
    E = [math.cos(math.radians(30.0 * m)) * u + math.sin(math.radians(30.0 * m)) * w for m in range(N_RING)]
    return a, u, w, E


def ray_ring(src, c, E):
    out = []
    for e in E:
        loc, _ = src.ray(c, e, RAY_MAX)
        out.append(None if loc is None else float(np.linalg.norm(loc - c)))
    return out


def build_tube(kind, side, src, shell, lm):
    """kind 'arm' / 'leg'. Returns dict(V, F, rings {name: local ids}, ring_meta [...], hidden (vertex mask), info)."""
    if kind == "arm":
        o, mid, tip = mirror(sym(lm, "shoulder"), side), mirror(sym(lm, "elbow"), side), mirror(sym(lm, "wrist"), side)
        jp, jm, je, ref = "shoulder", "elbow", "wrist", (0.0, 0.0, 1.0)
    else:
        o, mid, tip = mirror(sym(lm, "hip"), side), mirror(sym(lm, "knee"), side), mirror(sym(lm, "ankle"), side)
        jp, jm, je, ref = "hip", "knee", "ankle", (0.0, -1.0, 0.0)
    a, u, w, E = tube_frame(o, tip, ref)
    s_mid = float((mid - o) @ a)
    s_end = float((tip - o) @ a)
    rr_mid = [r for r in ray_ring(src, o + s_mid * a, E) if r is not None]
    r_mid = float(np.mean(rr_mid))
    off = r_mid * SUPPORT_OFFSET_FACTOR[jm]
    x = side
    if kind == "arm":
        head = [(f"{jp}_{x}_0", ARM_ROOT_S, "end0"), (f"{jp}_{x}_1", ARM_SH1_S, "joint")]
    else:
        head = [(f"{jp}_{x}_0", LEG_TOP_S, "end0"), (f"{jp}_{x}_1", LEG_HIP1_S, "joint")]
    named = [(f"{jm}_{x}_0", s_mid - off), (f"{jm}_{x}_1", s_mid), (f"{jm}_{x}_2", s_mid + off)]
    seq = list(head)
    gap = named[0][1] - head[-1][1]
    n = max(1, math.ceil(gap / MID_LOOP_MAX_GAP))
    seq += [(None, head[-1][1] + gap * i / n, "mid") for i in range(1, n)]
    for i, (nm, s) in enumerate(named):
        seq.append((nm, s, "joint"))
        if i < 2:
            k = JOINT_MID_LOOPS[jm] + 1
            seq += [(None, s + (named[i + 1][1] - s) * q / k, "jmid") for q in range(1, k)]
    w0 = s_end - END_PAIR_GAP
    gap = w0 - named[2][1]
    n = max(1, math.ceil(gap / MID_LOOP_MAX_GAP))
    seq += [(None, named[2][1] + gap * i / n, "mid") for i in range(1, n)]
    seq += [(f"{je}_{x}_0", w0, "joint"), (f"{je}_{x}_1", s_end, "joint")]

    radii = [ray_ring(src, o + s * a, E) for _n, s, _k in seq]
    # distal end
    info = {"o": o, "a": a, "s_mid": s_mid, "s_end": s_end, "r_mid": r_mid, "off": off}
    if kind == "arm":
        lo, hi, st = END_SEARCH
        s_e = None
        for k in range(int(round((hi - lo) / st)) + 1):
            s = s_end + lo + k * st
            c = o + s * a
            rr = ray_ring(src, c, E)
            rr = [r if r is not None else radii[-1][m] for m, r in enumerate(rr)]
            if any(r is None for r in rr):
                continue
            ring = np.array([c + r * e for r, e in zip(rr, E)])
            if min(shell.depth(p) for p in ring) < END_DEPTH_MIN:
                continue
            inner, _ = cap_inner(ring, c, a, float(np.mean(rr)))
            if min(shell.depth(p) for p in inner.values()) < EXT_DEPTH_MIN:
                continue
            s_e, rr_e = s, rr
            break
        if s_e is None:
            raise RuntimeError(f"arm_{x}: no wrist end position inside the fist")
        n = max(1, math.ceil((s_e - s_end) / EXT_STEP))
        for i in range(1, n):
            s = s_end + (s_e - s_end) * i / n
            seq.append((None, s, "ext"))
            radii.append(ray_ring(src, o + s * a, E))
        seq.append((f"{je}_{x}_end", s_e, "end1"))
        radii.append(rr_e)
    else:
        s_e = s_end + ANKLE_END_S
        seq.append((f"{je}_{x}_end", s_e, "end1"))
        radii.append(ray_ring(src, o + s_e * a, E))
    ss = [s for _n, s, _k in seq]
    if any(b <= a_ for a_, b in zip(ss, ss[1:])):
        raise RuntimeError(f"{kind}_{x}: loop order broken {[mm(s) for s in ss]}")
    # fill ray misses from the nearest ring (by s) that has a hit at that azimuth
    n_fill = 0
    for i in range(len(seq)):
        for m in range(N_RING):
            if radii[i][m] is None:
                cand = sorted((abs(ss[j] - ss[i]), j) for j in range(len(seq)) if radii[j][m] is not None)
                radii[i][m] = radii[cand[0][1]][m]
                n_fill += 1
    V, F, rings, meta = [], [], {}, []
    prev = None
    for (nm, s, kd), rr in zip(seq, radii):
        c = o + s * a
        ids = []
        for r, e in zip(rr, E):
            V.append(c + r * e)
            ids.append(len(V) - 1)
        if prev is not None:
            for m in range(N_RING):
                m1 = (m + 1) % N_RING
                F.append((prev[m], prev[m1], ids[m1], ids[m]))
        prev = ids
        meta.append({"name": nm, "s": s, "kind": kd, "ids": ids, "r_mean": float(np.mean(rr))})
        if nm:
            rings[nm] = ids
    for idx, adir in ((0, -a), (len(meta) - 1, a)):
        ring_ids = meta[idx]["ids"]
        c = o + meta[idx]["s"] * a
        inner, bmap = cap_inner(np.array([V[i] for i in ring_ids]), c, adir, meta[idx]["r_mean"])
        gid = {ij: ring_ids[m] for ij, m in bmap.items()}
        for ij, p in inner.items():
            V.append(p)
            gid[ij] = len(V) - 1
        for i in range(3):
            for j in range(3):
                F.append((gid[(i, j)], gid[(i + 1, j)], gid[(i + 1, j + 1)], gid[(i, j + 1)]))
    V = np.array(V)
    # consistent outward winding (closed tube)
    F = orient_closed(V, F)
    hidden = np.zeros(len(V), bool)
    for m_ in meta:
        if m_["kind"] in ("end0", "ext", "end1"):
            hidden[m_["ids"]] = True
    ring_ids_all = set(i for m_ in meta for i in m_["ids"])
    for i in range(len(V)):
        if i not in ring_ids_all:
            hidden[i] = True    # cap interior
    info.update({"n_fill": n_fill, "seq": [(nm, s, kd) for nm, s, kd in seq]})
    return {"V": V, "F": F, "rings": rings, "meta": meta, "hidden": hidden, "info": info}


def orient_closed(V, F):
    me = bpy.data.meshes.new("_p20_orient")
    me.from_pydata(np.asarray(V).tolist(), [], [list(f) for f in F])
    me.validate(clean_customdata=False)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    vol = bm.calc_volume(signed=True)
    bm.free()
    out = [tuple(p.vertices) for p in me.polygons]
    bpy.data.meshes.remove(me)
    if len(out) != len(F):
        raise RuntimeError("orient_closed changed the face count")
    if vol < 0:
        out = [tuple(reversed(f)) for f in out]
    return out


# ---------------------------------------------------------------- straddle (s02c finish_head)
def poly_normals(V, F):
    N = np.zeros((len(F), 3))
    for t, f in enumerate(F):
        p = V[list(f)]
        n = np.zeros(3)
        for k in range(1, len(f) - 1):
            n += np.cross(p[k] - p[0], p[k + 1] - p[0])
        N[t] = n
    return N


def straddle(V, F, movable, src, hid_T, passes=STRADDLE_PASSES):
    V = V.copy()
    FN = poly_normals(V, F)
    VN = np.zeros_like(V)
    for t, f in enumerate(F):
        for v in f:
            VN[v] += FN[t]
    VN /= np.maximum(np.linalg.norm(VN, axis=1), 1e-20)[:, None]
    tot = np.zeros(len(V))
    mv_any = np.zeros(len(V), bool)
    for _ in range(passes):
        acc, cnt = np.zeros(len(V)), np.zeros(len(V))
        for f in F:
            q = V[list(f)].mean(0)
            loc, nrm, fi, _d = src.nearest(q)
            if loc is None or hid_T[fi]:
                continue
            sd = float((q - loc) @ nrm)
            for v in f:
                acc[v] += sd
                cnt[v] += 1
        mv = movable & (cnt > 0)
        mv_any |= mv
        off = np.where(mv, -0.5 * acc / np.maximum(cnt, 1), 0.0)
        V = V + off[:, None] * VN
        tot += off
    return V, {"moved": int(mv_any.sum()), "mean_mm": mm(tot[mv_any].mean()) if mv_any.any() else 0.0,
               "min_mm": mm(tot[mv_any].min()) if mv_any.any() else 0.0,
               "max_mm": mm(tot[mv_any].max()) if mv_any.any() else 0.0}


# ---------------------------------------------------------------- tunic
def build_tunic(src, head, p0b):
    cx, cy = p0b["skirt"]["hem_ellipse"]["center_xy"]
    az = [360.0 * k / N_COL for k in range(N_COL)]
    D = [np.array((math.sin(math.radians(t)), -math.cos(math.radians(t)), 0.0)) for t in az]

    def row(z):
        pts = []
        for d in D:
            o = np.array((cx, cy, z)) + d * 0.6
            loc, _ = src.ray(o, -d, 0.6)
            if loc is None:
                raise RuntimeError(f"tunic ray miss z {z} dir {v3(d)}")
            pts.append(loc)
        return np.array(pts)

    # torso rows between the belt top and Z_TORSO_TOP evenly by mean arc length
    zf = np.arange(Z_BELT_TOP, Z_TORSO_TOP + 1e-9, 0.002)
    prof = np.array([np.hypot(row(z)[:, 0] - cx, row(z)[:, 1] - cy).mean() for z in zf])
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(zf), np.diff(prof)))])
    n_t = max(1, math.ceil(arc[-1] / TORSO_ARC_MAX))
    z_torso = [float(np.interp(arc[-1] * i / n_t, arc, zf)) for i in range(1, n_t + 1)]
    z_rows = [Z_HEM] + Z_SKIRT + [Z_BELT_BOT, Z_BELT_MID, Z_BELT_TOP] + z_torso
    rows = [row(z) for z in z_rows]
    # neck rim: rays from (cx, cy, NECK_ORIGIN_Z) in each column's meridian plane, raised until NECK_HIDE inside head
    o_n = np.array((cx, cy, NECK_ORIGIN_Z))
    rim = []
    top = rows[-1]
    for k, d in enumerate(D):
        v0 = top[k] - o_n
        phi0 = math.atan2(v0[2], float(v0[:2] @ d[:2]))
        hit = None
        for step in range(0, 400):
            phi = phi0 + math.radians(0.25 * step)
            e = math.cos(phi) * d + math.sin(phi) * np.array((0.0, 0.0, 1.0))
            loc, _ = src.ray(o_n, e, 0.5)
            if loc is None:
                continue
            if head.depth(loc) >= NECK_HIDE:
                hit = loc
                break
        if hit is None:
            raise RuntimeError(f"neck rim: no hidden point for column {k}")
        rim.append(hit)
    rows.append(np.array(rim))
    z_rows.append(float(np.mean([p[2] for p in rim])))
    return {"cx": cx, "cy": cy, "az": az, "D": D, "z_rows": z_rows, "rows": rows}


def assemble_tunic(T):
    """Outer wall rows (straddled), hem roll, inner wall, ceiling -> V, F, index maps."""
    cx, cy = T["cx"], T["cy"]
    rows = T["rows"]
    nr = len(rows)
    V, F = [], []
    O = []
    for r in rows:
        ids = []
        for p in r:
            V.append(np.asarray(p, float))
            ids.append(len(V) - 1)
        O.append(ids)
    for i in range(nr - 1):
        for k in range(N_COL):
            k1 = (k + 1) % N_COL
            F.append((O[i][k], O[i][k1], O[i + 1][k1], O[i + 1][k]))
    return V, F, O


def add_inner(V, F, O, T, z_rows):
    cx, cy = T["cx"], T["cy"]
    V = [np.asarray(p, float) for p in V]

    def inward(p):
        h = np.array((p[0] - cx, p[1] - cy, 0.0))
        return p - SKIRT_T * h / np.linalg.norm(h)

    n_in = 1 + len(Z_SKIRT)               # hem + skirt rings below the ceiling
    if not all(z < CEILING_Z for z in z_rows[:n_in]) or z_rows[n_in] <= CEILING_Z:
        raise RuntimeError("ceiling must lie between the last skirt ring and the belt bottom row")
    I = []
    for i in range(n_in):
        ids = []
        for k in range(N_COL):
            V.append(inward(V[O[i][k]]))
            ids.append(len(V) - 1)
        I.append(ids)
    # ceiling ring: outer wall interpolated at CEILING_Z (between rows n_in-1 and n_in), moved inward
    za, zb = z_rows[n_in - 1], z_rows[n_in]
    ring = []
    for k in range(N_COL):
        pa, pb = V[O[n_in - 1][k]], V[O[n_in][k]]
        t = (CEILING_Z - pa[2]) / (pb[2] - pa[2])
        p = pa + t * (pb - pa)
        p[2] = CEILING_Z
        V.append(inward(p))
        ring.append(len(V) - 1)
    I.append(ring)
    # hem roll (normal down): outer row 0 -> inner row 0
    for k in range(N_COL):
        k1 = (k + 1) % N_COL
        F.append((O[0][k], I[0][k], I[0][k1], O[0][k1]))
    # inner wall (normal toward the axis)
    for i in range(len(I) - 1):
        for k in range(N_COL):
            k1 = (k + 1) % N_COL
            F.append((I[i][k], I[i + 1][k], I[i + 1][k1], I[i][k1]))
    # ceiling grid, corners at the diagonal azimuths (k = N_COL/8 + j * N_COL/4)
    k0 = N_COL // 8
    bring = [ring[(k0 + m) % N_COL] for m in range(N_COL)]
    inner, bmap = coons(np.array([V[i] for i in bring]), CEIL_N)
    gid = {ij: bring[m] for ij, m in bmap.items()}
    ceil_ids = []
    for ij, p in inner.items():
        q = np.array(p)
        q[2] = CEILING_Z
        V.append(q)
        gid[ij] = len(V) - 1
        ceil_ids.append(len(V) - 1)
    Vn = np.array(V)
    ceil_faces = []
    for i in range(CEIL_N):
        for j in range(CEIL_N):
            f = (gid[(i, j)], gid[(i + 1, j)], gid[(i + 1, j + 1)], gid[(i, j + 1)])
            n = poly_normals(Vn, [f])[0]
            if n[2] > 0:                      # ceiling faces the skirt cavity (down)
                f = tuple(reversed(f))
            ceil_faces.append(f)
    F += ceil_faces
    return Vn, F, I, ceil_ids


# ---------------------------------------------------------------- merge / normals / UV (s02d)
def build_pl_mesh(geo, colors):
    verts, faces, pid = [], [], []
    offsets = {}
    nv = 0
    for i, name in enumerate(PART_NAMES):
        V, F = geo[name]
        offsets[name] = (nv, len(V))
        verts.append(np.asarray(V, float))
        faces += [tuple(int(x) + nv for x in f) for f in F]
        pid += [i] * len(F)
        nv += len(V)
    V = np.vstack(verts)
    me = bpy.data.meshes.new(MESH)
    me.from_pydata(V.tolist(), [], faces)
    me.update()
    if len(me.polygons) != len(faces) or len(me.vertices) != len(V):
        raise RuntimeError("from_pydata changed the element counts")
    att = me.attributes.new("part_id", "INT", "FACE")
    att.data.foreach_set("value", np.array(pid, dtype=np.int32))
    for name in PART_NAMES:
        mat = bpy.data.materials.new(f"M_{name}")
        c = colors[name]
        mat.diffuse_color = (c[0], c[1], c[2], 1.0)
        me.materials.append(mat)
    me.polygons.foreach_set("material_index", np.array(pid, dtype=np.int32))
    me.update()
    ob = bpy.data.objects.new(MESH, me)
    coll = bpy.data.collections.new(COLL)
    bpy.context.scene.collection.children.link(coll)
    coll.objects.link(ob)
    return ob, offsets


def set_normals(ob, designed_pairs):
    """Smooth; sharp edges = rigid-part edges above SHARP_ANGLE_RIGID (their modelled creases) + designed pairs on
    the rebuilt parts; default smooth corner normals with those sharp edges stored as custom normals."""
    me = ob.data
    me.shade_smooth()
    me.set_sharp_from_angle(angle=SHARP_ANGLE_RIGID)
    ne = len(me.edges)
    sh = np.zeros(ne, dtype=bool)
    if me.attributes.get("sharp_edge") is None:
        me.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
    me.attributes["sharp_edge"].data.foreach_get("value", sh)
    pv = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    le = np.empty(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("edge_index", le)
    epart = np.full(ne, -1)
    epart[le] = np.repeat(pv, lt)
    rebuilt_ids = [PART_NAMES.index(n) for n in REBUILT]
    reb = np.isin(epart, rebuilt_ids)
    sh[reb] = False
    E = np.empty(ne * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    key = {(min(a, b), max(a, b)): i for i, (a, b) in enumerate(E.reshape(-1, 2).tolist())}
    n_des = 0
    for a, b in designed_pairs:
        i = key.get((min(a, b), max(a, b)))
        if i is None:
            raise RuntimeError(f"designed hard edge {a}-{b} not in mesh")
        sh[i] = True
        n_des += 1
    me.attributes["sharp_edge"].data.foreach_set("value", sh)
    me.update()
    cn = np.empty(len(me.loops) * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", cn)
    me.normals_split_custom_set(cn.reshape(-1, 3).tolist())
    per_part = {n: int((sh & (epart == i)).sum()) for i, n in enumerate(PART_NAMES)}
    return {"sharp_total": int(sh.sum()), "designed_on_rebuilt": n_des, "sharp_per_part": per_part}


def uv_islands_bm(bm, uvl):
    parent = list(range(len(bm.faces)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for e in bm.edges:
        lf = e.link_loops
        if len(lf) != 2:
            continue
        l1, l2 = lf
        a1, b1 = l1[uvl].uv, l1.link_loop_next[uvl].uv
        a2, b2 = l2.link_loop_next[uvl].uv, l2[uvl].uv
        if (a1 - a2).length <= UV_MATCH and (b1 - b2).length <= UV_MATCH:
            ra, rb = find(l1.face.index), find(l2.face.index)
            if ra != rb:
                parent[ra] = rb
    return [find(i) for i in range(len(bm.faces))]


def make_uvs(ob, head_value):
    me = ob.data
    if not me.uv_layers:
        me.uv_layers.new(name="UVMap")
    bpy.context.view_layer.objects.active = ob
    for o in bpy.context.view_layer.objects:
        o.select_set(o is ob)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=UV_ANGLE_LIMIT, island_margin=0.0, area_weight=0.0,
                             correct_aspect=True, scale_to_bounds=False)
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.seams_from_islands(mark_seams=True, mark_sharp=False)
    bpy.ops.uv.unwrap(method="ANGLE_BASED", margin=PACK_MARGIN)
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.average_islands_scale()
    bpy.ops.object.mode_set(mode="OBJECT")
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    uvl = bm.loops.layers.uv.active
    isl = uv_islands_bm(bm, uvl)
    pv = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    front = set()
    for f in bm.faces:
        if pv[f.index] == head_value and f.normal.dot((0.0, -1.0, 0.0)) > HEAD_FRONT_DOT:
            front.add(isl[f.index])
    for island in front:
        fs = [f for f in bm.faces if isl[f.index] == island]
        pts = [l[uvl].uv.copy() for f in fs for l in f.loops]
        cx = sum(p.x for p in pts) / len(pts)
        cy = sum(p.y for p in pts) / len(pts)
        for f in fs:
            for l in f.loops:
                uv = l[uvl].uv
                l[uvl].uv = (cx + (uv.x - cx) * HEAD_FRONT_SCALE, cy + (uv.y - cy) * HEAD_FRONT_SCALE)
    bm.to_mesh(me)
    bm.free()
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.pack_islands(margin=PACK_MARGIN, rotate=True, scale=True)
    bpy.ops.object.mode_set(mode="OBJECT")
    return len(front)


def uv_report(ob, head_value, body_values):
    me = ob.data
    nl, npoly = len(me.loops), len(me.polygons)
    uv = np.empty(nl * 2, dtype=np.float32)
    me.uv_layers.active.uv.foreach_get("vector", uv)
    uv = uv.reshape(-1, 2).astype(np.float64)
    bm = bmesh.new()
    bm.from_mesh(me)
    uvl = bm.loops.layers.uv.active
    isl = np.array(uv_islands_bm(bm, uvl))
    bm.free()
    _, isl = np.unique(isl, return_inverse=True)
    me.calc_loop_triangles()
    nt = len(me.loop_triangles)
    tl = np.empty(nt * 3, dtype=np.int32)
    tp = np.empty(nt, dtype=np.int32)
    tv = np.empty(nt * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("loops", tl)
    me.loop_triangles.foreach_get("polygon_index", tp)
    me.loop_triangles.foreach_get("vertices", tv)
    tl, tv = tl.reshape(-1, 3), tv.reshape(-1, 3)
    owner = np.full((UV_RES, UV_RES), -1, dtype=np.int32)
    conflict = np.zeros((UV_RES, UV_RES), dtype=bool)
    T = uv[tl] * UV_RES
    for t in range(nt):
        p = T[t]
        a2 = (p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[1, 1] - p[0, 1]) * (p[2, 0] - p[0, 0])
        if abs(a2) < 1e-12:
            continue
        x0 = max(int(math.floor(p[:, 0].min() - 0.5)), 0)
        x1 = min(int(math.ceil(p[:, 0].max() - 0.5)), UV_RES - 1)
        y0 = max(int(math.floor(p[:, 1].min() - 0.5)), 0)
        y1 = min(int(math.ceil(p[:, 1].max() - 0.5)), UV_RES - 1)
        if x1 < x0 or y1 < y0:
            continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        s = 1.0 if a2 > 0 else -1.0
        inside = np.ones(X.shape, bool)
        for i in range(3):
            q0, q1 = p[i], p[(i + 1) % 3]
            inside &= s * ((q1[0] - q0[0]) * (Y - q0[1]) - (q1[1] - q0[1]) * (X - q0[0])) >= 0
        il = int(isl[tp[t]])
        sub = owner[y0:y1 + 1, x0:x1 + 1]
        csub = conflict[y0:y1 + 1, x0:x1 + 1]
        csub[inside & (sub >= 0) & (sub != il)] = True
        sub[inside & (sub < 0)] = il
    rep = {"uv_min": uv.min(0).round(5).tolist(), "uv_max": uv.max(0).round(5).tolist(),
           "loops_out_of_0_1": int((np.any(uv < -1e-6, axis=1) | np.any(uv > 1 + 1e-6, axis=1)).sum()),
           "islands": int(isl.max()) + 1, "overlap_pixels": int(conflict.sum()),
           "covered_pixels": int((owner >= 0).sum())}
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    P3 = V[tv]
    cr = np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0])
    a3 = 0.5 * np.linalg.norm(cr, axis=1)
    pn = np.zeros((npoly, 3))
    np.add.at(pn, tp, cr)
    pn /= np.maximum(np.linalg.norm(pn, axis=1), 1e-20)[:, None]
    U = uv[tl]
    auv = 0.5 * np.abs((U[:, 1, 0] - U[:, 0, 0]) * (U[:, 2, 1] - U[:, 0, 1])
                       - (U[:, 1, 1] - U[:, 0, 1]) * (U[:, 2, 0] - U[:, 0, 0]))
    pv = np.empty(npoly, dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    masks = {"head_front": (pv == head_value) & (pn @ np.array([0.0, -1.0, 0.0]) > HEAD_FRONT_DOT),
             "body": np.isin(pv, body_values), "tunic": pv == PART_NAMES.index("tunic")}
    dens = {}
    for key, m in masks.items():
        mt = m[tp]
        dens[key] = float(auv[mt].sum() / a3[mt].sum())
    rep["density"] = dens
    rep["density_ratio_head_front_over_body"] = dens["head_front"] / dens["body"]
    rep["density_ratio_head_front_over_tunic"] = dens["head_front"] / dens["tunic"]
    rep["head_front_faces"] = int(masks["head_front"].sum())
    rep["faces_without_uv_area"] = int((np.bincount(tp, weights=auv, minlength=npoly) <= 0).sum())
    rep["_owner"] = owner
    return rep


def write_uv_png(owner, path):
    o = owner[::2, ::2]
    rgb = np.zeros(o.shape + (3,), dtype=np.float32)
    m = o >= 0
    h = (o[m].astype(np.int64) * 2654435761) & 0xFFFFFF
    rgb[m, 0] = 0.25 + 0.75 * ((h & 0xFF) / 255.0)
    rgb[m, 1] = 0.25 + 0.75 * (((h >> 8) & 0xFF) / 255.0)
    rgb[m, 2] = 0.25 + 0.75 * (((h >> 16) & 0xFF) / 255.0)
    hh, ww = rgb.shape[:2]
    rgba = np.ones((hh, ww, 4), dtype=np.float32)
    rgba[:, :, :3] = rgb
    im = bpy.data.images.new("_p20_uv", width=ww, height=hh, alpha=True)
    im.pixels.foreach_set(rgba.ravel())
    im.filepath_raw = str(path)
    im.file_format = "PNG"
    im.save()
    bpy.data.images.remove(im)


# ---------------------------------------------------------------- render (goblib.ortho_render)
def view_basis(view):
    if view == "front":
        d, wup = Vector((0, 1, 0)), Vector((0, 0, 1))
    elif view == "back":
        d, wup = Vector((0, -1, 0)), Vector((0, 0, 1))
    elif view == "side":
        d, wup = Vector((-1, 0, 0)), Vector((0, 0, 1))
    elif view == "three_quarter":
        az, el = math.radians(45.0), math.radians(15.0)
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    elif view == "below":
        d, wup = Vector((0, 0, 1)), Vector((0, -1, 0))
    elif view == "below_front":
        el = math.radians(-35.0)
        pos = Vector((0.0, -math.cos(el), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    else:
        raise ValueError(view)
    d = d.normalized()
    right = d.cross(wup).normalized()
    up = right.cross(d).normalized()
    return d, right, up


def ortho_render(objs, view, out_png, res_h=1024, frame_bbox=None, wire=False):
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    d, right, up = view_basis(view)
    if frame_bbox is None:
        dg = bpy.context.evaluated_depsgraph_get()
        pts = []
        for o in objs:
            oe = o.evaluated_get(dg)
            pts.extend(oe.matrix_world @ Vector(c) for c in oe.bound_box)
        mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
        mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
        pad = True
    else:
        mn, mx = Vector(frame_bbox[0]), Vector(frame_bbox[1])
        pad = False
    corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
    us = [c.dot(right) for c in corners]
    vs = [c.dot(up) for c in corners]
    ws = [c.dot(d) for c in corners]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    if pad:
        m = 0.05 * max(u1 - u0, v1 - v0)
        u0, u1, v0, v1 = u0 - m, u1 + m, v0 - m, v1 + m
    w = max(u1 - u0, 1e-6)
    h = max(v1 - v0, 1e-6)
    depth = max(ws) - min(ws)
    res_w = max(1, int(round(res_h * w / h)))
    center_plane = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
    dist = depth + max(w, h) + 1.0
    cam_loc = center_plane + d * (min(ws) - dist)
    sc = bpy.data.scenes.new("_p20_render")
    cam_data = cam_obj = wire_mat = None
    temp_objs, temp_meshes = [], []
    try:
        for o in objs:
            sc.collection.objects.link(o)
        cam_data = bpy.data.cameras.new("_p20_cam")
        cam_data.type = "ORTHO"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.ortho_scale = h
        cam_data.clip_start = 0.001
        cam_data.clip_end = dist + depth + 10.0
        cam_obj = bpy.data.objects.new("_p20_cam", cam_data)
        rot = Matrix((right, up, -d)).transposed()
        cam_obj.matrix_world = Matrix.Translation(cam_loc) @ rot.to_4x4()
        sc.collection.objects.link(cam_obj)
        sc.camera = cam_obj
        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = res_w
        r.resolution_y = res_h
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_xray = False
        if wire:
            dg = bpy.context.evaluated_depsgraph_get()
            px = h / res_h
            wire_mat = bpy.data.materials.new("_p20_wire")
            wire_mat.diffuse_color = (0.0, 0.0, 0.0, 1.0)
            for o in objs:
                oe = o.evaluated_get(dg)
                me = bpy.data.meshes.new_from_object(oe, preserve_all_data_layers=False, depsgraph=dg)
                me.materials.clear()
                me.materials.append(wire_mat)
                temp_meshes.append(me)
                wo = bpy.data.objects.new("_p20_wire_" + o.name, me)
                wo.matrix_world = oe.matrix_world.copy()
                sc.collection.objects.link(wo)
                temp_objs.append(wo)
                mod = wo.modifiers.new("wire", "WIREFRAME")
                mod.thickness = 1.5 * px
                mod.offset = 0.0
                mod.use_boundary = True
                mod.use_replace = True
                mod.use_even_offset = True
        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
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


def subset_object(ob, names):
    """Temporary copy of PL_mesh holding only the faces of `names` (caller removes it)."""
    me = ob.data.copy()
    bm = bmesh.new()
    bm.from_mesh(me)
    lay = bm.faces.layers.int.get("part_id")
    keep = {PART_NAMES.index(n) for n in names}
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if f[lay] not in keep], context="FACES")
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
    bm.to_mesh(me)
    bm.free()
    so = bpy.data.objects.new("_p20_sub", me)
    return so


# ---------------------------------------------------------------- deviation report (s02c sample_surface / occluded)
def tris_of(V, F):
    T = [(f[0], f[k], f[k + 1]) for f in F for k in range(1, len(f) - 1)]
    fi = [i for i, f in enumerate(F) for _k in range(1, len(f) - 1)]
    return np.array(T, np.int64), np.array(fi, np.int64)


def sample_surface(V, T, n, rng):
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    ti = rng.choice(len(T), size=n, p=area / area.sum())
    r1 = np.sqrt(rng.random(n))
    r2 = rng.random(n)
    pts = (1 - r1)[:, None] * a[ti] + (r1 * (1 - r2))[:, None] * b[ti] + (r1 * r2)[:, None] * c[ti]
    return pts, ti


def occluded(pts, own, closed_shells):
    hid = np.zeros(len(pts), bool)
    for name, S in closed_shells.items():
        if name == own:
            continue
        inb = np.all((pts >= S.mn - 1e-4) & (pts <= S.mx + 1e-4), axis=1)
        for i in np.nonzero(inb & ~hid)[0].tolist():
            if S.dist(pts[i]) > HIDE_DEPTH and S.inside(pts[i]):
                hid[i] = True
    return hid


def dstats(d):
    if not len(d):
        return {"n": 0}
    return {"n": int(len(d)), "mean_mm": mm(d.mean()), "p95_mm": mm(np.percentile(d, 95)), "max_mm": mm(d.max())}


# ---------------------------------------------------------------- main
def main():
    p0b = json.loads(P0B.read_text(encoding="utf-8"))
    lm = p0b["landmarks"]
    src_ob, sha = import_source(p0b)
    parts = split_parts(src_ob, p0b)
    src = {n: part_arrays(parts[n]) for n in PART_NAMES}
    S = {n: Surf(*src[n]) for n in PART_NAMES}
    log("PARTS " + ", ".join(f"{n} {len(src[n][1])} tris" for n in PART_NAMES))

    geo, rings, tube_meta, hidden_local, straddle_rep = {}, {}, {}, {}, {}
    for n in RIGID:
        geo[n] = src[n]
    # ---- T222: shoe top rim tuck (rim verts with cuff depth <= 0 -> RIM_TUCK_DEPTH inside the cuff)
    for x in ("l", "r"):
        n, cuff = f"shoe_{x}", S[f"cuff_{x}"]
        Vn, Fn = src[n]
        Vn = Vn.copy()
        ec = {}
        for f in Fn:
            for a, b in zip(f, f[1:] + f[:1]):
                k = (min(a, b), max(a, b))
                ec[k] = ec.get(k, 0) + 1
        rim = sorted({v for k, c in ec.items() if c == 1 for v in k})
        moved = []
        for i in rim:
            d0 = cuff.depth(Vn[i])
            if d0 > 0.0:
                continue
            _loc, nrm, _fi, _d = cuff.nearest(Vn[i])
            p0 = Vn[i].copy()
            t = 0.0
            while cuff.depth(p0 - nrm * t) < RIM_TUCK_DEPTH:
                t += RIM_TUCK_STEP
                if t > 0.01:
                    raise RuntimeError(f"{n} rim vertex {i}: no tuck within 10 mm")
            Vn[i] = p0 - nrm * t
            moved.append((i, d0, cuff.depth(Vn[i]), t, nrm))
        geo[n] = (Vn, Fn)
        log(f"RIM TUCK {n}: rim verts {len(rim)}, moved {len(moved)}: " + "; ".join(
            f"v{i} depth {mm(a)} -> {mm(b)} mm, displacement {mm(t)} mm along {v3(-nr, 3)}" for i, a, b, t, nr in moved))
    # ---- tubes
    for kind in ("arm", "leg"):
        for side in ("l", "r"):
            name = f"{kind}_{side}"
            shell = S[f"fist_{side}"] if kind == "arm" else S[f"cuff_{side}"]
            tb = build_tube(kind, side, S[name], shell, lm)
            movable = ~tb["hidden"]
            hid_T = np.zeros(len(S[name].T), bool)
            V2, rep = straddle(tb["V"], tb["F"], movable, S[name], hid_T)
            geo[name] = (V2, tb["F"])
            straddle_rep[name] = rep
            rings[name] = tb["rings"]
            tube_meta[name] = tb
            hidden_local[name] = tb["hidden"]
            log(f"TUBE {name}: {len(V2)} verts, {len(tb['F'])} quads, loops {len(tb['meta'])} "
                f"(ray misses filled {tb['info']['n_fill']}), r_mid {mm(tb['info']['r_mid'])} mm, support offset "
                f"{mm(tb['info']['off'])} mm, straddle {rep}")
            log(f"   s (mm from the proximal pivot): " + ", ".join(
                f"{m_['name'] or m_['kind']}@{mm(m_['s'])}(r{mm(m_['r_mean'])})" for m_ in tb["meta"]))
    # ---- tunic
    T = build_tunic(S["tunic"], S["head"], p0b)
    V, F, O = assemble_tunic(T)
    V = np.array(V)
    nr = len(O)
    movable = np.zeros(len(V), bool)
    for i in range(nr - 1):          # neck rim (hidden in the head) stays on the source
        movable[O[i]] = True
    st = S["tunic"]
    hid_T = ((st.TN[:, 2] < CAP_NZ) & (st.TC[:, 2] < CAP_ZMAX)) | (st.TC[:, 2] > TOP_ZMIN)
    V, rep = straddle(V, F, movable, st, hid_T)
    straddle_rep["tunic"] = rep
    Vt, Ft, I, ceil_ids = add_inner(V, F, O, T, T["z_rows"])
    geo["tunic"] = (Vt, Ft)
    log(f"TUNIC rows z {[round(z, 4) for z in T['z_rows']]} (last = neck rim mean z), columns {N_COL}, "
        f"inner rows {len(I)} (last = ceiling ring z {CEILING_Z}), ceiling interior verts {len(ceil_ids)}, "
        f"verts {len(Vt)}, faces {len(Ft)}, straddle {rep}")
    rim = Vt[O[-1]]
    dep = np.array([S["head"].depth(p) for p in rim])
    log(f"NECK rim z {mm(rim[:, 2].min())}..{mm(rim[:, 2].max())} mm, r "
        f"{mm(np.hypot(rim[:, 0] - T['cx'], rim[:, 1] - T['cy']).min())}.."
        f"{mm(np.hypot(rim[:, 0] - T['cx'], rim[:, 1] - T['cy']).max())} mm, depth inside head min/mean "
        f"{mm(dep.min())}/{mm(dep.mean())} mm")

    # ---- merge
    ob, offsets = build_pl_mesh(geo, p0b["colors"])
    me = ob.data

    def g(name, ids):
        return [int(i) + offsets[name][0] for i in ids]

    # designed hard edges on rebuilt parts: hem outer ring, hem inner ring, ceiling ring
    designed = []
    for ring in (O[0], I[0], I[-1]):
        gr = g("tunic", ring)
        designed += [(gr[k], gr[(k + 1) % N_COL]) for k in range(N_COL)]
    nrep = set_normals(ob, designed)
    n_front = make_uvs(ob, PART_NAMES.index("head"))

    # ---- loops json
    loops = {"mesh": MESH, "note": "vertex indices of PL_mesh; every ring is a closed edge loop in the listed order",
             "rings": {}, "tunic": {}}
    for name in ("arm_l", "arm_r", "leg_l", "leg_r"):
        for m_ in tube_meta[name]["meta"]:
            key = m_["name"] or f"{name}_{m_['kind']}_{mm(m_['s'])}"
            jn, k = (m_["name"].rsplit("_", 1) if m_["name"] else (None, None))
            loops["rings"][key] = {"verts": g(name, m_["ids"]), "part": name, "kind": m_["kind"],
                                   "joint": jn, "k": (int(k) if k and k.isdigit() else k),
                                   "center": bool(m_["name"] and m_["name"].split("_")[0] in ("elbow", "knee")
                                                  and m_["name"].endswith("_1")),
                                   "s_from_pivot_m": round(m_["s"], 5)}
    z_rows = T["z_rows"]
    row_names = (["hem_outer"] + [f"skirt_{i + 1}" for i in range(len(Z_SKIRT))]
                 + ["belt_bot", "belt_mid", "belt_top"]
                 + [f"torso_{i + 1}" for i in range(nr - 1 - 7)] + ["neck_rim"])
    loops["tunic"]["outer_rows"] = [{"name": row_names[i], "z": round(z_rows[i], 5), "verts": g("tunic", O[i])}
                                    for i in range(nr)]
    loops["tunic"]["inner_rows"] = [{"name": ("hem_inner" if i == 0 else ("ceiling_ring" if i == len(I) - 1
                                                                        else f"skirt_{i}_inner")),
                                     "z": round(float(Vt[I[i][0]][2]), 5), "verts": g("tunic", I[i]),
                                     "paired_outer_row": (i if i < len(I) - 1 else None)}
                                    for i in range(len(I))]
    loops["tunic"]["ceiling_interior"] = g("tunic", ceil_ids)
    loops["tunic"]["skirt_rings"] = row_names[:5]
    loops["tunic"]["columns"] = {}
    for tag, az in SKIRT_AZ:
        k = int(round(az / (360.0 / N_COL))) % N_COL
        loops["tunic"]["columns"][tag] = {"azimuth_deg": az, "column": k,
                                          "outer_hem_to_belt_bot": [g("tunic", [O[i][k]])[0] for i in range(5)],
                                          "inner_hem_to_ceiling": [g("tunic", [I[i][k]])[0] for i in range(len(I))]}
    loops["tunic"]["n_columns"] = N_COL
    loops["tunic"]["axis_xy"] = [T["cx"], T["cy"]]
    loops["tunic"]["thickness_m"] = SKIRT_T
    loops["tunic"]["ceiling_z"] = CEILING_Z

    # ---- ring closure check
    E = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    nv = len(me.vertices)
    ek = set((np.minimum(E[:, 0], E[:, 1]) * nv + np.maximum(E[:, 0], E[:, 1])).tolist())

    def closed(vs):
        a = np.array(vs)
        b = np.roll(a, -1)
        return all(k in ek for k in (np.minimum(a, b) * nv + np.maximum(a, b)).tolist())
    open_rings = [k for k, r in loops["rings"].items() if not closed(r["verts"])]
    open_rows = [r["name"] for r in loops["tunic"]["outer_rows"] + loops["tunic"]["inner_rows"] if not closed(r["verts"])]

    parts_json = {"mesh": MESH, "attribute": "part_id (FACE, INT)", "part_id": {n: i for i, n in enumerate(PART_NAMES)},
                  "slots": SLOTS, "rigid": RIGID, "rebuilt": REBUILT,
                  "materials": {f"M_{n}": {"material_index": i, "color": p0b["colors"][n]}
                                for i, n in enumerate(PART_NAMES)}}
    PARTS_JSON.parent.mkdir(parents=True, exist_ok=True)
    PARTS_JSON.write_text(json.dumps(parts_json, indent=1) + "\n", encoding="utf-8")
    LOOPS_JSON.write_text(json.dumps(loops, indent=1) + "\n", encoding="utf-8")

    # ---- save
    for o in list(bpy.data.objects):
        if o is not ob:
            bpy.data.objects.remove(o, do_unlink=True)
    for c in list(bpy.data.collections):
        if c.name != COLL:
            bpy.data.collections.remove(c)
    bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
    bpy.context.preferences.filepaths.save_version = 0
    OUT_BLEND.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND))
    log(f"OUTPUT {OUT_BLEND}; {PARTS_JSON}; {LOOPS_JSON}; objects {[(o.name, o.type) for o in bpy.data.objects]}")

    # ---- report
    me.calc_loop_triangles()
    tp = np.empty(len(me.loop_triangles), dtype=np.int32)
    me.loop_triangles.foreach_get("polygon_index", tp)
    pv = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    bm = bmesh.new()
    bm.from_mesh(me)
    lay = bm.faces.layers.int.get("part_id")
    rows = []
    for i, n in enumerate(PART_NAMES):
        fs = [f for f in bm.faces if f[lay] == i]
        es = {e for f in fs for e in f.edges}
        nb = sum(1 for e in es if len(e.link_faces) == 1)
        nm_ = sum(1 for e in es if len(e.link_faces) > 2)
        quads = sum(1 for f in fs if len(f.verts) == 4)
        rows.append((n, len(fs), int((pv[tp] == i).sum()), quads, nb, nm_))
    bm.free()
    log(f"TRIS total {len(me.loop_triangles)} (budget {TRI_BUDGET}), verts {nv}, faces {len(me.polygons)}")
    for n, nf, nt, nq, nb, nm_ in rows:
        log(f"   PART {n:10s} id {PART_NAMES.index(n):2d} faces {nf:4d} tris {nt:4d} quads {nq:4d} boundary "
            f"{nb:3d} nonmanifold {nm_} {'rebuilt' if n in REBUILT else 'rigid (source)'}")
    log(f"NORMALS custom {me.has_custom_normals} {nrep}")
    body_vals = [PART_NAMES.index(n) for n in SLOTS["Armor_Default"]]
    ur = uv_report(ob, PART_NAMES.index("head"), body_vals)
    log(f"UV range {ur['uv_min']}..{ur['uv_max']} out_of_0_1 {ur['loops_out_of_0_1']} islands {ur['islands']} "
        f"overlap_px({UV_RES}^2) {ur['overlap_pixels']} covered {ur['covered_pixels']} faces without uv area "
        f"{ur['faces_without_uv_area']} head-front islands scaled {n_front} (x{HEAD_FRONT_SCALE}) head_front faces "
        f"{ur['head_front_faces']} density {ur['density']} ratio head_front/Armor_Default "
        f"{round(ur['density_ratio_head_front_over_body'], 3)} head_front/tunic "
        f"{round(ur['density_ratio_head_front_over_tunic'], 3)}")
    log(f"LOOPS rings {len(loops['rings'])} open {open_rings}; tunic rows {len(loops['tunic']['outer_rows'])} outer + "
        f"{len(loops['tunic']['inner_rows'])} inner, open {open_rows}")
    for name in ("arm_l", "arm_r", "leg_l", "leg_r"):
        log(f"   {name}: " + ", ".join(k for k, r in loops["rings"].items() if r["part"] == name and r["joint"]))
    log("SKIRT rings z (outer): " + ", ".join(f"{r['name']} {r['z']}" for r in loops["tunic"]["outer_rows"][:5])
        + "; inner: " + ", ".join(f"{r['name']} {r['z']}" for r in loops["tunic"]["inner_rows"]))

    # ---- rigid parts unchanged
    Xall = np.empty(nv * 3)
    me.vertices.foreach_get("co", Xall)
    Xall = Xall.reshape(-1, 3)
    rig_dev = max(float(np.abs(Xall[offsets[n][0]:offsets[n][0] + offsets[n][1]] - src[n][0]).max()) for n in RIGID)
    log(f"RIGID parts: vertex positions vs source max abs diff {rig_dev:.3e} m, face lists copied unchanged")

    # ---- hidden ends (depth inside the covering source shells; + inside)
    down = [Vector(d).normalized() for d in ((0.1, 0.05, -1.0), (-0.08, 0.1, -1.0), (0.05, -0.1, -1.0))]
    for name in ("arm_l", "arm_r", "leg_l", "leg_r"):
        x = name[-1]
        for m_ in tube_meta[name]["meta"]:
            if m_["kind"] not in ("end0", "end1"):
                continue
            P = geo[name][0][m_["ids"]]
            if name.startswith("arm"):
                cov = {"sleeve": S[f"sleeve_{x}"], "tunic": S["tunic"]} if m_["kind"] == "end0" else {"fist": S[f"fist_{x}"]}
                dep = {k: np.array([sv.depth(p) for p in P]) for k, sv in cov.items()}
            elif m_["kind"] == "end0":
                dep = {"tunic(source)": np.array([S["tunic"].depth(p) for p in P])}
                dep["above_ceiling_mm"] = (P[:, 2] - CEILING_Z)
            else:
                dep = {"cuff": np.array([S[f"cuff_{x}"].depth(p) for p in P]),
                       "shoe(down rays)": np.array([S[f"shoe_{x}"].depth(p, down) for p in P])}
            log(f"END {m_['name']}: " + ", ".join(f"{k} min/max {mm(v.min())}/{mm(v.max())} mm" for k, v in dep.items()))

    # ---- deviation (rebuilt parts vs source part)
    rng = np.random.default_rng(0)
    closed_src = {n: S[n] for n in PART_NAMES if n not in ("arm_l", "arm_r", "leg_l", "leg_r", "shoe_l", "shoe_r")}
    n_out_rows = len(O)
    for name in REBUILT:
        Vr, Fr = geo[name]
        Vr = np.asarray(Vr)
        Tr, fr = tris_of(Vr, Fr)
        R = Surf(Vr, Fr)
        if name == "tunic":
            design_f = np.zeros(len(Fr), bool)
            design_f[(n_out_rows - 1) * N_COL:] = True      # hem roll, inner wall, ceiling (new surfaces)
        else:
            hid = hidden_local[name]
            design_f = np.array([bool(hid[list(f)].any()) for f in Fr])
        pr, ti = sample_surface(Vr, Tr, DEV_SAMPLES, rng)
        d_rs = np.array([S[name].dist(p) for p in pr])
        h_rs = occluded(pr, name, closed_src)
        dz = design_f[fr[ti]]
        Vs, Fs = src[name]
        Ts, _fs = tris_of(Vs, Fs)
        ps, ts = sample_surface(Vs, Ts, DEV_SAMPLES, rng)
        d_sr = np.array([R.dist(p) for p in ps])
        h_sr = occluded(ps, name, closed_src)
        red = np.zeros(len(ps), bool)
        if name == "tunic":
            red = ((S[name].TN[:, 2] < CAP_NZ) & (S[name].TC[:, 2] < CAP_ZMAX))[ts]
        log(f"DEV {name} retopo->SRC: all {dstats(d_rs)}; not hidden {dstats(d_rs[~h_rs])} (hidden {int(h_rs.sum())}); "
            f"not hidden and not design region {dstats(d_rs[~h_rs & ~dz])} (design-region samples {int(dz.sum())}: "
            f"{'hem roll / inner wall / ceiling' if name == 'tunic' else 'capped ends / extension bands'})")
        log(f"DEV {name} SRC->retopo: all {dstats(d_sr)}; not hidden {dstats(d_sr[~h_sr])} (hidden {int(h_sr.sum())}); "
            f"not hidden and not redesign {dstats(d_sr[~h_sr & ~red])}"
            + (f" (source bottom cap samples {int(red.sum())}: {dstats(d_sr[red])})" if name == "tunic" else ""))

    # ---- renders
    INSPECT.mkdir(parents=True, exist_ok=True)
    out = []
    for view in ("front", "side", "back", "three_quarter"):
        out.append(ortho_render([ob], view, INSPECT / f"parts_{view}.png"))
    el = mirror(sym(lm, "elbow"), "l")
    kn = mirror(sym(lm, "knee"), "l")
    shl = mirror(sym(lm, "shoulder"), "l")
    out.append(ortho_render([ob], "front", INSPECT / "wire_elbow_l.png", wire=True,
                            frame_bbox=((el[0] - 0.09, -0.3, el[2] - 0.09), (el[0] + 0.09, 0.3, el[2] + 0.09))))
    out.append(ortho_render([ob], "front", INSPECT / "wire_shoulder_l.png", wire=True,
                            frame_bbox=((shl[0] - 0.08, -0.3, shl[2] - 0.12), (shl[0] + 0.14, 0.3, shl[2] + 0.06))))
    out.append(ortho_render([ob], "front", INSPECT / "wire_knee_l.png", wire=True,
                            frame_bbox=((kn[0] - 0.08, -0.3, kn[2] - 0.09), (kn[0] + 0.08, 0.3, kn[2] + 0.07))))
    out.append(ortho_render([ob], "side", INSPECT / "wire_knee_l_side.png", wire=True,
                            frame_bbox=((0.0, kn[1] - 0.09, kn[2] - 0.09), (0.3, kn[1] + 0.09, kn[2] + 0.07))))
    out.append(ortho_render([ob], "three_quarter", INSPECT / "wire_skirt_outside.png", wire=True,
                            frame_bbox=((-0.25, -0.25, 0.26), (0.25, 0.25, 0.50))))
    for names, fname in ((["arm_l"], "wire_tube_arm_l.png"), (["leg_l"], "wire_tube_leg_l.png")):
        so = subset_object(ob, names)
        out.append(ortho_render([so], "front", INSPECT / fname, wire=True))
        m2 = so.data
        bpy.data.objects.remove(so, do_unlink=True)
        bpy.data.meshes.remove(m2)
    so = subset_object(ob, ["tunic", "leg_l", "leg_r"])
    out.append(ortho_render([so], "below", INSPECT / "wire_skirt_below.png", wire=True,
                            frame_bbox=((-0.25, -0.23, 0.25), (0.25, 0.25, 0.45))))
    out.append(ortho_render([so], "below_front", INSPECT / "wire_skirt_below_front.png", wire=True,
                            frame_bbox=((-0.25, -0.25, 0.20), (0.25, 0.25, 0.48))))
    m2 = so.data
    bpy.data.objects.remove(so, do_unlink=True)
    bpy.data.meshes.remove(m2)
    write_uv_png(ur["_owner"], INSPECT / "uv_layout.png")
    out.append(INSPECT / "uv_layout.png")
    log("RENDERS " + ", ".join(str(Path(p).relative_to(REPO)).replace("\\", "/") for p in out))
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
