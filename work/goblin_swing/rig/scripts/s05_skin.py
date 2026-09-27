"""s05_skin.py - T21: skinning of the goblin rig (G5.1-G5.6).

Input : rig/gob_r02_skeleton.blend (GOB_mesh, GOB_club, GOB_rig), rig/data/parts.json,
        rig/data/retopo_loops.json, rig/data/pivots.json
Output: rig/gob_r03_skinned.blend (save copy; the input is not overwritten), rig/data/skin_decisions.json

Run:  bl.ps1 -Script s05_skin.py -Blend gob_r02_skeleton.blend

Bones (position / roll / hierarchy) are not changed; no bone is added.  Mesh topology / positions untouched.

Weights (22 deform-bone vertex groups; root / weapon_socket_r get no group = weight 0):
  1. body (part_id body) starts from Blender bone-heat automatic weights computed on a body-only copy
     (deform bones only); heat-failed vertices -> nearest deform bone.
  2. limb tubes (topological regions, BFS over body edges):
       arm_x = verts reached from ring wrist_x_end without crossing ring shoulder_x_2 (+ that ring),
       leg_x = verts reached from ring ankle_x_end without crossing ring hip_x_0 (+ that ring).
     Axial coordinate s along the tube axis (pivots.json axis) from each joint centre (elbow / wrist / knee /
     ankle = retopo ring _1 centre, shoulder / hip = pivot).  Per joint the distal share is a profile of s
     (constants ELBOW / WRIST / KNEE / ANKLE / HIP); for the hinge joints it also depends on the angular
     position around the tube (ci = cos of the angle to the bend-inner direction: elbow -Y, knee +Y).
       arm : hand = 1 for s_wrist >= -RIGID_EPS (wrist_x_1 .. end + cap, inside the hand shell), else WRIST;
             lowerarm share = ELBOW; proximal (non-chain) heat weight kept, faded out along the tube.
             upperarm share -> upperarm_twist (proximal) / upperarm by UA_TWIST_SPLIT along the upperarm;
             lowerarm share -> lowerarm / lowerarm_twist (distal) by LA_TWIST_SPLIT along the lowerarm.
       leg : foot = 1 for s_ankle >= -RIGID_EPS (ankle_x_1 .. end + cap, inside the shoe shell), else ANKLE;
             calf share = KNEE; thigh vs pelvis = HIP (s from the hip pivot).
     Arm junction (shoulder rings, torso side of shoulder_x_2): heat kept; upperarm+upperarm_twist heat moved to
     the twist split (upperarm_twist near the shoulder).
     Torso outside the leg regions: thigh weight limited to min(heat, HIP(s_hip)), calf/foot 0, rest -> pelvis.
  3. belt band: body verts with belt_bot_0.z <= z <= belt_top_0.z -> pelvis 100 %, faded back to the shaped
     weights over BELT_FADE above / below the band.
  4. head: body verts inside the head shell (ray parity) -> head 100 %.
  5. rigid parts 100 % to one bone (head, hand_x, shoe_x -> foot_x, belt -> pelvis).
  6. <= MAX_INF influences per vertex (largest kept), weights < EPS_W dropped, normalized.
Twist constraints (Transformation, from_rotation_mode SWING_TWIST_Y, rotation Y -> rotation Y only, LOCAL/LOCAL):
  only the pure axial twist of the source is mapped (swing ignored); to range = sign * ratio * from range.
  upperarm_twist_x <- upperarm_x, sign -1, ratio UA_TWIST_RATIO (counter-twist: world twist of the twist bone =
  (1 - ratio) of the upperarm twist);  lowerarm_twist_x <- hand_x, sign +1, ratio LA_TWIST_RATIO.
GOB_club: bone parent weapon_socket_r, world transform kept, no weights.  Armature modifier on GOB_mesh.
HR2 (rig/data/hr2_contract.md): GOB_mesh shape keys (Basis / mouth_open) and its 4 material slots are left as they are
(only vertex groups + the Armature modifier are added); every head-part vertex, including the mouth vertices of
data/mouth.json region output_verts, is 100 % DEF head through rule 5 (reported as MOUTH line).
"""
import json
import math
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r03_skinned.blend"
OUT_JSON = "skin_decisions.json"
ARM = "GOB_rig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
SIDES = ("l", "r")
MAX_INF = 4
EPS_W = 1e-4
RIGID_BONE = {"head": "head", "hand_l": "hand_l", "hand_r": "hand_r", "shoe_l": "foot_l",
              "shoe_r": "foot_r", "belt": "pelvis"}
NON_DEFORM = {"root", "weapon_socket_r"}

# ---------------------------------------------------------------- weight shaping constants (m)
RIGID_EPS = 0.001            # tube verts with s_wrist / s_ankle >= -RIGID_EPS follow hand / foot 100 %
# Joint profiles = distal-bone share as a function of s (m, along the tube axis from the joint centre).
#   {"s", "w"}           piecewise-linear knots (clamped outside).
#   {"c", "h"}           hinge: smoothstep ramp(s; c, h); centre c / half width h given as [inner, mid(90 deg),
#                        outer] and linear in ci = cos(angle to the bend-inner direction) (ci>0 in..mid, ci<0 mid..out).
#   {"s", "ci", "grid"}  hinge: bilinear table, grid[k] = knot values at s for ci = ci[k].
# Tuned with the G3/G5 deform metrics on the judged ROM poses (T21 debug harness, see skin_decisions.json).
ELBOW = {"s": [-0.045, -0.030, -0.020, -0.010, 0.000, 0.010, 0.020, 0.030, 0.045],
         "ci": [-1.0, -0.866, -0.5, 0.0, 0.5, 0.866, 1.0],
         "grid": [[0, 0, 0, 0, 0.500, 1, 1, 1, 1],
                  [0, 0, 0, 0, 0.458, 1, 1, 1, 1],
                  [0, 0, 0, 0, 0.394, 1, 1, 1, 1],
                  [0, 0, 0, 0, 0.352, 0.896, 1, 1, 1],
                  [0, 0, 0, 0.242, 0.470, 0.705, 0.896, 0.995, 1],
                  [0, 0, 0.134, 0.331, 0.494, 0.659, 0.807, 0.923, 1],
                  [0, 0.104, 0.176, 0.352, 0.490, 0.668, 0.784, 0.896, 1]]}
WRIST = {"s": [-0.070, -0.001], "w": [0.0, 1.0]}     # hand share for s_wrist < -RIGID_EPS
KNEE = {"c": [0.0, -0.005, 0.0], "h": [0.050, 0.0225, 0.005]}
ANKLE = {"s": [-0.023, -0.001], "w": [0.0, 1.0]}     # foot share for s_ankle < -RIGID_EPS
HIP = {"s": [-0.040, -0.030, -0.020, -0.010, 0.000, 0.020, 0.040],   # thigh share vs pelvis (s from hip pivot)
       "w": [0.0, 0.0, 0.22, 0.30, 0.44, 0.90, 1.0]}
ARM_PROX_FADE = {"c": 0.040, "h": 0.030}  # arm tube: non-chain heat weight *= 1 - ramp(s_shoulder; c, h)
# twist split: t = fraction along the segment (0 = proximal joint, 1 = distal joint)
UA_TWIST_SPLIT = {"c": 0.45, "h": 0.30}   # upperarm_twist part of the upperarm share = 1 - ramp(t; c, h)
LA_TWIST_SPLIT = {"c": 0.55, "h": 0.30}   # lowerarm_twist part of the lowerarm share = ramp(t; c, h)
UA_TWIST_RATIO = 0.5
LA_TWIST_RATIO = 0.5
TWIST_MIX_ROT = "REPLACE"    # Transformation mix_mode_rot (twist bones carry no pose of their own)
BELT_FADE = 0.030            # pelvis-only band fades back to the shaped weights over this distance in z
INNER_DIR = {"elbow": (0.0, -1.0, 0.0), "knee": (0.0, 1.0, 0.0)}
RAY_DIRS = ((0.5773, 0.5271, 0.6237), (-0.6428, 0.2819, 0.7124), (0.2113, -0.8356, -0.5071))

DEF_NAMES = ["pelvis", "spine_01", "spine_02", "head",
             "shoulder_l", "upperarm_l", "upperarm_twist_l", "lowerarm_l", "lowerarm_twist_l", "hand_l",
             "shoulder_r", "upperarm_r", "upperarm_twist_r", "lowerarm_r", "lowerarm_twist_r", "hand_r",
             "thigh_l", "calf_l", "foot_l", "thigh_r", "calf_r", "foot_r"]
BI = {n: i for i, n in enumerate(DEF_NAMES)}


def default_params():
    return json.loads(json.dumps({
        "ELBOW": ELBOW, "WRIST": WRIST, "KNEE": KNEE, "ANKLE": ANKLE, "HIP": HIP, "ARM_PROX_FADE": ARM_PROX_FADE,
        "UA_TWIST_SPLIT": UA_TWIST_SPLIT, "LA_TWIST_SPLIT": LA_TWIST_SPLIT, "BELT_FADE": BELT_FADE,
        "RIGID_EPS": RIGID_EPS}))


# ---------------------------------------------------------------- helpers
def ramp(x, c, h):
    x = np.asarray(x, dtype=np.float64)
    h = np.asarray(h, dtype=np.float64)
    c = np.asarray(c, dtype=np.float64)
    hh = np.maximum(h, 1e-9)
    t = np.clip((x - (c - hh)) / (2.0 * hh), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def profile(s, prof, cos_in=None):
    s = np.asarray(s, dtype=np.float64)
    if "w" in prof:
        return np.interp(s, prof["s"], prof["w"])
    if "grid" in prof:   # bilinear table: rows = ci knots, columns = s knots
        ci = np.asarray(cos_in, dtype=np.float64)
        rows = np.stack([np.interp(s, prof["s"], r) for r in prof["grid"]], axis=1)   # (n, n_ci)
        cik = np.asarray(prof["ci"], dtype=np.float64)
        j = np.clip(np.searchsorted(cik, ci) - 1, 0, len(cik) - 2)
        t = np.clip((ci - cik[j]) / (cik[j + 1] - cik[j]), 0.0, 1.0)
        n = np.arange(len(s))
        return (1.0 - t) * rows[n, j] + t * rows[n, j + 1]
    if "c" in prof:   # smoothstep ramp, centre / half width linear in ci between (in, mid) and (mid, out)
        ci = np.asarray(cos_in, dtype=np.float64)
        (c_in, c_mid, c_out), (h_in, h_mid, h_out) = prof["c"], prof["h"]
        cc = np.where(ci >= 0, ci * c_in + (1.0 - ci) * c_mid, -ci * c_out + (1.0 + ci) * c_mid)
        hh = np.where(ci >= 0, ci * h_in + (1.0 - ci) * h_mid, -ci * h_out + (1.0 + ci) * h_mid)
        return ramp(s, cc, hh)
    raise ValueError(f"unknown profile keys {sorted(prof)}")


def unit(v):
    v = np.asarray(v, dtype=np.float64)
    return v / max(float(np.linalg.norm(v)), 1e-12)


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab[None, :]), axis=1)


def bfs(adj, seeds, blocked):
    seen = np.zeros(len(adj), dtype=bool)
    q = deque()
    for s in seeds:
        if not blocked[s] and not seen[s]:
            seen[s] = True
            q.append(s)
    while q:
        v = q.popleft()
        for u in adj[v]:
            if not seen[u] and not blocked[u]:
                seen[u] = True
                q.append(u)
    return seen


def inside_mask(pts, bvh):
    out = np.zeros(len(pts), dtype=bool)
    dirs = [Vector(d).normalized() for d in RAY_DIRS]
    for i, p in enumerate(pts.tolist()):
        votes = 0
        for d in dirs:
            o, n = Vector(p), 0
            for _ in range(256):
                hit = bvh.ray_cast(o, d)
                if hit[0] is None:
                    break
                n += 1
                o = hit[0] + d * 1e-6
            votes += n % 2
        out[i] = votes >= 2
    return out


# ---------------------------------------------------------------- context
class Ctx:
    pass


def load_ctx():
    c = Ctx()
    c.arm = bpy.data.objects[ARM]
    c.gm = bpy.data.objects[MESH]
    c.club = bpy.data.objects[CLUB]
    c.parts = goblib.load_json("parts.json")
    c.rings = goblib.load_json("retopo_loops.json")["rings"]
    c.piv = goblib.load_json("pivots.json")["pivots"]
    me = c.gm.data
    nv = len(me.vertices)
    co = np.empty(nv * 3)
    me.vertices.foreach_get("co", co)
    c.co = co.reshape(nv, 3)
    c.nv = nv
    if not np.allclose(np.array(c.gm.matrix_world), np.eye(4)):
        raise RuntimeError("GOB_mesh matrix_world is not identity")
    if not np.allclose(np.array(c.arm.matrix_world), np.eye(4)):
        raise RuntimeError("GOB_rig matrix_world is not identity")
    if len(c.gm.vertex_groups) or len(c.gm.modifiers):
        raise RuntimeError("GOB_mesh already has vertex groups / modifiers")
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    ls = np.empty(len(me.polygons), dtype=np.int32)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_start", ls)
    me.polygons.foreach_get("loop_total", lt)
    lv = np.empty(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("vertex_index", lv)
    lp = np.repeat(pid, lt)
    vpart = np.full(nv, -1, dtype=np.int32)
    vpart[lv] = lp
    chk = np.full(nv, -1, dtype=np.int32)
    np.maximum.at(chk, lv, lp)
    if (vpart < 0).any() or (chk != vpart).any():
        raise RuntimeError("vertex part assignment ambiguous / loose vertices")
    c.vpart = vpart
    c.pid = pid
    ed = np.empty(len(me.edges) * 2, dtype=np.int32)
    me.edges.foreach_get("vertices", ed)
    ed = ed.reshape(-1, 2)
    c.body = vpart == c.parts["body"]
    ed = ed[c.body[ed[:, 0]] & c.body[ed[:, 1]]]
    c.edges = ed
    adj = [[] for _ in range(nv)]
    for a, b in ed.tolist():
        adj[a].append(b)
        adj[b].append(a)
    c.adj = adj
    c.bones = c.arm.data.bones
    missing = [n for n in DEF_NAMES if n not in c.bones or not c.bones[n].use_deform]
    extra = [b.name for b in c.bones if b.use_deform and b.name not in BI]
    if missing or extra:
        raise RuntimeError(f"deform bone mismatch: missing={missing} extra={extra}")
    print(f"[s05] GOB_mesh verts={nv} per part: " + ", ".join(
        f"{n}={int((vpart == v).sum())}" for n, v in c.parts.items()) + f"; body edges={len(ed)}")
    return c


def ring_idx(c, name):
    return np.array(c.rings[name]["verts"], dtype=np.int64)


def ring_center(c, name):
    return c.co[ring_idx(c, name)].mean(axis=0)


# ---------------------------------------------------------------- heat weights
def heat_weights(c):
    me = c.gm.data
    nv = c.nv
    body_idx = np.nonzero(c.body)[0]
    tmp_me = me.copy()
    tmp_me.name = "_s05_body_tmp"
    tmp = bpy.data.objects.new("_s05_body_tmp", tmp_me)
    bpy.context.scene.collection.objects.link(tmp)
    oi = tmp_me.attributes.new("_orig_idx", "INT", "POINT")
    oi.data.foreach_set("value", np.arange(nv, dtype=np.int32))
    bm = bmesh.new()
    bm.from_mesh(tmp_me)
    bm.verts.ensure_lookup_table()
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not c.body[v.index]], context="VERTS")
    bm.to_mesh(tmp_me)
    bm.free()
    tmp_me.update()
    orig = np.empty(len(tmp_me.vertices), dtype=np.int32)
    tmp_me.attributes["_orig_idx"].data.foreach_get("value", orig)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    tmp.select_set(True)
    c.arm.select_set(True)
    vl.objects.active = c.arm
    bpy.ops.object.parent_set(type="ARMATURE_AUTO")
    W = np.zeros((nv, len(DEF_NAMES)))
    gname = {g.index: g.name for g in tmp.vertex_groups}
    for v in tmp_me.vertices:
        for g in v.groups:
            n = gname[g.group]
            if n in BI:
                W[orig[v.index], BI[n]] = g.weight
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.meshes.remove(tmp_me)
    for o in vl.objects:
        o.select_set(False)
    seg = [(np.array(c.bones[n].head_local), np.array(c.bones[n].tail_local)) for n in DEF_NAMES]
    zero = body_idx[W[body_idx].sum(axis=1) <= EPS_W]
    if len(zero):
        dist = np.stack([seg_dist(c.co[zero], a, b) for a, b in seg], axis=1)
        near = dist.argmin(axis=1)
        W[zero] = 0.0
        W[zero, near] = 1.0
    s = W[body_idx].sum(axis=1)
    W[body_idx] /= s[:, None]
    print(f"[s05] heat: body verts={len(body_idx)}, heat-failed filled by nearest bone={len(zero)}")
    return W


# ---------------------------------------------------------------- geometry per limb
def limb_geometry(c):
    """Regions and axial coordinates of the limb tubes (per side)."""
    g = {}
    for x in SIDES:
        blocked = np.zeros(c.nv, dtype=bool)
        blocked[ring_idx(c, f"shoulder_{x}_2")] = True
        arm = bfs(c.adj, ring_idx(c, f"wrist_{x}_end").tolist(), blocked)
        arm[ring_idx(c, f"shoulder_{x}_2")] = True
        blocked = np.zeros(c.nv, dtype=bool)
        blocked[ring_idx(c, f"hip_{x}_0")] = True
        leg = bfs(c.adj, ring_idx(c, f"ankle_{x}_end").tolist(), blocked)
        leg[ring_idx(c, f"hip_{x}_0")] = True
        a_ax = unit(c.piv[f"shoulder_{x}"]["axis"])
        l_ax = unit(c.piv[f"hip_{x}"]["axis"])
        sh = np.array(c.piv[f"shoulder_{x}"]["co"])
        el = ring_center(c, f"elbow_{x}_1")
        wr = ring_center(c, f"wrist_{x}_1")
        hp = np.array(c.piv[f"hip_{x}"]["co"])
        kn = ring_center(c, f"knee_{x}_1")
        an = ring_center(c, f"ankle_{x}_1")
        P = c.co

        def cos_inner(center, ax, inner):
            d = P - center
            r = d - np.outer(d @ ax, ax)
            rn = np.linalg.norm(r, axis=1)
            iv = unit(np.array(inner) - np.dot(inner, ax) * ax)
            return np.where(rn > 1e-9, (r @ iv) / np.maximum(rn, 1e-9), 0.0)

        g[x] = {
            "arm": arm, "leg": leg, "arm_axis": a_ax, "leg_axis": l_ax,
            "s_sh": (P - sh) @ a_ax, "s_el": (P - el) @ a_ax, "s_wr": (P - wr) @ a_ax,
            "s_hp": (P - hp) @ l_ax, "s_kn": (P - kn) @ l_ax, "s_an": (P - an) @ l_ax,
            "L_ua": float((el - sh) @ a_ax), "L_la": float((wr - el) @ a_ax),
            "cos_el": cos_inner(el, a_ax, INNER_DIR["elbow"]),
            "cos_kn": cos_inner(kn, l_ax, INNER_DIR["knee"]),
        }
        g[x]["ring_s"] = {}
        for r in (f"shoulder_{x}_0", f"shoulder_{x}_1", f"shoulder_{x}_2", f"elbow_{x}_0", f"elbow_{x}_1",
                  f"elbow_{x}_2", f"wrist_{x}_0", f"wrist_{x}_1", f"wrist_{x}_end"):
            ri = ring_idx(c, r)
            g[x]["ring_s"][r] = (float(g[x]["s_sh"][ri].mean()), float(g[x]["s_el"][ri].mean()),
                                 float(g[x]["s_wr"][ri].mean()))
        for r in (f"hip_{x}_0", f"hip_{x}_1", f"hip_{x}_2", f"knee_{x}_0", f"knee_{x}_1", f"knee_{x}_2",
                  f"ankle_{x}_0", f"ankle_{x}_1", f"ankle_{x}_end"):
            ri = ring_idx(c, r)
            g[x]["ring_s"][r] = (float(g[x]["s_hp"][ri].mean()), float(g[x]["s_kn"][ri].mean()),
                                 float(g[x]["s_an"][ri].mean()))
    return g


def torso_masks(c):
    body = c.body
    zb0 = float(c.co[ring_idx(c, "belt_bot_0"), 2].mean())
    zb1 = float(c.co[ring_idx(c, "belt_top_0"), 2].mean())
    head_f = np.nonzero(c.pid == c.parts["head"])[0]
    me = c.gm.data
    me.calc_loop_triangles()
    nt = len(me.loop_triangles)
    tv = np.empty(nt * 3, dtype=np.int32)
    tp = np.empty(nt, dtype=np.int32)
    me.loop_triangles.foreach_get("vertices", tv)
    me.loop_triangles.foreach_get("polygon_index", tp)
    tv = tv.reshape(-1, 3)
    htri = tv[np.isin(tp, head_f)]
    bvh = BVHTree.FromPolygons([tuple(p) for p in c.co.tolist()], htri.tolist(), all_triangles=True)
    hz = float(c.co[c.vpart == c.parts["head"], 2].min())
    cand = np.nonzero(body & (c.co[:, 2] >= hz - 0.002))[0]
    ins = np.zeros(c.nv, dtype=bool)
    ins[cand] = inside_mask(c.co[cand], bvh)
    return {"zb0": zb0, "zb1": zb1, "head_in": ins, "head_zmin": hz}


# ---------------------------------------------------------------- weights
def build_weights(c, H, geo, tm, p, verbose=True):
    W = H.copy()
    body = c.body
    rig_eps = p["RIGID_EPS"]
    in_limb = np.zeros(c.nv, dtype=bool)
    for x in SIDES:
        g = geo[x]
        chain = [BI[n + "_" + x] for n in ("upperarm", "upperarm_twist", "lowerarm", "lowerarm_twist", "hand")]
        nonchain = [i for i in range(len(DEF_NAMES)) if i not in chain]
        ua, uat, la, lat, hd = chain
        # ---- arm tube
        m = g["arm"] & body
        in_limb |= m
        e = profile(g["s_el"][m], p["ELBOW"], g["cos_el"][m])
        hnd = np.where(g["s_wr"][m] >= -rig_eps, 1.0, profile(g["s_wr"][m], p["WRIST"]))
        Pn = H[m][:, nonchain] * (1.0 - ramp(g["s_sh"][m], p["ARM_PROX_FADE"]["c"], p["ARM_PROX_FADE"]["h"]))[:, None]
        Pn = np.where((hnd >= 1.0)[:, None], 0.0, Pn)
        ch = 1.0 - Pn.sum(axis=1)
        upper = ch * (1.0 - hnd) * (1.0 - e)
        lower = ch * (1.0 - hnd) * e
        f_ut = 1.0 - ramp(g["s_sh"][m] / g["L_ua"], p["UA_TWIST_SPLIT"]["c"], p["UA_TWIST_SPLIT"]["h"])
        f_lt = ramp(g["s_el"][m] / g["L_la"], p["LA_TWIST_SPLIT"]["c"], p["LA_TWIST_SPLIT"]["h"])
        Wm = np.zeros((int(m.sum()), len(DEF_NAMES)))
        Wm[:, nonchain] = Pn
        Wm[:, uat] = upper * f_ut
        Wm[:, ua] = upper * (1.0 - f_ut)
        Wm[:, lat] = lower * f_lt
        Wm[:, la] = lower * (1.0 - f_lt)
        Wm[:, hd] = ch * hnd
        W[m] = Wm
        # ---- arm junction (body outside the tube): upperarm heat -> twist split
        j = body & ~g["arm"] & ((H[:, ua] + H[:, uat]) > 0)
        u = H[j, ua] + H[j, uat]
        fj = 1.0 - ramp(g["s_sh"][j] / g["L_ua"], p["UA_TWIST_SPLIT"]["c"], p["UA_TWIST_SPLIT"]["h"])
        W[j, uat] = u * fj
        W[j, ua] = u * (1.0 - fj)
        # ---- leg region
        th, ca, ft = BI["thigh_" + x], BI["calf_" + x], BI["foot_" + x]
        pe = BI["pelvis"]
        m = g["leg"] & body
        in_limb |= m
        k = profile(g["s_kn"][m], p["KNEE"], g["cos_kn"][m])
        fo = np.where(g["s_an"][m] >= -rig_eps, 1.0, profile(g["s_an"][m], p["ANKLE"]))
        t = profile(g["s_hp"][m], p["HIP"])
        Wm = np.zeros((int(m.sum()), len(DEF_NAMES)))
        Wm[:, ft] = fo
        Wm[:, ca] = (1.0 - fo) * k
        Wm[:, th] = (1.0 - fo) * (1.0 - k) * t
        Wm[:, pe] = (1.0 - fo) * (1.0 - k) * (1.0 - t)
        W[m] = Wm
    # ---- torso outside the limb regions: limit thigh / calf / foot to the hip ramp, rest -> pelvis
    tor = body & ~in_limb
    for x in SIDES:
        g = geo[x]
        lim = profile(g["s_hp"][tor], p["HIP"])
        for n in ("thigh_", "calf_", "foot_"):
            i = BI[n + x]
            old = W[tor, i]
            new = np.minimum(old, lim) if n == "thigh_" else np.zeros_like(old)
            W[tor, i] = new
            W[tor, BI["pelvis"]] += old - new
    # ---- belt band -> pelvis
    z = c.co[:, 2]
    dz = np.maximum(tm["zb0"] - z, z - tm["zb1"])
    gfade = ramp(dz, 0.5 * p["BELT_FADE"], 0.5 * p["BELT_FADE"]) if p["BELT_FADE"] > 0 else (dz > 0).astype(float)
    bm = tor & (gfade < 1.0)
    onehot = np.zeros(len(DEF_NAMES))
    onehot[BI["pelvis"]] = 1.0
    W[bm] = (1.0 - gfade[bm])[:, None] * onehot[None, :] + gfade[bm][:, None] * W[bm]
    # ---- head dome
    hin = tm["head_in"]
    onehot = np.zeros(len(DEF_NAMES))
    onehot[BI["head"]] = 1.0
    W[hin] = onehot
    # ---- rigid parts
    for pname, bname in RIGID_BONE.items():
        mm_ = c.vpart == c.parts[pname]
        W[mm_] = 0.0
        W[mm_, BI[bname]] = 1.0
    # ---- limit + normalize
    W = np.where(W > 0, W, 0.0)
    before = int(((W > EPS_W).sum(axis=1) > MAX_INF).sum())
    order = np.argsort(-W, axis=1)
    keep = np.zeros_like(W, dtype=bool)
    np.put_along_axis(keep, order[:, :MAX_INF], True, axis=1)
    W = np.where(keep & (W > EPS_W), W, 0.0)
    s = W.sum(axis=1)
    if (s <= 0).any():
        raise RuntimeError(f"{int((s <= 0).sum())} vertices with zero weight after limit")
    W = W / s[:, None]
    if verbose:
        print(f"[s05] limit: verts with >{MAX_INF} influences before limit={before}")
    return W


def assign_groups(c, W):
    gm = c.gm
    for vg in list(gm.vertex_groups):
        gm.vertex_groups.remove(vg)
    for n in DEF_NAMES:
        vg = gm.vertex_groups.new(name=n)
        col = W[:, BI[n]]
        idx = np.nonzero(col > 0)[0]
        for i in idx.tolist():
            vg.add([i], float(col[i]), "REPLACE")


# ---------------------------------------------------------------- constraints / modifier / club
def setup_rig(c):
    arm = c.arm
    info = {}
    for x in SIDES:
        for tw, src, inv, ratio in ((f"upperarm_twist_{x}", f"upperarm_{x}", True, UA_TWIST_RATIO),
                                    (f"lowerarm_twist_{x}", f"hand_{x}", False, LA_TWIST_RATIO)):
            pb = arm.pose.bones[tw]
            for con in list(pb.constraints):
                pb.constraints.remove(con)
            sign = -1 if inv else 1
            con = pb.constraints.new("TRANSFORM")
            con.name = "twist"
            con.target = arm
            con.subtarget = src
            con.target_space = "LOCAL"
            con.owner_space = "LOCAL"
            con.map_from = "ROTATION"
            con.map_to = "ROTATION"
            con.from_rotation_mode = "SWING_TWIST_Y"
            con.map_to_x_from, con.map_to_y_from, con.map_to_z_from = "X", "Y", "Z"
            con.use_motion_extrapolate = False
            for ax in "xz":   # swing components -> 0
                setattr(con, f"from_min_{ax}_rot", -math.pi)
                setattr(con, f"from_max_{ax}_rot", math.pi)
                setattr(con, f"to_min_{ax}_rot", 0.0)
                setattr(con, f"to_max_{ax}_rot", 0.0)
            con.from_min_y_rot, con.from_max_y_rot = -math.pi, math.pi
            con.to_min_y_rot, con.to_max_y_rot = -sign * ratio * math.pi, sign * ratio * math.pi
            con.mix_mode_rot = TWIST_MIX_ROT
            con.influence = 1.0
            info[tw] = {"source": src, "constraint": "TRANSFORM", "from_rotation_mode": "SWING_TWIST_Y",
                        "axis": "Y", "ratio": ratio, "sign": sign, "target_space": "LOCAL", "owner_space": "LOCAL",
                        "mix_mode_rot": TWIST_MIX_ROT,
                        "effect": ("counter-twist: twist bone rotates -ratio * upperarm swing-twist Y twist relative to "
                                   "upperarm (world twist = (1 - ratio) * upperarm twist); swing ignored"
                                   if inv else "twist bone rotates +ratio * hand swing-twist Y twist relative to "
                                   "lowerarm; swing ignored")}
    mod = c.gm.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    mod.use_vertex_groups = True
    mod.use_bone_envelopes = False
    mod.use_deform_preserve_volume = False
    club = c.club
    club_mw = club.matrix_world.copy()
    club.parent = arm
    club.parent_type = "BONE"
    club.parent_bone = "weapon_socket_r"
    bpy.context.view_layer.update()
    club.matrix_world = club_mw
    bpy.context.view_layer.update()
    cdev = float(np.abs(np.array(club.matrix_world) - np.array(club_mw)).max())
    return info, cdev


def reset_pose(arm):
    for pb in arm.pose.bones:
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion()
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)


def eval_co(gm):
    dg = bpy.context.evaluated_depsgraph_get()
    e = gm.evaluated_get(dg)
    em = e.to_mesh()
    a = np.empty(len(em.vertices) * 3)
    em.vertices.foreach_get("co", a)
    e.to_mesh_clear()
    return a.reshape(-1, 3)


def _rel_twist_deg(arm, bone):
    """Swing-twist Y angle of the twist bone relative to its parent: (rest parent->bone)^-1 (posed parent->bone)."""
    par = arm.data.bones[bone].parent.name
    rest_rel = arm.data.bones[par].matrix_local.inverted() @ arm.data.bones[bone].matrix_local
    pose_rel = arm.pose.bones[par].matrix.inverted() @ arm.pose.bones[bone].matrix
    q = (rest_rel.inverted() @ pose_rel).to_quaternion()
    ang = math.degrees(2.0 * math.atan2(q.y, q.w))
    return (ang + 180.0) % 360.0 - 180.0


def twist_selftest(c):
    """Twist bone axial rotation relative to its parent: (a) source local Y +90 deg twist (expected
    sign * ratio * 90); (b) twist-free diagonal swing of the source about local (X+Z)/sqrt2 (upperarm 90 deg,
    hand 60 deg): leak = |axial rotation| (expected 0)."""
    arm = c.arm
    out = {}
    for x in SIDES:
        for tw, src, swing in ((f"upperarm_twist_{x}", f"upperarm_{x}", 90.0),
                               (f"lowerarm_twist_{x}", f"hand_{x}", 60.0)):
            reset_pose(arm)
            arm.pose.bones[src].rotation_quaternion = Quaternion((0, 1, 0), math.radians(90.0))
            bpy.context.view_layer.update()
            out[tw + "_twist90"] = round(_rel_twist_deg(arm, tw), 3)
            reset_pose(arm)
            arm.pose.bones[src].rotation_quaternion = Quaternion(Vector((1, 0, 1)).normalized(), math.radians(swing))
            bpy.context.view_layer.update()
            out[tw + f"_swing{int(swing)}_leak"] = round(abs(_rel_twist_deg(arm, tw)), 3)
    reset_pose(arm)
    bpy.context.view_layer.update()
    return out


# ---------------------------------------------------------------- report
def report(c, W):
    for pname, pv in c.parts.items():
        m = c.vpart == pv
        cnt = (W[m] > 0).sum(axis=1)
        serr = int((np.abs(W[m].sum(axis=1) - 1.0) > 1e-4).sum())
        dom = np.bincount(W[m].argmax(axis=1), minlength=len(DEF_NAMES))
        print(f"[s05] WEIGHTS part {pname} ({int(m.sum())} v): max influences={int(cnt.max())}, "
              f"sum errors(|sum-1|>1e-4)={serr}; dominant bone counts="
              + ", ".join(f"{DEF_NAMES[i]}:{int(dom[i])}" for i in np.argsort(-dom) if dom[i] > 0))
    for i, n in enumerate(DEF_NAMES):
        col = W[:, i]
        nz = col > 0
        cnt = (W[nz] > 0).sum(axis=1)
        serr = int((np.abs(W[nz].sum(axis=1) - 1.0) > 1e-4).sum())
        print(f"[s05] WEIGHTS bone {n}: verts={int(nz.sum())}, full(=1)={int((col >= 1 - 1e-6).sum())}, "
              f"max influences among its verts={int(cnt.max()) if nz.any() else 0}, sum errors={serr}")


def main():
    c = load_ctx()
    p = default_params()
    geo = limb_geometry(c)
    for x in SIDES:
        g = geo[x]
        print(f"[s05] limb {x}: arm tube verts={int((g['arm'] & c.body).sum())}, leg region verts="
              f"{int((g['leg'] & c.body).sum())}, L_upperarm={g['L_ua'] * 1000:.1f} mm, "
              f"L_lowerarm={g['L_la'] * 1000:.1f} mm")
        print(f"[s05] rings {x} (s from shoulder|elbow|wrist or hip|knee|ankle, mm): " + "; ".join(
            f"{r}=" + "/".join(f"{v * 1000:+.1f}" for v in s) for r, s in g["ring_s"].items()))
    tm = torso_masks(c)
    print(f"[s05] belt band z [{tm['zb0']:.4f}, {tm['zb1']:.4f}], head shell z min {tm['head_zmin']:.4f}, "
          f"body verts inside head shell={int(tm['head_in'].sum())}")
    H = heat_weights(c)
    W = build_weights(c, H, geo, tm, p)
    assign_groups(c, W)
    info, cdev = setup_rig(c)
    report(c, W)
    me = c.gm.data
    sk = [kb.name for kb in me.shape_keys.key_blocks] if me.shape_keys else []
    mouth_v = goblib.load_json("mouth.json").get("region", {}).get("output_verts", [])
    hw = W[mouth_v, BI["head"]] if mouth_v else np.zeros(0)
    print(f"[s05] HR2 GOB_mesh shape keys {sk} (relative {me.shape_keys.use_relative if me.shape_keys else None}); "
          f"materials {[m.name if m else None for m in me.materials]}; modifiers {[m.type for m in c.gm.modifiers]}")
    print(f"[s05] MOUTH verts (mouth.json region output_verts) {len(mouth_v)}: part head "
          f"{int((c.vpart[mouth_v] == c.parts['head']).sum()) if mouth_v else 0}, DEF head weight min "
          f"{float(hw.min()) if len(hw) else None}, max influences {int((W[mouth_v] > 0).sum(axis=1).max()) if mouth_v else None}")
    print(f"[s05] BELT bone = {RIGID_BONE['belt']} (Option A, 100 %)")
    for tw, d in info.items():
        print(f"[s05] TWIST {tw}: {d['constraint']} {d['from_rotation_mode']} from {d['source']}, axis {d['axis']}, "
              f"ratio {d['ratio']}, sign {d['sign']:+d}, mix_mode_rot {d['mix_mode_rot']}, "
              f"space {d['target_space']}/{d['owner_space']}")
    st = twist_selftest(c)
    print("[s05] TWIST selftest (twist bone axial deg rel. parent: source local Y +90 -> *_twist90 "
          "[expected sign*ratio*90]; diagonal (X+Z)/sqrt2 swing -> *_leak [expected 0]): "
          + ", ".join(f"{k}={v}" for k, v in st.items()))
    print(f"[s05] CLUB GOB_club parent={c.club.parent.name}/{c.club.parent_type}/{c.club.parent_bone}, "
          f"world matrix max abs diff={cdev:.3e}")
    reset_pose(c.arm)
    c.arm.data.pose_position = "POSE"
    bpy.context.view_layer.update()
    rdev = np.linalg.norm(eval_co(c.gm) - c.co, axis=1)
    print(f"[s05] REST deformation max={rdev.max() * 1000:.6f} mm")
    dec = {"belt_bone": RIGID_BONE["belt"], "belt_option": "A",
           "rigid": RIGID_BONE, "twist": info, "twist_selftest_deg": st,
           "weights": {"method": "body = bone-heat automatic weights; limb tubes (topological regions) replaced by "
                                 "per-joint axial profiles (hinge elbow/knee also by angle to the bend-inner side), "
                                 "wrist/ankle _1.._end tube = hand/foot 100 %, twist split along upper/lower arm, "
                                 "torso thigh limited to the hip profile, belt band -> pelvis, head dome -> head, "
                                 "rigid parts 100 %, limit 4, normalize",
                       "params": p, "INNER_DIR": INNER_DIR,
                       "MAX_INF": MAX_INF, "EPS_W": EPS_W, "zero_weight_bones": sorted(NON_DEFORM)}}
    out = goblib.save_json(OUT_JSON, dec)
    print(f"[s05] DECISIONS {out}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s05] OUTPUT {OUT_BLEND}")


if __name__ == "__main__":
    goblib.run_main(main)
