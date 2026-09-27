"""Which vertices land outside the egg+tube footprint, and what are they?"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MESH = argv[argv.index("--mesh") + 1] if "--mesh" in argv else "GOB_body_lo"
CELL = 0.002
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
SE = {"R": 0.020, "L": -0.030}
rig = bpy.data.objects["GOB_rig"]; arm = rig.data; PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
for c in ("GOB_hi", "GOB_lo"):
    cc = bpy.data.collections.get(c)
    if cc:
        cc.hide_viewport = False; cc.hide_render = False
for o in (bpy.data.objects["GOB_body"], bpy.data.objects["GOB_body_lo"]):
    o.hide_viewport = False; o.hide_render = False
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False; lc.hide_viewport = False
ob = bpy.data.objects[MESH]; me = ob.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)
gi = {g.index: g.name for g in ob.vertex_groups}
W = np.zeros((N, 18))
for v in me.vertices:
    for g in v.groups:
        nm = gi.get(g.group)
        if nm in BI:
            W[v.index, BI[nm]] = g.weight


def poly_d(pts):
    pts = np.asarray(pts, float); A, B = pts[:-1], pts[1:]; AB = B - A
    L2 = (AB ** 2).sum(1); bd = np.full(N, 1e9)
    for k in range(len(A)):
        w = P0 - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0, 1)
        bd = np.minimum(bd, np.linalg.norm(w - tt[:, None] * AB[k], axis=1))
    return bd


bh = lambda n: np.array(arm.bones[n].head_local)
bt = lambda n: np.array(arm.bones[n].tail_local)
CH = {S: poly_d([bh("UPPERARM_" + S), bh("FOREARM_" + S), bh("HAND_" + S), bt("HAND_" + S)])
      for S in ("R", "L")}
SC = {}
for S in ("R", "L"):
    u = bt("UPPERARM_" + S) - bh("UPPERARM_" + S); u /= np.linalg.norm(u)
    SC[S] = (P0 - bh("UPPERARM_" + S)) @ u
AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
V = P0 - np.array([-0.6661, -0.3447, 0.9910]); S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
CLUB = (S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > 0.1134) & (D_AX < 0.22)
HANDR = np.linalg.norm(P0 - np.array([-0.676, -0.364, 0.992]), axis=1) <= 0.155
HANDL = np.linalg.norm(P0 - np.array([0.772, 0.056, 0.720]), axis=1) <= 0.155
TUBEV = CLUB | HANDR | HANDL
for S in ("R", "L"):
    TUBEV |= (CH[S] < 0.10) & (SC[S] > SE[S])
EGGV = ~TUBEV
TRI_TUBE = np.nonzero(TUBEV[TRI].any(axis=1))[0]
TRI_EGG = np.nonzero(EGGV[TRI].all(axis=1))[0]
_BA = []
K = 7
for i in range(K + 1):
    for j in range(K + 1 - i):
        _BA.append((i / K, j / K, 1 - i / K - j / K))
_BA = np.array(_BA)


def fp(p2, tris, shape, org):
    g = np.zeros(shape, bool)
    if not len(tris):
        return g
    A, B, C = p2[TRI[tris, 0]], p2[TRI[tris, 1]], p2[TRI[tris, 2]]
    for w in _BA:
        Pt = w[0] * A + w[1] * B + w[2] * C
        ij = np.floor((Pt - org) / CELL).astype(np.int64)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[1])
        g[ij[ok, 0], ij[ok, 1]] = True
    return g


def dil(g, k):
    o = g.copy()
    for _ in range(k):
        t = o.copy()
        t[1:] |= o[:-1]; t[:-1] |= o[1:]; t[:, 1:] |= o[:, :-1]; t[:, :-1] |= o[:, 1:]
        o = t
    return o


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1); pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0); pb.rotation_axis_angle = (0, 0, 1, 0)


def wrot(b, ax, dg, add=False):
    M = REST[b].to_3x3(); q = Quaternion(Vector(ax).normalized(), math.radians(dg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion(); pb = PB[b]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ ql) if add else ql
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ ql).to_euler(pb.rotation_mode)


out = {}
for lbl, build in (("T2_armR_up90", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 90)),
                   ("armR_up120", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 120))):
    clear(); build()
    rig.update_tag(); bpy.context.view_layer.update()
    ev = ob.evaluated_get(bpy.context.evaluated_depsgraph_get()); dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    p0 = np.stack([P0[:, 0], P0[:, 2]], 1); pq = np.stack([Q[:, 0], Q[:, 2]], 1)
    allp = np.concatenate([p0, pq]); org = allp.min(0) - 0.02
    shape = tuple(np.ceil((allp.max(0) + 0.02 - org) / CELL).astype(int))
    ideal = dil(fp(p0, TRI_EGG, shape, org) | fp(pq, TRI_TUBE, shape, org), 1)
    # which POSED vertices sit outside the ideal footprint?
    ij = np.floor((pq - org) / CELL).astype(np.int64)
    ij[:, 0] = np.clip(ij[:, 0], 0, shape[0] - 1); ij[:, 1] = np.clip(ij[:, 1], 0, shape[1] - 1)
    outside = ~ideal[ij[:, 0], ij[:, 1]]
    # distance of each outside vertex from the ideal footprint, in cells
    grown = ideal.copy(); dist = np.full(N, 0)
    for k in range(1, 41):
        grown = dil(grown, 1)
        still = outside & (~grown[ij[:, 0], ij[:, 1]])
        dist[still] = k
    dist[outside & (dist == 0)] = 1
    m = outside & (dist >= 3)
    r = {"n_outside": int(outside.sum()), "n_outside_ge6mm": int(m.sum()),
         "max_dist_mm": round(float(dist.max() * CELL * 1000), 1)}
    if m.any():
        r["rest_bbox"] = [[round(float(x), 3) for x in P0[m].min(0)],
                          [round(float(x), 3) for x in P0[m].max(0)]]
        r["chainR"] = [round(float(CH['R'][m].min()), 3), round(float(CH['R'][m].max()), 3)]
        r["is_tubev"] = int(TUBEV[m].sum())
        r["is_club_hand"] = int((CLUB | HANDR | HANDL)[m].sum())
        idx = np.nonzero(m)[0][np.argsort(-dist[np.nonzero(m)[0]])][:14]
        r["worst"] = [{"rest": [round(float(x), 3) for x in P0[i]],
                       "posed": [round(float(x), 3) for x in Q[i]],
                       "out_mm": int(dist[i] * CELL * 1000),
                       "chainR": round(float(CH['R'][i]), 3),
                       "tubev": bool(TUBEV[i]),
                       "w": {BONES[j]: round(float(W[i, j]), 3) for j in range(18) if W[i, j] > 1e-3}}
                      for i in idx]
    out[lbl] = r
    clear()
print("@@@JSON_START@@@"); print(json.dumps(out, indent=1, default=str)); print("@@@JSON_END@@@")
