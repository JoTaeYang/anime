"""Which vertices make the flank bulge in the T2 arm-up pose?"""
import bpy, json, math, os
import numpy as np
from mathutils import Vector, Quaternion

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}


def wrot(bone, axis, deg):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


ob = bpy.data.objects["GOB_body_lo"]
me = ob.data
n = len(me.vertices)
P = np.empty(n * 3); me.vertices.foreach_get("co", P); P = P.reshape(n, 3)
gi = {g.index: g.name for g in ob.vertex_groups}
W = np.zeros((n, NB))
for v in me.vertices:
    for g in v.groups:
        nm = gi.get(g.group)
        if nm in BI:
            W[v.index, BI[nm]] = g.weight

wrot("UPPERARM_FK_R", (0, 1, 0), 90)
rig.update_tag(); bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ev = ob.evaluated_get(dg)
dm = ev.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
ev.to_mesh_clear()
disp = np.linalg.norm(Q - P, axis=1)

sh = np.array(arm.bones["UPPERARM_R"].head_local)
tl = np.array(arm.bones["UPPERARM_R"].tail_local)
u = tl - sh; u /= np.linalg.norm(u)
v = P - sh
s = v @ u
d = np.linalg.norm(v - s[:, None] * u, axis=1)
dsh = np.linalg.norm(v, axis=1)

# torso flank: not the tube, not the club/hand, inside the egg
rT = np.hypot(P[:, 0], P[:, 1] - 0.088)
flank = (d > 0.14) & (P[:, 2] < 1.30) & (P[:, 2] > 0.35) & (rT < 0.50)
out = {"n_flank": int(flank.sum()),
       "flank_max_disp": round(float(disp[flank].max()), 4),
       "flank_p99_disp": round(float(np.percentile(disp[flank], 99)), 4),
       "flank_n_gt_5mm": int((disp[flank] > 0.005).sum()),
       "flank_n_gt_20mm": int((disp[flank] > 0.020).sum())}
mv = flank & (disp > 0.010)
out["moving_flank_n"] = int(mv.sum())
if mv.any():
    out["moving_flank_bbox"] = [[round(float(x), 3) for x in P[mv].min(0)],
                                [round(float(x), 3) for x in P[mv].max(0)]]
    idx = np.nonzero(mv)[0]
    order = np.argsort(-disp[idx])[:25]
    out["worst"] = []
    for i in idx[order]:
        out["worst"].append({
            "co": [round(float(x), 3) for x in P[i]],
            "disp_mm": round(float(disp[i]) * 1000, 1),
            "s": round(float(s[i]), 3), "d": round(float(d[i]), 3),
            "dsh": round(float(dsh[i]), 3),
            "w": {BONES[j]: round(float(W[i, j]), 3) for j in range(NB) if W[i, j] > 1e-4}})
    # how much upper-arm weight do the moving flank verts have?
    aw = W[:, BI["UPPERARM_R"]]
    out["moving_flank_armw"] = {"min": round(float(aw[mv].min()), 4),
                                "max": round(float(aw[mv].max()), 4),
                                "mean": round(float(aw[mv].mean()), 4),
                                "n_zero": int((aw[mv] < 1e-6).sum())}
print("@@@JSON_START@@@")
print(json.dumps(out, indent=1, default=str))
print("@@@JSON_END@@@")
