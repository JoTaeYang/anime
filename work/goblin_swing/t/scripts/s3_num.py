"""Numeric diagnosis: weight profile along the right arm + posed tube cross-sections."""
import bpy, math, os, json, sys
import numpy as np
from mathutils import Vector

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
rig = bpy.data.objects["GOB_rig"]; arm = rig.data; PB = rig.pose.bones
lo = bpy.data.objects["GOB_body_lo"]
me = lo.data
N = len(me.vertices)
P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
DEF = [b.name for b in arm.bones if b.use_deform]
DI = {n: i for i, n in enumerate(DEF)}
gi = {g.index: g.name for g in lo.vertex_groups}
W = np.zeros((N, len(DEF)))
for v in me.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in DI:
            W[v.index, DI[n]] = g.weight

CH = [np.array(arm.bones[b].head_local) for b in ("UPPERARM_R", "FOREARM_R", "HAND_R")]
CH.append(np.array(arm.bones["HAND_R"].tail_local))


def poly_td(Q, pts):
    pts = np.asarray(pts, float); A, B = pts[:-1], pts[1:]
    AB = B - A; L2 = (AB ** 2).sum(1); L = np.sqrt(L2)
    cum = np.concatenate([[0.0], np.cumsum(L)])
    bd = np.full(len(Q), 1e9); bt = np.zeros(len(Q))
    for k in range(len(A)):
        w = Q - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0, 1)
        d = np.linalg.norm(w - tt[:, None] * AB[k], axis=1)
        m = d < bd; bd[m] = d[m]; bt[m] = cum[k] + tt[m] * L[k]
    return bt, bd


s, d = poly_td(P, CH)
ARMM = (P[:, 0] < -0.385) & (d < 0.13)
R = {"arm_verts_tube": int(ARMM.sum())}
rows = []
for lo_ in np.arange(0.00, 0.56, 0.02):
    m = ARMM & (s >= lo_) & (s < lo_ + 0.02)
    if m.sum() >= 2:
        rows.append([round(float(lo_) + 0.01, 3), int(m.sum()),
                     round(float(np.median(d[m])), 4)] +
                    [round(float(W[m, DI[b]].mean()), 3) for b in
                     ("CHEST", "UPPERARM_R", "FOREARM_R", "FOREARM_TWIST_R", "HAND_R")])
R["profile_s_n_r_CHEST_UA_FA_TW_HA"] = rows

# ring structure: how many distinct s-rings on the tube
srt = np.sort(s[ARMM])
R["tube_s_min_max"] = [round(float(srt[0]), 4), round(float(srt[-1]), 4)]
R["tube_gaps_over_15mm"] = int((np.diff(srt) > 0.015).sum())


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0


def upd():
    rig.update_tag(); lo.update_tag(); bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def posed(dg):
    ev = lo.evaluated_get(dg); dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    Q = Q.reshape(-1, 3); ev.to_mesh_clear()
    return Q


CASES = {"rest": {},
         "u_twist100": {"UPPERARM_FK_R": (0, 100, 0)},
         "u_windup": {"UPPERARM_FK_R": (-52.15, 100.89, 32.7)},
         "f_flex128": {"FOREARM_FK_R": (127.7, 0, 0)},
         "h_pron135": {"HAND_FK_R": (0, 135, 0)}}
for k, dd in CASES.items():
    clear()
    for b, e in dd.items():
        PB[b].rotation_euler = [math.radians(v) for v in e]
    dg = upd()
    Q = posed(dg)
    ev = rig.evaluated_get(dg)
    # posed bone chain -> new arclength frame
    ch = [np.array(ev.pose.bones[b].head) for b in ("UPPERARM_R", "FOREARM_R", "HAND_R")]
    ch.append(np.array(ev.pose.bones["HAND_R"].matrix @ Vector((0, arm.bones["HAND_R"].length, 0))))
    sp, dp = poly_td(Q, ch)
    out = []
    for lo_ in np.arange(0.00, 0.56, 0.03):
        m = ARMM & (s >= lo_) & (s < lo_ + 0.03)
        if m.sum() >= 5:
            # cross-section: for the SAME rest ring, measure the posed spread
            pts = Q[m]
            c = pts.mean(0)
            v = pts - c
            u, sv, vt = np.linalg.svd(v, full_matrices=False)
            out.append([round(float(lo_) + 0.015, 3), int(m.sum()),
                        round(float(sv[1] / max(sv[0], 1e-9)), 3),
                        round(float(sv[2] / max(sv[0], 1e-9)), 3),
                        round(float(np.median(dp[m])), 4)])
    R[k + "_ringSVD_s_n_r21_r31_r"] = out
clear()
print("@@@JSON_START@@@"); print(json.dumps(R, default=str)); print("@@@JSON_END@@@")
