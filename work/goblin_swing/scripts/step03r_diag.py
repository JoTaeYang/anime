"""Diagnose which vertices crumple at the leg/torso junction in a given pose."""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MESH = argv[argv.index("--mesh") + 1] if "--mesh" in argv else "hi"

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
body = bpy.data.objects["GOB_body_lo" if MESH == "lo" else "GOB_body"]
body.hide_viewport = False
for c in body.users_collection:
    c.hide_viewport = False
me = body.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
gi = {g.index: g.name for g in body.vertex_groups}
W = np.zeros((N, len(BONES)))
for v in me.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in BI:
            W[v.index, BI[n]] = g.weight


def wloc(b, d):
    PB[b].location = REST[b].to_3x3().inverted() @ Vector(d)


def wrot(b, ax, deg, add=False):
    M = REST[b].to_3x3()
    q = Quaternion(Vector(ax).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    cur = PB[b].rotation_euler.to_quaternion() if add else Quaternion()
    PB[b].rotation_euler = (cur @ ql).to_euler(PB[b].rotation_mode)


wloc("FOOT_IK_R", (-0.15, 0, 0))
wloc("COG_CTRL", (-0.05, 0, -0.15))
wrot("CHEST_CTRL", (1, 0, 0), 25)
wrot("CHEST_CTRL", (0, 0, 1), 30, True)
rig.update_tag(); bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ev = rig.evaluated_get(dg)
evb = body.evaluated_get(dg)
dm = evb.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
evb.to_mesh_clear()

OUTR = {}
for S in ("L", "R"):
    A = np.array(arm.bones["FOOT_" + S].head_local)
    H = np.array(arm.bones["THIGH_" + S].head_local)
    L = float(np.linalg.norm(H - A)); u = (H - A) / L
    V = P0 - A
    s = V @ u
    r = np.linalg.norm(V - s[:, None] * u, axis=1)
    legw = W[:, BI["THIGH_" + S]] + W[:, BI["SHIN_" + S]]
    m = (legw > 1e-6) & (W[:, BI["HIPS"]] > 1e-6)     # the blend band
    # where would they be if they were 100% rigid leg?
    Mth = np.array(ev.pose.bones["THIGH_" + S].matrix @ REST["THIGH_" + S].inverted())
    rigid = (np.c_[P0, np.ones(N)] @ Mth.T)[:, :3]
    Mhp = np.array(ev.pose.bones["HIPS"].matrix @ REST["HIPS"].inverted())
    hips = (np.c_[P0, np.ones(N)] @ Mhp.T)[:, :3]
    dev_leg = np.linalg.norm(Q - rigid, axis=1)
    dev_hip = np.linalg.norm(Q - hips, axis=1)
    idx = np.nonzero(m)[0]
    ordr = idx[np.argsort(-dev_leg[idx])]
    rows = []
    for i in ordr[:12]:
        rows.append({"v": int(i), "rest": [round(float(x), 4) for x in P0[i]],
                     "s": round(float(s[i]), 4), "r": round(float(r[i]), 4),
                     "legw": round(float(legw[i]), 4),
                     "hipsw": round(float(W[i, BI["HIPS"]]), 4),
                     "footw": round(float(W[i, BI["FOOT_" + S]]), 4),
                     "posed_z": round(float(Q[i][2]), 4),
                     "dev_from_rigid_leg": round(float(dev_leg[i]), 4),
                     "dev_from_rigid_hips": round(float(dev_hip[i]), 4)})
    OUTR[S] = {"band_n": int(m.sum()),
               "band_s_range": [round(float(s[m].min()), 4), round(float(s[m].max()), 4)],
               "band_r_range": [round(float(r[m].min()), 4), round(float(r[m].max()), 4)],
               "band_z_range": [round(float(P0[m][:, 2].min()), 4),
                                round(float(P0[m][:, 2].max()), 4)],
               "posed_z_range": [round(float(Q[m][:, 2].min()), 4),
                                 round(float(Q[m][:, 2].max()), 4)],
               "torso_bottom_posed_z": round(float(Q[W[:, BI["HIPS"]] > 0.9995][:, 2].min()), 4),
               "legtop_posed_z": round(float(ev.pose.bones["THIGH_" + S].matrix.to_translation().z), 4),
               "worst": rows}
    # pure-leg tube verts and their posed z span
    pure = legw > 0.9995
    OUTR[S]["pure_tube_n"] = int(pure.sum())
    OUTR[S]["pure_tube_posed_z"] = [round(float(Q[pure][:, 2].min()), 4),
                                    round(float(Q[pure][:, 2].max()), 4)]

print("@@@JSON_START@@@")
print(json.dumps(OUTR, indent=1))
print("@@@JSON_END@@@")
