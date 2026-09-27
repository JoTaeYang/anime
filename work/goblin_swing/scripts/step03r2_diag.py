"""STEP 03R2 diagnosis - what actually moves when the right arm goes up?

Run: blender -b goblin_v05_legfix.blend --python step03r2_diag.py
NEVER saves.
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
OUT = os.path.join(ROOT, "inspect", "step03r2")
os.makedirs(OUT, exist_ok=True)

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

R = {}


def getW(ob):
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
    return P, W


def wrot(bone, axis, deg):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def deformed(ob):
    rig.update_tag(); bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return Q.reshape(-1, 3)


for obname in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[obname]
    P, W = getW(ob)
    N = len(P)
    r = {"verts": N}
    for S in ("R", "L"):
        sh = np.array(arm.bones["UPPERARM_" + S].head_local)
        tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
        u = tl - sh; u /= np.linalg.norm(u)
        v = P - sh
        s = v @ u
        d = np.linalg.norm(v - s[:, None] * u, axis=1)
        aw = W[:, BI["UPPERARM_" + S]]
        rr = {"shoulder_head": [round(float(x), 4) for x in sh],
              "upperarm_dir": [round(float(x), 4) for x in u],
              "upperarm_len": round(float(np.linalg.norm(tl - sh)), 4)}
        # geometry profile: surface radius vs s
        prof = []
        for lo_ in np.arange(-0.02, 0.22, 0.01):
            m = (s >= lo_) & (s < lo_ + 0.01) & (d < 0.30)
            if m.sum() >= 3:
                prof.append([round(float(lo_) + 0.005, 3), int(m.sum()),
                             round(float(np.percentile(d[m], 90)), 4),
                             round(float(d[m].max()), 4),
                             round(float(aw[m].max()), 3),
                             round(float(aw[m].mean()), 3)])
        rr["profile_s_d90_dmax_awmax_awmean"] = prof
        # where does the arm weight live?
        m = aw > 0.02
        rr["n_aw_gt_002"] = int(m.sum())
        if m.any():
            rr["aw_s_range"] = [round(float(s[m].min()), 4), round(float(s[m].max()), 4)]
            rr["aw_d_max"] = round(float(d[m].max()), 4)
            # arm weight sitting on verts far from the arm axis = leak onto torso
            for dd in (0.09, 0.11, 0.14, 0.18):
                mm = m & (d > dd)
                rr["leak_d_gt_%.2f" % dd] = [int(mm.sum()),
                                             round(float(aw[mm].max()), 3) if mm.any() else 0.0]
            for ss in (0.0, 0.02, 0.04, 0.06):
                mm = (aw > 0.02) & (s < ss)
                rr["inboard_s_lt_%.2f" % ss] = [int(mm.sum()),
                                                round(float(aw[mm].max()), 3) if mm.any() else 0.0]
        r[S] = rr

    # --- T2 pose: right arm up 90; torso must not move at all
    clear_pose()
    wrot("UPPERARM_FK_R", (0, 1, 0), 90)
    Q = deformed(ob)
    disp = np.linalg.norm(Q - P, axis=1)
    sh = np.array(arm.bones["UPPERARM_R"].head_local)
    tl = np.array(arm.bones["UPPERARM_R"].tail_local)
    u = tl - sh; u /= np.linalg.norm(u)
    vv = P - sh
    s = vv @ u
    d = np.linalg.norm(vv - s[:, None] * u, axis=1)
    armw = W[:, BI["UPPERARM_R"]] + W[:, BI["FOREARM_R"]] + W[:, BI["HAND_R"]] + W[:, BI["WEAPON"]]
    torso = (armw < 1e-6)
    r["T2_armR_up90"] = {
        "max_disp_all": round(float(disp.max()), 4),
        "n_torso_verts(no arm weight)": int(torso.sum()),
        "torso_max_disp": round(float(disp[torso].max()), 5),
        "moved_verts_gt_5mm": int((disp > 0.005).sum()),
        "moved_verts_gt_5mm_with_d_gt_0.10": int(((disp > 0.005) & (d > 0.10)).sum()),
        "moved_verts_gt_20mm_with_d_gt_0.10": int(((disp > 0.020) & (d > 0.10)).sum()),
    }
    # displacement by distance-from-arm-axis band
    tab = []
    for lo_ in (0.06, 0.09, 0.12, 0.16, 0.20, 0.26, 0.34, 0.45):
        m = (d >= lo_) & (d < lo_ * 1.45)
        if m.sum():
            tab.append([lo_, int(m.sum()), round(float(disp[m].max()), 4),
                        round(float(np.percentile(disp[m], 99)), 4),
                        int((disp[m] > 0.005).sum())])
    r["T2_disp_by_d"] = tab
    # z-extent of dragged verts
    mv = disp > 0.010
    if mv.any():
        r["T2_moved10mm_bbox"] = [[round(float(x), 3) for x in P[mv].min(0)],
                                  [round(float(x), 3) for x in P[mv].max(0)]]
        r["T2_moved10mm_n"] = int(mv.sum())
    clear_pose()
    R[obname] = r

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2_diag.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
