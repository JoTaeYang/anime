"""Diff GOB_body's vertex weights in v05 against v04, group by group.

blender -b goblin_v05_legfix.blend --python step03r_weightdiff.py
"""
import bpy, json, os
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
V04 = os.path.join(ROOT, "goblin_v04_skinned.blend")
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
LEGBONES = {"HIPS", "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"}


def grab(ob):
    me = ob.data
    N = len(me.vertices)
    W = np.zeros((N, len(BONES)))
    gi = {g.index: g.name for g in ob.vertex_groups}
    for v in me.vertices:
        for g in v.groups:
            n = gi.get(g.group)
            if n in BI:
                W[v.index, BI[n]] = g.weight
    P = np.empty(N * 3); me.vertices.foreach_get("co", P)
    return W, P.reshape(N, 3)


W5, P5 = grab(bpy.data.objects["GOB_body"])

with bpy.data.libraries.load(V04, link=False) as (src, dst):
    dst.objects = ["GOB_body"]
old = dst.objects[0]
bpy.context.scene.collection.objects.link(old)
W4, P4 = grab(old)

R = {"n_verts_v04": len(P4), "n_verts_v05": len(P5),
     "vert_coords_identical": bool(np.abs(P4 - P5).max() < 1e-9)}
d = np.abs(W5 - W4)
R["max_diff_per_group"] = {n: float(d[:, BI[n]].max()) for n in BONES}
R["max_diff_non_leg_groups"] = float(max(d[:, BI[n]].max() for n in BONES
                                         if n not in LEGBONES))
Z = P4[:, 2]
R["max_diff_any_group_above_hips_z0.55"] = float(d[Z > 0.55].max())
R["n_verts_changed_over_1e-6"] = int((d.max(1) > 1e-6).sum())
R["changed_z_range"] = [round(float(Z[d.max(1) > 1e-6].min()), 4),
                        round(float(Z[d.max(1) > 1e-6].max()), 4)]
print("@@@JSON_START@@@")
print(json.dumps(R, indent=1))
print("@@@JSON_END@@@")
