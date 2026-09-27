"""Where does the rest-pose deviation come from?  (read-only)
Run: blender -b goblin_v04_skinned.blend --python step03_restcheck.py -- [tag]
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Matrix

tag = (sys.argv[sys.argv.index("--") + 1:] or ["dq"])[0]
rig = bpy.data.objects["GOB_rig"]
body = bpy.data.objects["GOB_body"]
me = body.data
N = len(me.vertices)
P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)

R = {"tag": tag}
md = body.modifiers[0]
R["modifier"] = {"preserve_volume": md.use_deform_preserve_volume,
                 "envelopes": md.use_bone_envelopes,
                 "vgroups": md.use_vertex_groups}
R["body_matrix_world"] = [[round(c, 9) for c in r] for r in body.matrix_world]
R["body_matrix_basis"] = [[round(c, 9) for c in r] for r in body.matrix_basis]
R["body_parent_inverse"] = [[round(c, 9) for c in r] for r in body.matrix_parent_inverse]

# double-precision check: pose_matrix @ rest_matrix^-1 must be identity at rest
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
rev = rig.evaluated_get(dg)
worst = 0.0
per = {}
for b in rig.data.bones:
    if not b.use_deform:
        continue
    D = rev.pose.bones[b.name].matrix @ b.matrix_local.inverted()
    e = max(abs(D[i][j] - (1.0 if i == j else 0.0)) for i in range(4) for j in range(4))
    per[b.name] = float(e)
    worst = max(worst, e)
R["delta_matrix_identity_err_python_double"] = {"max": worst, "per_bone": per}

ev = body.evaluated_get(dg)
dm = ev.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
dev = np.linalg.norm(Q - P, axis=1)
ev.to_mesh_clear()

gi = {g.index: g.name for g in body.vertex_groups}
dom = []
for v in me.vertices:
    best, bw = "", -1.0
    for g in v.groups:
        if g.weight > bw:
            bw, best = g.weight, gi[g.group]
    dom.append(best)
dom = np.array(dom)
R["max_dev"] = float(dev.max())
R["dev_per_dominant_bone"] = {n: round(float(dev[dom == n].max()), 9)
                              for n in sorted(set(dom.tolist()))}
R["bone_head_dist_from_origin"] = {b.name: round(float(b.head_local.length), 4)
                                   for b in rig.data.bones if b.use_deform}
# does the deviation scale with |vertex| ?  (float32 signature)
m = dev > 1e-6
if m.any():
    R["dev_over_coord_norm"] = {
        "max_ratio": float((dev[m] / np.linalg.norm(P[m], axis=1)).max()),
        "median_ratio": float(np.median(dev[m] / np.linalg.norm(P[m], axis=1)))}
R["dev_hist"] = {"gt_1e-5": int((dev > 1e-5).sum()), "gt_1e-4": int((dev > 1e-4).sum()),
                 "gt_1e-3": int((dev > 1e-3).sum()), "mean": float(dev.mean())}

# control: same mesh, modifier disabled
md.show_viewport = False
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ev = body.evaluated_get(dg)
dm = ev.to_mesh()
Q2 = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q2)
R["dev_with_modifier_disabled"] = float(np.linalg.norm(Q2.reshape(-1, 3) - P, axis=1).max())
ev.to_mesh_clear()
md.show_viewport = True

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
