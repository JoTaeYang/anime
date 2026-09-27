"""What exactly is the remaining flap in T2 made of?"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MESH = argv[argv.index("--mesh") + 1] if "--mesh" in argv else "GOB_body_lo"
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
ob = bpy.data.objects[MESH]
me = ob.data
n = len(me.vertices)
P = np.empty(n * 3); me.vertices.foreach_get("co", P); P = P.reshape(n, 3)
E = np.empty(len(me.edges) * 2, dtype=np.int32); me.edges.foreach_get("vertices", E); E = E.reshape(-1, 2)
gi = {g.index: g.name for g in ob.vertex_groups}
W = np.zeros((n, NB))
for v in me.vertices:
    for g in v.groups:
        nm = gi.get(g.group)
        if nm in BI:
            W[v.index, BI[nm]] = g.weight
M = REST["UPPERARM_FK_R"].to_3x3()
q = Quaternion(Vector((0, 1, 0)).normalized(), math.radians(90))
ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
pb = PB["UPPERARM_FK_R"]
if pb.rotation_mode == 'QUATERNION':
    pb.rotation_quaternion = ql
else:
    pb.rotation_euler = ql.to_euler(pb.rotation_mode)
rig.update_tag(); bpy.context.view_layer.update()
ev = ob.evaluated_get(bpy.context.evaluated_depsgraph_get())
dm = ev.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
ev.to_mesh_clear()
disp = np.linalg.norm(Q - P, axis=1)
rT = np.hypot(P[:, 0], P[:, 1] - 0.088)
aw = W[:, BI["UPPERARM_R"]]
out = {"mesh": MESH}
# smoothness: max weight jump across an edge, in the shoulder neighbourhood
sh = np.array(arm.bones["UPPERARM_R"].head_local)
dsh = np.linalg.norm(P - sh, axis=1)
loc = (dsh[E[:, 0]] < 0.30) | (dsh[E[:, 1]] < 0.30)
jump = np.abs(aw[E[:, 0]] - aw[E[:, 1]])
out["shoulder_edge_weight_jump"] = {
    "max": round(float(jump[loc].max()), 3),
    "p99": round(float(np.percentile(jump[loc], 99)), 3),
    "n_gt_0.5": int((jump[loc] > 0.5).sum()), "n_edges": int(loc.sum())}
# edge stretch in the shoulder region
L0 = np.linalg.norm(P[E[:, 0]] - P[E[:, 1]], axis=1)
L1 = np.linalg.norm(Q[E[:, 0]] - Q[E[:, 1]], axis=1)
ratio = L1 / np.maximum(L0, 1e-9)
out["shoulder_edge_stretch"] = {"max": round(float(ratio[loc].max()), 3),
                                "p99": round(float(np.percentile(ratio[loc], 99)), 3),
                                "min": round(float(ratio[loc].min()), 3)}
# rest-position profile of the verts that move a lot
for lo_, hi_ in ((0.03, 0.08), (0.08, 0.15), (0.15, 10.0)):
    m = (disp >= lo_) & (disp < hi_)
    if m.sum():
        out["moved_%.2f_%.2f" % (lo_, hi_)] = {
            "n": int(m.sum()),
            "rest_z": [round(float(P[m][:, 2].min()), 3), round(float(P[m][:, 2].max()), 3)],
            "rest_rT": [round(float(rT[m].min()), 3), round(float(rT[m].max()), 3)],
            "armw": [round(float(aw[m].min()), 3), round(float(aw[m].max()), 3)],
            "dsh": [round(float(dsh[m].min()), 3), round(float(dsh[m].max()), 3)]}
# how far out does the arm-weighted region reach on the torso surface at rest?
m = aw > 0.5
out["armw_gt_0.5"] = {"n": int(m.sum()),
                      "rest_z": [round(float(P[m][:, 2].min()), 3), round(float(P[m][:, 2].max()), 3)],
                      "rest_rT_max": round(float(rT[m].max()), 3),
                      "dsh_max": round(float(dsh[m].max()), 3)}
print("@@@JSON_START@@@"); print(json.dumps(out, indent=1, default=str)); print("@@@JSON_END@@@")
