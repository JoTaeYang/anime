"""STEP 03R2 - fresh-process verification of goblin_v05b_shoulderfix.blend."""
import bpy, json, os
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
R = {"file": bpy.data.filepath}
rig = bpy.data.objects["GOB_rig"]
arm = rig.data

R["objects"] = sorted(o.name for o in bpy.data.objects)
R["n_actions"] = len(bpy.data.actions)
R["rig_has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["ref_cam"] = "REF_CAM" in bpy.data.objects
if R["ref_cam"]:
    c = bpy.data.objects["REF_CAM"]
    R["ref_cam_state"] = {"loc": [round(v, 5) for v in c.location],
                          "rot": [round(v, 5) for v in c.rotation_euler],
                          "lens": round(c.data.lens, 4) if c.data.type == 'PERSP' else None,
                          "type": c.data.type,
                          "is_scene_camera": bpy.context.scene.camera == c}
R["bones"] = len(arm.bones)
R["deform_bones"] = sorted(b.name for b in arm.bones if b.use_deform)
R["bone_constraints"] = {pb.name: [c.type for c in pb.constraints]
                         for pb in rig.pose.bones if pb.constraints}
R["posed_bones"] = [pb.name for pb in rig.pose.bones
                    if pb.location.length > 1e-9
                    or abs(pb.rotation_quaternion.w - 1) > 1e-9
                    or np.linalg.norm(pb.rotation_quaternion[1:]) > 1e-9
                    or np.linalg.norm(pb.rotation_euler[:]) > 1e-9
                    or abs(pb.scale[0] - 1) > 1e-9]
R["collections"] = {c.name: {"objs": sorted(o.name for o in c.objects),
                             "hide_viewport": c.hide_viewport,
                             "hide_render": c.hide_render}
                    for c in bpy.data.collections}
for n in ("GOB_body", "GOB_body_lo"):
    o = bpy.data.objects[n]
    R[n] = {"verts": len(o.data.vertices), "hide_viewport": o.hide_viewport,
            "hide_render": o.hide_render, "parent": o.parent.name if o.parent else None,
            "groups": len(o.vertex_groups),
            "group_names_ok": sorted(g.name for g in o.vertex_groups) == sorted(BONES),
            "modifiers": [(m.type, m.object.name, m.use_deform_preserve_volume)
                          for m in o.modifiers if m.type == 'ARMATURE']}
    me = o.data
    N = len(me.vertices)
    P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
    gi = {g.index: g.name for g in o.vertex_groups}
    W = np.zeros((N, NB))
    for v in me.vertices:
        for g in v.groups:
            nm = gi.get(g.group)
            if nm in BI:
                W[v.index, BI[nm]] = g.weight
    R[n]["max_influences"] = int((W > 0).sum(1).max())
    R[n]["max_sum_error"] = float(np.abs(W.sum(1) - 1.0).max())
    R[n]["zero_weight_verts"] = int((W.sum(1) < 1e-9).sum())
    np.save(os.path.join(ROOT, "inspect", "step03r2_W_%s.npy" % n), W)
    # rest deformation
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(dg)
    try:
        dm = ev.to_mesh()
        Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
        ev.to_mesh_clear()
        R[n]["rest_max_dev_mm"] = round(float(
            np.linalg.norm(Q.reshape(-1, 3) - P, axis=1).max()) * 1000, 6)
    except Exception as e:
        R[n]["rest_eval"] = "hidden: " + str(e)
print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2_reopen.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
