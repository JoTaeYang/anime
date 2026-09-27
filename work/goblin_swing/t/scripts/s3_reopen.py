"""Fresh-process reopen check of gobT_v03_skinned.blend."""
import bpy, json, math
import numpy as np
from mathutils import Vector
rig = bpy.data.objects["GOB_rig"]; arm = rig.data
R = {"file": bpy.data.filepath}
R["n_actions"] = len(bpy.data.actions)
R["rig_has_anim"] = bool(rig.animation_data and rig.animation_data.action)
posed = []
for pb in rig.pose.bones:
    ok = (pb.location.length < 1e-6 and (Vector(pb.scale) - Vector((1, 1, 1))).length < 1e-6)
    if pb.rotation_mode == 'QUATERNION':
        ok = ok and abs(pb.rotation_quaternion.w - 1) < 1e-6 and pb.rotation_quaternion.axis.length * abs(pb.rotation_quaternion.angle) < 1e-6
    else:
        ok = ok and Vector(pb.rotation_euler).length < 1e-6
    if not ok:
        posed.append(pb.name)
R["posed_bones"] = posed
R["n_bones"] = len(arm.bones)
R["deform_bones"] = sorted(b.name for b in arm.bones if b.use_deform)
R["twist_ok"] = all(n in arm.bones for n in ("FOREARM_TWIST_L", "FOREARM_TWIST_R"))
for nm in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[nm]
    R[nm] = {"parent": ob.parent.name if ob.parent else None,
             "mods": [(m.type, getattr(m, "object", None).name if getattr(m, "object", None) else
                       getattr(m, "vertex_group", "")) for m in ob.modifiers],
             "dq": next((m.use_deform_preserve_volume for m in ob.modifiers if m.type == 'ARMATURE'), None),
             "vgroups": len(ob.vertex_groups),
             "nondeform_groups": [g.name for g in ob.vertex_groups
                                  if g.name not in R["deform_bones"]],
             "hide_v": ob.hide_viewport, "hide_r": ob.hide_render}
R["lo_visible"] = not bpy.data.objects["GOB_body_lo"].hide_render
R["hi_hidden"] = bpy.data.objects["GOB_body"].hide_render or bpy.data.collections["GOB_hi"].hide_render
club = bpy.data.objects["GOB_club"]
R["club"] = {"parent": club.parent.name if club.parent else None,
             "parent_type": club.parent_type, "bone": club.parent_bone,
             "vgroups": len(club.vertex_groups), "mods": [m.type for m in club.modifiers]}
R["REF_CAM"] = "REF_CAM" in bpy.data.objects
cam = bpy.data.objects.get("REF_CAM")
if cam:
    R["refcam_matrix"] = [[round(float(v), 5) for v in r] for r in cam.matrix_world]
    R["refcam_lens"] = round(cam.data.lens, 3) if cam.data.type == 'PERSP' else cam.data.ortho_scale
# rest deviation of both meshes
for c in ("GOB_hi", "GOB_lo"):
    bpy.data.collections[c].hide_viewport = False
for nm in ("GOB_body_lo", "GOB_body"):
    bpy.data.objects[nm].hide_viewport = False
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for nm in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[nm]
    P = np.empty(len(ob.data.vertices) * 3); ob.data.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
    ev = ob.evaluated_get(dg); dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    R[nm]["rest_max_dev_mm"] = round(float(np.linalg.norm(Q - P, axis=1).max()) * 1000, 6)
# spot-check pose: it must actually deform
pb = rig.pose.bones["UPPERARM_FK_R"]
pb.rotation_euler = [math.radians(v) for v in (-52.15, 100.89, 32.7)]
rig.update_tag(); bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ob = bpy.data.objects["GOB_body"]
P = np.empty(len(ob.data.vertices) * 3); ob.data.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
ev = ob.evaluated_get(dg); dm = ev.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
ev.to_mesh_clear()
d = np.linalg.norm(Q - P, axis=1)
R["spotcheck_windup"] = {"max_move_mm": round(float(d.max()) * 1000, 1),
                         "n_moved_over_1mm": int((d > 0.001).sum()),
                         "torso_egg_max_move_mm": round(float(
                             d[(np.abs(P[:, 0]) < 0.385) & (P[:, 2] > 0.45) & (P[:, 2] < 1.30)].max()) * 1000, 3)}
print("@@@J@@@"); print(json.dumps(R, indent=1, default=str)); print("@@@E@@@")
