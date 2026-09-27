"""Fresh-process re-open check of goblin_v05_legfix.blend."""
import bpy, json, math, os
import numpy as np
from mathutils import Vector

R = {"file": bpy.data.filepath}
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones

R["objects"] = sorted([(o.name, o.type) for o in bpy.data.objects])
R["cameras"] = [o.name for o in bpy.data.objects if o.type == 'CAMERA']
R["scene_camera"] = sc.camera.name if sc.camera else None
R["n_actions"] = len(bpy.data.actions)
R["has_animdata_action"] = bool(rig.animation_data and rig.animation_data.action)
R["n_drivers"] = len(rig.animation_data.drivers) if rig.animation_data else 0
R["frame_range"] = [sc.frame_start, sc.frame_end, sc.render.fps]
R["collections"] = {c.name: {"objs": sorted(o.name for o in c.objects),
                             "hide_viewport": c.hide_viewport,
                             "hide_render": c.hide_render}
                    for c in bpy.data.collections}
R["mesh_visibility"] = {o.name: {"hide_viewport": o.hide_viewport,
                                 "hide_render": o.hide_render,
                                 "verts": len(o.data.vertices),
                                 "modifiers": [(m.name, m.type,
                                                m.use_deform_preserve_volume)
                                               for m in o.modifiers],
                                 "parent": o.parent.name if o.parent else None,
                                 "groups": len(o.vertex_groups)}
                        for o in bpy.data.objects if o.name.startswith("GOB_body")}
R["residual_posed_bones"] = [pb.name for pb in PB
                             if pb.location.length > 1e-9 or
                             (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9 or
                             Vector(pb.rotation_euler).length > 1e-9 or
                             abs(pb.rotation_quaternion.w - 1) > 1e-9 or
                             Vector(pb.rotation_quaternion[1:]).length > 1e-9]
CTRL = ["ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL",
        "UPPERARM_FK_L", "FOREARM_FK_L", "HAND_FK_L",
        "UPPERARM_FK_R", "FOREARM_FK_R", "HAND_FK_R",
        "HAND_IK_L", "HAND_IK_R", "ELBOW_POLE_L", "ELBOW_POLE_R",
        "FOOT_IK_L", "FOOT_IK_R", "KNEE_POLE_L", "KNEE_POLE_R", "WEAPON_CTRL"]
R["ctrl_collection"] = sorted(b.name for c in arm.collections if c.name == "CTRL"
                              for b in c.bones)
R["ctrl_list_unchanged"] = sorted(CTRL) == R["ctrl_collection"]
R["custom_props"] = {pb.name: {k: pb[k] for k in pb.keys()}
                     for pb in PB if list(pb.keys())}
R["invalid_constraints"] = [(pb.name, c.name) for pb in PB for c in pb.constraints
                            if not c.is_valid]
R["def_bones_constraint_driven"] = {
    n: [c.type for c in PB[n].constraints]
    for n in ("THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R",
              "MCH_LEGTOP_L", "MCH_LEGAIM_L")}

bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for name in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[name]
    hv = ob.hide_viewport
    ob.hide_viewport = False
    for c in ob.users_collection:
        c.hide_viewport = False
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    P = np.empty(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    R["rest_dev_" + name] = float(np.linalg.norm(Q.reshape(-1, 3) - P, axis=1).max())
    ob.hide_viewport = hv

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
