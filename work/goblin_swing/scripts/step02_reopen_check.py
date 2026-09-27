import bpy, json
sc = bpy.context.scene
o = {"objects": sorted([ob.name + ":" + ob.type for ob in bpy.data.objects]),
     "meshes": sorted([m.name for m in bpy.data.meshes]),
     "armatures": sorted([a.name for a in bpy.data.armatures]),
     "materials": sorted([m.name for m in bpy.data.materials]),
     "cameras": [c.name for c in bpy.data.cameras],
     "lights": [l.name for l in bpy.data.lights],
     "worlds": [w.name for w in bpy.data.worlds],
     "collections": [c.name for c in bpy.data.collections],
     "actions": [a.name for a in bpy.data.actions],
     "engine": sc.render.engine,
     "fps": sc.render.fps, "start": sc.frame_start, "end": sc.frame_end}
b = bpy.data.objects["GOB_body"]; r = bpy.data.objects["GOB_rig"]
o["body"] = {"verts": len(b.data.vertices), "faces": len(b.data.polygons),
             "mods": [m.type for m in b.modifiers], "vgroups": len(b.vertex_groups),
             "parent": b.parent.name if b.parent else None,
             "loc": list(b.location), "scale": list(b.scale)}
o["rig"] = {"bones": len(r.data.bones), "loc": list(r.location), "scale": list(r.scale),
            "names": [bb.name for bb in r.data.bones],
            "deform": sorted([bb.name for bb in r.data.bones if bb.use_deform]),
            "nondeform": sorted([bb.name for bb in r.data.bones if not bb.use_deform]),
            "pose_bones": len(r.pose.bones)}
print("@@@J@@@" + json.dumps(o))
