import bpy, json, hashlib
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\goblin_v01_prerig.blend"

ob = bpy.data.objects["mesh_node"]
me = ob.data
rep = {}

def stats(o):
    m = o.data
    mw = o.matrix_world
    P = [mw @ v.co for v in m.vertices]
    mn = Vector((min(p.x for p in P), min(p.y for p in P), min(p.z for p in P)))
    mx = Vector((max(p.x for p in P), max(p.y for p in P), max(p.z for p in P)))
    h = hashlib.sha1()
    for v in m.vertices:
        h.update(b"%.6f|%.6f|%.6f;" % (v.co.x, v.co.y, v.co.z))
    return {
        "verts": len(m.vertices), "edges": len(m.edges), "faces": len(m.polygons),
        "bbox_min": [round(c, 6) for c in mn],
        "bbox_max": [round(c, 6) for c in mx],
        "size": [round(c, 6) for c in (mx - mn)],
        "loc": [round(c, 6) for c in o.location],
        "rot_quat": [round(c, 6) for c in o.rotation_quaternion],
        "scale": [round(c, 6) for c in o.scale],
        "local_hash": h.hexdigest(),
    }

rep["before"] = stats(ob)
zmin = rep["before"]["bbox_min"][2]
rep["shift_z"] = round(-zmin, 6)

# 1. translate so lowest vert sits at Z=0, then apply transform
ob.location = (0.0, 0.0, -zmin)
bpy.context.view_layer.objects.active = ob
for o in bpy.data.objects:
    o.select_set(False)
ob.select_set(True)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# origin to world (0,0,0)
ob.location = (0.0, 0.0, 0.0)
bpy.context.view_layer.update()

# 2. renames
ob.name = "GOB_body"
me.name = "GOB_body_mesh"
mat = bpy.data.materials.get("material")
if mat:
    mat.name = "GOB_mat"
rep["material_names"] = [m.name for m in bpy.data.materials]

# 3. scene
sc = bpy.context.scene
sc.render.fps = 24
sc.render.fps_base = 1.0
sc.frame_start = 1
sc.frame_end = 39
rep["scene"] = {"fps": sc.render.fps, "fps_base": sc.render.fps_base,
                "start": sc.frame_start, "end": sc.frame_end}

rep["after"] = stats(ob)

b, a = rep["before"], rep["after"]
checks = {
    "vert_count_same": b["verts"] == a["verts"],
    "edge_count_same": b["edges"] == a["edges"],
    "face_count_same": b["faces"] == a["faces"],
    "size_same": all(abs(b["size"][i] - a["size"][i]) < 1e-5 for i in range(3)),
    "x_unchanged": abs(b["bbox_min"][0] - a["bbox_min"][0]) < 1e-5,
    "y_unchanged": abs(b["bbox_min"][1] - a["bbox_min"][1]) < 1e-5,
    "minz_zero": abs(a["bbox_min"][2]) < 1e-5,
    "loc_identity": all(abs(c) < 1e-9 for c in a["loc"]),
    "rot_identity": abs(a["rot_quat"][0] - 1.0) < 1e-9 and all(abs(c) < 1e-9 for c in a["rot_quat"][1:]),
    "scale_identity": all(abs(c - 1.0) < 1e-9 for c in a["scale"]),
    "obj_name": ob.name == "GOB_body",
    "mesh_name": me.name == "GOB_body_mesh",
    "mat_name": "GOB_mat" in bpy.data.materials,
}
rep["checks"] = checks
rep["all_ok"] = all(checks.values())

print("@@@JSON_START@@@"); print(json.dumps(rep, indent=1)); print("@@@JSON_END@@@")
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
