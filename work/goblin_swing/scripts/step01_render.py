import bpy, math, json, os
from mathutils import Vector

sc = bpy.context.scene
OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect"
os.makedirs(OUT, exist_ok=True)

info = {}
# --- material base color probe
for m in bpy.data.materials:
    nd = m.node_tree.nodes.get("Principled BSDF") if m.node_tree else None
    if nd:
        info[m.name] = {
            "base_color": [round(v, 4) for v in nd.inputs["Base Color"].default_value],
            "base_color_linked": nd.inputs["Base Color"].is_linked,
            "metallic": round(nd.inputs["Metallic"].default_value, 3),
            "roughness": round(nd.inputs["Roughness"].default_value, 3),
        }
info["datablock_counts"] = {k: len(getattr(bpy.data, k)) for k in
    ("images", "materials", "textures", "node_groups", "armatures", "actions",
     "meshes", "objects", "collections", "cameras", "lights", "libraries")}
print("@@@MAT@@@", json.dumps(info))

ob = bpy.data.objects["mesh_node"]
bb = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
mn = Vector((min(p.x for p in bb), min(p.y for p in bb), min(p.z for p in bb)))
mx = Vector((max(p.x for p in bb), max(p.y for p in bb), max(p.z for p in bb)))
ctr = (mn + mx) / 2
rad = max((mx - mn)) * 0.75

# render settings
sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'STUDIO'
sh.color_type = 'MATERIAL'
sh.show_cavity = True
sc.render.resolution_x = 1024
sc.render.resolution_y = 1024
sc.render.resolution_percentage = 100
sc.render.film_transparent = False
sc.render.image_settings.file_format = 'PNG'

cam_data = bpy.data.cameras.new("tmp_cam")
cam_data.type = 'ORTHO'
cam_data.ortho_scale = max(mx - mn) * 1.15
cam = bpy.data.objects.new("tmp_cam", cam_data)
sc.collection.objects.link(cam)
sc.camera = cam

def shoot(name, az_deg, el_deg=0.0):
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    d = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)))
    cam.location = ctr + d * (rad * 6)
    # point camera at ctr
    direction = ctr - cam.location
    rot = direction.to_track_quat('-Z', 'Y').to_euler()
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = rot
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("WROTE", sc.render.filepath)

shoot("01_front", 0)
shoot("02_side_right", 90)
shoot("03_back", 180)
shoot("04_three_quarter", 35, 12)
shoot("05_top", 0, 89)
print("DONE")
