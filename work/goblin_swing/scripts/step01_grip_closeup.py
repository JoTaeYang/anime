import bpy, math, os
from mathutils import Vector
sc = bpy.context.scene
OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect"
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.show_cavity = True
sc.render.resolution_x = 1024; sc.render.resolution_y = 1024
sc.render.image_settings.file_format = 'PNG'

cd = bpy.data.cameras.new("c"); cd.type = 'ORTHO'; cd.ortho_scale = 0.55
cam = bpy.data.objects.new("c", cd); sc.collection.objects.link(cam); sc.camera = cam

targets = {"06_grip_back": (Vector((-0.45, 0.05, 0.10)), 180.0, 10.0),
           "07_grip_front": (Vector((-0.45, 0.05, 0.10)), 0.0, 10.0),
           "08_grip_top": (Vector((-0.45, 0.05, 0.10)), 90.0, 45.0),
           "09_legs_front": (Vector((0.0, 0.0, -0.72)), 0.0, 0.0),
           "10_armpit_right": (Vector((-0.45, 0.0, 0.32)), 0.0, 20.0)}
for name, (ctr, az, el) in targets.items():
    if name == "09_legs_front": cd.ortho_scale = 1.3
    elif name == "10_armpit_right": cd.ortho_scale = 0.7
    else: cd.ortho_scale = 0.55
    a = math.radians(az); e = math.radians(el)
    d = Vector((math.sin(a)*math.cos(e), -math.cos(a)*math.cos(e), math.sin(e)))
    cam.location = ctr + d*4
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("WROTE", sc.render.filepath)
