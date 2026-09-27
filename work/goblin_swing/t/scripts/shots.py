import bpy, sys, os, math, json
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]
SPEC = json.loads(argv[1])   # list of [name, cx,cy,cz, radius, azim_deg, elev_deg]
os.makedirs(OUT, exist_ok=True)

sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.show_cavity = True
sc.display.shading.cavity_type = 'BOTH'
sc.render.resolution_x = 768
sc.render.resolution_y = 768
sc.render.film_transparent = False
sc.render.image_settings.file_format = 'PNG'

cam = bpy.data.objects.get("SHOTCAM")
if not cam:
    cd = bpy.data.cameras.new("SHOTCAM")
    cam = bpy.data.objects.new("SHOTCAM", cd)
    sc.collection.objects.link(cam)
sc.camera = cam
cam.data.type = 'ORTHO'

for name, cx, cy, cz, rad, az, el in SPEC:
    c = Vector((cx, cy, cz))
    a = math.radians(az); e = math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    cam.location = c + d * 10.0
    cam.data.ortho_scale = rad * 2
    # aim at c
    fwd = (c - cam.location).normalized()
    cam.rotation_euler = fwd.to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("SHOT", sc.render.filepath)
