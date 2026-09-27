import bpy, os, math, sys
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\t\inspect\step01\cut"
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.show_cavity = True
sc.display.shading.cavity_type = 'BOTH'
sc.render.resolution_x = sc.render.resolution_y = 768
sc.render.resolution_percentage = 100
cd = bpy.data.cameras.new("SHOTCAM")
cam = bpy.data.objects.new("SHOTCAM", cd)
sc.collection.objects.link(cam)
sc.camera = cam
cd.type = 'ORTHO'
hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
club = bpy.data.objects["GOB_club"]
G = tuple(club.matrix_world.translation)


def vis(**kw):
    for o, s in kw.items():
        ob = {"hi": hi, "lo": lo, "club": club}[o]
        ob.hide_viewport = ob.hide_render = not s


def shot(name, c, rad, az, el):
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    c = Vector(c)
    cam.location = c + d * 12
    cd.ortho_scale = rad * 2
    cam.rotation_euler = (c - cam.location).normalized().to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("SHOT", name)


BODY = (0, 0.05, 0.95)
vis(hi=False, lo=True, club=True)
shot("full_lo_front", BODY, 1.06, 0, 0)
shot("full_lo_34", BODY, 1.06, 40, 12)
shot("full_lo_side", BODY, 1.06, -90, 0)
shot("lo_face", (0, 0.05, 1.60), 0.42, 0, 0)
vis(hi=True, lo=False, club=True)
shot("hi_face", (0, 0.05, 1.60), 0.42, 0, 0)
shot("hand_club_front", G, 0.34, 0, 0)
shot("hand_club_top", G, 0.34, 0, 88)
shot("hand_club_side", G, 0.34, -90, 0)
shot("hand_club_34", G, 0.34, 35, 25)
vis(hi=True, lo=False, club=False)
shot("hand_only_front", G, 0.26, 0, 0)
shot("hand_only_top", G, 0.26, 0, 88)
shot("hand_only_side", G, 0.26, -90, 0)
shot("hand_only_below", G, 0.26, 0, -65)
shot("hand_only_34", G, 0.26, 35, 25)
vis(hi=False, lo=True, club=False)
shot("lo_hand_only_34", G, 0.26, 35, 25)
vis(hi=False, lo=True, club=True)
