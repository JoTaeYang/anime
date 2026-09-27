"""Isolate which joint collapses: rotate one joint at a time, render the right arm."""
import bpy, math, os, sys, json
import numpy as np
from mathutils import Vector

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
TAG = argv[argv.index("--tag") + 1] if "--tag" in argv else "iso"
OUT = os.path.join(T, "inspect", "step03", TAG)
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]; arm = rig.data; PB = rig.pose.bones
MESH = argv[argv.index("--mesh") + 1] if "--mesh" in argv else "lo"
lo = bpy.data.objects["GOB_body_lo" if MESH == "lo" else "GOB_body"]
other = bpy.data.objects["GOB_body" if MESH == "lo" else "GOB_body_lo"]
for c, vis in ((bpy.data.collections["GOB_hi"], MESH == "hi"),
               (bpy.data.collections["GOB_lo"], MESH == "lo")):
    c.hide_viewport = c.hide_render = not vis
lo.hide_viewport = lo.hide_render = False
other.hide_viewport = other.hide_render = True
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name in ("GOB_hi", "GOB_lo"):
        lc.hide_viewport = (lc.name == "GOB_hi") != (MESH == "hi")
        lc.exclude = False
for o in ("GOB_club", "REF_ground"):
    if o in bpy.data.objects:
        bpy.data.objects[o].hide_viewport = bpy.data.objects[o].hide_render = True
for p in lo.data.polygons:
    p.use_smooth = True
lo.data.update()

sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'STUDIO'; sh.studio_light = 'Default'; sh.color_type = 'SINGLE'
sh.single_color = (0.62, 0.68, 0.45); sh.show_cavity = True; sh.cavity_type = 'BOTH'
sh.curvature_ridge_factor = 1.0; sh.curvature_valley_factor = 1.0
sh.show_object_outline = False
sc.display.render_aa = '8'
sc.render.image_settings.file_format = 'PNG'
sc.render.resolution_x = sc.render.resolution_y = 512
cd = bpy.data.cameras.new("ic"); cd.type = 'ORTHO'
cam = bpy.data.objects.new("ic", cd); sc.collection.objects.link(cam); sc.camera = cam


def shot(n, t, az, el, s):
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    ctr = Vector(t)
    cam.location = ctr + d * 9
    cd.ortho_scale = s
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, n + ".png")
    bpy.ops.render.render(write_still=True)


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0


def upd():
    rig.update_tag(); lo.update_tag(); bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


CASES = {
    "u_windup": {"UPPERARM_FK_R": (-52.15, 100.89, 32.7)},
    "u_impact": {"UPPERARM_FK_R": (41.27, -54.39, 117.86)},
    "u_twist100": {"UPPERARM_FK_R": (0, 100, 0)},
    "u_swing100": {"UPPERARM_FK_R": (0, 0, 100)},
    "f_flex128": {"FOREARM_FK_R": (127.7, 0, 0)},
    "h_pron135": {"HAND_FK_R": (0, 135, 0)},
    "f_flex_pron": {"FOREARM_FK_R": (127.7, 0, 0), "HAND_FK_R": (0, 135, 0)},
}
R = {}
clear(); dg = upd()
shot("rest_arm", (-0.66, 0.15, 1.26), 0, 25, 1.30)
shot("rest_sh", (-0.42, 0.13, 1.26), -30, 15, 0.75)
for k, d in CASES.items():
    clear()
    for b, e in d.items():
        PB[b].rotation_euler = [math.radians(v) for v in e]
    dg = upd()
    ev = rig.evaluated_get(dg)
    sh_p = rig.matrix_world @ ev.pose.bones["UPPERARM_R"].head
    el_p = rig.matrix_world @ ev.pose.bones["FOREARM_R"].head
    ctr = (Vector(sh_p) + Vector(el_p)) * 0.5
    shot(k + "_A", tuple(ctr), -30, 15, 1.00)
    shot(k + "_B", tuple(ctr), 60, 35, 1.00)
print("@@@JSON_START@@@"); print(json.dumps(R)); print("@@@JSON_END@@@")
