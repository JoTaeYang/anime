"""Tight look at one leg junction in one pose.  Renders only; never saves.

blender -b <file> --python step03r_zoom.py -- --mesh hi --tag Z --side R
"""
import bpy, math, os, sys
from mathutils import Vector, Quaternion
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d=None):
    return argv[argv.index(n) + 1] if n in argv else d


MESH = opt("--mesh", "hi")
TAG = opt("--tag", "Z")
OUT = os.path.join(ROOT, "inspect", "step03r")
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
body = bpy.data.objects["GOB_body_lo" if MESH == "lo" else "GOB_body"]
other = bpy.data.objects["GOB_body" if MESH == "lo" else "GOB_body_lo"]
for o, hid in ((body, False), (other, True)):
    o.hide_render = hid
    o.hide_viewport = hid
    for c in o.users_collection:
        c.hide_render = hid
        c.hide_viewport = hid
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
if "--compress" in argv:
    # COMPARISON ONLY: disable the Limit Distance clamp, so the rigid leg shortens
    # to reach the hip socket instead of sliding up into the torso.
    for S in ("L", "R"):
        for c in PB["MCH_LEGTOP_" + S].constraints:
            c.influence = 0.0

sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'STUDIO'; sh.studio_light = 'Default'; sh.color_type = 'SINGLE'
sh.single_color = (0.72, 0.72, 0.74); sh.show_cavity = True; sh.cavity_type = 'BOTH'
if "--color" in argv:
    sys.path.append(os.path.join(ROOT, "scripts"))
    import step03_renderlib as RL
    RL.colorize_by_group(body)
    sh.color_type = 'MATERIAL'
sh.curvature_ridge_factor = 1.0; sh.curvature_valley_factor = 1.0
sh.show_shadows = True; sh.shadow_intensity = 0.45; sh.show_object_outline = False
sc.display.render_aa = '8'
sc.render.film_transparent = False
sc.render.resolution_x = sc.render.resolution_y = 768
sc.render.resolution_percentage = 100
cd = bpy.data.cameras.get("zcam") or bpy.data.cameras.new("zcam")
cd.type = 'ORTHO'
cam = bpy.data.objects.get("zcam") or bpy.data.objects.new("zcam", cd)
if cam.name not in sc.collection.objects:
    sc.collection.objects.link(cam)
sc.camera = cam


def wloc(bone, delta):
    PB[bone].location = REST[bone].to_3x3().inverted() @ Vector(delta)


def wrot(bone, axis, deg, add=False):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
    pb.rotation_euler = (cur @ ql).to_euler(pb.rotation_mode)


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)


def shot(name, target, az, el, scale):
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    ctr = Vector(target)
    cam.location = ctr + d * 8.0
    cd.ortho_scale = scale
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, "%s_%s.png" % (TAG, name))
    bpy.ops.render.render(write_still=True)


POSES = {
    "rest": lambda: None,
    "f15like": lambda: (wloc("FOOT_IK_R", (-0.15, 0, 0)),
                        wloc("COG_CTRL", (-0.05, 0, -0.15)),
                        wrot("CHEST_CTRL", (1, 0, 0), 25),
                        wrot("CHEST_CTRL", (0, 0, 1), 30, True)),
    "cog_dn20": lambda: wloc("COG_CTRL", (0, 0, -0.20)),
    "stanceR": lambda: (wloc("FOOT_IK_R", (-0.15, 0, 0)),
                        wloc("COG_CTRL", (-0.05, 0, -0.12))),
}
for pname, fn in POSES.items():
    clear()
    fn()
    rig.update_tag(); bpy.context.view_layer.update()
    ev = rig.evaluated_get(bpy.context.evaluated_depsgraph_get())
    for S in ("L", "R"):
        # centre on the middle of the posed leg
        ank = ev.pose.bones["FOOT_" + S].matrix.to_translation()
        top = ev.pose.bones["MCH_LEGTOP_" + S].matrix.to_translation()
        mid = (ank + top) * 0.5
        for az, el, sc_, nm in ((0, -6, 0.34, "below0"), (55, -6, 0.34, "below55"),
                                 (0, 6, 0.50, "cam0"), (25, 6, 0.50, "cam25")):
            shot("%s_%s_%s" % (pname, S, nm), mid, az, el, sc_)
clear()
print("ZOOM DONE")
