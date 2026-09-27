"""Label / weight visualisation of the shoulder region."""
import bpy, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion
sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
TAG = argv[argv.index("--tag") + 1] if "--tag" in argv else "x"
MESH = argv[argv.index("--mesh") + 1] if "--mesh" in argv else "GOB_body_lo"
OUT = os.path.join(ROOT, "inspect", "step03r2")

hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
for c, vis in (("GOB_hi", MESH == "GOB_body"), ("GOB_lo", MESH == "GOB_body_lo")):
    cc = bpy.data.collections.get(c)
    if cc:
        cc.hide_viewport = not vis
        cc.hide_render = not vis
hi.hide_viewport = MESH != "GOB_body"; hi.hide_render = hi.hide_viewport
lo.hide_viewport = MESH != "GOB_body_lo"; lo.hide_render = lo.hide_viewport
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False
    lc.hide_viewport = not ((lc.name == "GOB_hi") == (MESH == "GOB_body"))

ob = bpy.data.objects[MESH]
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
RL.colorize_by_group(ob)
cam, cd = RL.setup(res=1024, plain=False)


def wrot(bone, ax, deg):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(ax).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


VIEWS = {"front": ((0.0, 0.0, 0.95), 0, 0, 2.1),
         "tqR": ((0.0, 0.0, 0.95), -40, 12, 2.1),
         "shR": ((-0.40, 0.05, 1.18), -20, 5, 0.60),
         "shL": ((0.42, 0.05, 1.18), 20, 5, 0.60)}
for v, (t, az, el, sc) in VIEWS.items():
    RL.shoot(cam, cd, OUT, "%s_lbl_rest_%s" % (TAG, v), Vector(t), az, el, sc)
wrot("UPPERARM_FK_R", (0, 1, 0), 90)
rig.update_tag(); bpy.context.view_layer.update()
for v in ("front", "tqR"):
    t, az, el, sc = VIEWS[v]
    RL.shoot(cam, cd, OUT, "%s_lbl_T2_%s" % (TAG, v), Vector(t), az, el, sc)
print("LABELDONE")
