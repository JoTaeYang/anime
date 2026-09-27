"""Render the REST pose with the same close-up cameras the gate uses."""
import bpy, os, sys
sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step03\rest"
rig = bpy.data.objects["GOB_rig"]
body = bpy.data.objects["GOB_body"]
LEN = {n: b.length for n, b in [(b.name, b) for b in rig.data.bones]}
cam, cd = RL.setup(res=1024, plain=True)
bpy.context.view_layer.update()
ev = rig.evaluated_get(bpy.context.evaluated_depsgraph_get())


def head(n):
    return ev.pose.bones[n].matrix.to_translation()


SH = [("kneeL", head("SHIN_L"), 345, 0, 0.42),
      ("kneeR", head("SHIN_R"), 15, 0, 0.42),
      ("hipR", head("THIGH_R"), 15, -12, 0.55),
      ("ankleR", head("FOOT_R"), 20, 5, 0.50),
      ("neck", head("NECK"), 20, 8, 1.00),
      ("shoulderR", head("UPPERARM_R"), 20, 20, 0.80),
      ("elbowR", head("FOREARM_R"), 40, 10, 0.50),
      ("wristR", head("HAND_R"), 55, 5, 0.70),
      ("gripR", head("WEAPON"), 125, 5, 0.70),
      ("legs", Vector((0, 0.05, 0.28)), 0, 8, 1.05),
      ("front", Vector((0, 0, 0.95)), 0, 0, 2.1)]
for n, t, az, el, s in SH:
    RL.shoot(cam, cd, OUT, "rest_" + n, t, az, el, s)
print("DONE")
