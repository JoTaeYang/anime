"""Render the weight segmentation of goblin_v04_skinned.blend (read-only)."""
import bpy, os, sys
sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step03\labels"
body = bpy.data.objects["GOB_body"]
cam, cd = RL.setup(res=1100, plain=False)

RL.colorize_by_group(body)
C = (0.0, 0.0, 0.95)
RL.shoot(cam, cd, OUT, "lbl_front", C, 0, 0, 2.2)
RL.shoot(cam, cd, OUT, "lbl_back", C, 180, 0, 2.2)
RL.shoot(cam, cd, OUT, "lbl_sideR", C, 90, 0, 2.2)
RL.shoot(cam, cd, OUT, "lbl_sideL", C, 270, 0, 2.2)
RL.shoot(cam, cd, OUT, "lbl_shoulderR", (-0.42, 0.02, 1.22), 25, 18, 0.75)
RL.shoot(cam, cd, OUT, "lbl_shoulderL", (0.42, 0.06, 1.22), 335, 18, 0.75)
RL.shoot(cam, cd, OUT, "lbl_grip", (-0.68, -0.30, 0.99), 60, 5, 0.65)
RL.shoot(cam, cd, OUT, "lbl_legs", (0.0, 0.05, 0.28), 0, 8, 1.1)
RL.shoot(cam, cd, OUT, "lbl_neck", (0.0, 0.0, 1.31), 20, 5, 1.1)
RL.shoot(cam, cd, OUT, "lbl_handL", (0.77, 0.05, 0.75), 300, 5, 0.6)

RL.colorize_by_blend(body)
OUT2 = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step03\blend"
RL.shoot(cam, cd, OUT2, "bl_front", C, 0, 0, 2.2)
RL.shoot(cam, cd, OUT2, "bl_sideR", C, 90, 0, 2.2)
RL.shoot(cam, cd, OUT2, "bl_shoulderR", (-0.42, 0.02, 1.22), 25, 18, 0.75)
RL.shoot(cam, cd, OUT2, "bl_legs", (0.0, 0.05, 0.28), 0, 8, 1.1)
print("DONE")
