import bpy,sys,os
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx")
exec(open(r"C:\Users\whxod\orca\anime\work\goblin_swing\t\scripts\shots.py").read())
