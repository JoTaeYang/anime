import bpy, sys, os, math, json
import numpy as np
from mathutils import Vector
sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL
rig=bpy.data.objects["GOB_rig"]; body=bpy.data.objects["GOB_body"]; PB=rig.pose.bones
REST={n:b.matrix_local.copy() for n,b in [(b.name,b) for b in rig.data.bones]}
OUT=r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step03\kneerange"
cam,cd=RL.setup(res=1024,plain=True)
for dz in (0.005,0.01,0.02,0.03,0.05):
    for pb in PB:
        pb.location=(0,0,0); pb.scale=(1,1,1); pb.rotation_quaternion=(1,0,0,0); pb.rotation_euler=(0,0,0)
    PB["COG_CTRL"].location = REST["COG_CTRL"].to_3x3().inverted() @ Vector((0,0,-dz))
    rig.update_tag(); bpy.context.view_layer.update()
    ev=rig.evaluated_get(bpy.context.evaluated_depsgraph_get())
    hip=ev.pose.bones["THIGH_L"].matrix.to_translation(); kn=ev.pose.bones["SHIN_L"].matrix.to_translation()
    an=ev.pose.bones["FOOT_L"].matrix.to_translation()
    flex=math.degrees((kn-hip).angle(an-kn))
    tag="cog%03d_flex%02d"%(int(dz*1000),int(flex))
    RL.shoot(cam,cd,OUT,tag+"_legs",(0,0.05,0.28),0,5,1.0)
    RL.shoot(cam,cd,OUT,tag+"_knee",kn,15,0,0.40)
    print("FLEX",dz,round(flex,1))
for pb in PB:
    pb.location=(0,0,0); pb.rotation_quaternion=(1,0,0,0); pb.rotation_euler=(0,0,0)
