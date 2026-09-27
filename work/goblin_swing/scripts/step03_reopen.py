"""Fresh-process reopen check for goblin_v04_skinned.blend."""
import bpy, json, math, sys, os
import numpy as np
from mathutils import Vector
sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL
R={}
R["file"]=bpy.data.filepath
rig=bpy.data.objects["GOB_rig"]; body=bpy.data.objects["GOB_body"]
me=body.data; N=len(me.vertices)
P=np.empty(N*3); me.vertices.foreach_get("co",P); P=P.reshape(N,3)
R["objects"]=sorted(o.name for o in bpy.data.objects if not o.name.startswith(("WGT_","s3_")))
R["body_parent"]=body.parent.name if body.parent else None
R["modifiers"]=[(m.type,m.object.name,m.use_deform_preserve_volume,m.use_vertex_groups,m.use_bone_envelopes) for m in body.modifiers]
R["n_vgroups"]=len(body.vertex_groups)
R["n_actions"]=len(bpy.data.actions)
R["rig_has_action"]=bool(rig.animation_data and rig.animation_data.action)
R["posed_bones"]=[pb.name for pb in rig.pose.bones if pb.location.length>1e-9
                  or abs(pb.rotation_quaternion.w-1)>1e-9 or Vector(pb.rotation_quaternion[1:]).length>1e-9
                  or Vector(pb.rotation_euler).length>1e-9 or (Vector(pb.scale)-Vector((1,1,1))).length>1e-9]
def dm():
    rig.update_tag(); bpy.context.view_layer.update()
    ev=body.evaluated_get(bpy.context.evaluated_depsgraph_get()); d=ev.to_mesh()
    Q=np.empty(len(d.vertices)*3); d.vertices.foreach_get("co",Q); ev.to_mesh_clear()
    return Q.reshape(-1,3)
Q=dm(); R["rest_max_dev_m"]=float(np.linalg.norm(Q-P,axis=1).max())
# spot-check pose: right arm raised 95 deg + elbow bent (the wind-up shape)
REST={n:b.matrix_local.copy() for n,b in [(b.name,b) for b in rig.data.bones]}
from mathutils import Quaternion
def wrot(bone,axis,deg):
    M=REST[bone].to_3x3(); q=Quaternion(Vector(axis).normalized(),math.radians(deg))
    rig.pose.bones[bone].rotation_quaternion=(M.inverted()@q.to_matrix()@M).to_quaternion()
wrot("UPPERARM_FK_R",(0,1,0),95)
rig.pose.bones["FOREARM_FK_R"].rotation_quaternion=Quaternion(Vector((1,0,0)),math.radians(50))
Q2=dm(); d2=np.linalg.norm(Q2-P,axis=1)
R["spotcheck_max_disp_m"]=round(float(d2.max()),4)
R["spotcheck_moved_verts"]=int((d2>0.001).sum())
cam,cd=RL.setup(res=1024,plain=True)
OUT=r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step03\reopen"
RL.shoot(cam,cd,OUT,"reopen_spotcheck_front",(0,0,0.95),0,0,2.1)
RL.shoot(cam,cd,OUT,"reopen_spotcheck_tq",(0,0,0.95),-40,12,2.1)
for pb in rig.pose.bones:
    pb.rotation_quaternion=(1,0,0,0); pb.rotation_euler=(0,0,0); pb.location=(0,0,0)
rig.update_tag(); bpy.context.view_layer.update()
RL.shoot(cam,cd,OUT,"reopen_rest_front",(0,0,0.95),0,0,2.1)
print("@@@JSON_START@@@"); print(json.dumps(R,indent=1,default=str)); print("@@@JSON_END@@@")
