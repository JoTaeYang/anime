import bpy,os,math,json
from mathutils import Vector
sc=bpy.context.scene
sc.render.engine='BLENDER_WORKBENCH'; sc.display.shading.light='STUDIO'
sc.display.shading.show_cavity=True; sc.display.shading.cavity_type='BOTH'
sc.render.resolution_x=1200; sc.render.resolution_y=500; sc.render.resolution_percentage=100
cd=bpy.data.cameras.new("C"); cam=bpy.data.objects.new("C",cd); sc.collection.objects.link(cam)
sc.camera=cam; cd.type='ORTHO'
for o in bpy.data.objects:
    if o.type=='MESH': o.hide_viewport=o.hide_render=(o.name!="GOB_club")
club=bpy.data.objects["GOB_club"]
c=club.matrix_world @ Vector((0,0.32,0))
for nm,az in (("club_alone_side",-90),("club_alone_top",0)):
    el=88 if az==0 else 0
    a,e=math.radians(az),math.radians(el)
    d=Vector((math.sin(a)*math.cos(e),-math.cos(a)*math.cos(e),math.sin(e)))
    cam.location=c+d*12; cd.ortho_scale=1.5
    cam.rotation_euler=(c-cam.location).normalized().to_track_quat('-Z','Y').to_euler()
    sc.render.filepath=os.path.join(r"C:\Users\whxod\orca\anime\work\goblin_swing\t\inspect\step01\cut",nm+".png")
    bpy.ops.render.render(write_still=True); print("SHOT",nm)
