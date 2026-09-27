import bpy, bmesh, sys, math
from mathutils import Vector
mode = sys.argv[sys.argv.index('--')+1]
if mode == 'fbx':
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx")
for o in bpy.data.objects:
    print('OBJ', o.name, o.type, [round(v,4) for v in o.location], [round(v,4) for v in o.scale], [round(math.degrees(a),2) for a in o.rotation_euler], [m.type for m in o.modifiers], o.parent.name if o.parent else None)
    if o.type=='MESH':
        me=o.data; me.calc_loop_triangles()
        ws=[o.matrix_world@v.co for v in me.vertices]
        mn=[min(w[i] for w in ws) for i in range(3)]; mx=[max(w[i] for w in ws) for i in range(3)]
        bm=bmesh.new(); bm.from_mesh(me)
        nm=sum(1 for e in bm.edges if not e.is_manifold)
        seen=set(); isl=[]
        for v in bm.verts:
            if v.index in seen: continue
            st=[v]; n=0
            while st:
                a=st.pop()
                if a.index in seen: continue
                seen.add(a.index); n+=1
                st.extend(e.other_vert(a) for e in a.link_edges)
            isl.append(n)
        print(' verts',len(me.vertices),'tris',len(me.loop_triangles),'uv',[u.name for u in me.uv_layers],'nonmanifold',nm,'islands',sorted(isl,reverse=True)[:8])
        print(' bbox',[round(x,4) for x in mn],[round(x,4) for x in mx])
        print(' mats',[(s.material.name if s.material else None) for s in o.material_slots])
        # x-symmetry: for sample verts, distance to nearest mirrored vert
        from mathutils.kdtree import KDTree
        kd=KDTree(len(ws)); [kd.insert(w,i) for i,w in enumerate(ws)]; kd.balance()
        import random; random.seed(1)
        cx=(mn[0]+mx[0])/2
        ds=[kd.find(Vector((2*cx-w.x,w.y,w.z)))[2] for w in random.sample(ws,min(3000,len(ws)))]
        ds.sort(); print(' mirror_dist median %.4f p95 %.4f max %.4f'%(ds[len(ds)//2],ds[int(len(ds)*.95)],ds[-1]))
print('IMAGES',[(i.name,i.size[:],i.filepath) for i in bpy.data.images])
print('ARMS',len(bpy.data.armatures),'ACTIONS',len(bpy.data.actions))
# render front + 3/4
import os
sc=bpy.context.scene
meshes=[o for o in bpy.data.objects if o.type=='MESH']
pts=[o.matrix_world@Vector(c) for o in meshes for c in o.bound_box]
mn=Vector([min(p[i] for p in pts) for i in range(3)]); mx=Vector([max(p[i] for p in pts) for i in range(3)])
c=(mn+mx)/2; size=max(mx-mn)
cam=bpy.data.cameras.new('c'); cam.type='ORTHO'; cam.ortho_scale=size*1.15
co=bpy.data.objects.new('c',cam); sc.collection.objects.link(co); sc.camera=co
sc.render.engine='BLENDER_WORKBENCH'; sc.display.shading.light='STUDIO'; sc.display.shading.show_cavity=True
sc.display.shading.color_type='TEXTURE' if bpy.data.images else 'MATERIAL'
sc.render.resolution_x=sc.render.resolution_y=900
for name,d in [('front',Vector((0,-1,0))),('back',Vector((0,1,0))),('top',Vector((0,0,1))),('tq',Vector((0.7,-1,0.35)).normalized())]:
    co.location=c+d*size*3
    co.rotation_euler=(c-co.location).to_track_quat('-Z','Y').to_euler()
    sc.render.filepath=os.path.join(r"C:\Users\whxod\orca\anime\work\goblin_swing\tpose_check",mode+"2_"+name+'.png')
    bpy.ops.render.render(write_still=True)
