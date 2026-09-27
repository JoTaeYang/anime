import bpy,bmesh
from mathutils import Vector
sc=bpy.context.scene
print('FPS',sc.render.fps,'frames',sc.frame_start,sc.frame_end,'res',sc.render.resolution_x,sc.render.resolution_y,'cam',sc.camera.name if sc.camera else None)
print('COLLECTIONS',[(c.name,[o.name for o in c.objects]) for c in bpy.data.collections])
print('ARMATURES',[a.name for a in bpy.data.armatures],'| MATS',[m.name for m in bpy.data.materials])
print('MESH DATABLOCKS',[m.name for m in bpy.data.meshes])
for o in bpy.data.objects:
    print('OBJ %-14s %-9s hide_vp=%-5s hide_r=%-5s loc=%s rot=%s scl=%s'%(o.name,o.type,o.hide_viewport,o.hide_render,
        [round(c,5) for c in o.location],[round(c,4) for c in o.rotation_euler],[round(c,5) for c in o.scale]))
    if o.type!='MESH': continue
    m=o.data; P=[v.co for v in m.vertices]; bm=bmesh.new(); bm.from_mesh(m)
    seen=set(); ni=0; bm.verts.ensure_lookup_table()
    for v in bm.verts:
        if v.index in seen: continue
        ni+=1; st=[v]; seen.add(v.index)
        while st:
            x=st.pop()
            for e in x.link_edges:
                w=e.other_vert(x)
                if w.index not in seen: seen.add(w.index); st.append(w)
    print('   mesh=%s verts=%d tris=%d mats=%s nonman=%d loose=%d islands=%d smooth=%s'%(
        m.name,len(m.vertices),sum(len(p.vertices)-2 for p in m.polygons),
        [x.name if x else None for x in m.materials],
        sum(1 for e in bm.edges if not e.is_manifold),
        sum(1 for v in bm.verts if not v.link_faces),ni,all(p.use_smooth for p in m.polygons)))
    bm.free()
    print('   world bbox  x %.4f..%.4f  y %.4f..%.4f  z %.4f..%.4f'%(
        min((o.matrix_world@v).x for v in P),max((o.matrix_world@v).x for v in P),
        min((o.matrix_world@v).y for v in P),max((o.matrix_world@v).y for v in P),
        min((o.matrix_world@v).z for v in P),max((o.matrix_world@v).z for v in P)))
c=bpy.data.objects['GOB_club']; R=c.matrix_world.to_3x3()
print('CLUB origin',[round(x,5) for x in c.matrix_world.translation])
for ax,nm in ((Vector((1,0,0)),'+X'),(Vector((0,1,0)),'+Y'),(Vector((0,0,1)),'+Z')):
    print('   local %s -> world %s'%(nm,[round(x,5) for x in (R@ax)]))
