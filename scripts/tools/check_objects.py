import bpy

bpy.ops.wm.open_mainfile(filepath=r'C:/Users/whxod/orca/anime/build/00_mesh.blend')
print('Mesh Objects:', [o.name for o in bpy.data.objects if o.type == 'MESH'])
print('All Objects:', [o.name for o in bpy.data.objects])
