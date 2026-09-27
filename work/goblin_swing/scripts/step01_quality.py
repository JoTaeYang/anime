import bpy, bmesh, json
ob = bpy.data.objects["mesh_node"]; me = ob.data
bm = bmesh.new(); bm.from_mesh(me)
out = {}
out["zero_area_faces"] = sum(1 for f in bm.faces if f.calc_area() < 1e-9)
out["tiny_faces_lt_1e-7"] = sum(1 for f in bm.faces if f.calc_area() < 1e-7)
areas = sorted(f.calc_area() for f in bm.faces)
out["face_area_median"] = round(areas[len(areas)//2], 9)
out["face_area_max"] = round(areas[-1], 9)
el = sorted(e.calc_length() for e in bm.edges)
out["edge_len_min"] = round(el[0], 7)
out["edge_len_median"] = round(el[len(el)//2], 6)
out["edge_len_max"] = round(el[-1], 6)
# duplicate verts
seen = {}
dup = 0
for v in bm.verts:
    k = (round(v.co.x,6), round(v.co.y,6), round(v.co.z,6))
    if k in seen: dup += 1
    else: seen[k] = 1
out["duplicate_coord_verts"] = dup
out["loose_verts"] = sum(1 for v in bm.verts if not v.link_edges)
out["loose_edges"] = sum(1 for e in bm.edges if not e.link_faces)
out["wire_edges"] = sum(1 for e in bm.edges if len(e.link_faces) == 1)
out["volume_m3"] = round(bm.calc_volume(signed=True), 6)
bm.free()
print("@@@JSON_START@@@"); print(json.dumps(out, indent=1)); print("@@@JSON_END@@@")
