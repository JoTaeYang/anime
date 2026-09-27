import bpy, bmesh, json
from mathutils import Vector

ob = bpy.data.objects["mesh_node"]
me = ob.data
mw = ob.matrix_world
bm = bmesh.new(); bm.from_mesh(me)
bm.verts.ensure_lookup_table(); bm.edges.ensure_lookup_table(); bm.faces.ensure_lookup_table()

out = {}

# 1. non-manifold edges detail
nm = []
for e in bm.edges:
    if not e.is_manifold:
        nm.append({"index": e.index, "link_faces": len(e.link_faces),
                   "v0": [round(v,4) for v in (mw @ e.verts[0].co)],
                   "v1": [round(v,4) for v in (mw @ e.verts[1].co)]})
out["nonmanifold_edges"] = nm

# vertices shared by >2 shells: find verts whose link_faces do not form a single fan
# 2. face components across MANIFOLD edges only (splits shells joined only at nm edges/verts)
nf = len(bm.faces)
comp = [-1]*nf
comps = []
for f0 in bm.faces:
    if comp[f0.index] != -1: continue
    cid = len(comps); stack=[f0.index]; comp[f0.index]=cid; mem=[]
    while stack:
        fi = stack.pop(); mem.append(fi)
        for e in bm.faces[fi].edges:
            if len(e.link_faces) != 2: continue
            for g in e.link_faces:
                if comp[g.index] == -1:
                    comp[g.index]=cid; stack.append(g.index)
    comps.append(mem)

info=[]
for cid, mem in enumerate(comps):
    vs=set()
    for fi in mem:
        for v in bm.faces[fi].verts: vs.add(v.index)
    pts=[mw @ bm.verts[i].co for i in vs]
    mn=Vector((min(p.x for p in pts),min(p.y for p in pts),min(p.z for p in pts)))
    mx=Vector((max(p.x for p in pts),max(p.y for p in pts),max(p.z for p in pts)))
    info.append({"id":cid,"faces":len(mem),"verts":len(vs),
                 "bbox_min":[round(v,4) for v in mn],"bbox_max":[round(v,4) for v in mx],
                 "size":[round(v,4) for v in (mx-mn)],
                 "center":[round(v,4) for v in (mn+mx)/2]})
info.sort(key=lambda d:-d["faces"])
out["face_components_manifold_only"] = info

# 3. vertex-shell components (vertex adjacency) already known = 1; also check duplicate verts at same location between comps
# 4. Z/geometry profile of the whole mesh
zs=[(mw@v.co).z for v in bm.verts]
out["z_min"]=round(min(zs),4); out["z_max"]=round(max(zs),4)

# 5. probe: geometry near lowest Z (feet) and per-side hand region
# report count of verts below z_min+0.02
out["verts_within_2cm_of_lowest"]=sum(1 for z in zs if z < min(zs)+0.02)

# 6. distance between components (min vertex distance) for top few comps
import itertools
def vset(cid):
    vs=set()
    for fi in comps[cid]:
        for v in bm.faces[fi].verts: vs.add(v.index)
    return [mw @ bm.verts[i].co for i in vs]
if len(comps) <= 12:
    ids=[c["id"] for c in info]
    pairs={}
    for a,b in itertools.combinations(ids,2):
        A=vset(a); B=vset(b)
        # coarse: sample
        best=1e9
        for p in A[::max(1,len(A)//400)]:
            for q in B[::max(1,len(B)//400)]:
                d=(p-q).length
                if d<best: best=d
        pairs[f"{a}-{b}"]=round(best,4)
    out["component_min_distance_sampled"]=pairs

bm.free()
print("@@@JSON_START@@@")
print(json.dumps(out, indent=1))
print("@@@JSON_END@@@")
