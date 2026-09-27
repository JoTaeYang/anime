import bpy, bmesh, json, sys, math
from mathutils import Vector

out = {}
sc = bpy.context.scene

out["blend"] = bpy.data.filepath
out["blender_version"] = bpy.app.version_string
out["fps"] = sc.render.fps / sc.render.fps_base
out["frame_start"] = sc.frame_start
out["frame_end"] = sc.frame_end
out["unit_system"] = sc.unit_settings.system
out["unit_scale"] = sc.unit_settings.scale_length

# ---- collections
def coll_tree(c):
    return {"name": c.name, "objects": [o.name for o in c.objects],
            "children": [coll_tree(ch) for ch in c.children]}
out["collections"] = coll_tree(sc.collection)

objs = []
for o in bpy.data.objects:
    d = {
        "name": o.name, "type": o.type,
        "parent": o.parent.name if o.parent else None,
        "parent_type": o.parent_type if o.parent else None,
        "hide_viewport": o.hide_viewport, "hide_render": o.hide_render,
        "hide_get": o.hide_get() if o.name in sc.objects else None,
        "in_scene": o.name in sc.objects,
        "location": [round(v, 6) for v in o.location],
        "rotation_mode": o.rotation_mode,
        "rotation_euler_deg": [round(math.degrees(v), 4) for v in o.rotation_euler],
        "scale": [round(v, 6) for v in o.scale],
        "delta_location": [round(v, 6) for v in o.delta_location],
        "delta_scale": [round(v, 6) for v in o.delta_scale],
        "matrix_world_translation": [round(v, 6) for v in o.matrix_world.translation],
        "modifiers": [{"name": m.name, "type": m.type} for m in o.modifiers],
        "constraints": [{"name": c.name, "type": c.type} for c in o.constraints],
        "data": o.data.name if o.data else None,
        "data_users": o.data.users if o.data else None,
        "materials": [ms.material.name if ms.material else None for ms in o.material_slots],
        "animation_action": (o.animation_data.action.name if o.animation_data and o.animation_data.action else None),
    }
    objs.append(d)
out["objects"] = objs

# ---- meshes
meshes = []
for o in bpy.data.objects:
    if o.type != 'MESH':
        continue
    me = o.data
    me.calc_loop_triangles()
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    nonmanifold = sum(1 for e in bm.edges if not e.is_manifold)
    boundary = sum(1 for e in bm.edges if e.is_boundary)
    ngon = {}
    for f in bm.faces:
        n = len(f.verts)
        ngon[n] = ngon.get(n, 0) + 1

    # islands via flood fill on verts
    n = len(bm.verts)
    comp = [-1] * n
    islands = []
    for i in range(n):
        if comp[i] != -1:
            continue
        cid = len(islands)
        stack = [i]
        comp[i] = cid
        members = []
        while stack:
            v = stack.pop()
            members.append(v)
            for e in bm.verts[v].link_edges:
                w = e.other_vert(bm.verts[v]).index
                if comp[w] == -1:
                    comp[w] = cid
                    stack.append(w)
        islands.append(members)

    mw = o.matrix_world
    isl_info = []
    for cid, members in enumerate(islands):
        pts = [mw @ bm.verts[i].co for i in members]
        mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
        mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
        ctr = (mn + mx) / 2
        isl_info.append({
            "id": cid, "verts": len(members),
            "bbox_min_world": [round(v, 4) for v in mn],
            "bbox_max_world": [round(v, 4) for v in mx],
            "size_world": [round(v, 4) for v in (mx - mn)],
            "center_world": [round(v, 4) for v in ctr],
        })
    isl_info.sort(key=lambda d: -d["verts"])
    bm.free()

    bb = [mw @ Vector(c) for c in o.bound_box]
    bmn = [min(p[i] for p in bb) for i in range(3)]
    bmx = [max(p[i] for p in bb) for i in range(3)]

    meshes.append({
        "name": o.name,
        "verts": len(me.vertices), "edges": len(me.edges),
        "faces": len(me.polygons), "tris": len(me.loop_triangles),
        "all_tris": all(len(p.vertices) == 3 for p in me.polygons),
        "face_vert_counts": ngon,
        "nonmanifold_edges": nonmanifold,
        "boundary_edges": boundary,
        "island_count": len(islands),
        "islands": isl_info[:40],
        "uv_layers": [u.name for u in me.uv_layers],
        "color_attributes": [c.name for c in me.color_attributes],
        "vertex_groups": [g.name for g in o.vertex_groups],
        "shape_keys": ([k.name for k in me.shape_keys.key_blocks] if me.shape_keys else []),
        "bbox_world_min": [round(v, 4) for v in bmn],
        "bbox_world_max": [round(v, 4) for v in bmx],
        "bbox_world_size": [round(bmx[i] - bmn[i], 4) for i in range(3)],
        "origin_world": [round(v, 4) for v in mw.translation],
        "shade_smooth_faces": sum(1 for p in me.polygons if p.use_smooth),
    })
out["meshes"] = meshes

# ---- armatures
arms = []
for o in bpy.data.objects:
    if o.type == 'ARMATURE':
        arms.append({"name": o.name, "bones": [b.name for b in o.data.bones],
                     "bone_count": len(o.data.bones), "pose_position": o.data.pose_position})
out["armatures"] = arms
out["armature_datablocks"] = [a.name for a in bpy.data.armatures]

# ---- actions
acts = []
for a in bpy.data.actions:
    info = {"name": a.name, "users": a.users, "frame_range": [round(v, 2) for v in a.frame_range]}
    try:
        info["slots"] = [s.name_display for s in a.slots]
        info["layers"] = len(a.layers)
    except Exception as e:
        info["layers_err"] = str(e)
    acts.append(info)
out["actions"] = acts

# ---- materials / images
mats = []
for m in bpy.data.materials:
    nodes = []
    if m.use_nodes:
        for nd in m.node_tree.nodes:
            nodes.append(nd.type)
    mats.append({"name": m.name, "users": m.users, "use_nodes": m.use_nodes,
                 "node_types": sorted(set(nodes)),
                 "blend_method": getattr(m, "blend_method", None)})
out["materials"] = mats

imgs = []
for im in bpy.data.images:
    imgs.append({"name": im.name, "filepath": im.filepath, "packed": bool(im.packed_file),
                 "size": list(im.size), "source": im.source, "users": im.users,
                 "has_data": im.has_data,
                 "abspath_exists": bool(bpy.path.abspath(im.filepath)) and __import__("os").path.exists(bpy.path.abspath(im.filepath)) if im.filepath else False,
                 "colorspace": im.colorspace_settings.name})
out["images"] = imgs

print("@@@JSON_START@@@")
print(json.dumps(out, indent=1))
print("@@@JSON_END@@@")
