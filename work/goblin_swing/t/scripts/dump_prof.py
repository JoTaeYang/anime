import bpy, sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import profile_lib as PL

argv = sys.argv[sys.argv.index("--") + 1:]
mode = argv[0]           # "old" | "new"
outp = argv[1]

if mode == "new":
    src = r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx"
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=src)
    print("IMPORTED OBJECTS:")
    for o in bpy.data.objects:
        print("  ", o.name, o.type, "loc", [round(c, 5) for c in o.location],
              "rotE", [round(c, 5) for c in o.rotation_euler], "scale",
              [round(c, 6) for c in o.scale], "parent", o.parent.name if o.parent else None)
    obs = [o for o in bpy.data.objects if o.type == "MESH"]
    ob = obs[0]
    me = ob.data
    print("MESHES:", [o.name for o in obs], "| mesh datablocks:",
          [m.name for m in bpy.data.meshes])
    print("ARMATURES:", [o.name for o in bpy.data.objects if o.type == "ARMATURE"],
          "| armature data:", [a.name for a in bpy.data.armatures])
    print("verts", len(me.vertices), "edges", len(me.edges), "polys", len(me.polygons),
          "tris", sum(len(p.vertices) - 2 for p in me.polygons))
    print("uv_layers", [u.name for u in me.uv_layers], "| color_attrs",
          [c.name for c in me.color_attributes])
    print("materials", [(m.name if m else None) for m in me.materials])
    for m in me.materials:
        if m:
            print("  mat", m.name, "use_nodes", m.use_nodes,
                  "nodes", [n.type for n in m.node_tree.nodes] if m.use_nodes else None)
    print("images", [(i.name, i.filepath, i.size[:]) for i in bpy.data.images])
    print("vgroups", [g.name for g in ob.vertex_groups], "shapekeys",
          bool(me.shape_keys), "modifiers", [md.type for md in ob.modifiers])
    # topology
    import bmesh
    bm = bmesh.new(); bm.from_mesh(me)
    nonman_e = sum(1 for e in bm.edges if not e.is_manifold)
    nonman_v = sum(1 for v in bm.verts if not v.is_manifold)
    loose = sum(1 for v in bm.verts if not v.link_faces)
    ngons = sum(1 for f in bm.faces if len(f.verts) > 4)
    quads = sum(1 for f in bm.faces if len(f.verts) == 4)
    # islands
    seen = set(); isl = []
    for v in bm.verts:
        if v.index in seen:
            continue
        stack = [v]; seen.add(v.index); c = 0
        while stack:
            x = stack.pop(); c += 1
            for e in x.link_edges:
                o = e.other_vert(x)
                if o.index not in seen:
                    seen.add(o.index); stack.append(o)
        isl.append(c)
    isl.sort(reverse=True)
    print("nonmanifold_edges", nonman_e, "nonmanifold_verts", nonman_v,
          "loose_verts", loose, "ngons", ngons, "quads", quads,
          "islands", len(isl), isl[:10])
    bm.free()
    mw = ob.matrix_world
    P = [tuple(mw @ v.co) for v in me.vertices]
else:
    ob = bpy.data.objects["GOB_body"]
    me = ob.data
    print("OLD verts", len(me.vertices), "tris",
          sum(len(p.vertices) - 2 for p in me.polygons))
    mw = ob.matrix_world
    P = [tuple(mw @ v.co) for v in me.vertices]

prof, zmin, zmax = PL.profile(P)
print(PL.chart(prof, zmin, zmax, mode))
xs = [p[0] for p in P]; ys = [p[1] for p in P]
print("BBOX x %.5f..%.5f  y %.5f..%.5f  z %.5f..%.5f" %
      (min(xs), max(xs), min(ys), max(ys), zmin, zmax))
json.dump({"prof": prof, "zmin": zmin, "zmax": zmax,
           "bbox": [min(xs), max(xs), min(ys), max(ys), zmin, zmax]},
          open(outp, "w"))
print("WROTE", outp)
