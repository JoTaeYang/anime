"""s02d_finish.py - T11: merge body + rigid parts into GOB_mesh (part_id), custom normals, UVs.

Input : rig/work/r01c_rigid.blend (GOB_body_tmp, GOB_head, GOB_hand_l, GOB_hand_r, GOB_shoe_l, GOB_shoe_r,
        GOB_belt, GOB_club; SRC_* ignored), rig/work/loops_limbs.json
Output: rig/work/r01d_merged.blend (collection GOB: GOB_mesh, GOB_club only; HR2 contract: input of s02e_mouth.py,
        which writes the canonical rig/gob_r01_retopo.blend), rig/data/parts.json, rig/data/retopo_loops.json,
        rig/work/r01d_uv.png, rig/work/r01d_front.png

Run:  bl.ps1 -Script s02d_finish.py -Blend work/r01c_rigid.blend

GOB_mesh: vertices of GOB_body_tmp first in their original order (ring indices of loops_limbs.json stay
valid; checked by position and remapped if not), then head, hand_l, hand_r, shoe_l, shoe_r, belt; faces
copied polygon by polygon (no topology change); face INT attribute part_id (PARTS); identity transform.
Normals (GOB_mesh, GOB_club): smooth shading, sharp edges from angle (SHARP_ANGLE, Mesh.set_sharp_from_angle;
the parts are separate shells, so body / rigid boundaries never share normals), Weighted Normal modifier
(face area, keep sharp) applied -> custom normals, no modifier left.
Head part (T113c, d-29 HR1.3): sharp edges = only the designed hard edges tagged by s02c (EDGE bool HARD_ATTR on
GOB_head: eye junction / wall-bottom loops, fang outlines and corners); head corner normals = Blender's default
smooth corner normals with those sharp edges (corner-angle weighted, no face-area weighting), stored as custom
normals; the stored codes of every non-head corner are restored unchanged after the head write.
UVs: Smart UV Project (UV_ANGLE_LIMIT) -> seams from islands -> angle-based unwrap on those seams ->
average island scale -> islands holding head-front faces (normal . -Y > HEAD_FRONT_DOT) scaled by
HEAD_FRONT_SCALE (linear) -> pack (margin PACK_MARGIN, rotation allowed) into 0..1.
GOB_club keeps its object transform (origin grip_r, local +Y = axis); only its mesh data is edited.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402

IN_LOOPS = goblib.WORK / "loops_limbs.json"
OUT_BLEND = goblib.WORK / "r01d_merged.blend"   # HR2: s02e_mouth.py -> gob_r01_retopo.blend
OUT_UV = goblib.WORK / "r01d_uv.png"
OUT_FRONT = goblib.WORK / "r01d_front.png"
COLL = "GOB"
BODY = "GOB_body_tmp"
PARTS = {"body": 0, "head": 1, "hand_l": 2, "hand_r": 3, "shoe_l": 4, "shoe_r": 5, "belt": 6}
PART_OBJ = {"body": BODY, "head": "GOB_head", "hand_l": "GOB_hand_l", "hand_r": "GOB_hand_r",
            "shoe_l": "GOB_shoe_l", "shoe_r": "GOB_shoe_r", "belt": "GOB_belt"}
CLUB = "GOB_club"
SHARP_ANGLE = math.radians(60.0)
UV_ANGLE_LIMIT = math.radians(66.0)
PACK_MARGIN = 0.005
HEAD_FRONT_DOT = 0.7
HEAD_FRONT_SCALE = 1.45      # linear -> ~2.1 x area density before packing
HARD_ATTR = "gob_hard_edge"  # s02c: designed hard edges on GOB_head (T113c)
UV_RES = 2048
UV_MATCH = 1e-6


def mm(x):
    return round(float(x) * 1000.0, 2)


# ---------------------------------------------------------------- merge
def mesh_data(ob):
    me = ob.data
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    faces = [tuple(p.vertices) for p in me.polygons]
    return V, faces


def build_gob_mesh():
    verts, faces, part = [], [], []
    offsets = {}
    hard_pairs = []
    nv = 0
    for name in PARTS:
        ob = bpy.data.objects.get(PART_OBJ[name])
        if ob is None or ob.type != "MESH":
            raise RuntimeError(f"missing input object {PART_OBJ[name]}")
        if not np.allclose(np.array(ob.matrix_world), np.eye(4), atol=1e-9):
            raise RuntimeError(f"{ob.name}: transform is not identity")
        V, F = mesh_data(ob)
        if name == "head" and ob.data.attributes.get(HARD_ATTR) is not None:
            Eh = np.empty(len(ob.data.edges) * 2, dtype=np.int64)
            ob.data.edges.foreach_get("vertices", Eh)
            fl = np.zeros(len(ob.data.edges), dtype=bool)
            ob.data.attributes[HARD_ATTR].data.foreach_get("value", fl)
            hard_pairs += [(int(a) + nv, int(b) + nv) for a, b in Eh.reshape(-1, 2)[fl].tolist()]
        offsets[name] = (nv, len(V))
        verts.append(V)
        faces += [tuple(i + nv for i in f) for f in F]
        part += [PARTS[name]] * len(F)
        nv += len(V)
    V = np.vstack(verts)
    me = bpy.data.meshes.new("GOB_mesh")
    me.from_pydata(V.tolist(), [], faces)
    me.update()
    if len(me.polygons) != len(faces) or len(me.vertices) != len(V):
        raise RuntimeError("from_pydata changed the element counts")
    att = me.attributes.new("part_id", "INT", "FACE")
    att.data.foreach_set("value", np.array(part, dtype=np.int32))
    ob = bpy.data.objects.new("GOB_mesh", me)
    bpy.context.scene.collection.objects.link(ob)
    return ob, offsets, hard_pairs


def remap_loops(ob, body_V):
    with open(IN_LOOPS, "r", encoding="utf-8") as f:
        src = json.load(f)
    me = ob.data
    X = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", X)
    X = X.reshape(-1, 3)
    rings, n_remap = {}, 0
    for name, r in src["rings"].items():
        vs = []
        for v in r["verts"]:
            if v < len(body_V) and np.linalg.norm(X[v] - body_V[v]) < 1e-9:
                vs.append(int(v))
            else:
                vs.append(int(np.argmin(np.linalg.norm(X - body_V[v], axis=1))))
                n_remap += 1
        rings[name] = {"verts": vs, "joint": r["joint"], "k": r["k"], "center": r["center"]}
    E = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    nv = len(X)
    ek = set((np.minimum(E[:, 0], E[:, 1]) * nv + np.maximum(E[:, 0], E[:, 1])).tolist())
    closed = {}
    for name, r in rings.items():
        a = np.array(r["verts"])
        b = np.roll(a, -1)
        closed[name] = all(k in ek for k in (np.minimum(a, b) * nv + np.maximum(a, b)).tolist())
    return {"mesh": "GOB_mesh", "rings": rings}, n_remap, closed


# ---------------------------------------------------------------- normals
def custom_normals(ob):
    me = ob.data
    me.shade_smooth()
    me.set_sharp_from_angle(angle=SHARP_ANGLE)
    n_sharp = 0
    att = me.attributes.get("sharp_edge")
    if att is not None:
        buf = np.zeros(len(me.edges), dtype=bool)
        att.data.foreach_get("value", buf)
        n_sharp = int(buf.sum())
    mod = ob.modifiers.new("wn", "WEIGHTED_NORMAL")
    mod.mode = "FACE_AREA"
    mod.weight = 50
    mod.keep_sharp = True
    with bpy.context.temp_override(object=ob, active_object=ob, selected_objects=[ob]):
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return n_sharp


def head_normals(ob, hard_pairs, head_value):
    """T113c head rule (see the module docstring); returns (head edges, head sharp edges, head corners)."""
    me = ob.data
    nl = len(me.loops)
    pv = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    head_l = np.repeat(pv, lt) == head_value
    raw0 = np.empty(nl * 2, dtype=np.int32)
    me.attributes["custom_normal"].data.foreach_get("value", raw0)
    raw0 = raw0.reshape(-1, 2)
    cn0 = np.empty(nl * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", cn0)
    cn0 = cn0.reshape(-1, 3)
    E = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    le = np.empty(nl, dtype=np.int32)
    me.loops.foreach_get("edge_index", le)
    head_e = np.zeros(len(me.edges), dtype=bool)
    head_e[le[head_l]] = True
    key = {(min(a, b), max(a, b)): i for i, (a, b) in enumerate(E.tolist())}
    hard = np.zeros(len(me.edges), dtype=bool)
    for a, b in hard_pairs:
        hard[key[(min(a, b), max(a, b))]] = True
    if (hard & ~head_e).any():
        raise RuntimeError("hard edges outside the head part")
    sh = np.zeros(len(me.edges), dtype=bool)
    if me.attributes.get("sharp_edge") is None:
        me.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
    me.attributes["sharp_edge"].data.foreach_get("value", sh)
    sh[head_e] = hard[head_e]
    me.attributes["sharp_edge"].data.foreach_set("value", sh)
    me2 = me.copy()
    me2.attributes.remove(me2.attributes["custom_normal"])
    cn2 = np.empty(nl * 3, dtype=np.float32)
    me2.corner_normals.foreach_get("vector", cn2)
    bpy.data.meshes.remove(me2)
    target = cn0.copy()
    target[head_l] = cn2.reshape(-1, 3)[head_l]
    me.normals_split_custom_set(target.tolist())
    raw1 = np.empty(nl * 2, dtype=np.int32)
    me.attributes["custom_normal"].data.foreach_get("value", raw1)
    raw1 = raw1.reshape(-1, 2)
    raw1[~head_l] = raw0[~head_l]
    me.attributes["custom_normal"].data.foreach_set("value", raw1.ravel())
    return int(head_e.sum()), int(sh[head_e].sum()), int(head_l.sum())


# ---------------------------------------------------------------- uv
def uv_islands_bm(bm, uvl):
    """Island id per face (faces joined across edges whose endpoint UVs match)."""
    parent = list(range(len(bm.faces)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for e in bm.edges:
        lf = e.link_loops
        if len(lf) != 2:
            continue
        l1, l2 = lf
        a1, b1 = l1[uvl].uv, l1.link_loop_next[uvl].uv
        a2, b2 = l2.link_loop_next[uvl].uv, l2[uvl].uv    # opposite winding
        if (a1 - a2).length <= UV_MATCH and (b1 - b2).length <= UV_MATCH:
            ra, rb = find(l1.face.index), find(l2.face.index)
            if ra != rb:
                parent[ra] = rb
    return [find(i) for i in range(len(bm.faces))]


def make_uvs(ob, head_front_value=None):
    """Smart project -> seams from islands -> unwrap -> average scale -> head front scale -> pack."""
    me = ob.data
    if not me.uv_layers:
        me.uv_layers.new(name="UVMap")
    bpy.context.view_layer.objects.active = ob
    for o in bpy.context.view_layer.objects:
        o.select_set(o is ob)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=UV_ANGLE_LIMIT, island_margin=0.0, area_weight=0.0,
                             correct_aspect=True, scale_to_bounds=False)
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.seams_from_islands(mark_seams=True, mark_sharp=False)
    bpy.ops.uv.unwrap(method="ANGLE_BASED", margin=PACK_MARGIN)
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.average_islands_scale()
    bpy.ops.object.mode_set(mode="OBJECT")
    n_scaled = 0
    if head_front_value is not None:
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.faces.ensure_lookup_table()
        uvl = bm.loops.layers.uv.active
        isl = uv_islands_bm(bm, uvl)
        pid = me.attributes["part_id"]
        pv = np.empty(len(me.polygons), dtype=np.int32)
        pid.data.foreach_get("value", pv)
        front = set()
        for f in bm.faces:
            if pv[f.index] == head_front_value and f.normal.dot((0.0, -1.0, 0.0)) > HEAD_FRONT_DOT:
                front.add(isl[f.index])
        for island in front:
            fs = [f for f in bm.faces if isl[f.index] == island]
            pts = [l[uvl].uv.copy() for f in fs for l in f.loops]
            cx = sum(p.x for p in pts) / len(pts)
            cy = sum(p.y for p in pts) / len(pts)
            for f in fs:
                for l in f.loops:
                    uv = l[uvl].uv
                    l[uvl].uv = (cx + (uv.x - cx) * HEAD_FRONT_SCALE, cy + (uv.y - cy) * HEAD_FRONT_SCALE)
            n_scaled += 1
        bm.to_mesh(me)
        bm.free()
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.select_all(action="SELECT")
    bpy.ops.uv.pack_islands(margin=PACK_MARGIN, rotate=True, scale=True)
    bpy.ops.object.mode_set(mode="OBJECT")
    return n_scaled


# ---------------------------------------------------------------- uv report (same definitions as check_g2_static)
def uv_report(ob, head_value=None, body_value=None):
    me = ob.data
    nl, npoly = len(me.loops), len(me.polygons)
    uv = np.empty(nl * 2, dtype=np.float32)
    me.uv_layers.active.uv.foreach_get("vector", uv)
    uv = uv.reshape(-1, 2).astype(np.float64)
    bm = bmesh.new()
    bm.from_mesh(me)
    uvl = bm.loops.layers.uv.active
    isl = np.array(uv_islands_bm(bm, uvl))
    bm.free()
    _, isl = np.unique(isl, return_inverse=True)
    me.calc_loop_triangles()
    nt = len(me.loop_triangles)
    tl = np.empty(nt * 3, dtype=np.int32)
    tp = np.empty(nt, dtype=np.int32)
    tv = np.empty(nt * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("loops", tl)
    me.loop_triangles.foreach_get("polygon_index", tp)
    me.loop_triangles.foreach_get("vertices", tv)
    tl, tv = tl.reshape(-1, 3), tv.reshape(-1, 3)
    owner = np.full((UV_RES, UV_RES), -1, dtype=np.int32)
    conflict = np.zeros((UV_RES, UV_RES), dtype=bool)
    T = uv[tl] * UV_RES
    for t in range(nt):
        p = T[t]
        a2 = (p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[1, 1] - p[0, 1]) * (p[2, 0] - p[0, 0])
        if abs(a2) < 1e-12:
            continue
        x0 = max(int(math.floor(p[:, 0].min() - 0.5)), 0)
        x1 = min(int(math.ceil(p[:, 0].max() - 0.5)), UV_RES - 1)
        y0 = max(int(math.floor(p[:, 1].min() - 0.5)), 0)
        y1 = min(int(math.ceil(p[:, 1].max() - 0.5)), UV_RES - 1)
        if x1 < x0 or y1 < y0:
            continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        s = 1.0 if a2 > 0 else -1.0
        inside = np.ones(X.shape, bool)
        for i in range(3):
            q0, q1 = p[i], p[(i + 1) % 3]
            inside &= s * ((q1[0] - q0[0]) * (Y - q0[1]) - (q1[1] - q0[1]) * (X - q0[0])) >= 0
        il = int(isl[tp[t]])
        sub = owner[y0:y1 + 1, x0:x1 + 1]
        csub = conflict[y0:y1 + 1, x0:x1 + 1]
        csub[inside & (sub >= 0) & (sub != il)] = True
        sub[inside & (sub < 0)] = il
    rep = {"uv_min": uv.min(0).round(5).tolist(), "uv_max": uv.max(0).round(5).tolist(),
           "loops_out_of_0_1": int((np.any(uv < -1e-6, axis=1) | np.any(uv > 1 + 1e-6, axis=1)).sum()),
           "islands": int(isl.max()) + 1, "overlap_pixels": int(conflict.sum()),
           "covered_pixels": int((owner >= 0).sum())}
    if head_value is not None:
        V = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", V)
        V = V.reshape(-1, 3)
        P3 = V[tv]
        cr = np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0])
        a3 = 0.5 * np.linalg.norm(cr, axis=1)
        pn = np.zeros((npoly, 3))
        np.add.at(pn, tp, cr)
        pn /= np.maximum(np.linalg.norm(pn, axis=1), 1e-20)[:, None]
        U = uv[tl]
        auv = 0.5 * np.abs((U[:, 1, 0] - U[:, 0, 0]) * (U[:, 2, 1] - U[:, 0, 1])
                           - (U[:, 1, 1] - U[:, 0, 1]) * (U[:, 2, 0] - U[:, 0, 0]))
        pv = np.empty(npoly, dtype=np.int32)
        me.attributes["part_id"].data.foreach_get("value", pv)
        head = (pv == head_value) & (pn @ np.array([0.0, -1.0, 0.0]) > HEAD_FRONT_DOT)
        body = pv == body_value
        dens = {}
        for key, m in (("head_front", head), ("body", body)):
            mt = m[tp]
            dens[key] = float(auv[mt].sum() / a3[mt].sum())
        rep["density_head_front"] = dens["head_front"]
        rep["density_body"] = dens["body"]
        rep["density_ratio_area"] = dens["head_front"] / dens["body"]
        rep["head_front_faces"] = int(head.sum())
    rep["_owner"] = owner
    return rep


def write_uv_png(owners, path):
    """Side-by-side island maps (island colour hash), each UV_RES/2 px; row 0 = image top."""
    half = UV_RES // 2
    tiles = []
    for own in owners:
        o = own[::2, ::2]
        rgb = np.zeros(o.shape + (3,), dtype=np.float32)
        m = o >= 0
        h = (o[m].astype(np.int64) * 2654435761) & 0xFFFFFF
        rgb[m, 0] = 0.25 + 0.75 * ((h & 0xFF) / 255.0)
        rgb[m, 1] = 0.25 + 0.75 * (((h >> 8) & 0xFF) / 255.0)
        rgb[m, 2] = 0.25 + 0.75 * (((h >> 16) & 0xFF) / 255.0)
        tiles.append(rgb)
    img = np.concatenate(tiles, axis=1)                   # (half, half * n, 3), row 0 = v 0
    h, w = img.shape[:2]
    rgba = np.ones((h, w, 4), dtype=np.float32)
    rgba[:, :, :3] = img
    im = bpy.data.images.new("_s02d_uv", width=w, height=h, alpha=True)
    im.pixels.foreach_set(rgba.ravel())                   # blender rows start at the bottom = v 0
    im.filepath_raw = str(path)
    im.file_format = "PNG"
    im.save()
    bpy.data.images.remove(im)
    return path


def normals_info(ob):
    me = ob.data
    nl = len(me.loops)
    buf = np.empty(nl * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", buf)
    cn = buf.reshape(nl, 3)
    return {"has_custom_normals": bool(me.has_custom_normals),
            "nonfinite": int((~np.all(np.isfinite(cn), axis=1)).sum()),
            "modifiers": [m.type for m in ob.modifiers]}


# ---------------------------------------------------------------- main
def main():
    club = bpy.data.objects.get(CLUB)
    if club is None or club.type != "MESH":
        raise RuntimeError(f"missing input object {CLUB}")
    club_mw = club.matrix_world.copy()
    body_V, _ = mesh_data(bpy.data.objects[BODY])
    gm, offsets, hard_pairs = build_gob_mesh()
    loops, n_remap, closed = remap_loops(gm, body_V)

    n_sharp_m = custom_normals(gm)
    head_rule = head_normals(gm, hard_pairs, PARTS["head"]) if hard_pairs else None
    n_sharp_c = custom_normals(club)
    n_front_isl = make_uvs(gm, head_front_value=PARTS["head"])
    make_uvs(club)

    # ---- data json
    goblib.save_json("parts.json", PARTS)
    goblib.save_json("retopo_loops.json", loops)

    # ---- keep only GOB_mesh / GOB_club in collection GOB
    coll = bpy.data.collections.get(COLL)
    if coll is None:
        coll = bpy.data.collections.new(COLL)
        bpy.context.scene.collection.children.link(coll)
    for ob in list(bpy.data.objects):
        if ob not in (gm, club):
            bpy.data.objects.remove(ob, do_unlink=True)
    for c in list(bpy.data.collections):
        if c is not coll:
            bpy.data.collections.remove(c)
    for ob in (gm, club):
        for c in list(ob.users_collection):
            c.objects.unlink(ob)
        coll.objects.link(ob)
    bpy.data.orphans_purge(do_recursive=True)
    if (np.abs(np.array(club.matrix_world) - np.array(club_mw)) > 1e-9).any():
        raise RuntimeError("GOB_club matrix_world changed")

    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)

    # ---- reports
    me = gm.data
    me.calc_loop_triangles()
    tp = np.empty(len(me.loop_triangles), dtype=np.int32)
    me.loop_triangles.foreach_get("polygon_index", tp)
    pv = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pv)
    club.data.calc_loop_triangles()
    print(f"[s02d] OUTPUT blend={OUT_BLEND} parts={goblib.DATA / 'parts.json'} loops={goblib.DATA / 'retopo_loops.json'}")
    print(f"[s02d] objects in blend: {sorted(o.name for o in bpy.data.objects)}; collections: "
          f"{sorted(c.name for c in bpy.data.collections)}; meshes: {sorted(m.name for m in bpy.data.meshes)}")
    print(f"[s02d] GOB_mesh: verts={len(me.vertices)} faces={len(me.polygons)} tris={len(me.loop_triangles)} "
          f"matrix identity={bool(np.allclose(np.array(gm.matrix_world), np.eye(4)))}")
    print("[s02d] GOB_mesh per part (value: faces / tris / vert range): " + ", ".join(
        f"{n}={v}: {int((pv == v).sum())} / {int((pv[tp] == v).sum())} / "
        f"[{offsets[n][0]},{offsets[n][0] + offsets[n][1] - 1}]" for n, v in PARTS.items()))
    print(f"[s02d] part_id attribute: {me.attributes['part_id'].domain}/{me.attributes['part_id'].data_type}, "
          f"values present={sorted(set(pv.tolist()))}")
    print(f"[s02d] GOB_club: tris={len(club.data.loop_triangles)} matrix_world loc="
          f"{np.round(np.array(club.matrix_world)[:3, 3], 5).tolist()} unchanged=True")
    sh = np.zeros(len(me.edges), dtype=bool)
    if me.attributes.get("sharp_edge") is not None:
        me.attributes["sharp_edge"].data.foreach_get("value", sh)
    le = np.empty(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("edge_index", le)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    edge_part = np.full(len(me.edges), -1)
    edge_part[le] = np.repeat(pv, lt)
    print(f"[s02d] NORMALS GOB_mesh {normals_info(gm)} sharp edges (>{round(math.degrees(SHARP_ANGLE))} deg)="
          f"{n_sharp_m} per part " + str({n: int((sh & (edge_part == v)).sum()) for n, v in PARTS.items()})
          + f"; GOB_club {normals_info(club)} sharp edges={n_sharp_c}")
    print(f"[s02d] HEAD normals (T113c): " + (f"hard-edge tags from {HARD_ATTR}={len(hard_pairs)}; head edges / sharp / "
          f"corners = {head_rule[0]} / {head_rule[1]} / {head_rule[2]}; default smooth corner normals with those sharp "
          f"edges stored; non-head corner codes restored" if head_rule else "no tags on GOB_head -> angle rule"))
    rm = uv_report(gm, PARTS["head"], PARTS["body"])
    rc = uv_report(club)
    print(f"[s02d] UV GOB_mesh: range {rm['uv_min']}..{rm['uv_max']} out_of_0_1={rm['loops_out_of_0_1']} "
          f"islands={rm['islands']} overlap_pixels({UV_RES}^2)={rm['overlap_pixels']} "
          f"covered={rm['covered_pixels']} head-front islands scaled={n_front_isl} (x{HEAD_FRONT_SCALE} linear) "
          f"head_front faces={rm['head_front_faces']} density head_front/body (area)={round(rm['density_ratio_area'], 3)} "
          f"(head {rm['density_head_front']:.5f}, body {rm['density_body']:.5f} uv/m2)")
    print(f"[s02d] UV GOB_club: range {rc['uv_min']}..{rc['uv_max']} out_of_0_1={rc['loops_out_of_0_1']} "
          f"islands={rc['islands']} overlap_pixels({UV_RES}^2)={rc['overlap_pixels']} covered={rc['covered_pixels']}")
    print(f"[s02d] LOOPS rings={len(loops['rings'])} remapped verts={n_remap} all closed={all(closed.values())}"
          + ("" if all(closed.values()) else f" open={[n for n, c in closed.items() if not c]}"))
    for name, r in loops["rings"].items():
        print(f"   {name}: n={len(r['verts'])} closed={closed[name]} k={r['k']} center={r['center']}")

    # ---- renders
    write_uv_png([rm["_owner"], rc["_owner"]], OUT_UV)
    goblib.ortho_render([gm, club], "front", OUT_FRONT)
    print(f"[s02d] RENDERS {OUT_UV} (left GOB_mesh, right GOB_club; island colours) {OUT_FRONT}")
    print("[s02d] done")


if __name__ == "__main__":
    goblib.run_main(main)
