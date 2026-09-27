"""s00_source_prep - normalize the Meshy Clay Goblin source mesh and split off the club.

Design doc d-23 section 0 (source asset) and section 7 G1 (G1.1-G1.4).

  input : goblib.SRC_FBX (read only)
  output: rig/gob_r00_source.blend   collection SRC = {SRC_hi, SRC_club_hi}
          rig/data/source_prep.json

Normalization: transforms baked into the mesh data (loc 0, rot 0, scale 1), size kept.
Translation only: min z = 0 and torso egg centre (x, y) = (0, 0), where the egg centre
is the mean of the area centroids of the largest closed loop of horizontal sections at
z in [0.30H, 0.60H].  The source already faces -Y, so rotate_z_deg = 0.

Club split: the club and the right fist are one fused shell (1 loose part), so the club
is cut with two planes perpendicular to the shaft axis, one just in front of the fist
(head side) and one just behind it (butt side); every opening is capped so each part is
closed.  SRC_club_hi = head + front shaft and butt (2 shells); the shaft inside the fist
stays in SRC_hi.

Run:  bl.ps1 -Script s00_source_prep.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import math  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r00_source.blend"
OUT_JSON = "source_prep.json"

# Club landmarks measured on the imported source (source frame, before translation):
# right-side region x < -0.6; clean shaft bands between club head and fist
# (y -0.20..0.00) and behind the fist (butt, y 0.34..0.44).
SRC_CLUB_X = -0.60
SRC_SHAFT_BANDS = ((-0.20, 0.00), (0.34, 0.44))
BAND = 0.01
NON_SHAFT_R = 0.060      # radius from the shaft axis above which a vert is not shaft
CUT_MARGIN = 0.003       # plane offset in front of the fist's most forward vertex
HEAD_R = 0.070           # club head region: median radius above this


# ---------------------------------------------------------------- mesh io
def mesh_co(me):
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    return co.reshape(-1, 3)


def mesh_tris(me):
    me.calc_loop_triangles()
    t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", t)
    return t.reshape(-1, 3)


# ---------------------------------------------------------------- torso centre
def section_loops(co, tris, z0):
    """Closed loops of the section z = z0 as lists of (x, y) points."""
    s = co[:, 2] - z0
    s[s == 0.0] = 1e-12
    pos = s[tris] > 0
    cross = tris[pos.any(1) & ~pos.all(1)]
    if len(cross) == 0:
        return []
    n = len(co)
    segs = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        va, vb = cross[:, a], cross[:, b]
        diff = (s[va] > 0) != (s[vb] > 0)
        key = np.where(diff, np.minimum(va, vb) * n + np.maximum(va, vb), -1)
        segs.append(key)
    segs = np.stack(segs, 1)
    pairs = np.sort(segs, 1)[:, 1:]  # the two crossing edges per triangle
    adj = {}
    for k1, k2 in pairs.tolist():
        adj.setdefault(k1, []).append(k2)
        adj.setdefault(k2, []).append(k1)

    def point(k):
        a, b = divmod(k, n)
        f = s[a] / (s[a] - s[b])
        return co[a, :2] + (co[b, :2] - co[a, :2]) * f

    loops, seen = [], set()
    for start in adj:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nb = adj[cur]
            nxt = nb[0] if nb[0] != prev else (nb[1] if len(nb) > 1 else None)
            if nxt is None or nxt == start:
                closed = nxt == start
                break
            if nxt in seen:
                closed = False
                break
            seen.add(nxt)
            loop.append(nxt)
            prev, cur = cur, nxt
        if closed and len(loop) >= 3:
            loops.append(np.array([point(k) for k in loop]))
    return loops


def poly_area_centroid(p):
    x, y = p[:, 0], p[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    c = x * y1 - x1 * y
    a = c.sum() / 2.0
    cx = ((x + x1) * c).sum() / (6.0 * a)
    cy = ((y + y1) * c).sum() / (6.0 * a)
    return abs(a), cx, cy


def torso_center(co, tris):
    zmin, zmax = co[:, 2].min(), co[:, 2].max()
    h = zmax - zmin
    cs = []
    for f in np.linspace(0.30, 0.60, 31):
        loops = section_loops(co, tris, zmin + f * h + 1e-7)
        best = max((poly_area_centroid(p) for p in loops), key=lambda r: r[0])
        cs.append(best)
    cs = np.array(cs)
    return float(cs[:, 1].mean()), float(cs[:, 2].mean()), cs


# ---------------------------------------------------------------- club
def fit_shaft_axis(co, T):
    """Shaft axis from bbox centres of clean shaft bands. d points to the club head (-Y)."""
    right = co[co[:, 0] < SRC_CLUB_X + T[0]]
    rows = []
    for y_lo, y_hi in SRC_SHAFT_BANDS:
        for y0 in np.arange(y_lo, y_hi - 1e-9, BAND):
            yy = y0 + T[1]
            s = right[(right[:, 1] >= yy) & (right[:, 1] < yy + BAND)]
            rows.append((yy + BAND / 2, (s[:, 0].min() + s[:, 0].max()) / 2,
                         (s[:, 2].min() + s[:, 2].max()) / 2))
    rows = np.array(rows)
    px = np.polyfit(rows[:, 0], rows[:, 1], 1)
    pz = np.polyfit(rows[:, 0], rows[:, 2], 1)
    d = np.array([-px[0], -1.0, -pz[0]])
    d /= np.linalg.norm(d)
    y_ref = rows[:, 0].mean()
    p0 = np.array([np.polyval(px, y_ref), y_ref, np.polyval(pz, y_ref)])
    return p0, d


def axial(co, p0, d):
    rel = co - p0
    t = rel @ d
    r = np.linalg.norm(rel - np.outer(t, d), axis=1)
    return t, r


def sphere_fit(p):
    A = np.c_[2 * p, np.ones(len(p))]
    b = (p ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:3]
    r = math.sqrt(sol[3] + c @ c)
    rms = float(np.sqrt(((np.linalg.norm(p - c, axis=1) - r) ** 2).mean()))
    return c, r, rms


def islands_of(bm):
    bm.verts.ensure_lookup_table()
    seen = np.zeros(len(bm.verts), dtype=bool)
    out = []
    for v in bm.verts:
        if seen[v.index]:
            continue
        grp, st = [], [v]
        seen[v.index] = True
        while st:
            x = st.pop()
            grp.append(x.index)
            for e in x.link_edges:
                o = e.other_vert(x)
                if not seen[o.index]:
                    seen[o.index] = True
                    st.append(o)
        out.append(grp)
    return out


def cap_holes(bm, p0, d, planes):
    """Fill every boundary loop, triangulate the caps and orient each cap outward.

    planes: {label: (t_plane, outward_sign)}; a cap belongs to the plane whose axial t is
    nearest to its centre and faces outward_sign * d.  Returns per-plane stats.
    """
    bnd = [e for e in bm.edges if e.is_boundary]
    new = bmesh.ops.holes_fill(bm, edges=bnd, sides=0)["faces"]
    stats = {k: {"boundary_edges": 0, "ngons": 0, "tris": 0} for k in planes}

    def plane_of(f):
        c = np.array(f.calc_center_median())
        tc = float((c - p0) @ d)
        return min(planes, key=lambda k: abs(planes[k][0] - tc))

    for f in new:
        st = stats[plane_of(f)]
        st["ngons"] += 1
        st["boundary_edges"] += len(f.edges)
    caps = bmesh.ops.triangulate(bm, faces=new, quad_method="BEAUTY",
                                 ngon_method="BEAUTY")["faces"]
    for f in caps:
        k = plane_of(f)
        stats[k]["tris"] += 1
        out = Vector(d * planes[k][1])
        f.normal_update()
        if f.normal.dot(out) < 0:
            f.normal_flip()
    return stats


def bisect_ring(bm, p0, d, t_plane, n):
    """Cut the shaft faces near axial t_plane with the plane (p0 + d*t_plane, n)."""
    bm.verts.ensure_lookup_table()
    co = np.array([v.co for v in bm.verts])
    t, r = axial(co, p0, d)
    win = set(np.nonzero((r < 0.075) & (np.abs(t - t_plane) < 0.03))[0].tolist())
    faces = [f for f in bm.faces if any(v.index in win for v in f.verts)]
    edges = list({e for f in faces for e in f.edges})
    verts = list({v for f in faces for v in f.verts})
    res = bmesh.ops.bisect_plane(bm, geom=faces + edges + verts, dist=1e-7,
                                 plane_co=Vector(p0 + d * t_plane), plane_no=Vector(n),
                                 clear_inner=False, clear_outer=False)
    cut_edges = [g for g in res["geom_cut"] if isinstance(g, bmesh.types.BMEdge)]
    cut_verts = {v for e in cut_edges for v in e.verts}
    deg = {}
    for e in cut_edges:
        for v in e.verts:
            deg[v] = deg.get(v, 0) + 1
    cut_r = [float(np.linalg.norm((np.array(v.co) - p0) - d * ((np.array(v.co) - p0) @ d)))
             for v in cut_verts]
    info = {"verts": len(cut_verts), "edges": len(cut_edges),
            "closed": all(k == 2 for k in deg.values()),
            "r_min": min(cut_r), "r_max": max(cut_r)}
    return cut_edges, info


def split_club(me, p0, d):
    co = mesh_co(me)
    t, r = axial(co, p0, d)
    near = r < 0.20
    # fist = non-shaft verts near the shaft axis, behind the club head
    fist = near & (r > NON_SHAFT_R) & (t < 0.10)
    t_fist = float(t[fist].max())
    t_cut = t_fist + CUT_MARGIN
    plane_co = p0 + d * t_cut
    plane_no = d.copy()
    # rear plane: just behind the fist's rearmost (butt side) non-shaft vertex
    t_fist_rear = float(t[fist].min())
    t_cut2 = t_fist_rear - CUT_MARGIN
    plane2_co = p0 + d * t_cut2
    plane2_no = -d

    bm = bmesh.new()
    bm.from_mesh(me)
    cut1, loop1 = bisect_ring(bm, p0, d, t_cut, plane_no)
    cut2, loop2 = bisect_ring(bm, p0, d, t_cut2, plane2_no)
    bmesh.ops.split_edges(bm, edges=cut1 + cut2)
    isl = islands_of(bm)
    bm.verts.ensure_lookup_table()
    club_set, n_club = set(), 0
    for g in isl:
        tt = (np.array([bm.verts[i].co for i in g]) - p0) @ d
        if tt.min() > t_cut - 1e-5 or tt.max() < t_cut2 + 1e-5:  # head side / butt side
            club_set.update(g)
            n_club += 1
    if len(isl) != 3 or n_club != 2:
        raise RuntimeError(f"plane cuts did not give body + 2 club shells: "
                           f"islands={len(isl)} club={n_club}")

    bm_club = bm.copy()
    bm_body = bm
    bm_club.verts.ensure_lookup_table()
    bmesh.ops.delete(bm_club, geom=[v for v in bm_club.verts if v.index not in club_set],
                     context="VERTS")
    bm_body.verts.ensure_lookup_table()
    bmesh.ops.delete(bm_body, geom=[v for v in bm_body.verts if v.index in club_set],
                     context="VERTS")
    cap_body = cap_holes(bm_body, p0, d, {"front": (t_cut, 1.0), "rear": (t_cut2, -1.0)})
    cap_club = cap_holes(bm_club, p0, d, {"front": (t_cut, -1.0), "rear": (t_cut2, 1.0)})
    info = {"t_fist_front": t_fist, "t_cut": t_cut,
            "t_fist_rear": t_fist_rear, "t_cut2": t_cut2,
            "front_loop": loop1, "rear_loop": loop2,
            "cap_body": cap_body, "cap_club": cap_club}
    return bm_body, bm_club, plane_co, plane_no, plane2_co, plane2_no, info


def bm_stats(bm):
    return {"nonmanifold_edges": sum(1 for e in bm.edges if not e.is_manifold),
            "boundary_edges": sum(1 for e in bm.edges if e.is_boundary),
            "islands": len(islands_of(bm))}


# ---------------------------------------------------------------- main
def main():
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.preferences.filepaths.save_version = 0
    fbx = goblib.SRC_FBX
    fbx_sha = goblib.sha256(fbx)
    bpy.ops.import_scene.fbx(filepath=str(fbx))
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if len(meshes) != 1:
        raise RuntimeError(f"expected 1 mesh in the FBX, got {[o.name for o in meshes]}")
    src = meshes[0]
    for o in list(bpy.data.objects):
        if o is not src:
            bpy.data.objects.remove(o, do_unlink=True)
    src.parent = None

    # bake transform into mesh data
    me = src.data
    me.transform(src.matrix_world)
    src.matrix_world = Matrix.Identity(4)
    me.update()

    # torso egg centre + ground (source frame, after bake)
    co = mesh_co(me)
    tris = mesh_tris(me)
    cx, cy, _ = torso_center(co, tris)
    zmin = float(co[:, 2].min())
    T = np.array([-cx, -cy, -zmin])
    rotate_z_deg = 0.0  # measured: face, club head and nose already on -Y
    me.transform(Matrix.Translation(Vector(T)))
    me.update()

    # club: loose parts?
    bm = bmesh.new()
    bm.from_mesh(me)
    n_loose = len(islands_of(bm))
    bm.free()
    if n_loose != 1:
        raise RuntimeError(f"source has {n_loose} loose parts; loose_parts split not implemented")
    method = "plane_cut"

    p0, d = fit_shaft_axis(mesh_co(me), T)
    bm_body, bm_club, plane_co, plane_no, plane2_co, plane2_no, cut_info = \
        split_club(me, p0, d)

    me_body = bpy.data.meshes.new("SRC_hi")
    bm_body.to_mesh(me_body)
    st_body = bm_stats(bm_body)
    bm_body.free()
    me_club = bpy.data.meshes.new("SRC_club_hi")
    bm_club.to_mesh(me_club)
    st_club = bm_stats(bm_club)
    bm_club.free()
    for m in (me_body, me_club):
        for mat in me.materials:
            m.materials.append(mat)

    # club head: sphere fit on the head region of the club shell
    cco = mesh_co(me_club)
    ct, cr = axial(cco, p0, d)
    edges_b = np.arange(ct.min(), ct.max() + 0.005, 0.005)
    idx = np.digitize(ct, edges_b)
    head_start = None
    for k in range(len(edges_b), 0, -1):
        m = idx == k
        if not m.any():
            continue
        if np.median(cr[m]) > HEAD_R:
            head_start = edges_b[k - 1]
        elif head_start is not None:
            break
    head_pts = cco[ct >= head_start]
    hc, hr, hrms = sphere_fit(head_pts)

    # objects / collection
    sc = bpy.context.scene
    col = bpy.data.collections.new("SRC")
    sc.collection.children.link(col)
    ob_body = bpy.data.objects.new("SRC_hi", me_body)
    ob_club = bpy.data.objects.new("SRC_club_hi", me_club)
    col.objects.link(ob_body)
    col.objects.link(ob_club)
    bpy.data.objects.remove(src, do_unlink=True)
    bpy.data.meshes.remove(me)
    bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)

    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND))

    doc = {"fbx": {"path": str(fbx), "sha256": fbx_sha},
           "transform": {"translate": [float(x) for x in T], "rotate_z_deg": rotate_z_deg,
                         "scale": 1.0},
           "club_split": {"method": method,
                          "plane_co": [float(x) for x in plane_co],
                          "plane_no": [float(x) for x in plane_no],
                          "plane2_co": [float(x) for x in plane2_co],
                          "plane2_no": [float(x) for x in plane2_no],
                          "club_head_center": [float(x) for x in hc],
                          "club_head_radius": float(hr)}}
    goblib.save_json(OUT_JSON, doc)

    # ---- report
    def ntris(m):
        m.calc_loop_triangles()
        return len(m.loop_triangles)

    bco = mesh_co(me_body)
    btc = torso_center(bco, mesh_tris(me_body))
    print("=== s00_source_prep ===")
    print(f"fbx sha256 {fbx_sha}")
    print(f"loose parts in source: {n_loose} -> method {method}")
    print(f"translate {np.round(T, 6).tolist()}  rotate_z_deg {rotate_z_deg}  scale 1.0")
    for name, m, stt in (("SRC_hi", me_body, st_body), ("SRC_club_hi", me_club, st_club)):
        c = mesh_co(m)
        print(f"{name}: verts {len(m.vertices)} tris {ntris(m)} "
              f"bbox {np.round(c.min(0), 5).tolist()} ~ {np.round(c.max(0), 5).tolist()} "
              f"{stt}")
    h = float(bco[:, 2].max() - bco[:, 2].min())
    print(f"SRC_hi height {h:.5f} m  min z {bco[:, 2].min():.6f}")
    print(f"torso egg centre after normalize x {btc[0]:.6f} y {btc[1]:.6f}")
    print(f"shaft axis p0 {np.round(p0, 5).tolist()} d {np.round(d, 5).tolist()} "
          f"deg_from_-Y {math.degrees(math.acos(-d[1])):.3f}")
    for k, v in cut_info.items():
        print(f"cut {k}: {v}")
    print(f"plane_co {np.round(plane_co, 5).tolist()} plane_no {np.round(plane_no, 5).tolist()}")
    print(f"plane2_co {np.round(plane2_co, 5).tolist()} "
          f"plane2_no {np.round(plane2_no, 5).tolist()}")
    # butt direction u = -d (axial coordinate along the butt, same shaft axis p0/d)
    bt, br = axial(bco, p0, d)
    near_b = br < 0.060
    ct_all, _ = axial(mesh_co(me_club), p0, d)
    print(f"butt-side axial (u=-d): SRC_hi max u {float((-bt).max()):.5f} "
          f"(verts r<60mm: {float((-bt[near_b]).max()):.5f}); plane2 u {-cut_info['t_cut2']:.5f}; "
          f"SRC_club_hi butt end u {float((-ct_all).max()):.5f}")
    print(f"club head: region t>={head_start:.4f} n={len(head_pts)} centre "
          f"{np.round(hc, 5).tolist()} radius {hr:.5f} fit_rms {hrms:.5f}")
    cmax_x = float(mesh_co(me_club)[:, 0].max())
    dist = np.linalg.norm(bco - hc, axis=1)
    print(f"club max x {cmax_x:.5f}; SRC_hi verts within 30 mm of head centre "
          f"{int((dist < 0.030).sum())}; min dist {dist.min():.5f}")
    print(f"objects {[o.name for o in bpy.data.objects]}  SRC collection "
          f"{[o.name for o in col.objects]}")
    print(f"saved {OUT_BLEND}")
    print(f"saved {goblib.DATA / OUT_JSON}")


if __name__ == "__main__":
    goblib.run_main(main)
