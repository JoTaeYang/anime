"""STEP 03R - append the user's 12k-tri decimation as a LOW-POLY working proxy.

Run: blender -b goblin_v05_legfix.blend --python step03r_lowpoly.py
Saves back over goblin_v05_legfix.blend (or --out PATH).

  * appends `mesh_node_DECIMATED_12000` from the user's run folder (read-only),
  * applies the same +0.951933 m Z shift as STEP 01R, applies all transforms,
  * renames to GOB_body_lo / GOB_body_lo_mesh, reuses GOB_mat, smooth shading,
  * transfers the high-poly weights by nearest-face barycentric interpolation,
  * RE-IMPOSES the STEP 03 hard sets geometrically (club incl. butt, ring hand,
    left hand ball, head core incl. eyes/fangs, shoes, belt band),
  * prunes to <=4 influences and normalises,
  * binds to GOB_rig with the same Armature modifier settings (preserve volume ON),
  * organises collections GOB_hi / GOB_lo, low visible, high hidden in viewport+render.
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
SRC = (r"C:\Users\whxod\UVReviewProjects\Meshy_AI_Clubby_Goblin_0920122145_generate"
       r"\runs\run_70ade72b-9ac6-4f3e-bdfe-88840397144c\lowpoly.blend")
SRC_OBJ = "mesh_node_DECIMATED_12000"
SHIFT_Z = 0.951933

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[argv.index("--out") + 1] if "--out" in argv else \
    os.path.join(ROOT, "goblin_v05_legfix.blend")
REPORT = os.path.join(ROOT, "inspect", "step03r_lowpoly.json")

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)

R = {}
rig = bpy.data.objects["GOB_rig"]
hi = bpy.data.objects["GOB_body"]
hme = hi.data

# ================================================================ append
with bpy.data.libraries.load(SRC, link=False) as (src, dst):
    assert SRC_OBJ in src.objects, src.objects
    dst.objects = [SRC_OBJ]
lo = dst.objects[0]
bpy.context.scene.collection.objects.link(lo)
R["appended"] = {"name": lo.name, "verts": len(lo.data.vertices),
                 "polys": len(lo.data.polygons),
                 "loc": [round(c, 6) for c in lo.location],
                 "scale": [round(c, 6) for c in lo.scale]}

# ================================================================ shift + apply
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = lo
lo.select_set(True)
lo.parent = None
lo.matrix_world = lo.matrix_world.Identity(4)
lo.location = (0.0, 0.0, SHIFT_Z)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
lo.location = (0.0, 0.0, 0.0)

lo.name = "GOB_body_lo"
lo.data.name = "GOB_body_lo_mesh"
lme = lo.data
lme.materials.clear()
lme.materials.append(bpy.data.materials["GOB_mat"])
for p in lme.polygons:
    p.use_smooth = True
# Blender 5.x: auto-smooth is the "Smooth by Angle" modifier; the high-poly is
# fully smooth, so match it (no sharp edges marked).
R["hi_smooth_polys_pct"] = round(100.0 * sum(1 for p in hme.polygons if p.use_smooth)
                                 / len(hme.polygons), 2)
R["hi_modifiers"] = [(m.name, m.type) for m in hi.modifiers]

NL = len(lme.vertices)
PL = np.empty(NL * 3); lme.vertices.foreach_get("co", PL); PL = PL.reshape(NL, 3)
NH = len(hme.vertices)
PH = np.empty(NH * 3); hme.vertices.foreach_get("co", PH); PH = PH.reshape(NH, 3)
R["lo_verts"] = NL
R["lo_tris"] = len(lme.loop_triangles) if lme.loop_triangles else None
R["bbox_lo"] = [[round(float(v), 5) for v in PL.min(0)],
                [round(float(v), 5) for v in PL.max(0)]]
R["bbox_hi"] = [[round(float(v), 5) for v in PH.min(0)],
                [round(float(v), 5) for v in PH.max(0)]]

# ================================================================ hi weights
gi = {g.index: g.name for g in hi.vertex_groups}
WH = np.zeros((NH, NB), dtype=np.float64)
for v in hme.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in BI:
            WH[v.index, BI[n]] = g.weight

# ================================================================ transfer
hme.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in hme.loop_triangles], dtype=np.int32)
bvh = BVHTree.FromPolygons([Vector(p) for p in PH],
                           [tuple(int(i) for i in t) for t in TRI],
                           all_triangles=True, epsilon=0.0)
WL = np.zeros((NL, NB), dtype=np.float64)
dists = np.zeros(NL)
for i in range(NL):
    loc, nor, fi, dist = bvh.find_nearest(Vector(PL[i]))
    dists[i] = dist
    a, b, c = TRI[fi]
    A, B, C = PH[a], PH[b], PH[c]
    v0, v1, v2 = B - A, C - A, np.asarray(loc) - A
    d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
    d20, d21 = v2 @ v0, v2 @ v1
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-18:
        wb = np.array([1.0, 0.0, 0.0])
    else:
        vv = (d11 * d20 - d01 * d21) / den
        ww = (d00 * d21 - d01 * d20) / den
        wb = np.array([1.0 - vv - ww, vv, ww])
    wb = np.clip(wb, 0.0, 1.0)
    wb /= wb.sum()
    WL[i] = wb[0] * WH[a] + wb[1] * WH[b] + wb[2] * WH[c]
R["transfer_dist_m"] = {"max": round(float(dists.max()), 5),
                        "mean": round(float(dists.mean()), 6),
                        "p99": round(float(np.percentile(dists, 99)), 5)}

# ================================================================ hard sets (STEP 03)
E = np.empty(len(lme.edges) * 2, dtype=np.int32)
lme.edges.foreach_get("vertices", E); E = E.reshape(-1, 2)
ah = np.concatenate([E[:, 0], E[:, 1]])
at = np.concatenate([E[:, 1], E[:, 0]])
o_ = np.argsort(ah, kind="stable"); ah = ah[o_]; at = at[o_]
astart = np.searchsorted(ah, np.arange(NL + 1))


def components(mask):
    seen = np.zeros(NL, dtype=bool)
    out = []
    for i0 in np.nonzero(mask)[0]:
        if seen[i0]:
            continue
        st = [i0]; seen[i0] = True; c = []
        while st:
            v = st.pop(); c.append(v)
            for k in range(astart[v], astart[v + 1]):
                w = at[k]
                if mask[w] and not seen[w]:
                    seen[w] = True; st.append(w)
        out.append(np.array(c))
    out.sort(key=len, reverse=True)
    return out


AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
GP = np.array([-0.6661, -0.3447, 0.9910])
HC_R = np.array([-0.676, -0.364, 0.992])
HC_L = np.array([0.772, 0.056, 0.720])
S_BALL = 0.1134
V = PL - GP
S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
DHR = np.linalg.norm(PL - HC_R, axis=1)
DHL = np.linalg.norm(PL - HC_L, axis=1)
Z = PL[:, 2]

club_m = ((S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > S_BALL) & (D_AX < 0.22))
cc = components(club_m)
CLUB = np.zeros(NL, dtype=bool)
for c in cc:
    if len(c) >= 6:
        CLUB[c] = True
R["club_components"] = [int(len(c)) for c in cc[:6]]
HANDRB = (DHR <= 0.155) & (~CLUB)
HANDLB = (DHL <= 0.155)
FOOT_Z = 0.168
foot = (Z <= FOOT_Z) & (~CLUB) & (~HANDRB) & (~HANDLB)
FOOT_R_M = foot & (PL[:, 0] < 0)
FOOT_L_M = foot & (PL[:, 0] >= 0)
HEADCORE = (Z >= 1.375) & (~CLUB)
rT = np.hypot(PL[:, 0], PL[:, 1] - 0.088)
BELT = (Z >= 0.578) & (Z <= 0.782) & (rT < 0.45) & (~CLUB) & (~HANDRB) & (~HANDLB)

HARDSETS = [("WEAPON", CLUB), ("HAND_R", HANDRB), ("HAND_L", HANDLB),
            ("FOOT_R", FOOT_R_M), ("FOOT_L", FOOT_L_M),
            ("HEAD", HEADCORE), ("HIPS", BELT)]
for bn, m in HARDSETS:
    WL[m] = 0.0
    WL[m, BI[bn]] = 1.0
R["hard_sets"] = {bn: int(m.sum()) for bn, m in HARDSETS}

# ================================================================ prune/normalise
WL[WL < 0.004] = 0.0
part = np.argpartition(-WL, 4, axis=1)[:, 4:]
R["max_dropped_weight"] = float(np.take_along_axis(WL, part, axis=1).max())
np.put_along_axis(WL, part, 0.0, axis=1)
s = WL.sum(1)
bad = s <= 1e-9
R["zero_before_fix"] = int(bad.sum())
if bad.any():
    for i in np.nonzero(bad)[0]:
        loc, nor, fi, dist = bvh.find_nearest(Vector(PL[i]))
        WL[i] = WH[TRI[fi][0]]
    s = WL.sum(1)
WL = (WL.T / s).T
WL = np.round(WL, 5)
dom = np.argmax(WL, axis=1)
WL[np.arange(NL), dom] = 0.0
WL[np.arange(NL), dom] = 1.0 - WL.sum(1)

nz = WL > 0
R["influence_hist"] = [int((nz.sum(1) == k).sum()) for k in range(6)]
R["max_sum_error"] = float(np.abs(WL.sum(1) - 1.0).max())
R["verts_per_group"] = {BONES[j]: int(nz[:, j].sum()) for j in range(NB)}

# ================================================================ write + bind
for g in list(lo.vertex_groups):
    lo.vertex_groups.remove(g)
VG = {n: lo.vertex_groups.new(name=n) for n in BONES}
for j, n in enumerate(BONES):
    col = WL[:, j]
    idx = np.nonzero(col)[0]
    o2 = np.argsort(col[idx], kind="stable")
    idx = idx[o2]; vals = col[idx]
    starts = np.nonzero(np.diff(vals))[0] + 1
    for a_, b_ in zip(np.concatenate([[0], starts]), np.concatenate([starts, [len(idx)]])):
        VG[n].add([int(x) for x in idx[a_:b_]], float(vals[a_]), 'REPLACE')

for m in list(lo.modifiers):
    lo.modifiers.remove(m)
lo.parent = rig
lo.matrix_parent_inverse = rig.matrix_world.inverted()
md = lo.modifiers.new("Armature", 'ARMATURE')
md.object = rig
md.use_vertex_groups = True
md.use_bone_envelopes = False
md.use_deform_preserve_volume = True
R["modifier"] = {"object": md.object.name,
                 "preserve_volume": md.use_deform_preserve_volume}

# ================================================================ collections
sc = bpy.context.scene
for name, obs in (("GOB_hi", [hi]), ("GOB_lo", [lo])):
    col = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if col.name not in [c.name for c in sc.collection.children]:
        sc.collection.children.link(col)
    for ob in obs:
        for c in list(ob.users_collection):
            c.objects.unlink(ob)
        col.objects.link(ob)
bpy.data.collections["GOB_hi"].hide_viewport = True
bpy.data.collections["GOB_hi"].hide_render = True
bpy.data.collections["GOB_lo"].hide_viewport = False
bpy.data.collections["GOB_lo"].hide_render = False
hi.hide_viewport = True
hi.hide_render = True
lo.hide_viewport = False
lo.hide_render = False
vl = bpy.context.view_layer
for lc in vl.layer_collection.children:
    if lc.name == "GOB_hi":
        lc.hide_viewport = True
        lc.exclude = False
R["collections"] = {c.name: {"objs": [o.name for o in c.objects],
                             "hide_viewport": c.hide_viewport,
                             "hide_render": c.hide_render}
                    for c in bpy.data.collections}

# ================================================================ rest check
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ev = lo.evaluated_get(dg)
dm = ev.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
ev.to_mesh_clear()
d = np.linalg.norm(Q - PL, axis=1)
R["lo_rest_max_dev_m"] = float(d.max())
R["n_actions"] = len(bpy.data.actions)

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
