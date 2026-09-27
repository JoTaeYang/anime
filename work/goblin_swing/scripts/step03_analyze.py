"""STEP 03 analysis - segment the mesh geometrically before skinning.
Run: blender -b goblin_v03_controls.blend --python step03_analyze.py
Read-only: never saves.
"""
import bpy, json, math, os
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect"
os.makedirs(OUT, exist_ok=True)

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
me = body.data

R = {}
R["body_obj"] = {
    "matrix_world": [[round(c, 6) for c in row] for row in body.matrix_world],
    "is_identity": body.matrix_world == body.matrix_world.Identity(4),
    "parent": body.parent.name if body.parent else None,
    "n_verts": len(me.vertices), "n_polys": len(me.polygons),
    "n_vgroups": len(body.vertex_groups),
    "modifiers": [m.type for m in body.modifiers],
}
R["rig_obj"] = {
    "matrix_world": [[round(c, 6) for c in row] for row in rig.matrix_world],
    "is_identity": rig.matrix_world == rig.matrix_world.Identity(4),
}

MW = body.matrix_world
co = [MW @ v.co for v in me.vertices]
N = len(co)

# ---------------------------------------------------------------- club axis
AX = Vector((-0.10609, 0.03071, 0.99388)).normalized()
GP = Vector((-0.6661, -0.3447, 0.9910))       # socket / grip point on the axis
HC_R = Vector((-0.676, -0.364, 0.992))        # right hand ball centre
HC_L = Vector((0.772, 0.056, 0.720))          # left hand ball centre

s_arr = [0.0] * N
d_arr = [0.0] * N
for i, p in enumerate(co):
    v = p - GP
    s = v.dot(AX)
    s_arr[i] = s
    d_arr[i] = (v - AX * s).length

# histogram of (s, d) for verts that are plausibly club/hand
bins = {}
for i in range(N):
    s, d = s_arr[i], d_arr[i]
    if d < 0.30 and -0.40 < s < 0.95:
        k = round(s, 2)
        b = bins.setdefault(k, {"n": 0, "dmin": 9, "dmax": 0, "hist": [0] * 16})
        b["n"] += 1
        b["dmin"] = min(b["dmin"], round(d, 4))
        b["dmax"] = max(b["dmax"], round(d, 4))
        b["hist"][min(15, int(d / 0.02))] += 1
R["axis_profile"] = {str(k): bins[k] for k in sorted(bins)}

# radial histogram inside the ball band |s| < 0.12
rad = [0] * 20
for i in range(N):
    if abs(s_arr[i]) < 0.12 and d_arr[i] < 0.20:
        rad[min(19, int(d_arr[i] / 0.01))] += 1
R["ball_band_radial_hist_1cm"] = rad

# ---------------------------------------------------------------- z profile
zh = {}
for p in co:
    k = round(p.z, 2)
    zh[k] = zh.get(k, 0) + 1
R["z_hist_1cm"] = {str(k): zh[k] for k in sorted(zh)}

# ---------------------------------------------------------------- bone segments
BONES = [b.name for b in arm.bones if b.use_deform]
SEG = {}
for n in BONES:
    b = arm.bones[n]
    SEG[n] = (rig.matrix_world @ b.head_local, rig.matrix_world @ b.tail_local)
R["deform_bones"] = BONES


def seg_dist(p, a, b):
    ab = b - a
    L2 = ab.length_squared
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, (p - a).dot(ab) / L2))
    return (p - (a + ab * t)).length


# ---------------------------------------------------------------- connectivity
# edge-connected components restricted to candidate sets are useful; build adjacency
adj = [[] for _ in range(N)]
for e in me.edges:
    a, b = e.vertices
    adj[a].append(b)
    adj[b].append(a)
R["mesh_edges"] = len(me.edges)

# non-manifold / loose check
deg = [len(a) for a in adj]
R["vert_degree"] = {"min": min(deg), "max": max(deg), "isolated": deg.count(0)}


def comps(mask):
    seen = [False] * N
    out = []
    for i in range(N):
        if mask[i] and not seen[i]:
            st = [i]
            seen[i] = True
            c = []
            while st:
                v = st.pop()
                c.append(v)
                for w in adj[v]:
                    if mask[w] and not seen[w]:
                        seen[w] = True
                        st.append(w)
            out.append(c)
    out.sort(key=len, reverse=True)
    return out


# whole-mesh island count
allmask = [True] * N
isl = comps(allmask)
R["islands"] = [len(c) for c in isl[:10]]

# ---------------------------------------------------------------- candidate club
# club = near the axis, outside the hand-ball band OR deep inside it
CLUB_R_OUT = 0.22     # generous radius away from the ball
CLUB_R_BALL = 0.09    # inside the ball band the shaft surface is much thinner
S_LO, S_HI = -0.345, 0.88   # butt (z=0.655) .. top (z=1.854)
cand = [False] * N
for i in range(N):
    s, d = s_arr[i], d_arr[i]
    if s < S_LO - 0.02 or s > S_HI + 0.03:
        continue
    if abs(s) < 0.135:
        cand[i] = d < CLUB_R_BALL
    else:
        cand[i] = d < CLUB_R_OUT
cc = comps(cand)
R["club_candidate"] = {"n": sum(cand), "components": [len(c) for c in cc[:8]]}
if cc:
    for k, c in enumerate(cc[:4]):
        pts = [co[i] for i in c]
        R["club_candidate"]["comp%d_bbox" % k] = [
            [round(min(p[a] for p in pts), 4) for a in range(3)],
            [round(max(p[a] for p in pts), 4) for a in range(3)],
        ]
        R["club_candidate"]["comp%d_s_range" % k] = [
            round(min(s_arr[i] for i in c), 4), round(max(s_arr[i] for i in c), 4)]

# ---------------------------------------------------------------- hand ball R
hb = [False] * N
for i in range(N):
    if (co[i] - HC_R).length < 0.175 and not cand[i]:
        hb[i] = True
hc = comps(hb)
R["handR_candidate"] = {"n": sum(hb), "components": [len(c) for c in hc[:8]]}

# how close is the club candidate to the non-club rest of the body?
minsep = 9.0
arg = None
clubset = set()
for c in cc[:1]:
    clubset = set(c)
for i in clubset:
    pass
# distance from club verts to nearest non-club, non-handball vert (coarse: grid)
grid = {}
CELL = 0.05
for i in range(N):
    if i in clubset or hb[i]:
        continue
    k = (int(co[i].x / CELL), int(co[i].y / CELL), int(co[i].z / CELL))
    grid.setdefault(k, []).append(i)
best = (9.0, -1, -1)
for i in list(clubset)[::7]:
    p = co[i]
    k0 = (int(p.x / CELL), int(p.y / CELL), int(p.z / CELL))
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for j in grid.get((k0[0] + dx, k0[1] + dy, k0[2] + dz), ()):
                    dd = (p - co[j]).length
                    if dd < best[0]:
                        best = (dd, i, j)
R["club_to_body_min_dist"] = {"d": round(best[0], 5),
                              "club_v": best[1], "other_v": best[2],
                              "club_p": [round(c, 4) for c in co[best[1]]] if best[1] >= 0 else None,
                              "other_p": [round(c, 4) for c in co[best[2]]] if best[2] >= 0 else None}

# ---------------------------------------------------------------- head crease
zh2 = {}
for p in co:
    if 1.20 < p.z < 1.45:
        k = round(p.z, 3)
        zh2[k] = zh2.get(k, 0) + 1
R["crease_z_hist_1mm"] = {str(k): zh2[k] for k in sorted(zh2)}
# cross-section radius (in xy about x=0,y=0.05) per z slice near the crease
cs = {}
for i, p in enumerate(co):
    if 1.15 < p.z < 1.50 and abs(p.x) < 0.45:
        k = round(p.z, 2)
        r = math.hypot(p.x, p.y - 0.05)
        c = cs.setdefault(k, [9, 0, 0])
        c[0] = min(c[0], r)
        c[1] = max(c[1], r)
        c[2] += 1
R["neck_xsection"] = {str(k): [round(v[0], 4), round(v[1], 4), v[2]] for k, v in sorted(cs.items())}

# ---------------------------------------------------------------- feet / ankle
fz = {}
for p in co:
    if p.z < 0.30 and p.x < 0:
        k = round(p.z, 2)
        f = fz.setdefault(k, [9, 0, 9, 0, 0])
        f[0] = min(f[0], p.x); f[1] = max(f[1], p.x)
        f[2] = min(f[2], p.y); f[3] = max(f[3], p.y)
        f[4] += 1
R["footR_z_profile"] = {str(k): [round(v[0], 3), round(v[1], 3), round(v[2], 3),
                                 round(v[3], 3), v[4]] for k, v in sorted(fz.items())}

# ---------------------------------------------------------------- belt band
bb = {}
for p in co:
    if 0.50 < p.z < 0.86:
        k = round(p.z, 2)
        b = bb.setdefault(k, [0, 0])
        b[0] += 1
        b[1] = max(b[1], round(math.hypot(p.x, p.y - 0.09), 4))
R["belt_profile"] = {str(k): v for k, v in sorted(bb.items())}

# ---------------------------------------------------------------- nearest-bone stats
near = {}
for i in range(0, N, 5):
    p = co[i]
    bestb, bd = None, 9.0
    for n in BONES:
        a, b = SEG[n]
        d = seg_dist(p, a, b)
        if d < bd:
            bd, bestb = d, n
    near[bestb] = near.get(bestb, 0) + 1
R["nearest_bone_counts_sampled5"] = near

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(OUT, "step03_analyze.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("DONE")
