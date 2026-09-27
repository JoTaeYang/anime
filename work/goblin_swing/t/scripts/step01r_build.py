"""STEP 01R - align the T-pose goblin to the old world frame, separate the club,
repair the right hand, build the proxy, and save t/gobT_v01_prerig.blend.
Deterministic: rebuilds everything from the two read-only sources."""
import bpy, bmesh, math, json, os
from mathutils import Vector, Matrix

FBX = r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx"
LOW = r"C:\Users\whxod\UVReviewProjects\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate\runs\run_2a0171c1-f7f1-4d16-9c98-137d610c47df\lowpoly.blend"
OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\t\gobT_v01_prerig.blend"
REP = r"C:\Users\whxod\orca\anime\work\goblin_swing\t\inspect\step01\build_report.json"
rep = {}

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=FBX)
hi = bpy.data.objects["mesh_node"]
V = [v.co.copy() for v in hi.data.vertices]
ZB = min(v.z for v in V)
ARM_TOP = max(v.z for v in V if abs(v.x) > 0.50)      # 0.336: top of the arm/hand band

# ---- landmarks in FBX units (h = height above the sole) -------------------
H_FBX = max(v.z for v in V) - ZB
feet = [v for v in V if abs((v.z - ZB) - 0.033) < 0.010]
FEET_SPAN = max(v.x for v in feet) - min(v.x for v in feet)
head = [v for v in V if v.z > ARM_TOP + 0.02]          # strictly above the arms
HEAD_W = max(v.x for v in head) - min(v.x for v in head)
legR = [v for v in V if 0.135 < (v.z - ZB) < 0.235 and v.x < -0.02]
legL = [v for v in V if 0.135 < (v.z - ZB) < 0.235 and v.x > 0.02]
LEG_R = (max(v.x for v in legR) - min(v.x for v in legR)) / 2
LEG_CX = (min(v.x for v in legR) + max(v.x for v in legR)) / 2
LEG_CXL = (min(v.x for v in legL) + max(v.x for v in legL)) / 2
LEG_CY = (min(v.y for v in legR) + max(v.y for v in legR)) / 2
BELT_LO, BELT_HI = 0.4252, 0.5752      # sharp-crease clusters, probe2 section J

OLD = {"height": 1.903, "feet_span": 1.1824, "head_w": 0.780,
       "belt_c": 0.680, "belt_t": 0.204, "leg_r": 0.0755}
NEW = {"height": H_FBX, "feet_span": FEET_SPAN, "head_w": HEAD_W,
       "belt_c": (BELT_LO + BELT_HI) / 2, "belt_t": BELT_HI - BELT_LO, "leg_r": LEG_R}
S = sum(OLD[k] * NEW[k] for k in OLD) / sum(NEW[k] ** 2 for k in OLD)
rep["scale"] = S
rep["arm_top_fbx"] = round(ARM_TOP, 5)
rep["landmark_fit"] = {k: {"new_fbx": round(NEW[k], 5), "new_scaled": round(NEW[k] * S, 5),
                           "old_target": OLD[k],
                           "resid_mm": round((NEW[k] * S - OLD[k]) * 1000, 2),
                           "resid_pct": round(100 * (NEW[k] * S / OLD[k] - 1), 2)} for k in OLD}

core = [v for v in V if abs(v.x) < 0.46 and (v.z - ZB) < 0.90]
X_SYM = (((LEG_CX + LEG_CXL) / 2) +
         (min(v.x for v in core) + max(v.x for v in core)) / 2) / 2
OLD_LEG_CY = 0.106                     # old leg-tube y centre (measured on GOB_body)
loc = Vector((-X_SYM * S, OLD_LEG_CY - LEG_CY * S, -ZB * S))
rep["transform"] = {"scale": S, "loc": [round(c, 6) for c in loc],
                    "x_sym_fbx": round(X_SYM, 6), "leg_cy_fbx": round(LEG_CY, 6)}

hi.scale = (S, S, S); hi.location = loc; hi.rotation_euler = (0, 0, 0)

with bpy.data.libraries.load(LOW, link=False) as (src, dst):
    dst.objects = [n for n in src.objects if "DECIM" in n]
lo = dst.objects[0]
bpy.context.scene.collection.objects.link(lo)
lo.scale = (S, S, S); lo.location = loc; lo.rotation_euler = (0, 0, 0)

bpy.context.view_layer.update()
for ob in (hi, lo):
    ob.select_set(True); bpy.context.view_layer.objects.active = ob
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    ob.select_set(False)

W = [v.co.copy() for v in hi.data.vertices]
rep["after_align"] = {"bbox_min": [round(min(v[i] for v in W), 5) for i in range(3)],
                      "bbox_max": [round(max(v[i] for v in W), 5) for i in range(3)]}


def fx(x): return x * S + loc.x
def fy(y): return y * S + loc.y


# ---- club axis fit -------------------------------------------------------
XLIM, YF, YB = fx(-0.64), fy(0.00), fy(0.36)   # pure-club bands, hand fully excluded
pure = [v for v in W if v.x < XLIM and (v.y < YF or v.y > YB)]
rep["club_pure_verts"] = len(pure)
bands = {}
for v in pure:
    bands.setdefault(round(v.y / 0.02), []).append(v)
pts = [(k * 0.02, (min(v.x for v in s) + max(v.x for v in s)) / 2,
        (min(v.z for v in s) + max(v.z for v in s)) / 2)
       for k, s in sorted(bands.items()) if len(s) >= 20]


def linfit(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    m = (sum((a - mx) * (b - my) for a, b in zip(xs, ys)) /
         sum((a - mx) ** 2 for a in xs))
    return m, my - m * mx


ys = [p[0] for p in pts]
mx_, bx_ = linfit(ys, [p[1] for p in pts])
mz_, bz_ = linfit(ys, [p[2] for p in pts])
AXd = Vector((-mx_, -1.0, -mz_)).normalized()          # points toward the club head (-Y)
y_ref = sum(ys) / len(ys)
AXp = Vector((mx_ * y_ref + bx_, y_ref, mz_ * y_ref + bz_))
rep["club_axis"] = {"point": [round(c, 5) for c in AXp], "dir": [round(c, 5) for c in AXd],
                    "deg_from_minusY": round(math.degrees(
                        math.acos(AXd.dot(Vector((0, -1, 0))))), 3)}


def axial(v):
    d = Vector(v) - AXp
    t = d.dot(AXd)
    return t, (d - AXd * t).length


BIN = 0.008
acc, mx = {}, {}
for v in pure:
    t, r = axial(v)
    k = int(math.floor(t / BIN))
    a = acc.setdefault(k, [0.0, 0])
    a[0] += r; a[1] += 1
    mx[k] = max(mx.get(k, 0.0), r)
ks = sorted(acc)
# the bins that touch the hand gap pick up stray finger verts and read ~10 mm high;
# erode 3 bins either side of every gap so the interpolation is anchored to clean data.
lo_k, hi_k = ks[0], ks[-1]
miss = {k for k in range(lo_k, hi_k + 1) if k not in acc}
ero = {k for m in miss for k in range(m - 3, m + 4)} - {lo_k, hi_k}
for k in ero:
    acc.pop(k, None); mx.pop(k, None)
ks = sorted(acc)
# mean radius = unbiased estimate of the surface of revolution;  max = cutter
fm, fx_ = {}, {}
for k in range(ks[0], ks[-1] + 1):
    if k in acc:
        fm[k] = acc[k][0] / acc[k][1]; fx_[k] = mx[k]
    else:
        a = max(j for j in ks if j < k); b = min(j for j in ks if j > k)
        f = (k - a) / (b - a)
        fm[k] = (acc[a][0] / acc[a][1]) * (1 - f) + (acc[b][0] / acc[b][1]) * f
        fx_[k] = mx[a] * (1 - f) + mx[b] * f
KS = sorted(fm)
raw = [fm[k] for k in KS]
# median filter (kills per-bin sampling noise without biasing the steep club head)
med = []
for i in range(len(raw)):
    w = raw[max(0, i - 2):i + 3]
    med.append(sorted(w)[len(w) // 2])
smo = [sum(med[max(0, i - 1):i + 2]) / len(med[max(0, i - 1):i + 2])
       for i in range(len(med))]
for i in list(range(3)) + list(range(len(raw) - 3, len(raw))):
    smo[i] = raw[i]          # keep the real butt / tip caps
RIPPLE = max(abs(a - b) for a, b in zip(raw, smo))
STEP = max(abs(raw[i + 1] - raw[i]) for i in range(len(raw) - 1))
PROF = [(k * BIN + BIN / 2, r) for k, r in zip(KS, smo)]
CUTPROF = [(k * BIN + BIN / 2, max(fx_[k], r)) for k, r in zip(KS, smo)]
NOISE = max(fx_[k] - fm[k] for k in KS)
GAP = [k * BIN + BIN / 2 for k in range(ks[0], ks[-1] + 1) if k not in acc]

# grip centre: y-centroid of the real hand verts
hv = [v for v in W if v.x < fx(-0.66) and fy(0.05) < v.y < fy(0.33)]
GY = sum(v.y for v in hv) / len(hv)
t_g = (GY - AXp.y) / AXd.y
GRIP = AXp + AXd * t_g
rep["club"] = {
    "t_range_m": [round(PROF[0][0], 5), round(PROF[-1][0], 5)],
    "length_m": round(PROF[-1][0] - PROF[0][0], 5),
    "r_max_m": round(max(b for a, b in PROF), 5),
    "t_at_r_max": round(max(PROF, key=lambda p: p[1])[0], 5),
    "interpolated_gap_t": [round(GAP[0], 4), round(GAP[-1], 4)] if GAP else None,
    "grip_point": [round(c, 5) for c in GRIP], "grip_t": round(t_g, 5),
    "shaft_r_at_grip_m": round(min(PROF, key=lambda p: abs(p[0] - t_g))[1], 5),
    "butt_below_grip_m": round(t_g - PROF[0][0], 5),
    "head_above_grip_m": round(PROF[-1][0] - t_g, 5),
    "radius_profile": [[round(a, 4), round(b, 4)] for a, b in PROF]}


def revolve(name, samples, margin, seg=64, decimate=0.0):
    if decimate:      # keep a ring only where the radius actually changes
        keep = [samples[0]]
        for s in samples[1:-1]:
            if abs(s[1] - keep[-1][1]) > decimate or s[0] - keep[-1][0] > 0.06:
                keep.append(s)
        keep.append(samples[-1])
        samples = keep
    bm = bmesh.new()
    X = AXd.cross(Vector((0, 0, 1)))
    X = (X if X.length > 1e-6 else AXd.cross(Vector((1, 0, 0)))).normalized()
    Z = X.cross(AXd).normalized()
    sm = [(t, r + margin) for t, r in samples]
    r0, r1 = sm[0][1], sm[-1][1]
    sm = ([(sm[0][0] - r0, 1e-4), (sm[0][0] - r0 * 0.75, r0 * 0.66)] + sm +
          [(sm[-1][0] + r1 * 0.75, r1 * 0.66), (sm[-1][0] + r1, 1e-4)])
    rings = []
    for t, r in sm:
        c = AXp + AXd * t
        rings.append([bm.verts.new(c + (X * math.cos(a) + Z * math.sin(a)) * r)
                      for a in [2 * math.pi * i / seg for i in range(seg)]])
    for a, b in zip(rings, rings[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.faces.new(rings[0][::-1]); bm.faces.new(rings[-1])
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name + "_mesh")
    bm.to_mesh(me); bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    return o


# 3 mm: the fingers lie tangent to the shaft, so a hairline cut leaves knife-edge
# slivers.  3 mm puts the cut where hand and shaft are clearly separated.
MARGIN = 0.0050
rep["club"]["max_minus_mean_radius_mm"] = round(NOISE * 1000, 2)
rep["club"]["raw_profile_bin_step_mm"] = round(STEP * 1000, 2)
rep["club"]["median_filter_correction_mm"] = round(RIPPLE * 1000, 2)
cutter = revolve("CUTTER", CUTPROF, MARGIN)
club = revolve("GOB_club", PROF, 0.0, seg=32, decimate=0.0025)

pre = {tuple(round(c, 6) for c in v) for v in W}
for ob in (hi, lo):
    m = ob.modifiers.new("cutclub", 'BOOLEAN')
    m.operation = 'DIFFERENCE'; m.object = cutter
    try:
        m.solver = 'EXACT'
    except Exception:
        pass
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=m.name)
bpy.data.objects.remove(cutter, do_unlink=True)


def islands(bm):
    seen = set(); out = []
    bm.verts.ensure_lookup_table()
    for v in bm.verts:
        if v.index in seen:
            continue
        st = [v]; seen.add(v.index); grp = []
        while st:
            x = st.pop(); grp.append(x)
            for e in x.link_edges:
                o = e.other_vert(x)
                if o.index not in seen:
                    seen.add(o.index); st.append(o)
        out.append(grp)
    out.sort(key=len, reverse=True)
    return out


def clean(ob, tag):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    # cleanup is restricted to the cut region so the rest of the mesh stays bit-identical
    near = [v for v in bm.verts if axial(v.co)[1] < 0.30]
    bmesh.ops.dissolve_degenerate(
        bm, dist=2e-4, edges=[e for e in bm.edges
                              if axial(e.verts[0].co)[1] < 0.30])
    near = [v for v in bm.verts if axial(v.co)[1] < 0.30]
    bmesh.ops.remove_doubles(bm, verts=near, dist=2e-4)
    # relax the ragged cut rim: only verts the boolean created/moved
    fresh = [v for v in bm.verts
             if axial(v.co)[1] < 0.30 and tuple(round(c, 6) for c in v.co) not in pre]
    for _ in range(3):
        bmesh.ops.smooth_vert(bm, verts=fresh, factor=0.5,
                              use_axis_x=True, use_axis_y=True, use_axis_z=True)
    isl = islands(bm)
    killed = []
    for g in isl[1:]:
        c = sum((v.co for v in g), Vector()) / len(g)
        killed.append({"verts": len(g), "centroid": [round(x, 4) for x in c],
                       "axial": [round(x, 4) for x in axial(c)]})
        bmesh.ops.delete(bm, geom=g, context='VERTS')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data); ob.data.update()
    bm.free()
    bm = bmesh.new(); bm.from_mesh(ob.data)
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    loose = sum(1 for v in bm.verts if not v.link_faces)
    ni = len(islands(bm)); bm.free()
    P = [v.co for v in ob.data.vertices]
    rep[tag] = {"verts": len(ob.data.vertices),
                "tris": sum(len(p.vertices) - 2 for p in ob.data.polygons),
                "nonmanifold_edges": nm, "loose_verts": loose, "islands": ni,
                "removed_fragments": killed,
                "bbox_min": [round(min(p[i] for p in P), 5) for i in range(3)],
                "bbox_max": [round(max(p[i] for p in P), 5) for i in range(3)]}


clean(hi, "hi_after_cut")
clean(lo, "lo_after_cut")

# locality: verts far from the club must be untouched
far_t = far_c = 0
for v in hi.data.vertices:
    t, r = axial(v.co)
    if r > 0.15:
        far_t += 1
        if tuple(round(c, 6) for c in v.co) not in pre:
            far_c += 1
rep["locality"] = {"far_verts(r>0.15m)": far_t, "changed": far_c}

# tunnel / grip fit
gl, gh = (GAP[0], GAP[-1]) if GAP else (t_g - .1, t_g + .1)
near = [axial(v.co) for v in hi.data.vertices]
tun = [r for t, r in near if gl < t < gh]
rep["grip_fit"] = {
    "shaft_r_at_grip_mm": round(rep["club"]["shaft_r_at_grip_m"] * 1000, 2),
    "cutter_margin_mm": MARGIN * 1000,
    "tunnel_min_r_mm": round(min(tun) * 1000, 2),
    "hand_verts_inside_shaft_r": sum(
        1 for t, r in near if gl < t < gh and
        r < min(PROF, key=lambda p: abs(p[0] - t_g))[1] - 0.0005),
    "body_verts_within_club_surface_anywhere": sum(
        1 for t, r in near if PROF[0][0] < t < PROF[-1][0] and
        r < min(PROF, key=lambda p: abs(p[0] - t))[1] - 0.0005)}

# new club vs original club silhouette
dev = [abs(r - min(PROF, key=lambda p: abs(p[0] - t))[1])
       for t, r in (axial(v) for v in pure) if PROF[0][0] <= t <= PROF[-1][0]]
dev.sort()
rep["club_shape_vs_original"] = {
    "n": len(dev), "p50_mm": round(dev[len(dev) // 2] * 1000, 2),
    "p95_mm": round(dev[int(len(dev) * .95)] * 1000, 2),
    "max_mm": round(dev[-1] * 1000, 2)}

# ---- club object frame:  local +Y -> club head, origin at grip centre ----
Yl = AXd
Zl = (Vector((0, 0, 1)) - Yl * Yl.z).normalized()
Xl = Yl.cross(Zl).normalized()
M = Matrix((Xl, Yl, Zl)).transposed().to_4x4()
M.translation = GRIP
club.data.transform(M.inverted())
club.matrix_world = M
rep["club"]["object_matrix_world"] = [[round(c, 6) for c in r] for r in M]
Pc = [v.co for v in club.data.vertices]
rep["club"]["local_bbox"] = [[round(min(p[i] for p in Pc), 5) for i in range(3)],
                             [round(max(p[i] for p in Pc), 5) for i in range(3)]]
bm = bmesh.new(); bm.from_mesh(club.data)
rep["club"]["mesh"] = {"verts": len(bm.verts), "tris": len(bm.faces) * 0 + sum(
    len(f.verts) - 2 for f in bm.faces),
    "nonmanifold_edges": sum(1 for e in bm.edges if not e.is_manifold),
    "islands": len(islands(bm))}
bm.free()

# ---- naming / materials / collections / scene ---------------------------
hi.name = "GOB_body"; hi.data.name = "GOB_body_mesh"
lo.name = "GOB_body_lo"; lo.data.name = "GOB_body_lo_mesh"
mat = next(m for m in hi.data.materials if m is not None); mat.name = "GOB_mat"
for ob in (hi, lo):                 # the boolean adds an empty slot for the cutter
    ob.data.materials.clear(); ob.data.materials.append(mat)
    for p in ob.data.polygons:
        p.material_index = 0
cm = bpy.data.materials.new("GOB_club_mat"); cm.use_nodes = True
cm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.33, 0.21, 0.12, 1)
club.data.materials.append(cm)
club.data.name = "GOB_club_mesh"
for o in (hi, lo, club):
    for p in o.data.polygons:
        p.use_smooth = True

sc = bpy.context.scene
sc.render.fps = 24; sc.frame_start = 1; sc.frame_end = 39
for nm, o, hide in (("GOB_hi", hi, True), ("GOB_lo", lo, False),
                    ("GOB_weapon", club, False)):
    c = bpy.data.collections.new(nm)
    sc.collection.children.link(c)
    for cc in list(o.users_collection):
        cc.objects.unlink(o)
    c.objects.link(o)
    o.hide_viewport = hide; o.hide_render = hide

bpy.ops.wm.save_as_mainfile(filepath=OUT)
json.dump(rep, open(REP, "w"), indent=1)
short = {k: v for k, v in rep.items() if k != "club"}
short["club"] = {k: v for k, v in rep["club"].items() if k != "radius_profile"}
print(json.dumps(short, indent=1))
print("SAVED", OUT)
