"""STEP 02 (new T-pose goblin): club touch-up + deform armature.

blender -b gobT_v01_prerig.blend --python step02_build_rig.py -- --out PATH
Never overwrites the input.
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join(T, "gobT_v02a_def.blend")
INSP = os.path.join(T, "inspect", "step02")
os.makedirs(INSP, exist_ok=True)
R = {}

hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
club = bpy.data.objects["GOB_club"]
sc = bpy.context.scene

CM = club.matrix_world.copy()
R["club_matrix_world_in"] = [[round(v, 6) for v in row] for row in CM]

# =====================================================================  measure
def world_pts(ob):
    m = ob.data
    n = len(m.vertices)
    P = np.empty(n * 3); m.vertices.foreach_get("co", P); P = P.reshape(n, 3)
    M = np.array(ob.matrix_world)
    return (np.c_[P, np.ones(n)] @ M.T)[:, :3]

P = world_pts(hi)

AX0 = np.array((0.0, 0.148, 1.266))          # nominal arm axis point at |x|=0.45


def arm_slice(sgn, x, rad=0.16):
    m = np.abs(P[:, 0] - sgn * x) < 0.006
    if m.sum() < 5:
        return None
    q = P[m]
    d = np.hypot(q[:, 1] - AX0[1], q[:, 2] - AX0[2])
    q = q[d < rad]
    if len(q) < 5:
        return None
    return q


MEAS = {}
for sgn, S in ((1, "L"), (-1, "R")):
    prof = []
    for x in np.arange(0.30, 0.90, 0.005):
        q = arm_slice(sgn, x)
        if q is None:
            continue
        cy0 = (q[:, 1].max() + q[:, 1].min()) / 2
        cz0 = (q[:, 2].max() + q[:, 2].min()) / 2
        rmx = float(np.hypot(q[:, 1] - cy0, q[:, 2] - cz0).max())
        prof.append((float(x), rmx,
                     float(q[:, 2].max() - q[:, 2].min()),
                     float(cy0), float(cz0)))
    # free tube = local outer radius close to the nominal tube radius 0.0688
    free = [p for p in prof if p[1] < 0.085]
    # exit = smallest |x| from which the tube stays free all the way out to 0.84
    xs = sorted(p[0] for p in free)
    exit_x = None
    for x in xs:
        if x > 0.84:
            break
        run = [p for p in prof if x <= p[0] <= 0.84]
        if all(p[1] < 0.090 for p in run):
            exit_x = x
            break
    # wrist = largest |x| where the section is still tube-sized
    wrist_x = max(p[0] for p in free if p[0] < 1.0)
    # axis fit over the clean tube
    fit = [p for p in prof if exit_x + 0.02 <= p[0] <= wrist_x - 0.02]
    A = np.array([p[0] for p in fit])
    cy = np.polyfit(A, [p[3] for p in fit], 1)
    cz = np.polyfit(A, [p[4] for p in fit], 1)
    rmean = float(np.mean([p[1] for p in fit]))
    # palm / fingers
    xm = float(np.abs(P[:, 0]).max()) if sgn > 0 else None
    side = P[np.sign(P[:, 0]) == sgn]
    tipx = float(np.abs(side[:, 0]).max())
    palm = []
    for x in np.arange(wrist_x, tipx, 0.01):
        m = np.abs(np.abs(P[:, 0]) - x) < 0.006
        q = P[m & (np.sign(P[:, 0]) == sgn)]
        if len(q) < 5:
            continue
        palm.append((round(float(x), 3), round(float(q[:, 1].max() - q[:, 1].min()), 4),
                     round(float(q[:, 2].max() - q[:, 2].min()), 4)))
    palm_x = max(palm, key=lambda p: p[1] * p[2])[0]
    MEAS[S] = {"exit_x": round(exit_x, 4), "wrist_x": round(wrist_x, 4),
               "tube_r": round(rmean, 4), "tip_x": round(tipx, 4),
               "palm_x": round(palm_x, 4),
               "axis_y": [round(float(v), 6) for v in cy],
               "axis_z": [round(float(v), 6) for v in cz],
               "slope_deg_down": round(math.degrees(math.atan(-cz[0])), 3),
               "slope_deg_back": round(math.degrees(math.atan(cy[0])), 3),
               "span_profile": [(round(p[0], 3), round(p[1], 4), round(p[2], 4)) for p in prof
                                if p[0] < 0.56]}
    # leg
    ls = []
    for z in np.arange(0.185, 0.365, 0.005):
        m = (np.abs(P[:, 2] - z) < 0.004) & (np.sign(P[:, 0]) == sgn) & (np.abs(P[:, 0]) > 0.16)
        q = P[m]
        if len(q) < 6:
            continue
        ls.append((float(z), float((q[:, 0].max() + q[:, 0].min()) / 2),
                   float((q[:, 1].max() + q[:, 1].min()) / 2),
                   float((q[:, 0].max() - q[:, 0].min()) / 2)))
    Z = np.array([p[0] for p in ls])
    lx = np.polyfit(Z, [p[1] for p in ls], 1)
    ly = np.polyfit(Z, [p[2] for p in ls], 1)
    shoe = P[(P[:, 2] < 0.13) & (np.sign(P[:, 0]) == sgn)]
    MEAS[S].update({
        "leg_x": [round(float(v), 6) for v in lx], "leg_y": [round(float(v), 6) for v in ly],
        "leg_r": round(float(np.mean([p[3] for p in ls])), 4),
        "shoe_min": shoe.min(0).round(4).tolist(), "shoe_max": shoe.max(0).round(4).tolist()})

R["measure"] = MEAS


def arm_axis(S, x):
    m = MEAS[S]
    sgn = 1 if S == "L" else -1
    return Vector((sgn * x, m["axis_y"][0] * x + m["axis_y"][1],
                   m["axis_z"][0] * x + m["axis_z"][1]))


def leg_axis(S, z):
    m = MEAS[S]
    return Vector((m["leg_x"][0] * z + m["leg_x"][1], m["leg_y"][0] * z + m["leg_y"][1], z))


# =====================================================================  club rebuild
PROF = json.load(open(os.path.join(T, "inspect", "step01", "build_report.json")))["club"]
RAW = PROF["radius_profile"]
T0, T1 = -0.900, 0.420
GRIP_T = PROF["grip_t"]
R_GRIP_TARGET = PROF["shaft_r_at_grip_m"]

tt = np.array([p[0] for p in RAW]); rr = np.array([p[1] for p in RAW])
core = (tt >= -0.856) & (tt <= 0.380)
ct, cr = tt[core], rr[core]


def smooth(a, w=7, n=3):
    k = np.ones(w) / w
    for _ in range(n):
        pad = np.r_[np.full(w // 2, a[0]), a, np.full(w // 2, a[-1])]
        a = np.convolve(pad, k, 'valid')
    return a


def pava(y, inc=True):
    y = list(y if inc else y[::-1])
    lev, wt = [], []
    for v in y:
        lev.append(v); wt.append(1.0)
        while len(lev) > 1 and lev[-2] > lev[-1]:
            v2 = (lev[-1] * wt[-1] + lev[-2] * wt[-2]) / (wt[-1] + wt[-2])
            w2 = wt[-1] + wt[-2]
            lev = lev[:-2] + [v2]; wt = wt[:-2] + [w2]
    out = []
    for v, w in zip(lev, wt):
        out += [v] * int(round(w))
    out = np.array(out[:len(y)])
    return out if inc else out[::-1]


PK = 0.228
up = ct <= PK
cs = smooth(cr, 7, 3)
cs[up] = pava(cs[up], True)
cs[~up] = pava(cs[~up], False)
cs = smooth(cs, 5, 2)
# re-pin the grip radius (taper the correction out over the shaft)
r_now = float(np.interp(GRIP_T, ct, cs))
d = R_GRIP_TARGET - r_now
wgt = np.clip((0.06 - ct) / 0.32, 0.0, 1.0)
cs = cs + d * wgt
R["club_grip_correction_mm"] = round(d * 1000, 4)
f = PROF["r_max_m"] / float(cs.max())
cs = cs * (1.0 + (f - 1.0) * np.clip((ct - 0.02) / 0.18, 0.0, 1.0))
R["club_head_peak_gain"] = round(float(f), 5)


TA = 0.360
_ra = float(np.interp(TA, ct, cs))
_sa = (float(np.interp(TA + 0.004, ct, cs)) - float(np.interp(TA - 0.004, ct, cs))) / 0.008
CC = TA + _sa * _ra
RHO = math.hypot(_ra, TA - CC)
KCAP = (T1 - TA) / (CC + RHO - TA)
R["club_tip_arc"] = {"r_at_TA": round(_ra, 5), "slope": round(_sa, 4),
                     "rho": round(RHO, 5), "k": round(KCAP, 5)}


def Rad(t):
    if t <= -0.856:                      # butt cap
        rc = float(np.interp(-0.856, ct, cs)); a = -0.856 - T0
        u = min(1.0, (-0.856 - t) / a)
        return rc * math.sqrt(max(0.0, 1 - u * u))
    if t >= TA:                          # tip: circular arc, tangent at TA, 0 at T1
        u = (t - TA) / KCAP + (TA - CC)
        return math.sqrt(max(0.0, RHO * RHO - u * u))
    return float(np.interp(t, ct, cs))


TS = (list(np.linspace(T0, -0.856, 8))[1:] + list(np.linspace(-0.856, -0.26, 23))[1:]
      + list(np.linspace(-0.26, 0.06, 11))[1:] + list(np.linspace(0.06, 0.380, 23))[1:]
      + list(np.linspace(0.380, T1, 10))[1:-1])
TS = [T0] + TS
NSEG = 28
verts, faces = [], []
verts.append((0.0, T0 - GRIP_T, 0.0))             # butt pole
for t in TS[1:]:
    r = max(1e-4, Rad(t))
    for i in range(NSEG):
        a = 2 * math.pi * i / NSEG
        verts.append((r * math.cos(a), t - GRIP_T, r * math.sin(a)))
verts.append((0.0, T1 - GRIP_T, 0.0))             # tip pole
NR = len(TS) - 1
for i in range(NSEG):
    faces.append((0, 1 + (i + 1) % NSEG, 1 + i))
for k in range(NR - 1):
    b0 = 1 + k * NSEG; b1 = b0 + NSEG
    for i in range(NSEG):
        j = (i + 1) % NSEG
        faces.append((b0 + i, b0 + j, b1 + j, b1 + i))
last = 1 + (NR - 1) * NSEG
tip = len(verts) - 1
for i in range(NSEG):
    faces.append((last + i, last + (i + 1) % NSEG, tip))

oldme = club.data
newme = bpy.data.meshes.new("GOB_club_mesh_v2")
newme.from_pydata(verts, [], faces)
newme.update()
newme.validate()
for m in oldme.materials:
    newme.materials.append(m)
for p in newme.polygons:
    p.use_smooth = True

# ---- before/after render of the club alone
sc.render.engine = 'BLENDER_WORKBENCH'
sh = sc.display.shading
sh.light = 'STUDIO'; sh.color_type = 'SINGLE'; sh.single_color = (0.72, 0.72, 0.74)
sh.show_cavity = True; sh.cavity_type = 'BOTH'; sh.show_object_outline = False
sc.display.render_aa = '8'
sc.render.resolution_x = 1000; sc.render.resolution_y = 500
sc.render.image_settings.file_format = 'PNG'
cd = bpy.data.cameras.new("cl_cam"); cd.type = 'ORTHO'; cd.ortho_scale = 1.45
cam = bpy.data.objects.new("cl_cam", cd); sc.collection.objects.link(cam); sc.camera = cam
ctr = CM @ Vector((0, (T0 + T1) / 2 - GRIP_T, 0))
cam.location = ctr + Vector((0, 0, 6))
cam.rotation_mode = 'XYZ'
cam.rotation_euler = (0, 0, math.radians(90))
for o in bpy.context.view_layer.objects:
    o.hide_render = (o is not club)
for c in bpy.data.collections:
    c.hide_render = False
for nm, meh in (("club_before", oldme), ("club_after", newme)):
    club.data = meh
    sc.render.filepath = os.path.join(INSP, nm + ".png")
    bpy.ops.render.render(write_still=True)
for o in bpy.context.view_layer.objects:
    o.hide_render = False
hi.hide_render = True
bpy.data.objects.remove(cam)
club.data = newme
oldme.user_clear()

# ---- club checks
cme = club.data
cme.calc_loop_triangles()
ntri = len(cme.loop_triangles)
nonman = sum(1 for e in cme.edges if len(
    [p for p in cme.polygons if e.key in [tuple(sorted(k)) for k in p.edge_keys]]) != 2)
# faster manifold check via edge->face count
ef = {}
for p in cme.polygons:
    for k in p.edge_keys:
        ef[tuple(sorted(k))] = ef.get(tuple(sorted(k)), 0) + 1
nonman = sum(1 for v in ef.values() if v != 2)
# islands
adj = {i: set() for i in range(len(cme.vertices))}
for k in ef:
    adj[k[0]].add(k[1]); adj[k[1]].add(k[0])
seen = set(); isl = 0
for i in range(len(cme.vertices)):
    if i in seen:
        continue
    isl += 1; st = [i]; seen.add(i)
    while st:
        u = st.pop()
        for v in adj[u]:
            if v not in seen:
                seen.add(v); st.append(v)
R["club"] = {"verts": len(cme.vertices), "tris": ntri, "nonmanifold_edges": nonman,
             "islands": isl, "len_m": round(T1 - T0, 4),
             "r_grip_m": round(Rad(GRIP_T), 5), "r_max_m": round(max(Rad(t) for t in TS), 5),
             "head_dia_m": round(2 * max(Rad(t) for t in TS), 4),
             "bbox_local": [[round(float(np.min([v[k] for v in verts])), 5) for k in range(3)],
                            [round(float(np.max([v[k] for v in verts])), 5) for k in range(3)]],
             "rings": NR, "seg": NSEG}

# body-vs-club interference + fist bore clearance
CI = np.array(CM.inverted())
HL = (np.c_[P, np.ones(len(P))] @ CI.T)[:, :3]        # body verts in club local space
rad_h = np.hypot(HL[:, 0], HL[:, 2])
HT = HL[:, 1] + GRIP_T                                 # body verts in the profile t frame
inside = (HT > T0) & (HT < T1)
Rv = np.array([Rad(float(t)) for t in HT])
pen = np.where(inside, Rv - rad_h, -1.0)
R["club"]["body_verts_inside_club"] = int((pen > 0).sum())
R["club"]["max_penetration_mm"] = round(float(pen.max()) * 1000, 3)
# bore = body verts near the shaft over the fist span
bore = inside & (rad_h < 0.12) & (HL[:, 1] > -0.14) & (HL[:, 1] < 0.14)
R["club"]["bore_verts"] = int(bore.sum())
if bore.any():
    cl = (rad_h[bore] - Rv[bore])
    R["club"]["bore_min_clearance_mm"] = round(float(cl.min()) * 1000, 3)
    R["club"]["bore_min_r_mm"] = round(float(rad_h[bore].min()) * 1000, 3)
    R["club"]["bore_localy_span"] = [round(float(HL[bore][:, 1].min()), 4),
                                     round(float(HL[bore][:, 1].max()), 4)]
R["club"]["pen_pts"] = [[round(float(v), 4) for v in HL[i]] for i in
                        np.argsort(-pen)[:6] if pen[i] > 0]
R["club"]["fillet_scan"] = {S: MEAS[S]["span_profile"] for S in ("L", "R")}

# =====================================================================  bone table
SPY = 0.105
CREASE = 1.310
B = {}      # name: (head, tail, parent, connect, deform)
B["ROOT"] = ((0.0, 0.0, 0.0), (0.0, 0.35, 0.0), None, False, False)
B["COG"] = ((0.0, SPY, 0.80), (0.0, SPY, 0.98), "ROOT", False, False)
B["HIPS"] = ((0.0, SPY, 0.55), (0.0, SPY, 0.75), "COG", False, True)
B["SPINE_01"] = ((0.0, SPY, 0.75), (0.0, SPY, 1.01), "HIPS", True, True)
B["CHEST"] = ((0.0, SPY, 1.01), (0.0, SPY, 1.265), "SPINE_01", True, True)
B["NECK"] = ((0.0, SPY, 1.265), (0.0, SPY, CREASE), "CHEST", True, True)
B["HEAD"] = ((0.0, SPY, CREASE), (0.0, SPY + 0.005, 1.855), "NECK", True, True)

FIT = {}
for S in ("L", "R"):
    m = MEAS[S]
    sh_x = m["exit_x"] - 0.0688      # one nominal tube radius inboard of the exit
    sh = arm_axis(S, sh_x)
    wr = arm_axis(S, m["wrist_x"])
    el = (sh + wr) * 0.5 + Vector((0, 0.010, 0))
    hd_t = arm_axis(S, (m["palm_x"] + m["tip_x"]) * 0.5)
    B["UPPERARM_" + S] = (tuple(sh), tuple(el), "CHEST", False, True)
    B["FOREARM_" + S] = (tuple(el), tuple(wr), "UPPERARM_" + S, True, True)
    B["HAND_" + S] = (tuple(wr), tuple(hd_t), "FOREARM_" + S, True, True)
    hip = leg_axis(S, 0.405)
    ank = leg_axis(S, 0.175)
    kn = (hip + ank) * 0.5 + Vector((0, -0.010, 0))
    ft_t = Vector((ank.x + (((m["shoe_min"][0] + m["shoe_max"][0]) / 2) - ank.x) * 0.5,
                   m["shoe_min"][1] + 0.105, 0.072))
    B["THIGH_" + S] = (tuple(hip), tuple(kn), "HIPS", False, True)
    B["SHIN_" + S] = (tuple(kn), tuple(ank), "THIGH_" + S, True, True)
    B["FOOT_" + S] = (tuple(ank), tuple(ft_t), "SHIN_" + S, True, True)
    FIT[S] = {"shoulder_x": round(sh_x, 4), "shoulder": [round(v, 5) for v in sh],
              "elbow": [round(v, 5) for v in el], "wrist": [round(v, 5) for v in wr],
              "hand_tail": [round(v, 5) for v in hd_t],
              "hip": [round(v, 5) for v in hip], "knee": [round(v, 5) for v in kn],
              "ankle": [round(v, 5) for v in ank], "foot_tail": [round(v, 5) for v in ft_t]}
R["fit_per_side"] = FIT

# mirror residuals: how far the mirrored-L joints sit from the per-side R fit
MIR = {}
for k in ("shoulder", "elbow", "wrist", "hand_tail", "hip", "knee", "ankle", "foot_tail"):
    a = Vector(FIT["L"][k]); a.x *= -1
    b = Vector(FIT["R"][k])
    MIR[k] = {"d_mm": round((a - b).length * 1000, 2),
              "dx_mm": round((a.x - b.x) * 1000, 2), "dy_mm": round((a.y - b.y) * 1000, 2),
              "dz_mm": round((a.z - b.z) * 1000, 2)}
R["mirror_residual"] = MIR
R["mirror_max_mm"] = max(v["d_mm"] for v in MIR.values())
R["fit_mode"] = "per-side (mirror residual > 8 mm)" if R["mirror_max_mm"] > 8 else "mirrored"

# weapon socket from the club object axes
CX = CM.to_3x3().col[0].normalized()
CY = CM.to_3x3().col[1].normalized()
CZ = CM.to_3x3().col[2].normalized()
GRIP = CM.to_translation()
B["WEAPON_SOCKET"] = (tuple(GRIP), tuple(GRIP + CY * 0.12), "HAND_R", False, False)
B["WEAPON"] = (tuple(GRIP), tuple(GRIP + CY * PROF["head_above_grip_m"]), "WEAPON_SOCKET",
               False, False)

ORDER = ["ROOT", "COG", "HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L", "UPPERARM_R", "FOREARM_R", "HAND_R",
         "WEAPON_SOCKET", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]

# =====================================================================  build
arm_data = bpy.data.armatures.new("GOB_rig_data")
rig = bpy.data.objects.new("GOB_rig", arm_data)
sc.collection.objects.link(rig)
rig.location = (0, 0, 0)
rig.rotation_mode = 'QUATERNION'
rig.rotation_quaternion = (1, 0, 0, 0)
rig.scale = (1, 1, 1)
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
eb = arm_data.edit_bones
for n in ORDER:
    h, t, par, conn, dfm = B[n]
    b = eb.new(n); b.head = Vector(h); b.tail = Vector(t); b.use_deform = dfm
for n in ORDER:
    h, t, par, conn, dfm = B[n]
    if par:
        eb[n].parent = eb[par]; eb[n].use_connect = conn


def set_local_x(bone, xdir):
    y = (bone.tail - bone.head).normalized()
    x = Vector(xdir)
    x = x - y * x.dot(y)
    x.normalize()
    bone.align_roll(x.cross(y))


WX = Vector((1, 0, 0))


def bend_normal(u, f):
    a = (eb[u].tail - eb[u].head).normalized()
    b = (eb[f].tail - eb[f].head).normalized()
    return a.cross(b).normalized()


NRM = {"L": bend_normal("UPPERARM_L", "FOREARM_L"), "R": bend_normal("UPPERARM_R", "FOREARM_R")}
R["bend_normal"] = {S: [round(v, 5) for v in NRM[S]] for S in NRM}
for n in ORDER:
    if n in ("WEAPON_SOCKET", "WEAPON"):
        set_local_x(eb[n], CX)
    elif n[-2:] in ("_L", "_R") and n.split("_")[0] in ("UPPERARM", "FOREARM", "HAND"):
        set_local_x(eb[n], NRM[n[-1]])
    else:
        set_local_x(eb[n], WX)

ROLLS = {n: round(math.degrees(eb[n].roll), 4) for n in ORDER}
AXES = {n: {"x": [round(v, 5) for v in eb[n].x_axis], "y": [round(v, 5) for v in eb[n].y_axis],
            "z": [round(v, 5) for v in eb[n].z_axis]} for n in ORDER}
bpy.ops.object.mode_set(mode='OBJECT')

# weapon-socket axis fidelity
sock = arm_data.bones["WEAPON_SOCKET"].matrix_local.to_3x3()
R["weapon_socket_axis_err_deg"] = {
    "Y_vs_club_Y": round(math.degrees((sock.col[1].normalized()).angle(CY)), 5),
    "Z_vs_club_Z": round(math.degrees((sock.col[2].normalized()).angle(CZ)), 5)}

# =====================================================================  parent club
club.parent = rig
club.parent_type = 'BONE'
club.parent_bone = 'WEAPON'
club.matrix_parent_inverse = Matrix.Identity(4)
bpy.context.view_layer.update()
club.matrix_world = CM
bpy.context.view_layer.update()
dev = max(abs(club.matrix_world[i][j] - CM[i][j]) for i in range(4) for j in range(4))
R["club_parent"] = {"parent": club.parent.name, "type": club.parent_type,
                    "bone": club.parent_bone, "world_matrix_max_dev": float(dev)}
# rest offset of the club relative to the WEAPON bone (used by the gate)
WB = rig.matrix_world @ arm_data.bones["WEAPON"].matrix_local
R["club_rest_offset"] = [[round(v, 8) for v in row] for row in (WB.inverted() @ CM)]

# =====================================================================  verify
hi.data.calc_loop_triangles()
bvh = BVHTree.FromPolygons([Vector(p) for p in P],
                           [tuple(t.vertices) for t in hi.data.loop_triangles],
                           all_triangles=True, epsilon=0.0)
RAYS = [Vector(v) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (0.577, 0.577, 0.577),
                            (-0.577, 0.3, 0.75), (0.26, -0.8, 0.54), (-0.3, -0.5, -0.81))]


def inside_and_dist(p):
    votes = 0
    for d in RAYS:
        cur = Vector(p) + d * 1e-5
        n = 0
        for _ in range(90):
            hit = bvh.ray_cast(cur, d)
            if hit[0] is None:
                break
            n += 1
            cur = hit[0] + d * 1e-5
        if n % 2 == 1:
            votes += 1
    loc, nor, idx, dist = bvh.find_nearest(Vector(p))
    return (votes >= 4, votes, round(dist, 5) if loc else None)


jt = []
for n in ORDER:
    if n in ("ROOT",):
        continue
    b = arm_data.bones[n]
    pts = [("head", b.head_local)]
    if n in ("HAND_L", "HAND_R", "FOOT_L", "FOOT_R", "HEAD"):
        pts.append(("tail", b.tail_local))
    for w, p in pts:
        ins, votes, dist = inside_and_dist(p)
        jt.append({"bone": n, "pt": w, "p": [round(c, 5) for c in p],
                   "inside": ins, "votes": votes, "dist_surf": dist})
R["joint_tests"] = jt
R["joints_outside"] = [j for j in jt if not j["inside"] and not j["bone"].startswith("WEAPON")]
R["joints_tight"] = [j for j in jt if j["inside"] and j["dist_surf"] is not None
                     and j["dist_surf"] < 0.010]

bones = []
for n in ORDER:
    b = arm_data.bones[n]
    bones.append({"name": n, "head": [round(c, 5) for c in b.head_local],
                  "tail": [round(c, 5) for c in b.tail_local], "len": round(b.length, 5),
                  "roll_deg": ROLLS[n], "parent": b.parent.name if b.parent else None,
                  "connect": b.use_connect, "deform": b.use_deform, "axes": AXES[n]})
R["bones"] = bones
R["zero_len"] = [b["name"] for b in bones if b["len"] < 1e-4]
R["hierarchy_ok"] = all(b["parent"] == B[b["name"]][2] and b["connect"] == B[b["name"]][3]
                        and b["deform"] == B[b["name"]][4] for b in bones)
R["deform_bones"] = [b["name"] for b in bones if b["deform"]]
R["mesh_untouched"] = {"hi_mods": [m.type for m in hi.modifiers], "hi_vg": len(hi.vertex_groups),
                       "lo_mods": [m.type for m in lo.modifiers], "lo_vg": len(lo.vertex_groups),
                       "hi_parent": hi.parent.name if hi.parent else None,
                       "lo_parent": lo.parent.name if lo.parent else None}
R["n_actions"] = len(bpy.data.actions)

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(T, "inspect", "step02_rig.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
