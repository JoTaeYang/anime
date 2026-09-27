"""GATE 03 - deformation test for gobT_v03_skinned.blend.  NEVER saves.

blender -b t\gobT_v03_skinned.blend --python t\scripts\s3_gate.py -- \
        [--mesh lo|hi] [--res 768] [--sets ABC] [--norender] [--tag name]
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Quaternion, Matrix
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d, cast=str):
    return cast(argv[argv.index(n) + 1]) if n in argv else d


MESH = opt("--mesh", "lo")
RES = opt("--res", 768, int)
SETS = opt("--sets", "ABC")
TAG = opt("--tag", MESH)
NORENDER = "--norender" in argv
ONLY = opt("--only", "")
OUT = os.path.join(T, "inspect", "step03", TAG)
os.makedirs(OUT, exist_ok=True)

sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
club = bpy.data.objects["GOB_club"]
lo = bpy.data.objects["GOB_body_lo"]
hi = bpy.data.objects["GOB_body"]
BODY = lo if MESH == "lo" else hi
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
DEF = [n for n in arm.bones.keys() if arm.bones[n].use_deform]
R = {"mesh": MESH, "res": RES}

# make the target mesh visible / the other hidden
for col, ob, vis in ((bpy.data.collections["GOB_hi"], hi, MESH == "hi"),
                     (bpy.data.collections["GOB_lo"], lo, MESH == "lo")):
    col.hide_viewport = col.hide_render = not vis
    ob.hide_viewport = ob.hide_render = not vis
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name in ("GOB_hi", "GOB_lo"):
        lc.hide_viewport = (lc.name == "GOB_hi") != (MESH == "hi")
        lc.exclude = False


def upd():
    rig.update_tag()
    BODY.update_tag()
    bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["leg_roll"] = 0.0


def _apply(pb, q, add):
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ q) if add else q
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ q).to_euler(pb.rotation_mode)


def wrot(bone, axis, deg, add=False):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    _apply(PB[bone], (M.inverted() @ q.to_matrix() @ M).to_quaternion(), add)


def lrot(bone, axis, deg, add=False):
    _apply(PB[bone], Quaternion(Vector(axis).normalized(), math.radians(deg)), add)


def wloc(bone, delta, add=False):
    v = REST[bone].to_3x3().inverted() @ Vector(delta)
    PB[bone].location = (PB[bone].location + v) if add else v


def fk(bone, e):
    PB[bone].rotation_euler = [math.radians(v) for v in e]


# ---------------------------------------------------------------- mesh data
me = BODY.data
NV = len(me.vertices)
P0 = np.empty(NV * 3)
me.vertices.foreach_get("co", P0)
P0 = P0.reshape(NV, 3)
me.calc_loop_triangles()
TRI = np.array([tuple(t.vertices) for t in me.loop_triangles], dtype=np.int32)
EDG = np.empty(len(me.edges) * 2, dtype=np.int32)
me.edges.foreach_get("vertices", EDG)
EDG = EDG.reshape(-1, 2)
# weights -> dominant bone per vertex
gi = {g.index: g.name for g in BODY.vertex_groups}
Wm = np.zeros((NV, len(DEF)))
DI = {n: i for i, n in enumerate(DEF)}
for v in me.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in DI:
            Wm[v.index, DI[n]] = g.weight
DOM = np.argmax(Wm, axis=1)
PURE = Wm.max(1) > 0.9995

# rigid sets (rest-space geometric definitions)
aX0 = np.abs(P0[:, 0])
rT0 = np.hypot(P0[:, 0], P0[:, 1] - 0.105)
RIGID = {
    "head_core": (P0[:, 2] > 1.40) & (rT0 < 0.45), "HEAD": None,
    "hand_L": (P0[:, 0] > 0.96), "hand_R": (P0[:, 0] < -0.95),
    "shoe_L": (P0[:, 2] < 0.16) & (P0[:, 0] > 0), "shoe_R": (P0[:, 2] < 0.16) & (P0[:, 0] < 0),
    "belt": (P0[:, 2] > 0.60) & (P0[:, 2] < 0.77) & (aX0 < 0.40),
}
RIGID_BONE = {"head_core": "HEAD", "hand_L": "HAND_L", "hand_R": "HAND_R",
              "shoe_L": "FOOT_L", "shoe_R": "FOOT_R", "belt": "HIPS"}
RIGID = {k: v for k, v in RIGID.items() if k in RIGID_BONE}
for S in ("L", "R"):
    A = np.array(arm.bones["FOOT_" + S].head_local)
    H = np.array(arm.bones["THIGH_" + S].head_local)
    u = H - A
    u /= np.linalg.norm(u)
    v = P0 - A
    s = v @ u
    r = np.linalg.norm(v - s[:, None] * u, axis=1)
    RIGID["legtube_" + S] = ((P0[:, 0] >= 0) if S == "L" else (P0[:, 0] < 0)) & \
        (r < 0.11) & (s > 0.02) & (s < 0.185)
    RIGID_BONE["legtube_" + S] = "THIGH_" + S

# joint regions (rest space spheres) for wrinkle + silhouette probes
def sph(c, r):
    return np.linalg.norm(P0 - np.array(c), axis=1) < r


REGION = {}
for S in ("L", "R"):
    REGION["shoulder_" + S] = (np.array(arm.bones["UPPERARM_" + S].head_local), 0.20, "UPPERARM_" + S)
    REGION["elbow_" + S] = (np.array(arm.bones["FOREARM_" + S].head_local), 0.13, "FOREARM_" + S)
    REGION["wrist_" + S] = (np.array(arm.bones["HAND_" + S].head_local), 0.13, "HAND_" + S)
    REGION["hiproot_" + S] = (np.array(arm.bones["THIGH_" + S].head_local), 0.20, "HIPS")
    REGION["forearm_" + S] = (np.array(arm.bones["FOREARM_TWIST_" + S].head_local), 0.11,
                              "FOREARM_TWIST_" + S)
REGION["neck"] = (np.array((0.0, 0.105, 1.315)), 0.22, "NECK")
RMASK = {k: sph(c, r) for k, (c, r, _b) in REGION.items()}

# rest dihedral angles
EF = {}
for p in me.polygons:
    pass
me.calc_loop_triangles()


def dihedral(P):
    """p95 of |face-pair angle| per region, using edge-adjacent triangles."""
    n = np.cross(P[TRI[:, 1]] - P[TRI[:, 0]], P[TRI[:, 2]] - P[TRI[:, 0]])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1
    return n / ln[:, None]


# build triangle adjacency across shared edges once
ekey = {}
PAIR = []
for ti, t in enumerate(TRI):
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        k = (min(a, b), max(a, b))
        if k in ekey:
            PAIR.append((ekey[k], ti, k[0], k[1]))
        else:
            ekey[k] = ti
PAIR = np.array(PAIR, dtype=np.int32)


def dih_angles(P):
    N = dihedral(P)
    c = np.clip((N[PAIR[:, 0]] * N[PAIR[:, 1]]).sum(1), -1, 1)
    return np.degrees(np.arccos(c))


DIH0 = dih_angles(P0)
PAIRV = PAIR[:, 2]
# The Meshy mesh has hard feature edges and interior junction faces where the tubes
# merge into the egg (rest dihedral already > 60 deg).  Those carry no information
# about wrinkling, so the wrinkle metric only looks at pairs that are smooth at rest.
SMOOTH0 = DIH0 < 40.0

# ---------------------------------------------------------------- camera / raster
REFCAM = bpy.data.objects.get("REF_CAM")
gcd = bpy.data.cameras.get("g3_cam") or bpy.data.cameras.new("g3_cam")
gcd.type = 'ORTHO'
gcam = bpy.data.objects.get("g3_cam")
if gcam is None:
    gcam = bpy.data.objects.new("g3_cam", gcd)
    sc.collection.objects.link(gcam)


def aim(cam, cd, target, az, el, scale, dist=9.0):
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    ctr = Vector(target)
    cam.location = ctr + d * dist
    cd.ortho_scale = scale
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()


def setup_render():
    sc.render.engine = 'BLENDER_WORKBENCH'
    sh = sc.display.shading
    sh.light = 'STUDIO'
    sh.studio_light = 'Default'
    sh.color_type = 'SINGLE'
    sh.single_color = (0.62, 0.68, 0.45)
    sh.show_cavity = True
    sh.cavity_type = 'BOTH'
    sh.curvature_ridge_factor = 1.0
    sh.curvature_valley_factor = 1.0
    sh.show_object_outline = False
    try:
        sh.show_shadows = True
        sh.shadow_intensity = 0.4
    except Exception:
        pass
    sc.display.render_aa = '8'
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = 'PNG'
    sc.render.resolution_percentage = 100


if not NORENDER:
    setup_render()
SHOTS = []
if "--smooth" in argv:
    for p in me.polygons:
        p.use_smooth = True
    me.update()
NOCLUB = "--noclub" in argv
if NOCLUB:
    club.hide_viewport = club.hide_render = True

# ---- club penetration against the DEFORMED body
cme = club.data
CP = np.empty(len(cme.vertices) * 3)
cme.vertices.foreach_get("co", CP)
CP = CP.reshape(-1, 3)
cme.calc_loop_triangles()
CTRI = [tuple(t.vertices) for t in cme.loop_triangles]
CM0 = club.matrix_world.copy()
BELLY = (rT0 < 0.42) & (P0[:, 2] > 0.40) & (P0[:, 2] < 1.32)


def club_metrics(dg, Q):
    """club vs the POSED skinned torso: signed clearance + face overlaps."""
    M = np.array(club.evaluated_get(dg).matrix_world)
    CW = CP @ M[:3, :3].T + M[:3, 3]
    tris = [t for t in TRI if BELLY[t[0]] and BELLY[t[1]] and BELLY[t[2]]]
    pts = [Vector(q) for q in Q]
    bt = BVHTree.FromPolygons(pts, tris, all_triangles=True, epsilon=0.0)
    ct = BVHTree.FromPolygons([Vector(p) for p in CW], CTRI, all_triangles=True, epsilon=0.0)
    dmin, at = 1e9, None
    for i in range(0, len(CW), 2):
        loc, nor, fi, d = bt.find_nearest(Vector(CW[i]), 1.2)
        if loc is not None and d < dmin:
            dmin, at = d, [round(float(v), 3) for v in CW[i]]
    ov = ct.overlap(bt)
    # how deep does the club go inside?  count body verts inside the club hull slab
    return {"club_torso_min_mm": round(dmin * 1000, 1) if dmin < 1e8 else None,
            "club_torso_at": at, "club_torso_face_overlaps": len(ov)}


def shoot(name, view="ref", target=(0, 0.05, 0.95), scale=2.4, az=0, el=0, res=None):
    if NORENDER:
        return None
    r = res or RES
    if view == "ref" and REFCAM is not None:
        sc.camera = REFCAM
        sc.render.resolution_x = int(448 * r / 576.0)
        sc.render.resolution_y = r
    else:
        sc.camera = gcam
        sc.render.resolution_x = sc.render.resolution_y = r
        aim(gcam, gcd, target, az, el, scale)
    sc.render.filepath = os.path.join(OUT, "g_%s.png" % name)
    bpy.ops.render.render(write_still=True)
    SHOTS.append(sc.render.filepath)
    return sc.render.filepath


# ---------------------------------------------------------------- silhouette raster
SW, SH_ = 640, 824        # raster size for the silhouette metric (ref aspect)


def project(cam, Q):
    """world points -> pixel coords using the scene camera (ortho or persp)."""
    sc.render.resolution_x, sc.render.resolution_y = SW, SH_
    out = np.empty((len(Q), 2))
    for i, q in enumerate(Q):
        v = world_to_camera_view(sc, cam, Vector(q))
        out[i] = (v.x * SW, (1.0 - v.y) * SH_)
    return out


def project_fast(cam, Q):
    """vectorised ortho/persp projection"""
    Mi = np.array(cam.matrix_world.inverted())
    C = Q @ Mi[:3, :3].T + Mi[:3, 3]
    cd = cam.data
    if cd.type == 'ORTHO':
        sx = cd.ortho_scale
        ar = SW / float(SH_)
        if ar >= 1.0:
            hx, hy = sx / 2.0, sx / (2.0 * ar)
        else:
            hx, hy = sx * ar / 2.0, sx / 2.0
        u = (C[:, 0] / hx) * 0.5 + 0.5
        v = (C[:, 1] / hy) * 0.5 + 0.5
    else:
        z = -C[:, 2]
        z[z < 1e-6] = 1e-6
        f = cd.lens / cd.sensor_width
        ar = SW / float(SH_)
        if ar >= 1.0:
            hx = 1.0 / (2 * f); hy = hx / ar
        else:
            hy = 1.0 / (2 * f) * (cd.sensor_width / cd.sensor_height if cd.sensor_fit == 'VERTICAL' else 1.0)
            hx = 1.0 / (2 * f) * ar / ar
            hx = (1.0 / (2 * f)) * ar
            hy = (1.0 / (2 * f))
        u = (C[:, 0] / z) / (2 * hx) + 0.5
        v = (C[:, 1] / z) / (2 * hy) + 0.5
    return np.stack([u * SW, (1.0 - v) * SH_], 1)


def raster(pix):
    """boolean mask of the mesh silhouette, adaptive barycentric supersampling."""
    M = np.zeros((SH_, SW), bool)
    A, B, C = pix[TRI[:, 0]], pix[TRI[:, 1]], pix[TRI[:, 2]]
    emax = np.maximum(np.maximum(np.linalg.norm(B - A, axis=1),
                                 np.linalg.norm(C - B, axis=1)),
                      np.linalg.norm(A - C, axis=1))
    lev = np.clip(np.ceil(emax).astype(int), 1, 40)
    for k in np.unique(lev):
        sel = lev == k
        a, b, c = A[sel], B[sel], C[sel]
        gs = []
        for i in range(k + 1):
            for j in range(k + 1 - i):
                gs.append((i / (k + 1.0), j / (k + 1.0)))
        gs = np.array(gs)
        gs = np.c_[gs, 1.0 - gs.sum(1)]
        pts = (a[:, None, :] * gs[None, :, 0:1] + b[:, None, :] * gs[None, :, 1:2] +
               c[:, None, :] * gs[None, :, 2:3]).reshape(-1, 2)
        xi = np.clip(np.round(pts[:, 0]).astype(int), 0, SW - 1)
        yi = np.clip(np.round(pts[:, 1]).astype(int), 0, SH_ - 1)
        M[yi, xi] = True
    xi = np.clip(np.round(pix[:, 0]).astype(int), 0, SW - 1)
    yi = np.clip(np.round(pix[:, 1]).astype(int), 0, SH_ - 1)
    M[yi, xi] = True
    return M


def dist_out(mask, maxd=40):
    """8-connected distance (px) from every pixel to `mask` (0 inside)."""
    D = np.full(mask.shape, maxd + 1, np.int16)
    D[mask] = 0
    cur = mask.copy()
    for k in range(1, maxd + 1):
        g = cur.copy()
        g[1:, :] |= cur[:-1, :]; g[:-1, :] |= cur[1:, :]
        g[:, 1:] |= cur[:, :-1]; g[:, :-1] |= cur[:, 1:]
        g[1:, 1:] |= cur[:-1, :-1]; g[:-1, :-1] |= cur[1:, 1:]
        g[1:, :-1] |= cur[:-1, 1:]; g[:-1, 1:] |= cur[1:, :-1]
        new = g & ~cur
        if not new.any():
            break
        D[new] = k
        cur = g
    return D


def mm_per_px(cam):
    if cam.data.type == 'ORTHO':
        ar = SW / float(SH_)
        hx = cam.data.ortho_scale / 2.0 if ar >= 1 else cam.data.ortho_scale * ar / 2.0
        return (2 * hx / SW) * 1000.0
    # perspective: approximate at the character distance
    d = (cam.matrix_world.translation - Vector((0, 0.05, 0.95))).length
    f = cam.data.lens / cam.data.sensor_width
    return (d / f / SW) * 1000.0


# ---------------------------------------------------------------- evaluation
def posed_verts(dg):
    ev = BODY.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3)
    dm.vertices.foreach_get("co", Q)
    Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    return (np.c_[Q, np.ones(len(Q))] @ np.array(BODY.matrix_world).T)[:, :3]


def bone_mats(dg):
    ev = rig.evaluated_get(dg)
    return {n: np.array(rig.matrix_world @ ev.pose.bones[n].matrix @ REST[n].inverted())
            for n in DEF}


def ideal_verts(BM):
    """piecewise-rigid ideal: every vertex moved by its DOMINANT bone alone."""
    Q = np.empty_like(P0)
    H = np.c_[P0, np.ones(NV)]
    for j, n in enumerate(DEF):
        m = DOM == j
        if m.any():
            Q[m] = (H[m] @ BM[n].T)[:, :3]
    return Q


def volume(Q):
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    return float(np.abs((a * np.cross(b, c)).sum() / 6.0))


VOL0 = volume(P0)


def evaluate(key, dg, cams):
    Q = posed_verts(dg)
    BM = bone_mats(dg)
    I = ideal_verts(BM)
    row = {}
    row["volume_pct"] = round(100.0 * volume(Q) / VOL0, 2)
    # rigid residuals
    rr = {}
    for k, m in RIGID.items():
        if not m.any():
            continue
        M = BM[RIGID_BONE[k]]
        ref = (np.c_[P0[m], np.ones(int(m.sum()))] @ M.T)[:, :3]
        rr[k] = round(float(np.linalg.norm(Q[m] - ref, axis=1).max()) * 1000, 3)
    row["rigid_resid_mm"] = rr
    # wrinkle
    dih = dih_angles(Q)
    wk = {}
    for k, m in RMASK.items():
        sel = m[PAIRV] & SMOOTH0
        if sel.sum() > 20:
            wk[k] = [round(float(np.percentile(DIH0[sel], 95)), 2),
                     round(float(np.percentile(dih[sel], 95)), 2),
                     round(float(dih[sel].max()), 1), int(sel.sum())]
    row["dihedral_p95_rest_posed_max"] = wk
    ev = rig.evaluated_get(dg)
    row["leg_stretch"] = {S: round(float(ev.pose.bones["MCH_LEGAIM_" + S].matrix.to_scale()[1]), 4)
                          for S in ("L", "R")}
    row.update(club_metrics(dg, Q))
    # self-intersection indicators
    si = {}
    armsel = {S: (aX0 > 0.40) & ((P0[:, 0] > 0) if S == "L" else (P0[:, 0] < 0)) for S in ("L", "R")}
    torso = (aX0 < 0.385) & (P0[:, 2] > 0.40) & (P0[:, 2] < 1.30)
    headm = (P0[:, 2] > 1.36) & (rT0 < 0.45)
    for S in ("L", "R"):
        for tgt, tm in (("torso", torso), ("head", headm)):
            A = Q[armsel[S]]
            Bp = Q[tm]
            if len(A) and len(Bp):
                # coarse: count arm verts inside the target's convex slab (grid test)
                si["arm%s_min_d_%s_mm" % (S, tgt)] = round(float(np.min(
                    np.linalg.norm(A[::max(1, len(A) // 900)][:, None, :] -
                                   Bp[::max(1, len(Bp) // 900)][None, :, :], axis=2))) * 1000, 1)
    row["proximity_mm"] = si
    # silhouette
    sil = {}
    for cname, cam in cams:
        sc.camera = cam
        pq = project_fast(cam, Q)
        pi = project_fast(cam, I)
        MQ = raster(pq)
        MI = raster(pi)
        mpp = mm_per_px(cam)
        Dout = dist_out(MI)      # distance to the ideal union
        Din = dist_out(MQ)       # distance to the posed body
        prot = np.where(MQ & ~MI, Dout, 0)
        ind = np.where(MI & ~MQ, Din, 0)
        ent = {"mm_per_px": round(mpp, 2),
               "max_protrusion_px": int(prot.max()), "max_indent_px": int(ind.max()),
               "max_protrusion_mm": round(float(prot.max()) * mpp, 1),
               "max_indent_mm": round(float(ind.max()) * mpp, 1)}
        # per region (region centre carried by its own bone into the posed frame)
        yy, xx = np.mgrid[0:SH_, 0:SW]
        for k, (c, r, bn) in REGION.items():
            cw = (np.append(c, 1.0) @ BM[bn].T)[:3]
            p = project_fast(cam, cw[None, :])[0]
            rad = max(12, int(r * 1000.0 / mpp * 0.9))
            box = ((xx - p[0]) ** 2 + (yy - p[1]) ** 2) < rad * rad
            ent[k] = [int(prot[box].max()), int(ind[box].max()),
                      round(float(prot[box].max()) * mpp, 1),
                      round(float(ind[box].max()) * mpp, 1)]
        sil[cname] = ent
    row["silhouette"] = sil
    return row


# ---------------------------------------------------------------- poses
FKA = {
    "arms_down": {"UPPERARM_FK_L": (-2.59, 3.71, -69.79), "UPPERARM_FK_R": (-2.56, -3.67, 69.79)},
    "idle": {"UPPERARM_FK_R": (-15.97, -19.27, 19.28), "FOREARM_FK_R": (25.05, 0, 0),
             "HAND_FK_R": (19.14, 100.81, -12.14)},
    "windup": {"UPPERARM_FK_R": (-52.15, 100.89, 32.7), "FOREARM_FK_R": (127.7, 0, 0),
               "HAND_FK_R": (-0.49, 134.92, 1.29)},
    "impact": {"UPPERARM_FK_R": (41.27, -54.39, 117.86), "FOREARM_FK_R": (5.26, 0, 0),
               "HAND_FK_R": (-31.03, -107.0, -3.88)},
}


def apply_fk(d):
    for b, e in d.items():
        fk(b, e)


def p_armsdown():
    apply_fk(FKA["arms_down"])


def p_idle():
    apply_fk(FKA["idle"])


def p_windup():
    apply_fk(FKA["windup"])


def p_impact():
    apply_fk(FKA["impact"])


# --- set B composites
def armL(down=0.0, fwd=0.0, flex=0.0, twist=0.0):
    """down>0 lowers the L arm from T, fwd>0 swings it forward (-Y)."""
    wrot("UPPERARM_FK_L", (0, 1, 0), down)
    if fwd:
        wrot("UPPERARM_FK_L", (0, 0, 1), fwd, add=True)
    if flex:
        lrot("FOREARM_FK_L", (1, 0, 0), flex)
    if twist:
        lrot("HAND_FK_L", (0, 1, 0), twist)


def p_f1():
    apply_fk(FKA["idle"])
    armL(down=62, fwd=-8, flex=18)


def p_f11():
    wloc("COG_CTRL", (0, 0, 0.045))
    wrot("COG_CTRL", (1, 0, 0), -9)
    wrot("CHEST_CTRL", (0, 0, 1), -28)
    apply_fk(FKA["windup"])
    armL(down=-32, fwd=-18, flex=25)
    wloc("FOOT_IK_R", (-0.15, 0, 0.10))


def p_f15():
    wloc("COG_CTRL", (0.05, -0.03, -0.14))
    wrot("COG_CTRL", (1, 0, 0), 18)
    wrot("CHEST_CTRL", (1, 0, 0), 7)
    wrot("CHEST_CTRL", (0, 0, 1), 32, add=True)
    wrot("HEAD_CTRL", (1, 0, 0), 12)
    apply_fk(FKA["impact"])
    armL(down=-25, fwd=-35, flex=125)
    wloc("FOOT_IK_R", (-0.13, -0.05, 0))
    wloc("FOOT_IK_L", (0.09, 0.03, 0))


def p_f29():
    wrot("CHEST_CTRL", (0, 0, 1), -10)
    wrot("UPPERARM_FK_R", (0, 1, 0), -38)
    wrot("UPPERARM_FK_R", (0, 0, 1), -12, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 38)
    lrot("HAND_FK_R", (0, 1, 0), 95, add=True)
    armL(down=58, fwd=6, flex=14)


# --- set C single joint extremes
def _armraise(S, deg):
    return lambda: wrot("UPPERARM_FK_" + S, (0, 1, 0), -deg if S == "L" else deg)


def _armfwd(S, deg):
    return lambda: wrot("UPPERARM_FK_" + S, (0, 0, 1), -deg if S == "L" else deg)


def _armback(S, deg):
    return lambda: wrot("UPPERARM_FK_" + S, (0, 0, 1), deg if S == "L" else -deg)


SETC = {
    "C_armup60_L": _armraise("L", 60), "C_armup60_R": _armraise("R", 60),
    "C_armfwd90_L": _armfwd("L", 90), "C_armfwd90_R": _armfwd("R", 90),
    "C_armback45_R": _armback("R", 45),
    "C_elbow135_L": lambda: lrot("FOREARM_FK_L", (1, 0, 0), 135),
    "C_elbow135_R": lambda: lrot("FOREARM_FK_R", (1, 0, 0), 135),
    "C_pronP135_R": lambda: lrot("HAND_FK_R", (0, 1, 0), 135),
    "C_pronM135_R": lambda: lrot("HAND_FK_R", (0, 1, 0), -135),
    "C_pronP135_L": lambda: lrot("HAND_FK_L", (0, 1, 0), 135),
    "C_wristP35_R": lambda: lrot("HAND_FK_R", (1, 0, 0), 35),
    "C_wristM35_R": lambda: lrot("HAND_FK_R", (1, 0, 0), -35),
    "C_twistP45": lambda: wrot("CHEST_CTRL", (0, 0, 1), 45),
    "C_twistM45": lambda: wrot("CHEST_CTRL", (0, 0, 1), -45),
    "C_lean30": lambda: wrot("CHEST_CTRL", (1, 0, 0), 30),
    "C_side25": lambda: wrot("CHEST_CTRL", (0, 1, 0), 25),
    "C_headyawP40": lambda: wrot("HEAD_CTRL", (0, 0, 1), 40),
    "C_headyawM40": lambda: wrot("HEAD_CTRL", (0, 0, 1), -40),
    "C_headpitchP30": lambda: wrot("HEAD_CTRL", (1, 0, 0), 30),
    "C_headpitchM30": lambda: wrot("HEAD_CTRL", (1, 0, 0), -30),
    "C_hipsyawP30": lambda: wrot("HIPS_CTRL", (0, 0, 1), 30),
    "C_hipsyawM30": lambda: wrot("HIPS_CTRL", (0, 0, 1), -30),
    "C_cogdn20": lambda: wloc("COG_CTRL", (0, 0, -0.20)),
    "C_stepR": lambda: wloc("FOOT_IK_R", (-0.15, 0, 0.10)),
}

SETA = {"A_rest": (lambda: None), "A_arms_down": p_armsdown, "A_idle": p_idle,
        "A_windup": p_windup, "A_impact": p_impact}
SETB = {"B_f1": p_f1, "B_f11": p_f11, "B_f15": p_f15, "B_f29": p_f29}

POSES = {}
if "A" in SETS:
    POSES.update(SETA)
if "B" in SETS:
    POSES.update(SETB)
if "C" in SETS:
    POSES.update(SETC)
if ONLY:
    POSES = {k: v for k, v in POSES.items() if k in ONLY.split(",")}

CAMS = [("ref", REFCAM)] if REFCAM else []
CAMS.append(("front", gcam))

# (name, bone whose POSED head is framed, scale, az, el) - the joint is tracked into
# its posed position, otherwise the close-up frames empty space.
CLOSEUPS = [
    ("shR_front", "UPPERARM_R", 0.80, 0, 0),
    ("shR_back", "UPPERARM_R", 0.80, 180, 0),
    ("shR_low", "UPPERARM_R", 0.80, -40, -25),
    ("shR_top", "UPPERARM_R", 0.80, -20, 55),
    ("shL_front", "UPPERARM_L", 0.80, 0, 0),
    ("elbR", "FOREARM_R", 0.55, 0, 25),
    ("elbR2", "FOREARM_R", 0.55, 90, 10),
    ("elbL", "FOREARM_L", 0.55, 0, 25),
    ("wrR", "HAND_R", 0.60, -30, 15),
    ("wrL", "HAND_L", 0.55, 30, 15),
    ("neck", "NECK", 0.90, 0, 10),
    ("legs", "HIPS", 1.10, 15, -35),
]
GROUND = bpy.data.objects.get("REF_ground")

RES_ALL = {}
t0 = time.time()
DO_CU = opt("--closeups", "")
for key, fn in POSES.items():
    clear_pose()
    fn()
    dg = upd()
    # aim the front cam at the whole body for the silhouette metric
    aim(gcam, gcd, (0, 0.05, 0.95), 0, 0, 2.45)
    row = evaluate(key, dg, CAMS)
    RES_ALL[key] = row
    if not NORENDER:
        sc.render.resolution_x, sc.render.resolution_y = RES, RES
        shoot(key + "_ref", "ref")
        shoot(key + "_q34", "aux", (0, 0.05, 0.95), 2.5, 38, 8)
        if key in DO_CU.split(","):
            ev = rig.evaluated_get(dg)
            gvis = GROUND.hide_render if GROUND else None
            if GROUND:
                GROUND.hide_render = GROUND.hide_viewport = True
            for nm, bn, scl, az, el in CLOSEUPS:
                tgt = rig.matrix_world @ ev.pose.bones[bn].head
                if bn == "HIPS":
                    tgt = Vector((0, 0.05, 0.42))
                shoot(key + "_" + nm, "aux", tuple(tgt), scl, az, el, res=512)
            if GROUND:
                GROUND.hide_render = GROUND.hide_viewport = gvis
    print("### %s %s" % (key, json.dumps(row)[:300]))

clear_pose()
dg = upd()
Q = posed_verts(dg)
R["restore_max_dev_mm"] = round(float(np.linalg.norm(Q - P0, axis=1).max()) * 1000, 5)
R["poses"] = RES_ALL
R["shots"] = SHOTS
R["seconds"] = round(time.time() - t0, 1)
R["n_actions"] = len(bpy.data.actions)
print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(T, "inspect", "step03", "gate_%s.json" % TAG), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("NOT SAVED (gate is read-only)")
