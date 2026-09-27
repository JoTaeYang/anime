"""STEP 03R2 round 2 gate - HONEST silhouette + wrinkle metrics.

Run: blender -b <blend> --python step03r2b_gate.py -- --tag lo_v05c
     [--mesh GOB_body_lo] [--render] [--only T2_armR_up90]

Silhouette metric (the one that matters)
----------------------------------------
For a pose, rasterise on a 2 mm grid, in the front orthographic view AND in the
REF_CAM view:
    IDEAL  = footprint of the REST egg faces   U   footprint of the POSED tube faces
    ACTUAL = footprint of ALL posed faces
Any ACTUAL pixel outside IDEAL is mesh that has left the "egg + hose" shape.
Reported: protrusion area (cm^2) and the max distance (mm) of such a pixel from
the IDEAL footprint.  This counts the armpit web whichever group it is in, so it
cannot be gamed by relabelling verts as "arm".

Wrinkle metric
--------------
p95 / max dihedral angle (deg) over the edges of the web region, posed vs rest.
NEVER saves.
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Quaternion

sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d=None):
    return argv[argv.index(n) + 1] if n in argv else d


TAG = opt("--tag", "x")
MESH = opt("--mesh", "GOB_body_lo")
RENDER = "--render" in argv
ONLY = set((opt("--only", "") or "").split(",")) - {""}
OUT = os.path.join(ROOT, "inspect", "step03r2")
os.makedirs(OUT, exist_ok=True)
CELL = 0.002

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
SE = {"R": 0.020, "L": -0.030}

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
for c, vis in (("GOB_hi", MESH == "GOB_body"), ("GOB_lo", MESH == "GOB_body_lo")):
    cc = bpy.data.collections.get(c)
    if cc:
        cc.hide_viewport = not vis
        cc.hide_render = not vis
hi.hide_viewport = MESH != "GOB_body"; hi.hide_render = hi.hide_viewport
lo.hide_viewport = MESH != "GOB_body_lo"; lo.hide_render = lo.hide_viewport
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False
    lc.hide_viewport = not ((lc.name == "GOB_hi") == (MESH == "GOB_body"))

ob = bpy.data.objects[MESH]
me = ob.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)
gi = {g.index: g.name for g in ob.vertex_groups}
W = np.zeros((N, 18))
for v in me.vertices:
    for g in v.groups:
        nm = gi.get(g.group)
        if nm in BI:
            W[v.index, BI[nm]] = g.weight
E = np.empty(len(me.edges) * 2, dtype=np.int32); me.edges.foreach_get("vertices", E)
E = E.reshape(-1, 2)

R = {"tag": TAG, "mesh": MESH, "verts": N, "tris": len(TRI), "cell_m": CELL}


def poly_d(pts):
    pts = np.asarray(pts, dtype=np.float64)
    A, B = pts[:-1], pts[1:]
    AB = B - A
    L2 = (AB ** 2).sum(1)
    bd = np.full(N, 1e9)
    for k in range(len(A)):
        w = P0 - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0.0, 1.0)
        bd = np.minimum(bd, np.linalg.norm(w - tt[:, None] * AB[k], axis=1))
    return bd


def bh(n):
    return np.array(arm.bones[n].head_local)


def bt(n):
    return np.array(arm.bones[n].tail_local)


CH = {S: poly_d([bh("UPPERARM_" + S), bh("FOREARM_" + S),
                 bh("HAND_" + S), bt("HAND_" + S)]) for S in ("R", "L")}
SC = {}
for S in ("R", "L"):
    u = bt("UPPERARM_" + S) - bh("UPPERARM_" + S)
    u /= np.linalg.norm(u)
    SC[S] = (P0 - bh("UPPERARM_" + S)) @ u

# ------- TUBE: purely geometric, the hose the arm bones own past the exit.
# Deliberately generous (0.10 m of the chain) so the "ideal" outline is not
# understated; the club and hand balls are included.
AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
V = P0 - np.array([-0.6661, -0.3447, 0.9910])
S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
CLUB = (S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > 0.1134) & (D_AX < 0.22)
HANDR = np.linalg.norm(P0 - np.array([-0.676, -0.364, 0.992]), axis=1) <= 0.155
HANDL = np.linalg.norm(P0 - np.array([0.772, 0.056, 0.720]), axis=1) <= 0.155
TUBEV = CLUB | HANDR | HANDL
for S in ("R", "L"):
    TUBEV |= (CH[S] < 0.10) & (SC[S] > SE[S])
EGGV = ~TUBEV
TRI_TUBE = np.nonzero(TUBEV[TRI].any(axis=1))[0]     # any -> generous ideal
TRI_EGG = np.nonzero(EGGV[TRI].all(axis=1))[0]
R["tube_verts"] = int(TUBEV.sum())
R["tri_tube"] = int(len(TRI_TUBE))
R["tri_egg"] = int(len(TRI_EGG))

# ------- WEB region for the wrinkle metric: the shoulder annulus
WEB = np.zeros(N, dtype=bool)
for S in ("R", "L"):
    WEB |= ((np.linalg.norm(P0 - bh("UPPERARM_" + S), axis=1) < 0.30)
            & (CH[S] > 0.045) & (CH[S] < 0.22))
WEB &= ~CLUB & ~HANDR & ~HANDL & (P0[:, 2] < 1.30) & (P0[:, 2] > 0.80)
R["web_verts"] = int(WEB.sum())

# edges of the web, with their two adjacent triangles (for dihedral angles)
tri_of_edge = {}
for ti, t in enumerate(TRI):
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        k = (a, b) if a < b else (b, a)
        tri_of_edge.setdefault(k, []).append(ti)
WEB_E = [(k, v) for k, v in tri_of_edge.items()
         if len(v) == 2 and WEB[k[0]] and WEB[k[1]]]
WE_T = np.array([[v[0], v[1]] for _, v in WEB_E], dtype=np.int32) if WEB_E else np.zeros((0, 2), int)
R["web_edges"] = int(len(WE_T))


def dihedral(Q):
    if not len(WE_T):
        return None
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    nrm = np.cross(b - a, c - a)
    ln = np.linalg.norm(nrm, axis=1)
    nrm = nrm / np.maximum(ln, 1e-12)[:, None]
    n1, n2 = nrm[WE_T[:, 0]], nrm[WE_T[:, 1]]
    cosang = np.clip((n1 * n2).sum(1), -1.0, 1.0)
    return np.degrees(np.arccos(cosang))


# ---------------------------------------------------------------- rasteriser
_BA = []
_K = 7
for _i in range(_K + 1):
    for _j in range(_K + 1 - _i):
        _BA.append((_i / _K, _j / _K, 1.0 - _i / _K - _j / _K))
_BA = np.array(_BA)                                  # 36 barycentric samples


def footprint(pts2, tris, shape, org):
    """boolean occupancy grid of the given triangles projected to 2D"""
    g = np.zeros(shape, dtype=bool)
    if not len(tris):
        return g
    A = pts2[TRI[tris, 0]]; B = pts2[TRI[tris, 1]]; C = pts2[TRI[tris, 2]]
    for w in _BA:
        Pt = w[0] * A + w[1] * B + w[2] * C
        ij = np.floor((Pt - org) / CELL).astype(np.int64)
        ok = ((ij[:, 0] >= 0) & (ij[:, 0] < shape[0]) &
              (ij[:, 1] >= 0) & (ij[:, 1] < shape[1]))
        g[ij[ok, 0], ij[ok, 1]] = True
    return g


def dilate(g, k):
    out = g.copy()
    for _ in range(k):
        o = out.copy()
        o[1:] |= out[:-1]; o[:-1] |= out[1:]
        o[:, 1:] |= out[:, :-1]; o[:, :-1] |= out[:, 1:]
        out = o
    return out


def dist_outside(actual, ideal, cap=60):
    """max distance (cells) of an ACTUAL cell from IDEAL, plus the count"""
    out = actual & ~ideal
    if not out.any():
        return 0, 0
    grown = ideal.copy()
    for k in range(1, cap + 1):
        grown = dilate(grown, 1)
        if not (out & ~grown).any():
            return k, int(out.sum())
    return cap, int(out.sum())


# projections -------------------------------------------------------------
def proj_front(Q):
    return np.stack([Q[:, 0], Q[:, 2]], axis=1)       # ortho, looking along +Y


refcam = bpy.data.objects.get("REF_CAM")
if refcam is not None:
    CM = np.array(refcam.matrix_world.inverted())
    _ctr = np.array([0.0, 0.088, 0.95, 1.0])
    DREF = float(-(_ctr @ CM.T)[2])                    # camera -> body distance


def proj_ref(Q):
    """REF_CAM perspective projection expressed in metres in the body plane,
    so a protrusion in mm is directly comparable with the front view."""
    H = np.concatenate([Q, np.ones((len(Q), 1))], axis=1)
    C = H @ CM.T
    zc = np.maximum(-C[:, 2], 1e-6)
    return np.stack([C[:, 0] / zc, C[:, 1] / zc], axis=1) * DREF


# ---------------------------------------------------------------- pose helpers
ARM_CTRLS = ["UPPERARM_FK_L", "UPPERARM_FK_R", "FOREARM_FK_L", "FOREARM_FK_R",
             "HAND_FK_L", "HAND_FK_R"]


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0); pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def wrot(b, ax, dg, add=False):
    M = REST[b].to_3x3()
    q = Quaternion(Vector(ax).normalized(), math.radians(dg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[b]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ ql) if add else ql
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ ql).to_euler(pb.rotation_mode)


def lrot(b, ax, dg, add=False):
    q = Quaternion(Vector(ax).normalized(), math.radians(dg))
    pb = PB[b]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ q) if add else q
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ q).to_euler(pb.rotation_mode)


def upd():
    rig.update_tag(); bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def deformed():
    ev = ob.evaluated_get(upd())
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return Q.reshape(-1, 3)


# ---------------------------------------------------------------- poses
POSES = {}
POSES["T2_armR_up90"] = lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 90)
POSES["armR_up60"] = lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 60)
POSES["armR_up120"] = lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 120)


def _windup():
    wrot("UPPERARM_FK_R", (0, 1, 0), 95)
    wrot("UPPERARM_FK_R", (0, 0, 1), 25, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 100, add=True)


POSES["windup"] = _windup


def _across():
    wrot("UPPERARM_FK_R", (0, 1, 0), -25)
    wrot("UPPERARM_FK_R", (0, 0, 1), -70, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 35, add=True)


POSES["armR_across"] = _across
POSES["armL_up90"] = lambda: wrot("UPPERARM_FK_L", (0, -1, 0), 90)
POSES["armL_up130"] = lambda: wrot("UPPERARM_FK_L", (0, -1, 0), 130)


def _foldL():
    wrot("UPPERARM_FK_L", (0, -1, 0), 120)
    lrot("FOREARM_FK_L", (1, 0, 0), 110, add=True)


POSES["armL_folded"] = _foldL

# ---------------------------------------------------------------- renderer
cam, cd = RL.setup(res=1024, plain=True)
SHOTS = []
VIEWS = {"front": ((0.0, 0.0, 0.95), 0, 0, 2.1),
         "tqR": ((0.0, 0.0, 0.95), -40, 12, 2.1)}


def snap(pose):
    if not RENDER:
        return
    for v, (t, az, el, sc_) in VIEWS.items():
        SHOTS.append(RL.shoot(cam, cd, OUT, "%s_%s_%s" % (TAG, pose, v),
                              Vector(t), az, el, sc_))
    if refcam is not None:
        sc = bpy.context.scene
        old = sc.camera
        sc.camera = refcam
        sc.render.filepath = os.path.join(OUT, "%s_%s_refcam.png" % (TAG, pose))
        bpy.ops.render.render(write_still=True)
        sc.camera = old
        SHOTS.append(sc.render.filepath)


# ---------------------------------------------------------------- run
D0 = dihedral(P0)
R["rest_dihedral_web"] = {"p95": round(float(np.percentile(D0, 95)), 2),
                          "max": round(float(D0.max()), 2)} if D0 is not None else None
TESTS = {}
t_start = time.time()
for key, build in POSES.items():
    if ONLY and key not in ONLY:
        continue
    clear_pose(); build()
    Q = deformed()
    row = {}
    for vname, proj in (("front", proj_front), ("refcam", proj_ref)):
        if vname == "refcam" and refcam is None:
            continue
        pr_rest = proj(P0)
        pr_pose = proj(Q)
        allp = np.concatenate([pr_rest, pr_pose])
        org = allp.min(0) - 0.02
        shape = tuple((np.ceil((allp.max(0) + 0.02 - org) / CELL)).astype(int))
        ideal = footprint(pr_rest, TRI_EGG, shape, org) | \
            footprint(pr_pose, TRI_TUBE, shape, org)
        ideal = dilate(ideal, 1)                      # 2 mm tolerance
        actual = footprint(pr_pose, np.arange(len(TRI)), shape, org)
        cells, cnt = dist_outside(actual, ideal)
        row[vname + "_protrusion_mm"] = round(cells * CELL * 1000, 1)
        row[vname + "_protrusion_cm2"] = round(cnt * (CELL * 100) ** 2, 2)
    D = dihedral(Q)
    if D is not None:
        # per-edge CHANGE of the dihedral angle vs rest: static sharp features
        # (fangs, eye rims, the belt seam) cancel out, only new creasing shows.
        dD = np.abs(D - D0)
        row["web_crease_p95_deg"] = round(float(np.percentile(dD, 95)), 2)
        row["web_crease_max_deg"] = round(float(dD.max()), 2)
        row["web_edges_creased_gt20deg"] = int((dD > 20).sum())
    TESTS[key] = row
    print("### %s %s" % (key, json.dumps(row)))
    snap(key)

clear_pose()
R["tests"] = TESTS
R["renders"] = SHOTS
R["seconds"] = round(time.time() - t_start, 1)
with open(os.path.join(ROOT, "inspect", "step03r2b_gate_%s.json" % TAG), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", TAG, R["seconds"])
