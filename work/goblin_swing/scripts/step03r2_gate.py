"""STEP 03R2 gate - shoulder deformation test.

Run: blender -b <blend> --python step03r2_gate.py -- --tag lo_after [--mesh GOB_body_lo]
     [--render] [--only T2]
NEVER saves.  Renders go to inspect/step03r2/<tag>_<pose>_<view>.png
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

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}

# make the tested mesh visible & the other hidden
hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
for col, vis in (("GOB_hi", MESH == "GOB_body"), ("GOB_lo", MESH == "GOB_body_lo")):
    c = bpy.data.collections.get(col)
    if c:
        c.hide_viewport = not vis
        c.hide_render = not vis
hi.hide_viewport = MESH != "GOB_body"
hi.hide_render = MESH != "GOB_body"
lo.hide_viewport = MESH != "GOB_body_lo"
lo.hide_render = MESH != "GOB_body_lo"
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False
    lc.hide_viewport = not ((lc.name == "GOB_hi") == (MESH == "GOB_body"))

ob = bpy.data.objects[MESH]
me = ob.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
gi = {g.index: g.name for g in ob.vertex_groups}
W = np.zeros((N, NB))
for v in me.vertices:
    for g in v.groups:
        nm = gi.get(g.group)
        if nm in BI:
            W[v.index, BI[nm]] = g.weight
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)

R = {"tag": TAG, "mesh": MESH, "verts": N}

# ---------------------------------------------------------------- vertex sets
Z = P0[:, 2]
rT = np.hypot(P0[:, 0], P0[:, 1] - 0.088)
AX = {}
for S in ("R", "L"):
    sh = np.array(arm.bones["UPPERARM_" + S].head_local)
    tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
    u = tl - sh; u /= np.linalg.norm(u)
    v = P0 - sh
    s = v @ u
    d = np.linalg.norm(v - s[:, None] * u, axis=1)
    AX[S] = (sh, u, s, d)
# exit point of the tube axis through the torso surface (from step03r2_fix calib)
SE = {"R": 0.02, "L": -0.03}
EXIT = {S: AX[S][0] + AX[S][1] * SE[S] for S in ("R", "L")}
COLLAR = np.zeros(N, dtype=bool)
for S in ("R", "L"):
    COLLAR |= np.linalg.norm(P0 - EXIT[S], axis=1) < 0.10



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


# the egg: not any arm tube (whole chain), not the collar, not head/legs/shoes
def bhead(n):
    return np.array(arm.bones[n].head_local)


def btail(n):
    return np.array(arm.bones[n].tail_local)


ARMD = {}
for S in ("R", "L"):
    ARMD[S] = poly_d([bhead("UPPERARM_" + S), bhead("FOREARM_" + S),
                      bhead("HAND_" + S), btail("HAND_" + S)])
TUBE = (ARMD["R"] < 0.16) | (ARMD["L"] < 0.16)
TORSO = (~TUBE) & (~COLLAR) & (Z > 0.35) & (Z < 1.30) & (rT < 0.62)
R["n_torso_test_verts"] = int(TORSO.sum())
R["collar_radius_m"] = 0.10
R["exit_points"] = {S: [round(float(x), 4) for x in EXIT[S]] for S in ("R", "L")}

# rigid sets, re-derived geometrically (same constants as step03r_lowpoly)
AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
GP = np.array([-0.6661, -0.3447, 0.9910])
V = P0 - GP
S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
CLUB = (S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > 0.1134) & (D_AX < 0.22)
HANDR = (np.linalg.norm(P0 - np.array([-0.676, -0.364, 0.992]), axis=1) <= 0.155) & (~CLUB)
HANDL = np.linalg.norm(P0 - np.array([0.772, 0.056, 0.720]), axis=1) <= 0.155
HEADCORE = (Z >= 1.375) & (~CLUB)
BELT = (Z >= 0.578) & (Z <= 0.782) & (rT < 0.45) & (~CLUB) & (~HANDR) & (~HANDL)
FOOT = (Z <= 0.168) & (~CLUB) & (~HANDR) & (~HANDL)
RIGID = {"club": CLUB, "ring_hand": HANDR, "handL_ball": HANDL,
         "head_core": HEADCORE, "belt": BELT,
         "foot_R": FOOT & (P0[:, 0] < 0), "foot_L": FOOT & (P0[:, 0] >= 0)}
R["rigid_set_sizes"] = {k: int(v.sum()) for k, v in RIGID.items()}

# tube cross-section probes (ring of verts around the arm axis)
def ring(S, s0, half=0.016, rmax=0.105):
    _, _, s, d = AX[S]
    return np.nonzero((np.abs(s - s0) < half) & (d < rmax) & (Z < 1.30))[0]


PROBE = {}
for S in ("R", "L"):
    PROBE["root_" + S] = ring(S, SE[S] + 0.070)
    PROBE["mid_" + S] = ring(S, 0.17)
R["probe_sizes"] = {k: int(len(v)) for k, v in PROBE.items()}


def xsec(Q, idx):
    pts = Q[idx]
    return float(np.linalg.norm(pts - pts.mean(0), axis=1).mean())


XS0 = {k: xsec(P0, v) for k, v in PROBE.items() if len(v) >= 5}


def volume(Q):
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


V0 = volume(P0)


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, Sg, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rm = Vt.T @ D @ U.T
    return float(np.linalg.norm((Rm @ (A - ca).T).T + cb - B, axis=1).max())


# ---------------------------------------------------------------- pose helpers
ARM_CTRLS = ["UPPERARM_FK_L", "UPPERARM_FK_R", "FOREARM_FK_L", "FOREARM_FK_R",
             "HAND_FK_L", "HAND_FK_R"]


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def clear_arms():
    for n in ARM_CTRLS:
        pb = PB[n]
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)


def wrot(bone, ax, deg, add=False):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(ax).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ ql) if add else ql
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ ql).to_euler(pb.rotation_mode)


def lrot(bone, ax, deg, add=False):
    q = Quaternion(Vector(ax).normalized(), math.radians(deg))
    pb = PB[bone]
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


# ---------------------------------------------------------------- silhouette
ZB = np.arange(0.36, 1.30, 0.02)


BANDS = []      # fixed membership, taken from the REST positions
for _sgn in (-1, 1):
    for _z0 in ZB:
        _m = (TORSO & (P0[:, 2] >= _z0) & (P0[:, 2] < _z0 + 0.02) &
              ((P0[:, 0] < 0) if _sgn < 0 else (P0[:, 0] >= 0)))
        if _m.sum() >= 2:
            BANDS.append((_sgn, round(float(_z0), 3), np.nonzero(_m)[0]))


def outline(Q):
    """front view (along +Y): max |x| of each fixed rest-defined z band."""
    return np.array([float(np.abs(Q[ix, 0]).max()) for _, _, ix in BANDS])


# ---------------------------------------------------------------- renders
cam, cd = RL.setup(res=1024, plain=True)
refcam = bpy.data.objects.get("REF_CAM")
SHOTS = []


def shoot_ref(name):
    if refcam is None:
        return
    sc = bpy.context.scene
    old = sc.camera
    sc.camera = refcam
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    sc.camera = old
    SHOTS.append(sc.render.filepath)


def snap(pose, views):
    if not RENDER:
        return
    for v, (t, az, el, sc_) in views.items():
        if isinstance(t, str):
            bn, which = t.split(":")
            ev = rig.evaluated_get(upd())
            m = ev.pose.bones[bn].matrix
            t = m.to_translation() if which == "head" else m @ Vector((0, LEN[bn], 0))
        SHOTS.append(RL.shoot(cam, cd, OUT, "%s_%s_%s" % (TAG, pose, v),
                              Vector(t), az, el, sc_))
    shoot_ref("%s_%s_refcam" % (TAG, pose))


FRONT = {"front": ((0.0, 0.0, 0.95), 0, 0, 2.1)}
FULL = {"front": ((0.0, 0.0, 0.95), 0, 0, 2.1),
        "shoulderR": ("UPPERARM_R:head", 20, 20, 0.80),
        "armpitR": ("UPPERARM_R:head", 0, -30, 0.70),
        "tqR": ((0.0, 0.0, 0.95), -40, 12, 2.1)}
FULL_L = {"front": ((0.0, 0.0, 0.95), 0, 0, 2.1),
          "shoulderL": ("UPPERARM_L:head", 340, 20, 0.80),
          "tq": ((0.0, 0.0, 0.95), 35, 12, 2.1)}

# ---------------------------------------------------------------- poses
POSES = {}


def P_(k, f, views):
    POSES[k] = (f, views)


P_("T2_armR_up90", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 90), FULL)
P_("armR_up60", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 60), FULL)
P_("armR_up120", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 120), FULL)
P_("armR_fwd90", lambda: wrot("UPPERARM_FK_R", (1, 0, 0), -90), FULL)


def _across():
    wrot("UPPERARM_FK_R", (0, 1, 0), -25)
    wrot("UPPERARM_FK_R", (0, 0, 1), -70, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 35, add=True)


P_("armR_across", _across, FULL)


def _windup():
    wrot("UPPERARM_FK_R", (0, 1, 0), 95)
    wrot("UPPERARM_FK_R", (0, 0, 1), 25, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 100, add=True)


P_("windup", _windup, FULL)
P_("armL_up90", lambda: wrot("UPPERARM_FK_L", (0, -1, 0), 90), FULL_L)
P_("armL_up130", lambda: wrot("UPPERARM_FK_L", (0, -1, 0), 130), FULL_L)


def _foldL():
    wrot("UPPERARM_FK_L", (0, -1, 0), 120)
    lrot("FOREARM_FK_L", (1, 0, 0), 110, add=True)


P_("armL_folded", _foldL, FULL_L)
P_("chest_twist45", lambda: wrot("CHEST_CTRL", (0, 0, 1), 45), FRONT)
P_("head_yaw40", lambda: wrot("HEAD_CTRL", (0, 0, 1), 40), FRONT)

# ---------------------------------------------------------------- run
TESTS = {}
t0 = time.time()
for key, (build, views) in POSES.items():
    if ONLY and key not in ONLY:
        continue
    # reference = same pose with all ARM FK controls cleared (pure torso motion)
    clear_pose(); build(); clear_arms()
    Qref = deformed()
    clear_pose(); build()
    Q = deformed()

    dev = np.linalg.norm(Q - Qref, axis=1)          # motion NOT explained by torso
    row = {
        "torso_max_dev_mm": round(float(dev[TORSO].max()) * 1000, 2),
        "torso_p99_dev_mm": round(float(np.percentile(dev[TORSO], 99)) * 1000, 2),
        "torso_n_gt_3mm": int((dev[TORSO] > 0.003).sum()),
        "torso_n_gt_10mm": int((dev[TORSO] > 0.010).sum()),
        "volume_pct": round((volume(Q) / V0 - 1.0) * 100.0, 3),
    }
    if (dev[TORSO] > 0.003).any():
        w = np.nonzero(TORSO)[0][np.argmax(dev[TORSO])]
        row["worst_torso_vert"] = {"co": [round(float(x), 3) for x in P0[w]],
                                   "dev_mm": round(float(dev[w]) * 1000, 1),
                                   "w": {BONES[j]: round(float(W[w, j]), 3)
                                         for j in range(NB) if W[w, j] > 1e-4}}
    # silhouette
    dd = outline(Q) - outline(Qref)
    sgns = np.array([b[0] for b in BANDS])
    sil = {}
    for sgn in (-1, 1):
        k = sgns == sgn
        j = int(np.argmax(np.abs(dd[k])))
        sil["max_dev_mm_x%s" % ("neg" if sgn < 0 else "pos")] = round(float(dd[k][j]) * 1000, 2)
        sil["at_z_x%s" % ("neg" if sgn < 0 else "pos")] = [b[1] for b in BANDS if b[0] == sgn][j]
    row["silhouette"] = sil
    row["xsec_ratio"] = {k: round(xsec(Q, PROBE[k]) / XS0[k], 4) for k in XS0}
    row["rigid_resid_mm"] = {k: round(kabsch(P0[v], Q[v]) * 1000, 4)
                             for k, v in RIGID.items() if v.sum() >= 4}
    TESTS[key] = row
    print("### %s %s" % (key, json.dumps(row)))
    snap(key, views)

# rest render for reference
if RENDER and not ONLY:
    clear_pose(); upd()
    snap("rest", FULL)

clear_pose()
Q = deformed()
R["final_rest_max_dev_mm"] = round(float(np.linalg.norm(Q - P0, axis=1).max()) * 1000, 5)
R["tests"] = TESTS
R["renders"] = SHOTS
R["seconds"] = round(time.time() - t0, 1)
with open(os.path.join(ROOT, "inspect", "step03r2_gate_%s.json" % TAG), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", TAG, R["seconds"])
