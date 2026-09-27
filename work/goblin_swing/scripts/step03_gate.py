"""GATE 03 - deformation test for goblin_v04_skinned.blend.

Run: blender -b <file> --python step03_gate.py -- --tag dq [--only T5,TA] [--norender]
NEVER saves.  All poses are temporary; rest is restored and re-verified at the end.
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Matrix, Quaternion
from mathutils.bvhtree import BVHTree

sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(name, default=None):
    return argv[argv.index(name) + 1] if name in argv else default


TAG = opt("--tag", "dq")
ONLY = set(opt("--only", "").split(",")) - {""}
NORENDER = "--norender" in argv
OUT = os.path.join(ROOT, "inspect", "step03", "gate_" + TAG)
os.makedirs(OUT, exist_ok=True)
REPORT = os.path.join(ROOT, "inspect", "step03_gate_%s.json" % TAG)

sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
me = body.data
PB = rig.pose.bones
N = len(me.vertices)

P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
R = {"tag": TAG, "preserve_volume": body.modifiers[0].use_deform_preserve_volume}

# triangles (for volume + BVH)
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)

# ---------------------------------------------------------------- vertex sets
lab = np.load(os.path.join(ROOT, "inspect", "step03_labels.npy"))
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
Z = P0[:, 2]
SET = {n: (lab == BI[n]) for n in BONES}
SET["ARM_R"] = SET["UPPERARM_R"] | SET["FOREARM_R"] | SET["HAND_R"]
SET["ARM_L"] = SET["UPPERARM_L"] | SET["FOREARM_L"] | SET["HAND_L"]
SET["LEG_R"] = SET["THIGH_R"] | SET["SHIN_R"] | SET["FOOT_R"]
SET["LEG_L"] = SET["THIGH_L"] | SET["SHIN_L"] | SET["FOOT_L"]
SET["TORSO"] = SET["HIPS"] | SET["SPINE_01"] | SET["CHEST"]
SET["HEADCORE"] = SET["HEAD"] & (Z > 1.375)
SET["EYES"] = SET["HEAD"] & (Z > 1.45)
SET["LOWER_TORSO"] = SET["HIPS"] & (Z < 0.80)

# faces grouped by dominant label, for intersection tests
def face_group(mask):
    return np.nonzero(mask[TRI].all(axis=1))[0]


FG = {k: face_group(SET[k]) for k in ("WEAPON", "HEAD", "TORSO", "ARM_R", "ARM_L",
                                      "LEG_R", "LEG_L")}
R["face_groups"] = {k: int(len(v)) for k, v in FG.items()}

RIGID = {"club": SET["WEAPON"], "head_core": SET["HEADCORE"],
         "foot_L": SET["FOOT_L"], "foot_R": SET["FOOT_R"],
         "hand_L_ball": SET["HAND_L"], "ring_hand": SET["HAND_R"]}

# ---------------------------------------------------------------- joint probes
def probe(center, axis, r_in, r_out, half):
    c = np.array(center, dtype=np.float64)
    a = np.array(axis, dtype=np.float64)
    a /= np.linalg.norm(a)
    v = P0 - c
    s = v @ a
    d = np.linalg.norm(v - s[:, None] * a, axis=1)
    return np.nonzero((np.abs(s) < half) & (d > r_in) & (d < r_out))[0]


def bseg(n):
    return np.array(REST[n].to_translation()), \
           np.array(REST[n] @ Vector((0, LEN[n], 0)))


JOINTS = {}
for n, (r_in, r_out, half) in [("FOREARM_L", (0.02, 0.11, 0.012)),
                               ("FOREARM_R", (0.02, 0.11, 0.012)),
                               ("HAND_L", (0.02, 0.11, 0.012)),
                               ("HAND_R", (0.02, 0.11, 0.012)),
                               ("SHIN_L", (0.02, 0.12, 0.012)),
                               ("SHIN_R", (0.02, 0.12, 0.012)),
                               ("UPPERARM_L", (0.02, 0.12, 0.012)),
                               ("UPPERARM_R", (0.02, 0.12, 0.012)),
                               ("FOOT_L", (0.02, 0.13, 0.012)),
                               ("FOOT_R", (0.02, 0.13, 0.012))]:
    h, t = bseg(n)
    ax = t - h
    idx = probe(h, ax, r_in, r_out, half)
    if len(idx) >= 8:
        JOINTS[n] = idx
h, t = bseg("NECK")
idx = probe(np.array([0.0, 0.07, 1.30]), np.array([0.0, -0.35, 0.05]), 0.02, 0.40, 0.012)
if len(idx) >= 8:
    JOINTS["NECK"] = idx
R["joint_probe_sizes"] = {k: int(len(v)) for k, v in JOINTS.items()}


def xsec_radius(Q, idx):
    pts = Q[idx]
    c = pts.mean(0)
    return float(np.linalg.norm(pts - c, axis=1).mean())


REST_XSEC = {k: xsec_radius(P0, v) for k, v in JOINTS.items()}
R["rest_xsec"] = {k: round(v, 5) for k, v in REST_XSEC.items()}


# ---------------------------------------------------------------- metrics
def volume(Q):
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


V0 = volume(P0)
R["rest_volume_m3"] = round(V0, 6)


def kabsch_resid(A, B):
    """max residual after the best-fit rigid transform A -> B."""
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    Rm = Vt.T @ D @ U.T
    return float(np.linalg.norm((Rm @ (A - ca).T).T + cb - B, axis=1).max())


def bvh_of(Q, faces):
    return BVHTree.FromPolygons([Vector(p) for p in Q],
                                [tuple(int(i) for i in TRI[f]) for f in faces],
                                all_triangles=True, epsilon=0.0)


def penetration(Q, a, b):
    """(#overlapping face pairs, max depth of a-verts inside b) """
    if len(FG[a]) == 0 or len(FG[b]) == 0:
        return 0, 0.0
    ta, tb = bvh_of(Q, FG[a]), bvh_of(Q, FG[b])
    ov = ta.overlap(tb)
    if not ov:
        return 0, 0.0
    vs = set()
    for ia, ib in ov:
        vs.update(int(x) for x in TRI[FG[a][ia]])
    depth = 0.0
    for i in list(vs)[:400]:
        loc, nor, fi, dist = tb.find_nearest(Vector(Q[i]))
        if loc is not None and (Vector(Q[i]) - loc).dot(nor) < 0:
            depth = max(depth, dist)
    return len(ov), round(depth, 5)


# ---------------------------------------------------------------- pose helpers
def upd():
    rig.update_tag()
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
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def wloc(bone, delta, add=False):
    M = REST[bone].to_3x3()
    v = M.inverted() @ Vector(delta)
    PB[bone].location = (PB[bone].location + v) if add else v


def wrot(bone, axis, deg, add=False):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ ql) if add else ql
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ ql).to_euler(pb.rotation_mode)


def lrot(bone, axis, deg, add=False):
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ q) if add else q
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ q).to_euler(pb.rotation_mode)


def deformed():
    dg = upd()
    ev = body.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3)
    dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return Q.reshape(-1, 3)


# ---------------------------------------------------------------- renderer
cam, cd = RL.setup(res=1024, plain=True)
SHOTS = []
VIEWS = {
    "front": ((0.0, 0.0, 0.95), 0, 0, 2.1),
    "back": ((0.0, 0.0, 0.95), 180, 0, 2.1),
    "side": ((0.0, 0.0, 0.95), 90, 0, 2.1),
    "sideL": ((0.0, 0.0, 0.95), 270, 0, 2.1),
    "tq": ((0.0, 0.0, 0.95), 35, 12, 2.1),
    "tqR": ((0.0, 0.0, 0.95), -40, 12, 2.1),
    "top": ((0.0, 0.0, 0.95), 0, 75, 2.1),
    # close-ups track the POSED joint, otherwise the crop misses it entirely
    "shoulderR": ("UPPERARM_R:head", 20, 20, 0.80),
    "shoulderL": ("UPPERARM_L:head", 340, 20, 0.80),
    "armpitR": ("UPPERARM_R:head", 0, -30, 0.70),
    "elbowR": ("FOREARM_R:head", 40, 10, 0.50),
    "elbowR2": ("FOREARM_R:head", 130, 10, 0.50),
    "elbowL": ("FOREARM_L:head", 320, 10, 0.50),
    "wristR": ("HAND_R:head", 55, 5, 0.70),
    "gripR": ("WEAPON:head", 125, 5, 0.70),
    "hipR": ("THIGH_R:head", 15, -12, 0.55),
    "kneeR": ("SHIN_R:head", 15, 0, 0.42),
    "kneeL": ("SHIN_L:head", 345, 0, 0.42),
    "neck": ("NECK:head", 20, 8, 1.00),
    "neckside": ("NECK:head", 90, 8, 1.00),
    "ankleR": ("FOOT_R:head", 20, 5, 0.50),
    "legs": ((0.0, 0.05, 0.28), 0, 8, 1.05),
    "legsside": ((0.0, 0.05, 0.28), 90, 8, 1.05),
    "clubtop": ("WEAPON:tail", 70, 10, 1.0),
}


def resolve(t):
    if isinstance(t, str):
        bn, which = t.split(":")
        ev = rig.evaluated_get(upd())
        m = ev.pose.bones[bn].matrix
        return m.to_translation() if which == "head" else m @ Vector((0, LEN[bn], 0))
    return Vector(t)


def snap(tag, views):
    if NORENDER:
        return
    for v in views:
        t, az, el, sc_ = VIEWS[v]
        SHOTS.append(RL.shoot(cam, cd, OUT, "%s_%s" % (tag, v),
                              resolve(t), az, el, sc_))


# ---------------------------------------------------------------- test driver
def evaluate(tag, static_keys, pen_pairs, joint_keys):
    Q = deformed()
    d = np.linalg.norm(Q - P0, axis=1)
    row = {"volume_pct": round((volume(Q) / V0 - 1.0) * 100.0, 4),
           "max_disp_m": round(float(d.max()), 4)}
    st = np.zeros(N, dtype=bool)
    for k in static_keys:
        st |= SET[k]
    if st.any():
        row["static_max_disp_m"] = round(float(d[st].max()), 6)
        row["static_set"] = "+".join(static_keys)
        row["static_n"] = int(st.sum())
    rr = {}
    for k, m in RIGID.items():
        rr[k] = round(kabsch_resid(P0[m], Q[m]), 8)
    row["rigid_resid_m"] = rr
    row["pinch"] = {k: round(xsec_radius(Q, JOINTS[k]) / REST_XSEC[k], 4)
                    for k in joint_keys if k in JOINTS}
    pen = {}
    for a, b in pen_pairs:
        n_ov, dep = penetration(Q, a, b)
        pen["%s_in_%s" % (a, b)] = [n_ov, dep]
    row["penetration"] = pen
    return row, Q


TESTS = {}
t_start = time.time()


def run(key, name, build, static_keys, pen_pairs, joint_keys, views):
    if ONLY and key not in ONLY:
        return
    clear_pose()
    note = build()
    row, Q = evaluate(key, static_keys, pen_pairs, joint_keys)
    if note:
        row["note"] = note
    TESTS.setdefault(key, {})[name] = row
    snap("%s_%s" % (key, name), views)
    print("### %s %s %s" % (key, name, json.dumps(row)))


PEN_ARM = [("WEAPON", "HEAD"), ("WEAPON", "TORSO"), ("ARM_R", "HEAD"),
           ("ARM_R", "TORSO"), ("ARM_L", "TORSO"), ("ARM_L", "HEAD")]

# ---- T1 arm forward 90
run("T1", "armR_fwd90", lambda: wrot("UPPERARM_FK_R", (1, 0, 0), -90),
    ["HEAD", "ARM_L", "LEG_L", "LEG_R", "LOWER_TORSO"], PEN_ARM,
    ["UPPERARM_R", "FOREARM_R"], ["front", "side", "shoulderR", "armpitR"])
run("T1", "armL_fwd90", lambda: wrot("UPPERARM_FK_L", (1, 0, 0), -90),
    ["HEAD", "ARM_R", "WEAPON", "LEG_L", "LEG_R", "LOWER_TORSO"], PEN_ARM,
    ["UPPERARM_L", "FOREARM_L"], ["front", "sideL", "shoulderL"])

# ---- T2 arm up/out 90
run("T2", "armR_up90", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 90),
    ["HEAD", "ARM_L", "LEG_L", "LEG_R", "LOWER_TORSO"], PEN_ARM,
    ["UPPERARM_R", "FOREARM_R"], ["front", "tqR", "shoulderR", "armpitR"])
run("T2", "armL_up90", lambda: wrot("UPPERARM_FK_L", (0, -1, 0), 90),
    ["HEAD", "ARM_R", "WEAPON", "LEG_L", "LEG_R", "LOWER_TORSO"], PEN_ARM,
    ["UPPERARM_L", "FOREARM_L"], ["front", "shoulderL"])

# ---- T3 elbows
run("T3", "elbowR_flex90", lambda: lrot("FOREARM_FK_R", (1, 0, 0), 90),
    ["HEAD", "ARM_L", "LEG_L", "LEG_R", "TORSO"], PEN_ARM,
    ["FOREARM_R", "HAND_R"], ["front", "side", "elbowR", "wristR"])
run("T3", "elbowL_flex90", lambda: lrot("FOREARM_FK_L", (1, 0, 0), 90),
    ["HEAD", "ARM_R", "WEAPON", "LEG_L", "LEG_R", "TORSO"], PEN_ARM,
    ["FOREARM_L", "HAND_L"], ["front", "sideL", "elbowL"])


def straighten_R():
    """rotate FOREARM_FK_R until the shoulder-wrist distance is maximal."""
    best = (None, -1.0)
    for deg in range(-90, 91, 5):
        clear_pose()
        lrot("FOREARM_FK_R", (1, 0, 0), deg)
        dg = upd()
        ev = rig.evaluated_get(dg)
        d = (ev.pose.bones["HAND_R"].matrix.to_translation() -
             ev.pose.bones["UPPERARM_R"].matrix.to_translation()).length
        if d > best[1]:
            best = (deg, d)
    clear_pose()
    lrot("FOREARM_FK_R", (1, 0, 0), best[0])
    return "straightened by %d deg (shoulder-wrist %.4f m)" % (best[0], best[1])


def elbow_clear(S):
    def f():
        wrot("UPPERARM_FK_" + S, (0, 1, 0) if S == "R" else (0, -1, 0), 75)
        lrot("FOREARM_FK_" + S, (1, 0, 0), 90, add=True)
        return "arm out 75 deg then elbow flexed 90 deg (joint in clear air)"
    return f


run("T3", "elbowR_clear90", elbow_clear("R"),
    ["HEAD", "ARM_L", "LEG_L", "LEG_R", "TORSO"], PEN_ARM,
    ["FOREARM_R", "HAND_R", "UPPERARM_R"],
    ["front", "tqR", "elbowR", "elbowR2", "shoulderR"])
run("T3", "elbowL_clear90", elbow_clear("L"),
    ["HEAD", "ARM_R", "WEAPON", "LEG_L", "LEG_R", "TORSO"], PEN_ARM,
    ["FOREARM_L", "HAND_L", "UPPERARM_L"], ["front", "elbowL", "shoulderL"])

run("T3", "elbowR_straight", straighten_R,
    ["HEAD", "ARM_L", "LEG_L", "LEG_R", "TORSO"], PEN_ARM,
    ["FOREARM_R", "HAND_R"], ["front", "side", "elbowR"])

PEN_LEG = [("LEG_L", "TORSO"), ("LEG_R", "TORSO"), ("LEG_L", "LEG_R")]

# ---- T4 legs
run("T4", "legL_fwd", lambda: wloc("FOOT_IK_L", (0, -0.12, 0.05)),
    ["HEAD", "WEAPON", "ARM_R", "ARM_L", "FOOT_R"], PEN_LEG,
    ["SHIN_L", "FOOT_L", "SHIN_R"], ["front", "legsside", "kneeL", "hipR"])
run("T4", "footR_sidestep", lambda: wloc("FOOT_IK_R", (-0.15, 0, 0.10)),
    ["HEAD", "WEAPON", "ARM_R", "ARM_L", "FOOT_L"], PEN_LEG,
    ["SHIN_R", "FOOT_R", "HAND_R"], ["front", "legs", "kneeR", "ankleR"])

# ---- T4b how far can this character's 0.19 m leg tubes actually bend?
if not ONLY or "T4b" in ONLY:
    sweep = []
    L0e = np.linalg.norm(P0[TRI[:, 0]] - P0[TRI[:, 1]], axis=1)
    legmask = SET["LEG_L"]
    tril = np.nonzero(legmask[TRI].all(axis=1))[0]

    def leg_area(Q):
        a, b, c = Q[TRI[tril, 0]], Q[TRI[tril, 1]], Q[TRI[tril, 2]]
        return float(np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() / 2)

    A0l = leg_area(P0)
    for dz in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15):
        clear_pose()
        wloc("COG_CTRL", (0, 0, -dz))
        ev = rig.evaluated_get(upd())
        hip = ev.pose.bones["THIGH_L"].matrix.to_translation()
        kn = ev.pose.bones["SHIN_L"].matrix.to_translation()
        an = ev.pose.bones["FOOT_L"].matrix.to_translation()
        flex = 180.0 - math.degrees((kn - hip).angle(an - kn))
        Q = deformed()
        n_ov, dep = penetration(Q, "LEG_L", "TORSO")
        sweep.append({"cog_down_m": dz,
                      "knee_flex_deg": round(180 - flex, 2),
                      "leg_area_pct": round((leg_area(Q) / A0l - 1) * 100, 2),
                      "leg_into_torso_faces": n_ov,
                      "leg_into_torso_depth_m": dep})
    TESTS["T4b"] = {"knee_range_sweep": sweep}
    print("### T4b", json.dumps(sweep))
    clear_pose()
    wloc("COG_CTRL", (0, 0, -0.05))
    upd()
    snap("T4b_cog_down05", ["front", "legsside", "kneeR"])

# ---- T5 crouch
for dz in (0.10, 0.15):
    run("T5", "cog_down%02d" % int(dz * 100),
        (lambda d=dz: wloc("COG_CTRL", (0, 0, -d))),
        ["FOOT_L", "FOOT_R"], PEN_LEG,
        ["SHIN_L", "SHIN_R", "FOOT_L", "FOOT_R"],
        ["front", "legsside", "kneeR", "hipR"])

# ---- T6 chest
run("T6", "chest_twist+45", lambda: wrot("CHEST_CTRL", (0, 0, 1), 45),
    ["LEG_L", "LEG_R"], PEN_ARM, ["NECK", "UPPERARM_R", "UPPERARM_L"],
    ["front", "top", "neck", "shoulderR"])
run("T6", "chest_twist-45", lambda: wrot("CHEST_CTRL", (0, 0, 1), -45),
    ["LEG_L", "LEG_R"], PEN_ARM, ["NECK", "UPPERARM_R", "UPPERARM_L"],
    ["front", "top"])
run("T6", "chest_fwd30", lambda: wrot("CHEST_CTRL", (1, 0, 0), 30),
    ["LEG_L", "LEG_R"], PEN_ARM, ["NECK"], ["side", "tq", "neck"])
run("T6", "chest_side25", lambda: wrot("CHEST_CTRL", (0, 1, 0), 25),
    ["LEG_L", "LEG_R"], PEN_ARM, ["NECK"], ["front", "neck"])

# ---- T7 hips yaw, feet planted
def hips_yaw(deg, stretch):
    def f():
        wrot("HIPS_CTRL", (0, 0, 1), deg)
        for S in ("L", "R"):
            PB["FOOT_IK_" + S]["ik_stretch"] = stretch
        return "ik_stretch=%.1f" % stretch
    return f


run("T7", "hips_yaw+30", hips_yaw(30, 0.0), ["FOOT_L", "FOOT_R"],
    [("WEAPON", "TORSO")], ["SHIN_L", "SHIN_R", "FOOT_L", "FOOT_R"],
    ["front", "legs", "hipR"])
run("T7", "hips_yaw-30", hips_yaw(-30, 0.0), ["FOOT_L", "FOOT_R"],
    [("WEAPON", "TORSO")], ["SHIN_L", "SHIN_R"], ["front", "legs"])
run("T7", "hips_yaw+30_stretch", hips_yaw(30, 1.0), ["FOOT_L", "FOOT_R"],
    [("WEAPON", "TORSO")], ["SHIN_L", "SHIN_R"], ["front", "legs"])

# ---- T8 head
for lbl, ax, dg_ in [("yaw+40", (0, 0, 1), 40), ("yaw-40", (0, 0, 1), -40),
                     ("pitch+30", (1, 0, 0), 30), ("pitch-30", (1, 0, 0), -30)]:
    run("T8", "head_" + lbl,
        (lambda a=ax, d=dg_: wrot("HEAD_CTRL", a, d)),
        ["LEG_L", "LEG_R", "LOWER_TORSO", "ARM_R", "ARM_L", "WEAPON"],
        [("ARM_R", "HEAD"), ("WEAPON", "HEAD")], ["NECK"],
        ["front", "neck"] + (["side"] if "pitch" in lbl else ["top"]))

# ---- T9 wrist
for dg_ in (30, -30):
    run("T9", "wristR_%+d" % dg_,
        (lambda d=dg_: lrot("HAND_FK_R", (1, 0, 0), d)),
        ["HEAD", "ARM_L", "LEG_L", "LEG_R", "TORSO"],
        [("WEAPON", "HEAD"), ("WEAPON", "TORSO")], ["HAND_R"],
        ["front", "wristR", "gripR"])

# ---- T10 weapon control
for dg_ in (10, -10):
    run("T10", "weapon_%+d" % dg_,
        (lambda d=dg_: wrot("WEAPON_CTRL", (0, 1, 0), d)),
        ["HEAD", "ARM_R", "ARM_L", "LEG_L", "LEG_R", "TORSO"],
        [("WEAPON", "HEAD"), ("WEAPON", "TORSO")], ["HAND_R"],
        ["front", "gripR", "wristR"])

# ---- TA composite wind-up
def poseA():
    wloc("COG_CTRL", (0, 0, 0.03))
    wrot("CHEST_CTRL", (0, 0, 1), -30)          # twist toward character's right
    wrot("CHEST_CTRL", (1, 0, 0), -10, add=True)  # lean back
    wrot("UPPERARM_FK_R", (0, 1, 0), 100)
    lrot("FOREARM_FK_R", (1, 0, 0), 55)
    wrot("UPPERARM_FK_L", (0, -1, 0), 120)
    wloc("FOOT_IK_R", (-0.12, 0, 0.08))
    return "windup"


run("TA", "windup", poseA, ["FOOT_L"], PEN_ARM,
    ["UPPERARM_R", "FOREARM_R", "HAND_R", "NECK", "SHIN_R"],
    ["front", "tqR", "side", "shoulderR", "wristR", "neck", "legs"])


# ---- TB composite impact
def poseB():
    wloc("COG_CTRL", (-0.05, 0, -0.12))
    wrot("CHEST_CTRL", (0, 0, 1), 35)
    wrot("CHEST_CTRL", (1, 0, 0), 25, add=True)
    wrot("UPPERARM_FK_R", (0, 1, 0), -25)
    wrot("UPPERARM_FK_R", (0, 0, 1), -55, add=True)
    lrot("FOREARM_FK_R", (1, 0, 0), 10, add=True)
    wrot("UPPERARM_FK_L", (0, -1, 0), 35)
    wrot("HEAD_CTRL", (1, 0, 0), 25)
    wloc("FOOT_IK_R", (-0.16, -0.05, 0.0))
    wloc("FOOT_IK_L", (0.10, -0.02, 0.0))
    return "impact"


run("TB", "impact", poseB, [], PEN_ARM,
    ["UPPERARM_R", "FOREARM_R", "HAND_R", "NECK", "SHIN_R", "SHIN_L"],
    ["front", "tqR", "side", "shoulderR", "wristR", "neck", "legs", "kneeR"])

# ---------------------------------------------------------------- rest restore
clear_pose()
Q = deformed()
dev = np.linalg.norm(Q - P0, axis=1)
R["final_rest_max_dev_m"] = float(dev.max())
# with the leg IK switched off, how exact is the rest pose?
for S in ("L", "R"):
    for c in PB["SHIN_" + S].constraints:
        if c.type == 'IK':
            c.influence = 0.0
Q2 = deformed()
R["rest_max_dev_legIK_off_m"] = float(np.linalg.norm(Q2 - P0, axis=1).max())
for S in ("L", "R"):
    for c in PB["SHIN_" + S].constraints:
        if c.type == 'IK':
            c.influence = 1.0
clear_pose()

resid = []
for pb in PB:
    if (pb.location.length > 1e-9 or (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9
            or Vector(pb.rotation_euler).length > 1e-9
            or abs(pb.rotation_quaternion.w - 1) > 1e-9
            or Vector(pb.rotation_quaternion[1:]).length > 1e-9):
        resid.append(pb.name)
R["residual_posed_bones"] = resid
R["has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["tests"] = TESTS
R["renders"] = SHOTS
R["seconds"] = round(time.time() - t_start, 1)

with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", REPORT, R["seconds"], "s")
