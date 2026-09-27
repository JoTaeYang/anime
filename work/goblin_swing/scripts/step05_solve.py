"""STEP 05 - solve the blocking key poses against the reference (screen-space).

  blender -b goblin_v05_legfix.blend --python scripts/step05_solve.py -- [--only 15,18]

Writes scripts/step05_poses.json  (control values per key frame, the data that
scripts/step05_blocking.py turns into the GOB_swing action) and
inspect/step05_solve.json (per-frame residual report).

NEVER saves the .blend.

Parameterisation
----------------
  body DOFs use WORLD-aligned axes on the control's rest orientation
  (exactly the `wrot` convention of step03r_gate.py):
     +X rot = pitch forward / look down      +Y rot = top toward screen-RIGHT
     +Z rot = yaw toward the character's LEFT
  arm DOFs use BONE-LOCAL rotation vectors (gimbal free), so the elbow stays a
  real elbow: the forearm gets only local-X (flexion) and local-Y (twist).
  Rotations are carried in DEGREES, locations in CENTIMETRES, so the optimiser's
  regularisation weights are on one scale.
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion, Matrix
from bpy_extras.object_utils import world_to_camera_view

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import step05_targets as T

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ONLY = set(int(x) for x in (argv[argv.index("--only") + 1] if "--only" in argv else "").split(",") if x)

REF_W, REF_H = 448, 576
sc = bpy.context.scene
sc.render.resolution_x, sc.render.resolution_y = REF_W, REF_H
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
cam = bpy.data.objects["REF_CAM"]
sc.camera = cam
mesh = bpy.data.objects["GOB_body_lo"]
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}

MET = json.load(open(os.path.join(ROOT, "ref", "metrics.json")))["frames"]

# ------------------------------------------------------------------ landmarks
mwm = np.array(mesh.matrix_world)
co = np.empty(len(mesh.data.vertices) * 3)
mesh.data.vertices.foreach_get("co", co)
V = (co.reshape(-1, 3) @ mwm[:3, :3].T) + mwm[:3, 3]
hb = arm.bones["HEAD"]
z0, hl, cx = hb.head_local.z, hb.length, hb.head_local.x
_h = V[(V[:, 2] > z0 - 0.05 * hl) & (np.abs(V[:, 0] - cx) < 0.9 * hl)]
_f = _h[_h[:, 1] < _h[:, 1].min() + 0.015]
rest_pts = {"eye_L": ("HEAD", _f[_f[:, 0] > cx + 0.05 * hl].mean(0)),
            "eye_R": ("HEAD", _f[_f[:, 0] < cx - 0.05 * hl].mean(0)),
            "head_top": ("HEAD", V[V[:, 2] > V[:, 2].max() - 0.004].mean(0))}
zmin = V[:, 2].min()
gnd = V[V[:, 2] < zmin + 0.005]
for tag, bn, sel in (("foot_L", "FOOT_L", gnd[:, 0] > 0), ("foot_R", "FOOT_R", gnd[:, 0] < 0)):
    P = gnd[sel]
    rest_pts[tag] = (bn, np.array([(P[:, 0].min() + P[:, 0].max()) / 2,
                                   (P[:, 1].min() + P[:, 1].max()) / 2, zmin]))
awi = rig.matrix_world.inverted()
LOCALPT = {k: (bn, REST[bn].inverted() @ (awi @ Vector(p))) for k, (bn, p) in rest_pts.items()}

CLUB_BLOBFIT = 0.21      # m back from the WEAPON tail -> metrics club.head_center
CLUB_BUTT = 0.25         # m back from the WEAPON head -> centre of the butt blob

# ------------------------------------------------------------------ fast silhouette
# The landmark set alone does not constrain the SHAPE between the landmarks: the
# solver happily crushed the torso / twisted the belt while hitting every point.
# So project the whole deformed low-poly and constrain two measured boxes:
#   metrics silhouette_bbox   = every character pixel (body + club)
#   metrics hips_belt_bbox    = the brown belt blob
NV = len(mesh.data.vertices)
gi = {g.index: g.name for g in mesh.vertex_groups}
Wt = np.zeros((NV, 2))          # [HIPS+SPINE_01 share, WEAPON share]
for v in mesh.data.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in ("HIPS", "SPINE_01"):
            Wt[v.index, 0] += g.weight
        elif n == "WEAPON":
            Wt[v.index, 1] += g.weight
_z = V[:, 2]
BELT = np.nonzero((Wt[:, 0] > 0.5) & (_z > 0.53) & (_z < 0.80))[0]
print("step05_solve: belt verts %d / %d" % (len(BELT), NV))

_pm = np.array(cam.calc_matrix_camera(bpy.context.evaluated_depsgraph_get(),
                                      x=REF_W, y=REF_H))
_vm = np.array(cam.matrix_world.inverted())
_PV = _pm @ _vm


def project(P):
    """P = (n,3) world -> (n,2) px, same convention as world_to_camera_view."""
    H = np.c_[P, np.ones(len(P))] @ _PV.T
    w = H[:, 3]
    w[np.abs(w) < 1e-9] = 1e-9
    return np.c_[(H[:, 0] / w * 0.5 + 0.5) * REF_W,
                 (1.0 - (H[:, 1] / w * 0.5 + 0.5)) * REF_H]


def deformed():
    dg = bpy.context.evaluated_depsgraph_get()
    ev = mesh.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3)
    dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return (Q.reshape(-1, 3) @ mwm[:3, :3].T) + mwm[:3, 3]


def boxes():
    Q = project(deformed())
    sil = np.array([Q[:, 0].min(), Q[:, 1].min(), Q[:, 0].max(), Q[:, 1].max()])
    B = Q[BELT]
    belt = np.array([B[:, 0].min(), B[:, 1].min(), B[:, 0].max(), B[:, 1].max()])
    return sil, belt


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


def set_wloc(bone, delta):
    PB[bone].location = REST[bone].to_3x3().inverted() @ Vector(delta)


def set_wrot(bone, rv_deg):
    """rv_deg = rotation vector in WORLD axes, degrees."""
    v = Vector(rv_deg)
    ang = math.radians(v.length)
    q = Quaternion(v.normalized(), ang) if v.length > 1e-9 else Quaternion()
    M = REST[bone].to_3x3()
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


def set_lrot(bone, rv_deg):
    """rv_deg = rotation vector in BONE-LOCAL axes, degrees."""
    v = Vector(rv_deg)
    ang = math.radians(v.length)
    q = Quaternion(v.normalized(), ang) if v.length > 1e-9 else Quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = q
    else:
        pb.rotation_euler = q.to_euler(pb.rotation_mode)


# ------------------------------------------------------------------ DOF layout
DOF = [
    ("cog_loc", 3, "wloc", "COG_CTRL"),
    ("cog_rot", 3, "wrot", "COG_CTRL"),
    ("hips_rot", 3, "wrot", "HIPS_CTRL"),
    ("chest_rot", 3, "wrot", "CHEST_CTRL"),
    ("head_rot", 3, "wrot", "HEAD_CTRL"),
    ("footR_loc", 2, "footR", "FOOT_IK_R"),
    ("uaR", 3, "lrot", "UPPERARM_FK_R"),
    ("faR", 2, "lrotXY", "FOREARM_FK_R"),
    ("haR", 3, "lrot", "HAND_FK_R"),
    ("uaL", 3, "lrot", "UPPERARM_FK_L"),
    ("faL", 2, "lrotXY", "FOREARM_FK_L"),
]
IDX, _o = {}, 0
for nm, n, _k, _b in DOF:
    IDX[nm] = (_o, _o + n); _o += n
NDOF = _o
BODY_DOFS = ["cog_loc", "cog_rot", "hips_rot", "chest_rot", "head_rot", "footR_loc"]
ARMR_DOFS = ["uaR", "faR", "haR"]
ARML_DOFS = ["uaL", "faL"]


def slice_of(names):
    out = []
    for n in names:
        a, b = IDX[n]; out += list(range(a, b))
    return np.array(out, dtype=int)


def apply(x, footR_z):
    clear_pose()
    a, b = IDX["cog_loc"]; set_wloc("COG_CTRL", x[a:b] / 100.0)
    for nm, bone in (("cog_rot", "COG_CTRL"), ("hips_rot", "HIPS_CTRL"),
                     ("chest_rot", "CHEST_CTRL"), ("head_rot", "HEAD_CTRL")):
        a, b = IDX[nm]; set_wrot(bone, x[a:b])
    a, b = IDX["footR_loc"]
    set_wloc("FOOT_IK_R", (x[a] / 100.0, x[a + 1] / 100.0, footR_z))
    for nm, bone in (("uaR", "UPPERARM_FK_R"), ("haR", "HAND_FK_R"), ("uaL", "UPPERARM_FK_L")):
        a, b = IDX[nm]; set_lrot(bone, x[a:b])
    for nm, bone in (("faR", "FOREARM_FK_R"), ("faL", "FOREARM_FK_L")):
        a, b = IDX[nm]; set_lrot(bone, (x[a], x[a + 1], 0.0))
    rig.update_tag()
    bpy.context.view_layer.update()


def landmarks():
    dg = bpy.context.evaluated_depsgraph_get()
    ev = rig.evaluated_get(dg)
    pb = ev.pose.bones
    W = ev.matrix_world
    p = {}
    wb = pb["WEAPON"]
    tail = W @ wb.tail
    head = W @ wb.head
    ax = (tail - head)
    ax = ax.normalized() if ax.length > 1e-9 else Vector((0, 0, 1))
    p["weapon_tip"] = tail
    p["club_head"] = tail - ax * CLUB_BLOBFIT
    p["club_butt"] = head - ax * CLUB_BUTT
    p["hand_R"] = W @ pb["HAND_R"].head
    p["elbow_R"] = W @ pb["FOREARM_R"].head
    p["elbow_L"] = W @ pb["FOREARM_L"].head
    p["hand_L"] = W @ ((pb["HAND_L"].head + pb["HAND_L"].tail) / 2.0)
    p["hips"] = W @ pb["HIPS"].head
    for k, (bn, off) in LOCALPT.items():
        p[k] = W @ (pb[bn].matrix @ off)
    p["head"] = (p["eye_L"] + p["eye_R"]) / 2.0
    px = {}
    for k, v in p.items():
        c = world_to_camera_view(sc, cam, v)
        px[k] = np.array([c.x * REF_W, (1.0 - c.y) * REF_H])
    return p, px


def screen_metrics(px):
    d = px["eye_L"] - px["eye_R"]
    return {"roll": math.degrees(math.atan2(d[1], d[0])),
            "sep": float(np.hypot(*d)),
            "scalp": float(px["head"][1] - px["head_top"][1]),
            "tilt": math.degrees(math.atan2(px["head"][0] - px["hips"][0],
                                            px["hips"][1] - px["head"][1]))}


# f1 calibration: the reference is a different (rubber-hose) character, so both
# boxes carry a constant offset which we remove at the rest pose (= reference f1).
clear_pose()
rig.update_tag(); bpy.context.view_layer.update()
_s0, _b0 = boxes()
SIL_CAL = _s0 - np.array(MET[0]["silhouette_bbox"], dtype=float)
BELT_CAL = _b0 - np.array(MET[0]["hips_belt_bbox"], dtype=float)
print("step05_solve: sil_cal %s  belt_cal %s"
      % (np.round(SIL_CAL, 1).tolist(), np.round(BELT_CAL, 1).tolist()))


# ------------------------------------------------------------------ targets
def targets(f):
    m = MET[f - 1]
    t = {}
    h = m["hips_belt_centroid_filled"]
    t["hips"] = (h[0] + T.HIPS_CAL[0], h[1] + T.HIPS_CAL[1])
    t["head"] = tuple(m["eye_mid"])
    # metrics eyes[0] is the SCREEN-LEFT disc = the character's RIGHT eye
    t["eye_R"] = tuple(m["eyes"][0])
    t["eye_L"] = tuple(m["eyes"][1])
    t["roll"] = m["head_roll_deg"]
    t["sep"] = m["eye_sep_px"]
    t["scalp"] = m["scalp_above_eyes_px"]
    t["tilt"] = m["spine_lean_deg_screen"]
    fr = T.FOOT_R[f]
    if "px" in fr:
        t["foot_R"] = fr["px"]
    else:
        bb = m["feet_bboxes"][0]
        t["foot_R"] = ((bb[0] + bb[2]) / 2.0 + T.FOOT_CAL[0], bb[3] + T.FOOT_CAL[1])
    t["footR_z"] = fr["z"]
    c = T.CLUB[f]
    t["club_head"] = c["head"]
    t["club_w"] = c["w"]
    t["club_tip"] = None if c["tip"] is None else (c["tip"][0] + T.TIP_CAL[0],
                                                   c["tip"][1] + T.TIP_CAL[1])
    t["club_butt"] = c.get("butt")
    t["club_butt_w"] = c.get("butt_w", 0.0)
    t["club_depth"] = T.CLUB_DEPTH.get(f)
    t["hand_depth"] = T.HAND_DEPTH.get(f)
    t["elbowR_prior"] = T.ELBOW_R_PRIOR.get(f, 0.0)
    t["elbowL_prior"] = T.ELBOW_L_PRIOR.get(f, 0.0)
    t["elbow_R"] = T.ELBOW_R_PX.get(f)
    t["hand_R"] = T.grip_px(f)[0]
    t["hand_L"] = T.handl_px(f)[0]
    t["sil_box"] = np.array(m["silhouette_bbox"], dtype=float) + SIL_CAL
    t["belt_box"] = np.array(m["hips_belt_bbox"], dtype=float) + BELT_CAL
    # f13 is motion-blurred (the streak inflates the silhouette box); at f24-f27 the
    # club merges into the belt / foot blob so the belt box is only half trusted.
    t["sil_w"] = {13: 0.15}.get(f, 0.6)
    t["belt_w"] = {24: 0.6, 27: 0.9, 12: 1.2}.get(f, 1.5)
    t["head_yaw_prior"] = T.HEAD_YAW_PRIOR[f]
    t["torso_yaw_prior"] = T.TORSO_YAW_PRIOR[f]
    return t


# ------------------------------------------------------------------ residuals
STAGE_DOFS = {"body": BODY_DOFS, "armR": ARMR_DOFS, "armL": ARML_DOFS,
              "all": BODY_DOFS + ARMR_DOFS + ARML_DOFS}


def resid(x, f, t, stage, seed, regw):
    apply(x, t["footR_z"])
    p, px = landmarks()
    sm = screen_metrics(px)
    r = []
    if stage in ("body", "all"):
        r += list((px["hips"] - np.array(t["hips"])) * 3.0)
        # the two eye discs directly (well conditioned) instead of eye-mid + the
        # weak roll/separation scalars, which left the head yaw almost free.
        r += list((px["eye_R"] - np.array(t["eye_R"])) * 3.0)
        r += list((px["eye_L"] - np.array(t["eye_L"])) * 3.0)
        r += list((px["foot_R"] - np.array(t["foot_R"])) * 4.0)
        r += [(sm["scalp"] - t["scalp"]) * 2.5]                # head pitch proxy
        sil, belt = boxes()
        r += list((belt - t["belt_box"]) * t["belt_w"])
        # the silhouette box is dominated by the club, so it is only meaningful
        # once the arm is free too (stage "all"), never during the body stage.
        r += list((sil - t["sil_box"]) * (t["sil_w"] if stage == "all" else 0.0))
        cr, hr, chr_ = (x[IDX[n][0]:IDX[n][1]] for n in ("cog_rot", "hips_rot", "chest_rot"))
        tot = cr + hr + chr_
        # the egg body tilts mostly AS A WHOLE: fix the share of the total that each
        # control carries (COG 55 / HIPS 20 / CHEST 25).  This also kills the huge
        # cancelling COG/HIPS pairs the free solve kept finding.
        for v, sh in ((cr, 0.55), (hr, 0.20), (chr_, 0.25)):
            r += list((v - sh * tot) * 0.18)
        # total torso range, per axis
        for k, cap in ((0, 35.0), (1, 34.0), (2, 40.0)):
            r += [max(0.0, abs(tot[k]) - cap) * 8.0]
        r += [(tot[2] - t["torso_yaw_prior"]) * 0.12]
        # the HEAD_CTRL yaw is relative to the chest; total face yaw ~ torso + head
        hy = tot[2] + x[IDX["head_rot"][0] + 2]
        r += [(hy - t["head_yaw_prior"]) * 0.40]
        a, b = IDX["head_rot"]
        r += [max(0.0, float(np.linalg.norm(x[a:b])) - 55.0) * 8.0]
        a, b = IDX["cog_loc"]
        r += [max(0.0, abs(x[a]) - 28.0) * 8.0,
              max(0.0, -x[a + 2] - 22.0) * 8.0,
              max(0.0, x[a + 2] - 8.0) * 8.0]
    if stage in ("armR", "all"):
        w = t["club_w"]
        r += list((px["club_head"] - np.array(t["club_head"])) * 4.0 * w)
        if t["club_tip"] is not None:
            r += list((px["weapon_tip"] - np.array(t["club_tip"])) * 3.0 * w)
        if t["club_butt"] is not None:
            r += list((px["club_butt"] - np.array(t["club_butt"])) * t["club_butt_w"])
        r += list((px["hand_R"] - np.array(t["hand_R"])) * 2.0)
        if t["club_depth"] is not None:
            r += [(p["club_head"].y - t["club_depth"]) * 45.0]
        if t["hand_depth"] is not None:
            r += [(p["hand_R"].y - t["hand_depth"]) * 25.0]
        r += [(x[IDX["faR"][0]] - t["elbowR_prior"]) * 0.06]
        if t["elbow_R"] is not None:
            r += list((px["elbow_R"] - np.array(t["elbow_R"])) * T.ELBOW_R_PX_W)
        # wrist: keep HAND_FK_R within 35 deg of the forearm (club fused to a ring hand)
        a, b = IDX["haR"]
        wr = float(np.linalg.norm(x[a:b]))
        r += [max(0.0, wr - 35.0) * 12.0]
        # elbow range: rest flexion R ~ 44 deg -> -40 is straight, +115 is a deep fold
        e = x[IDX["faR"][0]]
        r += [max(0.0, -40.0 - e) * 12.0, max(0.0, e - 115.0) * 12.0]
        r += [max(0.0, abs(x[IDX["faR"][0] + 1]) - 60.0) * 8.0]
        r += [max(0.0, np.linalg.norm(x[IDX["uaR"][0]:IDX["uaR"][1]]) - 155.0) * 8.0]
    if stage in ("armL", "all"):
        r += list((px["hand_L"] - np.array(t["hand_L"])) * 2.0)
        e = x[IDX["faL"][0]]
        r += [(e - t["elbowL_prior"]) * 0.06]
        r += [max(0.0, -26.0 - e) * 12.0, max(0.0, e - 115.0) * 12.0]
        r += [max(0.0, abs(x[IDX["faL"][0] + 1]) - 60.0) * 8.0]
        r += [max(0.0, np.linalg.norm(x[IDX["uaL"][0]:IDX["uaL"][1]]) - 155.0) * 8.0]
    act = slice_of(STAGE_DOFS[stage])
    r += list((x[act] - seed[act]) * regw[act])
    return np.array(r, dtype=np.float64), px, p, sm


def lm(x0, f, t, stage, seed, regw, iters=60):
    act = slice_of(STAGE_DOFS[stage])
    x = x0.copy()
    r, _, _, _ = resid(x, f, t, stage, seed, regw)
    cost = float(r @ r)
    lam = 1e-2
    step = 0.4
    for _ in range(iters):
        J = np.empty((len(r), len(act)))
        for j, k in enumerate(act):
            xp = x.copy(); xp[k] += step
            rp, _, _, _ = resid(xp, f, t, stage, seed, regw)
            J[:, j] = (rp - r) / step
        A = J.T @ J
        g = J.T @ r
        ok = False
        for _try in range(8):
            try:
                d = np.linalg.solve(A + lam * (np.diag(np.diag(A)) + 1e-9 * np.eye(len(act))), -g)
            except np.linalg.LinAlgError:
                lam *= 10; continue
            xn = x.copy(); xn[act] += d
            rn, _, _, _ = resid(xn, f, t, stage, seed, regw)
            cn = float(rn @ rn)
            if cn < cost:
                x, r, cost, lam = xn, rn, cn, max(lam * 0.4, 1e-9)
                ok = True
                break
            lam *= 6.0
        if not ok or cost < 1e-9:
            break
    return x, cost


# ------------------------------------------------------------------ per-frame solve
def regweights(stage, f):
    w = np.zeros(NDOF)
    # cog depth (Y) is essentially unobservable from a single front view and moving
    # the body toward camera is a cheap way to fake reach -> pin it hard.
    a, b = IDX["cog_loc"]; w[a:b] = [0.06, 1.5, 0.06]
    a, b = IDX["cog_rot"]; w[a:b] = 0.10
    a, b = IDX["hips_rot"]; w[a:b] = 0.14
    a, b = IDX["chest_rot"]; w[a:b] = 0.22                    # keep the lean off the chest
    a, b = IDX["head_rot"]; w[a:b] = 0.08
    a, b = IDX["footR_loc"]; w[a:b] = 0.04
    for nm in ("uaR", "haR", "uaL"):
        a, b = IDX[nm]; w[a:b] = 0.02
    for nm in ("faR", "faL"):
        a, b = IDX[nm]; w[a] = 0.02; w[a + 1] = 0.05
    return w


rng = np.random.default_rng(20260920)
poses, report = {}, {}
prev = np.zeros(NDOF)

for f in T.KEYS:
    if ONLY and f not in ONLY:
        continue
    t = targets(f)
    if f in T.IDLE:
        x = np.zeros(NDOF)
        apply(x, 0.0)
        _, px = landmarks()
        sm = screen_metrics(px)
        poses[f] = x.tolist()
        report[f] = {"idle": True}
        prev = x.copy()
        print("f%02d IDLE (all controls at rest)" % f)
        continue

    seed = prev.copy()
    if f in T.BODY_SEED:
        cl, cr, hr, chr_, hd = T.BODY_SEED[f]
        a, b = IDX["cog_loc"]; seed[a:b] = np.array(cl) * 100.0
        a, b = IDX["cog_rot"]; seed[a:b] = cr
        a, b = IDX["hips_rot"]; seed[a:b] = hr
        a, b = IDX["chest_rot"]; seed[a:b] = chr_
        a, b = IDX["head_rot"]; seed[a:b] = hd
    a, b = IDX["footR_loc"]
    bb = MET[f - 1]["feet_bboxes"][0]
    seed[a] = -(151.5 - (bb[0] + bb[2]) / 2.0) / 197.0 * 100.0     # screen-left = -X
    seed[a + 1] = prev[a + 1]

    # `seed` stays the regularisation ANCHOR for every stage, including the joint
    # polish.  (Re-anchoring on the current x turns the prior into a trust region
    # with zero cost, and the torso then drifts into a big invisible-to-the-metric
    # twist - which is exactly what wrecked the first pass.)
    x = seed.copy()
    rw = regweights("body", f)
    x, c = lm(x, f, t, "body", seed, rw, iters=70)
    best = (c, x.copy())
    for _ in range(6):
        s2 = seed.copy()
        idx = slice_of(BODY_DOFS)
        s2[idx] += rng.normal(0, 4.0, len(idx))
        x2, c2 = lm(s2, f, t, "body", seed, rw, iters=45)
        if c2 < best[0]:
            best = (c2, x2.copy())
    x = best[1]
    body_cost = best[0]
    seed[slice_of(BODY_DOFS)] = x[slice_of(BODY_DOFS)]   # body is now the anchor

    # ---- right arm: multistart (the arm is the part that can flip)
    idxR = slice_of(ARMR_DOFS)
    cands = [prev[idxR].copy(), np.zeros(len(idxR))]
    for _ in range(64):
        cands.append(prev[idxR] + rng.normal(0, 55.0, len(idxR)))
    bestR = None
    sdR = x.copy(); sdR[idxR] = prev[idxR]
    for c0 in cands:
        s2 = x.copy(); s2[idxR] = c0
        x2, c2 = lm(s2, f, t, "armR", sdR, rw, iters=22)
        if bestR is None or c2 < bestR[0]:
            bestR = (c2, x2.copy())
    x, armR_cost = lm(bestR[1], f, t, "armR", sdR, rw, iters=60)
    seed[idxR] = x[idxR]

    # ---- left arm
    idxL = slice_of(ARML_DOFS)
    candsL = [prev[idxL].copy(), np.zeros(len(idxL))]
    for _ in range(32):
        candsL.append(prev[idxL] + rng.normal(0, 45.0, len(idxL)))
    bestL = None
    sdL = x.copy(); sdL[idxL] = prev[idxL]
    for c0 in candsL:
        s2 = x.copy(); s2[idxL] = c0
        x2, c2 = lm(s2, f, t, "armL", sdL, rw, iters=20)
        if bestL is None or c2 < bestL[0]:
            bestL = (c2, x2.copy())
    x, armL_cost = lm(bestL[1], f, t, "armL", sdL, rw, iters=40)
    seed[idxL] = x[idxL]

    # ---- joint polish: body + both arms together, so the torso twist can help the
    #      right arm reach across the body instead of the elbow hyper-extending.
    x, joint_cost = lm(x, f, t, "all", seed, rw, iters=60)

    apply(x, t["footR_z"])
    p, px = landmarks()
    sm = screen_metrics(px)
    poses[f] = x.tolist()

    def e(k, tk):
        return None if tk is None else round(float(np.linalg.norm(px[k] - np.array(tk))), 2)
    report[f] = {
        "cost": [round(body_cost, 1), round(armR_cost, 1), round(armL_cost, 1),
                 round(joint_cost, 1)],
        "err_px": {"club_head": e("club_head", t["club_head"]),
                   "weapon_tip": e("weapon_tip", t["club_tip"]),
                   "club_butt": e("club_butt", t["club_butt"]),
                   "head": e("head", t["head"]),
                   "hips": e("hips", t["hips"]),
                   "foot_R": e("foot_R", t["foot_R"]),
                   "hand_R": e("hand_R", t["hand_R"]),
                   "hand_L": e("hand_L", t["hand_L"])},
        "angles": {"roll": [round(sm["roll"], 2), t["roll"]],
                   "sep": [round(sm["sep"], 1), t["sep"]],
                   "scalp": [round(sm["scalp"], 1), t["scalp"]],
                   "tilt": [round(sm["tilt"], 2), t["tilt"]]},
        "club_depth_y": round(p["club_head"].y, 3),
        "wrist_deg": round(float(np.linalg.norm(x[IDX["haR"][0]:IDX["haR"][1]])), 1),
        "elbowR_deg": round(x[IDX["faR"][0]], 1),
        "elbowL_deg": round(x[IDX["faL"][0]], 1),
    }
    print("f%02d %s" % (f, json.dumps(report[f]["err_px"])))
    print("     ang %s  clubY %.2f wrist %.0f elbR %.0f" % (
        json.dumps(report[f]["angles"]), p["club_head"].y,
        report[f]["wrist_deg"], report[f]["elbowR_deg"]))
    prev = x.copy()

clear_pose()
out = {"dof_layout": {k: list(v) for k, v in IDX.items()}, "ndof": NDOF,
       "units": "rotations in degrees, locations in centimetres",
       "footR_z": {str(f): T.FOOT_R[f]["z"] for f in T.KEYS},
       "poses": {str(k): v for k, v in poses.items()}}
mode = "r+" if (ONLY and os.path.isfile(os.path.join(ROOT, "scripts", "step05_poses.json"))) else "w"
pth = os.path.join(ROOT, "scripts", "step05_poses.json")
if ONLY and os.path.isfile(pth):
    old = json.load(open(pth))
    old["poses"].update(out["poses"])
    out = old
with open(pth, "w") as fh:
    json.dump(out, fh, indent=1)
with open(os.path.join(ROOT, "inspect", "step05_solve.json"), "w") as fh:
    json.dump(report, fh, indent=1, default=str)
print("@@@SOLVE_DONE@@@")
