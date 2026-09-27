"""GATE 03R - rigid sliding leg test for goblin_v05_legfix.blend.

Run: blender -b goblin_v05_legfix.blend --python step03r_gate.py -- \
        [--mesh lo|hi] [--only k1,k2] [--res 768] [--norender] [--tag NAME]
NEVER saves.  Every pose is temporary; rest is restored and re-verified at the end.
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Quaternion, Matrix
from mathutils.bvhtree import BVHTree

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d=None):
    return argv[argv.index(n) + 1] if n in argv else d


MESH = opt("--mesh", "lo")
TAG = opt("--tag", MESH)
ONLY = set((opt("--only", "") or "").split(",")) - {""}
RES = int(opt("--res", "768"))
NORENDER = "--norender" in argv
OUT = os.path.join(ROOT, "inspect", "step03r")
os.makedirs(OUT, exist_ok=True)
REPORT = os.path.join(ROOT, "inspect", "step03r_gate_%s.json" % TAG)

sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
body = bpy.data.objects["GOB_body_lo" if MESH == "lo" else "GOB_body"]
other = bpy.data.objects["GOB_body" if MESH == "lo" else "GOB_body_lo"]
for o, hid in ((body, False), (other, True)):
    o.hide_render = hid
    o.hide_viewport = hid
    for c in o.users_collection:
        c.hide_render = hid
        c.hide_viewport = hid
me = body.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
Z0 = P0[:, 2]
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
R = {"tag": TAG, "mesh": body.name, "n_verts": N,
     "preserve_volume": body.modifiers[0].use_deform_preserve_volume}

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
gi = {g.index: g.name for g in body.vertex_groups}
W = np.zeros((N, len(BONES)))
for v in me.vertices:
    for g in v.groups:
        n = gi.get(g.group)
        if n in BI:
            W[v.index, BI[n]] = g.weight
lab = np.argmax(W, axis=1)
SET = {n: (lab == BI[n]) for n in BONES}
SET["TORSO"] = SET["HIPS"] | SET["SPINE_01"] | SET["CHEST"]
for S in ("L", "R"):
    # PURE tube = carried 100% by the rigid leg (no share of HIPS or FOOT at all)
    SET["TUBE_" + S] = (W[:, BI["THIGH_" + S]] + W[:, BI["SHIN_" + S]]) > 0.9995
    SET["LEG_" + S] = SET["TUBE_" + S] | SET["FOOT_" + S]
LEGW = sum(W[:, BI[n]] for n in ("THIGH_L", "SHIN_L", "FOOT_L",
                                 "THIGH_R", "SHIN_R", "FOOT_R"))
# the torso underside that the junction has to keep honest: every vertex below the
# belt that is NOT purely carried by the rigid leg (pure-HIPS skin + the blend band).
SET["TORSO_UNDER"] = (Z0 < 0.55) & (LEGW < 0.9995) & (W[:, BI["HIPS"]] > 1e-6)
SET["JUNCTION"] = SET["TORSO_UNDER"] & (LEGW > 1e-6)
# rigid reference sets defined by weight PURITY, so they mean the same thing on the
# high-poly and on the decimated proxy
SET["PURE_CLUB"] = W[:, BI["WEAPON"]] > 0.9995
SET["PURE_RINGHAND"] = W[:, BI["HAND_R"]] > 0.9995
SET["PURE_HEADCORE"] = (W[:, BI["HEAD"]] > 0.9995) & (Z0 > 1.375)
for S in ("L", "R"):
    SET["PURE_FOOT_" + S] = W[:, BI["FOOT_" + S]] > 0.9995
R["set_sizes"] = {k: int(v.sum()) for k, v in SET.items()}

me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)
FG = {k: np.nonzero(SET[k][TRI].all(axis=1))[0]
      for k in ("TORSO", "TUBE_L", "TUBE_R", "LEG_L", "LEG_R")}
R["face_groups"] = {k: int(len(v)) for k, v in FG.items()}

# ---- leg axial coordinates (rest)
AX = {}
for S in ("L", "R"):
    A = np.array(arm.bones["FOOT_" + S].head_local)
    H = np.array(arm.bones["THIGH_" + S].head_local)
    L = float(np.linalg.norm(H - A))
    u = (H - A) / L
    V = P0 - A
    s = V @ u
    r = np.linalg.norm(V - s[:, None] * u, axis=1)
    AX[S] = {"A": A, "H": H, "L": L, "u": u, "s": s, "r": r}
R["leg_L_m"] = {S: round(AX[S]["L"], 6) for S in ("L", "R")}

# tube cross-section ring: the middle of the tube
RING = {S: np.nonzero(SET["TUBE_" + S] & (AX[S]["s"] > 0.09) & (AX[S]["s"] < 0.15))[0]
        for S in ("L", "R")}
# top of the tube - the part that should get swallowed by the torso
TOP = {S: np.nonzero(SET["TUBE_" + S] & (AX[S]["s"] > 0.15))[0] for S in ("L", "R")}
R["ring_sizes"] = {S: int(len(RING[S])) for S in ("L", "R")}
R["legtop_sizes"] = {S: int(len(TOP[S])) for S in ("L", "R")}


def xsec(Q, idx, S, u=None):
    """mean radius of a tube ring, measured perpendicular to the CURRENT leg axis
    (using the rest axis would report a pure cos(tilt) artefact)."""
    p = Q[idx]
    c = p.mean(0)
    u = AX[S]["u"] if u is None else u
    v = p - c
    perp = v - (v @ u)[:, None] * u
    return float(np.linalg.norm(perp, axis=1).mean())


XS0 = {S: xsec(P0, RING[S], S) for S in ("L", "R")}


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, Sg, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return float(np.linalg.norm((Rm @ (A - ca).T).T + cb - B, axis=1).max())


def volume(Q):
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


V0 = volume(P0)


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
        PB["FOOT_IK_" + S]["leg_roll"] = 0.0


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


def deformed(dg):
    ev = body.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3)
    dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return Q.reshape(-1, 3)


# ---------------------------------------------------------------- renderer
def rsetup():
    sc.render.engine = 'BLENDER_WORKBENCH'
    sh = sc.display.shading
    sh.light = 'STUDIO'
    sh.studio_light = 'Default'
    sh.color_type = 'SINGLE'
    sh.single_color = (0.72, 0.72, 0.74)
    sh.show_cavity = True
    sh.cavity_type = 'BOTH'
    sh.curvature_ridge_factor = 1.0
    sh.curvature_valley_factor = 1.0
    sh.show_shadows = True
    sh.shadow_intensity = 0.45
    sh.show_object_outline = False
    sc.display.render_aa = '8'
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = 'PNG'
    sc.render.resolution_percentage = 100
    cd = bpy.data.cameras.get("s3r_cam") or bpy.data.cameras.new("s3r_cam")
    cd.type = 'ORTHO'
    cam = bpy.data.objects.get("s3r_cam")
    if cam is None:
        cam = bpy.data.objects.new("s3r_cam", cd)
        sc.collection.objects.link(cam)
    return cam, cd


AUXCAM, AUXCD = rsetup()
REFCAM = bpy.data.objects["REF_CAM"]
GROUND = bpy.data.objects.get("REF_ground")
if GROUND:
    GROUND.hide_render = True
SHOTS = []


def shoot_ref(name):
    if NORENDER:
        return
    sc.camera = REFCAM
    sc.render.resolution_x = int(448 * RES / 576.0)
    sc.render.resolution_y = RES
    sc.render.filepath = os.path.join(OUT, "%s_%s_ref.png" % (TAG, name))
    bpy.ops.render.render(write_still=True)
    SHOTS.append(sc.render.filepath)


def shoot_legs(name, az=32, el=-14, scale=1.15, target=(0.0, 0.05, 0.30), sfx="legs"):
    if NORENDER:
        return
    sc.camera = AUXCAM
    sc.render.resolution_x = RES
    sc.render.resolution_y = RES
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    ctr = Vector(target)
    AUXCAM.location = ctr + d * 8.0
    AUXCD.ortho_scale = scale
    AUXCAM.rotation_mode = 'XYZ'
    AUXCAM.rotation_euler = (ctr - AUXCAM.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, "%s_%s_%s.png" % (TAG, name, sfx))
    bpy.ops.render.render(write_still=True)
    SHOTS.append(sc.render.filepath)


# ---------------------------------------------------------------- measurement
def measure(dg):
    ev = rig.evaluated_get(dg)
    Q = deformed(dg)
    row = {"volume_pct": round((volume(Q) / V0 - 1.0) * 100.0, 3),
           "max_disp_m": round(float(np.linalg.norm(Q - P0, axis=1).max()), 4)}
    legs = {}
    for S in ("L", "R"):
        mf = ev.pose.bones["FOOT_" + S].matrix
        mi = ev.pose.bones["FOOT_IK_" + S].matrix
        aim = ev.pose.bones["MCH_LEGAIM_" + S].matrix
        top = ev.pose.bones["MCH_LEGTOP_" + S].matrix.to_translation()
        stretch = aim.to_scale()[1]
        d_ha = (top - aim.to_translation()).length
        # knee angle (THIGH dir vs SHIN dir)
        th = ev.pose.bones["THIGH_" + S].matrix
        shn = ev.pose.bones["SHIN_" + S].matrix
        kt = th.to_translation()
        kn = shn.to_translation()
        an = shn @ Vector((0, LEN["SHIN_" + S], 0))
        knee = math.degrees((kn - kt).angle(an - kn)) if (kn - kt).length > 1e-9 else 0.0
        # rigid residual of the tube (should be ~0 while stretch == 1)
        tub = np.nonzero(SET["TUBE_" + S])[0]
        legs[S] = {
            "foot_pos_err_m": round((mf.to_translation() - mi.to_translation()).length, 9),
            "foot_rot_err_deg": round(math.degrees(abs(
                mf.to_quaternion().rotation_difference(mi.to_quaternion()).angle)), 7),
            "shoe_rigid_resid_m": round(kabsch(P0[SET["PURE_FOOT_" + S]],
                                               Q[SET["PURE_FOOT_" + S]]), 8),
            "tube_rigid_resid_m": round(kabsch(P0[tub], Q[tub]), 6),
            "stretch": round(float(stretch), 6),
            "d_hip_ankle_m": round(d_ha, 6),
            "knee_deg": round(knee, 4),
            "xsec_ratio": round(xsec(Q, RING[S], S,
                                     np.array((aim.to_3x3() @ Vector((0, 1, 0))
                                               ).normalized())) / XS0[S], 4),
            "legtop_z": round(float(kt.z), 4),
        }
        # visible tube length above the torso underside is judged from renders
    row["legs"] = legs

    # torso-underside dent/tent: deviation from the rigid HIPS motion
    mh = ev.pose.bones["HIPS"].matrix @ REST["HIPS"].inverted()
    M = np.array(mh)
    for key, und in (("torso_under", SET["TORSO_UNDER"]), ("junction", SET["JUNCTION"])):
        row[key + "_n"] = int(und.sum())
        if not und.any():
            row[key + "_dent_max_m"] = None
            row[key + "_dent_p99_m"] = None
            continue
        pred = (np.c_[P0[und], np.ones(und.sum())] @ M.T)[:, :3]
        dv = np.linalg.norm(Q[und] - pred, axis=1)
        row[key + "_dent_max_m"] = round(float(dv.max()), 5)
        row[key + "_dent_p99_m"] = round(float(np.percentile(dv, 99)), 5)
        row[key + "_n"] = int(und.sum())

    # Leg-top poke-out.  The mesh is ONE manifold island, so the only way leg-top
    # geometry can become visible after it has sunk into the egg is if the tube
    # surface CROSSES the torso surface.  Test exactly that: BVH overlap between the
    # pure-tube faces and the torso faces of the DEFORMED mesh.
    def bvh_of(faces):
        return BVHTree.FromPolygons([Vector(p) for p in Q],
                                    [tuple(int(i) for i in TRI[f]) for f in faces],
                                    all_triangles=True, epsilon=0.0)

    po = {}
    if len(FG["TORSO"]):
        tb = bvh_of(FG["TORSO"])
        for S in ("L", "R"):
            if not len(FG["TUBE_" + S]):
                po[S] = {"crossing_face_pairs": None}
                continue
            ov = bvh_of(FG["TUBE_" + S]).overlap(tb)
            depth = 0.0
            if ov:
                vs = set()
                for ia, ib in ov:
                    vs.update(int(x) for x in TRI[FG["TUBE_" + S][ia]])
                for i in list(vs)[:300]:
                    loc, nor, fi, dist = tb.find_nearest(Vector(Q[i]))
                    if loc is not None and (Vector(Q[i]) - loc).dot(nor) < 0:
                        depth = max(depth, dist)
            po[S] = {"crossing_face_pairs": len(ov), "depth_m": round(depth, 5)}
    row["tube_crosses_torso"] = po
    return row, Q


# ---------------------------------------------------------------- poses
def P_rest():
    return "rest"


def P_cog(dz=0.0, dx=0.0):
    def f():
        wloc("COG_CTRL", (dx, 0.0, dz))
        return "COG (%.2f,0,%.2f)" % (dx, dz)
    return f


def P_hips(axis, deg):
    def f():
        wrot("HIPS_CTRL", axis, deg)
        return "HIPS_CTRL %s %d deg, feet planted" % (axis, deg)
    return f


def P_stepR_air():
    wloc("FOOT_IK_R", (-0.15, 0.0, 0.10))
    return "right foot airborne: out 0.15, up 0.10 (ref f11)"


def P_stanceR():
    wloc("FOOT_IK_R", (-0.15, 0.0, 0.0))
    wloc("COG_CTRL", (-0.05, 0.0, -0.12))
    return "wide planted stance (ref f13-f15)"


def P_f15():
    wloc("FOOT_IK_R", (-0.15, 0.0, 0.0))
    wloc("COG_CTRL", (-0.05, 0.0, -0.15))
    wrot("CHEST_CTRL", (1, 0, 0), 25)
    wrot("CHEST_CTRL", (0, 0, 1), 30, add=True)
    return "ref-f15-like composite"


POSES = [
    ("rest", P_rest),
    ("cog_dn05", P_cog(-0.05)),
    ("cog_dn10", P_cog(-0.10)),
    ("cog_dn15", P_cog(-0.15)),
    ("cog_dn20", P_cog(-0.20)),
    ("cog_latP10", P_cog(0.0, 0.10)),
    ("cog_latM10", P_cog(0.0, -0.10)),
    ("cog_up03", P_cog(0.03)),
    ("hips_yawP30", P_hips((0, 0, 1), 30)),
    ("hips_yawM30", P_hips((0, 0, 1), -30)),
    ("hips_rollP10", P_hips((0, 1, 0), 10)),
    ("hips_rollM10", P_hips((0, 1, 0), -10)),
    ("stepR_air", P_stepR_air),
    ("stanceR", P_stanceR),
    ("f15like", P_f15),
]
RENDER_REF = {"rest", "cog_dn05", "cog_dn10", "cog_dn15", "cog_dn20", "cog_up03",
              "hips_yawP30", "hips_rollP10", "stepR_air", "stanceR", "f15like",
              "cog_latM10"}
RENDER_LEGS = {"rest", "cog_dn10", "cog_dn15", "cog_dn20", "cog_up03",
               "stepR_air", "stanceR", "f15like", "hips_yawP30", "hips_rollP10"}
RENDER_ROOT = {"rest", "cog_dn20", "f15like"}

TESTS = {}
t0 = time.time()
for key, fn in POSES:
    if ONLY and key not in ONLY:
        continue
    clear_pose()
    note = fn()
    dg = upd()
    row, Q = measure(dg)
    row["note"] = note
    TESTS[key] = row
    if key in RENDER_REF:
        shoot_ref(key)
    if key in RENDER_LEGS:
        shoot_legs(key)
    if key in RENDER_ROOT:
        # tight on the character's LEFT leg root (screen-right), the junction that
        # has to absorb the sinking
        shoot_legs(key, az=15, el=-8, scale=0.42, target=(0.29, 0.09, 0.34), sfx="root")
    print("### %s %s" % (key, json.dumps(row["legs"])))

# ---------------------------------------------------------------- roll sweep
if not ONLY:
    clear_pose()
    sweep = []
    u0 = {}
    x0 = {}
    dg = upd()
    ev = rig.evaluated_get(dg)
    for S in ("L", "R"):
        m = ev.pose.bones["MCH_LEGAIM_" + S].matrix
        u0[S] = (m.to_3x3() @ Vector((0, 1, 0))).normalized()
        x0[S] = (m.to_3x3() @ Vector((1, 0, 0))).normalized()
    for k in range(21):
        t = k / 20.0
        clear_pose()
        wloc("FOOT_IK_R", (-0.15 * t, 0.0, 0.0))
        wloc("COG_CTRL", (-0.05 * t, 0.0, -0.15 * t))
        wrot("CHEST_CTRL", (1, 0, 0), 25 * t)
        dg = upd()
        ev = rig.evaluated_get(dg)
        rowk = {"t": round(t, 3)}
        for S in ("L", "R"):
            m = ev.pose.bones["MCH_LEGAIM_" + S].matrix
            u = (m.to_3x3() @ Vector((0, 1, 0))).normalized()
            x = (m.to_3x3() @ Vector((1, 0, 0))).normalized()
            # parallel-transport the rest X along the minimal rotation u0 -> u
            q = u0[S].rotation_difference(u)
            xp = (q @ x0[S]).normalized()
            tw = math.degrees(math.atan2((xp.cross(x)).dot(u), xp.dot(x)))
            rowk["tilt_" + S] = round(math.degrees(u0[S].angle(u)), 3)
            rowk["twist_" + S] = round(tw, 5)
        sweep.append(rowk)
    R["roll_sweep"] = sweep
    R["roll_max_twist_deg"] = {S: round(max(abs(r["twist_" + S]) for r in sweep), 5)
                               for S in ("L", "R")}
    R["roll_max_tilt_deg"] = {S: round(max(r["tilt_" + S] for r in sweep), 3)
                              for S in ("L", "R")}
    jump = {S: round(max(abs(sweep[i + 1]["twist_" + S] - sweep[i]["twist_" + S])
                         for i in range(len(sweep) - 1)), 5) for S in ("L", "R")}
    R["roll_max_step_deg"] = jump

# ---------------------------------------------------------------- upper body regression
UP = [("armR_up90", lambda: wrot("UPPERARM_FK_R", (0, 1, 0), 90), ["UPPERARM_R"]),
      ("elbowR_90", lambda: lrot("FOREARM_FK_R", (1, 0, 0), 90), ["FOREARM_R"]),
      ("chest_twist45", lambda: wrot("CHEST_CTRL", (0, 0, 1), 45), []),
      ("head_yaw40", lambda: wrot("HEAD_CTRL", (0, 0, 1), 40), []),
      ("wristR_p30", lambda: lrot("HAND_FK_R", (1, 0, 0), 30), []),
      ("wristR_m30", lambda: lrot("HAND_FK_R", (1, 0, 0), -30), [])]
UPR = {}
for name, fn, _ in UP:
    if "--noupper" in argv:
        break
    clear_pose()
    fn()
    dg = upd()
    Q = deformed(dg)
    UPR[name] = {
        "volume_pct": round((volume(Q) / V0 - 1.0) * 100.0, 3),
        "club_rigid_resid_m": round(kabsch(P0[SET["PURE_CLUB"]], Q[SET["PURE_CLUB"]]), 7),
        "ring_hand_rigid_m": round(kabsch(P0[SET["PURE_RINGHAND"]],
                                          Q[SET["PURE_RINGHAND"]]), 7),
        "headcore_rigid_m": round(kabsch(P0[SET["PURE_HEADCORE"]],
                                         Q[SET["PURE_HEADCORE"]]), 7),
        "leg_static_max_m": round(float(np.linalg.norm(
            Q[SET["LEG_L"] | SET["LEG_R"]] - P0[SET["LEG_L"] | SET["LEG_R"]],
            axis=1).max()), 6),
    }
    if MESH == "lo" and name in ("armR_up90", "chest_twist45", "wristR_p30"):
        shoot_ref("up_" + name)
R["upper_regression"] = UPR

# ---------------------------------------------------------------- restore
clear_pose()
dg = upd()
Q = deformed(dg)
R["final_rest_max_dev_m"] = float(np.linalg.norm(Q - P0, axis=1).max())
R["residual_posed_bones"] = [pb.name for pb in PB if pb.location.length > 1e-9 or
                             (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9]
R["has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["n_actions"] = len(bpy.data.actions)
R["tests"] = TESTS
R["renders"] = SHOTS
R["seconds"] = round(time.time() - t0, 1)
with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", REPORT, R["seconds"])
