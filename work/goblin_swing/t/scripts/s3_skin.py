"""STEP 03 - add FOREARM_TWIST_L/R and skin both goblin meshes analytically.

Run: blender -b t\gobT_v02_rig.blend --python t\scripts\s3_skin.py -- [--p k=v ...] [--out PATH]

Rules (identical for both meshes, purely analytic from the REST skeleton):
  torso  : z-gradient HIPS -> SPINE_01 -> CHEST, belt band rigid HIPS
  head   : above the crease rigid HEAD, tight NECK blend in the crease
  arms   : arm share = Ramp(|x|; 0.40 -> 0.48) * Collar(d - rtube(s))  then
           Laplacian-smoothed inside the band (torso=0 / tube=1 Dirichlet);
           along the arm: UPPERARM -> FOREARM -> FOREARM_TWIST -> HAND
  legs   : rigid sliding tube, blend band above the tube/torso merge ring,
           conic feather into the torso underside, shoes rigid FOOT
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Matrix

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
INSP = os.path.join(T, "inspect", "step03")
os.makedirs(INSP, exist_ok=True)
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join(T, "gobT_v03_skinned.blend")
TARGET = argv[argv.index("--target") + 1] if "--target" in argv else "both"
LINEAR = "--linear" in argv

P_ = {
    # shoulder
    "sh0": 0.400, "sh1": 0.480,          # Ramp on |x|
    "collar0": 0.010, "collar1": 0.040,  # Collar on dd = d - rtube(s)
    "smooth_it": 40,
    "arm_hardzero_x": 0.385,
    # arm chain (s = arc length along shoulder->elbow->wrist->handtail)
    "elbow_w": 0.095,                    # half-width of the elbow band
    "twist_w": 0.055,                    # half-width of the FOREARM->TWIST band
    # corrective smooth (joint-limited), 0 disables
    "cs_factor": 0.90, "cs_iter": 25,
    "cs_elbow_r0": 0.055, "cs_elbow_r1": 0.175,
    "cs_sh_r0": 0.070, "cs_sh_r1": 0.190,
    "cs_wrist_r0": 0.045, "cs_wrist_r1": 0.110,
    "wrist_lo": 0.024, "wrist_hi": 0.018,
    "arm_dmax_tube": 0.130,              # arm-zone radius on the tube
    "arm_s_hand": 0.480,                 # s past which the hand ball radius applies
    "arm_dmax_hand": 0.300,
    "arm_s0": 0.015,
    # torso / head
    "tz_hs0": 0.780, "tz_hs1": 1.000,
    "tz_sc0": 0.960, "tz_sc1": 1.220,
    "belt0": 0.583, "belt1": 0.788,
    "neck0": 1.288, "neck1": 1.313,      # CHEST -> NECK
    "head0": 1.308, "head1": 1.333,      # NECK -> HEAD
    "head_rt0": 0.355, "head_rt1": 0.400,
    "head_z0": 1.335, "head_z1": 1.350,
    "head_core_z": 1.360,
    # legs
    "leg_top0": 0.196, "leg_top1": 0.230,
    "leg_cone": 1.20, "leg_rtube": 0.110,
    "knee0": 0.100, "knee1": 0.135,
    "ankle0": 0.010, "ankle1": 0.078,
    "shoe_z": 0.180,
    "leg_rz": 0.46, "leg_sz": 0.34, "leg_zcap": 0.55,
}
for i, a in enumerate(argv):
    if a == "--p":
        k, v = argv[i + 1].split("=")
        P_[k] = float(v)

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
REP = {"params": dict(P_)}


def smoothstep(x, a, b):
    t = np.clip((np.asarray(x, dtype=float) - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# =====================================================================  RIG: twist bones
def add_twist():
    info = {}
    prev = bpy.context.view_layer.objects.active
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    bpy.context.view_layer.objects.active = rig
    rig.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.edit_bones
    for S in ("L", "R"):
        nm = "FOREARM_TWIST_" + S
        if nm in eb:
            eb.remove(eb[nm])
        fa = eb["FOREARM_" + S]
        h = fa.head.copy()
        t = fa.tail.copy()
        mid = (h + t) * 0.5
        b = eb.new(nm)
        b.head = mid
        b.tail = t.copy()
        b.roll = fa.roll                 # same frame as FOREARM / HAND
        b.parent = fa
        b.use_connect = False
        b.use_deform = True
        b.inherit_scale = fa.inherit_scale
        info[nm] = {"head": [round(float(x), 5) for x in b.head],
                    "tail": [round(float(x), 5) for x in b.tail],
                    "roll_deg": round(math.degrees(b.roll), 4),
                    "len": round(float(b.length), 5), "parent": fa.name}
    bpy.ops.object.mode_set(mode='OBJECT')
    # bone collection membership: follow FOREARM_x
    for S in ("L", "R"):
        nm = "FOREARM_TWIST_" + S
        src = arm.bones["FOREARM_" + S]
        for bc in arm.collections_all:
            if src.name in [b.name for b in bc.bones]:
                bc.assign(arm.bones[nm])
                info[nm]["collections"] = info[nm].get("collections", []) + [bc.name]
    # constraint: half the wrist twist
    for S in ("L", "R"):
        pb = rig.pose.bones["FOREARM_TWIST_" + S]
        for c in list(pb.constraints):
            pb.constraints.remove(c)
        c = pb.constraints.new('COPY_ROTATION')
        c.name = "CR_twist_half"
        c.target = rig
        c.subtarget = "HAND_" + S
        c.use_x = False
        c.use_y = True
        c.use_z = False
        c.euler_order = 'YXZ'            # Y innermost => pure twist about the bone axis
        c.owner_space = 'LOCAL'
        c.target_space = 'LOCAL'
        c.mix_mode = 'REPLACE'
        c.influence = 0.5
        pb.rotation_mode = 'YXZ'
        info["FOREARM_TWIST_" + S]["constraint"] = {
            "type": c.type, "target": c.subtarget, "axes": [c.use_x, c.use_y, c.use_z],
            "euler_order": c.euler_order, "owner_space": c.owner_space,
            "target_space": c.target_space, "influence": c.influence}
    bpy.context.view_layer.objects.active = prev
    return info


REP["twist_bones"] = add_twist()
rig.update_tag()
bpy.context.view_layer.update()

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "FOREARM_TWIST_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "FOREARM_TWIST_R", "HAND_R",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
assert sorted(BONES) == sorted([b.name for b in arm.bones if b.use_deform]), \
    sorted([b.name for b in arm.bones if b.use_deform])

# =====================================================================  geometry helpers
ARMCH, LEGAX = {}, {}
for S in ("L", "R"):
    ARMCH[S] = [np.array(arm.bones["UPPERARM_" + S].head_local),
                np.array(arm.bones["FOREARM_" + S].head_local),
                np.array(arm.bones["HAND_" + S].head_local),
                np.array(arm.bones["HAND_" + S].tail_local)]
    A = np.array(arm.bones["FOOT_" + S].head_local)
    H = np.array(arm.bones["THIGH_" + S].head_local)
    u = H - A
    LEGAX[S] = (A, u / np.linalg.norm(u), float(np.linalg.norm(u)))
S_EL = {S: float(np.linalg.norm(ARMCH[S][1] - ARMCH[S][0])) for S in ("L", "R")}
S_WR = {S: S_EL[S] + float(np.linalg.norm(ARMCH[S][2] - ARMCH[S][1])) for S in ("L", "R")}
REP["arm_arclen"] = {S: {"elbow": round(S_EL[S], 5), "wrist": round(S_WR[S], 5)} for S in ("L", "R")}


def poly_td(P, pts):
    pts = np.asarray(pts, float)
    A, B = pts[:-1], pts[1:]
    AB = B - A
    L2 = (AB ** 2).sum(1)
    L = np.sqrt(L2)
    cum = np.concatenate([[0.0], np.cumsum(L)])
    bd = np.full(len(P), 1e9)
    bt = np.zeros(len(P))
    for k in range(len(A)):
        w = P - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0.0, 1.0)
        d = np.linalg.norm(w - tt[:, None] * AB[k], axis=1)
        m = d < bd
        bd[m] = d[m]
        bt[m] = cum[k] + tt[m] * L[k]
    return bt, bd


def readP(ob):
    me = ob.data
    n = len(me.vertices)
    P = np.empty(n * 3)
    me.vertices.foreach_get("co", P)
    return P.reshape(n, 3)


# ---- calibrate the true tube envelope rtube(s) on the HIGH-poly
hi = bpy.data.objects["GOB_body"]
PH = readP(hi)
CAL = {}
for S in ("L", "R"):
    s, d = poly_td(PH, ARMCH[S])
    xs, rs = [], []
    for lo in np.arange(0.02, S_WR[S] - 0.02, 0.01):
        m = (s >= lo) & (s < lo + 0.01) & (d < 0.130)
        if m.sum() >= 6:
            xs.append(float(lo) + 0.005)
            rs.append(float(np.clip(np.percentile(d[m], 92), 0.055, 0.100)))
    xs = np.array(xs)
    rs = np.array(rs)
    rs = np.maximum(rs, np.convolve(np.pad(rs, 1, mode='edge'), np.ones(3) / 3.0, mode='valid'))
    CAL[S] = (xs, rs)
REP["rtube_cal"] = {S: {"s": [round(float(x), 3) for x in CAL[S][0]],
                        "r": [round(float(x), 4) for x in CAL[S][1]]} for S in ("L", "R")}


def rtube_of(S, s):
    xs, rs = CAL[S]
    return np.interp(s, xs, rs, left=rs[0], right=rs[-1])


# =====================================================================  weight builder
def build(ob, tag):
    me = ob.data
    P = readP(ob)
    N = len(P)
    X, Y, Z = P[:, 0], P[:, 1], P[:, 2]
    aX = np.abs(X)
    rT = np.hypot(X, Y - 0.105)
    W = np.zeros((N, NB))
    info = {"verts": N}

    E = np.empty(len(me.edges) * 2, dtype=np.int32)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    ah = np.concatenate([E[:, 0], E[:, 1]])
    at = np.concatenate([E[:, 1], E[:, 0]])
    DEG = np.bincount(ah, minlength=N).astype(float)
    DEG[DEG == 0] = 1.0

    # ---------------------------------------------------------------- arms
    ARMZ = np.zeros(N, bool)
    ARMW = np.zeros(N)                       # total arm share per vertex
    SS, DD = {}, {}
    for S in ("L", "R"):
        s, d = poly_td(P, ARMCH[S])
        SS[S], DD[S] = s, d
        side = (X > 0) if S == "L" else (X < 0)
        dmax = np.where(s < P_["arm_s_hand"], P_["arm_dmax_tube"], P_["arm_dmax_hand"])
        zone = side & (s > P_["arm_s0"]) & (d < dmax) & (aX > P_["arm_hardzero_x"])
        ARMZ |= zone

        dd = d - rtube_of(S, s)
        raw = smoothstep(aX, P_["sh0"], P_["sh1"]) * \
            (1.0 - smoothstep(dd, P_["collar0"], P_["collar1"]))
        raw = np.where(zone, np.clip(raw, 0.0, 1.0), 0.0)
        far = s > 0.200                      # past the fillet: pure arm, collar off
        raw[far] = np.where(zone[far], 1.0, 0.0)

        # Dirichlet: torso side 0, tube side 1; free band in between
        fixed0 = (aX <= P_["sh0"] - 0.005) | (~zone)
        fixed1 = zone & (aX >= P_["sh1"] + 0.005) & (dd <= 0.012)
        free = side & (~fixed0) & (~fixed1) & (aX < 0.62)
        f = raw.copy()
        f[fixed1] = 1.0
        f[fixed0] = 0.0
        if free.any():
            for _ in range(int(P_["smooth_it"])):
                nb = np.bincount(ah, weights=f[at], minlength=N) / DEG
                f = np.where(free, nb, f)
        f = np.minimum(f, smoothstep(aX, P_["sh0"], P_["sh1"]))
        f = np.where(zone, np.clip(f, 0.0, 1.0), 0.0)
        f[aX < P_["arm_hardzero_x"]] = 0.0
        ARMW += f
        info["arm_" + S] = {
            "zone_verts": int(zone.sum()), "free_band_verts": int(free.sum()),
            "band_verts_0_1": int(((f > 0.002) & (f < 0.998)).sum()),
            "band_x_range": [round(float(aX[(f > 0.002) & (f < 0.998)].min()), 4),
                             round(float(aX[(f > 0.002) & (f < 0.998)].max()), 4)]
            if ((f > 0.002) & (f < 0.998)).any() else None,
            "max_arm_w_on_egg_x_lt_0.385": round(float(f[aX < 0.385].max()), 6),
            "max_arm_w_at_x_lt_0.40": round(float(f[aX < 0.400].max()), 6),
        }
        # distribute along the arm
        g_el = smoothstep(s, S_EL[S] - P_["elbow_w"], S_EL[S] + P_["elbow_w"])
        s_mid = 0.5 * (S_EL[S] + S_WR[S])
        g_tw = smoothstep(s, s_mid - P_["twist_w"], s_mid + P_["twist_w"])
        g_h = smoothstep(s, S_WR[S] - P_["wrist_lo"], S_WR[S] + P_["wrist_hi"])
        rest = f * g_el
        W[:, BI["UPPERARM_" + S]] += f * (1.0 - g_el)
        W[:, BI["HAND_" + S]] += rest * g_h
        fa = rest * (1.0 - g_h)
        W[:, BI["FOREARM_" + S]] += fa * (1.0 - g_tw)
        W[:, BI["FOREARM_TWIST_" + S]] += fa * g_tw

    # ---------------------------------------------------------------- legs
    LEGZ = np.zeros(N, bool)
    for S in ("L", "R"):
        A, u, L = LEGAX[S]
        V = P - A
        s = V @ u
        r = np.linalg.norm(V - s[:, None] * u, axis=1)
        s_eff = s + P_["leg_cone"] * np.clip(r - P_["leg_rtube"], 0.0, None)
        side = (X >= 0) if S == "L" else (X < 0)
        zone = side & (r < P_["leg_rz"]) & (s < P_["leg_sz"]) & (s > -0.14) & (Z < P_["leg_zcap"])
        LEGZ |= zone
        leg = 1.0 - smoothstep(s_eff, P_["leg_top0"], P_["leg_top1"])
        g3 = smoothstep(s, P_["ankle0"], P_["ankle1"])
        g2 = smoothstep(s, P_["knee0"], P_["knee1"])
        W[zone] = 0.0
        W[zone, BI["HIPS"]] = (1.0 - leg)[zone]
        W[zone, BI["FOOT_" + S]] = (leg * (1 - g3))[zone]
        W[zone, BI["SHIN_" + S]] = (leg * g3 * (1 - g2))[zone]
        W[zone, BI["THIGH_" + S]] = (leg * g3 * g2)[zone]
        # the shoe is 100 % FOOT_S -- applied globally by Z, not via the leg zone,
        # so an outlying toe vertex can never fall back to the torso rule.
        shoe = (Z <= P_["shoe_z"]) & (side if S == "L" else (X < 0))
        LEGZ |= shoe
        W[shoe] = 0.0
        W[shoe, BI["FOOT_" + S]] = 1.0
        tube = zone & (Z > P_["shoe_z"]) & (r < 0.11) & (s < 0.19)
        info["leg_" + S] = {
            "zone": int(zone.sum()), "shoe": int(shoe.sum()), "tube": int(tube.sum()),
            "tube_min_leg_share": round(float((W[tube, BI["THIGH_" + S]] +
                                               W[tube, BI["SHIN_" + S]] +
                                               W[tube, BI["FOOT_" + S]]).min()), 5) if tube.any() else None,
            "tube_max_hips": round(float(W[tube, BI["HIPS"]].max()), 5) if tube.any() else None}

    # ---------------------------------------------------------------- torso / head
    body = (~LEGZ)
    tm = body.copy()
    mass = np.clip(1.0 - ARMW, 0.0, 1.0)     # what the spine chain / head may take
    mass = np.where(tm, mass, 0.0)
    gA = smoothstep(Z, P_["tz_hs0"], P_["tz_hs1"])
    gB = smoothstep(Z, P_["tz_sc0"], P_["tz_sc1"])
    # head rule vs torso rule blend factor
    # The head rule may only reach the head/neck column.  The arm tube's top rises
    # to z ~ 1.35 right next to the neck: if it were caught by the z-term the HEAD
    # would drag the shoulder, so the z-term is faded out laterally and the whole
    # head rule is switched off inside the arm zone.
    hg = np.maximum(1.0 - smoothstep(rT, P_["head_rt0"], P_["head_rt1"]),
                    smoothstep(Z, P_["head_z0"], P_["head_z1"]) *
                    (1.0 - smoothstep(aX, 0.360, 0.420)))
    hg = np.where(ARMZ, 0.0, hg)
    gN = smoothstep(Z, P_["neck0"], P_["neck1"])
    gH = smoothstep(Z, P_["head0"], P_["head1"])
    # torso-only share of the spine chain
    w_hips = mass * (1 - gA)
    w_sp = mass * gA * (1 - gB)
    w_ch_t = mass * gA * gB
    # head-rule: the CHEST part of the torso rule is further split chest/neck/head
    w_ch = w_ch_t * (1.0 - hg * gN)
    w_nk = w_ch_t * hg * gN * (1 - gH)
    w_hd = w_ch_t * hg * gN * gH
    W[:, BI["HIPS"]] += w_hips
    W[:, BI["SPINE_01"]] += w_sp
    W[:, BI["CHEST"]] += w_ch
    W[:, BI["NECK"]] += w_nk
    W[:, BI["HEAD"]] += w_hd

    # hard sets
    belt = tm & (Z >= P_["belt0"]) & (Z <= P_["belt1"]) & (~ARMZ)
    W[belt] = 0.0
    W[belt, BI["HIPS"]] = 1.0
    headcore = tm & (Z >= P_["head_core_z"]) & (~ARMZ) & (aX < 0.420)
    W[headcore] = 0.0
    W[headcore, BI["HEAD"]] = 1.0
    info["belt_verts"] = int(belt.sum())
    info["headcore_verts"] = int(headcore.sum())
    info["armzone_verts"] = int(ARMZ.sum())
    info["legzone_verts"] = int(LEGZ.sum())

    # ---------------------------------------------------------------- normalise
    W[W < 0.004] = 0.0
    part = np.argpartition(-W, 4, axis=1)[:, 4:]
    info["max_dropped_weight"] = round(float(np.take_along_axis(W, part, axis=1).max()), 6)
    np.put_along_axis(W, part, 0.0, axis=1)
    ssum = W.sum(1)
    bad = ssum <= 1e-9
    info["zero_weight_verts"] = int(bad.sum())
    if bad.any():
        # fallback: nearest spine bone by Z
        W[bad, BI["CHEST"]] = 1.0
        ssum = W.sum(1)
    W = (W.T / ssum).T
    W = np.round(W, 5)
    dom = np.argmax(W, axis=1)
    W[np.arange(N), dom] = 0.0
    W[np.arange(N), dom] = 1.0 - W.sum(1)

    nz = W > 0
    info["influence_hist"] = [int((nz.sum(1) == k).sum()) for k in range(6)]
    info["max_sum_error"] = float(np.abs(W.sum(1) - 1.0).max())
    info["verts_per_group"] = {BONES[j]: int(nz[:, j].sum()) for j in range(NB)}

    # ---------------------------------------------------------------- purity / bleed
    ARMTOT = sum(W[:, BI[n]] for n in ("UPPERARM_L", "FOREARM_L", "FOREARM_TWIST_L", "HAND_L",
                                       "UPPERARM_R", "FOREARM_R", "FOREARM_TWIST_R", "HAND_R"))
    LEGTOT = sum(W[:, BI[n]] for n in ("THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"))
    HEADNECK = W[:, BI["HEAD"]] + W[:, BI["NECK"]]
    EGG = (aX < 0.385) & (Z > 0.45) & (Z < 1.30)
    HEADGEO = (Z > 1.36) & (rT < 0.45)
    hand_geo = {}
    for S in ("L", "R"):
        wx = abs(float(arm.bones["HAND_" + S].head_local[0]))
        hm = (aX > wx + 0.035) & (((X > 0) if S == "L" else (X < 0)))
        hand_geo["hand_" + S] = {"n": int(hm.sum()),
                                 "min_w_HAND": round(float(W[hm, BI["HAND_" + S]].min()), 5),
                                 "max_other": round(float((1 - W[hm, BI["HAND_" + S]]).max()), 5)}
    info["bleed"] = {
        "egg_max_arm_w": round(float(ARMTOT[EGG].max()), 6),
        "egg_n_arm_gt_0.02": int((ARMTOT[EGG] > 0.02).sum()),
        "head_max_arm_w": round(float(ARMTOT[HEADGEO].max()), 6),
        "arm_max_headneck_w": round(float(HEADNECK[ARMZ].max()), 6),
        "arm_max_leg_w": round(float(LEGTOT[ARMZ].max()), 6),
        "torso_above_0.50_max_leg_w": round(float(LEGTOT[(rT < 0.42) & (Z > 0.52)].max()), 6),
        "belt_min_HIPS": round(float(W[belt, BI["HIPS"]].min()), 6),
        "headcore_min_HEAD": round(float(W[headcore, BI["HEAD"]].min()), 6),
    }
    info["bleed"].update(hand_geo)
    shoeL = (Z <= P_["shoe_z"]) & (X >= 0)
    shoeR = (Z <= P_["shoe_z"]) & (X < 0)
    info["bleed"]["shoe_L_min_FOOT"] = round(float(W[shoeL, BI["FOOT_L"]].min()), 5)
    info["bleed"]["shoe_R_min_FOOT"] = round(float(W[shoeR, BI["FOOT_R"]].min()), 5)

    # ---------------------------------------------------------------- write groups
    for g in list(ob.vertex_groups):
        ob.vertex_groups.remove(g)
    VG = {n: ob.vertex_groups.new(name=n) for n in BONES}
    for j, n in enumerate(BONES):
        col = W[:, j]
        idx = np.nonzero(col)[0]
        if not len(idx):
            continue
        o2 = np.argsort(col[idx], kind="stable")
        idx = idx[o2]
        vals = col[idx]
        st = np.nonzero(np.diff(vals))[0] + 1
        for a_, b_ in zip(np.concatenate([[0], st]), np.concatenate([st, [len(idx)]])):
            VG[n].add([int(x) for x in idx[a_:b_]], float(vals[a_]), 'REPLACE')

    for m in list(ob.modifiers):
        ob.modifiers.remove(m)
    ob.parent = rig
    ob.matrix_parent_inverse = rig.matrix_world.inverted()
    md = ob.modifiers.new("Armature", 'ARMATURE')
    md.object = rig
    md.use_vertex_groups = True
    md.use_bone_envelopes = False
    md.use_deform_preserve_volume = (not LINEAR)
    info["modifier"] = {"type": md.type, "object": md.object.name,
                        "use_vertex_groups": md.use_vertex_groups,
                        "use_bone_envelopes": md.use_bone_envelopes,
                        "use_deform_preserve_volume": md.use_deform_preserve_volume}

    # ---------------------------------------------------------------- corrective smooth
    # A 2-bone fold of a smooth tube at 130 deg pinches the inner side whatever the
    # weights do (measured: 84 mm of inward collapse at the elbow).  A joint-limited
    # Corrective Smooth relaxes that; it is a no-op at rest.
    if P_["cs_factor"] > 0:
        cw = np.zeros(N)
        for S in ("L", "R"):
            for jb, r0k, r1k in (("FOREARM_" + S, "cs_elbow_r0", "cs_elbow_r1"),
                                 ("UPPERARM_" + S, "cs_sh_r0", "cs_sh_r1"),
                                 ("HAND_" + S, "cs_wrist_r0", "cs_wrist_r1")):
                c = np.array(arm.bones[jb].head_local)
                dc = np.linalg.norm(P - c, axis=1)
                cw = np.maximum(cw, 1.0 - smoothstep(dc, P_[r0k], P_[r1k]))
        cw[aX < 0.392] = 0.0          # never touch the egg or the head
        cw[Z > 1.36] = 0.0
        g = ob.vertex_groups.new(name="CS_JOINTS")
        idx = np.nonzero(cw > 0.004)[0]
        vv = np.round(cw[idx], 4)
        o2 = np.argsort(vv, kind="stable")
        idx, vv = idx[o2], vv[o2]
        st = np.nonzero(np.diff(vv))[0] + 1
        for a_, b_ in zip(np.concatenate([[0], st]), np.concatenate([st, [len(idx)]])):
            g.add([int(x) for x in idx[a_:b_]], float(vv[a_]), 'REPLACE')
        cs = ob.modifiers.new("CorrSmooth", 'CORRECTIVE_SMOOTH')
        cs.smooth_type = 'LENGTH_WEIGHTED'
        cs.rest_source = 'ORCO'
        cs.factor = P_["cs_factor"]
        cs.iterations = int(P_["cs_iter"])
        cs.vertex_group = "CS_JOINTS"
        cs.use_only_smooth = False
        cs.use_pin_boundary = False
        info["corrective_smooth"] = {
            "smooth_type": cs.smooth_type, "rest_source": cs.rest_source,
            "factor": cs.factor, "iterations": cs.iterations,
            "vertex_group": cs.vertex_group,
            "masked_verts": int((cw > 0.004).sum()),
            "max_w": round(float(cw.max()), 3)}
    return info


names = {"lo": ["GOB_body_lo"], "hi": ["GOB_body"], "both": ["GOB_body_lo", "GOB_body"]}[TARGET]
for nm in names:
    REP[nm] = build(bpy.data.objects[nm], nm)

# =====================================================================  rest check
hicol = bpy.data.collections["GOB_hi"]
hv, hr = hicol.hide_viewport, hicol.hide_render
ohv, ohr = hi.hide_viewport, hi.hide_render
hicol.hide_viewport = hicol.hide_render = False
hi.hide_viewport = hi.hide_render = False
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for nm in names:
    ob = bpy.data.objects[nm]
    P = readP(ob)
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3)
    dm.vertices.foreach_get("co", Q)
    Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    dev = np.linalg.norm(Q - P, axis=1)
    REP[nm]["rest_max_dev_m"] = float(dev.max())
    REP[nm]["rest_n_over_1e5"] = int((dev > 1e-5).sum())
hicol.hide_viewport, hicol.hide_render = hv, hr
hi.hide_viewport, hi.hide_render = ohv, ohr

REP["n_actions"] = len(bpy.data.actions)
REP["posed_bones"] = [b.name for b in rig.pose.bones
                      if (b.location.length > 1e-6 or
                          abs(b.rotation_quaternion.w - 1) > 1e-6 or
                          (b.rotation_mode != 'QUATERNION' and Vector(b.rotation_euler).length > 1e-6) or
                          (Vector(b.scale) - Vector((1, 1, 1))).length > 1e-6)]
REP["deform_bones"] = sorted([b.name for b in arm.bones if b.use_deform])
REP["n_bones"] = len(arm.bones)

print("@@@JSON_START@@@")
print(json.dumps(REP, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(INSP, "skin.json"), "w") as f:
    json.dump(REP, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
