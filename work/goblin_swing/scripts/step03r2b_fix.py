"""STEP 03R2 round 2 - armpit web reworked as a smooth harmonic sheet.

Run: blender -b goblin_v05b_shoulderfix.blend --python step03r2b_fix.py
     -- [--out PATH] [--t0 0.40] [--t1 0.90] [--iters 600] [--redit 0.50]
Writes goblin_v05c_shoulderfix.blend.  Never touches v05 / v05b.

Why round 1 was not enough
--------------------------
Round 1 classified tube-vs-web by the distance `d` to the UPPERARM bone axis and
a per-slice p97 "tube envelope" clamped at 0.095.  The bone axis is NOT centred
in the tube: measured per slice the tube's own surface spans d = 0.035 .. 0.095,
so that envelope swallowed the armpit web and handed it 100% arm weight.  The
web therefore flew out with the arm as a wrinkled wing.

Round 2 rule - no tube/web classification at all
------------------------------------------------
Solve for the scalar field  A(v) = "how much this vertex follows the arm"
                                 = w_UPPERARM_S + w_FOREARM_S + w_HAND_S (+ WEAPON on R)

  * Dirichlet wall A = 1 : the arm core   (A_old > 0.99, within 0.080 of the arm
                           chain, and past s_e + 0.10 along the upper arm) plus
                           anything the hand / club / forearm already owns.
  * Dirichlet wall A = 0 : every vertex that currently has no arm weight at all
                           (the egg flank) - it stays exactly as it is.
  * FREE                 : the junction fillet, the armpit web and the tube root.

  t = harmonic (Laplace) extension over FREE.  A harmonic field obeys the maximum
  principle: it has no interior extrema, hence no wrinkles, and it stretches
  across the web like a rubber sheet regardless of how the web is shaped.

  A_new = min( A_old , smoothstep(t, T0, T1) )      <- REDUCE ONLY

  T0/T1 shape where the 50% line sits.  T0 = 0.40 pushes it well onto the tube,
  so the web reads as egg skin and the tube appears to come out of the egg.

The vertex's arm bones are scaled by A_new/A_old (their internal split is kept,
so the elbow/wrist blends are untouched); the freed mass goes to HIPS/SPINE_01/
CHEST by the STEP 03 torso Z-gradient.  HEAD and NECK are never written.
Because the rule only ever REMOVES arm weight, this also zeroes the leftover
FOREARM_L ~0.014 that was sitting on the right-hand torso flank.
"""
import bpy, json, math, os, sys
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d):
    return type(d)(argv[argv.index(n) + 1]) if n in argv else d


OUT = opt("--out", os.path.join(ROOT, "goblin_v05c_shoulderfix.blend"))
T0 = opt("--t0", 0.40)
T1 = opt("--t1", 0.90)
ITERS = opt("--iters", 600)
REDIT = opt("--redit", 0.50)
SMOOTH = opt("--smooth", 40)

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
TZ = {"hips_spine": (0.780, 1.000), "spine_chest": (0.960, 1.220)}   # == STEP 03
SE = {"R": 0.020, "L": -0.030}       # tube exit, measured in round 1

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
R = {"params": {"t0": T0, "t1": T1, "iters": ITERS, "redit": REDIT,
                "smooth": SMOOTH, "s_e": SE}}


def smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def bh(n):
    return np.array(arm.bones[n].head_local)


def bt(n):
    return np.array(arm.bones[n].tail_local)


def readW(ob):
    me = ob.data
    n = len(me.vertices)
    P = np.empty(n * 3); me.vertices.foreach_get("co", P); P = P.reshape(n, 3)
    gi = {g.index: g.name for g in ob.vertex_groups}
    W = np.zeros((n, NB))
    for v in me.vertices:
        for g in v.groups:
            nm = gi.get(g.group)
            if nm in BI:
                W[v.index, BI[nm]] = g.weight
    return P, W


def adjacency(me, n):
    E = np.empty(len(me.edges) * 2, dtype=np.int32)
    me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    ah = np.concatenate([E[:, 0], E[:, 1]])
    at = np.concatenate([E[:, 1], E[:, 0]])
    o = np.argsort(ah, kind="stable")
    ah, at = ah[o], at[o]
    start = np.searchsorted(ah, np.arange(n + 1))
    deg = np.diff(start).astype(np.float64)
    return ah, at, np.maximum(deg, 1.0)


def poly_d(P, pts):
    pts = np.asarray(pts, dtype=np.float64)
    A, B = pts[:-1], pts[1:]
    AB = B - A
    L2 = (AB ** 2).sum(1)
    bd = np.full(len(P), 1e9)
    for k in range(len(A)):
        w = P - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0.0, 1.0)
        bd = np.minimum(bd, np.linalg.norm(w - tt[:, None] * AB[k], axis=1))
    return bd


def apply_to(ob, tag):
    P, W = readW(ob)
    n = len(P)
    W0 = W.copy()
    Z = P[:, 2]
    rT = np.hypot(P[:, 0], P[:, 1] - 0.088)
    ah, at, deg = adjacency(ob.data, n)
    info = {"verts": n}

    # ---- hard sets (same geometric constants as STEP 03R) : never touched
    AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
    V = P - np.array([-0.6661, -0.3447, 0.9910])
    S_AX = V @ AXd
    D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
    CLUB = (S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > 0.1134) & (D_AX < 0.22)
    HANDR = (np.linalg.norm(P - np.array([-0.676, -0.364, 0.992]), axis=1) <= 0.155) & (~CLUB)
    HANDL = np.linalg.norm(P - np.array([0.772, 0.056, 0.720]), axis=1) <= 0.155
    HEADCORE = (Z >= 1.375) & (~CLUB)
    BELT = (Z >= 0.578) & (Z <= 0.782) & (rT < 0.45) & (~CLUB) & (~HANDR) & (~HANDL)
    FOOT = (Z <= 0.168) & (~CLUB) & (~HANDR) & (~HANDL)
    HARD = CLUB | HANDR | HANDL | HEADCORE | BELT | FOOT
    LEGW = W[:, [BI[x] for x in ("THIGH_L", "SHIN_L", "FOOT_L",
                                 "THIGH_R", "SHIN_R", "FOOT_R")]].sum(1)

    for S in ("R", "L"):
        O = "L" if S == "R" else "R"
        ARMB = [BI["UPPERARM_" + S], BI["FOREARM_" + S], BI["HAND_" + S]]
        if S == "R":
            ARMB.append(BI["WEAPON"])
        OTHERB = [BI["UPPERARM_" + O], BI["FOREARM_" + O], BI["HAND_" + O]]
        if O == "R":
            OTHERB.append(BI["WEAPON"])

        A0 = W[:, ARMB].sum(1)
        sh = bh("UPPERARM_" + S)
        u = bt("UPPERARM_" + S) - sh
        u /= np.linalg.norm(u)
        sc = (P - sh) @ u
        chain = poly_d(P, [sh, bh("FOREARM_" + S), bh("HAND_" + S), bt("HAND_" + S)])
        dsh = np.linalg.norm(P - sh, axis=1)

        eligible = ((dsh < REDIT) & (~HARD) & (LEGW < 0.02)
                    & (W[:, OTHERB].sum(1) < 0.02)
                    & (W[:, BI["FOREARM_" + S]] + W[:, ARMB[2]] < 0.5)
                    & (W[:, BI["WEAPON"]] < 0.5))
        CORE_ARM = ((A0 > 0.99) & (chain < 0.080) & (sc > SE[S] + 0.10)) \
            | (W[:, BI["FOREARM_" + S]] > 0.5) | (W[:, ARMB[2]] > 0.5) \
            | (W[:, BI["WEAPON"]] > 0.5)
        CORE_TORSO = A0 <= 1e-9
        FREE = eligible & (~CORE_ARM) & (~CORE_TORSO)

        # ---- harmonic extension (Jacobi); non-free verts hold their value
        t = np.clip(A0.copy(), 0.0, 1.0)
        t[CORE_ARM] = 1.0
        fixed = ~FREE
        fv = t[fixed].copy()
        for _ in range(ITERS):
            t = np.bincount(ah, weights=t[at], minlength=n) / deg
            t[fixed] = fv
        resid = float(np.abs(np.bincount(ah, weights=t[at], minlength=n) / deg
                             - t)[FREE].max()) if FREE.any() else 0.0

        Anew = np.minimum(A0, smoothstep(t, T0, T1))
        # a few constrained smoothing passes on the result itself
        for _ in range(SMOOTH):
            sm = np.bincount(ah, weights=Anew[at], minlength=n) / deg
            Anew[FREE] = np.minimum(A0[FREE], 0.5 * Anew[FREE] + 0.5 * sm[FREE])

        chg = FREE & (np.abs(Anew - A0) > 1e-5)
        idx = np.nonzero(chg)[0]
        if len(idx):
            a0 = A0[idx]
            an = Anew[idx]
            scale = np.where(a0 > 1e-9, an / np.maximum(a0, 1e-12), 0.0)
            Wn = W[idx].copy()
            Wn[:, ARMB] = (Wn[:, ARMB].T * scale).T
            freed = a0 - an
            zz = Z[idx]
            gA = smoothstep(zz, *TZ["hips_spine"])
            gB = smoothstep(zz, *TZ["spine_chest"])
            Wn[:, BI["HIPS"]] += freed * (1 - gA)
            Wn[:, BI["SPINE_01"]] += freed * gA * (1 - gB)
            Wn[:, BI["CHEST"]] += freed * gA * gB
            W[idx] = Wn
        info[S] = {
            "n_eligible": int(eligible.sum()), "n_core_arm": int(CORE_ARM.sum()),
            "n_free": int(FREE.sum()), "n_changed": int(len(idx)),
            "harmonic_residual": round(resid, 8),
            "max_A_removed": round(float((A0 - Anew)[chg].max()), 4) if len(idx) else 0.0,
            "mean_A_removed": round(float((A0 - Anew)[chg].mean()), 4) if len(idx) else 0.0,
            "changed_bbox": [[round(float(x), 3) for x in P[idx].min(0)],
                             [round(float(x), 3) for x in P[idx].max(0)]] if len(idx) else None,
            "changed_max_dist_from_shoulder": round(float(dsh[idx].max()), 4) if len(idx) else 0.0,
        }

    # ---- prune / normalise (identical to the original pipeline)
    W[W < 0.004] = 0.0
    part = np.argpartition(-W, 4, axis=1)[:, 4:]
    info["max_dropped_weight"] = float(np.take_along_axis(W, part, axis=1).max())
    np.put_along_axis(W, part, 0.0, axis=1)
    ssum = W.sum(1)
    bad = ssum <= 1e-9
    info["zero_weight_verts_before_fix"] = int(bad.sum())
    if bad.any():
        W[bad] = W0[bad]
        ssum = W.sum(1)
    W = (W.T / ssum).T
    W = np.round(W, 5)
    dom = np.argmax(W, axis=1)
    W[np.arange(n), dom] = 0.0
    W[np.arange(n), dom] = 1.0 - W.sum(1)

    dif = np.abs(W - W0).max(1)
    m = dif > 1e-6
    gd = np.abs(W - W0).max(0)
    info["diff_vs_v05b"] = {
        "n_verts_changed": int(m.sum()), "pct": round(100.0 * m.sum() / n, 3),
        "max_delta": round(float(dif.max()), 5),
        "bbox": [[round(float(x), 3) for x in P[m].min(0)],
                 [round(float(x), 3) for x in P[m].max(0)]] if m.any() else None,
        "groups_touched": {BONES[j]: round(float(gd[j]), 5) for j in range(NB) if gd[j] > 1e-6},
        "groups_identical": [BONES[j] for j in range(NB) if gd[j] <= 1e-6],
        "hardsets_changed": {k: int((m & s).sum()) for k, s in
                             (("club", CLUB), ("ring_hand", HANDR), ("handL_ball", HANDL),
                              ("head_core", HEADCORE), ("belt", BELT), ("shoes", FOOT))},
        "changed_outside_both_shoulder_spheres": int(
            (m & (np.linalg.norm(P - bh("UPPERARM_R"), axis=1) >= REDIT)
             & (np.linalg.norm(P - bh("UPPERARM_L"), axis=1) >= REDIT)).sum()),
    }
    info["influence_hist"] = [int(((W > 0).sum(1) == k).sum()) for k in range(6)]
    info["max_sum_error"] = float(np.abs(W.sum(1) - 1.0).max())

    for g in list(ob.vertex_groups):
        ob.vertex_groups.remove(g)
    VG = {nm: ob.vertex_groups.new(name=nm) for nm in BONES}
    for j, nm in enumerate(BONES):
        col = W[:, j]
        ii = np.nonzero(col)[0]
        if not len(ii):
            continue
        o2 = np.argsort(col[ii], kind="stable")
        ii = ii[o2]; vals = col[ii]
        starts = np.nonzero(np.diff(vals))[0] + 1
        for a_, b_ in zip(np.concatenate([[0], starts]), np.concatenate([starts, [len(ii)]])):
            VG[nm].add([int(x) for x in ii[a_:b_]], float(vals[a_]), 'REPLACE')
    R[tag] = info


hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
apply_to(lo, "GOB_body_lo")
apply_to(hi, "GOB_body")

# ---- rest check with both meshes visible, then restore visibility
bpy.data.collections["GOB_hi"].hide_viewport = False
bpy.data.collections["GOB_hi"].hide_render = False
hi.hide_viewport = False
hi.hide_render = False
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for ob, key in ((lo, "GOB_body_lo"), (hi, "GOB_body")):
    me = ob.data
    P = np.empty(len(me.vertices) * 3); me.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    R[key]["rest_max_dev_mm"] = round(float(np.linalg.norm(Q - P, axis=1).max()) * 1000, 6)
bpy.data.collections["GOB_hi"].hide_viewport = True
bpy.data.collections["GOB_hi"].hide_render = True
hi.hide_viewport = True
hi.hide_render = True
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name == "GOB_hi":
        lc.hide_viewport = True
        lc.exclude = False

R["n_actions"] = len(bpy.data.actions)
R["ref_cam"] = "REF_CAM" in bpy.data.objects
print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2b_fix.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
