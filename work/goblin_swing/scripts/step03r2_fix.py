"""STEP 03R2 - shoulder weight rework (WEIGHTS ONLY).

Run: blender -b goblin_v05_legfix.blend --python step03r2_fix.py -- [--out PATH]
     [--ramp 0.060] [--collar 0.025] [--redit 0.34]

Rule (identical for both meshes, both shoulders):
  s  = signed distance along the UPPERARM axis measured from the shoulder head
  d  = perpendicular distance to that axis
  s_e= where the arm tube leaves the torso surface (detected from the high-poly:
       the s-slice where the junction fillet ring, verts with d in [0.105,0.24],
       last spikes above the baseline)
  rtube(s) = p97 of d over the verts with d<0.13 in that slice (high-poly),
             clamped to <=0.095 and monotone-smoothed -> the tube envelope
  dd = d - rtube(s)                  (<=0 : ON the tube,  >0 : out on the torso)

  w_arm = min( old_arm , Ramp(s) * Collar(dd) )     <- REDUCE ONLY, never add
      Ramp   = smoothstep(s, s_e, s_e + RAMP)        -> band lives on the TUBE
      Collar = 1 - smoothstep(dd, 0.008, 0.008+COLLAR)  -> small collar only,
               hard 0 beyond, so the torso flank can never follow the arm.

Touched only if  ||P - shoulder_head|| < REDIT  and the vertex carries no weight
from any OTHER limb bone (FOREARM/HAND/WEAPON/legs/other arm) above 0.02 - so the
elbow blend, the hand/club hard sets and the legs cannot be affected.  Because the
rule only ever REMOVES upper-arm weight, a vertex that is genuinely on the tube
(dd<=0.008, s past the ramp) keeps exactly what it had.  The freed mass is given
back to the bones that vertex already had (proportionally), or, if it was a
pure-arm vert, by the STEP 03 torso Z-gradient.  Everything else is bit-identical.
"""
import bpy, json, math, os, sys
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d):
    return type(d)(argv[argv.index(n) + 1]) if n in argv else d


OUT = opt("--out", os.path.join(ROOT, "goblin_v05b_shoulderfix.blend"))
RAMP = opt("--ramp", 0.060)
COLLAR = opt("--collar", 0.025)
REDIT = opt("--redit", 0.50)
SE_OFF = opt("--seoff", 0.0)

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
SPINE3 = [BI["HIPS"], BI["SPINE_01"], BI["CHEST"]]
TZ = {"hips_spine": (0.780, 1.000), "spine_chest": (0.960, 1.220)}   # == STEP 03

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
R = {"params": {"ramp": RAMP, "collar": COLLAR, "redit": REDIT, "se_off": SE_OFF}}


def smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def axis(S):
    sh = np.array(arm.bones["UPPERARM_" + S].head_local)
    tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
    u = tl - sh
    return sh, u / np.linalg.norm(u)


def sd(P, S):
    sh, u = axis(S)
    v = P - sh
    s = v @ u
    return s, np.linalg.norm(v - s[:, None] * u, axis=1)


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


# ================================================ calibrate on the high-poly
hi = bpy.data.objects["GOB_body"]
PH, WH = readW(hi)
CAL = {}
for S in ("R", "L"):
    s, d = sd(PH, S)
    mids, raw = [], []
    for lo_ in np.arange(-0.12, 0.12, 0.01):
        m = (s >= lo_) & (s < lo_ + 0.01)
        mids.append([round(float(lo_) + 0.005, 3),
                     int((m & (d >= 0.105) & (d < 0.24)).sum())])
    cnt = np.array([c for _, c in mids], dtype=float)
    base = float(np.median(cnt[cnt > 0])) if (cnt > 0).any() else 1.0
    hot = [mids[i][0] for i in range(len(mids)) if cnt[i] > max(3.0 * base, 40)]
    s_e = (max(hot) + 0.005) if hot else 0.01
    s_e += SE_OFF
    # tube envelope radius per slice
    rt_s, rt_r = [], []
    for lo_ in np.arange(s_e, 0.30, 0.01):
        m = (s >= lo_) & (s < lo_ + 0.01) & (d < 0.13)
        if m.sum() >= 12:
            rt_s.append(float(lo_) + 0.005)
            rt_r.append(min(0.095, float(np.percentile(d[m], 97))))
    rt_s = np.array(rt_s); rt_r = np.array(rt_r)
    # light smoothing, then an envelope that never dips below the local value
    k = np.ones(3) / 3.0
    rt_sm = np.convolve(np.pad(rt_r, 1, mode='edge'), k, mode='valid')
    rt_r = np.maximum(rt_r, rt_sm)
    CAL[S] = {"s_e": round(float(s_e), 4), "fillet_hist": mids,
              "baseline": base, "hot_slices": hot,
              "rtube_s": [round(float(x), 3) for x in rt_s],
              "rtube_r": [round(float(x), 4) for x in rt_r]}
R["calibration"] = CAL


def rtube_of(S, s):
    c = CAL[S]
    xs = np.array(c["rtube_s"]); ys = np.array(c["rtube_r"])
    return np.interp(s, xs, ys, left=ys[0], right=ys[-1])


# ================================================ apply
def apply_to(ob, tag):
    P, W = readW(ob)
    n = len(P)
    W0 = W.copy()
    Z = P[:, 2]
    info = {"verts": n}
    for S in ("R", "L"):
        sh, u = axis(S)
        s, d = sd(P, S)
        ai = BI["UPPERARM_" + S]
        # any weight from another limb bone disqualifies the vertex: the elbow
        # blend, the hand / club hard sets and the legs must not be perturbed.
        OTHER = [BI[n] for n in ("FOREARM_L", "FOREARM_R", "HAND_L", "HAND_R",
                                 "WEAPON", "THIGH_L", "SHIN_L", "FOOT_L",
                                 "THIGH_R", "SHIN_R", "FOOT_R",
                                 "UPPERARM_" + ("L" if S == "R" else "R"))]
        clean = W[:, OTHER].sum(1) < 0.02
        near = np.linalg.norm(P - sh, axis=1) < REDIT
        edit = clean & near
        s_e = CAL[S]["s_e"]
        dd = d - rtube_of(S, s)
        tgt = smoothstep(s, s_e, s_e + RAMP) * (1.0 - smoothstep(dd, 0.008, 0.008 + COLLAR))
        tgt = np.clip(tgt, 0.0, 1.0)
        # REDUCE ONLY
        tgt = np.minimum(tgt, W[:, ai])

        old = W[:, ai].copy()
        chg = edit & (np.abs(tgt - old) > 1e-5)
        idx = np.nonzero(chg)[0]
        newa = tgt[idx]
        freed = old[idx] - newa                      # >= 0 (reduce-only)
        Wn = W[idx].copy()
        Wn[:, ai] = newa
        # The freed mass goes to the SPINE CHAIN ONLY, by the STEP 03 torso
        # Z-gradient.  HEAD / NECK keep their exact original weight, so head /
        # arm / torso independence is unchanged by this edit.
        zz = Z[idx]
        gA = smoothstep(zz, *TZ["hips_spine"])
        gB = smoothstep(zz, *TZ["spine_chest"])
        Wn[:, BI["HIPS"]] += freed * (1 - gA)
        Wn[:, BI["SPINE_01"]] += freed * gA * (1 - gB)
        Wn[:, BI["CHEST"]] += freed * gA * gB
        W[idx] = Wn
        info[S] = {
            "s_e": s_e,
            "n_clean_candidates": int(clean.sum()),
            "n_in_edit_radius": int(edit.sum()),
            "n_changed": int(len(idx)),
            "max_weight_delta": round(float(np.abs(W[idx, ai] - old[idx]).max()), 4) if len(idx) else 0.0,
            "freed_mass_max": round(float(freed.max()), 4) if len(idx) else 0.0,
            "arm_w_removed_from_flank_d_gt_0.12": int(((old > 0.02) & (d > 0.12) & chg).sum()),
        }
        if len(idx):
            info[S]["changed_bbox"] = [[round(float(x), 3) for x in P[idx].min(0)],
                                       [round(float(x), 3) for x in P[idx].max(0)]]
            info[S]["changed_max_dist_from_shoulder"] = round(
                float(np.linalg.norm(P[idx] - sh, axis=1).max()), 4)

    # ---- prune / normalise exactly as the original pipeline did
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

    # ---- diff vs original
    dif = np.abs(W - W0).max(1)
    m = dif > 1e-6
    info["diff"] = {
        "n_verts_changed": int(m.sum()),
        "pct": round(100.0 * m.sum() / n, 3),
        "max_delta": round(float(dif.max()), 5),
        "bbox": [[round(float(x), 3) for x in P[m].min(0)],
                 [round(float(x), 3) for x in P[m].max(0)]] if m.any() else None,
    }
    for S in ("R", "L"):
        sh, _ = axis(S)
        dsh = np.linalg.norm(P - sh, axis=1)
        info["diff"]["changed_within_REDIT_of_shoulder_" + S] = int((m & (dsh < REDIT)).sum())
        if m.any():
            info["diff"]["max_dist_from_shoulder_" + S] = round(float(dsh[m].min()), 4)
    info["diff"]["changed_outside_both_shoulder_spheres"] = int(
        (m & (np.linalg.norm(P - axis("R")[0], axis=1) >= REDIT)
         & (np.linalg.norm(P - axis("L")[0], axis=1) >= REDIT)).sum())
    # groups that changed at all
    gd = np.abs(W - W0).max(0)
    info["diff"]["groups_touched"] = {BONES[j]: round(float(gd[j]), 5)
                                      for j in range(NB) if gd[j] > 1e-6}
    info["influence_hist"] = [int(((W > 0).sum(1) == k).sum()) for k in range(6)]
    info["max_sum_error"] = float(np.abs(W.sum(1) - 1.0).max())

    # ---- write back
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
        for a_, b_ in zip(np.concatenate([[0], starts]),
                          np.concatenate([starts, [len(ii)]])):
            VG[nm].add([int(x) for x in ii[a_:b_]], float(vals[a_]), 'REPLACE')
    R[tag] = info
    return W


apply_to(bpy.data.objects["GOB_body_lo"], "GOB_body_lo")
apply_to(hi, "GOB_body")

# ================================================ rest / state checks
lo = bpy.data.objects["GOB_body_lo"]
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
    R[key]["rest_max_dev_m"] = float(np.linalg.norm(Q - P, axis=1).max())
# restore visibility state
bpy.data.collections["GOB_hi"].hide_viewport = True
bpy.data.collections["GOB_hi"].hide_render = True
hi.hide_viewport = True
hi.hide_render = True
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name == "GOB_hi":
        lc.hide_viewport = True
        lc.exclude = False

R["n_actions"] = len(bpy.data.actions)
R["has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["ref_cam"] = "REF_CAM" in bpy.data.objects
R["bone_count"] = len(arm.bones)
R["deform_bones"] = sorted([b.name for b in arm.bones if b.use_deform])

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2_fix.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
