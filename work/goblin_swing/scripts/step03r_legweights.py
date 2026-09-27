"""STEP 03R - analytic RIGID-LEG weighting, applied identically to GOB_body_lo and GOB_body.

Run: blender -b goblin_v05_legfix.blend --python step03r_legweights.py -- \
        [--target lo|hi|both] [--p key=value ...] [--no-save]

The leg is now one rigid body (THIGH_S and SHIN_S keep a fixed relative orientation and
share one axial scale about the ANKLE), so the only things that matter are:
  * WHERE the leg stops and the torso starts (the top band), and
  * that the shoe stays pure FOOT and the tube base blends into it over a short ankle band.
The THIGH/SHIN split is cosmetic while d <= L; under stretch any monotone split along the
leg axis gives the same, even, axial stretch, so it is placed at the rest knee.

Coordinates, per side S (analytic, mesh-independent):
    A = FOOT_S rest head (ankle),  H = THIGH_S rest head (hip socket),  u = (H-A)/|H-A|
    s = (P - A) . u          axial, 0 at the ankle, L at the hip socket
    r = |(P - A) - s u|      radial distance from the leg axis
    s_eff = s + cone * max(r - r_tube, 0)      conic feather into the torso underside
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
TARGET = argv[argv.index("--target") + 1] if "--target" in argv else "lo"
OUT = argv[argv.index("--out") + 1] if "--out" in argv else \
    os.path.join(ROOT, "goblin_v05_legfix.blend")
NOSAVE = "--no-save" in argv

P_ = {
    "ankle0": 0.008, "ankle1": 0.034,      # FOOT -> SHIN ramp (s)
    "knee0": 0.114, "knee1": 0.134,        # SHIN -> THIGH ramp (s)
    # THIGH -> HIPS ramp (s_eff).  s = 0.196 is where the tube geometry merges into
    # the egg's underside, so the band sits entirely ON/ABOVE that merge ring: every
    # vertex of the visible tube stays 100 % rigid leg, and the faces that have to
    # absorb the sinking are the torso-underside ring, which ends up inside the egg.
    "top0": 0.196, "top1": 0.216,
    "cone": 1.6, "r_tube": 0.125,
    "rz": 0.30, "sz": 0.32, "zcap": 0.52,
    "foot_z": 0.168,
}
for i, a in enumerate(argv):
    if a == "--p":
        k, v = argv[i + 1].split("=")
        P_[k] = float(v)

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
LEGB = [BI[n] for n in ("HIPS", "THIGH_L", "SHIN_L", "FOOT_L",
                        "THIGH_R", "SHIN_R", "FOOT_R")]

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
R = {"params": dict(P_), "target": TARGET}


def smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def do(obname):
    ob = bpy.data.objects[obname]
    me = ob.data
    N = len(me.vertices)
    P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
    gi = {g.index: g.name for g in ob.vertex_groups}
    W = np.zeros((N, NB))
    for v in me.vertices:
        for g in v.groups:
            n = gi.get(g.group)
            if n in BI:
                W[v.index, BI[n]] = g.weight
    W0 = W.copy()
    Z = P[:, 2]
    rep = {"n": N}

    for S in ("L", "R"):
        A = np.array(arm.bones["FOOT_" + S].head_local)
        H = np.array(arm.bones["THIGH_" + S].head_local)
        u = (H - A) / np.linalg.norm(H - A)
        L = float(np.linalg.norm(H - A))
        V = P - A
        s = V @ u
        r = np.linalg.norm(V - s[:, None] * u, axis=1)
        # + (not -): a vertex further out from the leg axis than the tube radius is
        # torso, so it must reach the "this is HIPS" end of the top ramp SOONER.
        s_eff = s + P_["cone"] * np.clip(r - P_["r_tube"], 0.0, None)

        side = (P[:, 0] >= 0) if S == "L" else (P[:, 0] < 0)
        zone = side & (r < P_["rz"]) & (s < P_["sz"]) & (s > -0.12) & (Z < P_["zcap"])

        leg = 1.0 - smoothstep(s_eff[zone], P_["top0"], P_["top1"])
        g3 = smoothstep(s[zone], P_["ankle0"], P_["ankle1"])     # 0 foot, 1 tube
        g2 = smoothstep(s[zone], P_["knee0"], P_["knee1"])       # 0 shin, 1 thigh

        W[zone] = 0.0
        W[zone, BI["HIPS"]] = 1.0 - leg
        W[zone, BI["FOOT_" + S]] = leg * (1.0 - g3)
        W[zone, BI["SHIN_" + S]] = leg * g3 * (1.0 - g2)
        W[zone, BI["THIGH_" + S]] = leg * g3 * g2

        # hard: the shoe is 100% FOOT_S
        shoe = zone & (Z <= P_["foot_z"])
        W[shoe] = 0.0
        W[shoe, BI["FOOT_" + S]] = 1.0

        rep["zone_" + S] = int(zone.sum())
        rep["shoe_" + S] = int(shoe.sum())
        tube = zone & (Z > P_["foot_z"]) & (r < 0.13) & (s < 0.19)
        rep["tube_" + S] = {
            "n": int(tube.sum()),
            "min_leg_share": round(float((W[tube, BI["THIGH_" + S]] +
                                          W[tube, BI["SHIN_" + S]] +
                                          W[tube, BI["FOOT_" + S]]).min()), 5),
            "max_hips": round(float(W[tube, BI["HIPS"]].max()), 5)}

    # normalise (the zone rows already sum to 1 by construction; be safe)
    W[W < 0.004] = 0.0
    part = np.argpartition(-W, 4, axis=1)[:, 4:]
    np.put_along_axis(W, part, 0.0, axis=1)
    ssum = W.sum(1)
    assert (ssum > 1e-9).all()
    W = (W.T / ssum).T
    W = np.round(W, 5)
    dom = np.argmax(W, axis=1)
    W[np.arange(N), dom] = 0.0
    W[np.arange(N), dom] = 1.0 - W.sum(1)

    # ---- diff vs the pre-existing weights, split leg-bones / everything else
    other = [j for j in range(NB) if j not in LEGB]
    dif = np.abs(W - W0)
    rep["max_diff_non_leg_bones"] = float(dif[:, other].max())
    rep["n_verts_changed"] = int((dif.max(1) > 1e-6).sum())
    above = Z > 0.55
    rep["max_diff_above_z0.55"] = float(dif[above].max())
    rep["influence_hist"] = [int(((W > 0).sum(1) == k).sum()) for k in range(6)]
    rep["max_sum_error"] = float(np.abs(W.sum(1) - 1.0).max())

    for g in list(ob.vertex_groups):
        ob.vertex_groups.remove(g)
    VG = {n: ob.vertex_groups.new(name=n) for n in BONES}
    for j, n in enumerate(BONES):
        col = W[:, j]
        idx = np.nonzero(col)[0]
        if not len(idx):
            continue
        o2 = np.argsort(col[idx], kind="stable")
        idx = idx[o2]; vals = col[idx]
        st = np.nonzero(np.diff(vals))[0] + 1
        for a_, b_ in zip(np.concatenate([[0], st]), np.concatenate([st, [len(idx)]])):
            VG[n].add([int(x) for x in idx[a_:b_]], float(vals[a_]), 'REPLACE')
    return rep


targets = {"lo": ["GOB_body_lo"], "hi": ["GOB_body"],
           "both": ["GOB_body_lo", "GOB_body"]}[TARGET]
for t in targets:
    R[t] = do(t)

bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for t in targets:
    ob = bpy.data.objects[t]
    P = np.empty(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    R[t]["rest_max_dev_m"] = float(np.linalg.norm(Q - P, axis=1).max())

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r_legweights.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
if not NOSAVE:
    bpy.ops.wm.save_as_mainfile(filepath=OUT)
    print("SAVED", OUT)
