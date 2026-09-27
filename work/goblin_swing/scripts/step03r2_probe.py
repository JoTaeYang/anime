"""STEP 03R2 probe - where does the arm tube actually exit the torso surface?

Run: blender -b goblin_v05_legfix.blend --python step03r2_probe.py
NEVER saves.
"""
import bpy, json, math, os
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
arm = bpy.data.objects["GOB_rig"].data
R = {}

for obname in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[obname]
    me = ob.data
    n = len(me.vertices)
    P = np.empty(n * 3); me.vertices.foreach_get("co", P); P = P.reshape(n, 3)
    r = {}
    for S in ("R", "L"):
        sh = np.array(arm.bones["UPPERARM_" + S].head_local)
        tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
        u = tl - sh; u /= np.linalg.norm(u)
        v = P - sh
        s = v @ u
        d = np.linalg.norm(v - s[:, None] * u, axis=1)
        rows = []
        for lo_ in np.arange(-0.10, 0.20, 0.01):
            m = (s >= lo_) & (s < lo_ + 0.01)
            near = m & (d < 0.105)
            mid = m & (d >= 0.105) & (d < 0.24)
            far = m & (d >= 0.24)
            rows.append({
                "s": round(float(lo_) + 0.005, 3),
                "n_near(d<0.105)": int(near.sum()),
                "near_med_d": round(float(np.median(d[near])), 4) if near.sum() else None,
                "near_max_d": round(float(d[near].max()), 4) if near.sum() else None,
                "n_mid(.105-.24)": int(mid.sum()),
                "n_far(>.24)": int(far.sum()),
            })
        r[S] = rows
        # torso "egg" envelope in this neighbourhood: cylindrical radius about torso axis
        rT = np.hypot(P[:, 0], P[:, 1] - 0.088)
        Z = P[:, 2]
        prof = []
        for z0 in np.arange(1.00, 1.34, 0.04):
            m = (Z >= z0) & (Z < z0 + 0.04) & (d > 0.14)
            if m.sum() > 5:
                prof.append([round(float(z0) + 0.02, 3), int(m.sum()),
                             round(float(np.percentile(rT[m], 95)), 4)])
        r["torso_rT_by_z_excl_arm" + S] = prof
    R[obname] = r

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2_probe.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
