"""STEP 03R2 round 2 - where is the true tube and where is the armpit web?

Run: blender -b goblin_v05b_shoulderfix.blend --python step03r2b_webprobe.py
NEVER saves.
"""
import bpy, json, os
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
arm = bpy.data.objects["GOB_rig"].data
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
OUT = {}

for obn in ("GOB_body", "GOB_body_lo"):
    ob = bpy.data.objects[obn]
    me = ob.data
    N = len(me.vertices)
    P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
    gi = {g.index: g.name for g in ob.vertex_groups}
    W = np.zeros((N, 18))
    for v in me.vertices:
        for g in v.groups:
            nm = gi.get(g.group)
            if nm in BI:
                W[v.index, BI[nm]] = g.weight
    r = {}
    for S in ("R", "L"):
        sh = np.array(arm.bones["UPPERARM_" + S].head_local)
        tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
        u = tl - sh; u /= np.linalg.norm(u)
        v = P - sh
        s = v @ u
        d = np.linalg.norm(v - s[:, None] * u, axis=1)
        aw = W[:, BI["UPPERARM_" + S]]
        rows = []
        for lo_ in np.arange(-0.02, 0.26, 0.02):
            m = (s >= lo_) & (s < lo_ + 0.02) & (d < 0.30)
            if m.sum() < 4:
                continue
            dm = d[m]
            # histogram of the distance-to-axis in this slice
            hist = [int(((dm >= a) & (dm < b)).sum())
                    for a, b in ((0, .05), (.05, .07), (.07, .085), (.085, .10),
                                 (.10, .12), (.12, .15), (.15, .20), (.20, .30))]
            rows.append({"s": round(float(lo_) + 0.01, 3), "n": int(m.sum()),
                         "d_p25": round(float(np.percentile(dm, 25)), 4),
                         "d_p50": round(float(np.percentile(dm, 50)), 4),
                         "d_p75": round(float(np.percentile(dm, 75)), 4),
                         "d_p97": round(float(np.percentile(dm, 97)), 4),
                         "hist_.05/.07/.085/.10/.12/.15/.20/.30": hist,
                         "armw_at_d_.07_.11": round(float(
                             aw[m & (d >= 0.07) & (d < 0.11)].mean()), 3)
                             if (m & (d >= 0.07) & (d < 0.11)).any() else None,
                         "n_d_.07_.11": int((m & (d >= 0.07) & (d < 0.11)).sum())})
        r[S] = rows
    OUT[obn] = r
print("@@@JSON_START@@@")
print(json.dumps(OUT, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2b_webprobe.json"), "w") as f:
    json.dump(OUT, f, indent=1, default=str)
