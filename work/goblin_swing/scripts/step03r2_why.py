import bpy, json, os
import numpy as np
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
arm = bpy.data.objects["GOB_rig"].data
out = {}
for obn in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[obn]
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
    r = {}
    for S in ("R", "L"):
        sh = np.array(arm.bones["UPPERARM_" + S].head_local)
        tl = np.array(arm.bones["UPPERARM_" + S].tail_local)
        u = tl - sh; u /= np.linalg.norm(u)
        v = P - sh
        s = v @ u
        d = np.linalg.norm(v - s[:, None] * u, axis=1)
        aw = W[:, BI["UPPERARM_" + S]]
        leak = (aw > 0.02) & (d > 0.11)
        allowed = np.zeros(NB, dtype=bool)
        allowed[[BI["HIPS"], BI["SPINE_01"], BI["CHEST"], BI["UPPERARM_" + S]]] = True
        pure = (W[:, ~allowed].sum(1) < 1e-6)
        dsh = np.linalg.norm(P - sh, axis=1)
        co = {}
        for j in range(NB):
            c = int((leak & (W[:, j] > 1e-6)).sum())
            if c:
                co[BONES[j]] = c
        r[S] = {"n_leak(aw>.02,d>.11)": int(leak.sum()),
                "of_which_pure": int((leak & pure).sum()),
                "of_which_within_0.34": int((leak & (dsh < 0.34)).sum()),
                "of_which_pure_and_near": int((leak & pure & (dsh < 0.34)).sum()),
                "dsh_range": [round(float(dsh[leak].min()), 3), round(float(dsh[leak].max()), 3)] if leak.any() else None,
                "s_range": [round(float(s[leak].min()), 3), round(float(s[leak].max()), 3)] if leak.any() else None,
                "d_range": [round(float(d[leak].min()), 3), round(float(d[leak].max()), 3)] if leak.any() else None,
                "max_aw": round(float(aw[leak].max()), 3) if leak.any() else 0,
                "bones_present_on_leak_verts": co}
    out[obn] = r
print("@@@JSON_START@@@")
print(json.dumps(out, indent=1, default=str))
print("@@@JSON_END@@@")
