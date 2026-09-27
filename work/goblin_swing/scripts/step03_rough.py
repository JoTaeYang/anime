"""Is the weight field rough, or is the crumple pure geometry?  (read-only)
Run: blender -b goblin_v04_skinned.blend --python step03_rough.py
"""
import bpy, json, os, sys, math
import numpy as np
from mathutils import Vector

sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
body = bpy.data.objects["GOB_body"]
rig = bpy.data.objects["GOB_rig"]
me = body.data
N = len(me.vertices)
P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
E = np.empty(len(me.edges) * 2, dtype=np.int32); me.edges.foreach_get("vertices", E)
E = E.reshape(-1, 2)
ah = np.concatenate([E[:, 0], E[:, 1]]); at = np.concatenate([E[:, 1], E[:, 0]])
o = np.argsort(ah, kind="stable"); ah, at = ah[o], at[o]
st = np.searchsorted(ah, np.arange(N + 1)); DEG = np.diff(st).astype(float)

GN = [g.name for g in body.vertex_groups]
gi = {g.index: k for k, g in enumerate(body.vertex_groups)}
W = np.zeros((N, len(GN)), dtype=np.float32)
for v in me.vertices:
    for g in v.groups:
        W[v.index, gi[g.group]] = g.weight

S = np.empty_like(W)
for j in range(len(GN)):
    S[:, j] = np.bincount(ah, weights=W[at, j], minlength=N)
S = (S.T / DEG).T
rough = np.abs(W - S).max(1)

Z = P[:, 2]
R = {"rough_max": float(rough.max()), "rough_mean": float(rough.mean()),
     "rough_p99": float(np.percentile(rough, 99))}
zones = {"leg_tube": (Z > 0.17) & (Z < 0.36) & (np.abs(P[:, 0]) > 0.15),
         "torso_mid": (Z > 0.78) & (Z < 1.05) & (np.abs(P[:, 0]) < 0.45),
         "belt": (Z > 0.58) & (Z < 0.78),
         "head": Z > 1.4,
         "leg_root": (Z > 0.30) & (Z < 0.46) & (np.abs(P[:, 0]) > 0.15)}
R["rough_by_zone"] = {k: [round(float(rough[m].max()), 4),
                          round(float(rough[m].mean()), 5), int(m.sum())]
                      for k, m in zones.items()}

# edge-length / triangle-quality statistics in the crumpling zones
EL = np.linalg.norm(P[ah] - P[at], axis=1)
for k, m in zones.items():
    sel = m[ah]
    if sel.any():
        R.setdefault("edge_len_by_zone", {})[k] = [
            round(float(EL[sel].min()), 5), round(float(np.median(EL[sel])), 5),
            round(float(EL[sel].max()), 5)]

# how many weight "rings" does the leg tube actually have?
for S_ in ("L", "R"):
    sg = 1.0 if S_ == "L" else -1.0
    m = (Z > 0.17) & (Z < 0.36) & (P[:, 0] * sg > 0.15)
    zs = np.sort(Z[m])
    R.setdefault("leg_rings", {})[S_] = {
        "n": int(m.sum()),
        "z_unique_1mm": int(len(np.unique(np.round(Z[m], 3)))),
        "z_unique_5mm": int(len(np.unique(np.round(Z[m] / 0.005).astype(int))))}

# colour the mesh by roughness and render the two problem zones
me.materials.clear()
NB_ = 6
for k in range(NB_):
    f = k / (NB_ - 1.0)
    mt = bpy.data.materials.get("RG_%d" % k) or bpy.data.materials.new("RG_%d" % k)
    mt.use_nodes = False
    mt.diffuse_color = (0.15 + 0.85 * f, 0.75 * (1 - f), 0.2, 1.0)
    me.materials.append(mt)
bins = np.clip((rough / 0.25 * (NB_ - 1)).astype(int), 0, NB_ - 1)
for p in me.polygons:
    p.material_index = int(max(bins[i] for i in p.vertices))
me.update()

OUT = os.path.join(ROOT, "inspect", "step03", "rough")
cam, cd = RL.setup(res=1000, plain=False)
RL.shoot(cam, cd, OUT, "rough_front", (0, 0, 0.95), 0, 0, 2.1)
RL.shoot(cam, cd, OUT, "rough_legs", (0, 0.05, 0.30), 0, 5, 0.9)
RL.shoot(cam, cd, OUT, "rough_torso", (0, 0.0, 0.92), 0, 0, 1.2)

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
