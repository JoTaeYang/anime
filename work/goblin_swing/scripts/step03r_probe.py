"""STEP 03R probe - dump leg rig geometry from v04."""
import bpy, json, os
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
me = body.data
N = len(me.vertices)
P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)

R = {}
R["objects"] = [(o.name, o.type) for o in bpy.data.objects]
R["collections"] = [c.name for c in bpy.data.collections]
R["n_verts"] = N
R["n_actions"] = len(bpy.data.actions)

bones = {}
for b in arm.bones:
    bones[b.name] = {
        "head": [round(x, 6) for x in b.head_local],
        "tail": [round(x, 6) for x in (b.matrix_local @ __import__("mathutils").Vector((0, b.length, 0)))],
        "len": round(b.length, 6),
        "parent": b.parent.name if b.parent else None,
        "connect": b.use_connect,
        "deform": b.use_deform,
        "inherit_scale": b.inherit_scale,
        "inherit_rot": b.use_inherit_rotation,
        "roll_matrix": [[round(v, 5) for v in row] for row in b.matrix_local.to_3x3()],
    }
R["bones"] = bones
R["bone_collections"] = {c.name: [b.name for b in c.bones] for c in arm.collections}

cons = {}
for pb in rig.pose.bones:
    if pb.constraints:
        cons[pb.name] = [{"type": c.type, "name": c.name,
                          "sub": getattr(c, "subtarget", None),
                          "infl": round(c.influence, 4),
                          "pole": getattr(c, "pole_subtarget", None),
                          "chain": getattr(c, "chain_count", None)}
                         for c in pb.constraints]
R["constraints"] = cons
R["custom_props"] = {pb.name: {k: pb[k] for k in pb.keys() if k not in ("_RNA_UI",)}
                     for pb in rig.pose.bones if [k for k in pb.keys() if k != "_RNA_UI"]}
R["drivers"] = [d.data_path + ("[%d]" % d.array_index if d.array_index >= 0 else "")
                for d in (rig.animation_data.drivers if rig.animation_data else [])]

# leg mesh geometry
lab = np.load(os.path.join(ROOT, "inspect", "step03_labels.npy"))
BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
Z = P[:, 2]
for S in ("L", "R"):
    m = (lab == BI["THIGH_" + S]) | (lab == BI["SHIN_" + S])
    pts = P[m]
    R["legtube_" + S] = {"n": int(m.sum()),
                         "z": [round(float(pts[:, 2].min()), 4), round(float(pts[:, 2].max()), 4)],
                         "x": [round(float(pts[:, 0].min()), 4), round(float(pts[:, 0].max()), 4)],
                         "y": [round(float(pts[:, 1].min()), 4), round(float(pts[:, 1].max()), 4)]}
    f = (lab == BI["FOOT_" + S])
    R["foot_" + S] = {"n": int(f.sum()),
                      "z": [round(float(P[f][:, 2].min()), 4), round(float(P[f][:, 2].max()), 4)]}

# torso underside: HIPS verts with z < 0.55
hm = (lab == BI["HIPS"])
R["hips_z"] = [round(float(Z[hm].min()), 4), round(float(Z[hm].max()), 4)]
for zc in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55):
    R["hips_below_%.2f" % zc] = int((hm & (Z < zc)).sum())

# cross-sections of the leg tube in z
for S in ("L", "R"):
    m = (lab == BI["THIGH_" + S]) | (lab == BI["SHIN_" + S])
    rows = []
    for z0 in np.arange(0.16, 0.42, 0.02):
        sel = m & (Z >= z0) & (Z < z0 + 0.02)
        if sel.sum() >= 3:
            pts = P[sel]
            c = pts[:, :2].mean(0)
            rr = np.linalg.norm(pts[:, :2] - c, axis=1)
            rows.append([round(float(z0), 3), int(sel.sum()),
                         round(float(c[0]), 4), round(float(c[1]), 4),
                         round(float(rr.mean()), 4), round(float(rr.max()), 4)])
    R["xsec_" + S] = rows

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
