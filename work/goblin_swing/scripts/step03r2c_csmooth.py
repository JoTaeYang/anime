"""STEP 03R2 round 2 - add a Corrective Smooth modifier limited to the armpits.

Run: blender -b goblin_v05c_shoulderfix.blend --python step03r2c_csmooth.py
     -- [--out PATH] [--factor 1.0] [--iters 20] [--r0 0.10] [--r1 0.26]

Adds, on BOTH meshes, AFTER the Armature modifier:
  * a vertex group `CS_armpits`: 1.0 within R0 of either armpit exit point,
    smoothstep down to 0 at R1, and hard 0 on every STEP 03 hard set
    (club, ring hand, left ball, head core, belt, shoes) and above z = 1.34.
  * a CORRECTIVE_SMOOTH modifier, rest_source = ORCO, restricted to that group.

rest_source = ORCO means the correction is the difference between the smoothed
DEFORMED mesh and the smoothed REST mesh, so at rest the correction is exactly
zero and the sculpt's shape is preserved; it only removes the creasing that the
skinning introduces.  No geometry and no bones are touched.
"""
import bpy, json, math, os, sys
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d):
    return type(d)(argv[argv.index(n) + 1]) if n in argv else d


OUT = opt("--out", os.path.join(ROOT, "goblin_v05c_shoulderfix.blend"))
FACTOR = opt("--factor", 1.0)
ITERS = opt("--iters", 20)
R0 = opt("--r0", 0.10)
R1 = opt("--r1", 0.26)
SCALE = opt("--scale", 1.0)
STYPE = opt("--stype", "LENGTH_WEIGHTED")

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
R = {"params": {"factor": FACTOR, "iters": ITERS, "r0": R0, "r1": R1,
                "scale": SCALE, "smooth_type": STYPE}}
SE = {"R": 0.020, "L": -0.030}


def smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


EXIT = {}
for S in ("R", "L"):
    h = np.array(arm.bones["UPPERARM_" + S].head_local)
    t = np.array(arm.bones["UPPERARM_" + S].tail_local)
    u = (t - h) / np.linalg.norm(t - h)
    EXIT[S] = h + u * SE[S]
R["exit_points"] = {S: [round(float(x), 4) for x in EXIT[S]] for S in ("R", "L")}

for name in ("GOB_body_lo", "GOB_body"):
    ob = bpy.data.objects[name]
    me = ob.data
    N = len(me.vertices)
    P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
    Z = P[:, 2]
    rT = np.hypot(P[:, 0], P[:, 1] - 0.088)

    # ---- hard sets (same constants as STEP 03R) - excluded from the group
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

    w = np.zeros(N)
    for S in ("R", "L"):
        dd = np.linalg.norm(P - EXIT[S], axis=1)
        w = np.maximum(w, 1.0 - smoothstep(dd, R0, R1))
    w[HARD] = 0.0
    w[Z > 1.34] = 0.0
    w[Z < 0.80] = 0.0
    w = np.round(w, 5)

    g = ob.vertex_groups.get("CS_armpits")
    if g:
        ob.vertex_groups.remove(g)
    g = ob.vertex_groups.new(name="CS_armpits")
    idx = np.nonzero(w > 0)[0]
    o = np.argsort(w[idx], kind="stable")
    idx = idx[o]; vals = w[idx]
    starts = np.nonzero(np.diff(vals))[0] + 1
    for a_, b_ in zip(np.concatenate([[0], starts]), np.concatenate([starts, [len(idx)]])):
        g.add([int(x) for x in idx[a_:b_]], float(vals[a_]), 'REPLACE')

    for m in list(ob.modifiers):
        if m.type == 'CORRECTIVE_SMOOTH':
            ob.modifiers.remove(m)
    cs = ob.modifiers.new("ArmpitSmooth", 'CORRECTIVE_SMOOTH')
    cs.smooth_type = STYPE
    cs.factor = FACTOR
    cs.iterations = ITERS
    cs.scale = SCALE
    cs.rest_source = 'ORCO'
    cs.use_only_smooth = False
    cs.use_pin_boundary = False
    cs.vertex_group = "CS_armpits"
    cs.invert_vertex_group = False
    # must run AFTER the armature
    names = [m.name for m in ob.modifiers]
    assert names.index("Armature") < names.index("ArmpitSmooth"), names

    R[name] = {"verts": N, "cs_group_verts": int((w > 0).sum()),
               "cs_group_verts_w1": int((w >= 0.999).sum()),
               "cs_group_bbox": [[round(float(x), 3) for x in P[w > 0].min(0)],
                                 [round(float(x), 3) for x in P[w > 0].max(0)]],
               "hardsets_in_group": {k: int((s & (w > 0)).sum()) for k, s in
                                     (("club", CLUB), ("ring_hand", HANDR),
                                      ("handL_ball", HANDL), ("head_core", HEADCORE),
                                      ("belt", BELT), ("shoes", FOOT))},
               "modifier_stack": [(m.name, m.type) for m in ob.modifiers],
               "vertex_groups": len(ob.vertex_groups)}

# ---- rest check with the modifier live
hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
bpy.data.collections["GOB_hi"].hide_viewport = False
bpy.data.collections["GOB_hi"].hide_render = False
hi.hide_viewport = False
hi.hide_render = False
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
for ob in (lo, hi):
    me = ob.data
    P = np.empty(len(me.vertices) * 3); me.vertices.foreach_get("co", P); P = P.reshape(-1, 3)
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    d = np.linalg.norm(Q - P, axis=1)
    gw = np.array([next((g.weight for g in v.groups
                         if ob.vertex_groups[g.group].name == "CS_armpits"), 0.0)
                   for v in me.vertices])
    R[ob.name]["rest_max_dev_mm"] = round(float(d.max()) * 1000, 6)
    R[ob.name]["rest_max_dev_in_group_mm"] = round(float(d[gw > 0].max()) * 1000, 6)
    R[ob.name]["rest_max_dev_outside_group_mm"] = round(float(d[gw <= 0].max()) * 1000, 6)
bpy.data.collections["GOB_hi"].hide_viewport = True
bpy.data.collections["GOB_hi"].hide_render = True
hi.hide_viewport = True
hi.hide_render = True
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name == "GOB_hi":
        lc.hide_viewport = True
        lc.exclude = False

R["n_actions"] = len(bpy.data.actions)
print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(ROOT, "inspect", "step03r2c_csmooth.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
