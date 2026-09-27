"""STEP03 probe: measure gobT geometry needed for the analytic skinning rules."""
import bpy, json, os, sys
import numpy as np

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
rig = bpy.data.objects["GOB_rig"]; arm = rig.data
R = {}

def getP(name):
    ob = bpy.data.objects[name]; me = ob.data
    n = len(me.vertices)
    P = np.empty(n*3); me.vertices.foreach_get("co", P)
    return ob, P.reshape(n,3)

R["objects"] = {o.name: {"type": o.type, "verts": len(o.data.vertices) if o.type=='MESH' else None,
                         "parent": o.parent.name if o.parent else None,
                         "mods": [m.type for m in o.modifiers],
                         "vg": len(o.vertex_groups) if o.type=='MESH' else None,
                         "hide_v": o.hide_viewport, "hide_r": o.hide_render,
                         "colls": [c.name for c in o.users_collection],
                         "mw_is_id": bool(np.allclose(np.array(o.matrix_world), np.eye(4), atol=1e-6))}
                for o in bpy.data.objects}
R["collections"] = {c.name: {"hv": c.hide_viewport, "hr": c.hide_render} for c in bpy.data.collections}
R["n_actions"] = len(bpy.data.actions)

for nm in ("GOB_body_lo", "GOB_body"):
    ob, P = getP(nm)
    Z = P[:,2]; X = P[:,0]; Y = P[:,1]
    d = {}
    d["bbox"] = [[round(float(v),4) for v in P.min(0)], [round(float(v),4) for v in P.max(0)]]
    # arm profile: per side, s along UPPERARM axis from shoulder head, d radial
    for S in ("L","R"):
        sh = np.array(arm.bones["UPPERARM_"+S].head_local)
        tl = np.array(arm.bones["UPPERARM_"+S].tail_local)
        u = (tl-sh); u/=np.linalg.norm(u)
        v = P - sh; s = v@u; rr = np.linalg.norm(v - s[:,None]*u, axis=1)
        prof = []
        for lo in np.arange(-0.20, 0.95, 0.01):
            m = (s>=lo)&(s<lo+0.01)
            if m.sum()>=4:
                prof.append([round(float(lo)+0.005,3), int(m.sum()),
                             round(float(np.percentile(rr[m],50)),4),
                             round(float(np.percentile(rr[m],97)),4),
                             round(float(rr[m].max()),4)])
        d["armprof_"+S] = prof
        # hand ball: verts beyond wrist |x|
        wx = abs(float(arm.bones["HAND_"+S].head_local[0]))
        hm = (np.abs(X) > wx - 0.02) & ((X>0) if S=="L" else (X<0))
        d["hand_"+S] = {"n": int(hm.sum()),
                        "xr": [round(float(X[hm].min()),4), round(float(X[hm].max()),4)],
                        "zr": [round(float(Z[hm].min()),4), round(float(Z[hm].max()),4)],
                        "yr": [round(float(Y[hm].min()),4), round(float(Y[hm].max()),4)]}
    # z-histogram of vert count in torso column (crease detection)
    rT = np.hypot(X, Y-0.105)
    zh = []
    for lo in np.arange(1.15, 1.45, 0.01):
        m = (Z>=lo)&(Z<lo+0.01)&(rT<0.55)
        zh.append([round(float(lo)+0.005,3), int(m.sum()),
                   round(float(rT[m].max()),4) if m.sum() else 0.0])
    d["neck_zhist"] = zh
    # leg tube per side
    for S in ("L","R"):
        A = np.array(arm.bones["FOOT_"+S].head_local)
        H = np.array(arm.bones["THIGH_"+S].head_local)
        u = (H-A); L=float(np.linalg.norm(u)); u/=L
        v = P - A; s = v@u; rr = np.linalg.norm(v - s[:,None]*u, axis=1)
        side = (X>=0) if S=="L" else (X<0)
        prof=[]
        for lo in np.arange(-0.05, 0.40, 0.01):
            m = side&(s>=lo)&(s<lo+0.01)&(rr<0.30)
            if m.sum()>=3:
                prof.append([round(float(lo)+0.005,3), int(m.sum()),
                             round(float(np.percentile(rr[m],97)),4), round(float(rr[m].max()),4)])
        d["legprof_"+S] = {"L_axis": round(L,4), "prof": prof}
    # shoe top z
    d["shoe_zhist"] = [[round(float(lo)+0.005,3), int(((Z>=lo)&(Z<lo+0.01)).sum())]
                       for lo in np.arange(0.10, 0.30, 0.01)]
    # belt region vert count
    d["nverts"] = len(P)
    d["edges"] = len(ob.data.edges)
    R[nm] = d

print("@@@JSON_START@@@"); print(json.dumps(R, default=str)); print("@@@JSON_END@@@")
with open(os.path.join(T,"inspect","step03","probe.json"),"w") as f:
    json.dump(R, f, indent=1, default=str)
