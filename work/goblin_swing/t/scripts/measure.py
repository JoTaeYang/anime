import bpy, json, math
import numpy as np

body = bpy.data.objects["GOB_body"]
me = body.data
N = len(me.vertices)
P = np.empty(N*3); me.vertices.foreach_get("co", P); P = P.reshape(N,3)
M = np.array(body.matrix_world)
P = (np.c_[P, np.ones(N)] @ M.T)[:, :3]
R = {}
R["bbox"] = [P.min(0).round(5).tolist(), P.max(0).round(5).tolist()]

# ---- spine: y-centroid of the body core (|x|<0.18) per z band
sp = []
for z in np.arange(0.40, 1.92, 0.04):
    m = (np.abs(P[:,2]-z) < 0.012) & (np.abs(P[:,0]) < 0.18)
    if m.sum() < 8: continue
    q = P[m]
    sp.append({"z": round(float(z),3), "n": int(m.sum()),
               "ymin": round(float(q[:,1].min()),4), "ymax": round(float(q[:,1].max()),4),
               "yc": round(float((q[:,1].min()+q[:,1].max())/2),4),
               "xhalf": round(float(np.abs(q[:,0]).max()),4)})
R["spine_scan"] = sp

# ---- full body width per z (for crease / shoulder)
wz = []
for z in np.arange(1.15, 1.45, 0.01):
    m = np.abs(P[:,2]-z) < 0.006
    if m.sum() < 8: continue
    q = P[m]
    core = q[np.abs(q[:,0]) < 0.30]
    wz.append({"z": round(float(z),3),
               "core_xmax": round(float(np.abs(core[:,0]).max()),4) if len(core) else None,
               "core_yspan": [round(float(core[:,1].min()),4), round(float(core[:,1].max()),4)] if len(core) else None})
R["crease_scan"] = wz

# ---- ARM tube scan: slices at x=const, cluster near the arm axis
AXY, AXZ = 0.148, 1.266
arm = []
for x in np.arange(0.28, 0.66, 0.01):
    for sgn in (1, -1):
        m = (np.abs(P[:,0]-sgn*x) < 0.006)
        if m.sum() < 4: continue
        q = P[m]
        d = np.hypot(q[:,1]-AXY, q[:,2]-AXZ)
        sel = q[d < 0.22]
        if len(sel) < 6: continue
        dd = np.hypot(sel[:,1]-AXY, sel[:,2]-AXZ)
        cy = (sel[:,1].min()+sel[:,1].max())/2
        cz = (sel[:,2].min()+sel[:,2].max())/2
        arm.append({"side": "L" if sgn>0 else "R", "x": round(float(x),3),
                    "n": int(len(sel)),
                    "rmax": round(float(dd.max()),4),
                    "rmean": round(float(dd.mean()),4),
                    "cy": round(float(cy),4), "cz": round(float(cz),4),
                    "yspan": round(float(sel[:,1].max()-sel[:,1].min()),4),
                    "zspan": round(float(sel[:,2].max()-sel[:,2].min()),4)})
R["arm_scan"] = arm

# ---- LEG tube scan
leg = []
for z in np.arange(0.12, 0.46, 0.01):
    m = np.abs(P[:,2]-z) < 0.006
    if m.sum() < 6: continue
    q = P[m]
    for sgn in (1, -1):
        s = q[np.sign(q[:,0])==sgn]
        s = s[np.abs(s[:,0]) > 0.15]
        if len(s) < 6: continue
        cx = (s[:,0].min()+s[:,0].max())/2
        cy = (s[:,1].min()+s[:,1].max())/2
        leg.append({"side":"L" if sgn>0 else "R","z":round(float(z),3),"n":int(len(s)),
                    "cx":round(float(cx),4),"cy":round(float(cy),4),
                    "rx":round(float((s[:,0].max()-s[:,0].min())/2),4),
                    "ry":round(float((s[:,1].max()-s[:,1].min())/2),4)})
R["leg_scan"] = leg

# ---- shoe extents
for sgn,nm in ((1,"L"),(-1,"R")):
    m = (P[:,2] < 0.13) & (np.sign(P[:,0])==sgn)
    q = P[m]
    R["shoe_"+nm] = {"n":int(len(q)),"min":q.min(0).round(4).tolist(),"max":q.max(0).round(4).tolist()}

# ---- hand region (right)
m = (P[:,0] < -0.80)
q = P[m]
R["hand_R_bbox"] = [q.min(0).round(4).tolist(), q.max(0).round(4).tolist()]
for x in (-0.82,-0.86,-0.90,-0.94,-0.98,-1.02,-1.06,-1.10,-1.14,-1.18,-1.22,-1.26,-1.30):
    m = np.abs(P[:,0]-x) < 0.007
    if m.sum()<4: continue
    s=P[m]
    R.setdefault("handR_scan",[]).append({"x":x,"n":int(m.sum()),
        "y":[round(float(s[:,1].min()),4),round(float(s[:,1].max()),4)],
        "z":[round(float(s[:,2].min()),4),round(float(s[:,2].max()),4)]})
# left hand
for x in (0.82,0.90,0.98,1.06,1.14,1.22,1.30):
    m = np.abs(P[:,0]-x) < 0.007
    if m.sum()<4: continue
    s=P[m]
    R.setdefault("handL_scan",[]).append({"x":x,"n":int(m.sum()),
        "y":[round(float(s[:,1].min()),4),round(float(s[:,1].max()),4)],
        "z":[round(float(s[:,2].min()),4),round(float(s[:,2].max()),4)]})

print("@@@JSON_START@@@")
print(json.dumps(R))
print("@@@JSON_END@@@")
