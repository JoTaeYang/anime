"""STEP03 probe 2: shoulder fillet/tube envelope, neck crease, hand/bore, islands."""
import bpy, json, os
import numpy as np

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
rig = bpy.data.objects["GOB_rig"]; arm = rig.data
R = {}

def getPE(name):
    ob = bpy.data.objects[name]; me = ob.data
    n = len(me.vertices)
    P = np.empty(n*3); me.vertices.foreach_get("co", P); P = P.reshape(n,3)
    E = np.empty(len(me.edges)*2, dtype=np.int32); me.edges.foreach_get("vertices", E)
    return ob, P, E.reshape(-1,2)

for nm in ("GOB_body_lo","GOB_body"):
    ob, P, E = getPE(nm)
    n = len(P); X,Y,Z = P[:,0],P[:,1],P[:,2]
    d = {}
    # ---- islands
    ah = np.concatenate([E[:,0],E[:,1]]); at = np.concatenate([E[:,1],E[:,0]])
    o = np.argsort(ah, kind="stable"); ah=ah[o]; at=at[o]
    st = np.searchsorted(ah, np.arange(n+1))
    seen = np.zeros(n,bool); isl=[]
    for i0 in range(n):
        if seen[i0]: continue
        stack=[i0]; seen[i0]=True; c=[]
        while stack:
            v=stack.pop(); c.append(v)
            for k in range(st[v],st[v+1]):
                w=at[k]
                if not seen[w]: seen[w]=True; stack.append(w)
        isl.append(np.array(c))
    isl.sort(key=len, reverse=True)
    d["islands"] = [{"n":int(len(c)),
                     "bbox":[[round(float(x),3) for x in P[c].min(0)],
                             [round(float(x),3) for x in P[c].max(0)]]} for c in isl[:12]]
    d["n_islands"] = len(isl)
    # ---- shoulder: slices along arm axis, filtered to near the arm axis & shoulder height
    for S in ("L","R"):
        sh = np.array(arm.bones["UPPERARM_"+S].head_local)
        tl = np.array(arm.bones["UPPERARM_"+S].tail_local)
        u=(tl-sh); u/=np.linalg.norm(u)
        v=P-sh; s=v@u; rr=np.linalg.norm(v-s[:,None]*u,axis=1)
        near = (rr<0.32) & (Z>1.02) & (Z<1.50)
        prof=[]
        for lo in np.arange(-0.10, 0.30, 0.01):
            m = near&(s>=lo)&(s<lo+0.01)
            if m.sum()>=3:
                prof.append([round(float(lo)+0.005,3), int(m.sum()),
                             round(float(np.percentile(rr[m],50)),4),
                             round(float(np.percentile(rr[m],97)),4),
                             round(float(rr[m].max()),4)])
        d["shprof_"+S]=prof
        # x-profile of the fillet/tube (|x| slices, near arm axis)
        xp=[]
        for lo in np.arange(0.30, 0.70, 0.01):
            m = (np.abs(X)>=lo)&(np.abs(X)<lo+0.01)&((X>0) if S=="L" else (X<0))&(Z>1.05)&(rr<0.35)
            if m.sum()>=3:
                xp.append([round(float(lo)+0.005,3), int(m.sum()),
                           round(float(np.percentile(rr[m],50)),4),
                           round(float(np.percentile(rr[m],97)),4)])
        d["xprof_"+S]=xp
        # wrist/hand transition along the arm axis
        wr=[]
        for lo in np.arange(0.40, 1.05, 0.01):
            m=(rr<0.35)&(s>=lo)&(s<lo+0.01)
            if m.sum()>=2:
                wr.append([round(float(lo)+0.005,3), int(m.sum()),
                           round(float(np.percentile(rr[m],97)),4), round(float(rr[m].max()),4)])
        d["wristprof_"+S]=wr
    # ---- neck crease: rT profile by z near the axis
    rT=np.hypot(X,Y-0.105)
    nz=[]
    for lo in np.arange(1.20,1.42,0.005):
        m=(Z>=lo)&(Z<lo+0.005)&(rT<0.45)&(np.abs(X)<0.40)
        if m.sum()>=2:
            nz.append([round(float(lo)+0.0025,4), int(m.sum()),
                       round(float(np.percentile(rT[m],97)),4), round(float(rT[m].max()),4)])
    d["creaseprof"]=nz
    R[nm]=d

# ---- club bore: club local verts + hand R verts inside the bore
club=bpy.data.objects["GOB_club"]
cm=club.data
cp=np.empty(len(cm.vertices)*3); cm.vertices.foreach_get("co",cp); cp=cp.reshape(-1,3)
mw=np.array(club.matrix_world)
cw=cp@mw[:3,:3].T+mw[:3,3]
R["club_world_bbox"]=[[round(float(x),4) for x in cw.min(0)],[round(float(x),4) for x in cw.max(0)]]
# grip axis in world = WEAPON bone y axis
wb=arm.bones["WEAPON"]
gy=np.array(wb.tail_local)-np.array(wb.head_local); gy/=np.linalg.norm(gy)
gh=np.array(wb.head_local)
for nm in ("GOB_body_lo","GOB_body"):
    ob=bpy.data.objects[nm]; me=ob.data
    P=np.empty(len(me.vertices)*3); me.vertices.foreach_get("co",P); P=P.reshape(-1,3)
    v=P-gh; sg=v@gy; rg=np.linalg.norm(v-sg[:,None]*gy,axis=1)
    bore=(rg<0.075)&(sg>-0.16)&(sg<0.16)
    R.setdefault("bore",{})[nm]={"n":int(bore.sum()),
        "bbox":[[round(float(x),4) for x in P[bore].min(0)],[round(float(x),4) for x in P[bore].max(0)]] if bore.sum() else None,
        "r_range":[round(float(rg[bore].min()),4),round(float(rg[bore].max()),4)] if bore.sum() else None}
print("@@@JSON_START@@@"); print(json.dumps(R,default=str)); print("@@@JSON_END@@@")
with open(os.path.join(T,"inspect","step03","probe2.json"),"w") as f: json.dump(R,f,indent=1,default=str)
