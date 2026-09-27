import bpy, json
from mathutils import Vector
ob = bpy.data.objects["mesh_node"]; mw = ob.matrix_world
P = [mw @ v.co for v in ob.data.vertices]

def bb(pts, label):
    if not pts: return {"label": label, "count": 0}
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return {"label": label, "count": len(pts),
            "min": [round(v,4) for v in mn], "max": [round(v,4) for v in mx],
            "size": [round(v,4) for v in (mx-mn)], "center": [round(v,4) for v in (mn+mx)/2]}

out = {}
out["all"] = bb(P, "all")
out["club_side_Xlt-0.30"] = bb([p for p in P if p.x < -0.30], "club+right arm")
out["left_side_Xgt0.30"] = bb([p for p in P if p.x > 0.30], "left arm")
out["head_Zgt0.42_absXlt0.45"] = bb([p for p in P if p.z > 0.42 and abs(p.x) < 0.45], "head")
out["legs_Zlt-0.55"] = bb([p for p in P if p.z < -0.55], "legs+feet")
out["foot_right_Zlt-0.55_Xlt0"] = bb([p for p in P if p.z < -0.55 and p.x < 0], "right foot")
out["foot_left_Zlt-0.55_Xgt0"] = bb([p for p in P if p.z < -0.55 and p.x > 0], "left foot")

# z-slice profile: X extent + count per 0.1 slice
prof = []
z0, z1 = min(p.z for p in P), max(p.z for p in P)
import math
n = 20
for i in range(n):
    a = z0 + (z1-z0)*i/n; b = z0 + (z1-z0)*(i+1)/n
    s = [p for p in P if a <= p.z < b]
    if s:
        prof.append({"z": [round(a,3), round(b,3)], "n": len(s),
                     "x": [round(min(p.x for p in s),3), round(max(p.x for p in s),3)],
                     "y": [round(min(p.y for p in s),3), round(max(p.y for p in s),3)]})
out["z_profile"] = prof

# symmetry: how well does the point cloud mirror across X=0 (coarse voxel hash)
def vox(p, s=0.03): return (round(p.x/s), round(p.y/s), round(p.z/s))
S = set(vox(p) for p in P)
M = set((-a, b, c) for (a, b, c) in S)
out["symmetry_voxel_overlap_ratio"] = round(len(S & M)/len(S), 4)
out["symmetry_note"] = "1.0 = perfectly X-symmetric at 3cm voxel resolution"

print("@@@JSON_START@@@"); print(json.dumps(out, indent=1)); print("@@@JSON_END@@@")
