import bpy, json, math
from mathutils import Vector

ob = bpy.data.objects["GOB_body"]
P = [v.co.copy() for v in ob.data.vertices]  # identity transform

AX = {"X": 0, "Y": 1, "Z": 2}


def clusters_2d(pts, a1, a2, cell):
    """grid connected-components on the 2 non-slice axes. pts = list of Vector."""
    grid = {}
    for p in pts:
        k = (int(math.floor(p[a1] / cell)), int(math.floor(p[a2] / cell)))
        grid.setdefault(k, []).append(p)
    seen = set()
    out = []
    for k in grid:
        if k in seen:
            continue
        stack = [k]
        seen.add(k)
        comp = []
        while stack:
            c = stack.pop()
            comp.extend(grid[c])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in grid and n not in seen:
                        seen.add(n)
                        stack.append(n)
        cx = sum(p.x for p in comp) / len(comp)
        cy = sum(p.y for p in comp) / len(comp)
        cz = sum(p.z for p in comp) / len(comp)
        e1 = [p[a1] for p in comp]
        e2 = [p[a2] for p in comp]
        out.append({
            "n": len(comp),
            "c": [round(cx, 4), round(cy, 4), round(cz, 4)],
            "ext1": [round(min(e1), 4), round(max(e1), 4)],
            "ext2": [round(min(e2), 4), round(max(e2), 4)],
            "r": round(0.25 * ((max(e1) - min(e1)) + (max(e2) - min(e2))), 4),
        })
    out.sort(key=lambda d: -d["n"])
    return out


def scan(label, subset, axis, lo, hi, step, cell=0.02, minn=15, maxclusters=4):
    a = AX[axis]
    o1, o2 = [i for i in (0, 1, 2) if i != a]
    rows = []
    t = lo
    while t < hi - 1e-9:
        sl = [p for p in subset if t <= p[a] < t + step]
        if len(sl) >= minn:
            cl = [c for c in clusters_2d(sl, o1, o2, cell) if c["n"] >= minn]
            rows.append({"t": round(t + step / 2, 4), "n": len(sl),
                         "cl": cl[:maxclusters]})
        t += step
    return {"label": label, "axis": axis, "rows": rows}


out = {}
bb_min = Vector((min(p.x for p in P), min(p.y for p in P), min(p.z for p in P)))
bb_max = Vector((max(p.x for p in P), max(p.y for p in P), max(p.z for p in P)))
out["bbox"] = {"min": [round(c, 4) for c in bb_min], "max": [round(c, 4) for c in bb_max]}

# ---- 1. right side (x<-0.25): arm tube + club, slice along X
right = [p for p in P if p.x < -0.25]
out["right_X"] = scan("right arm+club, X slabs", right, "X", -0.95, -0.25, 0.03)

# ---- 2. left side (x>0.25), slice along X
left = [p for p in P if p.x > 0.25]
out["left_X"] = scan("left arm, X slabs", left, "X", 0.25, 0.95, 0.03)

# ---- 3. legs: z < 0.45 (below torso underside), slice along Z
legs = [p for p in P if p.z < 0.45]
out["legs_Z"] = scan("legs, Z slabs", legs, "Z", 0.0, 0.46, 0.02)

# ---- 4. torso/head column: |x|<0.45, slice along Z  (full height)
col = [p for p in P if abs(p.x) < 0.42]
out["column_Z"] = scan("torso+head column, Z slabs", col, "Z", 0.30, 1.91, 0.03,
                       cell=0.03, minn=30, maxclusters=3)

# ---- 5. right arm/club separation at fine Z near hand: slice along Z for x<-0.5
rh = [p for p in P if p.x < -0.50]
out["right_lower_Z"] = scan("right hand/club region, Z slabs", rh, "Z", 0.0, 1.90, 0.03)

# ---- 6. feet: slice along Y for each foot below z=0.20
for nm, f in (("foot_R", lambda p: p.x < 0), ("foot_L", lambda p: p.x > 0)):
    s = [p for p in P if p.z < 0.22 and f(p)]
    out[nm + "_Y"] = scan(nm + " Y slabs", s, "Y", -0.30, 0.40, 0.04)

print("@@@JSON_START@@@")
print(json.dumps(out))
print("@@@JSON_END@@@")
