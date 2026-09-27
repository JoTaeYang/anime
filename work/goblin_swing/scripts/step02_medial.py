import bpy, json, math
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ob = bpy.data.objects["GOB_body"]
bvh = BVHTree.FromObject(ob, bpy.context.evaluated_depsgraph_get())

RAYS = [Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1)),
        Vector((0.577, 0.577, 0.577)), Vector((-0.577, 0.3, 0.75))]


def inside(p):
    votes = 0
    for d in RAYS:
        o = Vector(p)
        n = 0
        cur = o + d * 1e-5
        for _ in range(64):
            hit = bvh.ray_cast(cur, d)
            if hit[0] is None:
                break
            n += 1
            cur = hit[0] + d * 1e-5
        if n % 2 == 1:
            votes += 1
    return votes >= 3


def dsurf(p):
    loc, nor, idx, dist = bvh.find_nearest(Vector(p))
    return dist if loc is not None else 0.0


def ridge(box, step):
    x0, x1, y0, y1, z0, z1 = box
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    nz = int((z1 - z0) / step) + 1
    D = {}
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                p = Vector((x0 + i * step, y0 + j * step, z0 + k * step))
                D[(i, j, k)] = dsurf(p) if inside(p) else -1.0
    pts = []
    for (i, j, k), d in D.items():
        if d <= 0.004:
            continue
        best = True
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if di == dj == dk == 0:
                        continue
                    nd = D.get((i + di, j + dj, k + dk), -1.0)
                    if nd > d + 1e-9:
                        best = False
        if best:
            pts.append((Vector((x0 + i * step, y0 + j * step, z0 + k * step)), d))
    return pts


def dump(name, box, step, order_axis, order_sign=1):
    pts = ridge(box, step)
    pts.sort(key=lambda t: order_sign * t[0][order_axis])
    rows = [{"p": [round(x, 4) for x in p], "d": round(d, 4)} for p, d in pts]
    return {"n": len(rows), "step": step, "pts": rows}


out = {}
# right upper arm + forearm region (excludes club head which is z>1.30, x<-0.60)
out["armR"] = dump("armR", (-0.82, -0.28, -0.46, 0.28, 0.92, 1.34), 0.012, 2, -1)
# left arm
out["armL"] = dump("armL", (0.40, 0.92, -0.16, 0.30, 0.60, 1.34), 0.012, 0, -1)
# right leg
out["legR"] = dump("legR", (-0.46, -0.12, -0.06, 0.30, 0.14, 0.46), 0.010, 2)
# left leg
out["legL"] = dump("legL", (0.12, 0.46, -0.06, 0.30, 0.14, 0.46), 0.010, 2)
# right foot
out["footR"] = dump("footR", (-0.62, -0.05, -0.26, 0.36, 0.01, 0.20), 0.012, 1)
# left foot
out["footL"] = dump("footL", (0.05, 0.62, -0.26, 0.36, 0.01, 0.20), 0.012, 1)
# club (shaft + head)
out["club"] = dump("club", (-0.94, -0.54, -0.52, -0.24, 0.64, 1.87), 0.014, 2)
# torso + head central column
out["core"] = dump("core", (-0.30, 0.30, -0.34, 0.50, 0.34, 1.90), 0.020, 2)

print("@@@JSON_START@@@")
print(json.dumps(out))
print("@@@JSON_END@@@")
