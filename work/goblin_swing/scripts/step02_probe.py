import bpy, json
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ob = bpy.data.objects["GOB_body"]
bvh = BVHTree.FromObject(ob, bpy.context.evaluated_depsgraph_get())
RAYS = [Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1)),
        Vector((0.577, 0.577, 0.577)), Vector((-0.577, 0.3, 0.75)),
        Vector((0.26, -0.8, 0.54))]


def inside(p):
    v = 0
    for d in RAYS:
        cur = Vector(p) + d * 1e-5
        n = 0
        for _ in range(80):
            h = bvh.ray_cast(cur, d)
            if h[0] is None:
                break
            n += 1
            cur = h[0] + d * 1e-5
        if n % 2 == 1:
            v += 1
    return v >= 4


def span(z, y, xr=(-0.75, 0.75), step=0.005):
    """list of solid intervals along X"""
    segs = []
    cur = None
    x = xr[0]
    while x <= xr[1]:
        s = inside((x, y, z))
        if s and cur is None:
            cur = x
        if not s and cur is not None:
            segs.append([round(cur, 3), round(x - step, 3)])
            cur = None
        x += step
    if cur is not None:
        segs.append([round(cur, 3), round(xr[1], 3)])
    return segs


out = {}
for z in (1.20, 1.24, 1.27, 1.30):
    for y in (-0.10, 0.02, 0.09, 0.18, 0.28):
        out["z%.2f_y%.2f" % (z, y)] = span(z, y)
# torso width profile at the arm-root depth and behind it
for z in (0.60, 0.80, 1.00, 1.10, 1.15):
    out["z%.2f_y0.09" % z] = span(z, 0.09)
print("@@@JSON_START@@@")
print(json.dumps(out, indent=0))
print("@@@JSON_END@@@")
