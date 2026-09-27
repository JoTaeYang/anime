import bpy, json, math
from mathutils import Vector

ob = bpy.data.objects["GOB_body"]
V = [v.co.copy() for v in ob.data.vertices]
out = {}

CLUB_P = Vector((-0.6965, -0.3698, 1.2096))
CLUB_D = Vector((-0.0892, -0.0128, 0.9959)).normalized()


def dist_axis(p, o, d):
    v = p - o
    return (v - d * v.dot(d)).length


def prof(label, pts, d, t0, t1, step, origin=Vector((0, 0, 0))):
    """slice along unit dir d; report centroid + per-axis extents of each slab"""
    d = Vector(d).normalized()
    rows = []
    t = t0
    while t < t1 - 1e-9:
        sl = [p for p in pts if t <= (p - origin).dot(d) < t + step]
        if len(sl) >= 6:
            c = Vector((0, 0, 0))
            for p in sl:
                c += p
            c /= len(sl)
            ext = [[round(min(p[i] for p in sl), 4), round(max(p[i] for p in sl), 4)]
                   for i in range(3)]
            rr = [( (p - c) - d * (p - c).dot(d) ).length for p in sl]
            rr.sort()
            rows.append({"t": round(t + step / 2, 4), "n": len(sl),
                         "c": [round(x, 4) for x in c], "ext": ext,
                         "rmean": round(sum(rr) / len(rr), 4),
                         "rp90": round(rr[int(len(rr) * .9)], 4)})
        t += step
    return {"label": label, "dir": [round(x, 4) for x in d], "rows": rows}


# ---- right arm: exclude club cylinder
armR = [p for p in V if p.x < -0.30 and 0.80 < p.z < 1.40
        and dist_axis(p, CLUB_P, CLUB_D) > 0.105]
out["armR_alongX"] = prof("right arm (club removed), slice X", armR, (1, 0, 0),
                          -0.80, -0.28, 0.025)

# ---- right hand ring: verts near club axis between z .85 and 1.15
ring = [p for p in V if p.x < -0.50 and 0.80 < p.z < 1.20
        and 0.05 < dist_axis(p, CLUB_P, CLUB_D) < 0.20]
out["ringR"] = {"n": len(ring),
                "bbox": [[round(min(p[i] for p in ring), 4), round(max(p[i] for p in ring), 4)] for i in range(3)],
                "c": [round(sum(p[i] for p in ring) / len(ring), 4) for i in range(3)]}

# ---- left arm
armL = [p for p in V if p.x > 0.30 and 0.55 < p.z < 1.40]
out["armL_alongX"] = prof("left arm, slice X", armL, (1, 0, 0), 0.30, 0.92, 0.025)

# ---- torso only: |x| < 0.45 after removing arms is hard; profile X-extent per z of
#      the central body (exclude anything farther than 0.45 from x=0)
torso = [p for p in V if abs(p.x) < 0.45 and 0.30 < p.z < 1.45]
out["torso_Z"] = prof("torso slice Z", torso, (0, 0, 1), 0.30, 1.45, 0.025)

# ---- head
head = [p for p in V if abs(p.x) < 0.45 and p.z > 1.28 and abs(p.y) < 0.45]
out["head_bbox"] = {"n": len(head),
                    "bbox": [[round(min(p[i] for p in head), 4), round(max(p[i] for p in head), 4)] for i in range(3)],
                    "c": [round(sum(p[i] for p in head) / len(head), 4) for i in range(3)]}

# ---- legs: profile each leg tube along Z, and feet
for sgn, nm in ((-1, "R"), (1, "L")):
    leg = [p for p in V if sgn * p.x > 0.12 and sgn * p.x < 0.50 and 0.10 < p.z < 0.46]
    out["leg" + nm + "_Z"] = prof("leg %s slice Z" % nm, leg, (0, 0, 1), 0.10, 0.46, 0.02)
    foot = [p for p in V if sgn * p.x > 0.05 and p.z < 0.19]
    out["foot" + nm] = {"n": len(foot),
                        "bbox": [[round(min(p[i] for p in foot), 4), round(max(p[i] for p in foot), 4)] for i in range(3)],
                        "c": [round(sum(p[i] for p in foot) / len(foot), 4) for i in range(3)]}
    out["foot" + nm + "_Y"] = prof("foot %s slice Y" % nm, foot, (0, 1, 0), -0.30, 0.40, 0.04)

# ---- whole-body X extent per Z (to see torso vs arms)
out["allZ"] = prof("all verts slice Z", V, (0, 0, 1), 0.0, 1.91, 0.05)

print("@@@JSON_START@@@")
print(json.dumps(out))
print("@@@JSON_END@@@")
