"""STEP 03 analysis pass 2 - locate the shoulder / hip / ankle / wrist junctions.
Run: blender -b goblin_v03_controls.blend --python step03_analyze2.py   (read-only)
"""
import bpy, json, math, os
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect"
body = bpy.data.objects["GOB_body"]
me = body.data
co = [v.co.copy() for v in me.vertices]
N = len(co)
R = {}

CH = {
    "R": [Vector((-0.312, 0.118, 1.272)), Vector((-0.582, 0.016, 1.072)),
          Vector((-0.676, -0.234, 0.98)), Vector((-0.683, -0.408, 0.995))],
    "L": [Vector((0.32, 0.1, 1.27)), Vector((0.55, 0.152, 1.092)),
          Vector((0.724, 0.08, 0.84)), Vector((0.788, 0.048, 0.681))],
}
LEG = {
    "R": [Vector((-0.278, 0.1, 0.405)), Vector((-0.293, 0.092, 0.283)),
          Vector((-0.307, 0.103, 0.16)), Vector((-0.382, -0.08, 0.09))],
    "L": [Vector((0.278, 0.1, 0.405)), Vector((0.292, 0.092, 0.283)),
          Vector((0.305, 0.103, 0.16)), Vector((0.38, -0.08, 0.09))],
}


def poly_param(p, pts):
    """(arc-length t from pts[0], perpendicular distance) to a polyline."""
    best = (1e9, 0.0)
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        ab = b - a
        L = ab.length
        t = max(0.0, min(1.0, (p - a).dot(ab) / (L * L)))
        d = (p - (a + ab * t)).length
        if d < best[0]:
            best = (d, acc + t * L)
        acc += L
    return best[1], best[0]


# --- arm: perpendicular radius profile along the chain, per side ------------
for S in ("R", "L"):
    prof = {}
    for p in co:
        t, d = poly_param(p, CH[S])
        if d < 0.45:
            k = round(t, 2)
            e = prof.setdefault(k, [0, 0.0, 9.0])
            e[0] += 1
            e[1] = max(e[1], round(d, 4))
            e[2] = min(e[2], round(d, 4))
    # only report the first 25 cm along the chain (the shoulder junction)
    R["arm%s_profile" % S] = {str(k): prof[k] for k in sorted(prof) if k <= 0.30}

# --- leg: radius profile by z ---------------------------------------------
for S in ("R", "L"):
    sg = 1.0 if S == "L" else -1.0
    prof = {}
    for p in co:
        if p.z > 0.62:
            continue
        r = math.hypot(p.x - sg * 0.285, p.y - 0.10)
        if r < 0.40:
            k = round(p.z, 2)
            e = prof.setdefault(k, [0, 0.0])
            e[0] += 1
            e[1] = max(e[1], round(r, 4))
    R["leg%s_profile" % S] = {str(k): prof[k] for k in sorted(prof) if 0.30 <= k <= 0.62}

# --- torso radius profile about (0, 0.09) ---------------------------------
tp = {}
for p in co:
    if abs(p.x) < 0.95:
        r = math.hypot(p.x, p.y - 0.09)
        k = round(p.z, 2)
        e = tp.setdefault(k, [0, 0.0])
        e[0] += 1
        e[1] = max(e[1], round(r, 4))
R["torso_profile"] = {str(k): tp[k] for k in sorted(tp)}

# --- wrist: distance from the right hand ball centre along the forearm -----
HC_R = Vector((-0.676, -0.364, 0.992))
HC_L = Vector((0.772, 0.056, 0.720))
for S, HC in (("R", HC_R), ("L", HC_L)):
    prof = {}
    for p in co:
        t, d = poly_param(p, CH[S])
        if d < 0.20:
            k = round((p - HC).length, 2)
            e = prof.setdefault(k, [0, 0.0])
            e[0] += 1
            e[1] = max(e[1], round(d, 4))
    R["ball%s_profile" % S] = {str(k): prof[k] for k in sorted(prof) if k <= 0.35}

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(OUT, "step03_analyze2.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("DONE")
