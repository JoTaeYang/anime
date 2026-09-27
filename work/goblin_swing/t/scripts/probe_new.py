import bpy, sys, json, math
from mathutils import Vector

SRC = r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx"
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=SRC)
ob = bpy.data.objects["mesh_node"]
me = ob.data
V = [tuple(v.co) for v in me.vertices]
N = len(V)
ZB = min(p[2] for p in V)


def prof(pts, lo, hi, bins):
    d = (hi - lo) / bins
    sl = [[] for _ in range(bins)]
    for p in pts:
        i = int((p[2] - lo) / d)
        if 0 <= i < bins:
            sl[i].append(p)
    r = []
    for i, s in enumerate(sl):
        z = lo + (i + .5) * d
        if not s:
            r.append((z, 0, 0, 0, 0, 0)); continue
        xs = [p[0] for p in s]; ys = [p[1] for p in s]
        r.append((z, len(s), min(xs), max(xs), min(ys), max(ys)))
    return r


print("=== A. CENTRAL COLUMN PROFILE (|x|<0.30, arms/club excluded) ===")
cen = [p for p in V if abs(p[0]) < 0.30]
for (z, n, x0, x1, y0, y1) in prof(cen, ZB, 0.70, 140):
    if n == 0:
        print("  z=%+.4f  (h=%.4f)  EMPTY" % (z, z - ZB)); continue
    print("  z=%+.4f h=%.4f n=%5d  x[%+.4f,%+.4f] w=%.4f  y[%+.4f,%+.4f] d=%.4f" %
          (z, z - ZB, n, x0, x1, x1 - x0, y0, y1, y1 - y0))

print("\n=== B. LEFT/RIGHT ARM BAND, x-sweep of tube radius ===")
# arm band: z where |x|>0.34 exists
arm = [p for p in V if abs(p[0]) > 0.34]
zs = [p[2] for p in arm]
print("  verts with |x|>0.34: %d   z range %.4f..%.4f" % (len(arm), min(zs), max(zs)))
for side, sgn in (("R(-X)", -1), ("L(+X)", 1)):
    print("  -- side %s" % side)
    xa = 0.30
    while xa < 0.96:
        s = [p for p in V if sgn * p[0] >= xa and sgn * p[0] < xa + 0.04]
        if s:
            ys = [p[1] for p in s]; zz = [p[2] for p in s]
            print("     |x|=%.2f n=%5d  y[%+.4f,%+.4f] dy=%.4f  z[%+.4f,%+.4f] dz=%.4f"
                  % (xa, len(s), min(ys), max(ys), max(ys) - min(ys),
                     min(zz), max(zz), max(zz) - min(zz)))
        xa += 0.04

print("\n=== C. CLUB SEED + AXIS FIT ===")
seed = [p for p in V if p[1] < -0.32 and p[0] < -0.30]
print("  seed verts (y<-0.32, x<-0.30): %d" % len(seed))
c = [sum(p[i] for p in seed) / len(seed) for i in range(3)]
# power iteration for principal axis
ax = Vector((0, -1, 0))
for _ in range(60):
    acc = Vector((0, 0, 0))
    for p in seed:
        d = Vector(p) - Vector(c)
        acc += d * d.dot(ax)
    ax = acc.normalized()
if ax.y > 0:
    ax = -ax
print("  centroid %s  axis %s" % ([round(v, 5) for v in c], [round(v, 5) for v in ax]))
print("  axis angle from -Y: %.2f deg ; elevation from horiz: %.2f deg"
      % (math.degrees(math.acos(max(-1, min(1, ax.dot(Vector((0, -1, 0))))))),
         math.degrees(math.asin(ax.z))))
C = Vector(c)
# radial profile along axis over the WHOLE mesh
print("  t = axial coord from centroid (+ = toward club head/-Y)")
rows = []
t = -1.2
while t < 1.2:
    s = []
    for p in V:
        d = Vector(p) - C
        tt = d.dot(ax)
        if t <= tt < t + 0.04:
            s.append((d - ax * tt).length)
    if s:
        s.sort()
        rows.append((t, len(s), s[len(s) // 2], s[-1], s[int(len(s) * .9)]))
    t += 0.04
for (t, n, med, mx, p90) in rows:
    print("     t=%+.3f n=%5d  r_med=%.4f r_p90=%.4f r_max=%.4f %s"
          % (t, n, med, p90, mx, "#" * int(mx * 200)))

print("\n=== D. SYMMETRY (grid NN of mirrored cloud) ===")
CELL = 0.02
grid = {}
for i, p in enumerate(V):
    k = (int(p[0] / CELL), int(p[1] / CELL), int(p[2] / CELL))
    grid.setdefault(k, []).append(i)


def nn(q):
    best = 1e9
    k0 = (int(q[0] / CELL), int(q[1] / CELL), int(q[2] / CELL))
    for r in range(1, 5):
        for a in range(-r, r + 1):
            for b in range(-r, r + 1):
                for cc in range(-r, r + 1):
                    if max(abs(a), abs(b), abs(cc)) != r - 1 and r > 1:
                        continue
                    for i in grid.get((k0[0] + a, k0[1] + b, k0[2] + cc), ()):
                        p = V[i]
                        d = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
                        if d < best:
                            best = d
        if best < ((r - 1) * CELL) ** 2:
            break
    return math.sqrt(best)


def region(p):
    h = p[2] - ZB
    if abs(p[0]) > 0.34:
        return "arm/hand/club"
    if h > 0.98:
        return "head"
    if h > 0.83:
        return "collar/shoulder"
    if h > 0.42:
        return "torso"
    if h > 0.20:
        return "hip"
    if h > 0.10:
        return "leg"
    return "foot"


import random
random.seed(1)
samp = random.sample(range(N), 9000)
agg = {}
for i in samp:
    p = V[i]
    d = nn((-p[0], p[1], p[2]))
    r = region(p)
    agg.setdefault(r, []).append(d)
for r in sorted(agg):
    a = sorted(agg[r])
    print("  %-16s n=%5d  median=%.5f  p95=%.5f  max=%.5f" %
          (r, len(a), a[len(a) // 2], a[int(len(a) * .95)], a[-1]))

print("\n=== E. BACK LUMPS (y>0 protrusions) ===")
for zlo in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]:
    s = [p for p in V if zlo <= p[2] < zlo + 0.05 and abs(p[0]) < 0.45]
    if s:
        ymx = max(p[1] for p in s)
        far = [p for p in s if p[1] > ymx - 0.05]
        print("  z=%.2f  y_max=%+.4f  n_far=%d  x of far verts: %.4f..%.4f" %
              (zlo, ymx, len(far), min(p[0] for p in far), max(p[0] for p in far)))

print("\n=== F. HEAD BOX + COLLAR ===")
head = [p for p in V if p[2] > 0.36]
print("  head(z>0.36) x[%+.4f,%+.4f] y[%+.4f,%+.4f] z[%+.4f,%+.4f]" %
      (min(p[0] for p in head), max(p[0] for p in head), min(p[1] for p in head),
       max(p[1] for p in head), min(p[2] for p in head), max(p[2] for p in head)))
json.dump({"zbottom": ZB}, open(r"C:\Users\whxod\orca\anime\work\goblin_swing\t\inspect\step01\probe.json", "w"))
