import bpy, math, bmesh
from mathutils import Vector

SRC = r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx"
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=SRC)
ob = bpy.data.objects["mesh_node"]
me = ob.data
V = [tuple(v.co) for v in me.vertices]
ZB = min(p[2] for p in V)

print("=== G. OUTER-ARM Y-SWEEP (cross sections) ===")
for lbl, sel in (("RIGHT x<-0.50", lambda p: p[0] < -0.50),
                 ("LEFT  x>+0.50", lambda p: p[0] > 0.50)):
    print("--", lbl)
    y = -0.56
    while y < 0.52:
        s = [p for p in V if sel(p) and y <= p[1] < y + 0.03]
        if s:
            xs = [p[0] for p in s]; zs = [p[2] for p in s]
            print("  y=%+.3f n=%5d  x[%+.4f,%+.4f] cx=%+.4f dx=%.4f  z[%+.4f,%+.4f] cz=%+.4f dz=%.4f"
                  % (y, len(s), min(xs), max(xs), (min(xs) + max(xs)) / 2, max(xs) - min(xs),
                     min(zs), max(zs), (min(zs) + max(zs)) / 2, max(zs) - min(zs)))
        y += 0.03

print("\n=== H. ARM TUBE along X (cross-section centre+radius), torso-side ===")
for lbl, sgn in (("RIGHT", -1), ("LEFT", 1)):
    print("--", lbl)
    xa = 0.24
    while xa < 0.68:
        s = [p for p in V if sgn * p[0] >= xa and sgn * p[0] < xa + 0.02
             and 0.10 < p[2] < 0.36 and 0.05 < p[1] < 0.40]
        if s:
            ys = [p[1] for p in s]; zs = [p[2] for p in s]
            print("  |x|=%.3f n=%5d  cy=%+.4f dy=%.4f  cz=%+.4f dz=%.4f  r~%.4f"
                  % (xa, len(s), (min(ys) + max(ys)) / 2, max(ys) - min(ys),
                     (min(zs) + max(zs)) / 2, max(zs) - min(zs),
                     (max(ys) - min(ys) + max(zs) - min(zs)) / 4))
        xa += 0.02

print("\n=== I. CLEARANCES ===")
head = [p for p in V if p[2] > 0.345 and abs(p[0]) < 0.30]
arms = {-1: [p for p in V if p[0] < -0.30 and 0.10 < p[2] < 0.36],
        1: [p for p in V if p[0] > 0.30 and 0.10 < p[2] < 0.36]}
torso = [p for p in V if -0.30 < p[2] < 0.22 and abs(p[0]) < 0.42]


def mind(A, B):
    best = 1e9; ba = None
    for a in A:
        for b in B:
            d = (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
            if d < best:
                best = d; ba = (a, b)
    return math.sqrt(best), ba


import random
random.seed(2)
for sgn, nm in ((-1, "RIGHT"), (1, "LEFT")):
    A = [p for p in arms[sgn] if abs(p[0]) > 0.30 and abs(p[0]) < 0.50]
    A = random.sample(A, min(900, len(A)))
    H = random.sample(head, min(2500, len(head)))
    d, pr = mind(A, H)
    print("  %s arm(0.30<|x|<0.50)  ->  head : %.4f fbx-units   at arm%s head%s"
          % (nm, d, [round(v, 3) for v in pr[0]], [round(v, 3) for v in pr[1]]))
    T = [p for p in torso if sgn * p[0] > 0.20]
    T = random.sample(T, min(2500, len(T)))
    d2, pr2 = mind(A, T)
    print("  %s arm                 ->  torso flank(z<0.22): %.4f  at arm%s torso%s"
          % (nm, d2, [round(v, 3) for v in pr2[0]], [round(v, 3) for v in pr2[1]]))

print("\n=== J. CREASE / BELT DETECTION (sharp edges) ===")
bm = bmesh.new(); bm.from_mesh(me)
bm.edges.ensure_lookup_table()
buck = {}
for e in bm.edges:
    if len(e.link_faces) != 2:
        continue
    a = e.calc_face_angle(0.0)
    if a > 0.6:
        mid = (e.verts[0].co + e.verts[1].co) / 2
        if abs(mid.x) < 0.42 and mid.y < 0.1:
            buck.setdefault(round(mid.z, 2), 0)
            buck[round(mid.z, 2)] += 1
bm.free()
for z in sorted(buck):
    if buck[z] > 15:
        print("  z=%+.2f h=%.4f  sharp-edge count %d  %s" % (z, z - ZB, buck[z], "*" * (buck[z] // 20)))

print("\n=== K. LEGS ===")
z = -0.62
while z < -0.30:
    s = [p for p in V if z <= p[2] < z + 0.02]
    if s:
        L = [p for p in s if p[0] > 0.02]; R = [p for p in s if p[0] < -0.02]
        if L and R:
            print("  z=%+.3f h=%.4f  R x[%+.4f,%+.4f] cy=%+.4f  | L x[%+.4f,%+.4f]  gap=%.4f"
                  % (z, z - ZB, min(p[0] for p in R), max(p[0] for p in R),
                     (min(p[1] for p in R) + max(p[1] for p in R)) / 2,
                     min(p[0] for p in L), max(p[0] for p in L),
                     min(p[0] for p in L) - max(p[0] for p in R)))
    z += 0.02
