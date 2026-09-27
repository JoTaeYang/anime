import bpy, json, os, math
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

G = r"C:\Users\whxod\orca\anime\work\goblin_swing"
OUT = os.path.join(G, "t", "inspect", "step04")
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
cam = bpy.data.objects["REF_CAM"]
sc.camera = cam
W, H = sc.render.resolution_x, sc.render.resolution_y
ob = bpy.data.objects["GOB_body"]        # hi-poly: landmark source
V = [v.co.copy() for v in ob.data.vertices]


def px(p):
    c = world_to_camera_view(sc, cam, Vector(p))
    return (c.x * W, (1 - c.y) * H)


ZT = max(v.z for v in V); ZB = min(v.z for v in V)
head = [v for v in V if v.z > 1.30]
yf = min(v.y for v in head)
eyes = [v for v in head if v.y < yf + 0.012]
L = [v for v in eyes if v.x > 0]; R = [v for v in eyes if v.x < 0]


def cen(s): return Vector((sum(v.x for v in s) / len(s), sum(v.y for v in s) / len(s),
                           sum(v.z for v in s) / len(s)))


eL, eR = cen(L), cen(R)
BL, BH = 0.4252 * 1.36999, 0.5752 * 1.36999      # belt creases -> world z
belt = [v for v in V if BL - .01 < v.z < BH + .01 and abs(v.x) < 0.7]
feet = [v for v in V if v.z < ZB + 0.02]
torso = [v for v in V if abs(v.z - (BL + BH) / 2) < 0.02 and abs(v.x) < 0.8]

pt_head = px((0, 0, ZT))
pL, pR = px(eL), px(eR)
pbelt_t = px((0, 0, BH)); pbelt_b = px((0, 0, BL))
pfeet = px((0, 0, ZB))
fx0 = px((min(v.x for v in feet), 0, ZB))[0]; fx1 = px((max(v.x for v in feet), 0, ZB))[0]
tx0 = px((min(v.x for v in torso), 0, (BL + BH) / 2))[0]
tx1 = px((max(v.x for v in torso), 0, (BL + BH) / 2))[0]
bx0 = px((min(v.x for v in belt), 0, (BL + BH) / 2))[0]
bx1 = px((max(v.x for v in belt), 0, (BL + BH) / 2))[0]

ref = json.load(open(os.path.join(G, "ref", "metrics.json")))["frames"][0]
rows = [
    ("head_top_y", pt_head[1], ref["head_top_y"]),
    ("eye_mid_x", (pL[0] + pR[0]) / 2, ref["eye_mid"][0]),
    ("eye_mid_y", (pL[1] + pR[1]) / 2, ref["eye_mid"][1]),
    ("eye_sep_px", abs(pL[0] - pR[0]), ref["eye_sep_px"]),
    ("belt_top_y", pbelt_t[1], ref["hips_belt_bbox"][1]),
    ("belt_bottom_y", pbelt_b[1], ref["hips_belt_bbox"][3]),
    ("belt_x_min", bx0, ref["hips_belt_bbox"][0]),
    ("belt_x_max", bx1, ref["hips_belt_bbox"][2]),
    ("feet_bottom_y", pfeet[1], ref["feet_bboxes"][0][3]),
    ("feet_x_min", fx0, ref["feet_bboxes"][0][0]),
    ("feet_x_max", fx1, ref["feet_bboxes"][1][2]),
    ("torso_w_at_belt", tx1 - tx0, ref["hips_belt_bbox"][2] - ref["hips_belt_bbox"][0]),
]
print("landmark               ours      ref      dpx")
res = []
for n, a, b in rows:
    print("  %-20s %8.1f %8.1f %+8.1f" % (n, a, b, a - b))
    res.append({"landmark": n, "ours_px": round(a, 1), "ref_px": b, "d_px": round(a - b, 1)})
json.dump(res, open(os.path.join(OUT, "ref_landmarks.json"), "w"), indent=1)

# overlay render on reference frame 1
sc.render.film_transparent = True
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'FLAT'
sc.display.shading.color_type = 'SINGLE'
sc.display.shading.single_color = (1, 0.25, 0.1)
sc.frame_set(1)
sc.render.filepath = os.path.join(OUT, "render_f0001.png")
bpy.ops.render.render(write_still=True)
r = bpy.data.images.load(os.path.join(OUT, "render_f0001.png"))
b = bpy.data.images.load(os.path.join(G, "ref", "frames", "f_0001.png"))
rp = list(r.pixels); bp = list(b.pixels)
n = min(len(rp), len(bp))
out = bpy.data.images.new("ov", W, H, alpha=True)
o = [0.0] * n
for i in range(0, n, 4):
    a = rp[i + 3] * 0.55
    for c in range(3):
        o[i + c] = bp[i + c] * (1 - a) + rp[i + c] * a
    o[i + 3] = 1.0
out.pixels = o
out.filepath_raw = os.path.join(OUT, "overlay_f0001.png")
out.file_format = 'PNG'
out.save()
print("WROTE", out.filepath_raw)
