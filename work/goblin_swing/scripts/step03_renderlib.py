"""Shared Workbench render helper for STEP 03 / GATE 03."""
import bpy, math, os
from mathutils import Vector

PALETTE = [
    ("HIPS", (0.95, 0.35, 0.20)), ("SPINE_01", (0.98, 0.70, 0.15)),
    ("CHEST", (0.85, 0.95, 0.20)), ("NECK", (0.35, 0.85, 0.30)),
    ("HEAD", (0.15, 0.60, 0.95)),
    ("UPPERARM_L", (0.20, 0.95, 0.85)), ("FOREARM_L", (0.10, 0.55, 0.75)),
    ("HAND_L", (0.55, 0.30, 0.95)),
    ("UPPERARM_R", (0.95, 0.30, 0.75)), ("FOREARM_R", (0.70, 0.10, 0.35)),
    ("HAND_R", (1.00, 0.85, 0.85)), ("WEAPON", (0.35, 0.25, 0.10)),
    ("THIGH_L", (0.55, 0.95, 0.55)), ("SHIN_L", (0.20, 0.70, 0.35)),
    ("FOOT_L", (0.05, 0.35, 0.15)),
    ("THIGH_R", (0.95, 0.75, 0.55)), ("SHIN_R", (0.80, 0.50, 0.15)),
    ("FOOT_R", (0.40, 0.25, 0.05)),
]


def setup(res=1024, plain=True):
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sh = sc.display.shading
    sh.light = 'STUDIO'
    sh.studio_light = 'Default'
    sh.color_type = 'SINGLE' if plain else 'MATERIAL'
    sh.single_color = (0.72, 0.72, 0.74)
    sh.show_cavity = True
    sh.cavity_type = 'BOTH'
    sh.curvature_ridge_factor = 1.0
    sh.curvature_valley_factor = 1.0
    try:
        sh.show_shadows = True
        sh.shadow_intensity = 0.45
    except Exception:
        pass
    sh.show_object_outline = False
    sc.display.render_aa = '8'
    sc.render.resolution_x = res
    sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = 'PNG'
    cd = bpy.data.cameras.get("s3_cam") or bpy.data.cameras.new("s3_cam")
    cd.type = 'ORTHO'
    cam = bpy.data.objects.get("s3_cam")
    if cam is None:
        cam = bpy.data.objects.new("s3_cam", cd)
        sc.collection.objects.link(cam)
    sc.camera = cam
    return cam, cd


def shoot(cam, cd, out_dir, name, target, az_deg, el_deg, scale):
    sc = bpy.context.scene
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el),
                math.sin(el)))
    ctr = Vector(target)
    cam.location = ctr + d * 8.0
    cd.ortho_scale = scale
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    os.makedirs(out_dir, exist_ok=True)
    sc.render.filepath = os.path.join(out_dir, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("WROTE", sc.render.filepath)
    return sc.render.filepath


def colorize_by_group(body, names=None):
    """one material per deform bone; each face takes its dominant vertex group."""
    names = names or [n for n, _ in PALETTE]
    me = body.data
    me.materials.clear()
    mats = {}
    for n, rgb in PALETTE:
        m = bpy.data.materials.get("LBL_" + n) or bpy.data.materials.new("LBL_" + n)
        m.use_nodes = False
        m.diffuse_color = (rgb[0], rgb[1], rgb[2], 1.0)
        me.materials.append(m)
        mats[n] = len(me.materials) - 1
    gi = {g.index: g.name for g in body.vertex_groups}
    vlab = [0] * len(me.vertices)
    for v in me.vertices:
        best, bw = None, -1.0
        for g in v.groups:
            if g.weight > bw:
                bw, best = g.weight, gi[g.group]
        vlab[v.index] = mats.get(best, 0)
    for p in me.polygons:
        cnt = {}
        for i in p.vertices:
            cnt[vlab[i]] = cnt.get(vlab[i], 0) + 1
        p.material_index = max(cnt.items(), key=lambda kv: kv[1])[0]
    me.update()


def colorize_by_blend(body):
    """grey = single influence, red = 2, orange = 3, yellow = 4+"""
    me = body.data
    me.materials.clear()
    cols = [(0.72, 0.72, 0.74), (0.90, 0.10, 0.10), (1.0, 0.55, 0.05),
            (1.0, 0.95, 0.15)]
    for k, c in enumerate(cols):
        m = bpy.data.materials.get("BL_%d" % k) or bpy.data.materials.new("BL_%d" % k)
        m.use_nodes = False
        m.diffuse_color = (c[0], c[1], c[2], 1.0)
        me.materials.append(m)
    vn = [0] * len(me.vertices)
    for v in me.vertices:
        vn[v.index] = min(3, max(0, sum(1 for g in v.groups if g.weight > 1e-4) - 1))
    for p in me.polygons:
        p.material_index = max(vn[i] for i in p.vertices)
    me.update()
