"""Renders the rig as TEMPORARY proxy geometry over a semi-transparent body.
Run on goblin_v02_armature.blend; NEVER saves (throwaway session)."""
import bpy, math, os, sys
from mathutils import Vector, Matrix

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02"
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
body = bpy.data.objects["GOB_body"]
rig = bpy.data.objects["GOB_rig"]

# ---------------- engine
try:
    sc.render.engine = 'BLENDER_EEVEE_NEXT'
except TypeError:
    sc.render.engine = 'BLENDER_EEVEE'
print("ENGINE", sc.render.engine)
sc.render.resolution_x = 1100
sc.render.resolution_y = 1100
sc.render.resolution_percentage = 100
sc.render.film_transparent = False
sc.render.image_settings.file_format = 'PNG'
try:
    sc.eevee.taa_render_samples = 16
except Exception:
    pass
sc.world = bpy.data.worlds.new("tmp_world")
sc.world.use_nodes = True
_wnt = sc.world.node_tree
_wnt.nodes.clear()
_wo = _wnt.nodes.new("ShaderNodeOutputWorld")
_wb = _wnt.nodes.new("ShaderNodeBackground")
_wb.inputs[0].default_value = (0.06, 0.06, 0.07, 1)
_wb.inputs[1].default_value = 1.0
_wnt.links.new(_wb.outputs[0], _wo.inputs[0])

# ---------------- materials
def emis(name, rgb, strength=2.2):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    o = nt.nodes.new("ShaderNodeOutputMaterial")
    e = nt.nodes.new("ShaderNodeEmission")
    e.inputs[0].default_value = (rgb[0], rgb[1], rgb[2], 1)
    e.inputs[1].default_value = strength
    nt.links.new(e.outputs[0], o.inputs[0])
    return m

COL = {
    "spine": emis("c_spine", (1.0, 0.85, 0.10)),
    "L":     emis("c_L", (0.15, 0.55, 1.0)),
    "R":     emis("c_R", (1.0, 0.18, 0.18)),
    "weap":  emis("c_weap", (0.15, 1.0, 0.35)),
    "root":  emis("c_root", (1.0, 1.0, 1.0), 1.4),
    "joint": emis("c_joint", (1.0, 0.45, 1.0), 3.0),
}

# body: translucent
bmat = bpy.data.materials.new("tmp_body_xray")
bmat.use_nodes = True
nt = bmat.node_tree
nt.nodes.clear()
out = nt.nodes.new("ShaderNodeOutputMaterial")
mix = nt.nodes.new("ShaderNodeMixShader")
dif = nt.nodes.new("ShaderNodeBsdfDiffuse")
dif.inputs[0].default_value = (0.75, 0.75, 0.78, 1)
tra = nt.nodes.new("ShaderNodeBsdfTransparent")
fre = nt.nodes.new("ShaderNodeFresnel")
fre.inputs[0].default_value = 1.25
nt.links.new(fre.outputs[0], mix.inputs[0])
nt.links.new(tra.outputs[0], mix.inputs[1])
nt.links.new(dif.outputs[0], mix.inputs[2])
nt.links.new(mix.outputs[0], out.inputs[0])
for a in ("surface_render_method",):
    if hasattr(bmat, a):
        setattr(bmat, a, 'BLENDED')
bmat.blend_method = 'BLEND' if hasattr(bmat, "blend_method") else bmat.blend_method
if hasattr(bmat, "show_transparent_back"):
    bmat.show_transparent_back = False
body.data.materials.clear()
body.data.materials.append(bmat)

# ---------------- proxy geometry
tmp = bpy.data.collections.new("TMP_PROXY")
sc.collection.children.link(tmp)


def cylinder(name, a, b, r, mat):
    d = b - a
    L = d.length
    me = bpy.data.meshes.new(name)
    bm_verts = []
    faces = []
    N = 8
    for k in (0, 1):
        for i in range(N):
            ang = 2 * math.pi * i / N
            bm_verts.append((r * math.cos(ang), r * math.sin(ang), k * L))
    for i in range(N):
        j = (i + 1) % N
        faces.append((i, j, N + j, N + i))
    faces.append(tuple(range(N - 1, -1, -1)))
    faces.append(tuple(range(N, 2 * N)))
    me.from_pydata(bm_verts, [], faces)
    me.update()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    tmp.objects.link(ob)
    q = d.to_track_quat('Z', 'Y')
    ob.matrix_world = Matrix.Translation(a) @ q.to_matrix().to_4x4()
    return ob


def sphere(name, c, r, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=c, segments=12, ring_count=8)
    ob = bpy.context.object
    ob.name = name
    ob.data.materials.append(mat)
    for col in list(ob.users_collection):
        col.objects.unlink(ob)
    tmp.objects.link(ob)
    return ob


def group(n):
    if n in ("WEAPON", "WEAPON_SOCKET"):
        return "weap"
    if n in ("ROOT", "COG"):
        return "root"
    if n.endswith("_L"):
        return "L"
    if n.endswith("_R"):
        return "R"
    return "spine"


for b in rig.data.bones:
    a = Vector(b.head_local)
    t = Vector(b.tail_local)
    r = max(0.008, min(0.020, b.length * 0.07))
    cylinder("BX_" + b.name, a, t, r, COL[group(b.name)])
    sphere("JT_" + b.name + "_h", a, 0.018, COL["joint"])
sphere("JT_tips_HAND_R", Vector(rig.data.bones["HAND_R"].tail_local), 0.018, COL["joint"])
sphere("JT_tips_HAND_L", Vector(rig.data.bones["HAND_L"].tail_local), 0.018, COL["joint"])
sphere("JT_tips_FOOT_R", Vector(rig.data.bones["FOOT_R"].tail_local), 0.018, COL["joint"])
sphere("JT_tips_FOOT_L", Vector(rig.data.bones["FOOT_L"].tail_local), 0.018, COL["joint"])
sphere("JT_tips_HEAD", Vector(rig.data.bones["HEAD"].tail_local), 0.018, COL["joint"])
sphere("JT_tips_WEAPON", Vector(rig.data.bones["WEAPON"].tail_local), 0.018, COL["joint"])

# ---------------- camera
cd = bpy.data.cameras.new("tmp_cam")
cd.type = 'ORTHO'
cam = bpy.data.objects.new("tmp_cam", cd)
sc.collection.objects.link(cam)
sc.camera = cam

lamp = bpy.data.lights.new("tmp_l", 'SUN')
lamp.energy = 3.0
lo = bpy.data.objects.new("tmp_l", lamp)
sc.collection.objects.link(lo)
lo.rotation_euler = (math.radians(55), 0, math.radians(30))


def shoot(name, target, az_deg, el_deg, scale):
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)))
    ctr = Vector(target)
    cam.location = ctr + d * 8.0
    cd.ortho_scale = scale
    rot = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = rot
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("WROTE", sc.render.filepath)


def shoot_dir(name, target, d, scale):
    d = Vector(d).normalized()
    ctr = Vector(target)
    cam.location = ctr + d * 8.0
    cd.ortho_scale = scale
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("WROTE", sc.render.filepath)


def bnorm(u, f):
    bu = rig.data.bones[u]
    bf = rig.data.bones[f]
    return (Vector(bu.tail_local) - Vector(bu.head_local)).normalized().cross(
        (Vector(bf.tail_local) - Vector(bf.head_local)).normalized()).normalized()


C = (0.0, 0.0, 0.95)
shoot("b01_front", C, 0, 0, 2.15)
shoot("b02_side_right", C, 90, 0, 2.15)
shoot("b03_back", C, 180, 0, 2.15)
shoot("b04_three_quarter", C, 35, 10, 2.15)
shoot("b05_side_left", C, -90, 0, 2.15)
shoot("b06_armR_club_front", (-0.55, -0.15, 1.20), 0, 0, 1.05)
shoot("b07_armR_club_side", (-0.55, -0.15, 1.20), 100, 0, 1.05)
shoot("b08_legs_front", (0.0, 0.05, 0.30), 0, 0, 0.95)
shoot("b09_legs_side", (0.0, 0.05, 0.30), 90, 0, 0.95)
shoot("b10_torso_head_side", (0.0, 0.06, 1.30), 90, 0, 1.10)
shoot("b11_armL_front", (0.60, 0.05, 1.05), 0, 0, 0.95)
shoot_dir("b12_armR_bendplane", (-0.48, -0.05, 1.12), bnorm("UPPERARM_R", "FOREARM_R"), 0.95)
shoot_dir("b13_armL_bendplane", (0.55, 0.11, 1.05), bnorm("UPPERARM_L", "FOREARM_L"), 0.95)
shoot("b14_shoulders_front", (0.0, 0.05, 1.25), 0, 0, 1.20)
print("DONE")
