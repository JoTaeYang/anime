import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Vector
from lib import paths
from lib.blender_utils import open_blend, ensure_rigify
from lib.profiles import get_profile

PROFILE = get_profile()
rigged = paths.blend_path("01_rigged")

if rigged.exists():
    open_blend(rigged)
    arm = bpy.data.objects.get("metarig") or bpy.data.objects["rig"]
else:
    # 01_rig 이전: 00_mesh 위에 랜드마크로 메타리그만 배치해서 본다
    open_blend(paths.blend_path("00_mesh"))
    ensure_rigify()
    bpy.ops.object.armature_human_metarig_add()
    arm = bpy.context.active_object
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.data.edit_bones
    for name, ht in PROFILE.LANDMARKS.items():
        for side_name, mirror in ((name, 1.0), (name.replace(".L", ".R"), -1.0)):
            if side_name in eb:
                eb[side_name].head = (ht["head"][0] * mirror, ht["head"][1], ht["head"][2])
                eb[side_name].tail = (ht["tail"][0] * mirror, ht["tail"][1], ht["tail"][2])
    bpy.ops.object.mode_set(mode='OBJECT')

# 본 프록시 메시 (원뿔대: head가 굵고 tail이 뾰족)
mat = bpy.data.materials.new("BoneProxy")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
bsdf.inputs["Emission Color"].default_value = (1.0, 0.1, 0.1, 1.0)
bsdf.inputs["Emission Strength"].default_value = 3.0

proxies = []
for b in arm.data.bones:
    if b.name.startswith(("ORG-", "MCH-", "WGT-")):
        continue
    head, tail = Vector(b.head_local), Vector(b.tail_local)
    vec = tail - head
    if vec.length < 1e-5:
        continue
    bpy.ops.mesh.primitive_cone_add(radius1=max(0.012, vec.length * 0.07), radius2=0.002,
                                    depth=vec.length, location=(head + tail) / 2)
    p = bpy.context.active_object
    p.rotation_mode = 'QUATERNION'
    p.rotation_quaternion = vec.to_track_quat('Z', 'Y')
    p.data.materials.append(mat)
    proxies.append(p)

# 메시 반투명
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
for slot in mesh.material_slots:
    m = slot.material
    if m and m.use_nodes:
        pb = m.node_tree.nodes.get("Principled BSDF")
        if pb:
            pb.inputs["Alpha"].default_value = 0.35
        m.blend_method = 'BLEND'

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = 900
scene.render.resolution_y = 900
scene.render.image_settings.file_format = 'PNG'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.1, 0.1, 0.1, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 2.0
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
scene.collection.objects.link(sun)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
cz = mesh.dimensions.z / 2
for name, loc, rz in (("front", (0, -4, cz), 0), ("side", (4, 0, cz), 90), ("threequarter", (2.8, -2.8, cz), 45)):
    cam.location = loc
    cam.rotation_euler = (math.radians(90), 0, math.radians(rz))
    scene.render.filepath = str(paths.PREVIEWS_DIR / f"overlay_{name}.png")
    bpy.ops.render.render(write_still=True)
    print("OVERLAY", name)
print("OVERLAY OK")
