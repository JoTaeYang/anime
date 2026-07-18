import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from mathutils import Matrix
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()

TILE = 512
open_blend(paths.blend_path("03_baked"))
paths.ensure_dirs()
tiles_dir = paths.PREVIEWS_DIR / "tiles"
tiles_dir.mkdir(exist_ok=True)

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'   # 5.1에서 EEVEE 식별자 (실측). GPU 없으면 'CYCLES'+16샘플로 폴백
scene.render.resolution_x = TILE
scene.render.resolution_y = TILE
scene.render.image_settings.file_format = 'PNG'

world = bpy.data.worlds.new("W") if scene.world is None else scene.world
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.15, 0.15, 0.15, 1)

sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
scene.collection.objects.link(sun)

cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam

rig = bpy.data.objects["DummyRig"]

VIEWS = {  # (위치, 회전) — 타깃 (0,0,0.85), 거리 4m
    "front": ((0, -4, 0.85), (math.radians(90), 0, 0)),
    "side": ((4, 0, 0.85), (math.radians(90), 0, math.radians(90))),
    "threequarter": ((2.8, -2.8, 0.85), (math.radians(90), 0, math.radians(45))),
    "back": ((0, 4, 0.85), (math.radians(90), 0, math.radians(180))),
}


def set_view(name):
    cam.location, cam.rotation_euler = VIEWS[name]


def render_tile(filename):
    scene.render.filepath = str(tiles_dir / filename)
    bpy.ops.render.render(write_still=True)


def reset_pose():
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


def rotate_world(bone, axis, deg):
    """본을 자기 헤드 기준으로 월드축 회전 (부모부터 순서대로 호출할 것)"""
    pb = rig.pose.bones[bone]
    bpy.context.view_layer.update()
    head = pb.matrix.to_translation()
    R = Matrix.Translation(head) @ Matrix.Rotation(math.radians(deg), 4, axis) @ Matrix.Translation(-head)
    pb.matrix = R @ pb.matrix
    bpy.context.view_layer.update()


def translate_world(bone, dz):
    pb = rig.pose.bones[bone]
    bpy.context.view_layer.update()
    m = pb.matrix.copy()
    m.translation.z += dz
    pb.matrix = m
    bpy.context.view_layer.update()


order = []

# 1) 레스트 4각도
rig.data.pose_position = 'REST'
if rig.animation_data:
    rig.animation_data.action = None
for view in ("front", "side", "threequarter", "back"):
    set_view(view)
    render_tile(f"rest_{view}.png")
    order.append(f"rest_{view}.png")

# 2) 극단 포즈 3종 (POSE 모드, 액션 없음, deform 본 직접 회전)
rig.data.pose_position = 'POSE'

reset_pose()
# RESOLVED 2026-07-18: L/R 스왑 제거 후 "LeftUpperArm"은 다시 Blender +X(해부학적 왼쪽) 본이다
# → 스왑 때 뒤집었던 부호를 원복한다 (좌우 팔을 대칭으로 위로).
rotate_world("LeftUpperArm", 'Y', -80)
rotate_world("RightUpperArm", 'Y', 80)
set_view("front")
render_tile("pose_armsup.png")
order.append("pose_armsup.png")

reset_pose()
translate_world("Hips", -0.30)
for side in ("Left", "Right"):
    rotate_world(f"{side}UpperLeg", 'X', -60)
    rotate_world(f"{side}LowerLeg", 'X', 100)
    rotate_world(f"{side}Foot", 'X', -40)
set_view("side")
render_tile("pose_crouch.png")
order.append("pose_crouch.png")

reset_pose()
rotate_world("Spine", 'Z', 25)
rotate_world("Chest", 'Z', 25)
set_view("front")
render_tile("pose_twist.png")
order.append("pose_twist.png")

for pose in PROFILE.EXTRA_POSES:
    reset_pose()
    for op in pose["ops"]:
        if op[0] == "rotate":
            rotate_world(op[1], op[2], op[3])
        elif op[0] == "translate":
            translate_world(op[1], op[2])
    set_view(pose["view"])
    render_tile(f"pose_{pose['name']}.png")
    order.append(f"pose_{pose['name']}.png")

# 3) idle 2프레임
reset_pose()
act = bpy.data.actions.get("Idle")
rig.animation_data.action = act
set_view("threequarter")
for frame in (1, 24):
    scene.frame_set(frame)
    render_tile(f"idle_f{frame}.png")
    order.append(f"idle_f{frame}.png")

# 4) 격자 합성 (Blender 이미지 원점은 좌하단 → 행을 위에서부터 채우려면 뒤집기)
rows = math.ceil(len(order) / 3)
sheet = np.zeros((TILE * rows, TILE * 3, 4), dtype=np.float32)
for i, name in enumerate(order):
    img = bpy.data.images.load(str(tiles_dir / name))
    px = np.array(img.pixels[:], dtype=np.float32).reshape(TILE, TILE, 4)
    row, col = divmod(i, 3)
    y0 = TILE * rows - (row + 1) * TILE
    sheet[y0:y0 + TILE, col * TILE:(col + 1) * TILE] = px

out = bpy.data.images.new("sheet", TILE * 3, TILE * rows, alpha=True)
out.pixels.foreach_set(sheet.ravel())
out.filepath_raw = str(paths.PREVIEWS_DIR / "contact_sheet.png")
out.file_format = 'PNG'
out.save()
print("CONTACT SHEET OK:", paths.PREVIEWS_DIR / "contact_sheet.png")
