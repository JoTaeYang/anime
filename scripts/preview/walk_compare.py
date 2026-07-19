"""レファレンス シート vs 我々の歩き렌더の2x8比較シート. T5 收束ループの目."""
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile
from anim.walk_poses import KEY_FRAMES

PROFILE = get_profile()
TILE = 384
# レファレンス8칸 실측 x구간 (2172x724px)
REF_BOXES = [(33, 276), (318, 542), (597, 815), (880, 1074),
             (1159, 1399), (1455, 1617), (1727, 1891), (1975, 2139)]
# レファレンス피규어는 화면 오른쪽을 본다. 우리 캐릭터(-Y 전방)를 오른쪽 보기로 렌더하려면
# 카메라를 -X쪽에 두고 +X를 바라보게 한다. (첫 실행에서 방향 어긋나면 이 상수만 뒤집는다)
CAM_X = -4.0
CAM_ROT_Z = -90.0

open_blend(paths.blend_path("02_animated"))
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = TILE
scene.render.resolution_y = TILE
scene.render.image_settings.file_format = 'PNG'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.92, 0.92, 0.92, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(50), 0, math.radians(-120))
scene.collection.objects.link(sun)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
cz = mesh.dimensions.z * 0.5
cam.location = (CAM_X, 0, cz)
cam.rotation_euler = (math.radians(90), 0, math.radians(CAM_ROT_Z))

tiles_dir = paths.PREVIEWS_DIR / "walk_tiles"
tiles_dir.mkdir(parents=True, exist_ok=True)
ours = []
for i, f in enumerate(KEY_FRAMES):
    scene.frame_set(f)
    scene.render.filepath = str(tiles_dir / f"ours_{i}.png")
    bpy.ops.render.render(write_still=True)
    ours.append(tiles_dir / f"ours_{i}.png")

# レファレンス スライス → タイル サイズで リサンプル
ref_img = bpy.data.images.load(str(paths.PROJECT_ROOT / "refs" / "walk" / "walk_side_8frames.png"))
rw, rh = ref_img.size
ref_px = np.array(ref_img.pixels[:], dtype=np.float32).reshape(rh, rw, 4)


def _resize_nearest(a, out_h, out_w):
    ys = (np.arange(out_h) * a.shape[0] / out_h).astype(int)
    xs = (np.arange(out_w) * a.shape[1] / out_w).astype(int)
    return a[ys][:, xs]


sheet = np.ones((TILE * 2, TILE * 8, 4), dtype=np.float32)
for i, (x0, x1) in enumerate(REF_BOXES):
    crop = ref_px[:, x0:x1]
    h, w = crop.shape[0], crop.shape[1]
    scale = min(TILE / h, TILE / w)
    th, tw = int(h * scale), int(w * scale)
    tile = _resize_nearest(crop, th, tw)
    y0 = TILE + (TILE - th) // 2                   # 상단 행 (Blender 좌표: 위가 큰 y)
    x0o = i * TILE + (TILE - tw) // 2
    sheet[y0:y0 + th, x0o:x0o + tw] = tile
for i, p in enumerate(ours):
    img = bpy.data.images.load(str(p))
    px = np.array(img.pixels[:], dtype=np.float32).reshape(TILE, TILE, 4)
    sheet[0:TILE, i * TILE:(i + 1) * TILE] = px

out = bpy.data.images.new("cmp", TILE * 8, TILE * 2, alpha=True)
out.pixels.foreach_set(sheet.ravel())
out.filepath_raw = str(paths.PREVIEWS_DIR / "walk_compare.png")
out.file_format = 'PNG'
out.save()
print("WALK COMPARE OK:", paths.PREVIEWS_DIR / "walk_compare.png")
