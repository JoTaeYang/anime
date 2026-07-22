import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, save_as
from lib.profiles import get_profile

PROFILE = get_profile()

open_blend(paths.blend_path("01_rigged"))

scene = bpy.context.scene
rig = bpy.data.objects["rig"]

if PROFILE.ANIM_ACTION == "carrier":
    scene.render.fps = 24
    scene.frame_start = 1
    scene.frame_end = 48

    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='POSE')

    torso = rig.pose.bones["torso"]
    # 호흡: 중간(f24)에 1.5cm 하강, 시작/끝 동일 포즈로 루프 성립
    for frame, offset in ((1, 0.0), (24, -0.015), (48, 0.0)):
        scene.frame_set(frame)
        # torso 본 로컬축 z 방향 직접 이동
        torso.location = (0.0, 0.0, offset)
        torso.keyframe_insert(data_path="location", frame=frame)

    bpy.ops.object.mode_set(mode='OBJECT')
    rig.animation_data.action.name = "Idle"
else:
    import importlib
    builder = importlib.import_module(f"anim.{PROFILE.ANIM_ACTION}_build")
    builder.build(rig, scene)

save_as(paths.blend_path("02_animated"))
print("STAGE 02_anim OK")
