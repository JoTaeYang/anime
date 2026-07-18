import sys
import math
import shutil
import subprocess
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Matrix
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()
open_blend(paths.blend_path("03_baked"))
out_dir = paths.PREVIEWS_DIR / "user"
out_dir.mkdir(parents=True, exist_ok=True)

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.12, 0.12, 0.13, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(55), 0, math.radians(35))
scene.collection.objects.link(sun)
fill = bpy.data.objects.new("Fill", bpy.data.lights.new("Fill", 'SUN'))
fill.data.energy = 1.0
fill.rotation_euler = (math.radians(60), 0, math.radians(-120))
scene.collection.objects.link(fill)

rig = bpy.data.objects["DummyRig"]
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
cz = mesh.dimensions.z * 0.52

# 턴테이블 리그: 카메라를 엠티 자식으로, 엠티 회전 키
pivot = bpy.data.objects.new("Pivot", None)
scene.collection.objects.link(pivot)
pivot.location = (0, 0, cz)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
cam.parent = pivot
cam.location = (0, -3.6, 0)
cam.rotation_euler = (math.radians(90), 0, 0)
scene.camera = cam


def render_video(filepath, frame_end, setup):
    # Blender 5.1 quirk (this environment, undocumented elsewhere in repo):
    # scene.render.image_settings.file_format rejects 'FFMPEG' (and every movie
    # format — AVI_JPEG/AVI_RAW too) at runtime with "enum 'FFMPEG' not found",
    # even though bpy.app.build_options.codec_ffmpeg is True and 'FFMPEG' is
    # listed in the static bl_rna enum_items. Reproduced on a bare
    # --background --factory-startup scene with no blend loaded, so it is not
    # specific to this project's file. The brief's direct
    # image_settings.file_format='FFMPEG' + scene.render.ffmpeg.* path is not
    # settable in this build. Fallback: render a PNG sequence to a temp dir,
    # then mux to H.264 mp4 with the system ffmpeg binary via subprocess.
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.fps = 24
    scene.frame_start = 1
    scene.frame_end = frame_end
    setup()

    frames_dir = out_dir / f"_frames_{filepath.stem}"
    frames_dir.mkdir(parents=True, exist_ok=True)
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = str(frames_dir / "f_")
    bpy.ops.render.render(animation=True)

    ffmpeg = shutil.which("ffmpeg")
    assert ffmpeg, "system ffmpeg not found on PATH; required for PNG->mp4 mux fallback"
    subprocess.run([
        ffmpeg, "-y", "-framerate", str(scene.render.fps),
        "-i", str(frames_dir / "f_%04d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
        str(filepath),
    ], check=True)
    shutil.rmtree(frames_dir)


def reset_pose():
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


# 1) 레스트 턴테이블 (5초 = 120f)
def setup_turntable():
    rig.animation_data.action = None
    rig.data.pose_position = 'REST'
    pivot.rotation_euler = (0, 0, 0)
    pivot.keyframe_insert("rotation_euler", frame=1)
    pivot.rotation_euler = (0, 0, math.radians(360))
    pivot.keyframe_insert("rotation_euler", frame=120)
    # Blender 5.1: layered action API (action.fcurves 제거됨). check_03.py의
    # action_fcurves() 순회 패턴을 따른다. 실패해도(보간 기본값 유지) 시각적
    # 차이가 미미하므로 브리프 승인된 fallback으로 try/except 처리.
    try:
        action = pivot.animation_data.action
        for layer in action.layers:
            for strip in layer.strips:
                for cb in strip.channelbags:
                    for fc in cb.fcurves:
                        for kp in fc.keyframe_points:
                            kp.interpolation = 'LINEAR'
    except Exception as exc:
        print("WARN: could not set LINEAR interpolation on turntable pivot:", exc)


render_video(out_dir / "turntable.mp4", 120, setup_turntable)

# 2) idle 재생 (2루프 = 96f, 고정 3/4 뷰)
def setup_idle():
    pivot.animation_data_clear()
    pivot.rotation_euler = (0, 0, math.radians(45))
    rig.data.pose_position = 'POSE'
    rig.animation_data.action = bpy.data.actions.get("Idle")


scene.frame_start = 1
render_video(out_dir / "idle.mp4", 96, setup_idle)

# 3) 극단 포즈 스틸 1536² — 부호는 contact_sheet.py 현행 규약과 동일
#    (armsup: LeftUpperArm Y-80/RightUpperArm Y80, crouch: UpperLeg X-60/
#    LowerLeg X100/Foot X-40, twist: Spine/Chest Z25) — 확인 완료, 변경 없음.


def rotate_world(bone, axis, deg):
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


STILL_POSES = [
    {"name": "armsup", "view_rz": 0, "ops": [("rotate", "LeftUpperArm", 'Y', -80), ("rotate", "RightUpperArm", 'Y', 80)]},
    {"name": "crouch", "view_rz": 90, "ops": [("translate", "Hips", -0.30),
        ("rotate", "LeftUpperLeg", 'X', -60), ("rotate", "RightUpperLeg", 'X', -60),
        ("rotate", "LeftLowerLeg", 'X', 100), ("rotate", "RightLowerLeg", 'X', 100),
        ("rotate", "LeftFoot", 'X', -40), ("rotate", "RightFoot", 'X', -40)]},
    {"name": "twist", "view_rz": 0, "ops": [("rotate", "Spine", 'Z', 25), ("rotate", "Chest", 'Z', 25)]},
] + [{"name": p["name"], "view_rz": {"front": 0, "side": 90, "threequarter": 45}[p["view"]], "ops": p["ops"]}
     for p in PROFILE.EXTRA_POSES]

scene.render.image_settings.file_format = 'PNG'
scene.render.resolution_x = 1536
scene.render.resolution_y = 1536
rig.animation_data.action = None
rig.data.pose_position = 'POSE'
for pose in STILL_POSES:
    reset_pose()
    for op in pose["ops"]:
        if op[0] == "rotate":
            rotate_world(op[1], op[2], op[3])
        else:
            translate_world(op[1], op[2])
    pivot.rotation_euler = (0, 0, math.radians(pose["view_rz"]))
    scene.render.filepath = str(out_dir / f"still_{pose['name']}.png")
    bpy.ops.render.render(write_still=True)
    print("STILL", pose["name"])
print("USER PREVIEW OK:", out_dir)
